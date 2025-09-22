import math, time, rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
from std_msgs.msg import String
from .utils import dist, heading_to, sat


class TangentBug(Node):
    """
    Classic TangentBug (functional minimal):
      - MOTION_TO_GOAL: go straight to goal if LoS clear
      - FOLLOW_BOUNDARY: wall-follow around obstacle keeping safe clearance
        and leave when LoS to goal opens again.
    """

    def __init__(self):
        super().__init__('tangent_bug_node')

        # goal & speeds
        self.declare_parameter('goal_x', 7.5)
        self.declare_parameter('goal_y', -8.5)
        self.declare_parameter('v_lin', 0.26)
        self.declare_parameter('v_follow', 0.14)
        self.declare_parameter('v_min', 0.10)
        self.declare_parameter('v_ang', 1.6)

        # sensing & safety
        self.declare_parameter('obs_dist', 0.35)
        self.declare_parameter('clear_dist', 0.50)
        self.declare_parameter('emergency_dist', 0.22)
        self.declare_parameter('obs_cone_deg', 50.0)
        self.declare_parameter('goal_cone_deg', 6.0)
        self.declare_parameter('edge_thresh', 0.25)
        self.declare_parameter('avoid_cone_deg', 24.0)
        self.declare_parameter('k_avoid', 0.9)
        # (kept for compatibility, not required in minimal TB)

        # control smoothing
        self.declare_parameter('heading_kp', 1.05)
        self.declare_parameter('heading_kd', 0.25)
        self.declare_parameter('ang_filter_alpha', 0.4)
        self.declare_parameter('k_wall', 1.6)

        # runtime state
        self.pose = (0.0, 0.0, 0.0)
        self.scan = None; self.ang0 = 0.0; self.dang = 0.0
        self.state = 'MOTION_TO_GOAL'
        self.follow_side = 'left'
        self.best_h = float('inf')
        self.leave_pt = None
        self.leave_t0 = 0.0
        self.err_hdg = 0.0; self.err_hdg_prev = 0.0; self.last_t = time.time()

        # ROS I/O
        self.create_subscription(Odometry, '/odom', self.on_odom, 10)
        self.create_subscription(LaserScan, '/scan', self.on_scan, qos_profile_sensor_data)
        self.pub_cmd  = self.create_publisher(Twist, '/cmd_vel', 10)
        self.pub_state= self.create_publisher(String, '/bug_state', 10)
        self.timer    = self.create_timer(0.05, self.control)

        self.get_logger().info('TangentBug ready: MOTION_TO_GOAL ↔ FOLLOW_BOUNDARY')

    # --- ROS helpers ---
    def on_state(self, s: str):
        self.state = s
        msg = f'TANGENT:{s}'
        self.pub_state.publish(String(data=msg))
        self.get_logger().info(msg)

    def on_cmd(self, v: float, w: float):
        t = Twist(); t.linear.x = v; t.angular.z = w; self.pub_cmd.publish(t)

    def on_odom(self, msg: Odometry):
        x = msg.pose.pose.position.x; y = msg.pose.pose.position.y
        q = msg.pose.pose.orientation
        siny = 2*(q.w*q.z + q.x*q.y); cosy = 1 - 2*(q.y*q.y + q.z*q.z)
        self.pose = (x, y, math.atan2(siny, cosy))

    def on_scan(self, msg: LaserScan):
        self.scan = list(msg.ranges); self.ang0 = msg.angle_min; self.dang = msg.angle_increment

    # --- Perception utilities ---
    def cone_min_at(self, center_ang: float, deg: float) -> float:
        if self.scan is None: return 10.0
        cone = math.radians(deg)
        i0 = int(((center_ang - cone/2) - self.ang0)/self.dang)
        i1 = int(((center_ang + cone/2) - self.ang0)/self.dang)
        n  = len(self.scan); vals=[]
        for i in range(max(0,i0), min(n,i1)):
            r = self.scan[i]
            if math.isfinite(r) and r>0.0: vals.append(r)
        return min(vals) if vals else 10.0

    def cone_min(self, deg: float) -> float:
        return self.cone_min_at(0.0, deg)

    def ray_range(self, ang: float) -> float:
        if self.scan is None: return 10.0
        i = int((ang - self.ang0)/self.dang); n = len(self.scan)
        vals=[]
        for k in range(i-1, i+2):
            if 0<=k<n:
                r = self.scan[k]
                if math.isfinite(r) and r>0.0: vals.append(r)
        return min(vals) if vals else 10.0

    def goal_visible(self, th: float, gx: float, gy: float) -> bool:
        # Consider LoS if there is no obstacle within a small cone around goal bearing
        # closer than a conservative clear distance. This avoids dependence on lidar max range.
        x,y,_ = self.pose
        hdg   = heading_to(x,y,gx,gy)
        err   = (hdg - th + math.pi)%(2*math.pi) - math.pi
        r     = self.cone_min_at(err, float(self.get_parameter('goal_cone_deg').value))
        return r > float(self.get_parameter('clear_dist').value) + 0.05

    def pick_follow_side(self):
        # choose the side with closer wall so we keep it on that side
        left = self.scan_at(+math.radians(60.0))
        right= self.scan_at(-math.radians(60.0))
        return 'left' if left < right else 'right'

    def tangent_candidates(self, x, y, th, gx, gy):
        if self.scan is None: return []
        edge_thr = float(self.get_parameter('edge_thresh').value)
        clear = float(self.get_parameter('clear_dist').value)
        emer = float(self.get_parameter('emergency_dist').value)
        off = math.radians(12.0)
        step_max = 0.6
        n=len(self.scan); cand=[]
        # edges-based
        for i in range(1,n-1):
            r=self.scan[i]; r0=self.scan[i-1]; r1=self.scan[i+1]
            if not (r and r>0.0 and math.isfinite(r) and math.isfinite(r0) and math.isfinite(r1)): continue
            if abs(r-r0) > edge_thr or abs(r-r1) > edge_thr:
                base=self.ang0+i*self.dang
                if r < emer + 0.08: continue
                for ang in (base-off, base+off):
                    Rdir=self.cone_min_at(ang, 10.0)
                    avail=max(0.0, min(Rdir, r))
                    if avail < clear + 0.10: continue
                    d_step=min(avail-clear, step_max)
                    if d_step < 0.25: continue
                    # LoS até o ponto precisa ser >= d_step
                    if self.cone_min_at(ang, 10.0) + 1e-3 < d_step: continue
                    px=x + d_step*math.cos(th+ang); py=y + d_step*math.sin(th+ang)
                    h=dist(x,y,px,py) + dist(px,py,gx,gy)
                    cand.append((h, px, py, ang))
        # around goal bearing
        hdg=heading_to(x,y,gx,gy); err=(hdg-th+math.pi)%(2*math.pi)-math.pi
        for offg in (-0.35,-0.2,0.2,0.35):
            ang=err+offg; r=self.ray_range(ang)
            if not math.isfinite(r) or r<=0.0: continue
            Rdir=self.cone_min_at(ang,10.0); avail=max(0.0,min(Rdir,r))
            if avail < max(emer+0.10, 0.8*clear): continue
            d_step=min(avail-clear, step_max)
            if d_step < 0.25: continue
            if self.cone_min_at(ang, 10.0) + 1e-3 < d_step: continue
            px=x + d_step*math.cos(th+ang); py=y + d_step*math.sin(th+ang)
            h=dist(x,y,px,py) + dist(px,py,gx,gy) + 0.05*abs(offg)
            cand.append((h, px, py, ang))
        cand.sort(key=lambda t:t[0])
        return cand[:6]

    def select_tangent_point(self, x: float, y: float, th: float, gx: float, gy: float):
        if self.scan is None: return None
        edge_thr = float(self.get_parameter('edge_thresh').value)
        clear    = float(self.get_parameter('clear_dist').value)
        emer     = float(self.get_parameter('emergency_dist').value)
        off      = math.radians(float(self.get_parameter('tangent_offset_deg').value))
        step_max = float(self.get_parameter('target_step_max').value)
        n=len(self.scan); cand=[]
        # edges
        for i in range(1,n-1):
            r=self.scan[i]; r0=self.scan[i-1]; r1=self.scan[i+1]
            if not (r and r>0.0 and math.isfinite(r) and math.isfinite(r0) and math.isfinite(r1)): continue
            if abs(r-r0)>edge_thr or abs(r-r1)>edge_thr:
                base=self.ang0+i*self.dang
                if r < emer + 0.08: continue
                for ang in (base-off, base+off):
                    Rdir=self.cone_min_at(ang, 10.0)
                    avail=max(0.0, min(Rdir, r))
                    if avail < clear + 0.10: continue
                    d_step=min(avail - clear, step_max)
                    if d_step < 0.25: continue
                    px=x + d_step*math.cos(th+ang); py=y + d_step*math.sin(th+ang)
                    h=dist(x,y,px,py)+dist(px,py,gx,gy)
                    cand.append((h,px,py))
        # around goal bearing
        hdg=heading_to(x,y,gx,gy); err=(hdg-th+math.pi)%(2*math.pi)-math.pi
        for offg in (-0.35,-0.2,0.2,0.35):
            ang=err+offg; r=self.ray_range(ang)
            if not math.isfinite(r) or r<=0.0: continue
            Rdir=self.cone_min_at(ang,10.0); avail=max(0.0,min(Rdir,r))
            if avail < max(float(self.get_parameter('emergency_dist').value) + 0.10, clear*0.8): continue
            d_step=min(avail-clear, step_max)
            if d_step < 0.25: continue
            px=x + d_step*math.cos(th+ang); py=y + d_step*math.sin(th+ang)
            h=dist(x,y,px,py)+dist(px,py,gx,gy)+0.05*abs(offg)
            cand.append((h,px,py))
        if not cand: return None
        cand.sort(key=lambda t:t[0])
        return (cand[0][1], cand[0][2])

    # --- Main control ---
    def control(self):
        x,y,th = self.pose
        gx=float(self.get_parameter('goal_x').value); gy=float(self.get_parameter('goal_y').value)
        now=time.time(); dt=max(1e-3, now-(self.last_t or now)); self.last_t=now

        dgoal=dist(x,y,gx,gy)
        if dgoal < 0.12:
            self.on_state('ARRIVED'); self.on_cmd(0.0,0.0); return

        dfront=self.cone_min(float(self.get_parameter('obs_cone_deg').value))
        vmin=float(self.get_parameter('v_min').value)
        wmax=float(self.get_parameter('v_ang').value)
        kp=float(self.get_parameter('heading_kp').value)
        kd=float(self.get_parameter('heading_kd').value)
        alpha=float(self.get_parameter('ang_filter_alpha').value)
        emer=float(self.get_parameter('emergency_dist').value)
        clear=float(self.get_parameter('clear_dist').value)

        if self.state=='MOTION_TO_GOAL':
            if (not self.goal_visible(th,gx,gy)) or dfront < float(self.get_parameter('obs_dist').value):
                self.follow_side = self.pick_follow_side()
                self.on_state('FOLLOW_BOUNDARY'); return
            # PD to goal
            hdg=heading_to(x,y,gx,gy)
            err=(hdg-th+math.pi)%(2*math.pi)-math.pi
            self.err_hdg=(1.0-alpha)*self.err_hdg + alpha*err
            derr=(self.err_hdg - self.err_hdg_prev)/dt; self.err_hdg_prev=self.err_hdg
            w=sat(kp*self.err_hdg + kd*derr, wmax)
            v=float(self.get_parameter('v_lin').value)
            v = max(vmin, v*(1.0 - 0.5*min(1.0, abs(w)/wmax)))
            v = min(v, max(0.08, 0.6*(self.cone_min_at(0.0,15.0) - emer)))
            self.on_cmd(v,w)
            return

        if self.state=='FOLLOW_BOUNDARY':
            # Wall-follow with frontal avoidance bias
            sign = +1.0 if self.follow_side=='left' else -1.0
            dside = self.scan_at(sign*math.radians(60.0))
            e_side = (dside - clear)
            k_wall=float(self.get_parameter('k_wall').value)
            # Frontal left/right clearance to steer away from the nearer side
            avoid_deg=float(self.get_parameter('avoid_cone_deg').value)
            left_c  = self.cone_min_at(+0.35, avoid_deg)
            right_c = self.cone_min_at(-0.35, avoid_deg)
            inv_left  = 1.0 / max(0.05, left_c)
            inv_right = 1.0 / max(0.05, right_c)
            k_avoid=float(self.get_parameter('k_avoid').value)
            w_avoid = k_avoid * (inv_right - inv_left)
            w = sat(sign*(k_wall*e_side) + w_avoid, 1.2)
            v_follow=float(self.get_parameter('v_follow').value)
            v = max(vmin, v_follow*(1.0-0.4*min(1.0,abs(w)/1.2)))
            v = min(v, max(0.04, 0.8*(dfront - emer)))
            if dfront < emer + 0.02:
                self.on_cmd(0.0, sat(-sign*0.9,1.2)); return
            self.on_cmd(v,w)

            # Leave when LoS to goal is open and front is clear enough
            if self.goal_visible(x,y,th,gx,gy) and dfront > clear + 0.03:
                self.on_state('MOTION_TO_GOAL'); return

            # Try a safe tangent leave if a good candidate exists
            cand = self.tangent_candidates(x,y,th,gx,gy)
            if cand:
                h, px, py, ang = cand[0]
                # only leave if the bearing is roughly forward (within ±90°) and LoS to point good
                if -math.pi/2 < ang < math.pi/2 and self.cone_min_at(ang, 10.0) > dist(x,y,px,py) - 0.05:
                    self.leave_pt=(px,py); self.leave_t0=now
                    self.on_state('LEAVE_TO_POINT'); return

        if self.state=='LEAVE_TO_POINT':
            if self.leave_pt is None:
                self.on_state('FOLLOW_BOUNDARY'); return
            px,py=self.leave_pt
            # If LoS to goal opens, go to goal
            if self.goal_visible(x,y,th,gx,gy) and dfront > clear + 0.02:
                self.leave_pt=None
                self.on_state('MOTION_TO_GOAL'); return
            # If blocked toward point, revert to boundary
            ang=math.atan2(py - y, px - x) - th
            ang=(ang+math.pi)%(2*math.pi)-math.pi
            if self.cone_min_at(ang, 10.0) < emer + 0.03:
                self.leave_pt=None
                self.on_state('FOLLOW_BOUNDARY'); return
            # PD to point
            self.err_hdg=(1.0-alpha)*self.err_hdg + alpha*ang
            derr=(self.err_hdg - self.err_hdg_prev)/dt; self.err_hdg_prev=self.err_hdg
            w=sat(kp*self.err_hdg + kd*derr, wmax)
            v_follow=float(self.get_parameter('v_follow').value)
            v = max(vmin, v_follow*(1.0-0.5*min(1.0,abs(w)/wmax)))
            v = min(v, max(0.06, 0.6*(self.cone_min_at(0.0,15.0)-emer)))
            self.on_cmd(v,w)
            # Terminate near point or if taking too long
            if dist(x,y,px,py) < 0.35 or (now - self.leave_t0) > 2.5:
                self.leave_pt=None
                if self.goal_visible(x,y,th,gx,gy) and dfront > clear:
                    self.on_state('MOTION_TO_GOAL')
                else:
                    self.on_state('FOLLOW_BOUNDARY')
                return


def main():
    rclpy.init(); rclpy.spin(TangentBug()); rclpy.shutdown()
