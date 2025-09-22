import rclpy, math, time
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
from std_msgs.msg import String
from .utils import dist, heading_to, sat

class Bug2(Node):
    def __init__(self):
        super().__init__('bug2_node')
        self.declare_parameter('goal_x', 7.5)
        self.declare_parameter('goal_y', -8.5)
        self.declare_parameter('v_lin', 0.26)
        self.declare_parameter('v_ang', 1.82)
        self.declare_parameter('v_fb',  0.18)
        self.declare_parameter('v_min', 0.12)
        self.declare_parameter('obs_dist', 0.35)
        self.declare_parameter('clear_dist', 0.45)
        self.declare_parameter('follow_side', 'left')
        self.declare_parameter('follow_dist', 0.45)
        self.declare_parameter('obs_cone_deg', 50.0)
        self.declare_parameter('emergency_dist', 0.22)
        self.declare_parameter('mline_eps', 0.12)

        self.state = 'MOTION_TO_GOAL'
        self.pose = (0.0,0.0,0.0)
        self.scan = None
        self.ang0 = 0.0; self.dang = 0.0
        self.start = None
        self.hit = None
        self.hit_dgoal = None
        self.side = None
        self.loop_t0 = None
        self.leave_t0 = None

        self.create_subscription(Odometry, '/odom', self.on_odom, 10)
        self.create_subscription(LaserScan, '/scan', self.on_scan, qos_profile_sensor_data)
        self.pub_cmd = self.create_publisher(Twist, '/cmd_vel', 10)
        self.pub_state = self.create_publisher(String, '/bug_state', 10)
        self.timer = self.create_timer(0.05, self.control)

        self.get_logger().info('BUG2 ready: MOTION_TO_GOAL, FOLLOW_BOUNDARY (classic)')

    def on_state(self, s):
        self.state = s
        self.pub_state.publish(String(data=f'BUG2:{s}'))

    def on_cmd(self, v, w):
        t = Twist(); t.linear.x=v; t.angular.z=w; self.pub_cmd.publish(t)

    def on_odom(self, msg):
        x = msg.pose.pose.position.x; y = msg.pose.pose.position.y
        q = msg.pose.pose.orientation
        siny_cosp = 2*(q.w*q.z + q.x*q.y)
        cosy_cosp = 1 - 2*(q.y*q.y + q.z*q.z)
        yaw = math.atan2(siny_cosp, cosy_cosp)
        self.pose = (x,y,yaw)

    def on_scan(self, msg):
        self.scan = list(msg.ranges); self.ang0 = msg.angle_min; self.dang = msg.angle_increment

    def cone_min(self, deg):
        if self.scan is None: return 10.0
        cone = math.radians(deg)
        i0 = int(( -cone - self.ang0)/self.dang); i1 = int(( cone - self.ang0)/self.dang)
        n = len(self.scan)
        vals = []
        for i in range(max(0,i0), min(n,i1)):
            r = self.scan[i]
            if math.isfinite(r) and r>0.0:
                vals.append(r)
        return min(vals) if vals else 10.0

    def scan_at(self, ang):
        if self.scan is None: return 10.0
        i = int((ang - self.ang0)/self.dang); n = len(self.scan)
        vals=[]
        for k in range(i-1,i+2):
            if 0<=k<n:
                r=self.scan[k]
                if math.isfinite(r) and r>0.0: vals.append(r)
        return min(vals) if vals else 10.0

    def on_mline(self, x, y, gx, gy):
        if self.start is None:
            return False
        sx, sy = self.start
        dx = gx - sx; dy = gy - sy
        L2 = dx*dx + dy*dy
        if L2 < 1e-9:
            return True
        # Cross-track error to the line
        cte = abs((x - sx)*dy - (y - sy)*dx) / math.sqrt(L2)
        eps = float(self.get_parameter('mline_eps').value)
        return (cte <= eps)

    def cone_min_at(self, center_ang, deg):
        if self.scan is None:
            return 10.0
        cone = math.radians(deg)
        i0 = int(((center_ang - cone/2) - self.ang0)/self.dang)
        i1 = int(((center_ang + cone/2) - self.ang0)/self.dang)
        n = len(self.scan)
        vals = []
        for i in range(max(0,i0), min(n,i1)):
            r = self.scan[i]
            if math.isfinite(r) and r>0.0:
                vals.append(r)
        return min(vals) if vals else 10.0

    def control(self):
        x,y,th = self.pose
        if self.start is None:
            self.start = (x,y)
        gx = float(self.get_parameter('goal_x').value); gy = float(self.get_parameter('goal_y').value)
        dgoal = dist(x,y,gx,gy)
        if dgoal < 0.12:
            self.on_state('ARRIVED'); self.on_cmd(0.0,0.0); return

        dfront = self.cone_min(float(self.get_parameter('obs_cone_deg').value))

        if self.state == 'MOTION_TO_GOAL':
            # Drive toward goal until obstacle
            if dfront < float(self.get_parameter('obs_dist').value):
                self.hit = (x,y); self.hit_dgoal = dgoal
                # Use configured follow_side (default 'left') to ensure consistent contour direction
                self.side = str(self.get_parameter('follow_side').value) or 'left'
                self.loop_t0 = time.time()
                self.on_state('FOLLOW_BOUNDARY'); return
            hdg = heading_to(x,y,gx,gy)
            err = (hdg - th + math.pi)%(2*math.pi) - math.pi
            v = float(self.get_parameter('v_lin').value)
            w = sat(1.2*err, float(self.get_parameter('v_ang').value))
            self.on_cmd(v,w)

        elif self.state == 'FOLLOW_BOUNDARY':
            # Wall-follow controller
            sign = 1.0 if (self.side or self.get_parameter('follow_side').value) == 'left' else -1.0
            dside = self.scan_at(sign*math.radians(60.0))
            target = float(self.get_parameter('follow_dist').value)
            clear  = float(self.get_parameter('clear_dist').value)
            vfb    = float(self.get_parameter('v_fb').value)
            vmin   = float(self.get_parameter('v_min').value)
            if dfront < float(self.get_parameter('emergency_dist').value):
                # Curved escape to avoid in-place spin loops
                self.on_cmd(0.06, sat(1.0*sign,1.2)); return
            band = 0.05
            if dside > target + band:
                w = sat(0.9*sign,1.2); v = vmin
            elif dside < target - band:
                w = sat(-0.9*sign,1.2); v = vmin
            else:
                # Straight-ish segment: go a bit faster if front is clear
                v_boost = 0.22 if (dfront > target + 0.2) else vfb
                w = 0.0; v = max(v_boost, vmin)
            self.on_cmd(v,w)

            # Bug2 leave condition: on M-line and closer to goal than at hit point
            hdg = heading_to(x,y,gx,gy)
            err = (hdg - th + math.pi)%(2*math.pi) - math.pi
            clear_goal = self.cone_min_at(err, 30.0) > clear and self.cone_min_at(0.0, 20.0) > clear
            if (self.hit_dgoal is not None and
                self.on_mline(x,y,gx,gy) and (dgoal + 1e-3 < self.hit_dgoal) and clear_goal and
                (time.time() - (self.loop_t0 or time.time())) > 0.8):
                self.leave_t0 = time.time()
                self.on_state('LEAVE_TO_GOAL'); return

        elif self.state == 'LEAVE_TO_GOAL':
            # Smoothly turn toward goal and move out of the obstacle vicinity
            hdg = heading_to(x,y,gx,gy)
            err = (hdg - th + math.pi)%(2*math.pi) - math.pi
            forward_clear = self.cone_min_at(0.0, 20.0)
            v = 0.18 if forward_clear > float(self.get_parameter('clear_dist').value) else 0.10
            w = sat(1.2*err, float(self.get_parameter('v_ang').value))
            self.on_cmd(v, w)
            # After short cooldown and alignment, go to MOTION_TO_GOAL
            if (time.time() - (self.leave_t0 or time.time())) > 0.8 and abs(err) < 0.5 and forward_clear > float(self.get_parameter('obs_dist').value):
                self.on_state('MOTION_TO_GOAL'); return
            # Safety: if blocked again, resume FOLLOW_BOUNDARY
            if dfront < float(self.get_parameter('obs_dist').value):
                self.on_state('FOLLOW_BOUNDARY'); return

def main():
    rclpy.init(); rclpy.spin(Bug2()); rclpy.shutdown()
