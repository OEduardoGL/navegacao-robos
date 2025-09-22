import rclpy, math
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
from std_msgs.msg import String
from .utils import dist, heading_to, sat

class TangentBug(Node):
    def __init__(self):
        super().__init__('tangent_bug_node')
        self.declare_parameter('goal_x', 7.5)
        self.declare_parameter('goal_y', -8.5)
        self.declare_parameter('v_lin', 0.24)
        self.declare_parameter('v_ang', 1.8)
        self.declare_parameter('obs_dist', 0.35)
        self.pose=(0.0,0.0,0.0); self.scan=None; self.ang0=0.0; self.dang=0.0
        self.state='MOTION_TO_GOAL'
        self.best_h=1e9
        self.create_subscription(Odometry,'/odom',self.on_odom,10)
        self.create_subscription(LaserScan,'/scan',self.on_scan,qos_profile_sensor_data)
        self.pub_cmd=self.create_publisher(Twist,'/cmd_vel',10)
        self.pub_state=self.create_publisher(String,'/bug_state',10)
        self.timer=self.create_timer(0.05,self.control)

    def on_state(self,s): self.state=s; self.pub_state.publish(String(data=f'TANGENT:{s}'))
    def on_cmd(self,v,w): t=Twist(); t.linear.x=v; t.angular.z=w; self.pub_cmd.publish(t)
    def on_odom(self,msg):
        x=msg.pose.pose.position.x; y=msg.pose.pose.position.y; q=msg.pose.pose.orientation
        siny=2*(q.w*q.z+q.x*q.y); cosy=1-2*(q.y*q.y+q.z*q.z)
        self.pose=(x,y,math.atan2(siny,cosy))
    def on_scan(self,msg): self.scan=list(msg.ranges); self.ang0=msg.angle_min; self.dang=msg.angle_increment

    def control(self):
        x,y,th=self.pose; gx=float(self.get_parameter('goal_x').value); gy=float(self.get_parameter('goal_y').value)
        dgoal=dist(x,y,gx,gy)
        if dgoal<0.12: self.on_state('ARRIVED'); self.on_cmd(0.0,0.0); return
        if self.state=='MOTION_TO_GOAL':
            dmin=self.min_scan()
            if dmin< float(self.get_parameter('obs_dist').value):
                self.best_h=1e9; self.on_state('FOLLOW_BOUNDARY'); return
            hdg=heading_to(x,y,gx,gy)
            err=(hdg-th+math.pi)%(2*math.pi)-math.pi
            v=float(self.get_parameter('v_lin').value); w=sat(err,float(self.get_parameter('v_ang').value))
            self.on_cmd(v,w)
        elif self.state=='FOLLOW_BOUNDARY':
            if self.scan is None: self.on_cmd(0.0,0.0); return
            candidates=[]
            n=len(self.scan)
            for i in range(2,n-2):
                r=self.scan[i]; r1=self.scan[i-2]; r2=self.scan[i+2]
                if not (r and r>0.0 and math.isfinite(r)): continue
                if r1 and r2 and abs(r1-r)>0.25 or abs(r2-r)>0.25:
                    ang=self.ang0+i*self.dang
                    nx=x+r*math.cos(th+ang); ny=y+r*math.sin(th+ang)
                    h=dist(x,y,nx,ny)+dist(nx,ny,gx,gy)
                    candidates.append((h,ang))
            if candidates:
                candidates.sort(key=lambda t:t[0])
                h,ang=candidates[0]
                if h<self.best_h-0.02: self.best_h=h
                elif h>self.best_h+0.25:
                    self.on_state('MOTION_TO_GOAL'); return
                v=0.18; w=sat(1.2*ang,1.2); self.on_cmd(v,w)
            else:
                # keep a safe turn if no edges detected
                self.on_cmd(0.12, 0.6)

    def min_scan(self):
        if self.scan is None: return 10.0
        vals=[r for r in self.scan if r and r>0.0 and math.isfinite(r)]
        return min(vals) if vals else 10.0

def main():
    rclpy.init(); rclpy.spin(TangentBug()); rclpy.shutdown()

