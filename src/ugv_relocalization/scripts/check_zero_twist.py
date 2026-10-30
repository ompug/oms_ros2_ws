#!/usr/bin/env python3
"""Read-only safety check: require observed zero Twist throughout a wall-time window."""
import argparse
import time
import rclpy
from geometry_msgs.msg import Twist
from rclpy.qos import qos_profile_sensor_data
p=argparse.ArgumentParser(description=__doc__)
p.add_argument('--topic',default='/cmd_vel');p.add_argument('--seconds',type=float,default=3.)
a=p.parse_args()
rclpy.init();n=rclpy.create_node('zero_twist_observer');seen=[]
def check(m):
    seen.append(m)
    if any(abs(x)>1e-6 for x in [m.linear.x,m.linear.y,m.linear.z,m.angular.x,m.angular.y,m.angular.z]):
        raise RuntimeError('NONZERO Twist observed; engage physical e-stop')
n.create_subscription(Twist,a.topic,check,qos_profile_sensor_data)
try:
    deadline=time.monotonic()+a.seconds
    while time.monotonic()<deadline:
        rclpy.spin_once(n,timeout_sec=.1)
    if not seen:
        raise RuntimeError('No Twist received: zero output has NOT been demonstrated')
    print(f'PASS: {len(seen)} zero Twist messages on {a.topic}')
finally:
    n.destroy_node();rclpy.shutdown()
