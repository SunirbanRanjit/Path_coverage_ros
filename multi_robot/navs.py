#! /usr/bin/env python

import rospy
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion
from geometry_msgs.msg import Point,Twist
from math import atan2,pi
from pathsequence import DARPinPoly
x_a=0.0
y_a=0.0
theta_a = 0.0

x_b=0.0
y_b=0.0
theta_b = 0.0
### DARP Values

MaxIter = 80000
CCvariation = 0.01
randomLevel = 0.0001
dcells = 2
importance = False
rows=8
cols=8
nep=False
map_index=int(input("Enter map Index: "))
visual=bool(int(input("Want to visualize paths? 1 for yes, 0 for no: ")))
initial_positions=[(0,0),(0,7)]
portions=[0.5,0.5]
maps=[]
map_1=[]
map_2=[[1,1],[1,2],[2,1],[2,2],[1,5],[2,5],[1,6],[2,6],[5,5],[5,6],[6,5],[6,6],[5,1],[5,2],[6,1],[6,2]]
map_3=[[0,3],[1,3],[4,5],[4,6],[4,7]]
map_4=[[2,3],[3,3],[4,3],[5,3],[6,3],[7,3],[3,4],[4,4],[5,4],[6,4],[7,4],[3,5],[4,5],[5,5],[6,5],[7,5],[3,6],[4,6],[5,6],[6,6],[7,6],[3,7],[4,7],[5,7],[6,7],[7,7],[0,3]]
maps.append(map_1)
maps.append(map_2)
maps.append(map_3)
maps.append(map_4)
obstacles_positions=maps[map_index]
vis=visual
###
### Declear darp
poly = DARPinPoly(rows, cols, MaxIter, CCvariation, randomLevel, dcells, importance, nep, initial_positions, portions, obstacles_positions,vis)
a=poly.clearPath()

###

def callback_a(msg):
    global x_a
    global y_a
    global theta_a

    x_a = msg.pose.pose.position.x
    y_a = msg.pose.pose.position.y

    rot_q=msg.pose.pose.orientation
    (roll,pitch,theta_a)=euler_from_quaternion([rot_q.x, rot_q.y, rot_q.z, rot_q.w])

def callback_b(msg):
    global x_b
    global y_b
    global theta_b

    x_b = msg.pose.pose.position.x
    y_b = msg.pose.pose.position.y

    rot_q=msg.pose.pose.orientation
    (roll,pitch,theta_b)=euler_from_quaternion([rot_q.x, rot_q.y, rot_q.z, rot_q.w])

rospy.init_node('get_pos', anonymous=True)
odom_sub_a = rospy.Subscriber('/robot1/odom', Odometry, callback_a)
pub_a = rospy.Publisher('/robot1/cmd_vel',Twist,queue_size=1)
speed_a = Twist()

odom_sub_b = rospy.Subscriber('/robot2/odom', Odometry, callback_b)
pub_b = rospy.Publisher('/robot2/cmd_vel',Twist,queue_size=1)
speed_b = Twist()
r = rospy.Rate(10)
goala = Point()
goalb=Point()
flag_1=False
flag_2=False
xa=int(0)
xb=int(0)
after_scaling_a=[]
after_scaling_b=[]
for i in range(len(a)):
    temp=[]
    temp1=[]
    x1,y1,x2,y2=a[i]
    x_1,y_1,x_2,y_2=(x1/4),(y1/4),(x2/4),(y2/4)
    temp.append(x_1)
    temp.append(y_1)
    after_scaling_a.append(temp)
    temp1.append(x_2)
    temp1.append(y_2)
    after_scaling_b.append(temp1)

#a=[[0,0],[0,1],[1,1],[-1,1]]
(goala.x,goala.y)=after_scaling_a[0]
(goalb.x,goalb.y)=after_scaling_b[0]
after_scaling_a.append([goala.x,goala.y])
after_scaling_b.append([goalb.x,goalb.y])
print(after_scaling_a,after_scaling_b)
turn_pi_1=False
turn_pi_2=False
while not rospy.is_shutdown():
    
    if flag_1:
        turn_pi_1=False
        for i in range(xa,len(after_scaling_a)):       
            (goala.x,goala.y)=after_scaling_a[i]
            flag_1=False
            xa=i+1
            break
    
    if flag_2:
        turn_pi_1=False
        for i in range(xb,len(after_scaling_b)):       
            (goalb.x,goalb.y)=after_scaling_b[i]
            flag_2=False
            xb=i+1
            break
    
    print(goala.x,goala.y,xa)
    print(goalb.x,goalb.y,xb)
    inc_x_1 = goala.x - x_a
    inc_y_1 = goala.y - y_a
    angle_to_goal_a = atan2(inc_y_1,inc_x_1)

    inc_x_2 = goalb.x - x_b
    inc_y_2 = goalb.y - y_b
    angle_to_goal_b = atan2(inc_y_2,inc_x_2)
    if abs(inc_x_1)<=0.1 and abs(inc_y_1)<=0.1:
        flag_1=True
    if abs(inc_x_2)<=0.1 and abs(inc_y_2)<=0.1:
        flag_2=True    
    
    abc_a=angle_to_goal_a-theta_a
    print(abc_a)
    
    if abs(abc_a) <= 0.1:
        speed_a.linear.x=0.5
        speed_a.angular.z=0.0
    elif abc_a>=0 and abc_a<=pi:
        speed_a.linear.x=0
        speed_a.angular.z=1.0
        #    speed_b.angular.z=-1.0
    elif abc_a<=-pi and abc_a>=-(2*pi):
        speed_a.linear.x=0
        speed_a.angular.z=0.5
    elif abc_a>=pi and abc_a<=(2*pi):
        speed_a.linear.x=0
        speed_a.angular.z=-0.5
    elif abc_a<=0 and abc_a>=-pi:
        speed_a.linear.x=0
        speed_a.angular.z=-1.0

    if xa==len(after_scaling_a) and flag_1:
        speed_a.linear.x=0.0
        speed_a.angular.y=0.0

    

    abc_b=angle_to_goal_b-theta_b
    print(abc_b)
    
    if abs(abc_b) <= 0.1:
        speed_b.linear.x=0.5
        speed_b.angular.z=0.0
    elif abc_b>=0 and abc_b<=pi:
        speed_b.linear.x=0
        speed_b.angular.z=1.0
        #    speed_b.angular.z=-1.0
    elif abc_b<=-pi and abc_b>=-(2*pi):
        speed_b.linear.x=0
        speed_b.angular.z=0.5
    elif abc_b>=pi and abc_b<=(2*pi):
        speed_b.linear.x=0
        speed_b.angular.z=-0.5
    elif abc_b<=0 and abc_b>=-pi:
        speed_b.linear.x=0
        speed_b.angular.z=-1.0
   
    if xb==len(after_scaling_b) and flag_2:
        speed_b.linear.x=0.0
        speed_b.angular.y=0.0

  
    pub_a.publish(speed_a)
    pub_b.publish(speed_b)
    r.sleep()
    
    