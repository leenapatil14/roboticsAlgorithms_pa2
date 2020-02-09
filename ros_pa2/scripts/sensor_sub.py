#!/usr/bin/env python
import rospy
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry
from math import *
import tf
import a_star   

#find euclidean distance and orientation of goal node
def get_target(start_position,next_goal):
   
   #print("here",start_position,next_goal)
   y_diff=(next_goal[1]-start_position.position.y)
   x_diff=(next_goal[0]-start_position.position.x)
   distance=sqrt((next_goal[0]-start_position.position.x)**2+(next_goal[1]-start_position.position.y)**2)
   target=0
   if(x_diff!=0):
      mline_inclination=atan2(y_diff,x_diff)
      target=degrees(mline_inclination)
      if(target<0):
         target=360+target
   return target,distance

#rotate agent and move or stop based on distance from goal node
def rotate_to_opening(min_angle,input_euler,goal_dist):
   global upcoming
   #print(min_angle,input_euler,(min_angle-input_euler))
   msg_twist=Twist()  
   ang=radians(fabs(min_angle-input_euler))
   #print(ang)
   if(ang>0.1):
      #print(" rotating")
      msg_twist.angular.z=0.3
      msg_twist.linear.x=0.0
   else:
      #print("stop rotating")
      if(goal_dist<0.4):
         msg_twist.linear.x=0.0
         msg_twist.angular.z=0.0
         #set goal as the next element in global map
         if(len(odomFrame)-1!=upcoming):
            upcoming=upcoming+1
      
      else:
         msg_twist.linear.x=0.5
         msg_twist.angular.z=0.0
         #print("stop rotating")
         #print("move",goal_dist)
      
      
   publisher_demo=rospy.Publisher('/cmd_vel',Twist,queue_size=1)
   publisher_demo.publish(msg_twist)


#Get laser-scan data
def callback(data):
   global range_array
   global int_range
   range_array=data.ranges
   int_range=range(len(range_array))
   
#vfh
def vfh(current_position,next_goal):
   #print(current_position,next_goal)
   global range_array
   global int_range
   global upcoming
   data=[]
   openings=[]
   bins = range(0, 361, 10)
   #print(bins)
   #count number of obstacles in every bin
   for i in range(len(bins)-1):
      current_count=0
      for j in range(10):
         #print(bins[i]+j)
         if(range_array[bins[i]+j]<3.0):
               current_count=current_count+1
      data.append(current_count)       
   #print((data))
   prev=15
   min_value=999999
   min_angle=0
   current_euler=0
   for i in range(len(data)):
      if data[i]<=3:
         #get middle range value from 10 elements of single bin
         chosen=(i*10)+5
         #print(chosen)
         #current orientation
         initial_quaternion=(
            current_position.orientation.x,
            current_position.orientation.y,
            current_position.orientation.z,
            current_position.orientation.w
         )
         input_euler=tf.transformations.euler_from_quaternion(initial_quaternion)
         current_euler=degrees(input_euler[2])
         #convert negative angles to positive
         if (current_euler < 0):
            current_euler=360+current_euler
         #print(current_euler)
         #get orientation of next block in a star path
         goal_target,goal_dist=get_target(current_position,next_goal)
        
         target=0.8*fabs(chosen-goal_target)
         current=0.3*fabs(chosen-current_euler)
         previous=0.1*fabs(chosen-prev)
         cost=target+current+previous
         #print(chosen,goal_target,goal_dist,current_euler,target,current,previous,cost)
         #store only the minimum value of cost
         if(cost<min_value):
            min_value=cost
            min_angle=chosen
         prev=chosen
   #print("min: ",min_angle)
   #if(min_angle<175):
   #   min_angle=360+min_angle
   #upcoming=False
   #rotate 
   rotate_to_opening(min_angle,current_euler,goal_dist)
   
#a-star
def get_a_star():
   global isStart
   global current_position
   global odomFrame
   start=[int(abs(9-current_position.position.y)),int(abs(current_position.position.x+8))]
   goalx=rospy.get_param("/goalx")
   goaly=rospy.get_param("/goaly")
   goal_position=[int(abs(9-goaly)),int(abs(goalx+9))]
   global_path=a_star.impl_astar(start,goal_position)
   #print(global_path)
   global_path.remove(None)
   print("Global Path after A-Star: ",global_path)
   odomFrame=[]
   #map the map array in a-star with odom frame 
   for row in range(len(global_path)):
      odomFramey=9-global_path[row][0]
      odomFramex=global_path[row][1]-8
      odomFrameElem=[odomFramex,odomFramey]
      odomFrame.append(odomFrameElem)
   
   
   isStart=False

#get current position- at first callback, execute astar, later execute vfh
def callback_base_pose_ground_truth(data):
   global current_position
   global odomFrame
   global upcoming
   current_position=data.pose.pose
   if(isStart):
      get_a_star()
   else:
      #get goal values using rosparams
      goalx=rospy.get_param("/goalx")
      goaly=rospy.get_param("/goaly")
      goal_position=[goalx,goaly]
      element=odomFrame[upcoming]
      #print("goal: ",goal_position)
      if(element != goal_position):
         
         #print("ok",element,upcoming)
         vfh(current_position,element)

   
   
global current_position
global goal_position
global isStart
global odomFrame
global range_array
global int_range
global upcoming
isStart=True
upcoming=1
def listener():

   rospy.init_node('sensor_sub', anonymous=False)
   rospy.Subscriber('base_pose_ground_truth',Odometry, callback_base_pose_ground_truth)


   
   rospy.Subscriber('base_scan', LaserScan, callback)
   
    # spin() simply keeps python from exiting until this node is stopped
   rospy.spin()

if __name__ == '__main__':
    listener()
