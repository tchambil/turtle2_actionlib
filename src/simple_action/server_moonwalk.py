#! /usr/bin/env python

import roslib
roslib.load_manifest('turtle2_actionlib')
import rospy
import actionlib
import turtle2_actionlib.msg
from geometry_msgs.msg import Twist,Point
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion
import math

class TurtleServer(object):
  _feedback = turtle2_actionlib.msg.MoonWalkFeedback()
  _result = turtle2_actionlib.msg.MoonWalkResult()

  dist_to_start = 0
  current_angle = 0
  max_dist = 1.0
  x_velocity = 0.2
  angular_velocity = -0.4

  def reciver_odom(self,data):
    global current_angle
    global dist_to_start
    rospy.loginfo
    pose=data.pose.pose
    quaternion = (pose.orientation.x,pose.orientation.y,pose.orientation.z,pose.orientation.w)
    (roll , pitch , yaw) = euler_from_quaternion(quaternion)
    tAngle = math.degrees(yaw)
    if (tAngle<0):
       tAngle = tAngle+360
    current_angle = tAngle
    self.dist_to_start = math.sqrt(pow(data.pose.pose.position.x,2)+pow(data.pose.pose.position.y,2)) #calculo de la distancia con x2 e y2 = 0

  def __init__(self):
    self.server = actionlib.SimpleActionServer('turtle_dance', turtle2_actionlib.msg.MoonWalkAction, self.execute, False)
    self.server.start()
    print "server running!!"
  
  def dance(self,step):
    #rospy.init_node('moonwalk')
    pub = rospy.Publisher('mobile_base/commands/velocity', Twist, queue_size=10)
    rospy.Subscriber('odom',Odometry, self.reciver_odom)  
    rate = rospy.Rate(10) # 10hz
    twist = Twist()
    rospy.sleep(1)
    if step==1 or step==4:
      self.spin(twist,pub,rate,360)
    elif step==2 or step==5:
      self.spin(twist,pub,rate,90,3)
    elif step==3:
      self.go_back(twist,pub,rate,-1)
    elif step==6: 
      self.go(twist,pub,rate,1)
    
    

#Paso 1,2,4,5
  def spin(self,twist,pub,r,max_angle,diff = 0):
    global current_angle
    twist = Twist()
    twist.angular.z = self.angular_velocity
    twist
    if max_angle == 360:
        print "360"
        while current_angle > 3.0:
            print "angulo actual _360:",current_angle
            print "diff angulo actual y max angle:",abs(current_angle - max_angle) 
            pub.publish(twist)
            r.sleep()
    else:
        max_angle = 360 - max_angle
        while abs(max_angle - current_angle) > diff:
            print "angulo actual_90:",current_angle
            print "diff angulo actual y max angle:",abs(current_angle - max_angle) 
            pub.publish(twist)
            r.sleep()
    rospy.sleep(4)

  def go_back(self,twist,pub,r,direction):
    global dist_to_start
    twist.linear.x = direction*self.x_velocity
    twist.linear.y=0.0
    twist.linear.z=0.0
    twist.angular.z = 0.0 
    while abs(self.dist_to_start - self.max_dist) > 0.2:
        print "retrocediendo ..."
        print "distancia al inicio:",self.dist_to_start
        print "diff al inicio y actual:",abs(self.dist_to_start - self.max_dist)
        pub.publish(twist)
        r.sleep()
    rospy.sleep(1)

 #Paso 3
  def go(self,twist,pub,r,direction):
    global dist_to_start
    twist = Twist()
    twist.linear.x = direction*self.x_velocity
    twist.linear.y=0.0
    twist.linear.z=0.0
    twist.angular.z = 0.0  
    print "Avanzando . . ."
    print "diff al inicio y actual:",abs(self.dist_to_start - self.max_dist)
    while abs(self.dist_to_start - self.max_dist) <0.8:
        print "Avanzando . . ."
        print "distancia al inicio:",self.dist_to_start
        print "diff al inicio y actual:",abs(self.dist_to_start - self.max_dist)
        pub.publish(twist)
        r.sleep()
    rospy.sleep(1)

  def execute(self, goal):
    # Do lots of awesome groundbreaking robot stuff here
    rospy.Rate(1)
    if self.server.is_preempt_requested():
      self._feedback = "ocupado"
      self._as.publish_feedback(self._feedback)
      # self.server.set_preempted()

    self.dance(goal.step)
    self._feedback.status = "algo"
    self.server.publish_feedback(self._feedback)
    self._result.success = "Step "+str(goal.step)+" finished"
    print self._result
    self.server.set_succeeded(self._result)      

if __name__ == '__main__':
  rospy.init_node('moonwalk_server')
  server = TurtleServer()
  rospy.spin()