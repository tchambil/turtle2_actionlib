#! /usr/bin/env python

import roslib
roslib.load_manifest('turtle2_actionlib')
import rospy
import actionlib
import turtle2_actionlib.msg
import sys, select, termios, tty

def Turtle_client(n):
    client = actionlib.SimpleActionClient('turtle_dance', turtle2_actionlib.msg.MoonWalkAction)
    client.wait_for_server()
    goal = turtle2_actionlib.msg.MoonWalkGoal(step = n)
    client.send_goal(goal)

    #while(client.get_state()!=10):
    print "Esperando ..."
    client.wait_for_result(rospy.Duration.from_sec(30))
    # print client.get_result()
    return client.get_result()

if __name__ == '__main__':
    rospy.init_node('moonwalk_client')
    while True:
        n = int(raw_input("Step:"))
        result = Turtle_client(n)
        print result.success
    
    # if result.oddpair == 1:
    # 	print "el numero es par"
    # else:
    # 	print "el numero es impar"
    # Fill in the goal here

