#!/usr/bin/env python
import rospy
from std_msgs.msg import String
from geometry_msgs.msg import Pose, Quaternion, Point, PoseStamped, PoseWithCovariance, TwistWithCovariance, Twist, Vector3, Wrench
from gazebo_msgs.srv import ApplyBodyWrench, SetModelState, GetModelState, SetLinkState, GetLinkState
import tf.transformations as tft
from numpy import float64
import time

rospy.wait_for_service('/gazebo/apply_body_wrench')
rospy.wait_for_service('/gazebo/set_model_state')
rospy.wait_for_service('/gazebo/set_link_state')
rospy.wait_for_service('/gazebo/get_model_state')
rospy.wait_for_service('/gazebo/get_link_state')
get_link_state = rospy.ServiceProxy('/gazebo/get_link_state', GetLinkState)
get_model_state = rospy.ServiceProxy('/gazebo/get_model_state', GetModelState)
set_link_state = rospy.ServiceProxy('/gazebo/set_link_state', SetLinkState)
set_model_state = rospy.ServiceProxy('/gazebo/set_model_state', SetModelState)
apply_body_wrench = rospy.ServiceProxy('/gazebo/apply_body_wrench', ApplyBodyWrench)

'''
   takes
     string link_name          # name of link
     string reference_frame    # reference frame of returned information, must be a valid link
                               # if empty, use inertial (gazebo world) frame
   returns  
     gazebo/LinkState link_state  # (string link_name , Pose pose, Twist twist, string reference_frame)  
     bool success              # return true if get info is successful
     string status_message     # comments if available
'''
def getLinkState(linkName, refFrame = 'world'):
    return get_link_state(link_name=str(linkName),reference_frame=refFrame)

'''
   takes
     string model_name                    # name of Gazebo Model
     string relative_entity_name          # return pose and twist relative to this entity
                                          # an entity can be a model, body, or geom
                                          # be sure to use gazebo notation (e.g. [model_name::body_name])
                                          # leave empty or "world" will use inertial world frame
   returns 
     geometry_msgs/Pose pose              # pose of model in relative entity frame
     geometry_msgs/Twist twist            # twist of model in relative entity frame
     bool success                         # return true if get successful
     string status_message                # comments if available
'''
def getModelState(modelName, relName):
    return get_model_state(model_name=str(modelName) , relative_entity_name=str(relEName)) 
'''
   takes
     gazebo/LinkState link_state 
   returns 
     bool success                # return true if set wrench successful
     string status_message       # comments if available

'''
def setLinkState(linkStateObj):
    return set_link_state(link_state=linkStateObj)

'''
    takes 
      gazebo/ModelState model_state
    returns
      bool success                  # return true if setting state successful
      string status_message         # comments if available
'''
def setModelState(modelStateObj):
    return SetModelState(model_state=modelStateObj)
'''
    takes 
      string body_name                          # Gazebo body to apply wrench (linear force and torque)
                                                # wrench is applied in the gazebo world by default
      string reference_frame                    # wrench is defined in the reference frame of this entity
                                                # use inertial frame if left empty
      geometry_msgs/Point  reference_point      # wrench is defined at this location in the reference frame
      geometry_msgs/Wrench wrench               # wrench applied to the origin of the body
      time start_time                           # (optional) wrench application start time (seconds)
                                                # if start_time is not specified, or
                                                # start_time < current time, start as soon as possible
      duration duration                         # optional duration of wrench application time (seconds)
                                                # if duration < 0, apply wrench continuously without end
                                                # if duration = 0, do nothing
                                                # if duration < step size, apply wrench
                                                # for one step size
    returns
      bool success                              # return true if set wrench successful
      string status_message                     # comments if available
'''
def applyBodyWrench(bName, refFrame, refPoint, wren, dur=0.5):
    return apply_body_wrench(body_name = str(bName),
                reference_frame = str(refFrame),
                reference_point = refPoint,
                wrench = wren,
                duration = rospy.Duration(dur))
            

def printLink(linkState):
    print "\n"
    print "X: ", str(linkState.pose.position.x)
    print "Y: ", str(linkState.pose.position.y)
    print "Z: ", str(linkState.pose.position.z)
           


'''
Reference : 

ModelState 
# Set Gazebo Model pose and twist
string model_name           # model to set state (pose and twist)
geometry_msgs/Pose pose     # desired pose in reference frame
geometry_msgs/Twist twist   # desired twist in reference frame
string reference_frame      # set pose/twist relative to the frame of this entity (Body/Model)
                            # leave empty or "world" or "map" defaults to world-frame

LinkState
string link_name            # link name
geometry_msgs/Pose pose     # desired pose in reference frame
geometry_msgs/Twist twist   # desired twist in reference frame
string reference_frame      # set pose/twist relative to the frame of this link/body
                            # leave empty or "world" or "map" defaults to world-frame


'''
