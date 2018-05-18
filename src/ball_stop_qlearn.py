#!/usr/bin/env python
import rospy
from std_msgs.msg import String
from geometry_msgs.msg import Pose, Quaternion, Point, PoseStamped, PoseWithCovariance, TwistWithCovariance, Twist, Vector3, Wrench
from gazebo_msgs.srv import ApplyBodyWrench, SetModelState, GetModelState, SetLinkState, GetLinkState
from gazebo_msgs.msg import LinkState
import tf.transformations as tft
from numpy import float64
from rosUtil import *
import pickle
import time

class State:
    def __init__(self,bx,ob1,ob2,ob3):
        self.x = bx
        self.y1 = ob1
        self.y2 = ob2
        self.y3 = ob3
    def __hash__(self):
        return hash((self.x, self.y1, self.y2, self.y3))
    def __eq__(self, other):
        return (self.x, self.y1, self.y2, self.y3) == (other.x, other.y1, other.y2, other.y3)

class StateAction:
    def __init__(self,state,action):
        self.state = state
        self.action = action
    def __hash__(self):
        return hash((self.state,self.action))
    def __eq__(self, other):
        return (self.state,self.action) == (other.state, other.action)


bx = 0.0
y1 = -8.0
y2 = 0.0
y3 = 8.0
num_episodes = 0
eps = 0.3
current = State(bx,y1,y2,y3)
previous = State(0.0,8.0,0.0,-8.0)
qStates = dict()
curReward = 0.0
prevReward = 0.0
episodes = [100, 500, 1000] #5000, 8000, 10000, 50000, 100000, 1000000]

def setLocalVars():
    global bx,y1,y2,y3, current, previous
    bx = getLinkState("link",'world').link_state.pose.position.x
    y1 = getLinkState("link_0",'world').link_state.pose.position.y
    y2 = getLinkState("link_1",'world').link_state.pose.position.y
    y3 = getLinkState("link_2",'world').link_state.pose.position.y
    
    
def sendBallPose():
    global bx 
    p = Pose(Point(bx,0.0,0.0), Quaternion(0.0,0.0,0.0,0.0))
    t = Twist(Vector3(0.0,0.0,0.0), Vector3(0.0,0.0,0.0))
    obj = LinkState('link',p,t,'world') 
    setLinkState(obj)

def sendObsPose():
    global y1,y2,y3
    p_0 = Pose(Point(9.0,y1,0.0), Quaternion(0.0,0.0,0.0,0.0))
    t_0 = Twist(Vector3(0.0,0.0,0.0), Vector3(0.0,0.0,0.0))
    obj_0 = LinkState('link_0',p_0,t_0,'world') 
    setLinkState(obj_0)
    p_1 = Pose(Point(9.0,y2,0.0), Quaternion(0.0,0.0,0.0,0.0))
    t_1 = Twist(Vector3(0.0,0.0,0.0), Vector3(0.0,0.0,0.0))
    obj_1 = LinkState('link_1',p_1,t_1,'world') 
    setLinkState(obj_1)
    p_2 = Pose(Point(9.0,y3,0.0), Quaternion(0.0,0.0,0.0,0.0))
    t_2 = Twist(Vector3(0.0,0.0,0.0), Vector3(0.0,0.0,0.0))
    obj_2 = LinkState('link_2',p_2,t_2,'world') 
    setLinkState(obj_2)

    
def incrementBallState():
    global bx, num_episodes, episodes
    if bx == 9.0:
        bx = 0.0
        num_episodes += 1
        if num_episodes in episodes:
            print(str(num_episodes) + " completed")
            pickleQStates(num_episodes)
    else :
        bx += 1
    sendBallPose()

def incrementObState():
    global bx, y1, y2, y3
    if y1 >= 8.0:
        y1 = -8.0
    else :
        y1+=1   
    if y2 >= 8.0:
        y2 = -8.0
    else :
        y2+=1 
    if y3 >= 8.0:
        y3 = -8.0
    else :
        y3+=1 
    sendObsPose()


def didCollide():
    global bx,y1,y2,y3
    if ((bx,0.0) == (9.0,y1)) or ((bx,0.0) == (9.0,y2)) or ((bx,0.0) == (9.0,y3)):
        return True
    else :
        return False

def didReachGoal():
    global bx,y1,y2,y3
    if (bx == 9.0) and (y1!=0) and (y2!=0) and (y3!=0):
        return True
    else :
        return False
           


def pickAction():
    global eps, current, previous
    import random
    p = random.uniform(0, 1)
    if p < eps:
        return random.choice([True,False])
    else:
        return maxStateAction(current)[0]

def addQValue(state):
    global qStates
    import random
    if (state not in qStates) and (state.x != 9) :
        qStates[state] = dict()
        qStates[state][True] = random.randint(-100,100)
        qStates[state][False] = random.randint(-100,100)
    if (state not in qStates) and (state.x == 9.0):
        qStates[state] = dict()    
        qStates[state][True]=0.0
        qStates[state][False] = 0.0
        

def maxStateAction(state):
    global qStates
    addQValue(state)
    if qStates[state][True] > qStates[state][False]:
        return (True,qStates[state][True])
    else:
        return (False,qStates[state][False])

def act(action, prev):
    global current, previous, bx, y1, y2, y3
    if action:
        incrementBallState()
        previous = current
        current = State(bx,y1,y2,y3)
        if didCollide():
            return -100.0
        elif didReachGoal() :
            return 100.0
        else :
            return 0.0
    else:
        return prev    

def qLearnUpdate(action, reward, alpha, gamma):
    global qStates, previous, current
    addQValue(current)
    addQValue(previous)
    qStates[previous][action] += alpha*(reward + gamma*(maxStateAction(current)[1]) - qStates[previous][action] )
    
def pickleQStates(num):
    global qStates
    output = open('data_'+str(num)+'.pkl', 'wb')
    pickle.dump(qStates,output)
    output.close()


while not rospy.is_shutdown():
    try:
        setLocalVars()
        addQValue(current)
        action = pickAction()
        curReward = act(action,prevReward)
        prevReward = curReward
        qLearnUpdate(action,curReward, 0.1, 0.4)
        incrementObState()
        #print(str(num_episodes) + " completed")
        if num_episodes > 1000 :
            break
                 
    except rospy.ServiceException as e:
        print e
        break
    #rospy.sleep(1)

