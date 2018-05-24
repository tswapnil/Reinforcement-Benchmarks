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

def setLocalVars():
    global bx,y1,y2,y3, current, previous
    bx = round(getLinkState("link",'world').link_state.pose.position.x)
    y1 = round(getLinkState("link_0",'world').link_state.pose.position.y)
    y2 = round(getLinkState("link_1",'world').link_state.pose.position.y)
    y3 = round(getLinkState("link_2",'world').link_state.pose.position.y)

bx = 0.0
y1 = -3.0
y2 = 0.0
y3 = 3.0
x_ob1 = 8.0
x_ob2 = 9.0
x_ob3 = 10.0
setLocalVars()
num_episodes = 0
eps = 0.3
current = State(bx,y1,y2,y3)
previous = State(0.0,8.0,0.0,-8.0)
qStates = dict()
curReward = 0.0
prevReward = 0.0
#episodes = [2000, 3500, 5000, 7500, 10000, 15000, 20000, 50000, 100000] #5000, 8000, 10000, 50000, 100000, 1000000]
episodes = [100, 200, 300, 400, 500, 600, 700, 800, 900, 1000]

    
    
def sendBallPose():
    global bx 
    p = Pose(Point(bx,0.0,0.0), Quaternion(0.0,0.0,0.0,0.0))
    t = Twist(Vector3(0.0,0.0,0.0), Vector3(0.0,0.0,0.0))
    obj = LinkState('link',p,t,'world') 
    setLinkState(obj)

def sendObsPose():
    global y1,y2,y3, x_ob1, x_ob2, x_ob3
    p_0 = Pose(Point(x_ob1,y1,0.0), Quaternion(0.0,0.0,0.0,0.0))
    t_0 = Twist(Vector3(0.0,0.0,0.0), Vector3(0.0,0.0,0.0))
    obj_0 = LinkState('link_0',p_0,t_0,'world') 
    setLinkState(obj_0)
    p_1 = Pose(Point(x_ob2,y2,0.0), Quaternion(0.0,0.0,0.0,0.0))
    t_1 = Twist(Vector3(0.0,0.0,0.0), Vector3(0.0,0.0,0.0))
    obj_1 = LinkState('link_1',p_1,t_1,'world') 
    setLinkState(obj_1)
    p_2 = Pose(Point(x_ob3,y3,0.0), Quaternion(0.0,0.0,0.0,0.0))
    t_2 = Twist(Vector3(0.0,0.0,0.0), Vector3(0.0,0.0,0.0))
    obj_2 = LinkState('link_2',p_2,t_2,'world') 
    setLinkState(obj_2)

    
def incrementBallState():
    global bx, num_episodes, episodes, x_ob3
    if bx == x_ob3:
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
    if y1 <= -8.0:
        y1 = 8.0
    else :
        y1-=1   
    if y2 >= 8.0:
        y2 = -8.0
    else :
        y2+=2 
    if y3 >= 8.0:
        y3 = -8.0
    else :
        y3+=1 
    sendObsPose()


def didCollide():
    global current, x_ob1, x_ob2, x_ob3
    bx = current.x
    y1 = current.y1
    y2 = current.y2
    y3 = current.y3
    if (bx == x_ob1) and (y1 == 0.0):
        #print("Collision .... Ah ")
        return True
    elif (bx == x_ob2) and ( y2 == 0.0 or y2 == 1.0):
        #print("Collision .... Ah ")
        return True
    elif (bx == x_ob3) and (y3 == 0.0):
        #print("Collision .... Ah ")
        return True
    else :
        return False

def didReachGoal():
    global current, x_ob1, x_ob2, x_ob3
    bx = current.x
    y1 = current.y1
    y2 = current.y2
    y3 = current.y3
    if (bx == x_ob1) and (y1!=0.0) :
        #print("Hurray .. Goal Reached")
        return True
    elif (bx == x_ob2) and (y2!=0.0):
        #print("Hurray .. Goal Reached")
        return True
    elif (bx == x_ob3) and (y3!=0.0):
        #print("Hurray .. Goal Reached")
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
    global qStates, x_ob1, x_ob2, x_ob3
    if state in qStates:
        return
    import random
    if (state not in qStates) and (state.x != x_ob1) and (state.x != x_ob2) and (state.x != x_ob3) :
        qStates[state] = dict()
        qStates[state][True] = random.randint(-100,100)
        qStates[state][False] = random.randint(-100,100)
    if (state not in qStates) and (state.x == x_ob1 or state.x == x_ob2 or state.x == x_ob3):
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
    incrementObState()
    rew = 0.0
    if action:
        incrementBallState()
        if didCollide():
            rew = -10000.0
        elif didReachGoal() :
            rew = 100.0
        else :
            rew = 0.0
    else:
        if didCollide():
            #incrementBallState()
            rew = -10000.0
        elif didReachGoal():
            #incrementBallState()
            rew = 100.0
        else:
            rew = 0.0 
    previous = current
    current = State(bx,y1,y2,y3)   
    return rew

def qLearnUpdate(action, reward, alpha, gamma):
    global qStates, previous, current
    addQValue(current)
    addQValue(previous)
    qStates[previous][action] += alpha*(reward + gamma*(maxStateAction(current)[1]) - qStates[previous][action] )
    
def pickleQStates(num):
    global qStates
    output = open('normal_'+str(num)+'.pkl', 'wb')
    pickle.dump(qStates,output)
    output.close()

iterations = 0
while not rospy.is_shutdown():
    try:
        iterations += 1
        #setLocalVars()
        addQValue(current)
        action = pickAction()
        curReward = act(action,prevReward)
        prevReward = curReward
        qLearnUpdate(action,curReward, 0.1,1)
        #if iterations%2 == 0:
        #print("Iteration "+str(iterations))
        #print(action)
        #print(str(bx) + " " + str(y1) + " " + str(y2) + " " + str(y3))
        #rospy.sleep(1);
        
        #print(str(num_episodes) + " completed")
        if num_episodes > max(episodes) :
            break
                 
    except rospy.ServiceException as e:
        print e
        break
    #rospy.sleep(1)

