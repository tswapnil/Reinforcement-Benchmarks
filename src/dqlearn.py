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
import tensorflow as tflow
from collections import deque


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
y1 = 0.0
y2 = 0.0
y3 = 0.0
num_episodes = 0
eps = 0.3
current = State(bx,y1,y2,y3)
previous = State(0.0,0.0,0.0,0.0)
qStates = dict()
curReward = 0.0
prevReward = 0.0
stack_size = 4
stacked_states = deque([np.zeros((1,4), dtype=np.float) for i in range(stack_size)],maxlen=4)

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

def incrementBallState():
    global bx, num_episodes
    if bx == 9.0:
        bx = 0.0
        num_episodes += 1
    else :
        bx += 1
    sendBallPose()


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
    qStates[previous][action] += alpha*(reward + gamma*(maxStateAction(current)[1]) - qStates[previous][action] )
    
def pickleQStates(num):
    global qStates
    output = open('data_'+str(num)+'.pkl', 'wb')
    pickle.dump(qStates,output)
    output.close()

def stack_states(stacked_states, state_obj, is_new_episode):
    global bx, y1, y2, y3, stack_size
    state = [state_obj.x, state_obj.y1, state_obj.y2, state_obj.y3]
    if is_new_episode:
        stacked_states = deque([np.zeros((1,4), dtype=np.float) for i in range(stack_size)],maxlen=4)
        for i in range(stack_size):
            stacked_states.append(state)
        stacked_state = np.stack(stacked_states,axis=2)
    else:
        stacked_states.append(state)
        stacked_state = np.stack(stacked_states,axis=2)
    return stacked_state, stacked_states


### MODEL HYPERPARAMETERS
state_size = [1, 4, 4]      # Our input is a stack of 4 frames hence 1x4x4 (Width, height, channels) 
action_size = 2 # 2 possible actions
learning_rate =  0.00025      # Alpha (aka learning rate)

### TRAINING HYPERPARAMETERS
total_episodes = 50            # Total episodes for training
max_steps = 9              # Max possible steps in an episode
batch_size = 64                # Batch size

# Exploration parameters for epsilon greedy strategy
explore_start = 1.0            # exploration probability at start
explore_stop = 0.01            # minimum exploration probability 
decay_rate = 0.00001           # exponential decay rate for exploration prob

# Q learning hyperparameters
gamma = 0.9                    # Discounting rate

### MEMORY HYPERPARAMETERS
pretrain_length = batch_size   # Number of experiences stored in the Memory when initialized for the first time
memory_size = 10000          # Number of experiences the Memory can keep

### PREPROCESSING HYPERPARAMETERS
stack_size = 4                 # Number of frames stacked

### MODIFY THIS TO FALSE IF YOU JUST WANT TO SEE THE TRAINED AGENT
training = True

## TURN THIS TO TRUE IF YOU WANT TO RENDER THE ENVIRONMENT
episode_render = False

class DQNetwork:
    def __init__(self, state_size, action_size, learning_rate, name='DQNetwork'):
        self.state_size = state_size
        self.action_size = action_size
        self.learning_rate = learning_rate
        
        with tflow.variable_scope(name):
            # We create the placeholders
            # *state_size means that we take each elements of state_size in tuple hence is like if we wrote
            # [None, 4, 4, 4]
            self.inputs_ = tflow.placeholder(tf.float32, [None, *state_size], name="inputs")
            self.actions_ = tflow.placeholder(tf.float32, [None, self.action_size], name="actions_")
            
            # Remember that target_Q is the R(s,a) + ymax Qhat(s', a')
            self.target_Q = tflow.placeholder(tf.float32, [None], name="target")
            
            self.flatten = tflow.contrib.layers.flatten(self.inputs_)
            
            self.fc = tflow.layers.dense(inputs = self.flatten,
                                         units = 512,
                                         activation = tflow.nn.elu,
                                         kernel_initializer=tflow.contrib.layers.xavier_initializer(),
                                         name="fc1")
            
            self.output = tf.layers.dense(inputs = self.fc, 
                                          kernel_initializer=tflow.contrib.layers.xavier_initializer(),
                                          units = self.action_size, 
                                          activation=None)
              
            # Q is our predicted Q value.
            self.Q = tflow.reduce_sum(tflow.multiply(self.output, self.actions_))
            
            # The loss is the difference between our predicted Q_values and the Q_target
            # Sum(Qtarget - Q)^2
            self.loss = tflow.reduce_mean(tflow.square(self.target_Q - self.Q))
            
            self.optimizer = tflow.train.AdamOptimizer(self.learning_rate).minimize(self.loss)


# Reset the graph
tflow.reset_default_graph()

# Instantiate the DQNetwork
DQN = DQNetwork(state_size, action_size, learning_rate)

class Memory():
    def __init__(self, max_size):
        self.buffer = deque(maxlen = max_size)
    
    def add(self, experience):
        self.buffer.append(experience)
    
    def sample(self, batch_size):
        buffer_size = len(self.buffer)
        index = np.random.choice(np.arange(buffer_size),
                                size = batch_size,
                                replace = False)
        
        return [self.buffer[i] for i in index]



episodes = [100, 500, 1000, 5000]#, 8000, 10000, 50000, 100000, 1000000]
while not rospy.is_shutdown():
    try:
        setLocalVars()
        addQValue(current)
        action = pickAction()
        curReward = act(action,prevReward)
        prevReward = curReward
        qLearnUpdate(action,curReward, 0.1, 0.4)
        #print(str(num_episodes) + " completed")
        if num_episodes > 1000000 :
            break
        if num_episodes in episodes:
            print(str(num_episodes) + " completed")
            pickleQStates(num_episodes)         
    except rospy.ServiceException as e:
        print e
        break
    #rospy.sleep(1)

