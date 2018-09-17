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
    def __init__(self,bx,ob1,ob2,ob3,vel1,vel2,vel3):
        self.x  = bx
        self.y1 = ob1
        self.y2 = ob2
        self.y3 = ob3
        self.v1 = vel1
        self.v2 = vel2
        self.v3 = vel3 
    def __hash__(self):
        return hash((self.x, self.y1, self.y2, self.y3, self.v1, self.v2, self.v3))
    def __eq__(self, other):
        return (self.x, self.y1, self.y2, self.y3, self.v1, self.v2, self.v3) == (other.x, other.y1, other.y2, other.y3, other.v1, other.v2, other.v3)

class StateAction:
    def __init__(self,state,action):
        self.state = state
        self.action = action
    def __hash__(self):
        return hash((self.state,self.action))
    def __eq__(self, other):
        return (self.state,self.action) == (other.state, other.action)


bx = 0.0
y1 = -3.0
y2 = 0.0
y3 = 3.0
x_ob1 = 8.0
x_ob2 = 9.0
x_ob3 = 10.0
v1 = 1
v2 = 1
v3 = 1
num_episodes = 0
episodes = [10,20,30,40,50,60,70,80,90,100]
eps = 0.3
current = State(bx,y1,y2,y3, v1, v2, v3)
previous = State(0.0,-3.0,0.0,3.0, 1.0, 1.0, 1.0)
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
    
    
def sendBallPose(bx):
    p = Pose(Point(bx,0.0,0.0), Quaternion(0.0,0.0,0.0,0.0))
    t = Twist(Vector3(0.0,0.0,0.0), Vector3(0.0,0.0,0.0))
    obj = LinkState('link',p,t,'world') 
    setLinkState(obj)

def sendObsPose(y1, y2, y3):
    global x_ob1, x_ob2, x_ob3
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

def reset():
    global current, previous
    sendObsPose(-3.0, 0.0, 3.0)
    sendBallPose(0.0)
    previous = current
    current = State(0.0, -3.0, 0.0, 3.0, 1.0, 1.0, 1.0)

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
    sendBallPose(bx)

def incrementObState():
    global bx, y1, y2, y3, v1, v2, v3
    if y1 <= -8.0:
        y1 = 8.0
    else :
        y1-=v1
    if y2 >= 8.0:
        y2 = -8.0
    else :
        y2+=v2 
    if y3 >= 8.0:
        y3 = -8.0
    else :
        y3+=v3 
    sendObsPose(y1,y2,y3)

def didCollide():
    global current, x_ob1, x_ob2, x_ob3, v1, v2, v3
    bx = current.x
    y1 = current.y1
    y2 = current.y2
    y3 = current.y3
    if (bx == x_ob1) and (y1 >= 1-v1) and (y1 <= 0):
        #print("Collision .... Ah ")
        return True
    elif (bx == x_ob2) and ( y2 >= 0) and (y2 <= v2-1):
        #print("Collision .... Ah ")
        return True
    elif (bx == x_ob3) and (y3 >= 0) and (y3 <= v3-1):
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

def done():
    return didReachGoal() or didCollide()
           


def pickAction():
    global eps, current, previous
    import random
    p = random.uniform(0, 1)
    if p < eps:
        return random.choice([True,False])
    else:
        return maxStateAction(current)[0]

def pickRandomAction():
    global eps, current, previous
    import random
    p = random.uniform(0, 1)
    return random.choice([True,False])

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

def act(action):
    global current, previous, bx, y1, y2, y3, v1, v2, v3
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
    current = State(bx,y1,y2,y3,v1,v2,v3)   
    return rew 

def qLearnUpdate(action, reward, alpha, gamma):
    global qStates, previous, current
    addQValue(current)
    addQValue(previous)
    qStates[previous][action] += alpha*(reward + gamma*(maxStateAction(current)[1]) - qStates[previous][action] )
    
def pickleQStates(num):
    global qStates
    output = open('normal_statevel_'+str(num)+'.pkl', 'wb')
    pickle.dump(qStates,output)
    output.close()

def stack_states(stacked_states, state_obj, is_new_episode):
    global bx, y1, y2, y3, stack_size, stacked_states
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

# Reset the graph
tflow.reset_default_graph()

# Instantiate the DQNetwork
DQN = DQNetwork(state_size, action_size, learning_rate)

# Instantiate memory
memory = Memory(max_size = memory_size)
for i in range(pretrain_length):
    # If it's the first step
    if i == 0:
        reset()
        state, stacked_states = stack_states(stacked_states, current, True)
        
    # Get the next_state, the rewards, done by taking a random action
    #choice = random.randint(1,len(possible_actions))-1
    #action = possible_actions[choice]
    #next_state, reward, done, _ = env.step(action)
    action = pickRandomAction()
    reward = act(action)
    done_ = done()
    #next_state = [current.x, current.y1, current.y2, current.y3]
    
    #env.render()
    
    # Stack the frames
    next_state, stacked_states = stack_states(stacked_states, current, False)
    
    
    # If the episode is finished ()
    if done_:
        # We finished the episode
        next_state = np.zeros(next_state.shape)
        
        # Add experience to memory
        memory.add((state, action, reward, next_state, done_))
        
        # Start a new episode
        reset()
        # Stack the frames
        state, stacked_states = stack_states(stacked_states, current, True)
        
    else:
        # Add experience to memory
        memory.add((state, action, reward, next_state, done_))
        
        # Our new state is now the next_state. Already set the current global. Following line is not needed anymore
        #state = next_state
        


# Setup TensorBoard Writer
writer = tflow.summary.FileWriter("/tensorboard/dqn/1")

## Losses
tflow.summary.scalar("Loss", DQN.loss)

write_op = tflow.summary.merge_all()




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

