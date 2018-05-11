#include <functional>
#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/common/common.hh>
//#include <ignition/math/Vector3.hh>
#include <gazebo/math/gzmath.hh>
#include <chrono>
#include <thread>
#include <unordered_map>
#include <vector>

using namespace std;

struct StateAction {
     public: double ball_x;
     public: double ob_a_y;
     public: double ob_b_y;
     public: double ob_c_y;
     public: bool action;

     };

     bool operator==(const StateAction& lhs, const StateAction& other) 
     { 
       return (lhs.ball_x == other.ball_x
             && lhs.ob_a_y == other.ob_a_y
             && lhs.ob_b_y == other.ob_b_y
             && lhs.ob_c_y == other.ob_c_y
             && lhs.action == other.action);
      }

    struct key_hash {
    
        size_t operator () (StateAction const &p) const noexcept{
        
            size_t res = 17;
            res = res * 31 + hash<double>()( p.ball_x );
            res = res * 31 + hash<double>()( p.ob_a_y );
            res = res * 31 + hash<double>()( p.ob_b_y );
            res = res * 31 + hash<double>()( p.ob_c_y );
            res = res * 31 + hash<bool>()(p.action);
            return res;
            
        } 
    };

namespace gazebo
{



  class QLearnAlgo : public ModelPlugin
  {
    


    private : double link_x;
    private : double lin_0_y;
    private : double lin_1_y;
    private : double lin_2_y;
    private : double alpha; // iteration coefficient
    private : double gamma; // discount factor  
    private : double eps;  
    private: unordered_map<StateAction,double,key_hash> qtable;
    private : double totalReward;
    private : double reward;
    private : StateAction current;
    private : StateAction prev;
   
    public: void setBallPose(){
        math::Pose temp = this->model->GetLink("link")->GetWorldPose();
        math::Vector3 tempVec = temp.pos;
        link_x = tempVec.x; 
    } 
    public: void setLink(int type){
        math::Pose temp = this->model->GetLink("link_"+to_string(type))->GetWorldPose();
        
        if(type == 0) lin_0_y = temp.pos.y;
        else if(type == 1) lin_1_y = temp.pos.y;
        else lin_2_y = temp.pos.y;
    } 
    public: void setVars(){
      //setBallPose();
      setLink(0);
      setLink(1);
      setLink(2);
    }
    public: void Load(physics::ModelPtr _parent, sdf::ElementPtr /*_sdf*/)
    {
      // Store the pointer to the model
      this->model = _parent;
      setVars(); 
      alpha = 0.01;
      gamma = 1;
      eps = 0.3;
      totalReward = 0;
      reward = 0;
   
      current.ball_x = link_x;
      current.ob_a_y = lin_0_y;
      current.ob_b_y = lin_1_y;
      current.ob_c_y = lin_2_y;

      
      // Listen to the update event. This event is broadcast every
      // simulation iteration.
      this->updateConnection = event::Events::ConnectWorldUpdateBegin(
          std::bind(&QLearnAlgo::OnUpdate, this));
    }

    public: bool pickAction(){
      	srand( time(NULL) ); //Randomize seed initialization
        int randNum = rand()%10;
        double p= (double)randNum/10.0;
        if(p < eps){
         // choose random action
         int temp = rand()%2;
         if(temp == 0) return true;
         else return false; 
        }
        else {
         // choose action that maximizes qtable 
            // build state to retrieve qvalue from qtable
            StateAction sa;
            sa.ball_x = link_x;
            sa.ob_a_y = lin_0_y;
            sa.ob_b_y = lin_1_y;
            sa.ob_c_y = lin_2_y;
            sa.action = true; 
            if(qtable.find(sa)==qtable.end()){
               qtable[sa] = 0;
            }
            
            StateAction sb;
            sb.ball_x = link_x;
            sb.ob_a_y = lin_0_y;
            sb.ob_b_y = lin_1_y;
            sb.ob_c_y = lin_2_y;
            sb.action = false; 
            if(qtable.find(sa)==qtable.end()){
               qtable[sb] = 0;
            }
           if(qtable[sa] >= qtable[sb]){
           return sa.action;
           }
           else
           return sb.action;
       } 
    }
    public: void setLinkPos(double x){
        this->model->GetLink("link")->SetWorldPose(math::Pose(math::Vector3(x,0.0,0.5),math::Quaternion(0.0,0.0,0.0)), true,true);
    }
    
    public: double getMaxQ(){
       bool action = current.action;
       StateAction sb ;
       sb.ball_x = current.ball_x;
       sb.ob_a_y = current.ob_a_y;
       sb.ob_b_y = current.ob_b_y;
       sb.ob_c_y = current.ob_c_y;
       sb.action = !action;
       if(qtable.find(sb) == qtable.end()){
         qtable[sb] = 0.0;
       }
       if(qtable[current] >= qtable[sb]){
          return qtable[current]; 
       }
       else return qtable[sb];
    }
    public: void updateQState(){
         
          if(qtable.find(current)==qtable.end()){
             qtable[current] = 0.0;
          }
          double maxQ = getMaxQ();
          double bracket = reward + gamma*(getMaxQ()) - qtable[current]; 
          qtable[current] = qtable[current] + alpha*(bracket);
     }
   
    public: void update(){
      setVars();
      bool action = pickAction();
      if(action){
            link_x++;
      }
      if(link_x>10) {link_x = 0.0;}
      setLinkPos(link_x);
      // Check if episode finished . Also check collision
      if(link_x == 0.0){
         //epsiode over
         totalReward = 0;
         reward = 0;
      }
      if(didCollide()){
       //high negative reward
          reward = -1000;
      }
      else{
        // Zero reward or positive reward
          if(link_x == 10){
         //+
             reward = 1000;
            }
         else{
          //0
             reward = 0;
          }
       }
      // Geting prev state
      prev = current;
       // Update the current state
      current.ball_x = link_x;
      current.ob_a_y = lin_0_y;
      current.ob_b_y = lin_1_y;
      current.ob_c_y = lin_2_y;
      current.action = action;

      totalReward += reward;
      updateQState();
      //std::cout << "Link 0 is at " << lin_0_y << "\n";
      //std::cout << "Link 1 is at " << lin_1_y << "\n";
      //std::cout << "Link 2 is at " << lin_2_y << "\n";
    }

    

    public : bool didCollide(){
       return (link_x == 9.0) && (lin_0_y == 0 || lin_1_y == 0 || lin_2_y == 0);
    }
    // Called by the world update start event
    public: void OnUpdate()
    {
      update();
      
      //std::this_thread::sleep_for(std::chrono::milliseconds(1000));
      // Apply a small linear velocity to the model.
      //this->model->SetLinearVel(ignition::math::Vector3d(1, 0, 0));
      //  this->model->GetLink("box_link")->AddForce(math::Vector3(10,0,12));
        
    }

    // Pointer to the model
    private: physics::ModelPtr model;

    // Pointer to the update event connection
    private: event::ConnectionPtr updateConnection;
  };

  // Register this plugin with the simulator
  GZ_REGISTER_MODEL_PLUGIN(QLearnAlgo)
}
