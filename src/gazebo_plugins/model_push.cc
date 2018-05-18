#include <functional>
#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/common/common.hh>
//#include <ignition/math/Vector3.hh>
#include <gazebo/math/gzmath.hh>
#include <chrono>
#include <thread>

namespace gazebo
{
  class ModelPush : public ModelPlugin
  {
    private : double link_x;
    private : double lin_0_y;
    private : double lin_1_y;
    private : double lin_2_y;
    private : double lin_f_x;

    public: void reset(){

        setLinkPos(0.0);
	resetObstacles();
    }
    public: void resetObstacles(){
        	
        setLinkWithNum(0,-8.0);
        setLinkWithNum(1,0.0);
        setLinkWithNum(2,8.0);

    }
    public: void setLinkPos(double x){
        this->model->GetLink("link")->SetWorldPose(math::Pose(math::Vector3(x,0.0,0.5),math::Quaternion(0.0,0.0,0.0)), true,true);
    }
    
    public : void setLinkWithNum(int type, double y){
     this->model->GetLink("link_"+std::to_string(type))->SetWorldPose(math::Pose(math::Vector3(lin_f_x,y,0.5),math::Quaternion(0.0,0.0,0.0)), true,true);
    }
    public: void Load(physics::ModelPtr _parent, sdf::ElementPtr /*_sdf*/)
    {
      // Store the pointer to the model
      this->model = _parent;
      link_x = 0;
      lin_0_y = -8.0;
      lin_1_y = 0.0;
      lin_2_y = 5.0;
      lin_f_x = 9.0;
      reset();
      //cout << (this->model->GetLink("link")->GetWorldPose());
      // Listen to the update event. This event is broadcast every
      // simulation iteration.
      this->updateConnection = event::Events::ConnectWorldUpdateBegin(
          std::bind(&ModelPush::OnUpdate, this));
    }

    public: void update(){
      //setLinkPos(link_x);
      setLinkWithNum(0,lin_0_y);
      setLinkWithNum(1,lin_1_y);
      setLinkWithNum(2,lin_2_y);
      //std::cout << "Link 0 is at " << lin_0_y << "\n";
      //std::cout << "Link 1 is at " << lin_1_y << "\n";
      //std::cout << "Link 2 is at " << lin_2_y << "\n";
    }

    public : bool didCollide(){
       return (link_x == lin_f_x) && (lin_0_y == 0 || lin_1_y == 0 || lin_2_y == 0);
    }
    // Called by the world update start event
    public: void OnUpdate()
    {
      //link_x++;
      //if(link_x>10) {link_x = 0.0;}
      lin_0_y++;
      if(lin_0_y > 8) { lin_0_y = -8.0;}
      lin_1_y+=1;
      if(lin_1_y > 8) { lin_1_y = -8.0;}
      lin_2_y++;
      if(lin_2_y > 8) { lin_2_y = -8.0;}
      update();
     // std::this_thread::sleep_for(std::chrono::milliseconds(800));
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
  GZ_REGISTER_MODEL_PLUGIN(ModelPush)
}
