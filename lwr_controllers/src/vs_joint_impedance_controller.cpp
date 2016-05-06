#include <angles/angles.h>
#include <pluginlib/class_list_macros.h>
#include <algorithm>
#include <kdl/tree.hpp>
#include <kdl/chainfksolvervel_recursive.hpp>
#include <kdl_parser/kdl_parser.hpp>
#include <urdf/model.h>

#include <lwr_controllers/vs_joint_impedance_controller.h>



namespace lwr_controllers {

VSJointImpedanceController::VSJointImpedanceController() {}

VSJointImpedanceController::~VSJointImpedanceController() {}

bool VSJointImpedanceController::init(hardware_interface::EffortJointInterface *robot, ros::NodeHandle &n)
{
  KinematicChainControllerBase<hardware_interface::EffortJointInterface>::init(robot, n);

  K_.resize(kdl_chain_.getNrOfJoints()); 
  D_.resize(kdl_chain_.getNrOfJoints());

  ROS_DEBUG(" Number of joints in handle = %lu", joint_handles_.size() );

  for (int i = 0; i < joint_handles_.size(); ++i){
    if ( !nh_.getParam("stiffness_gains", K_(i) ) ){
      ROS_WARN("Stiffness gain not set in yaml file, Using %f", K_(i));
      }
    }
  for (int i = 0; i < joint_handles_.size(); ++i){
    if ( !nh_.getParam("damping_gains", D_(i)) ){
      ROS_WARN("Damping gain not set in yaml file, Using %f", D_(i));
      }
    }



  id_solver_gravity_.reset( new KDL::ChainDynParam( kdl_chain_, gravity_) );

  dotq_msr_.resize(kdl_chain_.getNrOfJoints());
  q_msr_.resize(kdl_chain_.getNrOfJoints());
  q_des_.resize(kdl_chain_.getNrOfJoints());
  tau_des_.resize(kdl_chain_.getNrOfJoints());
  tau_cmd_.resize(kdl_chain_.getNrOfJoints());
  tau_gravity_.resize(kdl_chain_.getNrOfJoints());
  // K_.resize(kdl_chain_.getNrOfJoints());
  // D_.resize(kdl_chain_.getNrOfJoints());

  alpha_.resize(kdl_chain_.getNrOfJoints());
  gamma_.resize(kdl_chain_.getNrOfJoints());
  K_t_.resize(kdl_chain_.getNrOfJoints());
  D_t_.resize(kdl_chain_.getNrOfJoints());
  e_.resize(kdl_chain_.getNrOfJoints());
  
  
  state_now.desired.positions.resize(kdl_chain_.getNrOfJoints());
  state_now.desired.velocities.resize(kdl_chain_.getNrOfJoints());
  state_now.desired.accelerations.resize(kdl_chain_.getNrOfJoints());
  state_now.desired.effort.resize(kdl_chain_.getNrOfJoints());
    
  state_now.actual.positions.resize(kdl_chain_.getNrOfJoints());
  state_now.actual.velocities.resize(kdl_chain_.getNrOfJoints());
  state_now.actual.accelerations.resize(kdl_chain_.getNrOfJoints());
  state_now.actual.effort.resize(kdl_chain_.getNrOfJoints());
    
  state_now.error.positions.resize(kdl_chain_.getNrOfJoints());
  state_now.error.velocities.resize(kdl_chain_.getNrOfJoints());
  state_now.error.accelerations.resize(kdl_chain_.getNrOfJoints());
  state_now.error.effort.resize(kdl_chain_.getNrOfJoints());
  state_now.joint_names.resize(kdl_chain_.getNrOfJoints());

  sub_gains_ = nh_.subscribe("set_gains", 1, &VSJointImpedanceController::setGains, this);
  sub_posture_ = nh_.subscribe("command", 1, &VSJointImpedanceController::command, this);
  
  

  pub_state_ = nh_.advertise<control_msgs::JointTrajectoryControllerState>("state", 1);
  return true;

  
}

void VSJointImpedanceController::starting(const ros::Time& time)
{
  // get joint positions
  for(size_t i=0; i<joint_handles_.size(); i++) {
     K_(i) = 2500.0;
     D_(i) = 0.09;
     K_t_(i) = 2500.0;
     D_t_(i) = 0.09;
     alpha_(i)=100.0;
     gamma_(i)=2.0;
     e_(i)=0.0;

    // K_(i) = 1000.0;
    // D_(i) = 70.0;
    // K_t_(i) = 1000.0;
    // D_t_(i) = 70.0;
    // alpha_(i)=10;
    // gamma_(i)=2;
    // e_(i)=0.0;
    
     
     

    
    tau_des_(i) = 0.0;
    dotq_msr_.q(i) = joint_handles_[i].getPosition();
    q_des_(i) = dotq_msr_.q(i);
    dotq_msr_.qdot(i) = joint_handles_[i].getVelocity();
    }


}

void VSJointImpedanceController::update(const ros::Time& time, const ros::Duration& period)
{
  
  // get joint positions	
  for(size_t i=0; i<joint_handles_.size(); i++) {
    dotq_msr_.q(i) = joint_handles_[i].getPosition();
    q_msr_(i) = dotq_msr_.q(i);
    dotq_msr_.qdot(i) = joint_handles_[i].getVelocity();
    }

  //Compute control law
  id_solver_gravity_->JntToGravity( q_msr_ , tau_gravity_ );
  for(size_t i=0; i<joint_handles_.size(); i++) {
   
    e_(i)=(q_des_(i) - q_msr_(i));

    K_t_(i) = K_(i) * gamma_(i) / (1 + exp( -alpha_(i) * fabs(e_(i)) ) );

    D_t_(i) = D_(i) * gamma_(i) / (1 + exp( -alpha_(i) * fabs(e_(i)) ) );

    tau_cmd_(i) = K_t_(i) * e_(i) + D_t_(i)*dotq_msr_.qdot(i) + tau_des_(i) + tau_gravity_(i);
    
    joint_handles_[i].setCommand(tau_cmd_(i));
    
    state_now.desired.positions[i]=q_des_(i);
    state_now.desired.velocities[i]=0.0;
    state_now.desired.accelerations[i]=0.0;
    state_now.desired.effort[i]=tau_des_(i);
    
    state_now.actual.positions[i]=q_msr_(i);
    state_now.actual.velocities[i]=dotq_msr_.q(i);
    state_now.actual.accelerations[i]= dotq_msr_.qdot(i);
    state_now.actual.effort[i]=tau_cmd_(i);
    
    state_now.error.positions[i]=e_(i);
    state_now.error.velocities[i]=0.0;
    state_now.error.accelerations[i]=0.0;
    state_now.error.effort[i]=0.0;
    state_now.joint_names.resize(joint_handles_.size());
    state_now.joint_names[i]=joint_handles_[i].getName();
    
  }	
//   ROS_INFO("e_t: %g, %g, %g, %g, %g, %g, %g",
//   e_(0), e_(1), e_(2), e_(3), e_(4), e_(5), e_(6));
//   ROS_INFO("New gains K_t_: %g, %g, %g, %g, %g, %g, %g",
//   K_t_(0), K_t_(1), K_t_(2), K_t_(3), K_t_(4), K_t_(5), K_t_(6));
//   ROS_INFO("New gains D_t_: %g, %g, %g, %g, %g, %g, %g",
//   D_t_(0), D_t_(1), D_t_(2), D_t_(3), D_t_(4), D_t_(5), D_t_(6));
  state_now.header.stamp=time;
  state_now.header.frame_id=joint_handles_[0].getName();
  pub_state_.publish<control_msgs::JointTrajectoryControllerState>(state_now);
}

void VSJointImpedanceController::command(const std_msgs::Float64MultiArray::ConstPtr &msg){
  if (msg->data.size() == 0) {
    ROS_INFO("Desired configuration must be: %lu dimension", joint_handles_.size());
    }
  else if ((int)msg->data.size() != joint_handles_.size()) {
    ROS_ERROR("Posture message had the wrong size: %d", (int)msg->data.size());
    return;
    }
  else
  {
    for (unsigned int j = 0; j < joint_handles_.size(); ++j)
    q_des_(j) = msg->data[j];
  }

  }

void VSJointImpedanceController::setGains(const std_msgs::Float64MultiArray::ConstPtr &msg){
  ROS_INFO("Changing controller parameters");
  if (msg->data.size() == 4*joint_handles_.size()){
    for (unsigned int i = 0; i < joint_handles_.size(); ++i){
      K_(i) = msg->data[i];
      D_(i)= msg->data[i + joint_handles_.size()];
      alpha_(i)= msg->data[i + 2 * joint_handles_.size()];
      gamma_(i)= msg->data[i + 3 * joint_handles_.size()];
      }
    // for (unsigned int i = joint_handles_.size(); i < 2*joint_handles_.size(); ++i){
      // 	D_(i)= msg->data[i];
      // }
    }
  else
  {
    ROS_INFO("Num of Joint handles = %lu", joint_handles_.size());
  }

  ROS_INFO("Num of Joint handles = %lu, dimension of message = %lu", joint_handles_.size(), msg->data.size());

  ROS_INFO("New gains K: %.1lf, %.1lf, %.1lf %.1lf, %.1lf, %.1lf, %.1lf",
  K_t_(0), K_t_(1), K_t_(2), K_t_(3), K_t_(4), K_t_(5), K_t_(6));
  ROS_INFO("New gains D: %.1lf, %.1lf, %.1lf %.1lf, %.1lf, %.1lf, %.1lf",
  D_t_(0), D_t_(1), D_t_(2), D_t_(3), D_t_(4), D_t_(5), D_t_(6));
  ROS_INFO("New alpha: %.1lf, %.1lf, %.1lf %.1lf, %.1lf, %.1lf, %.1lf",
  alpha_(0), alpha_(1), alpha_(2), alpha_(3), alpha_(4), alpha_(5), alpha_(6));
  ROS_INFO("New gamma: %.1lf, %.1lf, %.1lf %.1lf, %.1lf, %.1lf, %.1lf",
  gamma_(0), gamma_(1), gamma_(2), gamma_(3), gamma_(4), gamma_(5), gamma_(6));
  ROS_INFO("Controller parameters changed");
  }


}                                                           // namespace

PLUGINLIB_EXPORT_CLASS( lwr_controllers::VSJointImpedanceController, controller_interface::ControllerBase)
