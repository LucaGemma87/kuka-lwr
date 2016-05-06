#include <pluginlib/class_list_macros.h>
#include <kdl_parser/kdl_parser.hpp>
#include <math.h>

#include <lwr_controllers/vs_computed_torque_controller.h>

namespace lwr_controllers 
{
	VSComputedTorqueController::VSComputedTorqueController() 
    {
       //sub_wrench_ = nh_.subscribe("/lwr/ft_compensated", 5, &VSComputedTorqueController::readWrench, this);      
       sub_wrench_ = nh_.subscribe("/lwr/ft_sensor_topic", 5, &VSComputedTorqueController::readWrench, this);      
    }
	VSComputedTorqueController::~VSComputedTorqueController() {
	 delete m_JntToJacSolver; 
	}

	bool VSComputedTorqueController::init(hardware_interface::EffortJointInterface *robot, ros::NodeHandle &n)
	{
        KinematicChainControllerBase<hardware_interface::EffortJointInterface>::init(robot, n);
        
		id_solver_.reset( new KDL::ChainDynParam( kdl_chain_, gravity_) );

		cmd_states_.resize(kdl_chain_.getNrOfJoints());
		tau_cmd_.resize(kdl_chain_.getNrOfJoints());
		tau_ext_.resize(kdl_chain_.getNrOfJoints());
		Kp_.resize(kdl_chain_.getNrOfJoints());
		Kv_.resize(kdl_chain_.getNrOfJoints());
        
        alpha_.resize(kdl_chain_.getNrOfJoints());
        gamma_.resize(kdl_chain_.getNrOfJoints());
	sigma_.resize(kdl_chain_.getNrOfJoints());
        Kp_t_.resize(kdl_chain_.getNrOfJoints());
        Kv_t_.resize(kdl_chain_.getNrOfJoints());

        e_.resize(kdl_chain_.getNrOfJoints());
        e_dot_.resize(kdl_chain_.getNrOfJoints());

		
        M_.resize(kdl_chain_.getNrOfJoints());
		C_.resize(kdl_chain_.getNrOfJoints());
		G_.resize(kdl_chain_.getNrOfJoints());
		
		m_JntToJacSolver = new KDL::ChainJntToJacSolver(kdl_chain_);
		
		
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

		sub_posture_ = nh_.subscribe("command", 1, &VSComputedTorqueController::command, this);
		sub_gains_ = nh_.subscribe("set_gains", 1, &VSComputedTorqueController::set_gains, this);
		
		
		
		pub_state_ = nh_.advertise<control_msgs::JointTrajectoryControllerState>("state", 1);

		return true;		
	}

	void VSComputedTorqueController::starting(const ros::Time& time)
	{
  		// get joint positions
  		for(size_t i=0; i<joint_handles_.size(); i++) 
  		{

  			Kp_(i) = 2500.0;
  			Kv_(i) = 20.7;
            Kp_t_(i) = 2500.0;
            Kv_t_(i) = 20.7;
            alpha_(i)=100;
	      gamma_(i)=2;
	      sigma_(i)=0.005;
	      tau_ext_(i)=0.0;
            //e_(i)=0.0;
            //e_dot_(i)=0.0;
    		joint_msr_states_.q(i) = joint_handles_[i].getPosition();
    		joint_msr_states_.qdot(i) = joint_handles_[i].getVelocity();
    		joint_msr_states_.qdotdot(i) = 0.0;
    		joint_des_states_.q(i) = joint_msr_states_.q(i);
    	}

    	lambda = 0.1;	// lower values: flatter
    	cmd_flag_ = 0;	
    	step_ = 0;

    	ROS_INFO(" Number of joints in handle = %lu", joint_handles_.size() );

    }
    
    void VSComputedTorqueController::readWrench(const geometry_msgs::WrenchStamped::ConstPtr &msg)
   {
	//	ROS_DEBUG("In ft sensorcallback");
		wrench_in = *msg;
	//	m_received_ft = true;
     
  }


    void VSComputedTorqueController::update(const ros::Time& time, const ros::Duration& period)
    {    
       
    	// get joint positions
  		for(size_t i=0; i<joint_handles_.size(); i++) 
  		{
    		joint_msr_states_.q(i) = joint_handles_[i].getPosition();
    		joint_msr_states_.qdot(i) = joint_handles_[i].getVelocity();
    		joint_msr_states_.qdotdot(i) = 0.0;
    	}

    	
    	if(cmd_flag_)
    	{
    		// reaching desired joint position using a hyperbolic tangent function
    		for(size_t i=0; i<joint_handles_.size(); i++)
    		{
    			joint_des_states_.q(i) = cmd_states_(i)*1/2*(1+ tanh(lambda*step_-M_PI)); 
    			joint_des_states_.qdot(i) = cmd_states_(i)*1/2*lambda*(1/(cosh(M_PI-lambda*step_)*cosh(M_PI-lambda*step_))); // 1/(cosh^2) = sech^2
    			joint_des_states_.qdotdot(i) = cmd_states_(i)*lambda*lambda*(1/(cosh(M_PI-step_)*cosh(M_PI-step_)))*tanh(M_PI-step_);
    		}
    		++step_;

    		if(joint_des_states_.q == cmd_states_)
    		{
    			cmd_flag_ = 0;	//reset command flag
    			step_ = 0;
    			ROS_INFO("Posture OK");
    		}
    	}

    	// computing Inertia, Coriolis and Gravity matrices
    	id_solver_->JntToMass(joint_msr_states_.q, M_);
    	id_solver_->JntToCoriolis(joint_msr_states_.q, joint_msr_states_.qdot, C_);
    	id_solver_->JntToGravity(joint_msr_states_.q, G_);
	
	
	
	KDL::Jacobian J(kdl_chain_.getNrOfJoints());
	m_JntToJacSolver->JntToJac(joint_msr_states_.q, J);
	Eigen::MatrixXd JT = J.data.transpose();
	Eigen::MatrixXd wrench_ext;
	wrench_ext.resize(6, 1);
	wrench_ext(0) = wrench_in.wrench.force.x;
        wrench_ext(1) = wrench_in.wrench.force.y;
	wrench_ext(2) = wrench_in.wrench.force.z;
	wrench_ext(3) = wrench_in.wrench.torque.x;
	wrench_ext(4) = wrench_in.wrench.torque.y;
	wrench_ext(5) = wrench_in.wrench.torque.z;
	//ROS_INFO("force x %g",wrench_ext(0));
	//tau_ext_ = J*wrench_ext;
	Eigen::MatrixXd tau_ext;
	tau_ext.resize(kdl_chain_.getNrOfJoints(), 1);
	
	
 	tau_ext = JT*wrench_ext; //  TO DO sistemare questo !!!
 
		for(size_t i=0; i<joint_handles_.size(); i++) 
		{
			// control law
            tau_ext_(i)= tau_ext(i);
            e_dot_(i) = (joint_des_states_.qdot(i) - joint_msr_states_.qdot(i));
            
            e_(i) = (joint_des_states_.q(i) - joint_msr_states_.q(i)); 

            Kp_t_(i) = Kp_(i) * gamma_(i) / (1 + exp( -alpha_(i) * fabs(e_(i)) ) );

            Kv_t_(i) = Kv_(i) * gamma_(i) / (1 + exp( -alpha_(i) * fabs(e_(i)) ) );

            // chiedere a Manuel perchè si moltiplicano solo gli elementi diagonali di M_
	    
	     // generalized Bell function  f ( x ) = 1 / ( 1 + | ( x - c) / a | ^ 2b )
			tau_cmd_(i) = M_(i,i)*(joint_des_states_.qdotdot(i) + Kv_t_(i)*e_dot_(i) + Kp_t_(i)*e_(i)) + C_(i)*joint_msr_states_.qdot(i) + G_(i) + 1 / ( 1 +  pow(fabs(e_(i)/ sigma_(i)), 2*2))*tau_ext_(i);
			//ROS_INFO("Tau ext gamma  %d: %g",(int)i, 1 / ( 1 +  pow(fabs(e_(i)/ sigma_(i)), 2*2))*tau_ext_(i));
		   	joint_handles_[i].setCommand(tau_cmd_(i));
    state_now.desired.positions[i]=joint_des_states_.q(i);
    state_now.desired.velocities[i]=joint_des_states_.qdot(i);
    state_now.desired.accelerations[i]=joint_des_states_.qdotdot(i);
    state_now.desired.effort[i]=0.0;
    
    state_now.actual.positions[i]=joint_msr_states_.q(i);
    state_now.actual.velocities[i]=joint_msr_states_.qdot(i);
    state_now.actual.accelerations[i]= 0.0;
    state_now.actual.effort[i]=tau_cmd_(i);
    
    state_now.error.positions[i]=e_(i);
    state_now.error.velocities[i]=e_dot_(i);
    state_now.error.accelerations[i]=0.0;
    state_now.error.effort[i]=0.0;
    state_now.joint_names.resize(joint_handles_.size());
    state_now.joint_names[i]=joint_handles_[i].getName();
		  
		}
//         ROS_INFO("e_t: %g, %g, %g, %g, %g, %g, %g",
//         e_(0), e_(1), e_(2), e_(3), e_(4), e_(5), e_(6));
// 	
//          ROS_INFO("New gains Kp_t_: %g, %g, %g, %g, %g, %g, %g", Kp_t_(0), Kp_t_(1), Kp_t_(2), Kp_t_(3), Kp_t_(4), Kp_t_(5), Kp_t_(6));
//         ROS_INFO("New gains Kv_t_: %g, %g, %g, %g, %g, %g, %g", Kv_t_(0), Kv_t_(1), Kv_t_(2), Kv_t_(3), Kv_t_(4), Kv_t_(5), Kv_t_(6));

    state_now.header.stamp=time;
    state_now.header.frame_id=joint_handles_[0].getName();
    pub_state_.publish<control_msgs::JointTrajectoryControllerState>(state_now);
    }

    void VSComputedTorqueController::command(const std_msgs::Float64MultiArray::ConstPtr &msg)
    {
    	if(msg->data.size() == 0)
    		ROS_INFO("Desired configuration must be of dimension %lu", joint_handles_.size());
    	else if(msg->data.size() != joint_handles_.size())
    	{
    		ROS_ERROR("Posture message had the wrong size: %u", (unsigned int)msg->data.size());
    		return;
    	}
    	else
    	{
    		for (unsigned int i = 0; i<joint_handles_.size(); i++)
    			cmd_states_(i) = msg->data[i];

    		cmd_flag_ = 1;
    	}

	}

	void VSComputedTorqueController::set_gains(const std_msgs::Float64MultiArray::ConstPtr &msg)
	{
		if(msg->data.size() == 4*joint_handles_.size())
		{
			for(unsigned int i = 0; i < joint_handles_.size(); i++)
			{
				Kp_(i) = msg->data[i];
				Kv_(i) = msg->data[i + joint_handles_.size()];
                alpha_(i) = msg->data[i + 2*joint_handles_.size()];
                gamma_(i) = msg->data[i + 3*joint_handles_.size()];
			}
		}
		else
			ROS_INFO("Number of Joint handles = %lu", joint_handles_.size());

		ROS_INFO("Num of Joint handles = %lu, dimension of message = %lu", joint_handles_.size(), msg->data.size());

		ROS_INFO("New gains Kp: %.1lf, %.1lf, %.1lf %.1lf, %.1lf, %.1lf, %.1lf", Kp_(0), Kp_(1), Kp_(2), Kp_(3), Kp_(4), Kp_(5), Kp_(6));
		ROS_INFO("New gains Kv: %.1lf, %.1lf, %.1lf %.1lf, %.1lf, %.1lf, %.1lf", Kv_(0), Kv_(1), Kv_(2), Kv_(3), Kv_(4), Kv_(5), Kv_(6));
        ROS_INFO("New alpha: %.1lf, %.1lf, %.1lf %.1lf, %.1lf, %.1lf, %.1lf", alpha_(0), alpha_(1), alpha_(2), alpha_(3), alpha_(4), alpha_(5), alpha_(6));
        ROS_INFO("New gamma: %.1lf, %.1lf, %.1lf %.1lf, %.1lf, %.1lf, %.1lf", gamma_(0), gamma_(1), gamma_(2), gamma_(3), gamma_(4), gamma_(5), gamma_(6));

	}
}

PLUGINLIB_EXPORT_CLASS(lwr_controllers::VSComputedTorqueController, controller_interface::ControllerBase)
