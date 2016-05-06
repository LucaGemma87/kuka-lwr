#ifndef LWR_CONTROLLERS__COMPUTED_TORQUE_CONTROLLER_H
#define LWR_CONTROLLERS__COMPUTED_TORQUE_CONTROLLER_H

#include "KinematicChainControllerBase.h"

#include <std_msgs/Float64MultiArray.h>

#include <boost/scoped_ptr.hpp>

#include <control_msgs/JointTrajectoryControllerState.h>

#include <geometry_msgs/WrenchStamped.h>

namespace lwr_controllers
{
	class VSComputedTorqueController: public controller_interface::KinematicChainControllerBase<hardware_interface::EffortJointInterface>
	{
	public:

		VSComputedTorqueController();
		~VSComputedTorqueController();

		bool init(hardware_interface::EffortJointInterface *robot, ros::NodeHandle &n);
		void starting(const ros::Time& time);
		void update(const ros::Time& time, const ros::Duration& period);
		void command(const std_msgs::Float64MultiArray::ConstPtr &msg);
		void set_gains(const std_msgs::Float64MultiArray::ConstPtr &msg);
                void readWrench(const geometry_msgs::WrenchStamped::ConstPtr &msg);
		control_msgs::JointTrajectoryControllerState state_now;
	private:

		ros::Subscriber sub_posture_;
		ros::Subscriber sub_gains_;
		ros::Subscriber sub_wrench_;
                
		ros::Publisher  pub_state_;
		
		KDL::JntArray cmd_states_;
		int cmd_flag_;	// discriminate if a user command arrived
		double lambda;	// flattening coefficient of tanh
		int step_;		// step used in tanh for reaching gradually the desired posture

		KDL::JntArray tau_cmd_,tau_ext_;
		KDL::JntSpaceInertiaMatrix M_; //Inertia matrix
		KDL::JntArray C_, G_;	//Coriolis and Gravitational matrices
		KDL::JntArray  Kp_, Kv_ , alpha_, gamma_, sigma_,Kp_t_, Kv_t_ , e_, e_dot_;	//Position and Velocity gains
               
                geometry_msgs::WrenchStamped wrench_in;
		KDL::ChainJntToJacSolver *m_JntToJacSolver;
		boost::scoped_ptr<KDL::ChainDynParam> id_solver_;

	};
}

#endif