
#ifndef LWR_CONTROLLERS__JOINT_INPEDANCE_CONTROLLER_H
#define LWR_CONTROLLERS__JOINT_INPEDANCE_CONTROLLER_H

#include "KinematicChainControllerBase.h"

#include <visualization_msgs/Marker.h>
#include <std_msgs/Float64MultiArray.h>

#include <boost/scoped_ptr.hpp>


#include <control_msgs/JointTrajectoryControllerState.h>
/*
	tau_cmd_ = K_*(q_des_ - q_msr_) + D_*dotq_msr_ + G(q_msr_)

*/

namespace lwr_controllers
{

	class VSJointImpedanceController: public controller_interface::KinematicChainControllerBase<hardware_interface::EffortJointInterface>
	{
	public:

		VSJointImpedanceController();
		~VSJointImpedanceController();

		bool init(hardware_interface::EffortJointInterface *robot, ros::NodeHandle &n);

		void starting(const ros::Time& time);

		void update(const ros::Time& time, const ros::Duration& period);
		void command(const std_msgs::Float64MultiArray::ConstPtr &msg);
		void setGains(const std_msgs::Float64MultiArray::ConstPtr &msg);
		
		 control_msgs::JointTrajectoryControllerState state_now;
                
	private:

		ros::Subscriber sub_gains_;
		ros::Subscriber sub_posture_;
                
		ros::Publisher  pub_state_;
		
		KDL::JntArrayVel dotq_msr_;
		KDL::JntArray q_msr_, q_des_;
		KDL::JntArray tau_des_, tau_cmd_, tau_gravity_;
		KDL::JntArray K_, D_ , alpha_, gamma_, K_t_, D_t_ , e_;

		boost::scoped_ptr<KDL::ChainDynParam> id_solver_gravity_;

	};

} // namespace

#endif
