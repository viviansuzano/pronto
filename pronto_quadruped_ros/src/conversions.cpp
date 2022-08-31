#include "pronto_quadruped_ros/conversions.hpp"
#include <ros/console.h>

namespace pronto {
namespace quadruped {

// fix the order of the joints from ROS to be compatible with Pronto
Eigen::VectorXd fromRosToModel(const sensor_msgs::JointState &msg, const int var) {

	int num_joints = msg.name.size();
	Eigen::VectorXd joints = Eigen::VectorXd::Zero(num_joints);
	int idx = 0;

	switch(var) {
	  case 0:
		idx = 0;
		for (int i = 0; i < 4; i++) {
			idx = 3*i;
			joints[idx] = msg.position.at(idx+1);
			joints[idx+1] = msg.position.at(idx+2);
			joints[idx+2] = msg.position.at(idx);
		}
	    break;
	  case 1:
		idx = 0;
		for (int i = 0; i < 4; i++) {
			idx = 3*i;
			joints[idx] = msg.velocity.at(idx+1);
			joints[idx+1] = msg.velocity.at(idx+2);
			joints[idx+2] = msg.velocity.at(idx);
		}
	    break;
	  case 2:
		idx = 0;
		for (int i = 0; i < 4; i++) {
			idx = 3*i;
			joints[idx] = msg.effort.at(idx+1);
			joints[idx+1] = msg.effort.at(idx+2);
			joints[idx+2] = msg.effort.at(idx);
		}
	    break;
	}

	return joints;
}

bool jointStateFromROS(const sensor_msgs::JointState& msg,
                       uint64_t& utime,
                       JointState& q,
                       JointState& qd,
                       JointState& qdd,
                       JointState& tau)
{
    // if the size of the joint state message does not match our own,
    // we silently return an invalid update
    if (static_cast<int>(static_cast<const sensor_msgs::JointState&>(msg).position.size()) != q.size() ||
        static_cast<int>(static_cast<const sensor_msgs::JointState&>(msg).velocity.size()) != q.size() ||
        static_cast<int>(static_cast<const sensor_msgs::JointState&>(msg).effort.size()) != q.size()){
        ROS_WARN_STREAM_THROTTLE(1, "Joint State is expected " << \
                                 q.size() << " joints but "\
                                 << msg.position.size() << " / " << msg.velocity.size() << " / " << msg.effort.size()
                                 << " are provided.");
        return false;
    }
    // store message time in microseconds
    utime = msg.header.stamp.toNSec() / 1000;
//    for(int i=0; i<12; i++){
//      q(i) = msg.position[i];
//      qd(i) = msg.velocity[i];
//      tau(i) = msg.effort[i];
//    }
    //q = Eigen::Map<const JointState>(msg.position.data());
    //qd = Eigen::Map<const JointState>(msg.velocity.data());
    //tau = Eigen::Map<const JointState>(msg.effort.data());

    q = fromRosToModel(msg, 0);
    qd = fromRosToModel(msg, 1);
    tau = fromRosToModel(msg, 2);

    qdd.setZero(); // TODO compute the acceleration

    return true;
}

bool jointStateWithAccelerationFromROS(const pronto_msgs::JointStateWithAcceleration& msg,
                               uint64_t& utime,
                               JointState& q,
                               JointState& qd,
                               JointState& qdd,
                               JointState& tau)
{
    // if the size of the joint state message does not match our own,
    // we silently return an invalid update
    if (static_cast<int>(static_cast<const pronto_msgs::JointStateWithAcceleration&>(msg).position.size()) != q.size() ||
        static_cast<int>(static_cast<const pronto_msgs::JointStateWithAcceleration&>(msg).velocity.size()) != q.size() ||
        static_cast<int>(static_cast<const pronto_msgs::JointStateWithAcceleration&>(msg).acceleration.size()) != q.size() ||
        static_cast<int>(static_cast<const pronto_msgs::JointStateWithAcceleration&>(msg).effort.size()) != q.size()){
        ROS_WARN_STREAM_THROTTLE(1, "Joint State is expected " << \
                                 q.size() << " joints but "\
                                 << msg.position.size() << " / " << msg.velocity.size() << " / " << msg.acceleration.size() << " / " << msg.effort.size()
                                 << " are provided.");
        return false;
    }
    // store message time in microseconds
    utime = 1e-3 * msg.header.stamp.toNSec();
    for(int i=0; i<12; i++){
      q(i) = msg.position[i];
      qd(i) = msg.velocity[i];
      qdd(i) = msg.acceleration[i];
      tau(i) = msg.effort[i];
    }

    return true;
}

}  // namespace quadruped
}  // namespace pronto
