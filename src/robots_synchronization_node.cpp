#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <std_srvs/SetBool.h>

class RobotsSynchronization {
 private:
  ros::NodeHandle nh_;
  ros::NodeHandle nh_private_;
  ros::Publisher robot_target_pub_;
  ros::Subscriber part_info_sub_;
  ros::Subscriber update_controller_sub_;
  ros::ServiceClient enable_grasping_client_;
  ros::ServiceClient dynamic_reconfigure_client_;
  ros::ServiceServer robot_synchronization_service_;

  bool busy_;


 public:
  RobotsSynchronization():
      nh_(),
      nh_private_("~")
  {
      // ros::param::param<std::string>("~base_frame", base_frame_, "base_link");
      
      robot_synchronization_service_ = nh_.advertiseService("/robot_synchronization",  &RobotsSynchronization::synchronizeRobots, this);

      busy_ = false;
  }

  bool synchronizeRobots(std_srvs::SetBool::Request  &req, std_srvs::SetBool::Response &res)
  {
    if(req.data)
    {
      if(!busy_)
      {
        res.success = true;
        busy_ = true;
      }
      else
      {
        res.success = false;
      }
    }
    else
    {
       busy_ = false;
       res.success = true;
    }

    return true;
  }

};

int main(int argc, char** argv) {
  ros::init(argc, argv, "robot_synchronization_node");

  RobotsSynchronization rs;
  
  ros::spin();

  return 0;
}
