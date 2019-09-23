#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <geometry_msgs/Vector3.h>
#include <fzi_manipulation_msgs/TrajectoryDesignerSettings.h>
#include <std_srvs/Trigger.h>
#include <std_msgs/Float32.h>

class ParametersManager {
 private:
  ros::NodeHandle nh_;
  ros::NodeHandle nh_private_;
  ros::Subscriber update_robot_speed_sub_;
  ros::ServiceClient update_robot_speed_client_;
  double vel_min_;
  double vel_max_;

 public:
  ParametersManager():
      nh_(),
      nh_private_("~")
  {
      update_robot_speed_client_ = nh_.serviceClient<fzi_manipulation_msgs::TrajectoryDesignerSettings>("pathloader/updateSettings");
      update_robot_speed_sub_ = nh_.subscribe("/update_robot_speed", 10, &ParametersManager::updateRobotSpeed, this);
      ros::param::param<double>("~vel_min", vel_min_, 0.1);
      ros::param::param<double>("~vel_max", vel_max_, 0.6);
  }

  void updateRobotSpeed(const std_msgs::Float32::ConstPtr& msg)
  {
    fzi_manipulation_msgs::TrajectoryDesignerSettings settings;
    double vel = vel_min_ + (vel_max_ * msg->data);
    settings.request.velocity = vel;
    settings.request.acceleration = vel * 4;
    update_robot_speed_client_.call(settings);
  }
};

int main(int argc, char** argv) {
  ros::init(argc, argv, "parameters_manager_node");
  ParametersManager pm;

  ros::Rate r(10);

  while (ros::ok())
  {
    ros::spinOnce();

    r.sleep();
  }

  return 0;
}