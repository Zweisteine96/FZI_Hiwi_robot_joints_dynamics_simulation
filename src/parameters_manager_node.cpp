#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <geometry_msgs/Vector3.h>
#include <fzi_manipulation_msgs/TrajectoryDesignerSettings.h>
#include <std_srvs/Trigger.h>
#include <std_msgs/Float32.h>

class ParametersManager {
 private:
  tf::TransformBroadcaster br;
  std::string target_tf;
  ros::NodeHandle nh_;
  ros::NodeHandle nh_private_;
  ros::Subscriber update_robot_speed_sub_;
  ros::ServiceClient update_robot_speed_client_;
  std::string base_name;
  std::string target_name;
  tf::Vector3 random_point;

 public:
  ParametersManager():
      nh_(),
      nh_private_("~")
  {
      update_robot_speed_client_ = nh_.serviceClient<fzi_manipulation_msgs::TrajectoryDesignerSettings>("pathloader/updateSettings");
      update_robot_speed_sub_ = nh_.subscribe("update_robot_speed", 10, &ParametersManager::updateRobotSpeed, this);
//     ros::param::param<std::string>("~target_tf", target_name, "random_people");
//     ros::param::param<std::string>("~base_tf", base_name, "base_link");
  }

  void updateRobotSpeed(const std_msgs::Float32::ConstPtr& msg)
  {
    fzi_manipulation_msgs::TrajectoryDesignerSettings settings;
    settings.request.velocity = msg->data;
    settings.request.acceleration = msg->data * 4;
    update_robot_speed_client_.call(settings);
//     tf::Quaternion q(0,0,0,1);
//     tf::StampedTransform random_people(tf::Transform(), ros::Time::now(), base_name, target_name);
//     random_people.setOrigin(random_point);
//     random_people.setRotation(q);
//     br.sendTransform(random_people);
  }

//   void publish()
//   {
// //     tf::Quaternion q(0,0,0,1);
// //     tf::StampedTransform random_people(tf::Transform(), ros::Time::now(), base_name, target_name);
// //     random_people.setOrigin(random_point);
// //     random_people.setRotation(q);
// //     br.sendTransform(random_people);
//   }
};

int main(int argc, char** argv) {
  ros::init(argc, argv, "parameters_manager_node");
  ParametersManager pm;

  ros::Rate r(10);

  while (ros::ok())
  {
//     pm.publish();

    ros::spinOnce();

    r.sleep();
  }

  return 0;
}