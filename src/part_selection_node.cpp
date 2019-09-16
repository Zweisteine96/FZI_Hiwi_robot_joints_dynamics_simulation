#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <geometry_msgs/Vector3.h>
#include <geometry_msgs/PoseStamped.h>
#include <std_srvs/Trigger.h>

class PartSelection {
 private:
  std::string target_tf;
  ros::NodeHandle nh_;
  ros::NodeHandle nh_private_;
  ros::Subscriber robot_target_sub_;
  ros::ServiceClient enable_placing_client_;
  ros::ServiceServer part_selection_service_;
  tf::TransformListener listener_;
  geometry_msgs::PoseStamped robot_target_;
  bool part_selected_;
  std::string base_name;
  std::string target_name;
  tf::Vector3 random_point;

 public:
  PartSelection():
      nh_(),
      nh_private_("~")
  {
      part_selected_ = true;

      robot_target_sub_ = nh_.subscribe("target_frame", 10, &PartSelection::getRobotTarget, this);
      enable_placing_client_ = nh_.serviceClient<std_srvs::Trigger>("enable_placing");
      part_selection_service_ = nh_.advertiseService("part_selection", &PartSelection::partSelection, this);
//     ros::param::param<std::string>("~target_tf", target_name, "random_people");
//     ros::param::param<std::string>("~base_tf", base_name, "base_link");
  }
  
  void getRobotTarget(const geometry_msgs::PoseStamped::ConstPtr& pose_msg)
  {
    robot_target_.pose.position.x = pose_msg->pose.position.x;
    robot_target_.pose.position.y = pose_msg->pose.position.y;
    robot_target_.pose.position.z = pose_msg->pose.position.z;
  }

  bool partSelection(std_srvs::TriggerRequest& req, std_srvs::TriggerResponse& res)
  {
    part_selected_ = false;
    return true;
  }


  void publishTarget()
  {
    tf::StampedTransform transform;
    try
    {
      listener_.lookupTransform("/ur10_base_link", "/ur10_tool0", ros::Time(0), transform);
    }
    catch (tf::TransformException ex)
    {
      ROS_ERROR("%s",ex.what());
      ros::Duration(1.0).sleep();
    }
    
    double x_tool, y_tool, z_tool, x_target, y_target, z_target, dist;
    x_tool = transform.getOrigin().x();
    y_tool = transform.getOrigin().y();
    z_tool = transform.getOrigin().z();
    
    x_target = robot_target_.pose.position.x;
    y_target = robot_target_.pose.position.y;
    z_target = robot_target_.pose.position.z;
    
    dist = sqrt((x_tool-x_target)*(x_tool-x_target) + (y_tool-y_target)*(y_tool-y_target) + (z_tool-z_target)*(z_tool-z_target));
    
    if(dist < 0.02)
    {
      std_srvs::Trigger srv;
      enable_placing_client_.call(srv);
    }

//     tf::Quaternion q(0,0,0,1);
//     tf::StampedTransform random_people(tf::Transform(), ros::Time::now(), base_name, target_name);
//     random_people.setOrigin(random_point);
//     random_people.setRotation(q);
//     br.sendTransform(random_people);
  }
};

int main(int argc, char** argv) {
  ros::init(argc, argv, "part_selection_node");
  
  PartSelection ps;

  ros::Rate r(10);

  while (ros::ok())
  {
    ps.publishTarget();

    ros::spinOnce();

    r.sleep();
  }

  return 0;
}
