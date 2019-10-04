#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <std_msgs/Bool.h>
#include <geometry_msgs/Vector3.h>
#include <geometry_msgs/PoseStamped.h>
#include <ar_conveyor_launch/EnableGrasping.h>
#include <box_info_msgs/BoxInformationStamped.h>
#include <dynamic_reconfigure/Reconfigure.h>
#include <dynamic_reconfigure/DoubleParameter.h>

class TargetPublisher {
 private:
  ros::NodeHandle nh_;
  ros::NodeHandle nh_private_;
  ros::Publisher robot_target_pub_;
  ros::Subscriber part_info_sub_;
  ros::Subscriber update_controller_sub_;
  ros::Subscriber enable_target_sub_;
  ros::ServiceClient enable_grasping_client_;
  ros::ServiceClient dynamic_reconfigure_client_;
  ros::ServiceServer part_selection_service_;
  tf::TransformListener listener_;
  geometry_msgs::PoseStamped robot_target_;
  std::string base_frame_;
  std::string end_effector_frame_;
  std::string part_type_;
  bool controller_updated_;
  bool publish_target_;

  double x_min_;
  double x_max_;
  double y_min_;
  double y_max_;
  double z_min_;
  double z_max_;
  double controller_mass_;

 public:
  TargetPublisher():
      nh_(),
      nh_private_("~")
  {
      ros::param::param<double>("~x_min", x_min_, 0.3);
      ros::param::param<double>("~x_max", x_max_, 0.5);
      ros::param::param<double>("~y_min", y_min_, -0.7);
      ros::param::param<double>("~y_max", y_max_, -0.3);
      ros::param::param<double>("~z_min", z_min_, 0.22);
      ros::param::param<double>("~z_max", z_max_, 0.4);

      ros::param::param<double>("~controller_mass", controller_mass_, 3.0);

      ros::param::param<std::string>("~base_frame", base_frame_, "base_link");
      ros::param::param<std::string>("~end_effector_frame", end_effector_frame_, "tool0");

      robot_target_pub_ = nh_.advertise<geometry_msgs::PoseStamped>("target_frame", 1);
      part_info_sub_ = nh_.subscribe("tp_box_pose", 1, &TargetPublisher::getPartInfo, this);
      update_controller_sub_ = nh_.subscribe("update_controller", 1, &TargetPublisher::updateController, this);
      enable_target_sub_ = nh_.subscribe("enable_target_publisher", 1, &TargetPublisher::enableTarget, this);
      enable_grasping_client_ = nh_.serviceClient<ar_conveyor_launch::EnableGrasping>("enable_grasping");
      dynamic_reconfigure_client_ = nh_.serviceClient<dynamic_reconfigure::Reconfigure>("cartesian_motion_controller/solver/set_parameters");

      robot_target_.pose.orientation =  tf::createQuaternionMsgFromRollPitchYaw(3.14, 0, 1.57);

      controller_updated_ = false;
      publish_target_ = true;
  }

  void getPartInfo(const box_info_msgs::BoxInformationStamped::ConstPtr& part_msg)
  {
    if(!publish_target_)
    {
      return;
    }

    if( (part_msg->pose_stamped.pose.position.x < x_min_) || (part_msg->pose_stamped.pose.position.x > x_max_) || 
        (part_msg->pose_stamped.pose.position.y < y_min_) || (part_msg->pose_stamped.pose.position.y > y_max_) )
    {
      return;
    }
    
    /*
    // update controller parameter if new target is far away from the previous goal
    double x, y, x_new, y_new, dist;
    
    x = robot_target_.pose.position.x;
    y = robot_target_.pose.position.y;
    x_new = part_msg->pose_stamped.pose.position.x;
    y_new = part_msg->pose_stamped.pose.position.y;
    
    dist = sqrt((x-x_new)*(x-x_new) + (y-y_new)*(y-y_new));
    
    std::cout << "distance: " << dist << std::endl;
    if(dist > 0.05 && controller_updated_)
    {
      std::cout << "updating mass for big distance " << std::endl;
      dynamic_reconfigure::Reconfigure reconf_srv;
      dynamic_reconfigure::DoubleParameter double_param;
      double_param.name = "mass";
      double_param.value = 12.0;
      reconf_srv.request.config.doubles.push_back(double_param);
      dynamic_reconfigure_client_.call(reconf_srv);

      controller_updated_ = false;
    }
    */

    part_type_ = part_msg->box_type;

    robot_target_.pose.position.x = part_msg->pose_stamped.pose.position.x;
    robot_target_.pose.position.y = part_msg->pose_stamped.pose.position.y;
    // robot_target_.pose.position.z = part_msg->pose_stamped.pose.position.z;
    robot_target_.pose.position.z = z_min_;

    if(part_type_.compare("TypeB") == 0)
    {
      robot_target_.pose.position.z += 0.05;
    }

    robot_target_.header.stamp = ros::Time::now();
    robot_target_.header.frame_id = base_frame_;

    robot_target_pub_.publish(robot_target_);
  }

  void updateController(std_msgs::Bool msg)
  {
    controller_updated_ = msg.data;
  }

  void enableTarget(std_msgs::Bool msg)
  {
    publish_target_ = msg.data;
  }

  void checkDistance()
  {
    if(!publish_target_)
    {
      return;
    }

    tf::StampedTransform transform;
    try
    {
      listener_.lookupTransform(base_frame_, end_effector_frame_, ros::Time(0), transform);
    }
    catch (tf::TransformException ex)
    {
      ROS_ERROR("%s",ex.what());
      ros::Duration(1.0).sleep();
      ROS_ERROR("Node: target publisher");
    }

    double x_tool, y_tool, z_tool, x_target, y_target, z_target, dist;
    x_tool = transform.getOrigin().x();
    y_tool = transform.getOrigin().y();
    z_tool = transform.getOrigin().z();

    x_target = robot_target_.pose.position.x;
    y_target = robot_target_.pose.position.y;
    z_target = robot_target_.pose.position.z;

    dist = sqrt((x_tool-x_target)*(x_tool-x_target) + (y_tool-y_target)*(y_tool-y_target) + (z_tool-z_target)*(z_tool-z_target));

    if(dist < 0.05 && !controller_updated_)
    {
      dynamic_reconfigure::Reconfigure reconf_srv;
      dynamic_reconfigure::DoubleParameter double_param;
      double_param.name = "mass";
      double_param.value = controller_mass_;
      reconf_srv.request.config.doubles.push_back(double_param);
      dynamic_reconfigure_client_.call(reconf_srv);

      controller_updated_ = true;
    }

    if(dist < 0.02)
    {
      ar_conveyor_launch::EnableGrasping srv;
      srv.request.part_type = part_type_;
      enable_grasping_client_.call(srv);
    }

  }
};

int main(int argc, char** argv) {
  ros::init(argc, argv, "target_publisher_node");

  TargetPublisher ps;

  ros::Rate r(5);

  while (ros::ok())
  {
    ps.checkDistance();

    ros::spinOnce();

    r.sleep();
  }

  return 0;
}
