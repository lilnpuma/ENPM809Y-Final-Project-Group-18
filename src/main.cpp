#include <actionlib/client/simple_action_client.h>
#include <fiducial_msgs/FiducialTransformArray.h>
#include <geometry_msgs/Twist.h>  //for geometry_msgs::Twist
#include <move_base_msgs/MoveBaseAction.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>
#include <ros/ros.h>
#include "std_msgs/String.h"

typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;

int pos_marker_loc;   //marker index
static std::array<std::array<double, 3>, 4> marker_loc; // locations of the markers after transformation


void  listen(tf2_ros::Buffer& tfBuffer) {
  //for listener

  geometry_msgs::TransformStamped transformStamped;
  try {
    transformStamped = tfBuffer.lookupTransform("map", "marker_frame", ros::Time(0));
    auto trans_x = transformStamped.transform.translation.x;
    auto trans_y = transformStamped.transform.translation.y;
    auto trans_z = transformStamped.transform.translation.z;
    
    marker_loc.at(pos_marker_loc).at(0) = pos_marker_loc;
    marker_loc.at(pos_marker_loc).at(1) = trans_x;
    marker_loc.at(pos_marker_loc).at(2) = trans_y;

    ROS_INFO_STREAM("pos_ marker loc new at listener is "<<pos_marker_loc);


    ROS_INFO_STREAM("Position in map frame: ["
      << trans_x << ","
      << trans_y << ","
      << trans_z << "]"
    );

    ROS_INFO_STREAM("Marker is at location:  ["<< marker_loc.at(0).at(0)<< "]"<<"Marker is at location:  ["
    << marker_loc.at(1).at(0)<< "]"<<"Marker is at location:  ["<< marker_loc.at(2).at(0)<< "]");
    
  }
  catch (tf2::TransformException& ex) {
    ROS_WARN("%s", ex.what());
    ros::Duration(1.0).sleep();
    
  }
}
static bool saw_marker{false};
//testing out the callback method
void fiducial_callback(const fiducial_msgs::FiducialTransformArray::ConstPtr& msg)
{
  ROS_INFO("Inside callback");

 if (!msg->transforms.empty())
 {  saw_marker = true;
    ROS_INFO_STREAM("Seen marker:  ["<< msg->transforms[0].fiducial_id<< "]");
    pos_marker_loc = msg->transforms[0].fiducial_id;
    static tf2_ros::TransformBroadcaster brc;
    geometry_msgs::TransformStamped transformStamped;
     
    //broadcast the new frame to /tf Topic
    transformStamped.header.stamp = ros::Time::now();
    transformStamped.header.frame_id = "explorer_tf/camera_rgb_optical_frame";
    transformStamped.child_frame_id = "marker_frame";
    //creation of marker_frame with relative to camera frame at given distance
    //which is exactly what we are seeing in the camera frame
    transformStamped.transform.translation.x = msg->transforms[0].transform.translation.x;
    transformStamped.transform.translation.y = msg->transforms[0].transform.translation.y;
    transformStamped.transform.translation.z = msg->transforms[0].transform.translation.z;
    transformStamped.transform.rotation.x = msg->transforms[0].transform.rotation.x;
    transformStamped.transform.rotation.y = msg->transforms[0].transform.rotation.y;
    transformStamped.transform.rotation.z = msg->transforms[0].transform.rotation.z;
    transformStamped.transform.rotation.w = msg->transforms[0].transform.rotation.w;
    
    ROS_INFO("Broadcasting marker location");
    brc.sendTransform(transformStamped);
    // marker_loc = listen(msg, marker_loc, tfBuffer);

    //we can create new moving goal over here
 }

}



void get_goal(ros::NodeHandle m_nh, std::array<std::array<double, 2>, 4> &aruco_loc)
{
  XmlRpc::XmlRpcValue pos_list1;
  XmlRpc::XmlRpcValue pos_list2;
  XmlRpc::XmlRpcValue pos_list3;
  XmlRpc::XmlRpcValue pos_list4;
  //this can be done using 1 array (will try after testing)
  m_nh.getParam("/aruco_lookup_locations/target_1", pos_list1);
  m_nh.getParam("/aruco_lookup_locations/target_2", pos_list2);
  m_nh.getParam("/aruco_lookup_locations/target_3", pos_list3);
  m_nh.getParam("/aruco_lookup_locations/target_4", pos_list4);
  
  ROS_ASSERT(pos_list1.getType() == XmlRpc::XmlRpcValue::TypeArray);
  ROS_ASSERT(pos_list2.getType() == XmlRpc::XmlRpcValue::TypeArray);
  ROS_ASSERT(pos_list3.getType() == XmlRpc::XmlRpcValue::TypeArray);
  ROS_ASSERT(pos_list1.getType() == XmlRpc::XmlRpcValue::TypeArray);

  for (int32_t i = 0; i < pos_list1.size(); ++i)
  {
    ROS_ASSERT(pos_list1[i].getType() == XmlRpc::XmlRpcValue::TypeDouble);
    aruco_loc.at(0).at(i) = static_cast<double>(pos_list1[i]);
  }

  for (int32_t i = 0; i < pos_list2.size(); ++i)
  {
    ROS_ASSERT(pos_list2[i].getType() == XmlRpc::XmlRpcValue::TypeDouble);
    aruco_loc.at(1).at(i) = static_cast<double>(pos_list2[i]);
  } 

  for (int32_t i = 0; i < pos_list3.size(); ++i)
  {
    ROS_ASSERT(pos_list3[i].getType() == XmlRpc::XmlRpcValue::TypeDouble);
    aruco_loc.at(2).at(i) = static_cast<double>(pos_list3[i]);
  } 

  for (int32_t i = 0; i < pos_list1.size(); ++i)
  {
    ROS_ASSERT(pos_list4[i].getType() == XmlRpc::XmlRpcValue::TypeDouble);
    aruco_loc.at(3).at(i) = static_cast<double>(pos_list4[i]);
  }

}

// void broadcast() {
//   //for broadcaster
//   static tf2_ros::TransformBroadcaster br;
//   geometry_msgs::TransformStamped transformStamped;

//   //broadcast the new frame to /tf Topic
//   transformStamped.header.stamp = ros::Time::now();
//   transformStamped.header.frame_id = "explorer_tf/camera_rgb_optical_frame";
//   transformStamped.child_frame_id = "my_frame";

//   transformStamped.transform.translation.x = 0.5;
//   transformStamped.transform.translation.y = 0.5;
//   transformStamped.transform.translation.z = 0.2;
//   transformStamped.transform.rotation.x = 0;
//   transformStamped.transform.rotation.y = 0;
//   transformStamped.transform.rotation.z = 0;
//   transformStamped.transform.rotation.w = 1;
//   ROS_INFO("Broadcasting");
//   br.sendTransform(transformStamped);
// }



int main(int argc, char** argv)
{
  bool explorer_goal_sent = false;
  
  std::array<std::array<double, 2>, 4> aruco_loc;
  
  
 
  

  ros::init(argc, argv, "simple_navigation_goals");
  // ros::init(argc, argv, "listening to fiducial");
  ros::NodeHandle nh;
  get_goal(nh, aruco_loc);
  


  

  // tell the action client that we want to spin a thread by default
  MoveBaseClient explorer_client("/explorer/move_base", true);
  // tell the action client that we want to spin a thread by default
  MoveBaseClient follower_client("/follower/move_base", true);

  // wait for the action server to come up
  while (!explorer_client.waitForServer(ros::Duration(5.0))) {
    ROS_INFO("Waiting for the move_base action server to come up for explorer");
  }

  // while (!follower_client.waitForServer(ros::Duration(5.0))) {
  //   ROS_INFO("Waiting for the move_base action server to come up for follower");
  // }

  move_base_msgs::MoveBaseGoal explorer_goal;
  move_base_msgs::MoveBaseGoal follower_goal;

  // Build goal for explorer
  int i = 0;
  explorer_goal.target_pose.header.frame_id = "map";
  explorer_goal.target_pose.header.stamp = ros::Time::now();
  explorer_goal.target_pose.pose.position.x = aruco_loc.at(i).at(0);//
  explorer_goal.target_pose.pose.position.y = aruco_loc.at(i).at(1);//
  explorer_goal.target_pose.pose.orientation.w = 1;
  i++;

  //Build goal for follower
  // follower_goal.target_pose.header.frame_id = "map";
  // follower_goal.target_pose.header.stamp = ros::Time::now();
  // follower_goal.target_pose.pose.position.x = -0.289296;//
  // follower_goal.target_pose.pose.position.y = -1.282680;//
  // follower_goal.target_pose.pose.orientation.w = 1.0;


  explorer_client.waitForResult();

  ROS_INFO("Sending goal");
  // follower_client.sendGoal(follower_goal);
  explorer_client.waitForResult();


  tf2_ros::Buffer tfBuffer;
  tf2_ros::TransformListener tfListener(tfBuffer);
  ros::Rate loop_rate(100);
 
  //buidling a publisher to do some tricks
  ros::Publisher m_velocity_publisher;
  ros::Subscriber fid_reader;
  geometry_msgs::Twist msg;
  msg.linear.x = 0;
  msg.angular.z = 0.4;
  //writing this to move the bot with a constant angular velocity
  m_velocity_publisher = nh.advertise<geometry_msgs::Twist>("/explorer/cmd_vel", 100);
  
  

  

  

  while (ros::ok()) {
    
    if (!explorer_goal_sent)     {
      ROS_INFO("Sending goal for explorer");
      explorer_client.sendGoal(explorer_goal);//this should be sent only once
      explorer_goal_sent = true;
      saw_marker = false;
    }
    if (explorer_client.getState() == actionlib::SimpleClientGoalState::SUCCEEDED) 
    {
      
      
      ROS_INFO("Hooray, robot reached goal");
     
      ROS_INFO("Imparting Angular Velocity");
      while (saw_marker != true)
      {
      m_velocity_publisher.publish(msg);
      fid_reader = nh.subscribe("/fiducial_transforms", 1000, fiducial_callback);
      ROS_INFO("I am near the callback");
      ros::spinOnce();
      loop_rate.sleep();
      }

    
      

       

        if (i > 3)
        {
        explorer_goal_sent = false;
        ROS_INFO_STREAM("Time to go home now");
        explorer_goal.target_pose.header.stamp = ros::Time::now();
        explorer_goal.target_pose.pose.position.x = -3.98;//
        explorer_goal.target_pose.pose.position.y = 2.44;
        explorer_goal.target_pose.pose.orientation.w = 1;
        }
        else
        {
        explorer_goal_sent = false;
        ROS_INFO_STREAM("Updatinng to next goal at "<<i);
        explorer_goal.target_pose.header.stamp = ros::Time::now();
        explorer_goal.target_pose.pose.position.x = aruco_loc.at(i).at(0);//
        explorer_goal.target_pose.pose.position.y = aruco_loc.at(i).at(1);//
        i++;
        }

       //this is nor required, rotate until it detects the marker
      // ros::Duration(10).sleep(); 
      // msg.angular.z = 0;
      // m_velocity_publisher.publish(msg);
      
      
      // ros::spinOnce();


    }
    // if (!follower_goal_sent) {
    //   ROS_INFO("Sending goal for explorer");
    //   follower_client.sendGoal(follower_goal);//this should be sent only once
    //   follower_goal_sent = true;
    // }
    // if (follower_client.getState() == actionlib::SimpleClientGoalState::SUCCEEDED) {
    //   ROS_INFO("Hooray, robot reached goal");
    // }
    //broadcast();
    
    //ros::spinOnce(); //uncomment this if you have subscribers in your code
    listen(tfBuffer);
    
    ROS_INFO_STREAM("Updated marker locations"<<marker_loc.at(0).at(1)<<marker_loc.at(0).at(2)
    <<std::endl<<"Updated marker 2 nd locations"<<marker_loc.at(1).at(1)<<marker_loc.at(1).at(2)
    <<std::endl<<"Updated marker 3 rd locations"<<marker_loc.at(2).at(1)<<marker_loc.at(2).at(2));
    // loop_rate.sleep();
  }


}