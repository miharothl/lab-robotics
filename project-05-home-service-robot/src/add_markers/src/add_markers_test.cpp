#include <std_msgs/Empty.h>
#include <std_msgs/UInt8.h>
#include <ros/ros.h>
#include <visualization_msgs/Marker.h>

visualization_msgs::Marker MarkerMessageTemplate()
{
  // Set our initial shape type to be a cube
  uint32_t shape = visualization_msgs::Marker::CUBE;

  visualization_msgs::Marker marker;
  // Set the frame ID and timestamp.  See the TF tutorials for information on these.
  marker.header.frame_id = "/map";
  marker.header.stamp = ros::Time::now();
  
  // Set the namespace and id for this marker.  This serves to create a unique ID
  // Any marker sent with the same namespace and id will overwrite the old one
  marker.ns = "basic_shapes";
  marker.id = 0;
  
  // Set the marker type.  Initially this is CUBE, and cycles between that and SPHERE, ARROW, and CYLINDER
  marker.type = shape;
  
  // Set the marker action.  Options are ADD, DELETE, and new in ROS Indigo: 3 (DELETEALL)
  marker.action = visualization_msgs::Marker::ADD;
  
  // Set the pose of the marker.  This is a full 6DOF pose relative to the frame/time specified in the header
  marker.pose.position.x = 0;
  marker.pose.position.y = 0;
  marker.pose.position.z = 0;
  marker.pose.orientation.x = 0.0;
  marker.pose.orientation.y = 0.0;
  marker.pose.orientation.z = 0.0;
  marker.pose.orientation.w = 1.0;
 
  // Set the scale of the marker -- 1x1x1 here means 1m on a side
  marker.scale.x = .20;
  marker.scale.y = .20;
  marker.scale.z = .20;
  
  // Set the color -- be sure to set alpha to something non-zero!
  marker.color.r = 0.0f;
  marker.color.g = 1.0f;
  marker.color.b = 0.0f;
  marker.color.a = 1.0;
  
  marker.lifetime = ros::Duration();

  return marker;
}

void NewLoadAtPickupLocation()
{
  ROS_INFO("New load at pick-up location.");

  ros::NodeHandle n;
  if (n.hasParam("/pick_up_location/tx"))
  {
    // ----------------------------------
    // 1. SHOW MARKER AT PICK-UP LOCATION

    visualization_msgs::Marker marker = MarkerMessageTemplate();

    n.getParam("/pick_up_location/tx", marker.pose.position.x);
    n.getParam("/pick_up_location/ty", marker.pose.position.y);
    
    //ROS_INFO("Pick-up location x= %f y= %f", marker.pose.position.x, marker.pose.position.y);

    // Publish the marker
    ros::Publisher marker_pub = n.advertise<visualization_msgs::Marker>("visualization_marker", 1);
    while (marker_pub.getNumSubscribers() < 1)
    {
      if (!ros::ok())
      {
        return;
      }
      ROS_WARN("Please create a subscriber to the marker.");
      sleep(1);
    }
    
    // Set the marker action.  Options are ADD, DELETE, and new in ROS Indigo: 3 (DELETEALL)
    marker.action = visualization_msgs::Marker::ADD;
    marker_pub.publish(marker);
    ros::spinOnce();
    sleep(5);
  }
  else
  {
    ROS_WARN("Pick-up and drop-off configuration is missing.");
  }
}

void LoadObject()
{
  ROS_INFO("Loading...");

  // ------------------------------------
  // 2. DELETE MARKER AT PICK-UP LOCATION
  
  ros::NodeHandle n;

  if (n.hasParam("/pick_up_location/tx"))
  {
    visualization_msgs::Marker marker = MarkerMessageTemplate();

    n.getParam("/pick_up_location/tx", marker.pose.position.x);
    n.getParam("/pick_up_location/ty", marker.pose.position.y);
    ROS_INFO("Pick-up location x= %f y= %f", marker.pose.position.x, marker.pose.position.y);

    // Publish the marker
    ros::Publisher marker_pub = n.advertise<visualization_msgs::Marker>("visualization_marker", 1);
    while (marker_pub.getNumSubscribers() < 1)
    {
      if (!ros::ok())
      {
        return;
      }
      ROS_WARN("Please create a subscriber to the marker");
      sleep(1);
    }
    
    // Set the marker action.  Options are ADD, DELETE, and new in ROS Indigo: 3 (DELETEALL)
    marker.action = visualization_msgs::Marker::DELETE;
    marker_pub.publish(marker);
    ros::spinOnce();
    sleep(5);
  }
  else
  {
    ROS_WARN("Pick-up and drop-off configuration is missing.");
  }
}

void UnloadObject()
{
  ROS_INFO("Unloading...");

  ros::NodeHandle n;

  if (n.hasParam("/pick_up_location/tx"))
  {
    // ----------------------------------
    // 3. ADD MARKER AT DROP-OFF LOCATION
   
    visualization_msgs::Marker marker = MarkerMessageTemplate();

    n.getParam("/drop_off_location/tx", marker.pose.position.x);
    n.getParam("/drop_off_location/ty", marker.pose.position.y);
    ROS_INFO("Pick-up location x= %f y= %f", marker.pose.position.x, marker.pose.position.y);

    // Publish the marker
    ros::Publisher marker_pub = n.advertise<visualization_msgs::Marker>("visualization_marker", 1);
    while (marker_pub.getNumSubscribers() < 1)
    {
      if (!ros::ok())
      {
        return;
      }
      ROS_WARN("Please create a subscriber to the marker");
      sleep(1);
    }
    
    // Set the marker action.  Options are ADD, DELETE, and new in ROS Indigo: 3 (DELETEALL)
    marker.action = visualization_msgs::Marker::ADD;
    marker_pub.publish(marker);
    ros::spinOnce();
    sleep(5);
    
    // -------------------------------------
    // 4. DELETE MARKER AT DROP-OFF LOCATION
    
    // Set the marker action.  Options are ADD, DELETE, and new in ROS Indigo: 3 (DELETEALL)
    marker.action = visualization_msgs::Marker::DELETE;
    marker_pub.publish(marker);
    ros::spinOnce();
  }
  else
  {
    ROS_WARN("Pick-up and drop-off configuration is missing.");
  }

  // ---------------------------------
  // 1. ADD MARKER AT PICK-UP LOCATION
  NewLoadAtPickupLocation();
}

bool initialize = false;
bool pickup_location_reached = false;
bool dropoff_location_reached = false;

void GoalReachedCallback(const std_msgs::UInt8::ConstPtr& msg)
{
  ROS_INFO("Received goal reached notification (%d)", msg->data);

  if (msg->data == 1)
  {
    ROS_INFO("Received pick-up location reached message.");
    pickup_location_reached = true;
  }
  else
  {
    ROS_INFO("Received drop-off location reached message.");
    dropoff_location_reached = true;
  }
}

int main( int argc, char** argv )
{
  ros::init(argc, argv, "add_markers");
  ros::NodeHandle n;
  ros::Rate r(1);
  ros::Publisher marker_pub = n.advertise<visualization_msgs::Marker>("visualization_marker", 1);
  ros::Subscriber sub_goal_reached = n.subscribe("/goalReached", 1000, GoalReachedCallback);

  while (ros::ok())
  {
    if (initialize == false)
    {
      NewLoadAtPickupLocation();
      initialize = true;
    }
 
    //sleep(5);
    pickup_location_reached = true;
    if (pickup_location_reached == true)
    {
      ROS_INFO("Load object...");
      pickup_location_reached = false; 

      LoadObject();
    }
    
    sleep(5);
    dropoff_location_reached = true;
    if (dropoff_location_reached == true)
    {
      ROS_INFO("Unload object...");
      dropoff_location_reached = false; 
     
      UnloadObject();
    }

    ros::spinOnce();
    r.sleep();
  } 
}
