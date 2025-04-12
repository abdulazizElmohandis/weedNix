#include <ros/ros.h>
#include <nav_msgs/Path.h>
#include <geometry_msgs/PoseStamped.h>
#include <tf/transform_listener.h>

int main(int argc, char** argv)
{
    ros::init(argc, argv, "path_publisher");
    ros::NodeHandle nh;

    ros::Publisher path_pub = nh.advertise<nav_msgs::Path>("robot_path", 10);
    tf::TransformListener listener;

    nav_msgs::Path path;
    path.header.frame_id = "odom";  // or "odom", depending on your setup

    ros::Rate rate(10);
    while (ros::ok())
    {
        tf::StampedTransform transform;
        try
        {
            listener.lookupTransform("odom", "base_link", ros::Time(0), transform);

            geometry_msgs::PoseStamped pose;
            pose.header.stamp = ros::Time::now();
            pose.header.frame_id = "odom";
            pose.pose.position.x = transform.getOrigin().x();
            pose.pose.position.y = transform.getOrigin().y();
            pose.pose.position.z = transform.getOrigin().z();
            pose.pose.orientation.x = transform.getRotation().x();
            pose.pose.orientation.y = transform.getRotation().y();
            pose.pose.orientation.z = transform.getRotation().z();
            pose.pose.orientation.w = transform.getRotation().w();

            path.poses.push_back(pose);
            path.header.stamp = pose.header.stamp;

            path_pub.publish(path);
        }
        catch (tf::TransformException &ex)
        {
            ROS_WARN("%s", ex.what());
        }

        rate.sleep();
    }

    return 0;
}
