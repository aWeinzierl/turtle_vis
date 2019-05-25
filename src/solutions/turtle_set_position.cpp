/*********************************************************************
* STD INCLUDES
********************************************************************/
#include <iostream>
#include <fstream>
#include <pthread.h>


/*********************************************************************
* ROS INCLUDES
********************************************************************/
#include <ros/ros.h>
#include <std_msgs/String.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <tf/transform_listener.h>
#include <tf/transform_broadcaster.h>
#include <tf_conversions/tf_eigen.h>

/*********************************************************************
* EIGEN INCLUDES
********************************************************************/
#include <Eigen/Eigen>
#include <Eigen/StdVector>
#include <Eigen/Geometry>
#include <Eigen/Core>


#include <boost/algorithm/string.hpp>
#include <turtle_vis/DesiredPose.h>
#include <turtle_vis/send_desired_pose.h>



/*********************************************************************
 * SEVICES AND MESSAGES
 * ******************************************************************/
//SET HEADERS FOR THE SERVICE AND THE MESSAGES OF THE TURTLE_VIS PACKAGE

using namespace Eigen;


int main(int argc, char** argv)
{

    ros::init(argc, argv, "turtle_visualization",ros::init_options::AnonymousName);

    ROS_INFO_STREAM("**Client turtle desired position");

    ros::NodeHandle n;
    ros::Rate r(60);

    //INITIALIZE THE CLIENT
    ros::ServiceClient client=n.serviceClient<turtle_vis::send_desired_pose>("TurtlePose");

    turtle_vis::send_desired_pose msg;

    std::string myString;

    static tf::TransformBroadcaster br;
    tf::Transform transform;
    tf::Quaternion qtf;

    while(ros::ok())
    {

        std::vector<double> vals;

        ROS_INFO_STREAM("Give me the desired position of the turtle: x,y,theta");
        std::cin>>myString;
        std::vector<std::string> fields;
        boost::split(fields, myString, boost::is_any_of(","));
        msg.request.desired_pose.x =     std::stod(fields[0]);
        msg.request.desired_pose.y =     std::stod(fields[1]);
        msg.request.desired_pose.theta = std::stod(fields[2]);

        ROS_INFO_STREAM("Received x="
        << msg.request.desired_pose.x << ", y="
        << msg.request.desired_pose.y<< ", theta="
        << msg.request.desired_pose.theta);

        qtf.setRPY(0,0,msg.request.desired_pose.theta);
        transform.setOrigin(tf::Vector3(msg.request.desired_pose.x, msg.request.desired_pose.y, 0));
        transform.setRotation(qtf);

        br.sendTransform(tf::StampedTransform(transform,ros::Time::now(),"/world","/turtle_desired"));


        if(client.call(msg))
        {
            //#>>>>TODO:PLOT THE MESSAGE
        }
        else
        {
            ROS_ERROR_STREAM("Failed to call the service 'TurtlePose'");
            return 1;
        }

    }



    return 0;
}
