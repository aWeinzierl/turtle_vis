#ifndef TURTLECLASS_H
#define TURTLECLASS_H

/*********************************************************************
* ROS INCLUDES
********************************************************************/
#include <ros/ros.h>

#include <std_msgs/String.h>
#include <turtle_vis/DesiredPose.h>
#include <turtle_vis/send_desired_pose.h>

/*********************************************************************
* EIGEN INCLUDES
********************************************************************/
#include <Eigen/Eigen>
#include <Eigen/StdVector>
#include <Eigen/Geometry>
#include <Eigen/Core>

using namespace Eigen;

namespace turtleSpace
{
    class TurtleClass
    {
    public:

				pthread_mutex_t count_mutex;

        TurtleClass();
        ~TurtleClass();

                bool getDPose(turtle_vis::send_desired_pose::Request &req, turtle_vis::send_desired_pose::Response &res);

                void getPose(const turtle_vis::DesiredPose::ConstPtr &msg);

                Vector3d getLocalPose();
                Vector3d getLocalDesiredPose();
                void SetLocalPose(const Vector3d &pose);
                void SetLocalDesiredPose(const Vector3d &pose);


    private:
                Vector3d turtlePose_g; //TOOD: add although no comment? what about setters?
                Vector3d turtlePose_desired_g;
    };


}

#endif // TURTLECLASS_H
