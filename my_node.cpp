#include <mav_trajectory_generation/polynomial_optimization_linear.h>
#include <mav_trajectory_generation/trajectory.h>
#include <mav_trajectory_generation_ros/ros_visualization.h>
#include <geometry_msgs/PoseArray.h>
#include <nav_msgs/Odometry.h>
#include "geometry_msgs/Pose.h"
#include "ros/ros.h"
#include <tf/transform_listener.h>
#include <tf2_msgs/TFMessage.h>
#include <mav_planning_msgs/PolynomialTrajectory4D.h>
#include <mav_trajectory_generation/polynomial_optimization_nonlinear.h>
#include <mav_trajectory_generation/timing.h>
#include <mav_trajectory_generation_ros/feasibility_analytic.h>
#include <mav_trajectory_generation_ros/ros_conversions.h>
// float x_, y_,z_;
// void p_get(const tf2_msgs::TFMessage& qd)
// {
//   if(qd.child_frame_id=="/firefly/current_reference")
//   {
//       x_ = qd.translation.x;
//       y_ = qd.translation.y;
//       z_ = qd.translation.z;
//         std::cout<<"test_done"<<std::endl;

//   }
// }
int main(int argc,char** argv)
{
    
    ros::init(argc,argv,"my_node");
    ros::NodeHandle nh;
    ros::Publisher traj_pub = nh.advertise<visualization_msgs::MarkerArray>("trajectory", 10);
    ros::Publisher polynomial_trajectory_pub_ = nh.advertise<mav_planning_msgs::PolynomialTrajectory4D>("path_segments", 1);
    ros::Rate sleep_Rate(.5);
    ros::init(argc, argv, "my_node");
    ros::NodeHandle nodeHandle;
    // ros::Subscriber subscriber = nodeHandle.subscribe("/tf",10,p_get);
    tf::TransformListener listener;
    while(ros::ok())
    {

        tf::StampedTransform transform;
        try{
        listener.lookupTransform("/firefly/current_reference", "/world",
                                ros::Time(0), transform);
        }
        catch (tf::TransformException &ex) {
        ROS_ERROR("%s",ex.what());
        ros::Duration(1.0).sleep();
        continue;
        }

        ros::Time t0 = ros::Time::now();
        mav_trajectory_generation::Vertex::Vector vertices;
        const int dimension = 3;
        const int derivative_to_optimize = mav_trajectory_generation::derivative_order::ACCELERATION;
        mav_trajectory_generation::Vertex start(dimension), middle(dimension), end(dimension);

        start.makeStartOrEnd(Eigen::Vector3d(transform.getOrigin().x(),transform.getOrigin().y(),transform.getOrigin().z()), derivative_to_optimize);
        vertices.push_back(start);

        middle.addConstraint(mav_trajectory_generation::derivative_order::POSITION, Eigen::Vector3d(3,3,1));
        vertices.push_back(middle);

        middle.addConstraint(mav_trajectory_generation::derivative_order::POSITION, Eigen::Vector3d(3,3,0.5));
        vertices.push_back(middle);

        end.makeStartOrEnd(Eigen::Vector3d(3,3,0), derivative_to_optimize);
        vertices.push_back(end);

        std::vector<double> segment_times;
        const double v_max = 2.0;
        const double a_max = 2.0;
        segment_times = estimateSegmentTimes(vertices, v_max, a_max);

        const int N = 10;
        mav_trajectory_generation::PolynomialOptimization<N> opt(dimension);
        opt.setupFromVertices(vertices, segment_times, derivative_to_optimize);
        opt.solveLinear();

        mav_trajectory_generation::Segment::Vector segments;
        opt.getSegments(&segments);
        std::cout<<"seg_size"<<segments.size()<<std::endl;

        mav_trajectory_generation::Trajectory trajectory;
        opt.getTrajectory(&trajectory);

        // double sampling_time = .1;
        // int derivative_order = mav_trajectory_generation::derivative_order::POSITION;
        // Eigen::VectorXd sample = trajectory.evaluate(sampling_time, derivative_order);

        // double t_start = 2.0;
        // double t_end = 10.0;
        // double dt = 0.01;
        // std::vector<Eigen::VectorXd> result;
        // std::vector<double> sampling_times; // Optional.
        // trajectory.evaluateRange(t_start, t_end, dt, derivative_order, &result, &sampling_times);

        visualization_msgs::MarkerArray markers;
        double distance = 1.0; // Distance by which to seperate additional markers. Set 0.0 to disable.
        std::string frame_id = "world";

        mav_trajectory_generation::drawMavTrajectory(trajectory, distance, frame_id, &markers);
        mav_planning_msgs::PolynomialTrajectory4D msg;
        mav_trajectory_generation::trajectoryToPolynomialTrajectoryMsg(trajectory,&msg);
        msg.header.stamp = ros::Time::now();
        msg.header.frame_id = "world";
        polynomial_trajectory_pub_.publish(msg);
        traj_pub.publish(markers);
        ROS_INFO("Take %f sec to get optimal traject", (ros::Time::now() - t0).toSec());

        

        

        vertices.clear();
        sleep_Rate.sleep();
        ros::spinOnce();
    }
    return 0;
}
