#ifndef __CIRCLE_DETECTION_NODE_H
#define __CIRCLE_DETECTION_NODE_H

#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TwistStamped.h>
#include <sensor_msgs/Imu.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <iostream>
#include <opencv2/opencv.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <vector>
#include <algorithm>
#include <queue>
#include <Eigen/Dense>
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <nav_msgs/Path.h>
#include <nav_msgs/Odometry.h>
#include <numeric>
#include <math.h>
#include <fstream>

#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TwistStamped.h>
#include <sensor_msgs/Imu.h>
//#include <geometry_msgs/Vector3.h>

#include <edcircles/EDCircles.h>

#include <float.h>

using namespace std;
using namespace cv;



    struct distance_i_j
    {
        double distance;
        int i;
        int j;
    };


    class CIRCLE //circle
    {
        public:
            CIRCLE() {
                Z = 1;
            }
            ~CIRCLE() {}
            
            void init(ros::NodeHandle& nh);
            

        private:
            cv::Mat color_pic;
            cv::Mat depth_pic;
            cv::Mat img_gray;               
           
            ros::Subscriber depth_sub, color_sub, odom_sub;
            ros::Publisher cir_pub, center_pub;
            ros::Timer detect_timer, pub_odom, circle_center_timer, position_timer;

            double fx_ , fy_ , cx_ , cy_;
            Eigen::Matrix<double,3,1> Circle_loc_in_World;
            Eigen::Matrix<double,3,1> point2D_h;
            Eigen::Matrix<double,3,1> position_list;
            Eigen::Matrix3d camera_inner_matrix;
            Eigen::Matrix3d camera_inner_matrix_inverse;
            Eigen::Matrix3d rotation_matrix;
            Eigen::Matrix3d rotation_matrix_inverse;
            Eigen::Vector3d odom_pos_ , odom_vel_ , odom_acc_; // odometry state
            Eigen::Quaterniond odom_orient_;

            bool have_odom_ , have_circle_;
            double Z;
            geometry_msgs::Point point_msg;
            cv::Point des_cen;            
            int des_r;
            vector<distance_i_j> ED_lib_dis;

            
            void color_pic_Callback(const sensor_msgs::Image::ConstPtr&msg);
            void depth_pic_Callback(const sensor_msgs::Image::ConstPtr&msg);
            void timerCallback(const ros::TimerEvent&);
            void waypoint_callback(const ros::TimerEvent&);
         
            inline double distance_cv_points(const cv::Point& a , const cv::Point& b);
            inline static bool cv_point_cmp(const distance_i_j& a , const distance_i_j& b);

            const int averageFrames = 5; // 每五帧图像计算一次均值
            const double maxError = 0.5; // 最大误差
            int numFrames;
            cv::Point3d detectedCircleCenter , targetCircleCenter , targetCircleCenter_cp ;
            cv::Point3d worldPoint;
            double targetCircleCenterX, targetCircleCenterY, targetCircleCenterZ;
            // double pertargetCircleCenterX, pretargetCircleCenterY, pretargetCircleCenterZ;
            int ring_width;
            bool have_loc;
            void moveTowardsTarget(const cv::Point3d& currentCircleCenter);
            cv::Point3d pixelToWorldCoordinate();
    };


# endif