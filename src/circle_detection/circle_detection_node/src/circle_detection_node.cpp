#include <edcircles/EDLib.h>
#include <edcircles/ED.h>
#include <circle_detection_node.h>
#include <edcircles/EDPF.h>
#include <edcircles/EDCircles.h>
#include <edcircles/EDColor.h>
#include <edcircles/EDLib.h>
#include <edcircles/EDLines.h>

// #include <EDCircles.h>
// #include <EDLib.h>
// #include <EDColor.h>
// #include <EDPF.h>
// #include <EDLines.h>
// #include <ED.h>


using namespace std;
using namespace cv;
using namespace Eigen;

int main(int argc, char** argv)
{
  ros::init(argc, argv, "circle_detection_node");
  ros::NodeHandle nh("~");
  CIRCLE circle;        //初始化circle类
  circle.init(nh);      //进入circle类的init函数
  ros::spin();              
  return 0;
}


/*创建一个节点circle_detection_node，订阅里程计信息、深度相机信息、光学相机信息，发布圆心在世界坐标系下的坐标点信息*/

void CIRCLE::init(ros::NodeHandle &nh)
  {
    have_odom_ = false;
    have_circle_ = false;
    have_loc = false;
    nh.param("camera_inner_varable/fx", fx_, 385.7967834472656);
    nh.param("camera_inner_varable/fy", fy_, 326.0679931640625);
    nh.param("camera_inner_varable/cx", cx_, 385.7967834472656);
    nh.param("camera_inner_varable/cy", cy_, 237.22044372558594);
    nh.param("target_circle_center/x", targetCircleCenterX, 1.7);
    nh.param("target_circle_center/y", targetCircleCenterY, 0.3);
    nh.param("target_circle_center/z", targetCircleCenterZ, 0.2);
    targetCircleCenter.x = targetCircleCenterX;
    targetCircleCenter.y = targetCircleCenterY;
    targetCircleCenter.z = targetCircleCenterZ;
    ROS_INFO("target circle1:(%f, %f, %f)", targetCircleCenter.x, targetCircleCenter.y, targetCircleCenter.z);
 
    color_sub = nh.subscribe("/camera/color/image_raw", 1, &CIRCLE::color_pic_Callback, this);          
    // depth_sub = nh.subscribe("/camera/aligned_depth_to_color/image_raw", 1, &CIRCLE::depth_pic_Callback, this);
    
    /*发布话题圆心坐标*/
     center_pub = nh.advertise<geometry_msgs::Point>("center", 10);
     circle_center_timer = nh.createTimer(ros::Duration(0.01), &CIRCLE::waypoint_callback, this);  
     position_timer = nh.createTimer(ros::Duration(0.01), &CIRCLE::timerCallback, this);  
    // //  int numFrames=0;
    //  int ring_width=0;
    
            

  }

void CIRCLE::color_pic_Callback(const sensor_msgs::Image::ConstPtr&msg){
    
    cv_bridge::CvImagePtr col;
    col = cv_bridge::toCvCopy(msg,sensor_msgs::image_encodings::TYPE_8UC3);
    color_pic=col->image;  
    cv::cvtColor(color_pic,color_pic,cv::COLOR_RGB2BGR);
 
    /**EDCIRCLES Circle Segment Detection**/

    //***************************** EDCIRCLES Circle Segment Detection *****************************
    // Detection of circles directly from the input image
    cv::cvtColor(color_pic, img_gray, cv::COLOR_BGR2GRAY);//转变为灰度图
    EDCircles testEDCircles = EDCircles(img_gray);//
    Mat circleImg = testEDCircles.drawResult(true, ImageStyle::CIRCLES);
    imshow("Circle Image ED", circleImg);
    cv::waitKey(10);

    //Get circle information as [cx, cy, r]
    vector<mCircle> circles = testEDCircles.getCircles();

    /*判断是否是同心圆*/
    if(circles.size() == 1){
        mCircle des_circle = circles[0];        
        des_cen = cv::Point(int(des_circle.center.x),int(des_circle.center.y));     
        des_r = int(des_circle.r);
    }
    else if(circles.size() > 1){
            
        for(auto i = 0 ; i < static_cast<int>(circles.size()) ; ++i){
                        
            for(auto j = i + 1 ; j <static_cast<int>( circles.size()); ++j){
                
                double dis = CIRCLE::distance_cv_points(circles[i].center,circles[j].center);
                distance_i_j hh;
                hh.distance = dis;//a b之间的欧氏距离
                hh.i = int(i);
                hh.j = int(j);
                ED_lib_dis.push_back(hh);//push_back() 是 C++ 中的一个向量成员函数，它用于将元素添加到向量的末尾。当调用 push_back() 时，它会将传入的元素添加到向量的末尾，并将向量的大小增加一个单位。也就是说，ED_lib_dis 中原本有 n 个元素，调用 push_back(hh) 后，ED_lib_dis 将会有 n+1 个元素，最后一个元素就是 hh。
            }
        }
        sort(ED_lib_dis.begin() , ED_lib_dis.end() ,  CIRCLE::cv_point_cmp);//这将使用 cv_point_cmp 函数作为排序的比较准则，按照 distance 的值从小到大对 ED_lib_dis 中的元素进行排序。
    
    
        /*画出ED_lib_dis中的同心圆*/
        if(ED_lib_dis[0].distance < 30){
            
            if(circles[ED_lib_dis[0].i].r > circles[ED_lib_dis[0].j].r){
                                
                ring_width=abs(circles[ED_lib_dis[0].i].r- circles[ED_lib_dis[0].j].r);
                cv::circle(color_pic,circles[ED_lib_dis[0].i].center,circles[ED_lib_dis[0].i].r,cv::Scalar(0,255,0) ,ring_width);//画圆
                des_cen = circles[ED_lib_dis[0].i].center;//
                des_r = circles[ED_lib_dis[0].i].r;//
            }
            else{
                
                ring_width=abs(circles[ED_lib_dis[0].i].r- circles[ED_lib_dis[0].j].r);
                cv::circle(color_pic,circles[ED_lib_dis[0].j].center,circles[ED_lib_dis[0].j].r,cv::Scalar(0,255,0) ,ring_width);
                des_cen = circles[ED_lib_dis[0].j].center;
                des_r = circles[ED_lib_dis[0].j].r;
            }                                   
            have_circle_ = true;
        }
    }else{
                
        have_circle_ = false;
    }
    cout<<"circle center:"<< des_cen <<endl;
    cout<<"circle radius"<<des_r<<endl;
    ED_lib_dis.clear();
    cv::imshow("ED_lib ",color_pic); 
    cv::waitKey(10);
          
  
}
  


void CIRCLE::depth_pic_Callback(const sensor_msgs::Image::ConstPtr&msg){

    cv_bridge::CvImagePtr dep;
    dep = cv_bridge::toCvCopy(msg,sensor_msgs::image_encodings::TYPE_16UC1);
    depth_pic = dep->image;
  
    if( have_circle_ && have_odom_){

        vector<cv::Point> test;
        vector<double> total_depth;
        for (int i = (int(des_cen.x) - des_r ) ; i < (int(des_cen.x) + des_r ) ; i++){

            ushort* ptr = depth_pic.ptr<ushort>(i);
            for (int j = (int(des_cen.y) - des_r  ) ; j < (int(des_cen.y) + des_r ) ; j++){    

                cv::Point points_in_circle(i,j);
                if(ptr[j] != 0 && ptr[j] != 65535 && ptr[j] < 4000 &&  (des_r-ring_width) < CIRCLE::distance_cv_points(points_in_circle,des_cen) && CIRCLE::distance_cv_points(points_in_circle,des_cen) < (des_r)){
                                
                    test.push_back(points_in_circle);
                    double d = double(ptr[j])/1000;
                    total_depth.push_back(d);
                    cout<<"present point depth:"<< d <<endl;
                }
            }
        }
      
        double sum = accumulate(total_depth.begin(), total_depth.end() , 0.0);  
        Z =  sum / total_depth.size(); //均值      
        cout << "distance" << Z <<endl;//打印深度信息

        cv::Mat mask= depth_pic.clone();
        cv::inRange(mask, 0, 255, mask);//？
        cv::circle(mask,des_cen,des_r,cv::Scalar(255) ,3);//画圆    
        cv::imshow("MASK",mask);
        cv::waitKey(10);

        total_depth.clear();
        test.clear();
    
    }

}



void CIRCLE::waypoint_callback(const ros::TimerEvent&){

    if( have_odom_ && have_circle_ ){
        
        cout<<"circle center:"<< des_cen <<endl;
        cout<<"circle radius"<<des_r<<endl;
        cv::Point3d detectedCircleCenter= pixelToWorldCoordinate(); // 将圆心坐标变换到世界坐标系中
        
    //      if (numFrames == 0)
    //         {
    //             targetCircleCenter_cp = detectedCircleCenter;
    //             numFrames++;
    //         }
    //         else if (numFrames < averageFrames)
    //         {
    //             targetCircleCenter_cp += detectedCircleCenter;
    //             numFrames++;
    //         }
    //         else
    //         {
    //             targetCircleCenter_cp /= averageFrames;
    //             CIRCLE::moveTowardsTarget(targetCircleCenter_cp);
    //             ROS_INFO("target circle3:(%f, %f, %f)", targetCircleCenter.x, targetCircleCenter.y, targetCircleCenter.z);
    //             numFrames = 0;
    //         }
    // }
    // else
    // {
    //     ROS_INFO("未检测到圆环");
    //     numFrames = 0;
    // }
    //     if (have_loc)    {
    //     targetCircleCenter=targetCircleCenter_cp;
    //     }
        // des_cen = cv::Point(int(des_circle.center.x),int(des_circle.center.y),double(z));
        // ROS_INFO("target circle3:(%f, %f, %f)", targetCircleCenter.x, targetCircleCenter.y, targetCircleCenter.z);
        targetCircleCenter=detectedCircleCenter;
        point_msg.x = targetCircleCenter.x; // 设置 x 坐标
        point_msg.y =targetCircleCenter.y; // 设置 y 坐标
        point_msg.z =targetCircleCenter.z; // 设置 z 坐标

        // 发布消息
        center_pub.publish(point_msg);

    }

}



cv::Point3d CIRCLE::pixelToWorldCoordinate(){
         
    rotation_matrix = odom_orient_.normalized().toRotationMatrix();     //将四元数转换为旋转矩阵
    camera_inner_matrix <<fx_,0,cx_,
                        0,fy_,cy_,
                        0,0,1;              //定义内参矩阵

    camera_inner_matrix_inverse = camera_inner_matrix.inverse();              //求内参矩阵的逆矩阵
    double u = des_cen.x;
    double v = des_cen.y;

    point2D_h << u,v,1;
    point2D_h = Z*point2D_h;
    point2D_h = camera_inner_matrix_inverse*point2D_h;                  //相机坐标系
    Eigen::Matrix3d R_c_b;
    R_c_b << 0,0,1,-1,0,0,0,-1,0;   
    point2D_h = R_c_b*point2D_h;                           //相机坐标系转机体坐标
    position_list << odom_pos_[0],odom_pos_[1],odom_pos_[2];                //平移矩阵
    Circle_loc_in_World = rotation_matrix*point2D_h + position_list;   //世界坐标系
    worldPoint.x=Circle_loc_in_World(0);
    worldPoint.y=Circle_loc_in_World(1);
    worldPoint.z=Circle_loc_in_World(2);
    

    return worldPoint;
  }



void CIRCLE::moveTowardsTarget(const cv::Point3d& currentCircleCenter){
    double error = cv::norm(currentCircleCenter - targetCircleCenter_cp);
    // 判断当前圆心是否在目标范围内
    if (error <= maxError){
        
        have_loc=true;
        targetCircleCenter_cp = currentCircleCenter;
        ROS_INFO("目标圆环检测成功，圆心坐标：(%f, %f, %f)", targetCircleCenter.x, targetCircleCenter.y, targetCircleCenter.z);
    }
    else{

        have_loc=false;
        ROS_INFO("当前未检测到目标圆环");
    
    }
}





inline double CIRCLE::distance_cv_points(const cv::Point &a , const cv::Point &b){

    return sqrt(pow(abs(a.x-b.x),2) + pow(abs(a.y-b.y),2));//a b两点之间的欧氏距离
}
 inline bool CIRCLE::cv_point_cmp(const distance_i_j &a , const distance_i_j &b){
    
    return a.distance < b.distance;
}







// void CIRCLE::odom_Callback(const nav_msgs::Odometry::ConstPtr &msg)
// {
//     odom_pos_(0) = msg->pose.pose.position.x;
//     odom_pos_(1) = msg->pose.pose.position.y;
//     odom_pos_(2) = msg->pose.pose.position.z;

//     odom_vel_(0) = msg->twist.twist.linear.x;
//     odom_vel_(1) = msg->twist.twist.linear.y;
//     odom_vel_(2) = msg->twist.twist.linear.z;

//     odom_orient_.w() = msg->pose.pose.orientation.w;
//     odom_orient_.x() = msg->pose.pose.orientation.x;
//     odom_orient_.y() = msg->pose.pose.orientation.y;
//     odom_orient_.z() = msg->pose.pose.orientation.z;

//     have_odom_ = true;
// }


void CIRCLE::timerCallback(const ros::TimerEvent&){
    odom_pos_(0) = 0;
    odom_pos_(1) = 0;
    odom_pos_(2) = 0;

    odom_vel_(0) = 0;
    odom_vel_(1) = 0;
    odom_vel_(2) = 0;
   

    odom_orient_.w() =1;
    odom_orient_.x() = 0;
    odom_orient_.y() = 0;
    odom_orient_.z() = 0;
    have_odom_ = true;
}
