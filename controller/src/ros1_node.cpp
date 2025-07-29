#include <geometry_msgs/PoseStamped.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>
#include <mavros_msgs/PositionTarget.h>
#include <mavros_msgs/AttitudeTarget.h>
#include <geometry_msgs/PointStamped.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/TwistStamped.h>
#include <sensor_msgs/Image.h>
#include <std_msgs/Header.h>

#include <fstream>  // 添加这个头文件来处理文件输出

// #include "/home/nvidia/catkin_ws/devel/include/common_msgs/Obj.h"
// #include "/home/nvidia/catkin_ws/devel/include/common_msgs/Objects.h"
// #include "/home/nvidia/catkin_ws/devel/include/common_msgs/Aruco.h"
// #include "/home/nvidia/catkin_ws/devel/include/common_msgs/MissionState.h"
// #include "/home/nvidia/catkin_ws/devel/include/quadrotor_msgs/PositionCommand.h"
#include <common_msgs/Obj.h>
#include <common_msgs/Objects.h>
#include <common_msgs/Aruco.h>
#include <common_msgs/MissionState.h>
#include <quadrotor_msgs/PositionCommand.h>

#include <ros/ros.h>
#include <ros/timer.h>
#include "std_msgs/String.h"

#include <sensor_msgs/Image.h>
#include <map>
#include <queue>
#include <mutex>

#include <opencv2/core.hpp>
#include <cv_bridge/cv_bridge.h>
#include <pcl/range_image/range_image.h>
#include <pcl/kdtree/kdtree.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/point_cloud.h>
#include <nav_msgs/Odometry.h>

#include <pcl_conversions/pcl_conversions.h>
#include <pcl/filters/extract_indices.h>

#include <pcl/segmentation/extract_clusters.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/features/normal_3d.h>
#include <pcl/keypoints/harris_3d.h>
#include <pcl/keypoints/iss_3d.h>
#include <climits>

#include <tf2/transform_datatypes.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

// #include "behavior_tree_ros/BaseCmd.h"
// #include "behavior_tree_ros/FeedBack.h"

#include <behaviortree_cpp/bt_factory.h>
// * nodes
#include <behaviortree_cpp/action_node.h>
#include <behaviortree_cpp/condition_node.h>
#include <behaviortree_cpp/control_node.h>
#include <behaviortree_cpp/decorator_node.h>


#include <opencv2/highgui.hpp>
#include <cmath>
#include <iomanip>
#include <pcl/point_types.h>

using namespace BT;

mavros_msgs::State current_state;
geometry_msgs::PoseStamped fcu_pose;
geometry_msgs::PoseStamped uav_pose;
common_msgs::Objects lzj_objs;
common_msgs::Objects forward_objs;
common_msgs::Obj forward_obj;
common_msgs::Obj down_obj;
common_msgs::Obj down_obj1;
common_msgs::Objects down_objs;
common_msgs::Aruco aruco;

geometry_msgs::PoseStamped goal_point;
geometry_msgs::PoseStamped send_local_pose;

geometry_msgs::PoseStamped goal_1;

mavros_msgs::PositionTarget cmd;
ros::Time last_timestamp;
mavros_msgs::SetMode offb_set_mode;
mavros_msgs::AttitudeTarget att_thrust;
mavros_msgs::CommandBool arm_cmd;
// mavros_msgs::State current_state;
mavros_msgs::SetMode       land;
mavros_msgs::State         fcu_state;
// mavros_msgs::SetMode offb_set_mode;

// ros::Rate rate();
// std::string bt_tree_path = "/mnt/e/linzejun01/catkin_ws/src/ros1_node/mav_baseaction.xml";
// std::string bt_tree_path = "/mnt/e/linzejun01/catkin_ws/src/ros1_node/tree.xml";
// std::string bt_tree_path = "/mnt/e/linzejun01/catkin_ws/src/ros1_node/mav_baseaction2.xml";
// std::string bt_tree_path = "/mnt/e/linzejun01/catkin_ws/src/ros1_node/mav_baseaction_detframe.xml";
//std::string bt_tree_path = "/home/nvidia/catkin_ws/mav_baseaction_detframe.xml";
// std::string bt_tree_path = "/mnt/e/linzejun01/BehaviorTree.CPP-master/BTbaseaction_MavrosCtrlC++/e1_CompSlamNav/Ubuntu/src/Challege_ROS/controller/config/mav_baseaction_detframe2.xml";

ros::Subscriber state_sub;
ros::Publisher cmd_pub; //
ros::Subscriber pose_usb;
ros::ServiceClient arming_client;
ros::ServiceClient set_mode_client;
ros::Publisher att_pub;
ros::Publisher  vis_pub;
ros::Publisher  full_points_pub;
ros::Publisher  goal_pub;
ros::Publisher  pose_pub;

ros::Subscriber forward_obj_sub;
ros::Subscriber forward_objs_sub;

ros::Subscriber down_obj_sub;
ros::Subscriber down_objs_sub;

ros::Subscriber aruco_sub;
ros::Subscriber depth_img_sub;

ros::Subscriber velocity_sub;
ros::Subscriber fcu_state_sub;
ros::Subscriber fcu_pose_sub;
ros::Subscriber pose_sub;

ros::Subscriber odom_sub;
ros::Subscriber tra_sub;

std::ofstream log_file_;   // 日志文件对象


double vx;
double vy;
double vz;

// 角速度 (angular velocity)
double wx;
double wy;
double wz;


// kx = 0.;
// ky = 0.;

bool                       is_recv_state = false;
bool                       is_recv_pose = false;
double                     kx = 0.;
double                     ky = 0.;
double                     vx_max;
double                     vy_max;
double                     takeoff_yaw;
bool                       is_aruco = false;
bool                       is_frame = false;
bool                       is_H = false;
bool                       auto_arming;
double                     takeoff_h;
double                     current_yaw; 


std::queue<sensor_msgs::Image> depth_queue;
std::mutex depth_mtx;


struct Position3D 
{ 
  double x;
  double y; 
  double z; 
};

struct Dxy 
{ 
  double dx;
  double dy; 
};


struct Distance 
{ 
  double x;
};

struct Detect2D_Output 
{ 
  char class_name;
};


typedef struct rflysim
{
    bool enable;
    int  rgb_image_width;
    int  rgb_image_height;
    int  depth_image_width;
    int  depth_image_height;

    double rgb_fov_h;  //相机的两个视场角
    double rgb_fov_v;

    double f_rgb;
    double f_depth;

    double hight_max;

    cv::Point2d rgb_cnt;
    cv::Point2d depth_cnt;

    int                 depth_down_sample;
    std::vector<double> depth_cam2body_R;
    std::vector<double> depth_cam2body_T;
    double              goal_x_t;
    double              min_score;
    bool                is_sim;
    bool is_S; //飞机的轨迹是S型还是反S型，需要判断第一个框和第二框的相对位置
} RflySimParam;

RflySimParam  rflysim_p;

struct ObjData {
    std::string class_name;  // 类别名称
    int class_id;            // 类别 ID
    int left_top_x;          // 左上角 X 坐标
    int left_top_y;          // 左上角 Y 坐标
    int right_bottom_x;      // 右下角 X 坐标
    int right_bottom_y;      // 右下角 Y 坐标
    geometry_msgs::Point coordinates;//坐标
    float score;             // 置信度
    
    // 可扩展的 vector
    std::vector<std::string> other;  // 其他信息

};

struct ObjsData3D {
    std::string class_name;  // 类别名称
    int class_id;            // 类别 ID
    double x;
    double y; 
    double z; //坐标
    float score;             // 置信度
    
    // 可扩展的 vector
    std::vector<std::string> other;  // 其他信息

};

typedef struct cameras
{ 
    /*暂时不做实现，后续可能改用eigen 存储*/
    //    camera() {}
    std::vector<double> rgb_K;        // rgb 相机内参矩阵
    std::vector<double> depth_K;      //深度相机内参矩阵
    std::vector<double> rgb2depth_R;  // rgb相机到深度相机的旋转矩阵
    std::vector<double> rgb2depth_T;  // rgb相机到深度相机的平移矩阵

} CamearasParam;

CamearasParam cam;

struct ArucoData {
    std_msgs::Header header;
    int id;            // 类别 ID
    uint cnt_x;
    uint cnt_y;
    int dx;
    int dy;

    float position_x;
    float position_y;
    float position_z;

    float r_x;
    float r_y;
    float r_z;

    float score;             // 置信度
};

double rad2deg(double rad){
    return rad*(180.0/M_PI);
}

bool compare_angles(double angle1_rad, double angle2_rad);

// Template specialization to converts a string to Position3D.
namespace BT
{
    template <> inline Position3D convertFromString(StringView str)
    {
      // We expect real numbers separated by semicolons
      auto parts = splitString(str, ';');
      if (parts.size() != 3)
      {
          throw RuntimeError("invalid input)");
      }
      else
      {
          Position3D output;
          output.x     = convertFromString<double>(parts[0]);
          output.y     = convertFromString<double>(parts[1]);
          output.z     = convertFromString<double>(parts[2]);
          return output;
      }
    }


    template <> inline Distance convertFromString(StringView str)
    {
      // We expect real numbers separated by semicolons
      if (str.find(';') != StringView::npos) 
      {
          throw RuntimeError("Invalid input: expected a single number without semicolons");
      }
      else
      {
          Distance output;
          output.x     = convertFromString<double>(str);
          return output;
      }
    }
} // end namespace BT



void velocityCallback(const geometry_msgs::TwistStamped::ConstPtr &msg) {
    // 线速度 (linear velocity)
    vx = msg->twist.linear.x;
    vy = msg->twist.linear.y;
    vz = msg->twist.linear.z;

    // 角速度 (angular velocity)
    wx = msg->twist.angular.x;
    wy = msg->twist.angular.y;
    wz = msg->twist.angular.z;
}



void state_cb(const mavros_msgs::State::ConstPtr &msg){
    current_state = *msg;
}

void PoseCB(const geometry_msgs::PoseStamped::ConstPtr &msg)
{
    uav_pose = *msg;
    tf2::Quaternion q(
        msg->pose.orientation.x,
        msg->pose.orientation.y,
        msg->pose.orientation.z,
        msg->pose.orientation.w);
    tf2::Matrix3x3 m(q);
    double roll, pitch;
    m.getRPY(roll, pitch, current_yaw);  // 获取当前的yaw角
}

void RecvPose(const nav_msgs::Odometry::ConstPtr &msg)
{
    fcu_pose.pose   = msg->pose.pose;
    fcu_pose.header = msg->header;
    is_recv_pose    = true;
}

// void poseCallback(const geometry_msgs::PoseStamped::ConstPtr& msg)
// {
//     tf2::Quaternion q(
//         msg->pose.orientation.x,
//         msg->pose.orientation.y,
//         msg->pose.orientation.z,
//         msg->pose.orientation.w);
//     tf2::Matrix3x3 m(q);
//     double roll, pitch;
//     m.getRPY(roll, pitch, current_yaw);  // 获取当前的yaw角
// }



void RecvForwardObjCB(const common_msgs::Obj::ConstPtr &obj)
{  
  forward_obj = *obj;
}

void RecvDownObjCB(const common_msgs::Obj::ConstPtr &obj)
{  
  down_obj = *obj;
}

void RecvAruco(const common_msgs::Aruco::ConstPtr &msg)
{
  is_aruco = true;
  aruco = *msg;
}

void RecvForwardObjectsCB(const common_msgs::Objects::ConstPtr &objs)
{  
  is_frame = true;
  forward_objs = *objs;
}

void RecvDownObjectsCB(const common_msgs::Objects::ConstPtr &objs)
{  
    down_objs = *objs;
    if(objs->objects.size() == 1 && objs->objects[0].score > rflysim_p.min_score
        && objs->objects[0].class_name == "land")
    {
        is_H = true;
        down_obj1 = objs->objects[0];
    }
}

void RecvFcuState(const mavros_msgs::State::ConstPtr &state)
{
  fcu_state     = *state;
  is_recv_state = true;
}

void RecvLIO(const nav_msgs::Odometry::ConstPtr &odom)
{
  send_local_pose.pose.position.x = odom->pose.pose.position.y;
  send_local_pose.pose.position.y = -odom->pose.pose.position.x;
  send_local_pose.pose.position.z = odom->pose.pose.position.z;

  tf2::Quaternion q;
  tf2::fromMsg(odom->pose.pose.orientation, q);
  tf2::Matrix3x3 att(q);
  double         roll, pitch, yaw;
  att.getRPY(roll, pitch, yaw);
  yaw += 1.5707;
  tf2::Quaternion q_;
  q_.setRPY(roll, pitch, yaw);

  // send_local_pose.pose.orientation = odom->pose.pose.orientation;
  send_local_pose.pose.orientation = tf2::toMsg(q_);

  send_local_pose.header.frame_id = "map";
  send_local_pose.header.stamp    = ros::Time::now();

  pose_pub.publish(send_local_pose);
  ros::spinOnce();
}


void DepthImgCB(const sensor_msgs::Image::ConstPtr &depth)
{
  if(!rflysim_p.is_sim)
  {
    return;
  }
  std::unique_lock<std::mutex> lock(depth_mtx);
  depth_queue.push(*depth);
  while(true)
  {
    auto front = depth_queue.front();
    if(ros::Time::now() - front.header.stamp > ros::Duration(1))
    {  //把超过一秒外的数据丢弃
      depth_queue.pop();
    }
    else
    {
      return;
    }
  }
}

void RecvTra(const quadrotor_msgs::PositionCommand::ConstPtr &msg)
{
  //使用直接赋值，默认认为给飞控的odom数据与给ego-planner的数据是同一坐标系的
  cmd.position.x   = msg->position.x;
  cmd.position.y   = msg->position.y;
  cmd.position.z   = msg->position.z;
  cmd.yaw          = takeoff_yaw;
  cmd.header.stamp = msg->header.stamp;
}


void ObjsConterRGB(const common_msgs::Objects objs, Position3D &cnt)
{  
    //该接口目标中心计算比较粗糙，直接使用目标检测框（这就要求标注的时候尽可能的贴着边缘标注），严格上来讲，应该对目标款内图像做角点检测，然后筛选出目标的实际目标点位置

  static double scale_ = rflysim_p.rgb_image_width / rflysim_p.depth_image_height;

  if(objs.objects.empty())
    return;

  for(int i = 0; i < objs.objects.size(); ++i)
  {  
    //可能会检测出多个目标，刷选的方方法有的有很多，例如通过score
     //过滤,，然后通过框的相对位置，比如第一个框一定在第二个框的右边等等
    auto &obj = objs.objects[i];
    if(obj.score < rflysim_p.min_score)
      continue;

    auto width  = obj.right_bottom_x - obj.left_top_x;
    auto height = obj.right_bottom_y - obj.left_top_y;

    auto ret = width / height;
    if(ret - scale_ > 0.5)
    {
      //如果比大很多，那可以判断这个目标不是我们需要的目标；
      continue;
    }

    //计算传感器的宽度和高度，unit:mm
    static double s_w = 2 * rflysim_p.f_rgb * std::tan(rflysim_p.rgb_fov_h / 2);
    static double s_h = 2 * rflysim_p.f_rgb * std::tan(rflysim_p.rgb_fov_v / 2);
    static double pw  = s_w / rflysim_p.rgb_image_width;   //像素的宽度
    static double ph  = s_h / rflysim_p.rgb_image_height;  //像素的高度
    static double wf = 1.3 * rflysim_p.f_rgb;

    //目标宽，与高比值1：1，1.3m, 默认目标框的高度是没有遮挡的，
    //先还原如果目标框没有被遮挡，应该在图像上的什么位置；
    int left_or_right = (obj.left_top_x + obj.right_bottom_x)/2 - rflysim_p.rgb_cnt.x;
    int det_x = 0;
    if(ret < scale_)
    {
      det_x = int(height * scale_ - width);
      // if(obj.left_top_x - det_x < 0)
      // {  // 目标根据场景布置，目标只有可能被左边的柱子遮挡，如果场景移动了，那就是右边,
      //   //所以这种情况是月边界不再图像内
      //   continue;
      // }
      
      if(rflysim_p.is_S)
        ROS_INFO("l_or_r: %d",left_or_right);
      if(!rflysim_p.is_S  && left_or_right > 0 || rflysim_p.is_S && left_or_right < 0 
      )
      { // 通过当前飞机正在执行的任务与飞机的轨迹形状，再接目标检测的结果判断，该目标是否有效
        continue;  
      }
        width +=  det_x;

    }
    //根据焦距计算相机坐标系的宽中心点位置（x,y,z）
    double        z  = wf / width;
    
    //需要考虑遮挡情况
    double cx = (obj.right_bottom_x + (obj.left_top_x - det_x)) / 2;
    double cy = (obj.right_bottom_y + obj.left_top_y) / 2;
    cx = ((obj.right_bottom_x + det_x) + obj.left_top_x) / 2;

     
    double xc = (cx - rflysim_p.rgb_cnt.x) * pw;
    double yc = (cy - rflysim_p.rgb_cnt.y) * ph;
    double x  = xc * z / rflysim_p.f_rgb;
    double y  = yc * z / rflysim_p.f_rgb;
    //至此，一致计算得到目标宽中心位置在相机坐标系里的位置了；

    cnt.x = x;
    cnt.y = y;
    cnt.z = z;

    //    pcl::PointXYZ p;
    //    p.x = x;
    //    p.y = y;
    //    p.z = z;
    //    VisionPointCloud(&p, vis_pub);  // 结合深度图像看看位置是否准确
  }
}

void ObjsConterRGB(const common_msgs::Objects objs, pcl::PointXYZ *cnt, ObjsData3D &objsdata3D )
{  
    //该接口目标中心计算比较粗糙，直接使用目标检测框（这就要求标注的时候尽可能的贴着边缘标定），严格上来讲，应该对目标款内图像做角点检测，然后筛选出目标的实际目标点位置

  static double scale_ =
    rflysim_p.rgb_image_width / rflysim_p.depth_image_height;

  if(objs.objects.empty())
    return;

  for(int i = 0; i < objs.objects.size(); ++i)
  {  //可能会检测出多个目标，刷选的方方法有的有很多，例如通过score
     //过滤,，然后通过框的相对位置，比如第一个框一定在第二个框的右边等等
    auto &obj = objs.objects[i];
    if(obj.score < rflysim_p.min_score)
      continue;

    auto width  = obj.right_bottom_x - obj.left_top_x;
    auto height = obj.right_bottom_y - obj.left_top_y;

    auto ret = width / height;
    if(ret - scale_ > 0.5)
    {
      //如果比大很多，那可以判断这个目标不是我们需要的目标；
      continue;
    }

    //计算传感器的宽度和高度，unit:mm
    static double s_w = 2 * rflysim_p.f_rgb * std::tan(rflysim_p.rgb_fov_h / 2);
    static double s_h = 2 * rflysim_p.f_rgb * std::tan(rflysim_p.rgb_fov_v / 2);
    static double pw  = s_w / rflysim_p.rgb_image_width;   //像素的宽度
    static double ph  = s_h / rflysim_p.rgb_image_height;  //像素的高度
    static double wf = 1.3 * rflysim_p.f_rgb;

    //目标宽，与高比值1：1，1.3m, 默认目标框的高度是没有遮挡的，
    //先还原如果目标框没有被遮挡，应该在图像上的什么位置；
    int left_or_right = (obj.left_top_x + obj.right_bottom_x)/2 - rflysim_p.rgb_cnt.x;
    int det_x = 0;
    if(ret < scale_)
    {
      det_x = int(height * scale_ - width);
      // if(obj.left_top_x - det_x < 0)
      // {  // 目标根据场景布置，目标只有可能被左边的柱子遮挡，如果场景移动了，那就是右边,
      //   //所以这种情况是月边界不再图像内
      //   continue;
      // }
      
      if(rflysim_p.is_S)
        ROS_INFO("l_or_r: %d",left_or_right);
      if(!rflysim_p.is_S && left_or_right > 0 || rflysim_p.is_S && left_or_right < 0 
      )
      { // 通过当前飞机正在执行的任务与飞机的轨迹形状，再接目标检测的结果判断，该目标是否有效
        continue;  
      }
        width +=  det_x;

    }
    //根据焦距计算相机坐标系的宽中心点位置（x,y,z）
    double        z  = wf / width;
    
    //需要考虑遮挡情况
    double cx = (obj.right_bottom_x + (obj.left_top_x - det_x)) / 2;
    double cy = (obj.right_bottom_y + obj.left_top_y) / 2;
    cx = ((obj.right_bottom_x + det_x) + obj.left_top_x) / 2;
     
    double xc = (cx - rflysim_p.rgb_cnt.x) * pw;
    double yc = (cy - rflysim_p.rgb_cnt.y) * ph;
    double x  = xc * z / rflysim_p.f_rgb;
    double y  = yc * z / rflysim_p.f_rgb;
    //至此，一致计算得到目标宽中心位置在相机坐标系里的位置了；

    cnt->x = x;
    cnt->y = y;
    cnt->z = z;
    objsdata3D.class_name = obj.class_name;
    objsdata3D.score = obj.score;
    objsdata3D.class_id = i;
  }
}



void ObjConterRGB(const common_msgs::Obj obj, Position3D &cnt)
{  
    //该接口目标中心计算比较粗糙，直接使用目标检测框（这就要求标注的时候尽可能的贴着边缘标注），严格上来讲，应该对目标款内图像做角点检测，然后筛选出目标的实际目标点位置

    static double scale_ = rflysim_p.rgb_image_width / rflysim_p.depth_image_height;



    auto width  = obj.right_bottom_x - obj.left_top_x;
    auto height = obj.right_bottom_y - obj.left_top_y;

    auto ret = width / height;

    //计算传感器的宽度和高度，unit:mm
    static double s_w = 2 * rflysim_p.f_rgb * std::tan(rflysim_p.rgb_fov_h / 2);
    static double s_h = 2 * rflysim_p.f_rgb * std::tan(rflysim_p.rgb_fov_v / 2);
    static double pw  = s_w / rflysim_p.rgb_image_width;   //像素的宽度
    static double ph  = s_h / rflysim_p.rgb_image_height;  //像素的高度
    static double wf = 1.3 * rflysim_p.f_rgb;

    //目标宽，与高比值1：1，1.3m, 默认目标框的高度是没有遮挡的，
    //先还原如果目标框没有被遮挡，应该在图像上的什么位置；
    int left_or_right = (obj.left_top_x + obj.right_bottom_x)/2 - rflysim_p.rgb_cnt.x;
    int det_x = 0;
    if(ret < scale_)
    {
        det_x = int(height * scale_ - width);
        // if(obj.left_top_x - det_x < 0)
        // {  // 目标根据场景布置，目标只有可能被左边的柱子遮挡，如果场景移动了，那就是右边,
        //   //所以这种情况是月边界不再图像内
        //   continue;
        // }
        
        if(rflysim_p.is_S)
        ROS_INFO("l_or_r: %d",left_or_right);
        width +=  det_x;

    }
    //根据焦距计算相机坐标系的宽中心点位置（x,y,z）
    double        z  = wf / width;

    //需要考虑遮挡情况
    double cx = (obj.right_bottom_x + (obj.left_top_x - det_x)) / 2;
    double cy = (obj.right_bottom_y + obj.left_top_y) / 2;
    cx = ((obj.right_bottom_x + det_x) + obj.left_top_x) / 2;

        
    double xc = (cx - rflysim_p.rgb_cnt.x) * pw;
    double yc = (cy - rflysim_p.rgb_cnt.y) * ph;
    double x  = xc * z / rflysim_p.f_rgb;
    double y  = yc * z / rflysim_p.f_rgb;
    //至此，一致计算得到目标宽中心位置在相机坐标系里的位置了；

    cnt.x = x;
    cnt.y = y;
    cnt.z = z;

    //    pcl::PointXYZ p;
    //    p.x = x;
    //    p.y = y;
    //    p.z = z;
    //    VisionPointCloud(&p, vis_pub);  // 结合深度图像看看位置是否准确

}


void VisionPointCloud(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, const ros::Publisher &pub)
{
  sensor_msgs::PointCloud2 data;
  pcl::toROSMsg(*cloud, data);
  data.header.frame_id = "map";
  pub.publish(data);
  ros::spinOnce();
}

void VisionPointCloud(const pcl::PointXYZ * p, const ros::Publisher &pub)
{
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(
    new pcl::PointCloud<pcl::PointXYZ>());
  cloud->points.push_back(*p);
  VisionPointCloud(cloud, pub);
}



// void CoordinateTrans(point3D &cloud)
// {
//     std::vector<double> R = rflysim_p.depth_cam2body_R;
//     std::vector<double> T = rflysim_p.depth_cam2body_T;
//     //        tf2::Vector3        bc(x, y, z);
//     tf2::Quaternion q;
//     tf2::convert(fcu_pose.pose.orientation, q);
//     tf2::Vector3 t;
//     tf2::convert(fcu_pose.pose.position, t);

//     tf2::Transform trans;
//     trans.setOrigin(t);
//     trans.setRotation(q);

//     auto         p = cloud;
//     double       x = p.x * R[0] + p.y * R[1] + p.z * R[2] + T[0];
//     double       y = p.x * R[3] + p.y * R[4] + p.z * R[5] + T[1];
//     double       z = p.x * R[6] + p.y * R[7] + p.z * R[8] + T[2];
//     tf2::Vector3 b_p(x, y, z);

//     tf2::Vector3 w_p = trans * b_p;
//     cloud.x = w_p.getX();
//     cloud.y = w_p.getY();
//     cloud.z = w_p.getZ();

// }

void CoordinateTrans(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud)
{
  std::vector<double> R = rflysim_p.depth_cam2body_R;
  std::vector<double> T = rflysim_p.depth_cam2body_T;
  //        tf2::Vector3        bc(x, y, z);
  tf2::Quaternion q;
  tf2::convert(fcu_pose.pose.orientation, q);
  tf2::Vector3 t;
  tf2::convert(fcu_pose.pose.position, t);

  tf2::Transform trans;
  trans.setOrigin(t);
  trans.setRotation(q);

  for(size_t i = 0; i < cloud->points.size(); ++i)
  {
    auto         p = cloud->points[i];
    double       x = p.x * R[0] + p.y * R[1] + p.z * R[2] + T[0];
    double       y = p.x * R[3] + p.y * R[4] + p.z * R[5] + T[1];
    double       z = p.x * R[6] + p.y * R[7] + p.z * R[8] + T[2];
    tf2::Vector3 b_p(x, y, z);

    tf2::Vector3 w_p = trans * b_p;
    cloud->points[i].x = w_p.getX();
    cloud->points[i].y = w_p.getY();
    cloud->points[i].z = w_p.getZ();
  }
  //  std::cout << "xxxxxx: " << tmp->points.size() << std::endl;
}





void DepthImgToCloud(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, const cv::Mat *img, const cv::Point2i &left, const cv::Point2i &right)
{
  //我们需要获得相机坐标系下的点云坐标，等计算中心点，只需要转换一个点坐标即可
  cloud->is_dense = false;
  //  cloud->points.resize(cloud->width * cloud->height);
  //  cv::imshow("depth_frame", *img);
  //  cv::waitKey(0);
  int           dx = left.x - rflysim_p.depth_cnt.x;
  int           dy = left.y - rflysim_p.depth_cnt.y;
  pcl::PointXYZ point;
  for(int row = 0; row < img->rows; row += rflysim_p.depth_down_sample)
  {
    for(int col = 0; col < img->cols; col += rflysim_p.depth_down_sample)
    {
      float depth =
        img->at<uint16_t>(row, col) * 0.001f;  // rflysim 精度为0.001
      if(depth > 0 && depth < 7)  //目标不能在7米外，减少计算量
      {
        point.z = depth;
        point.x = (col + dx) * depth / rflysim_p.f_depth;
        point.y = (row + dy) * depth / rflysim_p.f_depth;

        cloud->points.push_back(point);
      }
    }
  }
}

bool Cluster(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, cv::Point3d *cnt)
{
  pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(
    new pcl::search::KdTree<pcl::PointXYZ>);
  tree->setInputCloud(cloud);
  std::vector<pcl::PointIndices>                 cluster_indices;
  pcl::EuclideanClusterExtraction<pcl::PointXYZ> ec;
  ec.setClusterTolerance(0.4);  // 设置聚类的欧几里得距离阈值为 50cm
  ec.setMinClusterSize(10);     // 设置一个聚类需要的最小点数
  ec.setMaxClusterSize(10000);  // 设置一个聚类需要的最大点数
  ec.setSearchMethod(tree);
  ec.setInputCloud(cloud);
  // 提取聚类的索引（以点云簇的形式返回）
  ec.extract(cluster_indices);
  double obs_dist = FLT_MAX;
  std::cout << "Number of clusters: " << cluster_indices.size() << std::endl;
  // 输出每个聚类的点数和中心点
  pcl::PointCloud<pcl::PointXYZ>::Ptr ret_pc(
    new pcl::PointCloud<pcl::PointXYZ>);
  for(std::vector<pcl::PointIndices>::const_iterator it =
        cluster_indices.begin();
      it != cluster_indices.end(); ++it)
  {  
    //我们需要对目标应该是点最多的，还可以进一步求出聚类点角点,然后求里飞机最近的目标点
    pcl::PointCloud<pcl::PointXYZ>::Ptr cluster(
      new pcl::PointCloud<pcl::PointXYZ>);
    auto min_x = FLT_MAX;
    auto max_x = FLT_MIN;
    auto min_y = FLT_MAX;
    auto max_y = FLT_MIN;
    for(std::vector<int>::const_iterator pit = it->indices.begin();
        pit != it->indices.end(); ++pit)
    {
      if(min_x > cloud->points[*pit].x)
        min_x = cloud->points[*pit].x;
      if(max_x < cloud->points[*pit].x)
        max_x = cloud->points[*pit].x;
      if(min_y > cloud->points[*pit].y)
        min_y = cloud->points[*pit].y;
      if(max_y < cloud->points[*pit].y)
        max_y = cloud->points[*pit].y;
      cluster->points.push_back(
        cloud->points[*pit]);  // 将聚类中的点添加到点云中
    }

    auto width  = max_x - min_x;
    auto height = max_y - min_y;
    //    std::cout << " cloud w: " << width << " , h:" << height << std::endl;
    //    if(abs(width - 1.3) > 0.5 || abs(height - 1.3) > 0.5)
    //    {  //计算框的宽和高，以此过滤不符合要求的目标
    //      continue;
    //    }

    cluster->width    = cluster->points.size() + 1;
    cluster->height   = 1;
    cluster->is_dense = true;

    std::cout << "Cluster size: " << cluster->size() << std::endl;
    //    std::cout << "Cluster center: " << std::endl;
    Eigen::Vector4f centroid;
    auto            ret = pcl::compute3DCentroid(*cluster, centroid);
    if(ret == 0)
    {
      continue;
    }
    //    std::cout << "x: " << centroid[0] << ", y: " << centroid[1]
    //              << ", z: " << centroid[2] << std::endl;

    if(centroid[2] < obs_dist)
    {  //最后，距离最近的目标将被选出
      obs_dist = centroid[2];
      cnt->x   = centroid[0];
      cnt->y   = centroid[1];
      cnt->z   = centroid[2];
      pcl::PointXYZ p;
      p.x = cnt->x;
      p.y = cnt->y;
      p.z = cnt->z;
      cluster->points.push_back(p);
      ret_pc->points.swap(cluster->points);
    }
    //    VisionPointCloud(cluster);
    // ret_pc->points.insert(ret_pc->points.begin() +
    // ret_pc->points.size(),cluster->points.begin(),cluster->points.end());
  }
  //  VisionPointCloud(ret_pc);
  cloud->points.swap(ret_pc->points);
  if(obs_dist > 9)
  {
    ROS_ERROR("not cluster objects");
    return false;
  }
  return true;
}


class TakeOff: public SyncActionNode
{
  public:
    TakeOff(const std::string& name, const NodeConfig& config):
        SyncActionNode(name,config)
    {}

    static PortsList providedPorts()
    {
      // Optionally, a port can have a human readable description
      const char*  description = "Takeoff position...";
      return { InputPort<Position3D>("takeoff_target", description) };
    }
      
    NodeStatus tick() override
    {
      auto res = getInput<Position3D>("takeoff_target");
      if( !res )
      {
        throw RuntimeError("error reading port [takeoff_target]:", res.error());
      }
      ros::Rate rate(20.0);
      Position3D target = res.value();
      printf("takeoff_Target positions: [ %.1f, %.1f ,%.1f]\n", target.x, target.y, target.z );
      cmd.coordinate_frame = mavros_msgs::PositionTarget::FRAME_LOCAL_NED; // 选择控制坐标系，位置，速度，加速度使用local坐标系，姿态使用的是body坐标系
      cmd.type_mask = ~uint16_t(0);
      cmd.type_mask &= ~mavros_msgs::PositionTarget::FORCE; //px4 不响应力的控制方式
      cmd.type_mask &= ~mavros_msgs::PositionTarget::IGNORE_PX;
      cmd.type_mask &= ~mavros_msgs::PositionTarget::IGNORE_PY;
      cmd.type_mask &= ~mavros_msgs::PositionTarget::IGNORE_PZ;
      cmd.position.x = target.x;
      cmd.position.y = target.y;
      cmd.position.z = target.z;
      cmd.header.stamp = ros::Time::now();

      // mavros_msgs::AttitudeTarget att_thrust;
      // att_thrust.type_mask = ~uint8_t(0);

      for(int i = 100; ros::ok() && i > 0; --i){
          //发送目标指令，以便飞控切换offboard状态，这里可以发位置，速度，加速度，一般情况起飞，都是发送位置
          cmd_pub.publish(cmd);
          ros::spinOnce();
          rate.sleep();
      }

      // mavros_msgs::SetMode offb_set_mode;
      offb_set_mode.request.custom_mode = "OFFBOARD";

      // mavros_msgs::CommandBool arm_cmd;
      arm_cmd.request.value = true;

      ros::Time last_request = ros::Time::now();
      bool is_takeoff= false;
      bool is_attitude = false;
      while(ros::ok()){
          //切换offboard模式
          if( current_state.mode != "OFFBOARD" &&
              (ros::Time::now() - last_request > ros::Duration(5.0)) && !is_takeoff){
              if( set_mode_client.call(offb_set_mode) &&
                  offb_set_mode.response.mode_sent){
                  ROS_INFO("Offboard enabled");
              }
              last_request = ros::Time::now();
          } else if(!is_takeoff) {
              //解锁飞控
              if( !current_state.armed &&
                  (ros::Time::now() - last_request > ros::Duration(5.0)) ){
                  if( arming_client.call(arm_cmd) &&
                      arm_cmd.response.success){
                      ROS_INFO("Vehicle armed");
                  }
                  last_request = ros::Time::now();
              }
          }
          if(abs(fcu_pose.pose.position.z - target.z) < 0.1 && !is_takeoff)
          {
              ROS_INFO("takeoff finished");
              last_request = ros::Time::now();
              is_takeoff = true;
              return NodeStatus::SUCCESS;
          }
          if(!is_takeoff)
          {
            //起飞就是位置控制
            cmd_pub.publish(cmd);
            ros::spinOnce();
            rate.sleep();
            continue;
          }
          ros::spinOnce();
          rate.sleep();
      }

      return NodeStatus::FAILURE;
    }
};


Position3D flu2nwu(const Position3D& fluPos, double yaw) {
    // Convert yaw from degrees to radians if necessary
    // yaw = yaw * M_PI / 180.0;

    // Rotation matrix for yaw rotation around the up (z) axis
    double cosYaw = cos(yaw);
    double sinYaw = sin(yaw);

    // Apply the rotation
    Position3D nwuPos;
    nwuPos.x = cosYaw * fluPos.x - sinYaw * fluPos.y + fcu_pose.pose.position.x;
    nwuPos.y = sinYaw * fluPos.x + cosYaw * fluPos.y + fcu_pose.pose.position.y;
    nwuPos.z = fluPos.z + fcu_pose.pose.position.z; // The up component remains unchanged

    return nwuPos;
}



class MoveForward : public BT::ActionNodeBase
{
public:
    // 构造函数，传递名称和节点配置
    MoveForward(const std::string& name, const BT::NodeConfiguration& config)
        : BT::ActionNodeBase(name, config), is_moving_(false), is_finished(false)
    { 
        cmd.coordinate_frame = mavros_msgs::PositionTarget::FRAME_LOCAL_NED;
        cmd.type_mask =
            ~uint16_t(0) & ~(uint16_t(0xff) << 12);  //最后结果为0000 1111 1111 1111
        cmd.type_mask &= (~uint16_t(mavros_msgs::PositionTarget::FORCE)); 
        cmd.type_mask &= ~(mavros_msgs::PositionTarget::IGNORE_PX | mavros_msgs::PositionTarget::IGNORE_PY | mavros_msgs::PositionTarget::IGNORE_PZ);         
    }

    // 设置节点的输入端口
    static BT::PortsList providedPorts()
    {
        const char* description = "Simply print the goal on console...";
        return { BT::InputPort<Distance>("MoveForward_distance", description) };
    }

    // 核心逻辑，处理动作的执行
    virtual BT::NodeStatus tick() override
    {
        // 如果动作尚未开始，初始化目标距离
        if (!is_moving_)
        {
            auto res = getInput<Distance>("MoveForward_distance");
            if (!res)
            {
                throw BT::RuntimeError("error reading port [MoveForward_distance]:", res.error());
            }
            Position3D target = Position3D{res.value().x, 0.0, 0.0};
            printf("MoveForward target distance: %.1f\n", target.x);
            log_file_ << "MoveForward target distance: [ " << target.x << " ]\n"; 

            // 设定目标位置

            nwu_target = flu2nwu(target, current_yaw);
            is_moving_ = true;
            ROS_INFO("MoveForward: Moving to forward %.1f,%.1f", nwu_target.x,nwu_target.y);
        }

        // 检查无人机是否达到目标位置
        if (abs(fcu_pose.pose.position.y - nwu_target.y) < 0.1 && abs(fcu_pose.pose.position.x - nwu_target.x) < 0.1 && !is_finished)
        {
            ROS_INFO("MoveForward finished");
            log_file_ << "MoveForward finished. drone pose [nwu,yaw] [ " << fcu_pose.pose.position.x << ", " << fcu_pose.pose.position.y << ", " << fcu_pose.pose.position.z<<", " << rad2deg(current_yaw) << ")\n";
            is_finished = true;
            return BT::NodeStatus::SUCCESS;
        }

        // 如果尚未到达目标，继续移动并返回 RUNNING
        if (!is_finished)
        {
            cmd.position.x = nwu_target.x;
            cmd.position.y = nwu_target.y;
            cmd_pub.publish(cmd);

            ros::spinOnce();
            ros::Rate rate(20.0);
            rate.sleep();
            // log_file_ << "fcu_pose_pose_position: [ " << fcu_pose.pose.position.x << ", " << fcu_pose.pose.position.y << ", " << fcu_pose.pose.position.z << "]\n";
            return BT::NodeStatus::RUNNING;
        }
        log_file_ << "MoveForward FAILURE.\n";
        return BT::NodeStatus::FAILURE;
    }

    // 终止当前动作
    virtual void halt() override
    {
        ROS_INFO("MoveForward halted");
        is_moving_ = false;  // 重置状态
        is_finished = false;

    }

private:   
    bool is_moving_;      // 是否在移动中
    bool is_finished;     // 动作是否完成
    Position3D nwu_target;
};



class MoveBack : public BT::ActionNodeBase
{
public:
    // 构造函数，传递名称和节点配置
    MoveBack(const std::string& name, const BT::NodeConfiguration& config)
        : BT::ActionNodeBase(name, config), is_moving_(false), is_finished(false)
    { 
        cmd.coordinate_frame = mavros_msgs::PositionTarget::FRAME_LOCAL_NED;
        cmd.type_mask =
            ~uint16_t(0) & ~(uint16_t(0xff) << 12);  //最后结果为0000 1111 1111 1111
        cmd.type_mask &= (~uint16_t(mavros_msgs::PositionTarget::FORCE)); 
        cmd.type_mask &= ~(mavros_msgs::PositionTarget::IGNORE_PX | mavros_msgs::PositionTarget::IGNORE_PY | mavros_msgs::PositionTarget::IGNORE_PZ);         
    }

    // 设置节点的输入端口
    static BT::PortsList providedPorts()
    {
        const char* description = "Simply print the goal on console...";
        return { BT::InputPort<Distance>("MoveBack_distance", description) };
    }

    // 核心逻辑，处理动作的执行
    virtual BT::NodeStatus tick() override
    {
        // 如果动作尚未开始，初始化目标距离
        if (!is_moving_)
        {
            auto res = getInput<Distance>("MoveBack_distance");
            if (!res)
            {
                throw BT::RuntimeError("error reading port [MoveBack_distance]:", res.error());
            }
            Position3D target = Position3D{-res.value().x, 0.0, 0.0}; 
            printf("MoveBack positions: [ %.1f]\n", target.x);
            log_file_ << "MoveBack target distance: [ " << target.x << " ]\n"; 

            // 设定目标位置
            nwu_target = flu2nwu(target, current_yaw); 
            is_moving_ = true;
            ROS_INFO("MoveBack: Moving to back %.1f,%.1f", nwu_target.x,nwu_target.y);
        }

        // 检查无人机是否达到目标位置
        if (abs(fcu_pose.pose.position.y - nwu_target.y) < 0.1 && abs(fcu_pose.pose.position.x - nwu_target.x) < 0.1 && !is_finished)
        {
            ROS_INFO("MoveBack finished");
            log_file_ << "MoveBack finished. drone pose [nwu,yaw] [ " << fcu_pose.pose.position.x << ", " << fcu_pose.pose.position.y << ", " << fcu_pose.pose.position.z<<", " << rad2deg(current_yaw) << ")\n";

            is_finished = true;
            return BT::NodeStatus::SUCCESS;
        }

        // 如果尚未到达目标，继续移动并返回 RUNNING
        if (!is_finished)
        {
            cmd.position.x = nwu_target.x;
            cmd.position.y = nwu_target.y;
            cmd_pub.publish(cmd);

            ros::spinOnce();
            ros::Rate rate(20.0);
            rate.sleep();
            return BT::NodeStatus::RUNNING;
        }
        log_file_ << "MoveBack FAILURE.\n";
        return BT::NodeStatus::FAILURE;
    }

    // 终止当前动作
    virtual void halt() override
    {
        ROS_INFO("MoveBack halted");
        log_file_ << "MoveBack halted.\n";
        is_moving_ = false;  // 重置状态
        is_finished = false;
    }

private:   
    bool is_moving_;      // 是否在移动中
    bool is_finished;     // 动作是否完成
    Position3D nwu_target;
};




class MoveRight : public BT::ActionNodeBase
{
public:
    // 构造函数，传递名称和节点配置
    MoveRight(const std::string& name, const BT::NodeConfiguration& config)
        : BT::ActionNodeBase(name, config), is_moving_(false), is_finished(false)
    { 
        cmd.coordinate_frame = mavros_msgs::PositionTarget::FRAME_LOCAL_NED;
        cmd.type_mask =
            ~uint16_t(0) & ~(uint16_t(0xff) << 12);  //最后结果为0000 1111 1111 1111
        cmd.type_mask &= (~uint16_t(mavros_msgs::PositionTarget::FORCE)); 
        cmd.type_mask &= ~(mavros_msgs::PositionTarget::IGNORE_PX | mavros_msgs::PositionTarget::IGNORE_PY | mavros_msgs::PositionTarget::IGNORE_PZ);         
    }

    // 设置节点的输入端口
    static BT::PortsList providedPorts()
    {
        const char* description = "Simply print the goal on console...";
        return { BT::InputPort<Distance>("MoveRight_distance", description) };
    }

    // 核心逻辑，处理动作的执行
    virtual BT::NodeStatus tick() override
    {
        // 如果动作尚未开始，初始化目标距离
        if (!is_moving_)
        {
            auto res = getInput<Distance>("MoveRight_distance");
            if (!res)
            {
                throw BT::RuntimeError("error reading port [MoveRight_distance]:", res.error());
            }
            Position3D target = Position3D{0.0, -res.value().x, 0.0};
            printf("MoveRight positions: [ %.1f]\n", target.x);
            log_file_ << "MoveRight target positions: [ " << target.x << "]\n"; 

            // 设定目标位置
            is_moving_ = true;
            ROS_INFO("MoveRight: Moving to right %.1f,%.1f", nwu_target.x,nwu_target.y);
        }

        // 检查无人机是否达到目标位置
        if (abs(fcu_pose.pose.position.y - nwu_target.y) < 0.1 && abs(fcu_pose.pose.position.x - nwu_target.x) < 0.1 && !is_finished)
        {
            ROS_INFO("MoveRight finished");
            log_file_ << "MoveRight finished. drone pose [nwu,yaw] [ " << fcu_pose.pose.position.x << ", " << fcu_pose.pose.position.y << ", " << fcu_pose.pose.position.z<<", " << rad2deg(current_yaw) << ")\n";
            is_finished = true;
            return BT::NodeStatus::SUCCESS;
        }

        // 如果尚未到达目标，继续移动并返回 RUNNING
        if (!is_finished)
        {
            cmd.position.x = nwu_target.x;
            cmd.position.y = nwu_target.y;
            cmd_pub.publish(cmd);

            ros::spinOnce();
            ros::Rate rate(20.0);
            rate.sleep();
            return BT::NodeStatus::RUNNING;
        }
        log_file_ << "MoveRight FAILURE.\n";
        return BT::NodeStatus::FAILURE;
    }

    // 终止当前动作
    virtual void halt() override
    {
        ROS_INFO("MoveRight halted");
        log_file_ << "MoveRight halted.\n";
        is_moving_ = false;  // 重置状态
        is_finished = false;
    }

private:   
    bool is_moving_;      // 是否在移动中
    bool is_finished;     // 动作是否完成
    Position3D nwu_target;
};



class MoveLeft : public BT::ActionNodeBase
{
public:
    // 构造函数，传递名称和节点配置
    MoveLeft(const std::string& name, const BT::NodeConfiguration& config)
        : BT::ActionNodeBase(name, config), is_moving_(false), is_finished(false)
    { 
        cmd.coordinate_frame = mavros_msgs::PositionTarget::FRAME_LOCAL_NED;
        cmd.type_mask =
            ~uint16_t(0) & ~(uint16_t(0xff) << 12);  //最后结果为0000 1111 1111 1111
        cmd.type_mask &= (~uint16_t(mavros_msgs::PositionTarget::FORCE)); 
        cmd.type_mask &= ~(mavros_msgs::PositionTarget::IGNORE_PX | mavros_msgs::PositionTarget::IGNORE_PY | mavros_msgs::PositionTarget::IGNORE_PZ);         
    }

    // 设置节点的输入端口
    static BT::PortsList providedPorts()
    {
        const char* description = "Simply print the goal on console...";
        return { BT::InputPort<Distance>("MoveLeft_distance", description) };
    }

    // 核心逻辑，处理动作的执行
    virtual BT::NodeStatus tick() override
    {
        // 如果动作尚未开始，初始化目标距离
        if (!is_moving_)
        {
            auto res = getInput<Distance>("MoveLeft_distance");
            if (!res)
            {
                throw BT::RuntimeError("error reading port [MoveLeft_distance]:", res.error());
            }
            Position3D target = Position3D{0.0, res.value().x, 0.0};
            printf("MoveLeft positions: [ %.1f]\n", target.x);
            log_file_ << "MoveLeft target positions: [ " << target.x << "]\n"; 

            // 设定目标位置
            nwu_target = flu2nwu(target, current_yaw);
            is_moving_ = true;
            ROS_INFO("MoveLeft: Moving to left  %.1f, %.1f", nwu_target.x,nwu_target.y);
        }

        // 检查无人机是否达到目标位置
        if (abs(fcu_pose.pose.position.y - nwu_target.y) < 0.1 && abs(fcu_pose.pose.position.x - nwu_target.x) < 0.1 && !is_finished)
        {
            ROS_INFO("MoveLeft finished");
            log_file_ << "MoveLeft finished. drone pose [nwu,yaw] [ " << fcu_pose.pose.position.x << ", " << fcu_pose.pose.position.y << ", " << fcu_pose.pose.position.z<<", " << rad2deg(current_yaw) << ")\n";
            is_finished = true;
            return BT::NodeStatus::SUCCESS;
        }

        // 如果尚未到达目标，继续移动并返回 RUNNING
        if (!is_finished)
        {
            cmd.position.x = nwu_target.x;
            cmd.position.y =nwu_target.y;
            cmd_pub.publish(cmd);

            ros::spinOnce();
            ros::Rate rate(20.0);
            rate.sleep();
            return BT::NodeStatus::RUNNING;
        }
        log_file_ << "MoveLeft FAILURE.\n";
        return BT::NodeStatus::FAILURE;
    }

    // 终止当前动作
    virtual void halt() override
    {
        ROS_INFO("MoveLeft halted");
        log_file_ << "MoveLeft halted.\n";
        is_moving_ = false;  // 重置状态
        is_finished = false;
    }

private:   
    bool is_moving_;      // 是否在移动中
    bool is_finished;     // 动作是否完成
    Position3D nwu_target;
};





class TurnRight : public BT::ActionNodeBase
{
public:
    // 构造函数，传递名称和节点配置
    TurnRight(const std::string& name, const BT::NodeConfiguration& config)
        : BT::ActionNodeBase(name, config), is_turning_(false), is_finished(false)
    {
        cmd.coordinate_frame = mavros_msgs::PositionTarget::FRAME_LOCAL_NED;
        cmd.type_mask = ~uint16_t(0) & ~(uint16_t(0xff) << 12);  //最后结果为0000 1111 1111 1111
        cmd.type_mask &= (~uint16_t(mavros_msgs::PositionTarget::FORCE));
        cmd.type_mask &= ~(mavros_msgs::PositionTarget::IGNORE_PX | mavros_msgs::PositionTarget::IGNORE_PY | mavros_msgs::PositionTarget::IGNORE_PZ); 
        cmd.type_mask &= ~(mavros_msgs::PositionTarget::IGNORE_YAW);
    }

    // 设置节点的输入端口
    static BT::PortsList providedPorts()
    {
        const char* description = "Turn right target yaw...";
        return { BT::InputPort<double>("turn_attitude", description) };
    }

    // 核心逻辑，处理动作的执行
    virtual BT::NodeStatus tick() override
    {
        // 如果动作尚未开始，初始化目标偏航角
        if (!is_turning_)
        {
            cmd.coordinate_frame = mavros_msgs::PositionTarget::FRAME_LOCAL_NED;
            cmd.type_mask = ~uint16_t(0) & ~(uint16_t(0xff) << 12);  //最后结果为0000 1111 1111 1111
            cmd.type_mask &= (~uint16_t(mavros_msgs::PositionTarget::FORCE));
            cmd.type_mask &= ~(mavros_msgs::PositionTarget::IGNORE_PX | mavros_msgs::PositionTarget::IGNORE_PY | mavros_msgs::PositionTarget::IGNORE_PZ); 
            cmd.type_mask &= ~(mavros_msgs::PositionTarget::IGNORE_YAW);
            auto res = getInput<double>("turn_attitude");
            if (!res)
            {
                throw BT::RuntimeError("error reading port [turn_attitude]:", res.error());
            }
            target_yaw = res.value();
            log_file_ << "TurnRight degrees: [" << target_yaw << "]\n";
            while (target_yaw < 0)
            {
                target_yaw += 360;/* code */
            }
            while (target_yaw > 360)
            {
                target_yaw -= 360;/* code */
            }
            target_yaw = fmod(target_yaw, 360);
            
            target_yaw = target_yaw* M_PI / 180.0;
            target_yaw = current_yaw - target_yaw; // 目标偏航角是当前偏航角加上输入的角度
            if (target_yaw > M_PI) target_yaw -= 2 * M_PI; // 角度范围限制到 [-π, π]
            else if (target_yaw < -M_PI) target_yaw += 2 * M_PI;
            nwu_target = Position3D{fcu_pose.pose.position.x, fcu_pose.pose.position.y,
            fcu_pose.pose.position.z};
            cmd.yaw = target_yaw;
            cmd.position.x = nwu_target.x;
            cmd.position.y = nwu_target.y;
            cmd.position.z = nwu_target.z;
            cmd.header.stamp = ros::Time::now();
            
            //log_file_ << "TurnRight target nwu: [" << nwu_target.x << ", " << nwu_target.y << ", " << nwu_target.z << "]\n";
            printf("TurnRight target yaw: %.1f\n", target_yaw);
            

            // 开始转向
            is_turning_ = true;
            ROS_INFO("TurnRight: Turning to yaw %.1f", rad2deg(target_yaw));
        }

        // 检查无人机是否达到目标偏航角
        if (compare_angles(current_yaw, target_yaw)&& !is_finished) // 假设偏航角误差小于0.1即认为到达目标
        {
            ROS_INFO("TurnRight finished");
            log_file_ << "TurnRight finished. drone pose [nwu,yaw]:(" << fcu_pose.pose.position.x << ", " << fcu_pose.pose.position.y << ", " << fcu_pose.pose.position.z<<", " << rad2deg(current_yaw) << ")\n";
            is_finished = false;
            is_turning_ = false;
            for(int i=0; i<10; i++)
            {
                cmd.yaw = target_yaw;
                cmd.position.x = nwu_target.x;
                cmd.position.y = nwu_target.y;
                cmd.position.z = nwu_target.z;
                cmd.header.stamp = ros::Time::now();
                cmd_pub.publish(cmd);
                ros::spinOnce();
                ros::Rate rate(30.0);
                rate.sleep();
            }
            return BT::NodeStatus::SUCCESS;
        }
        else{
            ROS_INFO("TargetYaw:%.1f CurrentYaw:%.1f", target_yaw, current_yaw);
        }

        // 如果尚未到达目标，继续转向并返回 RUNNING

        cmd.yaw = target_yaw;
        cmd.position.x = nwu_target.x;
        cmd.position.y = nwu_target.y;
        cmd.position.z = nwu_target.z;
        cmd.header.stamp = ros::Time::now();
        cmd_pub.publish(cmd);
        ros::spinOnce();

        return BT::NodeStatus::RUNNING;
 
    }

    // 终止当前动作
    virtual void halt() override
    {
        ROS_INFO("TurnRight halted");
        log_file_ << "TurnRight halted.\n";
        is_turning_ = false;  // 重置状态
        is_finished = false;
    }

private:
    bool is_turning_;      // 是否在转向中
    bool is_finished;      // 动作是否完成
    double target_yaw;     // 目标偏航角
    Position3D nwu_target;
};



class TurnLeft : public BT::ActionNodeBase
{
    public:
    // 构造函数，传递名称和节点配置
    TurnLeft(const std::string& name, const BT::NodeConfiguration& config)
        : BT::ActionNodeBase(name, config), is_turning_(false), is_finished(false)
    {
        cmd.coordinate_frame = mavros_msgs::PositionTarget::FRAME_LOCAL_NED;
        cmd.type_mask = ~uint16_t(0) & ~(uint16_t(0xff) << 12);  //最后结果为0000 1111 1111 1111
        cmd.type_mask &= (~uint16_t(mavros_msgs::PositionTarget::FORCE));
        cmd.type_mask &= ~(mavros_msgs::PositionTarget::IGNORE_PX | mavros_msgs::PositionTarget::IGNORE_PY | mavros_msgs::PositionTarget::IGNORE_PZ); 
        cmd.type_mask &= ~(mavros_msgs::PositionTarget::IGNORE_YAW);
    }

    // 设置节点的输入端口
    static BT::PortsList providedPorts()
    {
        const char* description = "Turn left target yaw...";
        return { BT::InputPort<double>("turn_attitude", description) };
    }

    // 核心逻辑，处理动作的执行
    virtual BT::NodeStatus tick() override
    {
        // 如果动作尚未开始，初始化目标偏航角
        if (!is_turning_)
        {
            cmd.coordinate_frame = mavros_msgs::PositionTarget::FRAME_LOCAL_NED;
            cmd.type_mask = ~uint16_t(0) & ~(uint16_t(0xff) << 12);  //最后结果为0000 1111 1111 1111
            cmd.type_mask &= (~uint16_t(mavros_msgs::PositionTarget::FORCE));
            cmd.type_mask &= ~(mavros_msgs::PositionTarget::IGNORE_PX | mavros_msgs::PositionTarget::IGNORE_PY | mavros_msgs::PositionTarget::IGNORE_PZ); 
            cmd.type_mask &= ~(mavros_msgs::PositionTarget::IGNORE_YAW);
            auto res = getInput<double>("turn_attitude");
            if (!res)
            {
                throw BT::RuntimeError("error reading port [turn_attitude]:", res.error());
            }
            target_yaw = res.value();
            log_file_ << "TurnLeft degrees: [" << target_yaw << "]\n";
            while (target_yaw < 0)
            {
                target_yaw += 360;/* code */
            }
            while (target_yaw > 360)
            {
                target_yaw -= 360;/* code */
            }
            target_yaw = fmod(target_yaw, 360);
            
            target_yaw = target_yaw* M_PI / 180.0;
            target_yaw = current_yaw + target_yaw; // 目标偏航角是当前偏航角加上输入的角度
            if (target_yaw > M_PI) target_yaw -= 2 * M_PI; // 角度范围限制到 [-π, π]
            else if (target_yaw < -M_PI) target_yaw += 2 * M_PI;
            nwu_target = Position3D{fcu_pose.pose.position.x, fcu_pose.pose.position.y, fcu_pose.pose.position.z};
            cmd.yaw = target_yaw;
            cmd.position.x = nwu_target.x;
            cmd.position.y = nwu_target.y;
            cmd.position.z = nwu_target.z;
            cmd.header.stamp = ros::Time::now();
            printf("TurnLeft target yaw: %.1f\n", target_yaw);
            

            // 开始转向
            is_turning_ = true;
            ROS_INFO("TurnLeft: Turning to yaw %.1f", rad2deg(target_yaw));
        }

        // 检查无人机是否达到目标偏航角
        if (compare_angles(current_yaw, target_yaw) && !is_finished) // 假设偏航角误差小于0.1即认为到达目标
        {
            ROS_INFO("TurnLeft finished");
            log_file_ << "TurnLeft finished. drone pose [nwu,yaw]:(" << fcu_pose.pose.position.x << ", " << fcu_pose.pose.position.y << ", " << fcu_pose.pose.position.z<<", " << rad2deg(current_yaw) << ")\n";
            is_finished = false;
            is_turning_ = false;
            for(int i=0; i<3; i++)
            {
                cmd.yaw = target_yaw;
                cmd.position.x = nwu_target.x;
                cmd.position.y = nwu_target.y;
                cmd.position.z = nwu_target.z;
                cmd.header.stamp = ros::Time::now();
                cmd_pub.publish(cmd);
                ros::spinOnce();
                ros::Rate rate(30.0);
                rate.sleep();
            }
            return BT::NodeStatus::SUCCESS;
        }
        else{
            ROS_INFO("TargetYaw:%.1f CurrentYaw:%.1f", target_yaw, current_yaw);
            cmd.yaw = target_yaw;
            cmd.position.x = nwu_target.x;
            cmd.position.y = nwu_target.y;
            cmd.position.z = nwu_target.z;
            cmd.header.stamp = ros::Time::now();
            cmd_pub.publish(cmd);
            ros::spinOnce();
    
            return BT::NodeStatus::RUNNING;
        }

        // 如果尚未到达目标，继续转向并返回 RUNNING

        
 
    }

    // 终止当前动作
    virtual void halt() override
    {
        ROS_INFO("TurnLeft halted");
        log_file_ << "TurnLeft halted.\n";
        is_turning_ = false;  // 重置状态
        is_finished = false;
    }

private:
    bool is_turning_;      // 是否在转向中
    bool is_finished;      // 动作是否完成
    double target_yaw;     // 目标偏航角
    Position3D nwu_target;
};


class MoveUp : public BT::ActionNodeBase
{
public:
    // 构造函数，传递名称和节点配置
    MoveUp(const std::string& name, const BT::NodeConfiguration& config)
        : BT::ActionNodeBase(name, config), is_moving_(false), is_finished(false)
    { 
        cmd.coordinate_frame = mavros_msgs::PositionTarget::FRAME_LOCAL_NED;
        cmd.type_mask =
            ~uint16_t(0) & ~(uint16_t(0xff) << 12);  //最后结果为0000 1111 1111 1111
        cmd.type_mask &= (~uint16_t(mavros_msgs::PositionTarget::FORCE)); 
        cmd.type_mask &= ~(mavros_msgs::PositionTarget::IGNORE_PX | mavros_msgs::PositionTarget::IGNORE_PY | mavros_msgs::PositionTarget::IGNORE_PZ);        
    }

    // 设置节点的输入端口
    static BT::PortsList providedPorts()
    {
        const char* description = "Simply print the goal on console...";
        return { BT::InputPort<Distance>("MoveUp_distance", description) };
    }

    // 核心逻辑，处理动作的执行
    virtual BT::NodeStatus tick() override
    {
        // 如果动作尚未开始，初始化目标距离
        if (!is_moving_)
        {
            auto res = getInput<Distance>("MoveUp_distance");
            if (!res)
            {
                throw BT::RuntimeError("error reading port [MoveUp_distance]:", res.error());
            }
            Distance target = res.value();
            printf("MoveUp positions: [ %.1f]\n", target.x);
            log_file_ << "MoveUp target positions: [ " << target.x << "]\n"; 

            // 设定目标位置
            lzj_distance =  fcu_pose.pose.position.z + target.x;
            is_moving_ = true;
            ROS_INFO("MoveUp: Moving to %.1f", lzj_distance);
        }

        // 检查无人机是否达到目标位置
        if (abs(fcu_pose.pose.position.z - lzj_distance) < 0.1 && !is_finished)
        {
            ROS_INFO("MoveUp finished");
            log_file_ << "MoveUp finished. drone pose [nwu,yaw] [ " << fcu_pose.pose.position.x << ", " << fcu_pose.pose.position.y << ", " << fcu_pose.pose.position.z<<", " << rad2deg(current_yaw) << ")\n";
            is_finished = true;
            return BT::NodeStatus::SUCCESS;
        }

        // 如果尚未到达目标，继续移动并返回 RUNNING
        if (!is_finished)
        {
            cmd.position.z = lzj_distance;
            cmd_pub.publish(cmd);

            ros::spinOnce();
            ros::Rate rate(20.0);
            rate.sleep();
            return BT::NodeStatus::RUNNING;
        }
        log_file_ << "MoveUp FAILURE.\n";
        return BT::NodeStatus::FAILURE;
    }

    // 终止当前动作
    virtual void halt() override
    {
        ROS_INFO("MoveUp halted");
        log_file_ << "MoveUp halted.\n";
        is_moving_ = false;  // 重置状态
        is_finished = false;
    }

private:   
    bool is_moving_;      // 是否在移动中
    bool is_finished;     // 动作是否完成
    float lzj_distance;  // 目标距离
};



class MoveDown : public BT::ActionNodeBase
{
public:
    // 构造函数，传递名称和节点配置
    MoveDown(const std::string& name, const BT::NodeConfiguration& config)
        : BT::ActionNodeBase(name, config), is_moving_(false), is_finished(false)
    { 
        cmd.coordinate_frame = mavros_msgs::PositionTarget::FRAME_LOCAL_NED;
        cmd.type_mask =
            ~uint16_t(0) & ~(uint16_t(0xff) << 12);  //最后结果为0000 1111 1111 1111
        cmd.type_mask &= (~uint16_t(mavros_msgs::PositionTarget::FORCE)); 
        cmd.type_mask &= ~(mavros_msgs::PositionTarget::IGNORE_PX | mavros_msgs::PositionTarget::IGNORE_PY | mavros_msgs::PositionTarget::IGNORE_PZ); 
    }

    // 设置节点的输入端口
    static BT::PortsList providedPorts()
    {
        const char* description = "Simply print the goal on console...";
        return { BT::InputPort<Distance>("MoveDown_distance", description) };
    }

    // 核心逻辑，处理动作的执行
    virtual BT::NodeStatus tick() override
    {
        // 如果动作尚未开始，初始化目标距离
        if (!is_moving_)
        {
            auto res = getInput<Distance>("MoveDown_distance");
            if (!res)
            {
                throw BT::RuntimeError("error reading port [MoveDown_distance]:", res.error());
            }
            Distance target = res.value();
            printf("MoveDown positions: [ %.1f]\n", target.x);
            log_file_ << "MoveDown target positions: [ " << target.x << "]\n"; 

            // 设定目标位置
            lzj_distance =  fcu_pose.pose.position.z - target.x;
            is_moving_ = true;
            ROS_INFO("MoveDown: Moving to %.1f", lzj_distance);
        }

        // 检查无人机是否达到目标位置
        if (abs(fcu_pose.pose.position.z - lzj_distance) < 0.1 && !is_finished)
        {
            ROS_INFO("MoveDown finished");
            log_file_ << "MoveDown finished. drone pose [nwu,yaw] [ " << fcu_pose.pose.position.x << ", " << fcu_pose.pose.position.y << ", " << fcu_pose.pose.position.z<<", " << rad2deg(current_yaw) << ")\n";
            is_finished = true;
            return BT::NodeStatus::SUCCESS;
        }

        // 如果尚未到达目标，继续移动并返回 RUNNING
        if (!is_finished)
        {
            cmd.position.z = lzj_distance;
            cmd_pub.publish(cmd);

            ros::spinOnce();
            ros::Rate rate(20.0);
            rate.sleep();
            return BT::NodeStatus::RUNNING;
        }
        log_file_ << "MoveDown FAILURE.\n";
        return BT::NodeStatus::FAILURE;
    }

    // 终止当前动作
    virtual void halt() override
    {
        ROS_INFO("MoveDown halted");
        log_file_ << "MoveDown halted.\n";
        is_moving_ = false;  // 重置状态
        is_finished = false;
    }

private:   
    bool is_moving_;      // 是否在移动中
    bool is_finished;     // 动作是否完成
    float lzj_distance;  // 目标距离
};
 

void yolo2nwu(Position3D* pos) {
    if (pos == nullptr) {
        std::cerr << "Error: Invalid position pointer" << std::endl;
        return;
    }

    // Invert the z-axis for Down to Up conversion
    pos->y = -pos->y;
    pos->z = -pos->z;
}


class FlyToCoordinates : public BT::ActionNodeBase
{
public:
    FlyToCoordinates(const std::string& name, const BT::NodeConfiguration& config)
        : BT::ActionNodeBase(name, config), is_finished(false)
    {
        cmd.coordinate_frame = mavros_msgs::PositionTarget::FRAME_LOCAL_NED;
        cmd.type_mask =
            ~uint16_t(0) & ~(uint16_t(0xff) << 12);  //最后结果为0000 1111 1111 1111
        cmd.type_mask &= (~uint16_t(mavros_msgs::PositionTarget::FORCE)); 
        cmd.type_mask &= ~(mavros_msgs::PositionTarget::IGNORE_PX | mavros_msgs::PositionTarget::IGNORE_PY | mavros_msgs::PositionTarget::IGNORE_PZ); 

    }

    // 定义输入端口
    static BT::PortsList providedPorts()
    {
        const char* description = "Target position as x, y, z";
        return { BT::InputPort<Position3D>("coordinates", description) };
    }

    // tick()方法会在每次更新时调用
    virtual BT::NodeStatus tick() override
    {
        // 第一次进入时获取输入目标位置
        if (!is_finished)
        {
            auto res = getInput<Position3D>("coordinates");
            if (!res)
            {
                throw BT::RuntimeError("error reading port [coordinates]:", res.error());
            }
            target = res.value();
            printf("FlyToCoordinates NWU Target:(%.1f, %.1f, %.1f)\n", target.x, target.y, target.z);
            //log_file_ << "FlyToCoordinates nwu target positions:(" << target.x << ", " << target.y << ", " << target.z << ")\n";
            // cmd.coordinate_frame = mavros_msgs::PositionTarget::FRAME_LOCAL_NED;
            // cmd.type_mask = mavros_msgs::PositionTarget::IGNORE_VX |
            //                 mavros_msgs::PositionTarget::IGNORE_VY |
            //                 mavros_msgs::PositionTarget::IGNORE_VZ;

            cmd.position.x = target.x;
            cmd.position.y = target.y;
            cmd.position.z = target.z;
            cmd.header.stamp = ros::Time::now();
            is_finished = true;
        }

        // 检查无人机是否到达目标位置
        if (abs(fcu_pose.pose.position.x - target.x) < 0.05 &&
            abs(fcu_pose.pose.position.y - target.y) < 0.05 &&
            abs(fcu_pose.pose.position.z - target.z) < 0.05)
        {
            ROS_INFO("FlyToCoordinates finished");
            log_file_ << "FlyToCoordinates finished. drone pose [nwu,yaw] (" << fcu_pose.pose.position.x << ", " << fcu_pose.pose.position.y << ", " << fcu_pose.pose.position.z<<", " << rad2deg(current_yaw) << ")\n";
            // is_finished = true;
            return BT::NodeStatus::SUCCESS;
        }

        printf("FlyToCoordinates Target: [ %.1f, %.1f, %.1f]\n", target.x, target.y, target.z);
        printf("fcu_pose_pose_position [%.1f, %.1f, %.1f]\n", fcu_pose.pose.position.x, fcu_pose.pose.position.y, fcu_pose.pose.position.z);
        // 发送目标位置并返回 RUNNING
        cmd.coordinate_frame = mavros_msgs::PositionTarget::FRAME_LOCAL_NED;
        cmd.type_mask =
            ~uint16_t(0) & ~(uint16_t(0xff) << 12);  //最后结果为0000 1111 1111 1111
        cmd.type_mask &= (~uint16_t(mavros_msgs::PositionTarget::FORCE)); 
        cmd.type_mask &= ~(mavros_msgs::PositionTarget::IGNORE_PX | mavros_msgs::PositionTarget::IGNORE_PY | mavros_msgs::PositionTarget::IGNORE_PZ); 
        cmd_pub.publish(cmd);
        ros::spinOnce();
        // is_finished = true;
        return BT::NodeStatus::RUNNING;
    }

    // 当行为树终止该动作时调用halt()方法
    virtual void halt() override
    {
        ROS_INFO("FlyToCoordinates halted");
        log_file_ << "FlyToCoordinates halted.\n";
        is_finished = false; // 重置状态
    }

private:
    bool is_finished;                 // 标识任务是否完成
    Position3D target;                // 目标位置
};



class SendVelocity : public BT::ActionNodeBase
{
public:
    SendVelocity(const std::string& name, const BT::NodeConfiguration& config)
        : BT::ActionNodeBase(name, config), is_finished(false), is_getinput(false) 
    {}

    // 定义输入端口
    static BT::PortsList providedPorts()
    {
        const char* description = "Target SendVelocity as vx, vy, vz";
        return { BT::InputPort<Position3D>("SendVelocity", description) };
    }

    // tick()方法会在每次更新时调用
    virtual BT::NodeStatus tick() override
    {
        // 第一次进入时获取输入速度
        if(!is_getinput){

            auto res = getInput<Position3D>("SendVelocity");
            if (!res)
            {
                throw BT::RuntimeError("error reading port [SendVelocity]:", res.error());
            }
            target = res.value();
            printf("SendVelocity Target: [ %.1f, %.1f, %.1f]\n", target.x, target.y, target.z);
            log_file_ << "SendVelocity nwu target: (" << target.x << ", " << target.y << ", " << target.z << ")\n";
            is_getinput = true;
        }
        
        
        if (!is_finished)
        {
            // auto res = getInput<Position3D>("SendVelocity");
            // if (!res)
            // {
            //     throw BT::RuntimeError("error reading port [SendVelocity]:", res.error());
            // }
            // target = res.value();
            // printf("SendVelocity Target: [ %.1f, %.1f, %.1f]\n", target.x, target.y, target.z);

            // cmd.coordinate_frame = mavros_msgs::PositionTarget::FRAME_LOCAL_NED;
            cmd.type_mask =
                ~uint16_t(0) & ~(uint16_t(0xff) << 12);  //最后结果为0000 1111 1111 1111
            cmd.type_mask &= (~uint16_t(mavros_msgs::PositionTarget::FORCE));  //把 FORCE 屏蔽掉
            cmd.type_mask &= ~(mavros_msgs::PositionTarget::IGNORE_VX | mavros_msgs::PositionTarget::IGNORE_VY
                                | mavros_msgs::PositionTarget::IGNORE_VZ);  
            // 使用速度控制；
            cmd.coordinate_frame = mavros_msgs::PositionTarget::FRAME_BODY_NED; 
            cmd.header.frame_id = "base_link";
            // // cmd.coordinate_frame = Command::FRAME_BODY_NED;
            // cmd.type_mask = ~uint16_t(0);
            // // cmd.type_mask &= ~mavros_msgs::PositionTarget::FORCE;
            // // cmd.type_mask &= ~mavros_msgs::PositionTarget::IGNORE_VX;
            // // cmd.type_mask &= ~mavros_msgs::PositionTarget::IGNORE_VY;
            // // cmd.type_mask &= ~mavros_msgs::PositionTarget::IGNORE_VZ;  
            // cmd.type_mask = mavros_msgs::PositionTarget::IGNORE_PX |
            //                 mavros_msgs::PositionTarget::IGNORE_PY |
            //                 mavros_msgs::PositionTarget::IGNORE_PZ |
            //                 mavros_msgs::PositionTarget::IGNORE_AFX |
            //                 mavros_msgs::PositionTarget::IGNORE_AFY |
            //                 mavros_msgs::PositionTarget::IGNORE_AFZ |
            //                 mavros_msgs::PositionTarget::IGNORE_YAW |
            //                 mavros_msgs::PositionTarget::IGNORE_YAW_RATE;            
            cmd.velocity.x = target.x;
            cmd.velocity.y = target.y;
            cmd.velocity.z = target.z;
            // cmd.header.stamp = ros::Time::now();
            cmd.header.stamp = ros::Time::now();
            cmd_pub.publish(cmd);
            ros::spinOnce();
        }
        printf("SendVelocity Target: [ %.1f, %.1f, %.1f]\n", target.x, target.y, target.z);
        printf("uav Velocity: [ %.1f, %.1f, %.1f]\n", vx, vy, vz);
        printf("fcu_pose_pose_position [%.1f, %.1f, %.1f]\n", fcu_pose.pose.position.x, fcu_pose.pose.position.y, fcu_pose.pose.position.z);
        // 检查无人机是否到达设定速度
        if (abs(vx - target.x) < 0.1 &&
            abs(vy - target.y) < 0.1 &&
            abs(vz - target.z) < 0.1)
        {
            ROS_INFO("SendVelocity finished");
            log_file_ << "SendVelocity finished: [ " << fcu_pose.pose.position.x << ", " << fcu_pose.pose.position.y << ", " << fcu_pose.pose.position.z << "]\n";
            log_file_ << "SendVelocity finished vx vy vz: [ " << vx << ", " << vy << ", " << vz << "]\n";
            is_finished = true;
            return BT::NodeStatus::SUCCESS;
        }

        // 发送速度并返回 RUNNING
        cmd.header.stamp = ros::Time::now();
        cmd_pub.publish(cmd);
        cmd.velocity.x = target.x;
        cmd.velocity.y = target.y;
        cmd.velocity.z = target.z;
        ros::spinOnce();
        // is_finished = true;
        // return BT::NodeStatus::SUCCESS;
        return BT::NodeStatus::RUNNING;
    }

    // 当行为树终止该动作时调用halt()方法
    virtual void halt() override
    {
        ROS_INFO("SendVelocity halted");
        log_file_ << "SendVelocity halted.\n";
        is_finished = false; // 重置状态
    }

private:
    bool is_finished;                 // 标识任务是否完成
    bool is_getinput;                 // 获取输入标记
    Position3D target;                // 目标位置
};

const double PI = M_PI; // 定义圆周率
const double EPSILON = 0.01; // 定义你的差值范围

bool compare_angles(double angle1_rad, double angle2_rad) {

    // 计算两个角度的余弦值和正弦值
    double cos_angle1 = cos(angle1_rad);
    double cos_angle2 = cos(angle2_rad);
    double sin_angle1 = sin(angle1_rad);
    double sin_angle2 = sin(angle2_rad);

    // 计算余弦值和正弦值的差值
    double cos_diff = std::abs(cos_angle1 - cos_angle2);
    double sin_diff = std::abs(sin_angle1 - sin_angle2);

    // 如果差值在你定义的范围内，返回 true
    return (cos_diff < EPSILON && sin_diff < EPSILON);
}

class SetYawAngle : public BT::ActionNodeBase
{
public:
    SetYawAngle(const std::string& name, const BT::NodeConfiguration& config)
        : BT::ActionNodeBase(name, config), is_finished(false)
    {
        // pose_sub = nh_.subscribe("mavros/local_position/pose", 10, &SetYawAngle::poseCallback, this);
    }

    // 定义输入端口
    static BT::PortsList providedPorts()
    {
        const char* description = "Target yaw angle in radians";
        return { BT::InputPort<Distance>("yaw", description) };
    }

    // tick() 方法会在每次更新时调用
    virtual BT::NodeStatus tick() override
    {
        if (!is_finished)
        {
            // 获取输入的目标偏航角
            auto res = getInput<Distance>("yaw");
            if (!res)
            {
                throw BT::RuntimeError("error reading port [SetYawAngle]:", res.error());
            }
            target_yaw = res.value().x;
            while (target_yaw < 0)
            {
                target_yaw += 360;/* code */
            }
            while (target_yaw > 360)
            {
                target_yaw -= 360;/* code */
            }
            target_yaw = fmod(target_yaw, 360);
            target_yaw = target_yaw* M_PI / 180.0;
            if (target_yaw > M_PI)
                target_yaw -= 2 * M_PI;
            else if (target_yaw < -M_PI)
                target_yaw += 2 * M_PI;

            
            

            cmd.coordinate_frame = mavros_msgs::PositionTarget::FRAME_LOCAL_NED;
            // cmd.type_mask = mavros_msgs::PositionTarget::IGNORE_PX |
            //                 mavros_msgs::PositionTarget::IGNORE_PY |
            //                 mavros_msgs::PositionTarget::IGNORE_PZ |
            //                 mavros_msgs::PositionTarget::IGNORE_VX |
            //                 mavros_msgs::PositionTarget::IGNORE_VY |
            //                 mavros_msgs::PositionTarget::IGNORE_VZ |
            //                 mavros_msgs::PositionTarget::IGNORE_AFX |
            //                 mavros_msgs::PositionTarget::IGNORE_AFY |
            //                 mavros_msgs::PositionTarget::IGNORE_AFZ |
            //                 mavros_msgs::PositionTarget::IGNORE_YAW_RATE;
            cmd.type_mask =
            ~uint16_t(0) & ~(uint16_t(0xff) << 12);  //最后结果为0000 1111 1111 1111
            cmd.type_mask &= (~uint16_t(mavros_msgs::PositionTarget::FORCE)); 
            cmd.type_mask &=
            ~(mavros_msgs::PositionTarget::IGNORE_PX | mavros_msgs::PositionTarget::IGNORE_PY | mavros_msgs::PositionTarget::IGNORE_PZ
            | mavros_msgs::PositionTarget::IGNORE_YAW);  // 使用位置控制起飞
            cmd.position.x = fcu_pose.pose.position.x;
            cmd.position.y = fcu_pose.pose.position.y;
            cmd.position.z = fcu_pose.pose.position.z;
            cmd.yaw = target_yaw;
            cmd.header.stamp = ros::Time::now();
        }

        // 当前姿态的yaw角已经在poseCallback中更新
        if (compare_angles(current_yaw, target_yaw))
        {
            ROS_INFO("SetYawAngle finished");
            log_file_ << "SetYawAngle finished. drone pose [nwu,yaw]:(" << fcu_pose.pose.position.x << ", " << fcu_pose.pose.position.y << ", " << fcu_pose.pose.position.z << ", " << rad2deg(current_yaw) << ")\n";
            is_finished = false;
            return BT::NodeStatus::SUCCESS;
        }
        else{
            ROS_INFO("TargetYaw:%.1f CurrentYaw:%.1f", target_yaw, current_yaw);
        }

        // 发送偏航角控制命令
        cmd_pub.publish(cmd);
        ros::spinOnce();
        return BT::NodeStatus::RUNNING;
    }

    // 当行为树终止该动作时调用halt()方法
    virtual void halt() override
    {
        ROS_INFO("SetYawAngle halted");
        log_file_ << "SetYawAngle halted.\n";
        is_finished = false;  // 重置任务状态
    }

private:
    // 订阅当前姿态信息
    // void poseCallback(const geometry_msgs::PoseStamped::ConstPtr& msg)
    // {
    //     tf2::Quaternion q(
    //         msg->pose.orientation.x,
    //         msg->pose.orientation.y,
    //         msg->pose.orientation.z,
    //         msg->pose.orientation.w);
    //     tf2::Matrix3x3 m(q);
    //     double roll, pitch;
    //     m.getRPY(roll, pitch, current_yaw);  // 获取当前的yaw角
    // }

    // ros::NodeHandle nh_;
    // ros::Subscriber pose_sub;

    bool is_finished;                // 标识任务是否完成
    double target_yaw;               // 目标偏航角（弧度）
    // double current_yaw;              // 当前偏航角（弧度）
};





class Land : public BT::ActionNodeBase
{
public:
    Land(const std::string& name, const BT::NodeConfiguration& config)
        : BT::ActionNodeBase(name, config), is_finished(false)
    {}

    // 定义输入输出端口
    static BT::PortsList providedPorts()
    {
        return {};  // 降落不需要额外输入端口
    }

    // tick() 方法会在每次更新时调用
    virtual BT::NodeStatus tick() override
    {
        if (!is_finished)
        {
            // 切换到自动降落模式
            land.request.custom_mode = "AUTO.LAND";

            // 发送降落模式请求
            if (set_mode_client.call(land) && land.response.mode_sent)
            {
                ROS_INFO("Land mode enabled");
                is_finished = true;
            }
            else
            {
                ROS_ERROR("Failed to set Land mode");
                log_file_ << "Land FAILURE.\n";
                return BT::NodeStatus::FAILURE;
            }
        }

        // 在一定的时间内发布降落指令
        if (ros::Time::now().toSec() - last_command_time.toSec() > 10.0)
        {
            // 发送降落指令
            cmd.coordinate_frame = mavros_msgs::PositionTarget::FRAME_BODY_NED;
            cmd_pub.publish(cmd);
            ROS_INFO("Landing in progress...");
            last_command_time = ros::Time::now();
            return BT::NodeStatus::RUNNING;
        }

        ROS_INFO("Land finished");
        log_file_ << "Land finished. drone pose [nwu,yaw]:(" << fcu_pose.pose.position.x << ", " << fcu_pose.pose.position.y << ", " << fcu_pose.pose.position.z << ", " << rad2deg(current_yaw) << ")\n";
        return BT::NodeStatus::SUCCESS;
    }

    // 当行为树终止该动作时调用halt()方法
    virtual void halt() override
    {
        ROS_INFO("Land action halted");
        log_file_ << "Land halted.\n";
        is_finished = false;  // 重置任务状态
    }

private:
    ros::Time last_command_time;

    bool is_finished;  // 标识任务是否完成
};


class ForwardDetect2D : public BT::ActionNodeBase 
{
public:
    // 构造函数，传递名称和节点配置
    ForwardDetect2D(const std::string& name, const BT::NodeConfiguration& config)
        : BT::ActionNodeBase(name, config), is_finished(false)
    {}

    // 设置节点的输入输出端口
    static BT::PortsList providedPorts()
    {
        const char* inputDesc = "Input class type...";
        const char* outputDesc = "Output detected details: target category, ID, coordinates, confidence, and other info.";
        return {
            BT::InputPort<std::string>("classtype", inputDesc),
            // BT::OutputPort<std::string>("target_category", outputDesc),
            // BT::OutputPort<int>("target_id", outputDesc),
            // BT::OutputPort<geometry_msgs::msg::Point>("coordinates", outputDesc),
            // BT::OutputPort<float>("confidence", outputDesc),
            // BT::OutputPort<std::vector<std::string>>("other", outputDesc)
            BT::OutputPort<ObjData>("ForwardObjData", outputDesc)
        };
    }

    // 核心逻辑，处理动作的执行
    virtual BT::NodeStatus tick() override
    {
        ROS_INFO("ForwardDetect2D action START");
        
        std::string class_type;
        if (!getInput<std::string>("classtype", class_type)) {
            log_file_ << "ForwardDetect2D FAILURE. dont get classtype\n";
            return BT::NodeStatus::FAILURE;
        }

        // 检查是否有检测结果
        if (is_finished || forward_obj.class_name.empty()) {
            log_file_ << "ForwardDetect2D FAILURE. forward_obj empty\n";
            return BT::NodeStatus::FAILURE;
        }

        if (class_type != down_obj.class_name) {
            return BT::NodeStatus::RUNNING;
        }  
        // setOutput("target_category", lzj_obj.class_name);
        // setOutput("target_id", lzj_obj.class_id);
        
        // geometry_msgs::msg::Point coordinates;
        // coordinates.x = (lzj_obj.left_top_x + lzj_obj.right_bottom_x) / 2.0; // 中心点
        // coordinates.y = (lzj_obj.left_top_y + lzj_obj.right_bottom_y) / 2.0; // 中心点
        // setOutput("coordinates", coordinates);
        
        // setOutput("confidence", lzj_obj.score);
        // std::vector<std::string> other_info; // 填充其他信息
        // setOutput("other", other_info);

        objdata.class_name = forward_obj.class_name;
        // objdata.class_id = forward_obj.class_id;
        objdata.class_id = 1;
        objdata.left_top_x = forward_obj.left_top_x;
        objdata.left_top_y = forward_obj.left_top_y;
        objdata.right_bottom_x = forward_obj.right_bottom_x;
        objdata.right_bottom_y = forward_obj.right_bottom_y;
        objdata.coordinates.x = (forward_obj.left_top_x + forward_obj.right_bottom_x) / 2.0;
        objdata.coordinates.y = (forward_obj.left_top_y + forward_obj.right_bottom_y) / 2.0;
        objdata.score = forward_obj.score;
        setOutput("ForwardObjData", objdata);
        log_file_ << "ForwardDetect2D finished: [ " << fcu_pose.pose.position.x << ", " << fcu_pose.pose.position.y << ", " << fcu_pose.pose.position.z << ", " << rad2deg(current_yaw) << "]\n";
        is_finished = true;

        return BT::NodeStatus::SUCCESS;
    }

    // 终止当前动作
    virtual void halt() override
    {
        ROS_INFO("ForwardDetect2D action halted");
        is_finished = false;
    }

private:
    ObjData objdata;
    bool is_finished;     // 动作是否完成
};




class DownDetect2D : public BT::ActionNodeBase 
{
public:
    // 构造函数，传递名称和节点配置
    DownDetect2D(const std::string& name, const BT::NodeConfiguration& config)
        : BT::ActionNodeBase(name, config), is_finished(false)
    {}

    // 设置节点的输入输出端口
    static BT::PortsList providedPorts()
    {
        const char* inputDesc = "Input class type...";
        const char* outputDesc = "Output detected details: target category, ID, coordinates, confidence, and other info.";
        return {
            BT::InputPort<std::string>("classtype", inputDesc),
            // BT::OutputPort<std::string>("target_category", outputDesc),
            // BT::OutputPort<int>("target_id", outputDesc),
            // BT::OutputPort<geometry_msgs::msg::Point>("coordinates", outputDesc),
            // BT::OutputPort<float>("confidence", outputDesc),
            // BT::OutputPort<std::vector<std::string>>("other", outputDesc)
            BT::OutputPort<ObjData>("DownObjData", outputDesc)
        };
    }

    // 核心逻辑，处理动作的执行
    virtual BT::NodeStatus tick() override
    {
        ROS_INFO("DownDetect2D action START");
        std::string class_type;
        if (!getInput<std::string>("classtype", class_type)) {
            log_file_ << "DownDetect2D FAILURE. dont get classtype\n";
            return BT::NodeStatus::FAILURE;
        }

        // 检查是否有检测结果
        if (is_finished || down_obj.class_name.empty()) {
            log_file_ << "DownDetect2D FAILURE. down_obj empty\n";
            return BT::NodeStatus::FAILURE;
        }

        if (class_type != down_obj.class_name) {
            return BT::NodeStatus::RUNNING;
        }        
        // const auto& obj = lzj_objs.objects.front();  
        // setOutput("target_category", obj.class_name);
        // setOutput("target_id", obj.class_id);
        
        // geometry_msgs::msg::Point coordinates;
        // coordinates.x = (obj.left_top_x + obj.right_bottom_x) / 2.0; // 中心点
        // coordinates.y = (obj.left_top_y + obj.right_bottom_y) / 2.0; // 中心点
        // setOutput("coordinates", coordinates);
        
        // setOutput("confidence", obj.score);
        // std::vector<std::string> other_info; // 填充其他信息
        // setOutput("other", other_info);
        
        objdata.class_name = down_obj.class_name;
        // objdata.class_id = down_obj.class_id;
        objdata.class_id = 2;
        objdata.left_top_x = down_obj.left_top_x;
        objdata.left_top_y = down_obj.left_top_y;
        objdata.right_bottom_x = down_obj.right_bottom_x;
        objdata.right_bottom_y = down_obj.right_bottom_y;
        objdata.coordinates.x = (down_obj.left_top_x + down_obj.right_bottom_x) / 2.0;
        objdata.coordinates.y = (down_obj.left_top_y + down_obj.right_bottom_y) / 2.0;
        objdata.score = down_obj.score;

        setOutput("DownObjData", objdata);
        log_file_ << "DownDetect2D finished: [ " << fcu_pose.pose.position.x << ", " << fcu_pose.pose.position.y << ", " << fcu_pose.pose.position.z << ", " << rad2deg(current_yaw) << "]\n";
        is_finished = true;
        return BT::NodeStatus::SUCCESS;
    }

    // 终止当前动作
    virtual void halt() override
    {
        ROS_INFO("DownDetect2D action halted");
        is_finished = false;
    }

private:
    ObjData objdata;

    bool is_finished;     // 动作是否完成
};


class DetectArUco : public BT::ActionNodeBase 
{
public:
    // 构造函数，传递名称和节点配置
    DetectArUco(const std::string& name, const BT::NodeConfiguration& config)
        : BT::ActionNodeBase(name, config), is_finished(false)
    {}

    // 设置节点的输入输出端口
    static BT::PortsList providedPorts()
    {
        const char* inputDesc = "Input class type...";
        const char* outputDesc = "Output detected details: target category, ID, coordinates, confidence, and other info.";
        return {
            BT::OutputPort<Position3D>("vel", outputDesc),
            BT::OutputPort<ArucoData>("DetectArUcoData", outputDesc)
        };
    }

    // 核心逻辑，处理动作的执行
    virtual BT::NodeStatus tick() override
    {
        ROS_INFO("DetectArUco START");
        std::string class_type;
        
        // 检查是否有检测结果
        if (!is_aruco) {
            log_file_ << "DetectArUco FAILURE.\n";
            return BT::NodeStatus::FAILURE;
        }
     
        arucodata.header = aruco.header;
        arucodata.id = aruco.id;
        arucodata.cnt_x = aruco.cnt_x;
        arucodata.cnt_y = aruco.cnt_y;
        arucodata.position_x = aruco.position_x;
        arucodata.position_y = aruco.position_y;
        arucodata.position_z = aruco.position_z;
        arucodata.r_x = aruco.r_x;
        arucodata.r_y = aruco.r_y;
        arucodata.r_x = aruco.r_x;

    
        int    dx = int(aruco.cnt_x - rflysim_p.rgb_cnt.x);
        int    dy = int(aruco.cnt_y - rflysim_p.rgb_cnt.y);   
        arucodata.dx = dx;
        arucodata.dy = dy;

        double vx = -dy * kx;
        double vy = -dx * ky;    
        if(std::abs(vx) > vx_max)
          vx = vx / std::abs(vx) * vx_max;
        if(std::abs(vy) > vy_max)
          vy = vy / std::abs(vy) * vy_max;

        vel.x = vx;
        vel.y = vy;
        vel.z = 0;
        setOutput("vel", vel);

        setOutput("DetectArUcoData", arucodata);
        log_file_ << "DetectArUco finished: [ " << fcu_pose.pose.position.x << ", " << fcu_pose.pose.position.y << ", " << fcu_pose.pose.position.z << ", " << rad2deg(current_yaw) << "]\n";
        is_finished = true;
        is_aruco = false;
        return BT::NodeStatus::SUCCESS;
    }

    // 终止当前动作
    virtual void halt() override
    {
        ROS_INFO("DetectArUco action halted");
        log_file_ << "DetectArUco action halted.\n";
        is_finished = false;
    }

private:
    ArucoData arucodata;
    Position3D vel;
    
    bool is_finished;     // 动作是否完成
};

class Recognize_aruco : public BT::ActionNodeBase 
{
public:
    // 构造函数，传递名称和节点配置
    Recognize_aruco(const std::string& name, const BT::NodeConfiguration& config)
        : BT::ActionNodeBase(name, config), is_finished(false)
    {
        //如果到识别二维码的状态，可能二维码没在下视相机视场角内，这个时候应该飞高一点，使二维码或者降落标志在视场内
        cmd.coordinate_frame = mavros_msgs::PositionTarget::FRAME_LOCAL_NED;
        cmd.type_mask =
            ~uint16_t(0) & ~(uint16_t(0xff) << 12);  //最后结果为0000 1111 1111 1111
        cmd.type_mask &= (~uint16_t(mavros_msgs::PositionTarget::FORCE));  //把 FORCE 屏蔽掉
        cmd.type_mask &= ~(mavros_msgs::PositionTarget::IGNORE_VX | mavros_msgs::PositionTarget::IGNORE_VY
                            | mavros_msgs::PositionTarget::IGNORE_VZ);  // 使用速度控制；
        cmd.header.frame_id = "base_link";
    }

    // 设置节点的输入输出端口
    static BT::PortsList providedPorts()
    {

        const char* outputDesc = "Output detected vel";
        return {
            BT::OutputPort<Position3D>("arucoVel", outputDesc),
        };
    }

    // 核心逻辑，处理动作的执行
    virtual BT::NodeStatus tick() override
    {
        ROS_INFO("Recognize_aruco START");
        //此时二维码没在视场角内，应该飞高一点，但是如果识别到了降落标志，还没识别到二维码，说明图像清晰度不够，应超降落标志飞去，逐渐降低高度
        cmd.velocity.x = 0.01;
        cmd.velocity.y = 0.2;  //二维码趋势在此时机体坐标系的y方向
        if(rflysim_p.is_S)
        {
            cmd.velocity.y *= -1;
        }
        cmd.velocity.z = 0.05;  //速度不能太快，当然高度需要加上一个约束值，与此同时需要考虑气流的影响；
        if(fcu_pose.pose.position.z > rflysim_p.hight_max)
        {  
            //此时高度，不能再高了,此时降落表示应该识别了
            ROS_ERROR("vichle'z is too height");
            cmd.velocity.y = 0;
            cmd.velocity.z = 0;
            vel.z =0;
        }
        if(is_H)
        {  
            //发现降落标识，没识别出二维码
            //视觉伺服控制，x,y 往降落标志点飞行
            auto dx =
                (down_obj1.left_top_x + down_obj1.right_bottom_x) / 2. - rflysim_p.rgb_cnt.x;
            auto dy =
                (down_obj1.left_top_y + down_obj1.right_bottom_y) / 2. - rflysim_p.rgb_cnt.y;
            cmd.velocity.y =
                -dx * kx;  //至于方向，需要确定下视单目图像坐标系与机体坐标系的关系
            cmd.velocity.x = dy * ky;
            vel.x = dy * ky;
            vel.y = -dx * kx;
            setOutput("arucoVel", vel);

            if(fcu_pose.pose.position.z < rflysim_p.hight_max)
                cmd.velocity.z = 0.1;  //继续上升以便识别二维码
            else
            {  
                //这种情况放弃识别二维码
                log_file_ << "Recognize_aruco FAILURE.\n";
                return BT::NodeStatus::FAILURE;
            }
            cmd.header.stamp = ros::Time::now();
        }

        cmd_pub.publish(cmd);
        ros::spinOnce();
        cmd.velocity.y = 0;
        cmd.velocity.z = 0; 
        log_file_ << "Recognize_aruco finished: [ " << fcu_pose.pose.position.x << ", " << fcu_pose.pose.position.y << ", " << fcu_pose.pose.position.z << ", " << rad2deg(current_yaw) << "]\n";
        return BT::NodeStatus::SUCCESS;
    }

    // 终止当前动作
    virtual void halt() override
    {
        ROS_INFO("Recognize_aruco action halted");
        log_file_ << "Recognize_aruco action halted.\n";
        is_finished = false;
    }

private:
    Position3D vel;    
    bool is_finished;     // 动作是否完成
};


class Recognize_H : public BT::ActionNodeBase 
{
public:
    // 构造函数，传递名称和节点配置
    Recognize_H(const std::string& name, const BT::NodeConfiguration& config)
        : BT::ActionNodeBase(name, config), is_finished(false)
    {
          //如果到识别二维码的状态，可能二维码没在下视相机视场角内，这个时候应该飞高一点，使二维码或者降落标志在视场内
        cmd.coordinate_frame = mavros_msgs::PositionTarget::FRAME_BODY_NED;
        cmd.type_mask =
            ~uint16_t(0) & ~(uint16_t(0xff) << 12);  //最后结果为0000 1111 1111 1111
        cmd.type_mask &= (~uint16_t(mavros_msgs::PositionTarget::FORCE));  //把 FORCE 屏蔽掉
        cmd.type_mask &= ~(mavros_msgs::PositionTarget::IGNORE_VX | mavros_msgs::PositionTarget::IGNORE_VY
                            | mavros_msgs::PositionTarget::IGNORE_VZ);  // 使用速度控制；
        cmd.header.frame_id = "base_link";
    }

    // 设置节点的输入输出端口
    static BT::PortsList providedPorts()
    {

        const char* outputDesc = "Output detected vel";
        return {
            BT::OutputPort<Position3D>("HVel", outputDesc),
        };
    }

    // 核心逻辑，处理动作的执行
    virtual BT::NodeStatus tick() override
    {
        //如果识别出了二维码，没扫描到降落表示，可能降落标志没在视场内，此时应飞高，并往识别了二维码的位置飞去
        // // cmd.coordinate_frame = Command::FRAME_LOCAL_NED;
        // cmd.type_mask =
        //     ~uint16_t(0) & ~(uint16_t(0xff) << 12);  //最后结果为0000 1111 1111 1111
        // cmd.type_mask &= (~uint16_t(Command::FORCE));  //把 FORCE 屏蔽掉
        // cmd.type_mask &= ~(Command::IGNORE_VX | Command::IGNORE_VY
        //                     | Command::IGNORE_VZ);  // 使用速度控制；
        // cmd.coordinate_frame = Command::FRAME_BODY_NED;
        if(!is_H)
        {  
            //没识别到降落标志
            static  bool is_cnt_aurco = false;
            std::cout << " aruco: " << aruco.cnt_x << ", " << aruco.cnt_y
                    << std::endl;
            std::cout << " rgb_cnt: " << rflysim_p.rgb_cnt.x << ", "
                    << rflysim_p.rgb_cnt.y << std::endl;
            int    dx = int(aruco.cnt_x - rflysim_p.rgb_cnt.x);
            int    dy = int(aruco.cnt_y - rflysim_p.rgb_cnt.y);
            double vx = -dy * kx;
            double vy = -dx * ky;
            if(std::abs(vx) > vx_max)
            vx = vx / std::abs(vx) * vx_max;
            if(std::abs(vy) > vy_max)
            vy = vy / std::abs(vy) * vy_max;
            std::cout << "dx: " << dx << ", dy: " << dy << std::endl;
            std::cout << "vx: " << cmd.velocity.x << ", vy: " << cmd.velocity.y
                    << std::endl;
            cmd.velocity.x   = vx;
            cmd.velocity.y   = vy;
            cmd.velocity.z   = 0.05;
            vel.x = vx;
            vel.y = vy;
            vel.z = 0.05;
            setOutput("HVel", vel);

            if(fcu_pose.pose.position.z > rflysim_p.hight_max)
            {  
            //此时高度，不能再高了,此时降落表示应该识别了
            ROS_ERROR("vichle'z is too height");
            cmd.velocity.z = 0;
            }

            if(std::abs(dx) < 10 && std::abs(dy) < 10)
            {
            is_cnt_aurco = true;
            }
            if(is_cnt_aurco)
            {
                cmd.velocity.x = 0.05;
            }
            cmd.header.stamp = ros::Time::now();
            cmd_pub.publish(cmd);
        }
        is_H = false;    
        log_file_ << "Recognize_H finished: [ " << fcu_pose.pose.position.x << ", " << fcu_pose.pose.position.y << ", " << fcu_pose.pose.position.z << ", " << rad2deg(current_yaw) << "]\n";
        return BT::NodeStatus::SUCCESS;
    }

    // 终止当前动作
    virtual void halt() override
    {
        ROS_INFO("DetectArUco action halted");
        log_file_ << "DetectArUco action halted.\n";
        is_finished = false;
    }

private:
    Position3D vel;    
    bool is_finished;     // 动作是否完成
};

// class FindFrame : public BT::ActionNodeBase 
// {
// public:
//     // 构造函数，传递名称和节点配置
//     FindFrame(const std::string& name, const BT::NodeConfiguration& config)
//         : BT::ActionNodeBase(name, config), is_finished(false)
//     {
//             cmd.coordinate_frame = mavros_msgs::PositionTarget::FRAME_LOCAL_NED;
//             cmd.type_mask = mavros_msgs::PositionTarget::IGNORE_YAW;
//     }

//     // 设置节点的输入输出端口
//     static BT::PortsList providedPorts()
//     {}

//     // 核心逻辑，处理动作的执行
//     virtual BT::NodeStatus tick() override
//     {
//         for(float i = 0.1;i<3.1416;i=i+0.1){
//             cmd.yaw = current_yaw + i;
//             cmd.header.stamp = ros::Time::now();
//             cmd_pub.publish(cmd);
//             ros::spinOnce();
//             ros::Rate rate(20.0);
//             rate.sleep();                
//             sleep(0.1);
//             if(is_frame){
//                 is_frame = false;
//                 return BT::NodeStatus::SUCCESS;
//             }
//         }
//         return BT::NodeStatus::FAILURE;
//     }

//     // 终止当前动作
//     virtual void halt() override
//     {
//         ROS_INFO("DetectArUco action halted");
//         log_file_ << "DetectArUco action halted.\n";
//         is_finished = false;
//     }

// private:
//     Position3D vel;    
//     bool is_finished;     // 动作是否完成
// };

class FindFrame : public BT::ActionNodeBase 
{
public:
    FindFrame(const std::string& name, const BT::NodeConfiguration& config)
        : BT::ActionNodeBase(name, config), is_finished(false)
    {
        cmd.coordinate_frame = mavros_msgs::PositionTarget::FRAME_LOCAL_NED;
        cmd.type_mask = mavros_msgs::PositionTarget::IGNORE_YAW;
    }

    static BT::PortsList providedPorts()
    {
        return {}; // 预留，如果不需要端口，可以去掉这个函数
    }

    virtual BT::NodeStatus tick() override
    {
        ros::Rate rate(20.0); 
        for(float i = 0.1; i < 3.1416; i += 0.1) {
            cmd.yaw = current_yaw + i;
            cmd.header.stamp = ros::Time::now();
            cmd_pub.publish(cmd);
            ros::spinOnce();
            ros::Duration(0.2).sleep();
            rate.sleep(); // 使用创建的 Rate 对象

            if(is_frame) {
                is_frame = false;
                return BT::NodeStatus::SUCCESS;
            }
        }
        return BT::NodeStatus::FAILURE;
    }

    virtual void halt() override
    {
        ROS_INFO("FindFrame action halted");
        log_file_ << "FindFrame action halted.\n";
        // is_finished 的设置可以视具体逻辑而定
    }

private:
    Position3D vel;    //预留
    bool is_finished; 
};




class ForwardDetectFrame : public BT::ActionNodeBase 
{
public:
    // 构造函数，传递名称和节点配置
    ForwardDetectFrame(const std::string& name, const BT::NodeConfiguration& config)
        : BT::ActionNodeBase(name, config), is_finished(false)
    {}

    // 设置节点的输入输出端口
    static BT::PortsList providedPorts()
    {
        // const char* inputDesc = "Input class type...";
        const char* outputDesc = "Output detected details: target category, ID, coordinates, confidence, and other info.";
        return {
            // BT::InputPort<std::string>("classtype", inputDesc),

            BT::OutputPort<ObjsData3D>("ForwardDetectFrameData", outputDesc),
            BT::OutputPort<Position3D>("FramePosition", outputDesc)
        };
    }

    // 核心逻辑，处理动作的执行
    virtual BT::NodeStatus tick() override
    {
        std::string class_type;
        // if (!getInput<std::string>("classtype", class_type)) {
        //     return BT::NodeStatus::FAILURE;
        // }
        ROS_INFO("ForwardDetectFrame START");
        // 检查是否有检测结果
        if (is_finished || forward_objs.objects.empty()) {
            log_file_ << "ForwardDetectFrame FAILURE. There is no object to be detected within the field of view of the front camera.\n";
            return BT::NodeStatus::FAILURE;
        }

        if(!rflysim_p.is_sim)
        {  
            //如果是真机模式不需要使用深度读取
            ObjsConterRGB(forward_objs, &cnt, objsdata3D);
            ROS_INFO("camera corr p: x:%f, y:%f, z:%f",cnt.x,cnt.y,cnt.z);
            pcl::PointCloud<pcl::PointXYZ>::Ptr tmp(
            new pcl::PointCloud<pcl::PointXYZ>());
            //因为直接使用的检测框作为目标框，得加上一个往前的偏移量
            //cnt.z += rflysim_p.goal_x_t;
            ROS_INFO("cnt.z:%f goal_x_t:%f",cnt.z,rflysim_p.goal_x_t);
            tmp->points.push_back(cnt);
            CoordinateTrans(tmp);
            VisionPointCloud(tmp,vis_pub);
            /*
            Position3D relevant_point;
            relevant_point.x = tmp->points[0].x;
            relevant_point.y = tmp->points[0].y;
            relevant_point.z = tmp->points[0].z;
            log_file_ << "ForwardDetectFrame relevant_point: [ " << relevant_point.x << ", " << relevant_point.y << ", " << relevant_point.z << "]\n";
            ned2nwu(&relevant_point);

            point3D.x = relevant_point.x + fcu_pose.pose.position.x;
            point3D.y = relevant_point.y + fcu_pose.pose.position.y;
            point3D.z = relevant_point.z + fcu_pose.pose.position.z;
            */
            point3D.x = tmp->points[0].x;
            point3D.y = tmp->points[0].y;
            point3D.z = tmp->points[0].z;
            log_file_ << "ForwardDetectFrame frame position [nwu]:(" << point3D.x << ", " << point3D.y << ", " << point3D.z << ")\n";
            std::cout<<"Detect frame NWU position x:"<<point3D.x<<" y:"<<point3D.y<<" z:"<<point3D.z<<std::endl;
            setOutput("FramePosition", point3D);

            objsdata3D.x = tmp->points[0].x;
            objsdata3D.y = tmp->points[0].y;
            objsdata3D.z = tmp->points[0].z;
            goal_point.pose.position.x = tmp->points[0].x;
            goal_point.pose.position.y = tmp->points[0].y;
            goal_point.pose.position.z = tmp->points[0].z;
            ROS_INFO("ned goal: x:%f,y:%f,z:%f",goal_point.pose.position.x,goal_point.pose.position.y,goal_point.pose.position.z);
            setOutput("ForwardDetectFrameData", objsdata3D);
            // goal_pub.publish(goal_1);
            // setOutput("ForwardDetectFrameData", objsdata3D);
            log_file_ << "ForwardDetectFrame finished. drone pose [nwu,yaw]:(" << fcu_pose.pose.position.x << ", " << fcu_pose.pose.position.y << ", " << fcu_pose.pose.position.z << ", " << rad2deg(current_yaw) << ") \n";
            return BT::NodeStatus::SUCCESS;
        }        
        //这里需要做坐标转换，从RGB像素到深度图像里面的像素
        {  //需要这么几个矩阵，两个相机的畸变矫正矩阵，RGB相机到深度相机的变换矩阵，因为仿真里面没有畸变，也不产生旋转与平移，那么可以直接计算。
            if(depth_queue.empty())
            {
                ROS_WARN("current depth queue is empty!");
                return BT::NodeStatus::RUNNING;
            }
            //找与图像目标检测时间最近的深度图
            bool                         is_find_data = false;
            std::unique_lock<std::mutex> lock(depth_mtx);
            while(depth_queue.size() > 0)

            {
            auto   front = depth_queue.front();
            double dt    = front.header.stamp.toSec() - forward_objs.header.stamp.toSec();
            if(std::abs(dt) < 0.1)
            {
                depth = front;
                depth_queue.pop();
                is_find_data = true;
                break;
            }
            else
            {
                ROS_WARN("depth_queue size: %d, %f ", depth_queue.size(), dt);
                depth_queue.pop();
            }
            if(depth_queue.empty())
            {

                // ROS_WARN("not find the stamp recentest depth image");
                // sleep(0.1); // 睡眠一个时间，让深度图赋值
                return BT::NodeStatus::RUNNING;
            }
            }
            lock.unlock();
            if(is_find_data == false)
            {
                log_file_ << "ForwardDetectFrame FAILURE. is_find_data false\n";
                return BT::NodeStatus::FAILURE;
            }
        }
        std::cout << " find recent depth image: " << depth.width << ", "
                    << depth.height << std::endl;

        if(rflysim_p.enable)
        {
            pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(
            new pcl::PointCloud<pcl::PointXYZ>());
            static auto scale = rflysim_p.f_depth / rflysim_p.f_rgb;

            cv_bridge::CvImagePtr cv_ptr;
            try
            {
            cv_ptr =
                cv_bridge::toCvCopy(depth, sensor_msgs::image_encodings::TYPE_16UC1);
            }
            catch(cv_bridge::Exception &e)
            {
                ROS_ERROR("cv_bridge exception: %s", e.what());
                log_file_ << "ForwardDetectFrame FAILURE. cv_bridge exception\n";
                return BT::NodeStatus::FAILURE;
            }
            cv::Mat depth_img = cv_ptr->image;
            //    cv::imshow("depth", depth_img);
            //    cv::waitKey(0);
            for(size_t i = 0; i < forward_objs.objects.size(); ++i)
            {
            if(forward_objs.objects[i].score < rflysim_p.min_score)
            {  //置信度低的不做考虑，按原指令执行
                continue;
            }
            //截取深度图部分点云
            std::cout << "det left_top: " << forward_objs.objects[i].left_top_x << ", "
                        << forward_objs.objects[i].left_top_y << std::endl;
            std::cout << "det right_bottom: " << forward_objs.objects[i].right_bottom_x
                        << ", " << forward_objs.objects[i].right_bottom_y << std::endl;
            std::cout << "rgb_cnt: " << rflysim_p.rgb_cnt.x << ","
                        << rflysim_p.rgb_cnt.y << std::endl;
            std::cout << "depth_cnt: " << rflysim_p.depth_cnt.x << ","
                        << rflysim_p.depth_cnt.y << std::endl;

            int dx_left  = forward_objs.objects[i].left_top_x - rflysim_p.rgb_cnt.x;
            int dy_left  = forward_objs.objects[i].left_top_y - rflysim_p.rgb_cnt.y;
            int dx_right = forward_objs.objects[i].right_bottom_x - rflysim_p.rgb_cnt.x;
            int dy_right = forward_objs.objects[i].right_bottom_y - rflysim_p.rgb_cnt.y;

            cv::Point2i left;
            cv::Point2i right;
            left.x  = int(scale * dx_left + rflysim_p.depth_cnt.x);
            left.y  = int(scale * dy_left + rflysim_p.depth_cnt.y);
            right.x = int(scale * dx_right + rflysim_p.depth_cnt.x);
            right.y = int(scale * dy_right + rflysim_p.depth_cnt.y);
            std::cout << " lddeft: " << left << " , right: " << right << std::endl;
            if(left.x < 0 || left.x >= depth_img.cols || left.y < 0
                || left.y >= depth_img.rows || right.x < 0 || right.x >= depth_img.cols
                || right.y < 0 || right.y >= depth_img.cols)
            {
                ROS_ERROR("please check params is correct");
                log_file_ << "ForwardDetectFrame FAILURE. please check params is correct\n";
                return BT::NodeStatus::FAILURE;
            }

            pcl::PointCloud<pcl::PointXYZ>::Ptr tmp(
                new pcl::PointCloud<pcl::PointXYZ>());

            {
                ObjsConterRGB(forward_objs, &cnt, objsdata3D);
                ROS_ERROR("rgb obj cnt: x:%f,y:%f,z:%f", cnt.x, cnt.y, cnt.z);
                tmp->points.push_back(cnt);
                CoordinateTrans(tmp);
                VisionPointCloud(tmp, vis_pub);

                pcl::PointCloud<pcl::PointXYZ>::Ptr full(
                new pcl::PointCloud<pcl::PointXYZ>());

                DepthImgToCloud(full, &depth_img, cv::Point2i(0, 0), cv::Point2i(0, 0));
                CoordinateTrans(full);
                VisionPointCloud(full, full_points_pub);
            }
            // return BT::NodeStatus::RUNNING;
            cv::Point3d frame_cnt;
            if(Cluster(tmp, &frame_cnt))
            {  
                //当前仅仅是传感器坐标系的坐标标，应该转换成世界坐标系下的坐标
                //具体操作如下，1.把传感器坐标系的坐标转到机体坐标系,2.通过飞机位姿获得旋转与平移,将机体坐标系下的目标点位置转换成世界坐标系的位置
                std::cout << frame_cnt << std::endl;
                //对这个点进行坐标转；
                //获取飞机的在全局坐标系下的位姿
                std::vector<double> R = rflysim_p.depth_cam2body_R;
                std::vector<double> T = rflysim_p.depth_cam2body_T;
                double              x =
                frame_cnt.x * R[0] + frame_cnt.y * R[1] + frame_cnt.z * R[2] + T[0];
                double y =
                frame_cnt.x * R[3] + frame_cnt.y * R[4] + frame_cnt.z * R[5] + T[1];
                double z =
                frame_cnt.x * R[6] + frame_cnt.y * R[7] + frame_cnt.z * R[8] + T[2];
                std::cout << ">>>>>>>>>>>>>>bc: " << x << "," << y << "," << z
                        << std::endl;
                //从body坐标系到世界坐标系
                tf2::Vector3    bc(x, y, z);
                tf2::Quaternion q;
                tf2::convert(fcu_pose.pose.orientation, q);
                tf2::Vector3 t;
                tf2::convert(fcu_pose.pose.position, t);

                tf2::Transform trans;
                trans.setOrigin(t);
                trans.setRotation(q);
                tf2::Vector3  wc = trans * bc;
                pcl::PointXYZ cnt_p;
                cnt_p.x = bc.getX();
                cnt_p.y = bc.getY();
                cnt_p.z = bc.getZ();


                point3D.x = wc.getX() + fcu_pose.pose.position.x;
                point3D.y = wc.getY() + fcu_pose.pose.position.y;
                point3D.z = wc.getZ() + fcu_pose.pose.position.z;
                setOutput("FramePosition", point3D);


                objsdata3D.x = wc.getX();
                objsdata3D.y = wc.getY();
                objsdata3D.z = wc.getZ();

                goal_point.pose.position.x = wc.getX();
                goal_point.pose.position.y = wc.getY();
                goal_point.pose.position.z = wc.getZ();
                setOutput("ForwardDetectFrameData", objsdata3D);
                // goal_pub.publish(goal_1);

            }
            else
            {
                ROS_ERROR("cluser fail");
            }
            }
        }
        std::cout << ">>>>>>>>frame position in world: " << goal_point.pose.position.x
                    << "," << goal_point.pose.position.y << ","
                    << goal_point.pose.position.z << std::endl;
                log_file_<< std::fixed << std::setprecision(2) << "ForwardDetectFrame finished. drone pose [nwu,yaw]:" << fcu_pose.pose.position.x << ", " << fcu_pose.pose.position.y << ", " << fcu_pose.pose.position.z << ", " << rad2deg(current_yaw) << ")\n";
                log_file_ << std::fixed << std::setprecision(2) << "ForwardDetectFrame finished. frame position [nwu]:(" << point3D.x << ", " << point3D.y << ", " << point3D.z << ")\n";
            
                return BT::NodeStatus::SUCCESS;
    }

    // 终止当前动作
    virtual void halt() override
    {
        ROS_INFO("ForwardDetectFrameData action halted");
        log_file_ << "ForwardDetectFrameData action halted\n";
        is_finished = false;
    }

private:
    ObjsData3D objsdata3D;
    pcl::PointXYZ cnt;
    sensor_msgs::Image depth;
    Position3D point3D;
    bool is_finished;     // 动作是否完成
};


class DownDetectH : public BT::ActionNodeBase 
{
public:
    // 构造函数，传递名称和节点配置
    DownDetectH(const std::string& name, const BT::NodeConfiguration& config)
        : BT::ActionNodeBase(name, config), is_finished(false)
    {}

    // 设置节点的输入输出端口
    static BT::PortsList providedPorts()
    {
        // const char* inputDesc = "Input class type...";
        const char* outputDesc = "Output detected details: target category, ID, coordinates, confidence, and other info.";
        return {
            // BT::InputPort<std::string>("classtype", inputDesc),
            BT::OutputPort<ObjData>("DownDetectHData", outputDesc),
            BT::OutputPort<Dxy>("DownDetectHDxy", outputDesc),
            BT::OutputPort<Position3D>("SendVelocity", outputDesc)
        };
    }

    // 核心逻辑，处理动作的执行
    virtual BT::NodeStatus tick() override
    {
        ROS_INFO("DownDetectH START");
        
        std::string class_type;
        // if (!getInput<std::string>("classtype", class_type)) {
        //     return BT::NodeStatus::FAILURE;
        // }

        // 检查是否有检测结果
        if (is_finished || down_objs.objects.empty()) {
            log_file_ << "DownDetectH FAILURE. down_objs empty\n";
            return BT::NodeStatus::FAILURE;
        }

        for(int i = 0; i < down_objs.objects.size(); ++i)
        {
            if(down_objs.objects[i].class_name == "land"
                && down_objs.objects[i].score > rflysim_p.min_score)
            {
                // is_H  = true;
                auto obj = down_objs.objects[i];

                objdata.class_name = obj.class_name;
                objdata.class_id = i;
                objdata.left_top_x = forward_obj.left_top_x;
                objdata.left_top_y = forward_obj.left_top_y;
                objdata.right_bottom_x = forward_obj.right_bottom_x;
                objdata.right_bottom_y = forward_obj.right_bottom_y;                
                objdata.coordinates.x = (obj.left_top_x + obj.right_bottom_x) / 2.0;
                objdata.coordinates.y = (obj.left_top_y + obj.right_bottom_y) / 2.0;      
                objdata.score = obj.score;

                auto dx =
                (obj.left_top_x + obj.right_bottom_x) / 2. - rflysim_p.rgb_cnt.x;
                auto dy =
                (obj.left_top_y + obj.right_bottom_y) / 2. - rflysim_p.rgb_cnt.y;
                dxy.dx = dx;
                dxy.dy = dy;
                std::cout << "land dx:  " << dx << " dy: " << dy << std::endl;
                double vx = -dy * kx;
                double vy = -dx * ky;
                if(std::abs(vx) > vx_max)
                vx = vx / std::abs(vx) * vx_max;
                if(std::abs(vy) > vy_max)
                vy = vy / std::abs(vy) * vy_max;
                
                vel.x = vx;
                vel.y = vy;
                vel.z = 0;
                setOutput("SendVelocity", vel);
                setOutput("DownDetectHDxy", dxy);
                setOutput("DownDetectHData", objdata);
                // cmd.velocity.x = vx;
                // cmd.velocity.y = vy;
                // std::cout << "vx: " << cmd.velocity.x << ", vy: " << cmd.velocity.y
                //         << std::endl;
                // if(std::abs(dx) < 10 && std::abs(dy) < 10)
                // {
                // //cmd.velocity.z = -0.5;  // 符合降落条件了
                // set_mode_client.call(land);
                // }
                // cmd.header.stamp = ros::Time::now();
                log_file_ << "DownDetectH finished: [ " << fcu_pose.pose.position.x << ", " << fcu_pose.pose.position.y << ", " << fcu_pose.pose.position.z << ", " << rad2deg(current_yaw) << "]\n";
                return BT::NodeStatus::SUCCESS;
            }
        };

        // return BT::NodeStatus::SUCCESS;
        // return BT::NodeStatus::RUNNING;
        ROS_INFO("DownDetectH failed");
        log_file_ << "DownDetectH failed: [ " << fcu_pose.pose.position.x << ", " << fcu_pose.pose.position.y << ", " << fcu_pose.pose.position.z << ", " << rad2deg(current_yaw) << "]\n";        
        return BT::NodeStatus::FAILURE;
    }

    // 终止当前动作
    virtual void halt() override
    {
        ROS_INFO("DownDetect2D action halted");
        is_finished = false;
    }

private:
    ObjData objdata;
    Dxy dxy;
    Position3D vel;

    bool is_finished;     // 动作是否完成
};


// class HLand : public BT::ActionNodeBase
// {
// public:
//     HLand(const std::string& name, const BT::NodeConfiguration& config)
//         : BT::ActionNodeBase(name, config), is_finished(false)
//     {}

//     // 定义输入输出端口
//     static BT::PortsList providedPorts()
//     {
//         // const char* description = "land to H";
//         // return { BT::InputPort<Distance>("HLand", description) };
//         const char* inputDesc = "Input class type...";

//         return {BT::InputPort<ObjData>("HlandData", outputDesc),
//         BT::InputPort<Dxy>("HlandDxy", outputDesc)};
//     }

//     // tick() 方法会在每次更新时调用
//     virtual BT::NodeStatus tick() override
//     {
//         auto res = getInput<ObjData>("HlandData");
//         if (!res)
//         {
//             throw BT::RuntimeError("error reading port [HlandData]:", res.error());
//         }
//         objdata = res.value();
//         land.request.custom_mode = "AUTO.LAND";
//         ROS_INFO("HLand START");
//         // 检查是否有检测结果
//         if (is_finished || down_objs.objects.empty()) {
//             return BT::NodeStatus::FAILURE;
//         }

//         // for(int i = 0; i < down_objs.objects.size(); ++i)
//         // {
//         //     if(down_objs.objects[i].class_name == "land"
//         //         && down_objs.objects[i].score > rflysim_p.min_score)
//         //     {
//         //         auto obj = down_objs.objects[i];

//         //         objdata.class_name = obj.class_name;
//         //         objdata.class_id = i;
//         //         objdata.coordinates.x = (obj.left_top_x + obj.right_bottom_x) / 2.0;
//         //         objdata.coordinates.y = (obj.left_top_y + obj.right_bottom_y) / 2.0;      
//         //         objdata.score = obj.score;

//         //         auto dx =
//         //         (obj.left_top_x + obj.right_bottom_x) / 2. - rflysim_p.rgb_cnt.x;
//         //         auto dy =
//         //         (obj.left_top_y + obj.right_bottom_y) / 2. - rflysim_p.rgb_cnt.y;
//         //         std::cout << "land dx:  " << dx << " dy: " << dy << std::endl;
//         //         double vx = -dy * kx;
//         //         double vy = -dx * ky;
//         //         if(std::abs(vx) > vx_max)
//         //         vx = vx / std::abs(vx) * vx_max;
//         //         if(std::abs(vy) > vy_max)
//         //         vy = vy / std::abs(vy) * vy_max;
                

//         //         cmd.velocity.x = vx;
//         //         cmd.velocity.y = vy;
//         //         std::cout << "vx: " << cmd.velocity.x << ", vy: " << cmd.velocity.y
//         //                 << std::endl;
//         //         if(std::abs(dx) < 10 && std::abs(dy) < 10)
//         //         {
//         //             //cmd.velocity.z = -0.5;  // 符合降落条件了
//         //             set_mode_client.call(land);
//         //         }
//         //         cmd.header.stamp = ros::Time::now();
//         //         return BT::NodeStatus::SUCCESS;
//         //     }
//         // }

//         ROS_INFO("HLand finished");
//         return BT::NodeStatus::FAILURE;
//     }

//     // 当行为树终止该动作时调用halt()方法
//     virtual void halt() override
//     {
//         ROS_INFO("Land action halted");
//         is_finished = false;  // 重置任务状态
//     }

// private:
//     ros::Time last_command_time;
//     ObjData objdata;
//     bool is_finished;  // 标识任务是否完成
// };

class HLand : public BT::ActionNodeBase
{
public:
    HLand(const std::string& name, const BT::NodeConfiguration& config)
        : BT::ActionNodeBase(name, config), is_finished(false)
    {}

    // 定义输入输出端口
    static BT::PortsList providedPorts()
    {
        const char* inputDesc = "Input dx dy";
        return {BT::InputPort<Dxy>("HDxy", inputDesc)};  
    }

    // tick() 方法会在每次更新时调用
    virtual BT::NodeStatus tick() override
    {
        
        auto res = getInput<Dxy>("HDxy");
        if (!res)
        {
            throw BT::RuntimeError("error reading port [HDxy]:", res.error());
        }
        target = res.value();
        printf("HDxy Target: [ %.1f, %.1f, %.1f]\n", target.dx, target.dy);
        log_file_ << "HDxy target: [ " << target.dx << ", " << target.dy <<"]\n";

        if (!is_finished)
        {
            // 切换到自动降落模式
            land.request.custom_mode = "AUTO.LAND";

            if(std::abs(target.dx) < 10 && std::abs(target.dy) < 10)
            {
                ROS_INFO("HLand finished");
                log_file_ << "HLand finished: [ " << fcu_pose.pose.position.x << ", " << fcu_pose.pose.position.y << ", " << fcu_pose.pose.position.z << ", " << rad2deg(current_yaw) << "]\n";
                //cmd.velocity.z = -0.5;  // 符合降落条件了
                set_mode_client.call(land);
                is_finished = true;
                return BT::NodeStatus::SUCCESS;
            }
        }


        ROS_INFO("HLand failed");
        log_file_ << "HLand failed: [ " << fcu_pose.pose.position.x << ", " << fcu_pose.pose.position.y << ", " << fcu_pose.pose.position.z << ", " << rad2deg(current_yaw) << "]\n";
        return BT::NodeStatus::FAILURE;
    }

    // 当行为树终止该动作时调用halt()方法
    virtual void halt() override
    {
        ROS_INFO("HLand action halted");
        log_file_ << "HLand action halted. \n";
        is_finished = false;  // 重置任务状态
    }

private:
    ros::Time last_command_time;
    Dxy target;
    bool is_finished;  // 标识任务是否完成
};


class ControlHeigth : public BT::ActionNodeBase
{
public:
    ControlHeigth(const std::string& name, const BT::NodeConfiguration& config)
        : BT::ActionNodeBase(name, config), is_finished(false)
    {}

    // 定义输入输出端口
    static BT::PortsList providedPorts()
    {
        return {};  
    }

    // tick() 方法会在每次更新时调用
    virtual BT::NodeStatus tick() override
    {
        
        if(fcu_pose.pose.position.z > rflysim_p.hight_max)
        {  //此时高度，不能再高了,此时降落表示应该识别了
          ROS_ERROR("vichle'z is too height");
          cmd.velocity.x = 0;
          cmd.velocity.y = 0;
          cmd.velocity.z = 0;
          cmd_pub.publish(cmd);
          return BT::NodeStatus::SUCCESS;
        }



        ROS_INFO("ControlHeigth failed");
        log_file_ << std::fixed << std::setprecision(2)<< "ControlHeigth failed: [ " << fcu_pose.pose.position.x << ", " << fcu_pose.pose.position.y << ", " << fcu_pose.pose.position.z << ", " << rad2deg(current_yaw) << "]\n";
        // return BT::NodeStatus::FAILURE;
        return BT::NodeStatus::RUNNING;
    }

    // 当行为树终止该动作时调用halt()方法
    virtual void halt() override
    {
        ROS_INFO("ControlHeigth action halted");
        log_file_ << "ControlHeigth action halted. \n";
        is_finished = false;  // 重置任务状态
    }

private:
    ros::Time last_command_time;
    Dxy target;
    bool is_finished;  // 标识任务是否完成
};




int main(int argc, char **argv)
{
    ROS_INFO("befor befor befor befor 000000000000000000000000");
    ros::init(argc, argv, "offb_node");
    ros::NodeHandle nh;
    std::string depth_topic = "/camera/depth/image_raw";
    std::string bt_log_file = "/home/nvidia/catkin_ws/src/RFLYBT/controller/config/BTlog.txt";
    std::string bt_tree_path = "/home/nvidia/catkin_ws/test.xml";
    nh.param("depth_topic", depth_topic, std::string("/camera/depth/image_raw"));
    nh.param("bt_log_file", bt_log_file, std::string("/home/nvidia/catkin_ws/src/RFLYBT/controller/config/BTlog.txt"));
    // nh.param("bt_tree_path", bt_tree_path, std::string("/home/nvidia/catkin_ws/src/RFLYBT/controller/config/default.xml"));
    nh.param("cameras_param/rgb_K", cam.rgb_K, std::vector<double>());
    nh.param("cameras_param/depth_K", cam.depth_K, std::vector<double>());
    nh.param("cameras_param/rgb2depth_R", cam.rgb2depth_R,
                std::vector<double>());
    nh.param("cameras_param/rgb2depth_T", cam.rgb2depth_T,
                std::vector<double>());
    nh.param("rflysim/enable", rflysim_p.enable, true);
    nh.param("rflysim/f_rgb", rflysim_p.f_rgb, 320.);
    nh.param("rflysim/f_depth", rflysim_p.f_depth, 320.);
    nh.param("rflysim/rgb_image_w", rflysim_p.rgb_image_width, 640);
    nh.param("rflysim/rbg_image_h", rflysim_p.rgb_image_height, 480);
    nh.param("rflysim/depth_image_w", rflysim_p.depth_image_width, 640);
    nh.param("rflysim/depth_image_h", rflysim_p.depth_image_height, 480);
    nh.param<std::vector<double>>(
        "rflysim/cam2body_R", rflysim_p.depth_cam2body_R, std::vector<double>());
    nh.param<std::vector<double>>(
        "rflysim/cam2body_T", rflysim_p.depth_cam2body_T, std::vector<double>());
    nh.param("rflysim/depth_down_sample", rflysim_p.depth_down_sample, 5);
    nh.param("auto_arming", auto_arming, true);
    nh.param("takeoff_h", takeoff_h, 0.5);
    nh.param("takeoff_yaw", takeoff_yaw, 0.);
    nh.param("kx", kx, 0.0);
    nh.param("ky", ky, 0.0);
    nh.param("vx_max", vx_max, 0.0);
    nh.param("vy_max", vy_max, 0.0);
    nh.param("/rflysim/goal_x_t", rflysim_p.goal_x_t, 0.);
    nh.param("hight_max", rflysim_p.hight_max, 3.0);
    nh.param("rflysim/rgb_ppx", rflysim_p.rgb_cnt.x,
            rflysim_p.rgb_image_width / 2.);
    nh.param("rflysim/rgb_ppy", rflysim_p.rgb_cnt.y,
            rflysim_p.rgb_image_height / 2.);
    nh.param("rflysim/depth_ppx", rflysim_p.depth_cnt.x,
            rflysim_p.depth_image_width / 2.);
    nh.param("rflysim/depth_ppy", rflysim_p.depth_cnt.y,
            rflysim_p.rgb_image_height / 2.);
    nh.param("rflysim/min_score", rflysim_p.min_score, 0.7);
    nh.param("rflysim/rgb_fov_h", rflysim_p.rgb_fov_h, 90.);
    nh.param("rflysim/rgb_fov_v", rflysim_p.rgb_fov_v, 90.);
    nh.param("rflysim/is_sim", rflysim_p.is_sim, true);
    nh.param("is_S", rflysim_p.is_S, false);    

    rflysim_p.rgb_fov_h *= (M_PI / 180);
    rflysim_p.rgb_fov_v *= (M_PI / 180);

    // ros::Subscriber state_sub = nh.subscribe<mavros_msgs::State>
    //         ("/mavros/state", 10, state_cb);
    // ros::Publisher cmd_pub = nh.advertise<mavros_msgs::PositionTarget>
    //         ("/mavros/setpoint_raw/local", 10); //
    // ros::Subscriber pose_usb = nh.subscribe("/mavros/local_position/pose",10,PoseCB);
    // ros::ServiceClient arming_client = nh.serviceClient<mavros_msgs::CommandBool>
    //         ("/mavros/cmd/arming");
    // ros::ServiceClient set_mode_client = nh.serviceClient<mavros_msgs::SetMode>
    //         ("/mavros/set_mode");
    // ros::Publisher att_pub = nh.advertise<mavros_msgs::AttitudeTarget>("/mavros/setpoint_raw/attitude",10);

    state_sub = nh.subscribe<mavros_msgs::State>("/mavros/state", 10, state_cb);
    cmd_pub = nh.advertise<mavros_msgs::PositionTarget>("/mavros/setpoint_raw/local", 10); //

    pose_pub =nh.advertise<geometry_msgs::PoseStamped>("/mavros/vision_pose/pose", 10);

    pose_usb = nh.subscribe<geometry_msgs::PoseStamped>("/mavros/local_position/pose",100,PoseCB);
    fcu_pose_sub = nh.subscribe<nav_msgs::Odometry>("/mavros/local_position/odom",10,RecvPose);
    // fcu_pose_sub = nh.subscribe<nav_msgs::Odometry>("/mavros/local_position/odom",10,std::bind(&RecvPose, std::placeholders::_1, 2));
    tra_sub = nh.subscribe<quadrotor_msgs::PositionCommand>("/planning/pos_cmd", 10, RecvTra);
    // obj_sub = nh.subscribe("/objects", 10, RecvAiCB);

    forward_obj_sub = nh.subscribe<common_msgs::Obj>("/obj", 10, RecvForwardObjCB);
    down_obj_sub = nh.subscribe<common_msgs::Obj>("/obj2", 10, RecvDownObjCB);

    forward_objs_sub = nh.subscribe<common_msgs::Objects>("/objects", 10, RecvForwardObjectsCB);
    down_objs_sub = nh.subscribe<common_msgs::Objects>("/objects2", 10, RecvDownObjectsCB);

    fcu_state_sub = nh.subscribe<mavros_msgs::State>("/mavros/state", 10,RecvFcuState);

    // odom_sub = nh.subscribe("/Odometry", 10, RecvLIO);
    odom_sub = nh.subscribe<nav_msgs::Odometry>("/Odometry", 10, RecvLIO);

    aruco_sub = nh.subscribe<common_msgs::Aruco>("/Aruco", 10, RecvAruco);
    // depth_img_sub = nh.subscribe("/camera/depth/image_raw", 10, DepthImgCB);
    depth_img_sub = nh.subscribe<sensor_msgs::Image>(depth_topic, 10, DepthImgCB);

    vis_pub         = nh.advertise<sensor_msgs::PointCloud2>("/test", 10);
    full_points_pub = nh.advertise<sensor_msgs::PointCloud2>("/full_points", 10);


    goal_pub = nh.advertise<geometry_msgs::PoseStamped>("/move_base_simple/goal", 10);


    arming_client = nh.serviceClient<mavros_msgs::CommandBool>
            ("/mavros/cmd/arming");
    set_mode_client = nh.serviceClient<mavros_msgs::SetMode>
            ("/mavros/set_mode");
    att_pub = nh.advertise<mavros_msgs::AttitudeTarget>("/mavros/setpoint_raw/attitude",10);

    velocity_sub = nh.subscribe("/mavros/local_position/velocity_local", 10, velocityCallback);

    // pose_sub = nh.subscribe("mavros/local_position/pose", 10, poseCallback);

    ROS_INFO("befor befor 000000000000000000000000");
    //the setpoint publishing rate MUST be faster than 2Hz
    // rate(20.0);
    ros::Rate rate(20.0);
    // arm_cmd.request.value = true;
    offb_set_mode.request.custom_mode = "OFFBOARD";
    land.request.custom_mode          = "AUTO.LAND";
    arm_cmd.request.value = true;
    cmd.coordinate_frame = mavros_msgs::PositionTarget::FRAME_LOCAL_NED;
    cmd.type_mask =
        ~uint16_t(0) & ~(uint16_t(0xff) << 12);  //最后结果为0000 1111 1111 1111
    cmd.type_mask &= (~uint16_t(mavros_msgs::PositionTarget::FORCE));  //把 FORCE 屏蔽掉
    cmd.header.frame_id = "odom";    


    // 初始化日志文件
    log_file_.open(bt_log_file, std::ios::out | std::ios::app);  // 追加模式写入日志文件
    if (!log_file_.is_open())
    {
        ROS_ERROR("Failed to open log file");
    }
    std::ifstream bt_file(bt_tree_path);
    if (!bt_file)
    {
        ROS_ERROR("Failed to open BT file");
        return 1;
    }
    char ch;
    while(bt_file.get(ch))
    {
        log_file_.put(ch);
    }
    bt_file.close();
    BT::BehaviorTreeFactory factory;


    

    // wait for FCU connection
    ROS_INFO("000000000000000000000000");
    factory.registerNodeType<TakeOff>("TakeOff");
    ROS_INFO("TakeOff register end");
    factory.registerNodeType<MoveForward>("MoveForward");
    ROS_INFO("MoveForward register end");
    factory.registerNodeType<MoveBack>("MoveBack");
    ROS_INFO("MoveBack register end");
    factory.registerNodeType<MoveRight>("MoveRight");
    ROS_INFO("MoveRight register end");
    factory.registerNodeType<MoveLeft>("MoveLeft");
    ROS_INFO("MoveLeft register end");
    factory.registerNodeType<MoveUp>("MoveUp");
    ROS_INFO("MoveUp register end");
    factory.registerNodeType<MoveDown>("MoveDown");
    ROS_INFO("MoveDown register end");
    factory.registerNodeType<FlyToCoordinates>("FlyToCoordinates");  
    ROS_INFO("FlyToCoordinates register end");  
    factory.registerNodeType<SendVelocity>("SendVelocity");
    ROS_INFO("SendVelocity register end");  
    factory.registerNodeType<SetYawAngle>("SetYawAngle");
    ROS_INFO("SetYawAngle register end");  
    factory.registerNodeType<Land>("Land");
    ROS_INFO("Land register end");  
    factory.registerNodeType<ForwardDetect2D>("ForwardDetect2D");
    ROS_INFO("ForwardDetect2D register end");  
    factory.registerNodeType<DownDetect2D>("DownDetect2D");
    ROS_INFO("DownDetect2D register end");  
    factory.registerNodeType<DetectArUco>("DetectArUco");
    ROS_INFO("DetectArUco register end");  
    factory.registerNodeType<ForwardDetectFrame>("ForwardDetectFrame");
    ROS_INFO("ForwardDetectFrame register end");  
    factory.registerNodeType<DownDetectH>("DownDetectH");
    ROS_INFO("DownDetectH register end");  
    factory.registerNodeType<HLand>("HLand");
    ROS_INFO("HLand register end");  
    factory.registerNodeType<Recognize_aruco>("Recognize_aruco");
    ROS_INFO("Recognize_aruco register end");  
    factory.registerNodeType<Recognize_H>("Recognize_H");
    ROS_INFO("Recognize_H register end");  
    factory.registerNodeType<ControlHeigth>("ControlHeigth");    
    ROS_INFO("ControlHeigth register end");  
    factory.registerNodeType<FindFrame>("FindFrame");    
    ROS_INFO("FindFrame register end");     
    factory.registerNodeType<TurnRight>("TurnRight");
    ROS_INFO("TurnRight register end");
    factory.registerNodeType<TurnLeft>("TurnLeft");
    ROS_INFO("TurnLeft register end");
    ROS_INFO("000000000000000000000000 end end end ControlHeigth");
    auto tree = factory.createTreeFromFile(bt_tree_path);
    ROS_INFO("111111111111111111111111111111");

    while((!is_recv_pose || !is_recv_state) || !fcu_state.connected){
        //判断mavros`是否连接上飞控
        ros::spinOnce();
        ROS_INFO("22222222222222222222222");
        // rate.sleep()
        rate.sleep();
    };

    
    ROS_INFO("BT START");
  

    ROS_INFO("BT register end");
    tree.tickWhileRunning();


    // mavros_msgs::PositionTarget cmd;
    //是否需要控制角度与角速度，如果需要放开注释
//    cmd.type_mask &= ~mavros_msgs::PositionTarget::IGNORE_YAW;
//    cmd.type_mask &= ~mavros_msgs::PositionTarget::IGNORE_YAW_RATE;

    // 关闭日志文件
    if (log_file_.is_open())
    {
        log_file_.close();
    }


    return 0;
}
