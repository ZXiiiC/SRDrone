#ifndef CONTROLLER_H
#define CONTROLLER_H
#include <geometry_msgs/PoseStamped.h>
#include <mavros_msgs/State.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/PositionTarget.h>
#include <sensor_msgs/Image.h>

#include <common_msgs/Obj.h>
#include <common_msgs/Objects.h>
#include <common_msgs/Aruco.h>
#include <common_msgs/MissionState.h>
#include <quadrotor_msgs/PositionCommand.h>

#include <ros/ros.h>
#include <ros/timer.h>
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

using namespace BT;

class Controller
{
  enum class Mission : uint8_t
  {
    init,
    takeoff,
    cross_corridor,
    cross_frame1,
    cross_frame2,
    recognize_aruco,
    recognize_H,
    land,
    end
  };

  enum class CtrlSource : uint8_t
  {
    Servo,
    Trajectroy
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

  typedef struct cameras
  { /*暂时不做实现，后续可能改用eigen 存储*/
    //    camera() {}
    std::vector<double> rgb_K;        // rgb 相机内参矩阵
    std::vector<double> depth_K;      //深度相机内参矩阵
    std::vector<double> rgb2depth_R;  // rgb相机到深度相机的旋转矩阵
    std::vector<double> rgb2depth_T;  // rgb相机到深度相机的平移矩阵

  } CamearasParam;

  typedef mavros_msgs::PositionTarget Command;

public:
  Controller(ros::NodeHandle &nh);

  //发送控制指令以及实时判断当前飞机状态
  bool Init();
  void Run(const ros::TimerEvent &e);
  void DepthImgCB(const sensor_msgs::Image::ConstPtr &depth);
  void RecvPose(const nav_msgs::Odometry::ConstPtr &pose);
  void RecvFcuState(const mavros_msgs::State::ConstPtr &state);
  void
       RecvAiCB(const common_msgs::Objects::ConstPtr
                  &objs);  // 在这里计算框的中心位置，并计算它在全局坐标系里面的位置
  void ObjConterRGB(const common_msgs::Objects::ConstPtr objs,
                    pcl::PointXYZ *cnt);  //直接使用目标检测算结算位置
  void RecvTra(const quadrotor_msgs::PositionCommand::ConstPtr &msg);
  void RecvLIO(const nav_msgs::Odometry::ConstPtr &odom);
  void RecvAruco(const common_msgs::Aruco::ConstPtr &msg);
  // bool FeedBackCB(behavior_tree_ros::FeedBack::Request &req, behavior_tree_ros::FeedBack::Response &res);
  // bool BaseCmdCB(behavior_tree_ros::BaseCmd::Request &req, behavior_tree_ros::BaseCmd::Response &res);
  
  //行为树动作节点
  BT::NodeStatus Takeoff();
  BT::NodeStatus Land(); 
  BT::NodeStatus Cross_frame();
  BT::NodeStatus Recognize_aruco();
  BT::NodeStatus Recognize_H();
  BT::NodeStatus End(); 


  //执行动作函数定义
  int Action_Takeoff();
  int Action_Cross_corridor();
  int Action_Land(); 
  int Action_Cross_frame();
  int Action_Recognize_aruco();
  int Action_Recognize_H();
  int Action_End(); 

private:
  void StateChange();
  void CalYaw();
  void CalVelicty();
  void DepthImgToCloud(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud,
                       const cv::Mat *img, const cv::Point2i &left,
                       const cv::Point2i &right);
  bool Cluster(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, cv::Point3d *cnt);
  void CloudKeyPoint(const pcl::PointCloud<pcl::PointXYZ>::Ptr cloud);
  inline void PrintMission();
  void        VisionPointCloud(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud,
                               const ros::Publisher &              pub);
  void VisionPointCloud(const pcl::PointXYZ *p, const ros::Publisher &pub);
  bool RePlanReq();
  void CoordinateTrans(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud);

private:
  ros::NodeHandle nh_;

  std::string current_task; //，它的值之子接受行为书指令那里赋值
  // declear ros subcriver and publisher
  ros::Subscriber ret_ai_sub;
  ros::Subscriber depth_img_sub;
  ros::Subscriber fcu_state_sub;
  ros::Subscriber fcu_pose_sub;
  ros::Subscriber obj_sub;
  ros::Publisher  ctrl_cmd_pub;
  ros::Subscriber tra_sub;
  ros::Publisher  goal_pub;
  ros::Publisher  vis_pub;
  ros::Publisher  full_points_pub;
  ros::Timer      run_timer;
  ros::Publisher  pose_pub;
  ros::Publisher  task_state_pub;
  ros::Subscriber odom_sub;
  ros::Subscriber aruco_sub;

  ros::ServiceClient set_mode_client;
  ros::ServiceClient arming_client;

  ros::ServiceServer  feedback_server;
  ros::ServiceServer  cmd_server;

  common_msgs::Aruco aruco;
  common_msgs::Obj   obj;  //用来保存/land 标志

  geometry_msgs::PoseStamped send_local_pose;

  geometry_msgs::PoseStamped goal_point;
  geometry_msgs::PoseStamped goal_1;
  geometry_msgs::PoseStamped goal_2;
  int                        goal_flag;
  int                        send_goal_flag;
  geometry_msgs::PoseStamped fcu_pose;
  bool                       is_rec_frame1;
  bool                       is_rec_frame2;
  bool                       is_recv_pose;
  int                        is_recv_tra;
  mavros_msgs::State         fcu_state;
  bool                       is_recv_state;
  double                     kx;
  double                     ky;
  double                     vx_max;
  double                     vy_max;
  double                     takeoff_yaw;
  bool                       is_aruco;
  bool                       is_H;
  bool                       auto_arming;
  double                     takeoff_h;
  mavros_msgs::SetMode       offb_set_mode;
  mavros_msgs::SetMode       land;

  mavros_msgs::CommandBool arm_cmd;

  Command cmd;

  CtrlSource                     ctrl_source;
  Mission                        mission;
  std::queue<sensor_msgs::Image> depth_queue;

  std::mutex depth_mtx;

  //存储frame1,y与frame2的坐标点
  std::map<Mission, std::pair<double, double>> mission_points;

  RflySimParam  rflysim_p;
  CamearasParam cam;

  //linzejun
  // common_msgs::Objects::ConstPtr lzj_objs;
  // std::vector<common_msgs::Objects> lzj_objs;
  common_msgs::Objects lzj_objs;
  BT::BehaviorTreeFactory factory;
  bool cross_frame_statu = false;
  std::string bt_tree_path = "/mnt/e/linzejun01/8.RflySimVision-master/8.rflysimvision/3.CustExps/e1_CompSlamNav/Ubuntu/src/Challege_ROS/controller/config/mav.xml";
  bool is_H_statu = false;
  ros::Time last_timestamp;
};

#endif  // CONTROLLER_H
