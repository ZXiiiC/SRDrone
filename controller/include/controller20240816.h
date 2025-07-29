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

#include <behaviortree_cpp/bt_factory.h>
// * nodes
#include <behaviortree_cpp/action_node.h>
#include <behaviortree_cpp/condition_node.h>
#include <behaviortree_cpp/control_node.h>
#include <behaviortree_cpp/decorator_node.h>




// #include "behaviortree_cpp/bt_factory.h"
// // * nodes
// #include "behaviortree_cpp/action_node.h"
// #include "behaviortree_cpp/condition_node.h"
// #include "behaviortree_cpp/control_node.h"
// #include "behaviortree_cpp/decorator_node.h"

#include <opencv2/highgui.hpp>

// #include <sample_nodes/dummy_nodes.h>
// using namespace DummyNodes;

using namespace BT;

//linzejun +++
namespace BT {
/**
 * 原状态机使用枚举变量来选择无人机的动作
 * BehaviorTree.CPP的SwitchNode只能输入string类型的值
 * 这里添加可以输入枚举类型的值
 */
template <size_t NUM_CASES, typename T>
class EnumSwitchNode : public ControlNode {
 public:
  EnumSwitchNode(const std::string& name, const BT::NodeConfiguration& config)
      : ControlNode::ControlNode(name, config), running_child_(-1) {}

  // EnumSwitchNode(const std::string& name, BT::NodeConfiguration& config)
  //     : ControlNode::ControlNode(name, config), running_child_(-1) {}

  virtual ~EnumSwitchNode() override = default;
 
  void halt() override {
    running_child_ = -1;
    ControlNode::halt();
  }
 
  static PortsList providedPorts() {
    PortsList ports;
    ports.insert(BT::InputPort<T>("enum_variable"));
    for (unsigned i = 0; i < NUM_CASES; i++) {
      char case_str[20];
      sprintf(case_str, "case_%d", i + 1);
      ports.insert(BT::InputPort<std::string>(case_str));
    }
    return ports;
  }
 
 private:
  int running_child_;
  virtual BT::NodeStatus tick() override;
};
 
template <size_t NUM_CASES, typename T>
inline NodeStatus EnumSwitchNode<NUM_CASES, T>::tick() {
  // 如果是非枚举类型，会编译报错
  static_assert(std::is_enum<T>::value,
                "[registerNode]: accepts only enum classes!!!");
 
  constexpr const char* case_port_names[9] = {"case_1", "case_2", "case_3",
                                              "case_4", "case_5", "case_6",
                                              "case_7", "case_8", "case_9"};
  if (childrenCount() != NUM_CASES + 1) {
    throw LogicError(
        "Wrong number of children in EnumSwitchNode; "
        "must be (num_cases + default)");
  }
 
  T variable, value;
  int child_index = NUM_CASES;  // default index;
  if (getInput("enum_variable", variable)) {
    // check each case until you find a match
    for (unsigned index = 0; index < NUM_CASES; ++index) {
      bool found = false;
      if (index < 9) {
        found = (bool)getInput(case_port_names[index], value);
      } else {
        char case_str[20];
        sprintf(case_str, "case_%d", index + 1);
        found = (bool)getInput(case_str, value);
      }
      if (found && variable == value) {
        child_index = index;
        break;
      }
    }
  }
 
  // if another one was running earlier, halt it
  if (running_child_ != -1 && running_child_ != child_index) {
    haltChild(running_child_);
  }
 
  auto& selected_child = children_nodes_[child_index];
  NodeStatus ret = selected_child->executeTick();
  if (ret == NodeStatus::RUNNING) {
    running_child_ = child_index;
  } else {
    haltChildren();
    running_child_ = -1;
  }
  return ret;
}
 
} 

// using namespace BT;

// //行为树
// class UAV_fly{
//     public:
//         int x_step = 1;
//         int y_step = 1;
//         PointXY position = {0,0};
//         BT::NodeStatus moveX(){
//             std::cout << "Moving along the x axis: " << position.x << std::endl;
//             position.x += x_step;
//             return BT::NodeStatus::SUCCESS;
//         };
//         BT::NodeStatus moveY(){
//             std::cout << "Moving along the y axis: " << position.y << std::endl;
//             position.y += y_step;
//             return BT::NodeStatus::SUCCESS;
//         };
// };






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

  // //行为树的类
  // class Takeoff : public BT::SyncActionNode
  // {
  //   public:
  //     Takeoff(const std::string& name, const BT::NodeConfiguration& config)
  //         : BT::SyncActionNode(name, config) {}
  //     static BT::PortsList providedPorts();
  //     BT::NodeStatus tick() override;
  // }
  
  // class cross_corridor : public BT::SyncActionNode
  // {
  //   public:
  //     cross_corridor(const std::string& name, const BT::NodeConfiguration& config)
  //         : BT::SyncActionNode(name, config) {}
  //     static BT::PortsList providedPorts();  
  //     BT::NodeStatus tick() override;  
  // }

  // class land : public BT::SyncActionNode
  // {
  //   public:
  //     land(const std::string& name, const BT::NodeConfiguration& config)
  //         : BT::SyncActionNode(name, config) {}
  //     static BT::PortsList providedPorts();
  //     BT::NodeStatus tick() override;
  // }
  
  // class cross_frame1 : public BT::SyncActionNode
  // {
  //   public:
  //     cross_frame1(const std::string& name, const BT::NodeConfiguration& config)
  //         : BT::SyncActionNode(name, config) {}
  //     static BT::PortsList providedPorts();
  //     BT::NodeStatus tick() override;
  // }  

  class ApproachObject : public BT::SyncActionNode
  {
  public:
    ApproachObject(const std::string& name) :
        BT::SyncActionNode(name, {})
    {}

    // You must override the virtual function tick()
    BT::NodeStatus tick() override
    {
      std::cout << "ApproachObject: " << this->name() << std::endl;
      return BT::NodeStatus::SUCCESS;
    }
  };


public:
  // Controller(const std::string& name, const BT::NodeConfiguration& config): BT::SyncActionNode(name, config){}
  
  Controller(ros::NodeHandle &nh);

  //发送控制指令以及实时判断当前飞机状态
  bool Init();
  void Run(const ros::TimerEvent &e);
  void DepthImgCB(const sensor_msgs::Image::ConstPtr &depth);
  void RecvPose(const nav_msgs::Odometry::ConstPtr &pose);
  void RecvFcuState(const mavros_msgs::State::ConstPtr &state);
  void RecvAiCB(const common_msgs::Objects::ConstPtr
                  &objs);  // 在这里计算框的中心位置，并计算它在全局坐标系里面的位置
  void ObjConterRGB(const common_msgs::Objects::ConstPtr objs,
                    pcl::PointXYZ *cnt);  //直接使用目标检测算结算位置
  void RecvTra(const quadrotor_msgs::PositionCommand::ConstPtr &msg);
  void RecvLIO(const nav_msgs::Odometry::ConstPtr &odom);
  void RecvAruco(const common_msgs::Aruco::ConstPtr &msg);
  
  //行为树
  // BT::NodeStatus init();
  // static BT::NodeStatus Takeoff();
  // static BT::NodeStatus Cross_corridor();
  // static BT::NodeStatus Land(); 
  // static BT::NodeStatus Cross_frame1();
  // static BT::NodeStatus Cross_frame2();
  // static BT::NodeStatus Recognize_aruco();
  // static BT::NodeStatus Recognize_H();
  // static BT::NodeStatus End(); 
  
  //行为树动作节点
  BT::NodeStatus Takeoff();
  BT::NodeStatus Cross_corridor();
  BT::NodeStatus Land(); 
  BT::NodeStatus Cross_frame1();
  BT::NodeStatus Cross_frame2();
  BT::NodeStatus Recognize_aruco();
  BT::NodeStatus Recognize_H();
  BT::NodeStatus End(); 

  //行为树状态检查节点
  BT::NodeStatus Check_cross_frame1();
  BT::NodeStatus Check_result1();
  BT::NodeStatus Check_cross_frame2();
  BT::NodeStatus Check_result2();  
  BT::NodeStatus Check_recognize_aruco();
  BT::NodeStatus Check_recognize_H();
  BT::NodeStatus Check_Takeoff();
  BT::NodeStatus Check_recognize_aruco2();
  BT::NodeStatus Check_recognize_H2();
  BT::NodeStatus Check_End();
  BT::NodeStatus Check_Land();

  bool getRecvPose();
  mavros_msgs::State getFcuState();
  bool getRecvFcuState();


  // BT::NodeStatus Tick();
  // template <> inline Mission BT::convertFromString(BT::StringView key) {
  //   auto parts = BT::splitString(key, ',');
  //   if (parts.size() != 1) {
  //     throw BT::RuntimeError("invalid input");
  //   } else {
  //     auto str = parts[0];
  //     if ("Mission::takeoff" == str) {
  //       return Mission::takeoff;
  //     } else if ("Mission::cross_frame1" == str) {
  //       return Mission::cross_frame1;
  //     } else if ("Mission::cross_frame2" == str) {
  //       return Mission::cross_frame2;
  //     } else if ("Mission::recognize_aruco" == str) {
  //       return Mission::recognize_aruco;
  //     } else if ("Mission::recognize_H" == str) {
  //       return Mission::recognize_H;
  //     }else if ("Mission::land" == str) {
  //       return Mission::land;
  //     }else {
  //       throw BT::RuntimeError(std::string("invalid input, chars=") +
  //                             str.to_string());
  //     }
  //   }
  // }

  // class controller_uav : public BT::SyncActionNode{
  //   public:
  //     controller_uav(const std::string& name, const BT::NodeConfiguration& config): BT::SyncActionNode(name, config) {}
  //     BT::NodeStatus Takeoff(const Controller &controller);
  //     BT::NodeStatus Cross_corridor(const Controller &controller);
  //     BT::NodeStatus Land(const Controller &controller); 
  //     BT::NodeStatus Cross_frame1(const Controller &controller);
  //     BT::NodeStatus Cross_frame2(const Controller &controller);
  //     BT::NodeStatus Recognize_aruco(const Controller &controller);
  //     BT::NodeStatus Recognize_H(const Controller &controller);
  //     BT::NodeStatus End(const Controller &controller); 
  //     BT::NodeStatus Tick(const Controller &controller);
  // }




  // class Takeoff : public BT::SyncActionNode {
  // public:
  //   Controller* controller;
  //   Takeoff(const std::string& name, const BT::NodeConfiguration& config)
  //       : BT::SyncActionNode(name, config) {}
  
  //   // You must override the virtual function tick()
  //   // BT::NodeStatus tick() override {
  //   //   auto msg = getInput<std::string>("message");
  //   //   if (!msg) {
  //   //     throw BT::RuntimeError("missing required input [message]: ", msg.error());
  //   //   }
  //   //   std::cout << "SaySomething::tick()- " << msg.value() << std::endl;
  //   //   return BT::NodeStatus::SUCCESS;
  //   // }

  //   BT::NodeStatus tick() override;
  
  //   // It is mandatory to define this static method.
  //   static BT::PortsList providedPorts() {
  //     return {BT::InputPort<std::string>("message")};
  //   }

  //   // void process(const Controller &controller);

  // }uav_Takeoff;

  // class Land : public BT::SyncActionNode {
  // public:
  //   Land(const std::string& name, const BT::NodeConfiguration& config)
  //       : BT::SyncActionNode(name, config) {}
  
  //   // You must override the virtual function tick()
  //   BT::NodeStatus tick() override {
  //     auto msg = getInput<std::string>("message");
  //     if (!msg) {
  //       throw BT::RuntimeError("missing required input [message]: ", msg.error());
  //     }
  //     std::cout << "SaySomething::tick()- " << msg.value() << std::endl;
  //     return BT::NodeStatus::SUCCESS;
  //   }
  
  //   // It is mandatory to define this static method.
  //   static BT::PortsList providedPorts() {
  //     return {BT::InputPort<std::string>("message")};
  //   }

  //   void process(const Controller &controller);

  // };

  // class Cross_frame1 : public BT::SyncActionNode {
  // public:
  //   Cross_frame1(const std::string& name, const BT::NodeConfiguration& config)
  //       : BT::SyncActionNode(name, config) {}
  
  //   // You must override the virtual function tick()
  //   BT::NodeStatus tick() override {
  //     auto msg = getInput<std::string>("message");
  //     if (!msg) {
  //       throw BT::RuntimeError("missing required input [message]: ", msg.error());
  //     }
  //     std::cout << "SaySomething::tick()- " << msg.value() << std::endl;
  //     return BT::NodeStatus::SUCCESS;
  //   }
  
  //   // It is mandatory to define this static method.
  //   static BT::PortsList providedPorts() {
  //     return {BT::InputPort<std::string>("message")};
  //   }

  //   void process(const Controller &controller);

  // };





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

  BT::BehaviorTreeFactory factory;
  std::string bt_tree_path = "/mnt/e/linzejun01/8.RflySimVision-master/8.rflysimvision/3.CustExps/e1_CompSlamNav/Ubuntu/src/Challege_ROS/controller/config/mav.xml";
  int count_createTreeFromFile;
};

#endif  // CONTROLLER_H
