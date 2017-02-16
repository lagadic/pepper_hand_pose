#include <ros/ros.h>
#include <boost/thread.hpp>

#include <sensor_msgs/JointState.h>
#include <sensor_msgs/CameraInfo.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PoseArray.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Int8.h>
#include <whycon/PointArray.h>
#include <geometry_msgs/PointStamped.h>

#include <visp3/core/vpHomogeneousMatrix.h>
#include <visp3/core/vpMatrix.h>
#include <visp3/core/vpQuaternionVector.h>
#include <visp/vpXmlParserCamera.h>
#include <visp/vpXmlParserHomogeneousMatrix.h>

#include <visp_bridge/3dpose.h>
#include <visp_bridge/image.h>
#include <visp_bridge/camera.h>

#include <alproxies/almotionproxy.h>



class pepper_hand_pose
{
public:

  pepper_hand_pose(ros::NodeHandle &nh);
  ~pepper_hand_pose();
  void computeHandPose();
  void spin();
  void getPointArrayCb(const whycon::PointArrayConstPtr &msg);
  void getPoseArrayCb(const geometry_msgs::PoseArrayConstPtr &msg);
  void getStatusPointArrayCb(const std_msgs::Bool &status);
  void getCameraInfoCb(const sensor_msgs::CameraInfoConstPtr &msg);
  void publishCmdVel(const vpColVector &q);
  void computeTargetCalibration();

protected:

  boost::mutex lock_;

  //Visp
  vpCameraParameters m_cam;
  std::vector< vpImagePoint> m_points;
  std::vector< vpPoint> m_3Dpoints;

  vpHomogeneousMatrix m_cMh;
  vpImagePoint m_cog;
  bool m_initPose;
  int m_num_points;
  std::vector< unsigned int> m_map_index;
  bool m_map_index_initialized;

  vpImage<unsigned char> I;

  // CalibrationTsai
  std::vector<vpHomogeneousMatrix> m_tMe_stack;
  std::vector<vpHomogeneousMatrix> m_hMc_stack;
  AL::ALMotionProxy *m_motionProxy;
  std::string m_robotIp;
  int m_num_poses;


  // ROS
  ros::NodeHandle m_n;
  std::string m_pointArrayName;
  std::string m_cameraInfoName;
  std::string m_statusPointArrayName;
  std::string m_handPosePubName;
  std::string m_poseArrayName;
  std::string m_posePubName;
  std::string m_poseStatusPubName;

  ros::Subscriber m_pointArraySub;
  ros::Subscriber m_poseArraySub;
  ros::Subscriber m_statusPointArraySub;
  ros::Subscriber m_cameraInfoSub;

  ros::Publisher m_handPosePub;
  ros::Publisher m_handPoseStatusPub;
  ros::Publisher m_test;
  //  ros::Publisher m_point0;
  //  ros::Publisher m_point1;
  //  ros::Publisher m_point2;
  //  ros::Publisher m_point3;
  int freq;

  // Messages
  sensor_msgs::JointState m_q_dot_msg;

  //conditions
  bool m_camInfoIsInitialized;
  bool m_statusPointArray;
  bool m_mode_createTargeModel;
  bool m_mode_targetCalibration;
  bool m_node_init;

};


void pepper_hand_pose::getStatusPointArrayCb(const std_msgs::Bool &status)
{
  m_statusPointArray = status.data;
}
