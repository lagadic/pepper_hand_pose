#include <iostream>
#include <vector>
#include <algorithm>
#include <map>

#include <visp3/core/vpPoint.h>
#include <visp/vpPose.h>
#include <visp/vpPixelMeterConversion.h>
#include <visp/vpXmlParserHomogeneousMatrix.h>
#include <visp/vpCalibration.h>

#include "pepper_hand_pose.h"

#include <visp3/gui/vpDisplayX.h>

vpDisplayX d;
pepper_hand_pose::pepper_hand_pose(ros::NodeHandle &nh):
  lock_(), m_cam(), m_points(), m_initPose(true), m_statusPointArray(false), m_tMe_stack(), m_hMc_stack(), m_motionProxy(NULL),
  m_num_poses(0), m_node_init(false)

{
  // read in config options
  m_n = nh;
  m_cog.set_uv(0.0,0.0);

  m_n.param( "frequency", freq, 30);
  m_n.param<std::string>("pointArrayName", m_pointArrayName, "/whycon/points");
  m_n.param<std::string>("poseArrayName", m_poseArrayName, "/whycon/poses");
  m_n.param<std::string>("cameraInfoName", m_cameraInfoName, "/camera/camera_info");
  m_n.param<std::string>("statusPointArrayName", m_statusPointArrayName, "/whycon/status");
  m_n.param<std::string>("poseNamePub", m_statusPointArrayName, "/whycon/status");
  m_n.param("numPoints", m_num_points, 4);
  m_n.param("createTargetModel", m_mode_createTargeModel, false);
  m_n.param("targetCalibrationTsai", m_mode_targetCalibrationTsai, false);
  m_n.param<std::string>("robotIp", m_robotIp, "131.254.10.126");

  m_pointArraySub = m_n.subscribe( m_pointArrayName, 1, (boost::function < void(const whycon::PointArrayConstPtr &)>) boost::bind( &pepper_hand_pose::getPointArrayCb, this, _1 ));
  m_cameraInfoSub = m_n.subscribe( m_cameraInfoName, 1, (boost::function < void(const sensor_msgs::CameraInfoConstPtr & )>) boost::bind( &pepper_hand_pose::getCameraInfoCb, this, _1 ));
  m_statusPointArraySub = m_n.subscribe ( m_statusPointArrayName, 1, &pepper_hand_pose::getStatusPointArrayCb, this);

  if (m_mode_createTargeModel)
    m_poseArraySub = m_n.subscribe( m_poseArrayName, 1, (boost::function < void(const geometry_msgs::PoseArrayConstPtr & )>) boost::bind( &pepper_hand_pose::getPoseArrayCb, this, _1 ));

  m_3Dpoints.resize(m_num_points);

//    m_3Dpoints[0].setWorldCoordinates(0.02597527783, 0.005889128802, -0.001795542021  ) ;
//    m_3Dpoints[1].setWorldCoordinates(-0.003614372679, 0.00420168375, 0.009403076787  ) ;
//    m_3Dpoints[2].setWorldCoordinates(-0.01999512598, -0.002122752901, 0.0008281653174 ) ;
//    m_3Dpoints[3].setWorldCoordinates(-0.002365779171, -0.007968059652, -0.008435700083  ) ;

    m_3Dpoints[0].setWorldCoordinates(0.02546128917, 0.006560048472, -0.001485995456   ) ;
    m_3Dpoints[1].setWorldCoordinates( -0.00368857616, 0.00502116606, 0.008440775564   ) ;
    m_3Dpoints[2].setWorldCoordinates( -0.01953384973, -0.00216840105, 0.0001954474102 ) ;
    m_3Dpoints[3].setWorldCoordinates(-0.002238863281, -0.009412813483, -0.007150227518   ) ;


  m_map_index.resize(m_num_points);
  m_map_index_initialized = false;

  //  double dist_point_ = 0.025/2;

  //  m_3Dpoints[0].setWorldCoordinates(-0.023,-0.023, 0) ;
  //  m_3Dpoints[1].setWorldCoordinates(dist_point_,-dist_point_, 0) ;
  //  m_3Dpoints[2].setWorldCoordinates(dist_point_,dist_point_, 0) ;
  //  m_3Dpoints[3].setWorldCoordinates(-dist_point_,dist_point_,0) ;

  m_handPosePub = m_n.advertise<geometry_msgs::PoseStamped>(m_posePubName, 1);
  //  m_point0 = m_n.advertise<geometry_msgs::PointStamped>("/point0", 1);
  //  m_point1 = m_n.advertise<geometry_msgs::PointStamped>("/point1", 1);
  //  m_point2 = m_n.advertise<geometry_msgs::PointStamped>("/point2", 1);
  //  m_point3 = m_n.advertise<geometry_msgs::PointStamped>("/point3", 1);

  if (m_mode_targetCalibrationTsai || m_mode_createTargeModel)
    m_motionProxy = new AL::ALMotionProxy(m_robotIp);

  I.resize(480, 640);
  d.init(I);
  vpDisplay::setTitle(I, "ViSP viewer");

  m_node_init = true;

  ROS_INFO("Launch Pepper_hand_pose node");
}

pepper_hand_pose::~pepper_hand_pose(){
  if (m_motionProxy != NULL) {
    delete m_motionProxy;
    m_motionProxy = NULL;
  }
}

void pepper_hand_pose::spin()
{
  ros::Rate loop_rate(freq);
  while(ros::ok()){

    vpMouseButton::vpMouseButtonType button;
    bool ret = vpDisplay::getClick(I, button, false);
    this->computeHandPose();
    if (m_mode_targetCalibrationTsai)
    {
      if (ret && button == vpMouseButton::button1 && m_statusPointArray)
        this->prepareTargetCalibrationTsai();
      else if (ret && button == vpMouseButton::button2)
      {
        if (m_num_poses>4)
          this->computeTargetCalibrationTsai();
        else
          std::cout << "You need to save at least 5 poses"<< std::endl;
      }

      std::ostringstream s;
      s << "Counter pose saved: " << m_num_poses;
      vpDisplay::displayText(I, 10, 10, s.str(), vpColor::green);
      vpDisplay::flush(I);
    }

    ret = false;
    ros::spinOnce();
    loop_rate.sleep();
  }
}

void computePose(std::vector<vpPoint> &point, const std::vector<vpImagePoint> &corners,
                 const vpCameraParameters &cam, bool &init, vpHomogeneousMatrix &cMo)
{
  vpPose pose;
  double x=0, y=0;
  for (unsigned int i=0; i < point.size(); i ++) {
    vpPixelMeterConversion::convertPoint(cam, corners[i], x, y);
    point[i].set_x(x);
    point[i].set_y(y);
    pose.addPoint(point[i]);
  }

  if (init) {
    vpHomogeneousMatrix cMo_dementhon, cMo_lagrange;
    pose.computePose(vpPose::DEMENTHON_VIRTUAL_VS, cMo_dementhon);
    double residual_dementhon = pose.computeResidual(cMo_dementhon);
    pose.computePose(vpPose::LAGRANGE_VIRTUAL_VS, cMo_lagrange);
    double residual_lagrange = pose.computeResidual(cMo_lagrange);
    if (residual_dementhon < residual_lagrange)
      cMo = cMo_dementhon;
    else
      cMo = cMo_lagrange;
  }

  pose.computePose(vpPose::VIRTUAL_VS, cMo) ;
  init = false;
}

void pepper_hand_pose::computeHandPose()
{
  vpDisplay::display(I);
  if ( m_camInfoIsInitialized && m_statusPointArray)
  {
    // Compute cog
    m_cog.set_uv(0.0,0.0);
    for(unsigned int i = 0; i< m_num_points; i++)
      m_cog += m_points[i];
    m_cog /= m_num_points;

    // Find index of the point 0 (target near the thumb)
    unsigned int index_0 = 0;
    double dist_max = 0.0;
    for (unsigned int i = 0; i< m_num_points; i++)
    {
      double d = vpImagePoint::distance(m_cog, m_points[i]);
      if (d > dist_max)
      {
        index_0 = i;
        dist_max = d;
      }
    }

    std::map< double,vpImagePoint> poly_verteces;

    double theta;
    for(unsigned int i = 0; i< m_num_points; i++)
    {
      theta = atan2(m_points[i].get_v() - m_cog.get_v(), m_points[i].get_u() - m_cog.get_u());
      // Insert the vertexes in the map (ordered)
      poly_verteces.insert ( std::pair<double,vpImagePoint>(theta,m_points[i]) );
    }

    // Now we create a Vector containing the ordered vertexes
    std::vector<vpImagePoint> poly_vert;
    int index_first= 0;
    unsigned int count = 0;

    for( std::map<double,vpImagePoint>::iterator it = poly_verteces.begin();  it!=poly_verteces.end(); ++it )
    {
      poly_vert.push_back( it->second );
      if (m_points[index_0] == it->second )
        index_first = count;
      count++;
    }

    std::rotate(poly_vert.begin(), poly_vert.begin() + index_first, poly_vert.end());

    lock_.lock();

    for(unsigned int i = 0; i< m_num_points; i++)
    {
      std::vector<vpImagePoint>::iterator it = find (poly_vert.begin(), poly_vert.end(), m_points[i]);
      if (it != poly_vert.end())
        m_map_index[i] = distance (poly_vert.begin (), it);
      else
      {
        std::cout << "ERROR: Cannot order the vector of points\n";
        ros::shutdown();
      }
    }

    m_map_index_initialized = true;

    lock_.unlock();

    for(unsigned int i = 0; i< m_num_points; i++)
    {
      std::cout << " Index " << i <<" - " << m_map_index[i] << std::endl;
    }

    //        for(unsigned int i = 0; i< m_num_points; i++)
    //        {
    //          std::cout << " Init " << i <<" - " << m_points[i] << std::endl;
    //        }

    //        for(unsigned int i = 0; i< m_num_points; i++)
    //        {
    //          std::cout << " Fin " << i <<" - " << poly_vert[i] << std::endl;
    //        }
    computePose(m_3Dpoints, poly_vert, m_cam, m_initPose, m_cMh);

    // std::cout << "Pose:" <<  m_cMh << std::endl ;
    geometry_msgs::PoseStamped msg_des_pose;
    msg_des_pose.header.stamp = ros::Time::now();
    msg_des_pose.header.frame_id = "CameraBottom_optical_frame";
    msg_des_pose.pose = visp_bridge::toGeometryMsgsPose(m_cMh); //convert
    m_handPosePub.publish(msg_des_pose);


    vpDisplay::displayFrame(I,m_cMh,m_cam,0.05, vpColor::none, 2);
    for(unsigned int j = 0; j<poly_vert.size();j++)
    {
      std::ostringstream s;
      s << j;
      vpDisplay::displayText(I, poly_vert[j], s.str(), vpColor::green);
      vpDisplay::displayCross(I,poly_vert[j],10,  vpColor::red,2 );
    }
    vpDisplay::displayCross(I, m_cog,10,  vpColor::cyan,2 );
    //vpDisplay::displayText(I, 30,30, "VISP", vpColor::green);

  }
  vpDisplay::flush(I);
}

void pepper_hand_pose::getCameraInfoCb(const sensor_msgs::CameraInfoConstPtr &msg)
{
  std::cout << "Received Camera INFO"<<std::endl;
  // Convert the paramenter in the visp format
  m_cam = visp_bridge::toVispCameraParameters(*msg);
  m_cam.printParameters();

  // Stop the subscriber (we don't need it anymore)
  this->m_cameraInfoSub.shutdown();

  m_camInfoIsInitialized = 1;
}

void pepper_hand_pose::getPointArrayCb(const whycon::PointArrayConstPtr &msg)
{
  m_points.clear();
  for (unsigned int i = 0; i < m_num_points; i++)
  {
    vpImagePoint p(msg->points[i].y,msg->points[i].x );
    m_points.push_back(p);
  }
}

vpPoint getCogPoints(const std::vector <vpPoint> &points)
{
  vpPoint cog;
  for (unsigned int i = 0; i < points.size(); i++)
  {
    cog.set_oX(cog.get_oX() + points[i].get_oX());
    cog.set_oY(cog.get_oY() + points[i].get_oY());
    cog.set_oZ(cog.get_oZ() + points[i].get_oZ());
  }

  cog.set_oX(cog.get_oX()/points.size()) ;
  cog.set_oY(cog.get_oY()/points.size()) ;
  cog.set_oZ(cog.get_oZ()/points.size()) ;

  return cog;
}



static double distance3Dpoints (const vpPoint &iP1, const vpPoint &iP2) {
  return(sqrt(vpMath::sqr(iP1.get_oX()-iP2.get_oX())+vpMath::sqr(iP1.get_oY()-iP2.get_oY()) + vpMath::sqr(iP1.get_oZ()-iP2.get_oZ())  ));}

void pepper_hand_pose::getPoseArrayCb(const geometry_msgs::PoseArrayConstPtr &msg)
{
  std::cout << "test  " << std::endl ;

  if (!m_map_index_initialized || !m_node_init)
    return;

  std::vector <vpPoint> c_points;
  std::vector <vpColVector> o_points_vec;


  for (unsigned int i = 0; i < m_num_points; i++)
  {
    vpPoint p;
    p.set_oX(msg->poses[m_map_index[i]].position.x);
    p.set_oY(msg->poses[m_map_index[i]].position.y);
    p.set_oZ(msg->poses[m_map_index[i]].position.z);
    c_points.push_back(p);
  }
  // Compute cog
  vpPoint cog = getCogPoints(c_points);
  std::cout << "cog "  <<  cog.get_oX() <<" " <<  cog.get_oY() <<" " <<  cog.get_oZ() <<" " << std::endl ;

  // Introduce a matrix to pass from camera frame of Aldebaran to visp camera frame
  vpHomogeneousMatrix cam_alMe_camvisp;
  for(unsigned int i=0; i<3; i++)
    cam_alMe_camvisp[i][i] = 0; // remove identity
  cam_alMe_camvisp[0][2] = 1.;
  cam_alMe_camvisp[1][0] = -1.;
  cam_alMe_camvisp[2][1] = -1.;

  vpHomogeneousMatrix torsoMlcam_al(m_motionProxy->getTransform("CameraBottom", 0, true));
  vpHomogeneousMatrix torsoMlcam_visp = torsoMlcam_al * cam_alMe_camvisp;
  std::cout << "Torso M CameraBottom:\n" << torsoMlcam_visp << std::endl;

  vpHomogeneousMatrix torsoMLWristYaw( m_motionProxy->getTransform("RWristYaw", 0, true));
  std::cout << "Torso M RWristYaw:\n" << torsoMLWristYaw << std::endl;
  vpHomogeneousMatrix cMrw = torsoMlcam_visp.inverse()*torsoMLWristYaw;
  vpDisplay::displayFrame(I, cMrw, m_cam, 0.06, vpColor::none);

  //vpDisplay::flush(I);

  vpTranslationVector cto(cog.get_oX(), cog.get_oY(), cog.get_oZ());
  vpRotationMatrix cRo(cMrw);
  vpHomogeneousMatrix cMo(cto,cRo);
  std::cout << "cMo "  <<  cMo  << std::endl ;

  for (unsigned int i = 0; i < m_num_points; i++)
  {
    //    vpPoint oP;
    //    oP = cMo.inverse()*c_points[i];
    //    o_points.push_back(oP);

    vpColVector oP;
    c_points[i].changeFrame(cMo.inverse(),oP);
    o_points_vec.push_back(oP);
  }

  std::cout << "Distance 0 - 1 " << distance3Dpoints(c_points[0],c_points[1])  <<" " << std::endl ;
  std::cout << "Distance 1 - 2 " << distance3Dpoints(c_points[1],c_points[2])  <<" " << std::endl ;
  std::cout << "Distance 2 - 3 " << distance3Dpoints(c_points[2],c_points[3])  <<" " << std::endl ;
  std::cout << "Distance 3 - 0 " << distance3Dpoints(c_points[3],c_points[0])  <<" " << std::endl ;



  for (unsigned int i = 0; i < m_num_points; i++)
  {
    std::cout << "C_points " << i << " " <<  c_points[i].get_oX() <<" " <<  c_points[i].get_oY() <<" " <<  c_points[i].get_oZ() <<" " << std::endl ;
  }

  for (unsigned int i = 0; i < m_num_points; i++)
  {
    std::cout << "O_points " << i << " " <<  o_points_vec[i][0] <<" " <<  o_points_vec[i][1] <<" " <<  o_points_vec[i][2] <<" " << std::endl ;
  }


  vpHomogeneousMatrix wMh = cMrw.inverse() *m_cMh;
    std::cout << "wMh "  <<  wMh  << std::endl ;

//  geometry_msgs::PoseStamped msg_des_pose;
//  msg_des_pose.header.stamp = ros::Time::now();
//  msg_des_pose.header.frame_id = "CameraBottom_optical_frame";
//  msg_des_pose.pose = visp_bridge::toGeometryMsgsPose(cMo); //convert
//  m_handPosePub.publish(msg_des_pose);

  //  geometry_msgs::PointStamped msg_point;
  //  msg_point.header.stamp = ros::Time::now();
  //  msg_point.header.frame_id = "CameraBottom_optical_frame";

  //  geometry_msgs::PoseStamped msg_des_pose1;
  //  msg_des_pose1.header.stamp = ros::Time::now();
  //  msg_des_pose1.header.frame_id = m_posePubName;
  //  msg_des_pose1.pose = visp_bridge::toGeometryMsgsPose(cMo); //convert
  //  m_handPosePub.publish(msg_des_pose);
}


void pepper_hand_pose::prepareTargetCalibrationTsai()
{
  // Get transformation from Torso to RWristYaw
  std::vector<float> result = m_motionProxy->getTransform("RWristYaw", 0, 1);
  vpHomogeneousMatrix tMe(result);
  m_tMe_stack.push_back(tMe);
  m_hMc_stack.push_back(m_cMh.inverse());
  m_num_poses++;
}

void pepper_hand_pose::computeTargetCalibrationTsai()
{
  std::cout << "m_tMe_stack" << std::endl;

  for (unsigned int i = 0; i < m_tMe_stack.size(); i++ )
  {
    std::cout << m_tMe_stack[i] << std::endl;
  }

  std::cout << "m_hMc_stack" << std::endl;

  for (unsigned int i = 0; i < m_hMc_stack.size(); i++ )
  {
    std::cout << m_hMc_stack[i] << std::endl;
  }

  vpHomogeneousMatrix eMh;
  // Tsai calibration
  vpCalibration::calibrationTsai(m_hMc_stack, m_tMe_stack, eMh);
  std::cout << "Extrinsic camera parameters: " << eMh << std::endl;

  vpXmlParserHomogeneousMatrix p; // Create a XML parser
  std::string name_M =  "eMh_rightArm_Pepper";
  char filename[FILENAME_MAX];
  sprintf(filename, "%s", "/tmp/eMc_pepper.xml");

  if (p.save(eMh, filename, name_M) != vpXmlParserHomogeneousMatrix::SEQUENCE_OK)
    std::cout << "Cannot save the Homogeneous matrix" << std::endl;
  else
    std::cout << "Homogeneous matrix saved in " << filename << std::endl;


}





