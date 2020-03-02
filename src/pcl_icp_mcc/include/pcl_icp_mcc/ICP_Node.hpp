#include <memory>
#include <iostream>

#include <ros/ros.h>
#include <ros/time.h>
#include <ros/duration.h>
#include <pcl_ros/point_cloud.h>
#include <tf_conversions/tf_eigen.h>
#include <tf/transform_broadcaster.h>

#include <std_msgs/Time.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/PointCloud2.h>

#include <nodelet/nodelet.h>
#include <pluginlib/class_list_macros.h>

#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/approximate_voxel_grid.h>
#include <pcl/registration/ndt.h>
#include <pcl/registration/icp.h>
#include <pcl/registration/gicp.h>
#include <pcl_icp_mcc/pcl_mcc_icp_utilities.hpp>
//#include <hdl_graph_slam/ros_utils.hpp>
//#include <hdl_graph_slam/registrations.hpp>
//typedef pcl::PointXYZ PointT;
#include <pcl/registration/registration.h>
//boost::shared_ptr<pcl::Registration<pcl::PointXYZ, pcl::PointXYZ>> select_registration_method(ros::NodeHandle& pnh);


boost::shared_ptr<pcl::Registration<pcl::PointXYZ, pcl::PointXYZ>> select_registration_method(ros::NodeHandle& pnh) {
  using PointT = pcl::PointXYZ;

  // select a registration method (ICP, GICP, NDT)
  std::string registration_method = pnh.param<std::string>("registration_method", "ICP");
  if(registration_method == "ICP") {
    std::cout << "registration: ICP" << std::endl;
    boost::shared_ptr<pcl::IterativeClosestPoint<PointT, PointT>> icp(new pcl::IterativeClosestPoint<PointT, PointT>());
    return icp;
  } else if(registration_method.find("GICP") != std::string::npos) {
    if(registration_method.find("OMP") == std::string::npos) {
      std::cout << "registration: GICP" << std::endl;
      boost::shared_ptr<pcl::GeneralizedIterativeClosestPoint<PointT, PointT>> gicp(new pcl::GeneralizedIterativeClosestPoint<PointT, PointT>());
      return gicp;
    } 
    else 
    {
    //  std::cout << "registration: GICP_OMP" << std::endl;
     // boost::shared_ptr<pclomp::GeneralizedIterativeClosestPoint<PointT, PointT>> gicp(new pclomp::GeneralizedIterativeClosestPoint<PointT, PointT>());
     // return gicp;
    }
  } 
  // else 
  // {
  //   if(registration_method.find("NDT") == std::string::npos ) {
  //     std::cerr << "warning: unknown registration type(" << registration_method << ")" << std::endl;
  //     std::cerr << "       : use NDT" << std::endl;
  //   }

  //   double ndt_resolution = pnh.param<double>("ndt_resolution", 0.5);
  //   if(registration_method.find("OMP") == std::string::npos) {
  //     std::cout << "registration: NDT " << ndt_resolution << std::endl;
  //     boost::shared_ptr<pcl::NormalDistributionsTransform<PointT, PointT>> ndt(new pcl::NormalDistributionsTransform<PointT, PointT>());
  //     ndt->setTransformationEpsilon(0.01);
  //     ndt->setResolution(ndt_resolution);
  //     return ndt;
  //   } else {
  //     int num_threads = pnh.param<int>("ndt_num_threads", 0);
  //     std::string nn_search_method = pnh.param<std::string>("ndt_nn_search_method", "DIRECT7");
  //     std::cout << "registration: NDT_OMP " << nn_search_method << " " << ndt_resolution << " (" << num_threads << " threads)" << std::endl;
  //     boost::shared_ptr<pclomp::NormalDistributionsTransform<PointT, PointT>> ndt(new pclomp::NormalDistributionsTransform<PointT, PointT>());
  //     if(num_threads > 0) {
  //       ndt->setNumThreads(num_threads);
  //     }
  //     ndt->setTransformationEpsilon(0.01);
  //     ndt->setMaximumIterations(64);
  //     ndt->setResolution(ndt_resolution);
  //     if(nn_search_method == "KDTREE") {
  //       ndt->setNeighborhoodSearchMethod(pclomp::KDTREE);
  //     } else if (nn_search_method == "DIRECT1") {
  //       ndt->setNeighborhoodSearchMethod(pclomp::DIRECT1);
  //     } else {
  //       ndt->setNeighborhoodSearchMethod(pclomp::DIRECT7);
  //     }
  //     return ndt;
  //   }
  // }

  return nullptr;
}

// static geometry_msgs::TransformStamped matrix2transform(const ros::Time& stamp, const Eigen::Matrix4f& pose, const std::string& frame_id, const std::string& child_frame_id)
//  {
//   Eigen::Quaternionf quat(pose.block<3, 3>(0, 0));
//   quat.normalize();
//   geometry_msgs::Quaternion odom_quat;
//   odom_quat.w = quat.w();
//   odom_quat.x = quat.x();
//   odom_quat.y = quat.y();
//   odom_quat.z = quat.z();

//   geometry_msgs::TransformStamped odom_trans;
//   odom_trans.header.stamp = stamp;
//   odom_trans.header.frame_id = frame_id;
//   odom_trans.child_frame_id = child_frame_id;

//   odom_trans.transform.translation.x = pose(0, 3);
//   odom_trans.transform.translation.y = pose(1, 3);
//   odom_trans.transform.translation.z = pose(2, 3);
//   odom_trans.transform.rotation = odom_quat;

//   return odom_trans;
// }

// static Eigen::Isometry3d odom2isometry(const nav_msgs::OdometryConstPtr& odom_msg)
//  {
//   const auto& orientation = odom_msg->pose.pose.orientation;
//   const auto& position = odom_msg->pose.pose.position;

//   Eigen::Quaterniond quat;
//   quat.w() = orientation.w;
//   quat.x() = orientation.x;
//   quat.y() = orientation.y;
//   quat.z() = orientation.z;

//   Eigen::Isometry3d isometry = Eigen::Isometry3d::Identity();
//   isometry.linear() = quat.toRotationMatrix();
//   isometry.translation() = Eigen::Vector3d(position.x, position.y, position.z);
//   return isometry;
// }


class ICPNode{
public:
  typedef pcl::PointXYZ PointT;
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  ICPNode() {}
 ~ICPNode() {}

 void onInit() {
   // NODELET_DEBUG("initializing scan_matching_odometry_nodelet...");
   // nh = getNodeHandle();
    //private_nh = getPrivateNodeHandle();

    initialize_params();

    points_sub = nh.subscribe("/filtered_points", 1000, &ICPNode::cloud_callback, this);
    read_until_pub = nh.advertise<std_msgs::Header>("/scan_matching_odometry/read_until", 1000);
    odom_pub = nh.advertise<nav_msgs::Odometry>("/odom", 1000);
  }

private:
  /**
   * @brief initialize parameters
   */
  void initialize_params() {
    auto& pnh = private_nh;
    points_topic = pnh.param<std::string>("points_topic", "/velodyne_points");
    odom_frame_id = pnh.param<std::string>("odom_frame_id", "odom");

    // The minimum tranlational distance and rotation angle between keyframes.
    // If this value is zero, frames are always compared with the previous frame
    keyframe_delta_trans = pnh.param<double>("keyframe_delta_trans", 0.5);
    keyframe_delta_angle = pnh.param<double>("keyframe_delta_angle", 0.5);
    keyframe_delta_time = pnh.param<double>("keyframe_delta_time", 0.1);

    // Registration validation by thresholding
    transform_thresholding = pnh.param<bool>("transform_thresholding", false);
    max_acceptable_trans = pnh.param<double>("max_acceptable_trans", 0.01);
    max_acceptable_angle = pnh.param<double>("max_acceptable_angle", 0.01);

    // select a downsample method (VOXELGRID, APPROX_VOXELGRID, NONE)
    std::string downsample_method = pnh.param<std::string>("downsample_method", "NONE");
    double downsample_resolution = pnh.param<double>("downsample_resolution", 0.1);
    if(downsample_method == "VOXELGRID") {
      std::cout << "downsample: VOXELGRID " << downsample_resolution << std::endl;
      boost::shared_ptr<pcl::VoxelGrid<PointT>> voxelgrid(new pcl::VoxelGrid<PointT>());
      voxelgrid->setLeafSize(downsample_resolution, downsample_resolution, downsample_resolution);
      downsample_filter = voxelgrid;
    } else if(downsample_method == "APPROX_VOXELGRID") {
      std::cout << "downsample: APPROX_VOXELGRID " << downsample_resolution << std::endl;
      boost::shared_ptr<pcl::ApproximateVoxelGrid<PointT>> approx_voxelgrid(new pcl::ApproximateVoxelGrid<PointT>());
      approx_voxelgrid->setLeafSize(downsample_resolution, downsample_resolution, downsample_resolution);
      downsample_filter = approx_voxelgrid;
    } else {
      if(downsample_method != "NONE") {
        std::cerr << "warning: unknown downsampling type (" << downsample_method << ")" << std::endl;
        std::cerr << "       : use passthrough filter" <<std::endl;
      }
      std::cout << "downsample: NONE" << std::endl;
      boost::shared_ptr<pcl::PassThrough<PointT>> passthrough(new pcl::PassThrough<PointT>());
      downsample_filter = passthrough;
    }
    pcl::registration::TransformationEstimationSVD<pcl::PointXYZ, pcl::PointXYZ>::Ptr trans_svd; 
    trans_svd= pcl::registration::TransformationEstimationSVD<PointT,PointT>::Ptr (new pcl::registration::TransformationEstimationSVD<PointT,PointT>);
    boost::shared_ptr<pcl::IterativeClosestPoint<PointT, PointT>> icp(new pcl::IterativeClosestPoint<PointT, PointT>());
    icp->setTransformationEstimation (trans_svd);
       icp->setMaximumIterations (1000);
    icp->setEuclideanFitnessEpsilon(1e-3);


    // pcl::registration::TransformationEstimationCorrentropySVD<pcl::PointXYZ, pcl::PointXYZ>::Ptr trans_svd; 
    // trans_svd= pcl::registration::TransformationEstimationCorrentropySVD<PointT,PointT>::Ptr (new pcl::registration::TransformationEstimationCorrentropySVD<PointT,PointT>);
    // boost::shared_ptr<pcl::IterativeClosestPoint<PointT, PointT>> icp(new pcl::IterativeClosestPoint<PointT, PointT>());
    // icp->setTransformationEstimation (trans_svd);
    // icp->setMaximumIterations (1000);
    // icp->setEuclideanFitnessEpsilon(1e-3);
    registration = icp;
  }

  /**
   * @brief callback for point clouds
   * @param cloud_msg  point cloud msg
   */
  void cloud_callback(const sensor_msgs::PointCloud2ConstPtr& cloud_msg) {
    if(!ros::ok()) {
      return;
    }
    std::cout<<"\nStarted ICP";
    pcl::PointCloud<PointT>::Ptr cloud(new pcl::PointCloud<PointT>());
    pcl::fromROSMsg(*cloud_msg, *cloud);

    Eigen::Matrix4f pose = matching(cloud_msg->header.stamp, cloud);
    publish_odometry(cloud_msg->header.stamp, cloud_msg->header.frame_id, pose);

    // In offline estimation, point clouds until the published time will be supplied
    std_msgs::HeaderPtr read_until(new std_msgs::Header());
    read_until->frame_id = points_topic;
    read_until->stamp = cloud_msg->header.stamp + ros::Duration(1.0, 0);
    read_until_pub.publish(read_until);

    read_until->frame_id = "/filtered_points";
    read_until_pub.publish(read_until);

  }

  /**
   * @brief downsample a point cloud
   * @param cloud  input cloud
   * @return downsampled point cloud
   */
  pcl::PointCloud<PointT>::ConstPtr downsample(const pcl::PointCloud<PointT>::ConstPtr& cloud) const {
    if(!downsample_filter) {
      return cloud;
    }

    pcl::PointCloud<PointT>::Ptr filtered(new pcl::PointCloud<PointT>());
    downsample_filter->setInputCloud(cloud);
    downsample_filter->filter(*filtered);

    return filtered;
  }

  /**
   * @brief estimate the relative pose between an input cloud and a keyframe cloud
   * @param stamp  the timestamp of the input cloud
   * @param cloud  the input cloud
   * @return the relative pose between the input cloud and the keyframe cloud
   */
  Eigen::Matrix4f matching(const ros::Time& stamp, const pcl::PointCloud<PointT>::ConstPtr& cloud) {
    if(!keyframe) {
      prev_trans.setIdentity();
      keyframe_pose.setIdentity();
      keyframe_stamp = stamp;
      keyframe = downsample(cloud);
      registration->setInputTarget(keyframe);
      return Eigen::Matrix4f::Identity();
    }

    auto filtered = downsample(cloud);
    registration->setInputSource(filtered);

    pcl::PointCloud<PointT>::Ptr aligned(new pcl::PointCloud<PointT>());
    registration->align(*aligned, prev_trans);

    if(!registration->hasConverged()) {
       ROS_INFO("scan matching has not converged!!");
       std::cout<<"ignore this frame(" << stamp << ")";
      return keyframe_pose * prev_trans;
    }

    Eigen::Matrix4f trans = registration->getFinalTransformation();
    Eigen::Matrix4f odom = keyframe_pose * trans;

    if(transform_thresholding) {
      Eigen::Matrix4f delta = prev_trans.inverse() * trans;
      double dx = delta.block<3, 1>(0, 3).norm();
      double da = std::acos(Eigen::Quaternionf(delta.block<3, 3>(0, 0)).w());

      if(dx > max_acceptable_trans || da > max_acceptable_angle) {
        std::cout<<"too large transform!!  " << dx << "[m] " << da << "[rad]";
        std::cout<<"ignore this frame(" << stamp << ")";
        return keyframe_pose * prev_trans;
      }
    }

    prev_trans = trans;

    auto keyframe_trans = matrix2transform(stamp, keyframe_pose, odom_frame_id, "keyframe");
    keyframe_broadcaster.sendTransform(keyframe_trans);

    double delta_trans = trans.block<3, 1>(0, 3).norm();
    double delta_angle = std::acos(Eigen::Quaternionf(trans.block<3, 3>(0, 0)).w());
    double delta_time = (stamp - keyframe_stamp).toSec();
    if(delta_trans > keyframe_delta_trans || delta_angle > keyframe_delta_angle || delta_time > keyframe_delta_time) {
      keyframe = filtered;
      registration->setInputTarget(keyframe);

      keyframe_pose = odom;
      keyframe_stamp = stamp;
      prev_trans.setIdentity();
    }

    return odom;
  }

  /**
   * @brief publish odometry
   * @param stamp  timestamp
   * @param pose   odometry pose to be published
   */
  void publish_odometry(const ros::Time& stamp, const std::string& base_frame_id, const Eigen::Matrix4f& pose) {
    // broadcast the transform over tf
    geometry_msgs::TransformStamped odom_trans = matrix2transform(stamp, pose, odom_frame_id, base_frame_id);
    odom_broadcaster.sendTransform(odom_trans);

    // publish the transform
    nav_msgs::Odometry odom;
    odom.header.stamp = stamp;
    odom.header.frame_id = odom_frame_id;

    odom.pose.pose.position.x = pose(0, 3);
    odom.pose.pose.position.y = pose(1, 3);
    odom.pose.pose.position.z = pose(2, 3);
    odom.pose.pose.orientation = odom_trans.transform.rotation;

    odom.child_frame_id = base_frame_id;
    odom.twist.twist.linear.x = 0.0;
    odom.twist.twist.linear.y = 0.0;
    odom.twist.twist.angular.z = 0.0;

    odom_pub.publish(odom);
  }


private:
  // ROS topics
  ros::NodeHandle nh;
  ros::NodeHandle private_nh;

  ros::Subscriber points_sub;

  ros::Publisher odom_pub;
  tf::TransformBroadcaster odom_broadcaster;
  tf::TransformBroadcaster keyframe_broadcaster;

  std::string points_topic;
  std::string odom_frame_id;
  ros::Publisher read_until_pub;

  // keyframe parameters
  double keyframe_delta_trans;  // minimum distance between keyframes
  double keyframe_delta_angle;  //
  double keyframe_delta_time;   //

  // registration validation by thresholding
  bool transform_thresholding;  //
  double max_acceptable_trans;  //
  double max_acceptable_angle;

  // odometry calculation
  Eigen::Matrix4f prev_trans;                  // previous estimated transform from keyframe
  Eigen::Matrix4f keyframe_pose;               // keyframe pose
  ros::Time keyframe_stamp;                    // keyframe time
  pcl::PointCloud<PointT>::ConstPtr keyframe;  // keyframe point cloud

  //
  pcl::Filter<PointT>::Ptr downsample_filter;
  pcl::Registration<PointT, PointT>::Ptr registration;
};



