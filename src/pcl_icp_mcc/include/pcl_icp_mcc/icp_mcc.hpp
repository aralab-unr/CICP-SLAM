#include <ros/ros.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>
#include <sensor_msgs/PointCloud2.h>

#include <boost/make_shared.hpp>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/point_representation.h>
#include <pcl/impl/point_types.hpp>
#include <pcl/io/pcd_io.h>

#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/filter.h>
#include <pcl/common/time.h>

#include <message_filters/subscriber.h>
#include <pcl/features/normal_3d.h>

#include <pcl/registration/icp.h>
#include <pcl/registration/icp_nl.h>
#include <pcl/registration/transforms.h>
#include <pcl/registration/registration.h>

#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/registration/gicp.h>
#include <nav_msgs/Odometry.h>
//#include <pcl/features/from_meshes.h>
//#include <pcl/filters/uniform_sampling.h>
#include <pcl/registration/correspondence_estimation.h>
#include <pcl/registration/correspondence_rejection_distance.h>
#include <pcl/registration/correspondence_rejection_median_distance.h>
#include <pcl/registration/correspondence_rejection_sample_consensus.h>
#include <pcl/features/fpfh.h>
#include <pcl/filters/passthrough.h>
#include <pcl/features/pfh.h>
#include <pcl/registration/ia_ransac.h>
#include <pcl/features/multiscale_feature_persistence.h>
#include <pcl/keypoints/sift_keypoint.h>
#include <time.h>
#include <pcl/surface/mls.h>
#include <pcl/surface/poisson.h>
#include <pcl/features/normal_3d_omp.h>
#include <pcl/io/ply_io.h>
#include <pcl/common/common.h>
#include <pcl/io/vtk_io.h>
#include <pcl/surface/mls.h>
#include <pcl/registration/transformation_estimation_point_to_plane_lls.h>
#include <pcl/registration/transformation_estimation_point_to_plane.h>
#include <boost/filesystem.hpp>
#include <omp.h>
#include <pcl/registration/transformation_estimation_point_to_plane_weighted.h>
#include <pcl/registration/transformation_estimation_point_to_plane_lls_weighted.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_datatypes.h>
#include "tf_conversions/tf_eigen.h"
#include <tf/transform_listener.h>
#include <std_msgs/Float32.h>
#include <pcl/io/ply_io.h>
#include <boost/thread.hpp>
#include <boost/filesystem.hpp>
#include <boost/algorithm/string.hpp>

#include <pcl/filters/filter.h>
#include <boost/unordered_map.hpp>
//#include <pcl/filters/impl/uniform_sampling.hpp>
#include <pcl/common/common.h>
//#include <pcl/filters/uniform_sampling.h>
#include <pcl/registration/transformation_estimation_correntropy_svd.h>
#include <visualization_msgs/Marker.h>
#define PI 3.14159265
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
//using namespace sensor_msgs;
//using namespace message_filters;
typedef pcl::PointXYZ PointT;
typedef pcl::PointCloud<PointT> PointCloud;
typedef pcl::PointNormal PointNormalT;
typedef pcl::PointCloud<PointNormalT> PointCloudWithNormals;
//typedef pcl::PointCloud<pcl::PointXYZ> PointCloud;

struct Pose3d {
  Eigen::Vector3d p;
  Eigen::Quaterniond q;
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};
struct Constraint3d {
  int id_begin;
  int id_end;

  // The transformation that represents the pose of the end frame E w.r.t. the
  // begin frame B. In other words, it transforms a vector in the E frame to
  // the B frame.
  Pose3d t_be;

  // The inverse of the covariance matrix for the measurement. The order of the
  // entries are x, y, z, delta orientation.
  Eigen::Matrix<double, 6, 6> information;

  // The name of the data type in the g2o file format.
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

class icp_mcc_node
{
  private:
  	PointCloud::Ptr source,target,map;
    pcl::IterativeClosestPoint<PointT,PointT>::Ptr icp;
    pcl::PointCloud<PointT>::Ptr registration_output;
    pcl::registration::TransformationEstimationCorrentropySVD<pcl::PointXYZ, pcl::PointXYZ>::Ptr trans_svd; 
    //PointCloud::Ptr cloud_src,cloud_tgt;
    int ctr=0;
    double icp_euclidean_fitness_epsilon=0.01f,icp_set_max_iter;
    Eigen::Matrix4f Global_transform;
    tf::TransformBroadcaster br;
    tf::Transform transform;
    visualization_msgs::Marker marker;
    ros::Publisher marker_pub;
    ros::Publisher pcl_icp_publisher_;
    //message_filters::TimeSynchronizer<nav_msgs::Odometry, sensor_msgs::PointCloud2> sync1;
    std::unique_ptr<message_filters::Subscriber<nav_msgs::Odometry>> odom_sub;
    std::unique_ptr<message_filters::Subscriber<sensor_msgs::PointCloud2>> cloud_sub;
    std::unique_ptr<message_filters::TimeSynchronizer<nav_msgs::Odometry, sensor_msgs::PointCloud2>> sync;
    ofstream g2o_file;

    //std::unique_ptr<message_filters::Subscriber<nav_msgs::Odometry>> odom_sub;
  public:
    ros::NodeHandle nh;
    ros::Subscriber sub;
    

	icp_mcc_node()
	{
		source=PointCloud::Ptr(new PointCloud);
        target=PointCloud::Ptr(new PointCloud);
        map=PointCloud::Ptr(new PointCloud);
        icp = pcl::IterativeClosestPoint<PointT,PointT>::Ptr( new pcl::IterativeClosestPoint<PointT,PointT>());
        registration_output=pcl::PointCloud<PointT>::Ptr(new pcl::PointCloud<PointT> () );
        trans_svd= pcl::registration::TransformationEstimationCorrentropySVD<PointT,PointT>::Ptr (new pcl::registration::TransformationEstimationCorrentropySVD<PointT,PointT>);
        Global_transform = Eigen::Matrix4f::Identity();
	};
	~icp_mcc_node(){};

void param_init()
 {
   // ros::param::get("icp_ransac_outlier_rejection",icp_ransac_outlier_rejection);
   // ros::param::get("icp_transformation_epsilon",icp_transformation_epsilon);
   // ros::param::get("icp_set_max_iter",icp_set_max_iter);
   // ros::param::get("icp_euclidean_fitness_epsilon",icp_euclidean_fitness_epsilon);
   
 }

void process_icp(PointCloud::Ptr cloud_src,PointCloud::Ptr cloud_tgt,Eigen::Matrix4f &final_transform)
{
	//g2o_file<<"VERTEX_SE3:QUAT "<<ctr<<" "<<origin.x()<<" "<<origin.y()<<" "<<origin.z()<<" "<<tfqt.x()<<" "<<tfqt.y()<<" "<<tfqt.z()<<" "<<tfqt.w()<<"\n";
    
	icp->setMaximumIterations (1000);
    icp->setTransformationEstimation (trans_svd);
    //Set the maximum allowed Euclidean error between two consecutive steps in the ICP loop, before the algorithm is considered to have converged
    icp->setEuclideanFitnessEpsilon(icp_euclidean_fitness_epsilon);
      //icp.setRANSACIterations(ransac_iter);

    //icp.setTransformationEstimation(trans_lls);
 
      //PCL_INFO("\nICP started 2\n");
    icp->setInputSource(cloud_src); // from (1)
    icp->setInputTarget(cloud_tgt);
   // std::cout<<endl<<initial_guess<<endl;

      //Perform ICP with alignment with initial_guess
      icp->align (*registration_output);
     // *map+=*registration_output;
    // pcl::VoxelGrid<PointT> sor;
    // pcl::PointCloud<PointT>::Ptr cloud_filtered (new pcl::PointCloud<PointT>());
    //  sor.setInputCloud (map);
    //  sor.setLeafSize (0.5f, 0.5f, 0.5f);
    //  sor.filter (*cloud_filtered);
     
    //  sensor_msgs::PointCloud2 pcl2_msg_;
    //  pcl::toROSMsg(*cloud_filtered, pcl2_msg_); 
    //  pcl2_msg_.header.frame_id = "ouster_link";
    //  pcl2_msg_.header.stamp=ros::Time::now();;
    //  pcl_icp_publisher_.publish(pcl2_msg_);

    //PCL_INFO("\n ICPc=%d ",registration_output.size());

    final_transform=final_transform*icp->getFinalTransformation();
    std::cout<<final_transform;
    
    pcl::copyPointCloud(*cloud_src,*target);
    //pcl::transformPointCloud(*cloud_src,*target,final_transform);
    tf::Vector3 origin;
    origin.setValue(final_transform(0,3),final_transform(1,3),final_transform(2,3));

  cout << "\norignal_vals= "<<origin.x()<<origin.y()<<origin.z() << endl;
  tf::Matrix3x3 tf3d;
  tf3d.setValue((final_transform(0,0)), (final_transform(0,1)), (final_transform(0,2)), 
        (final_transform(1,0)), (final_transform(1,1)), (final_transform(1,2)), 
        (final_transform(2,0)), (final_transform(2,1)), (final_transform(2,2)));

  tf::Quaternion tfqt;
  tf3d.getRotation(tfqt);

  tf::Transform transform;
  transform.setOrigin(origin);
  transform.setRotation(tfqt);

   uint32_t shape = visualization_msgs::Marker::CUBE;
    marker.header.frame_id = "velo_link";
    marker.header.stamp = ros::Time::now();

    // Set the namespace and id for this marker.  This serves to create a unique ID
    // Any marker sent with the same namespace and id will overwrite the old one
    marker.ns = "basic_shapes";
    marker.id = ctr;

    // Set the marker type.  Initially this is CUBE, and cycles between that and SPHERE, ARROW, and CYLINDER
    marker.type = shape;

    // Set the marker action.  Options are ADD, DELETE, and new in ROS Indigo: 3 (DELETEALL)
    marker.action = visualization_msgs::Marker::ADD;

    // Set the pose of the marker.  This is a full 6DOF pose relative to the frame/time specified in the header
    marker.pose.position.x = origin.x();
    marker.pose.position.y = origin.y();
    marker.pose.position.z = origin.z();
    //tf::convert(tfqt,marker.pose.orientation);
    marker.pose.orientation.x = tfqt.x();
    marker.pose.orientation.y = tfqt.y();
    marker.pose.orientation.z = tfqt.z();
    marker.pose.orientation.w = tfqt.w();
    
    // Set the scale of the marker -- 1x1x1 here means 1m on a side
    marker.scale.x = 0.5;
    marker.scale.y = 0.5;
    marker.scale.z = 0.5;

    // Set the color -- be sure to set alpha to something non-zero!
    marker.color.r = 0.0f;
    marker.color.g = 1.0f;
    marker.color.b = 0.0f;
    marker.color.a = 1.0;

    marker.lifetime = ros::Duration();
  //br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "map", "odom"));
    marker_pub.publish(marker);

     //end = clock();
     //tot_icp+=((double)(end-start)/CLOCKS_PER_SEC);

     //icp_time=(float)timer_component.getTimeSeconds();
    ROS_INFO("STEP 5: ICP completeed\n");
}
void get_point_clouds(const nav_msgs::OdometryConstPtr& odom_msg,const sensor_msgs::PointCloud2ConstPtr& msg)
 {
      std::cout<<"\nGet point clouds!!"; 
      pcl::PointCloud<PointT>::Ptr cloud(new pcl::PointCloud<PointT>());
      pcl::fromROSMsg(*msg, *cloud);

      //pcl::PCLPointCloud2 cloud_original;
      //pcl_conversions::toPCL(*msg,cloud_original);
      //PointCloud::Ptr pcl_cloud_original(new PointCloud);
      PointCloud::Ptr cloud_no_nan(new PointCloud);
     // pcl::fromPCLPointCloud2(cloud,*pcl_cloud_original);
   //Remove Nan vals from point clouds 
      std::vector<int> indices;
      pcl::removeNaNFromPointCloud(*cloud,*cloud_no_nan, indices);
       pcl::VoxelGrid<PointT> sor;
      sor.setInputCloud (cloud_no_nan);
      sor.setLeafSize (2.0f, 2.0f, 2.0f);
      sor.filter (*cloud_no_nan);
      if(ctr==0) 
      	{
         g2o_file<<"VERTEX_SE3:QUAT "<<ctr<<" 0.000000 0.000000 0.000000 0.0000000 0.0000000 0.0000000 1.0000000\n";
         pcl::copyPointCloud(*cloud_no_nan,*target);
         //ctr++;
        }
       else
       {
       	 pcl::copyPointCloud(*cloud_no_nan,*source);
       	 process_icp(source,target,Global_transform);
         //ctr++;
       }
    ctr++;

 }
void ros_init()
 {
   ROS_INFO("Started system\n");
   g2o_file.open("kitti_odom.g2o");
   //odom_sub.subscribe(nh,"/odom",1000);
   //cloud_sub.subscribe(nh,"/points",1000);

   odom_sub.reset(new message_filters::Subscriber<nav_msgs::Odometry>(nh, "/odom", 1000));
   cloud_sub.reset(new message_filters::Subscriber<sensor_msgs::PointCloud2>(nh, "/kitti/velo/pointcloud", 1000));
    
   // sub=nh.subscribe<sensor_msgs::PointCloud2>("/kitti/velo/pointcloud", 1, &icp_mcc_node::get_point_clouds,this);

   sync.reset(new message_filters::TimeSynchronizer<nav_msgs::Odometry, sensor_msgs::PointCloud2>(*odom_sub, *cloud_sub, 1000));
   sync->registerCallback(boost::bind(&icp_mcc_node::get_point_clouds, this, _1, _2));

   //sync_.reset(new Sync(MySyncPolicy(10),odom_sub, cloud_sub));
   //sync_->registerCallback(boost::bind(&icp_mcc_node::get_point_clouds, this, _1, _2));

   marker_pub = nh.advertise<visualization_msgs::Marker>("visualization_marker", 1);
   pcl_icp_publisher_ = nh.advertise<sensor_msgs::PointCloud2>("/lidar/pcl/icp", 1000000);
   
 }
};
