#include <pcl_icp_mcc/icp_mcc.hpp>
#include <pcl_icp_mcc/preprocessing.hpp>
#include <pcl_icp_mcc/ICP_Node.hpp>
#include <pcl_icp_mcc/Pose_optimization.hpp>
int main(int argc, char** argv)
{
  ros::init(argc, argv, "sub_pcl"); 
  std::cout<<"\nstarting node";
  PrefilteringROSNode  pre;
  ICPNode icp;
  PoseOptimizationNode pn;
  pre.onInit();
  icp.onInit();
  pn.onInit();
  //icp_mcc_node node;
  //node.ros_init(); 
  ros::spin();
  return 0;
}