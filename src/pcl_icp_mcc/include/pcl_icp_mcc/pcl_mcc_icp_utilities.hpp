#include <Eigen/Dense>

#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/TransformStamped.h>
#include <unordered_map>
#include <boost/functional/hash.hpp>

#include <ros/time.h>
#include <boost/format.hpp>
#include <g2o/stuff/macros.h>
#include <g2o/core/factory.h>
#include <g2o/core/block_solver.h>
#include <g2o/core/linear_solver.h>
#include <g2o/core/sparse_optimizer.h>
#include <g2o/core/robust_kernel_factory.h>
#include <g2o/core/optimization_algorithm_factory.h>
#include <g2o/solvers/pcg/linear_solver_pcg.h>
#include <g2o/types/slam3d/types_slam3d.h>
#include <g2o/types/slam3d/edge_se3_pointxyz.h>
#include <g2o/types/slam3d_addons/types_slam3d_addons.h>
#include <g2o/edge_se3_plane.hpp>
#include <g2o/edge_se3_priorxy.hpp>
#include <g2o/edge_se3_priorxyz.hpp>
#include <g2o/edge_se3_priorvec.hpp>
#include <g2o/edge_se3_priorquat.hpp>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <boost/optional.hpp>

class RosTimeHash 
{
public:
  size_t operator() (const ros::Time& val) const 
  {
    size_t seed = 0;
    boost::hash_combine(seed, val.sec);
    boost::hash_combine(seed, val.nsec);
    return seed;
 }
};
//G2o workplaces.

G2O_USE_OPTIMIZATION_LIBRARY(pcg)
G2O_USE_OPTIMIZATION_LIBRARY(cholmod)
G2O_USE_OPTIMIZATION_LIBRARY(csparse)

namespace g2o 
{
  class VertexSE3;
  //class VertexPlane;
  class VertexPointXYZ;
  class EdgeSE3;
  //class EdgeSE3Plane;
  class EdgeSE3PointXYZ;
  class EdgeSE3PriorXY;
  class EdgeSE3PriorXYZ;
  class EdgeSE3PriorVec;
  class EdgeSE3PriorQuat;
  class RobustKernelFactory;
  G2O_REGISTER_TYPE(EDGE_SE3_PLANE, EdgeSE3Plane)
  G2O_REGISTER_TYPE(EDGE_SE3_PRIORXY, EdgeSE3PriorXY)
  G2O_REGISTER_TYPE(EDGE_SE3_PRIORXYZ, EdgeSE3PriorXYZ)
  G2O_REGISTER_TYPE(EDGE_SE3_PRIORVEC, EdgeSE3PriorVec)
  G2O_REGISTER_TYPE(EDGE_SE3_PRIORQUAT, EdgeSE3PriorQuat)

}

class GraphSLAM 
{
public:
  g2o::RobustKernelFactory* robust_kernel_factory;
  std::unique_ptr<g2o::SparseOptimizer> graph;  // g2o graph
public:
  GraphSLAM(const std::string& solver_type = "lm_var")
  {
  	graph.reset(new g2o::SparseOptimizer());

  	std::cout << "construct solver... " << std::endl;
    g2o::OptimizationAlgorithmFactory* solver_factory = g2o::OptimizationAlgorithmFactory::instance();
    g2o::OptimizationAlgorithmProperty solver_property;
    g2o::OptimizationAlgorithm* solver = solver_factory->construct(solver_type, solver_property);
    graph->setAlgorithm(solver);

    if (!graph->solver()) {
     std::cerr << std::endl;
     std::cerr << "error : failed to allocate solver!!" << std::endl;
     solver_factory->listSolvers(std::cerr);
     std::cerr << "-------------" << std::endl;
     std::cin.ignore(1);
    return;
  }
  std::cout << "done" << std::endl;

  robust_kernel_factory = g2o::RobustKernelFactory::instance();
  }
  ~GraphSLAM()
  {
  	graph.reset();
  }

  /**
   * @brief add a SE3 node to the graph
   * @param pose
   * @return registered node
   */
  g2o::VertexSE3* add_se3_node(const Eigen::Isometry3d& pose)
  {
   g2o::VertexSE3* vertex(new g2o::VertexSE3());
   vertex->setId(static_cast<int>(graph->vertices().size()));
   vertex->setEstimate(pose);
   graph->addVertex(vertex);

   return vertex;
  }

  /**
   * @brief add a plane node to the graph
   * @param plane_coeffs
   * @return registered node
   */
 // g2o::VertexPlane* add_plane_node(const Eigen::Vector4d& plane_coeffs);

  /**
   * @brief add a point_xyz node to the graph
   * @param xyz
   * @return registered node
   */
  g2o::VertexPointXYZ* add_point_xyz_node(const Eigen::Vector3d& xyz)
  {
  	g2o::VertexPointXYZ* vertex(new g2o::VertexPointXYZ());
    vertex->setId(static_cast<int>(graph->vertices().size()));
    vertex->setEstimate(xyz);
    graph->addVertex(vertex);

    return vertex;
  }

  /**
   * @brief add an edge between SE3 nodes
   * @param v1  node1
   * @param v2  node2
   * @param relative_pose  relative pose between node1 and node2
   * @param information_matrix  information matrix (it must be 6x6)
   * @return registered edge
   */
  g2o::EdgeSE3* add_se3_edge(g2o::VertexSE3* v1, g2o::VertexSE3* v2, const Eigen::Isometry3d& relative_pose, const Eigen::MatrixXd& information_matrix)
  {
  	g2o::EdgeSE3* edge(new g2o::EdgeSE3());
    edge->setMeasurement(relative_pose);
    edge->setInformation(information_matrix);
    edge->vertices()[0] = v1;
    edge->vertices()[1] = v2;
    graph->addEdge(edge);

    return edge;
  }

  /**
   * @brief add an edge between an SE3 node and a plane node
   * @param v_se3    SE3 node
   * @param v_plane  plane node
   * @param plane_coeffs  plane coefficients w.r.t. v_se3
   * @param information_matrix  information matrix (it must be 3x3)
   * @return registered edge
   */
  //g2o::EdgeSE3Plane* add_se3_plane_edge(g2o::VertexSE3* v_se3, g2o::VertexPlane* v_plane, const Eigen::Vector4d& plane_coeffs, const Eigen::MatrixXd& information_matrix);

  /**
   * @brief add an edge between an SE3 node and a point_xyz node
   * @param v_se3        SE3 node
   * @param v_xyz        point_xyz node
   * @param xyz          xyz coordinate
   * @param information  information_matrix (it must be 3x3)
   * @return registered edge
   */
  g2o::EdgeSE3PointXYZ* add_se3_point_xyz_edge(g2o::VertexSE3* v_se3, g2o::VertexPointXYZ* v_xyz, const Eigen::Vector3d& xyz, const Eigen::MatrixXd& information_matrix)
  {
  	 g2o::EdgeSE3PointXYZ* edge(new g2o::EdgeSE3PointXYZ());
     edge->setMeasurement(xyz);
     edge->setInformation(information_matrix);
     edge->vertices()[0] = v_se3;
     edge->vertices()[1] = v_xyz;
     graph->addEdge(edge);

     return edge;
  }

  /**
   * @brief add a prior edge to an SE3 node
   * @param v_se3
   * @param xy
   * @param information_matrix
   * @return
   */
  g2o::EdgeSE3PriorXY* add_se3_prior_xy_edge(g2o::VertexSE3* v_se3, const Eigen::Vector2d& xy, const Eigen::MatrixXd& information_matrix)
  {
  	g2o::EdgeSE3PriorXY* edge(new g2o::EdgeSE3PriorXY());
    edge->setMeasurement(xy);
    edge->setInformation(information_matrix);
    edge->vertices()[0] = v_se3;
    graph->addEdge(edge);

    return edge;
  }

  g2o::EdgeSE3PriorXYZ* add_se3_prior_xyz_edge(g2o::VertexSE3* v_se3, const Eigen::Vector3d& xyz, const Eigen::MatrixXd& information_matrix)
  {
    g2o::EdgeSE3PriorXYZ* edge(new g2o::EdgeSE3PriorXYZ());
    edge->setMeasurement(xyz);
    edge->setInformation(information_matrix);
    edge->vertices()[0] = v_se3;
    graph->addEdge(edge);  	
    return edge;
  }

  g2o::EdgeSE3PriorQuat* add_se3_prior_quat_edge(g2o::VertexSE3* v_se3, const Eigen::Quaterniond& quat, const Eigen::MatrixXd& information_matrix)
  {
  	g2o::EdgeSE3PriorQuat* edge(new g2o::EdgeSE3PriorQuat());
    edge->setMeasurement(quat);
    edge->setInformation(information_matrix);
    edge->vertices()[0] = v_se3;
    graph->addEdge(edge);

    return edge;
  }

  g2o::EdgeSE3PriorVec* add_se3_prior_vec_edge(g2o::VertexSE3* v_se3, const Eigen::Vector3d& direction, const Eigen::Vector3d& measurement, const Eigen::MatrixXd& information_matrix)
  {
  	Eigen::Matrix<double, 6, 1> m;
    m.head<3>() = direction;
    m.tail<3>() = measurement;

    g2o::EdgeSE3PriorVec* edge(new g2o::EdgeSE3PriorVec());
    edge->setMeasurement(m);
    edge->setInformation(information_matrix);
    edge->vertices()[0] = v_se3;
    graph->addEdge(edge);

    return edge;
  }

  void add_robust_kernel(g2o::OptimizableGraph::Edge* edge, const std::string& kernel_type, double kernel_size)
  {
  	if(kernel_type == "NONE") 
  	{
     return;
    }

  g2o::RobustKernel* kernel = robust_kernel_factory->construct(kernel_type);
  if(kernel == nullptr) 
   {
    std::cerr << "warning : invalid robust kernel type: " << kernel_type << std::endl;
    return;
   }

  kernel->setDelta(kernel_size);
  edge->setRobustKernel(kernel);
 }

  /**
   * @brief perform graph optimization
   */
  void optimize(int num_iterations)
  {
  	
   if(graph->edges().size() < 3) 
  	{
    return;
    }

  std::cout << std::endl;
  std::cout << "--- pose graph optimization ---" << std::endl;
  std::cout << "nodes: " << graph->vertices().size() << "   edges: " << graph->edges().size() << std::endl;
  std::cout << "optimizing... " << std::flush;

  graph->initializeOptimization();
  graph->computeInitialGuess();
  graph->computeActiveErrors();
  graph->setVerbose(true);

  double chi2 = graph->chi2();

  auto t1 = ros::Time::now();
  int iterations = graph->optimize(num_iterations);

  auto t2 = ros::Time::now();
  std::cout << "done" << std::endl;
  std::cout << "iterations: " << iterations << " / " << num_iterations << std::endl;
  std::cout << "chi2: (before)" << chi2 << " -> (after)" << graph->chi2() << std::endl;
  std::cout << "time: " << boost::format("%.3f") % (t2 - t1).toSec() << "[sec]" << std::endl;
  }

  /**
   * @brief save the pose graph
   * @param filename  output filename
   */
  void save(const std::string& filename)
  {
   std::ofstream ofs(filename);
   graph->save(ofs);
  }

public:
  
};

struct KeyFrame {
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  using PointT = pcl::PointXYZ;
  using Ptr = std::shared_ptr<KeyFrame>;

  KeyFrame(const ros::Time& stamp, const Eigen::Isometry3d& odom, double accum_distance, const pcl::PointCloud<PointT>::ConstPtr& cloud)
  : stamp(stamp),
    odom(odom),
    accum_distance(accum_distance),
    cloud(cloud),
    node(nullptr)
  {

  }
  ~KeyFrame()
  {

  }

  void dump(const std::string& directory) 
  {
    if(!boost::filesystem::is_directory(directory)) {
      boost::filesystem::create_directory(directory);
      }

    std::ofstream ofs(directory + "/data");
    ofs << "stamp " << stamp.sec << " " << stamp.nsec << "\n";

    ofs << "estimate\n";
    ofs << node->estimate().matrix() << "\n";

    ofs << "odom\n";
    ofs << odom.matrix() << "\n";

    ofs << "accum_distance " << accum_distance << "\n";

    // if(floor_coeffs) 
    //  {
    //     ofs << "floor_coeffs " << floor_coeffs->transpose() << "\n";
    //  }

  if(node) {
    ofs << "id " << node->id() << "\n";
  }
pcl::io::savePCDFileBinary(directory + "/cloud.pcd", *cloud);

}

public:
  ros::Time stamp;                                // timestamp
  Eigen::Isometry3d odom;                         // odometry (estimated by scan_matching_odometry)
  double accum_distance;                          // accumulated distance from the first node (by scan_matching_odometry)
  pcl::PointCloud<PointT>::ConstPtr cloud;        // point cloud
  //boost::optional<Eigen::Vector4d> floor_coeffs;  // detected floor's coefficients
  //boost::optional<Eigen::Vector3d> utm_coord;     // UTM coord obtained by GPS

  boost::optional<Eigen::Vector3d> acceleration;   //
  boost::optional<Eigen::Quaterniond> orientation; //

  g2o::VertexSE3* node;                           // node instance
};

/**
 * @brief KeyFramesnapshot for map cloud generation
 */
struct KeyFrameSnapshot 
{
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  using PointT = KeyFrame::PointT;
  using Ptr = std::shared_ptr<KeyFrameSnapshot>;

  KeyFrameSnapshot(const KeyFrame::Ptr& key): pose(key->node->estimate()),cloud(key->cloud)
    {

    }

  KeyFrameSnapshot(const Eigen::Isometry3d& pose, const pcl::PointCloud<PointT>::ConstPtr& cloud): pose(pose),cloud(cloud)
   {

   }

  ~KeyFrameSnapshot()
  {

  }

public:
  Eigen::Isometry3d pose;                   // pose estimated by graph optimization
  pcl::PointCloud<PointT>::ConstPtr cloud;  // point cloud
};

class KeyframeUpdater 
{
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  /**
   * @brief constructor
   * @param pnh
   */
  KeyframeUpdater(ros::NodeHandle& pnh)
    : is_first(true),
      prev_keypose(Eigen::Isometry3d::Identity())
  {
    keyframe_delta_trans = 0.5;//pnh.param<double>("keyframe_delta_trans", 0.5);
    keyframe_delta_angle = 0.5;////pnh.param<double>("keyframe_delta_angle", 0.5);

    accum_distance = 0.0;
  }

  /**
   * @brief decide if a new frame should be registered to the graph
   * @param pose  pose of the frame
   * @return  if true, the frame should be registered
   */
  bool update(const Eigen::Isometry3d& pose) 
  {
    // first frame is always registered to the graph
    if(is_first) 
    {
      is_first = false;
      prev_keypose = pose;
      return true;
    }

    // calculate the delta transformation from the previous keyframe
    Eigen::Isometry3d delta = prev_keypose.inverse() * pose;
    double dx = delta.translation().norm();
    //std::cout<<"\n dx= "<<dx;
    double da = std::acos(Eigen::Quaterniond(delta.linear()).w());
    std::cout<<"\n dx= "<<dx<<" da= "<<da;
    // too close to the previous frame
    if(dx < keyframe_delta_trans && da < keyframe_delta_angle)
    {
      return false;
    }

    accum_distance += dx;
    prev_keypose = pose;
    return true;
  }

  /**
   * @brief the last keyframe's accumulated distance from the first keyframe
   * @return accumulated distance
   */
  double get_accum_distance() const {
    return accum_distance;
  }

private:
  // parameters
  double keyframe_delta_trans;      //
  double keyframe_delta_angle;      //

  bool is_first;
  double accum_distance;
  Eigen::Isometry3d prev_keypose;
};


struct Loop {
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  using Ptr = std::shared_ptr<Loop>;

  Loop(const KeyFrame::Ptr& key1, const KeyFrame::Ptr& key2, const Eigen::Matrix4f& relpose)
    : key1(key1),
      key2(key2),
      relative_pose(relpose)
  {}

public:
  KeyFrame::Ptr key1;
  KeyFrame::Ptr key2;
  Eigen::Matrix4f relative_pose;
};

/**
 * @brief this class finds loops by scam matching and adds them to the pose graph
 */
class LoopDetector {
public:
  typedef pcl::PointXYZ PointT;

  /**
   * @brief constructor
   * @param pnh
   */
  LoopDetector(ros::NodeHandle& pnh) {
    distance_thresh = pnh.param<double>("distance_thresh", 5.0);
    accum_distance_thresh = pnh.param<double>("accum_distance_thresh", 8.0);
    distance_from_last_edge_thresh = pnh.param<double>("min_edge_interval", 5.0);

    fitness_score_max_range = pnh.param<double>("fitness_score_max_range", std::numeric_limits<double>::max());
    fitness_score_thresh = pnh.param<double>("fitness_score_thresh", 0.5);

    // pcl::registration::TransformationEstimationCorrentropySVD<pcl::PointXYZ, pcl::PointXYZ>::Ptr trans_svd; 
    // trans_svd= pcl::registration::TransformationEstimationCorrentropySVD<PointT,PointT>::Ptr (new pcl::registration::TransformationEstimationCorrentropySVD<PointT,PointT>);
    // boost::shared_ptr<pcl::IterativeClosestPoint<PointT, PointT>> icp(new pcl::IterativeClosestPoint<PointT, PointT>());
    // icp->setMaximumIterations (1000);
    // icp->setEuclideanFitnessEpsilon(1e-3);
    // icp->setTransformationEstimation (trans_svd);

    
     pcl::registration::TransformationEstimationSVD<pcl::PointXYZ, pcl::PointXYZ>::Ptr trans_svd; 
    trans_svd= pcl::registration::TransformationEstimationSVD<PointT,PointT>::Ptr (new pcl::registration::TransformationEstimationSVD<PointT,PointT>);
    boost::shared_ptr<pcl::IterativeClosestPoint<PointT, PointT>> icp(new pcl::IterativeClosestPoint<PointT, PointT>());
    icp->setMaximumIterations (1000);
    icp->setEuclideanFitnessEpsilon(1e-3);
    icp->setTransformationEstimation (trans_svd);
    
    registration = icp;
    last_edge_accum_distance = 0.0;
  }

  /**
   * @brief detect loops and add them to the pose graph
   * @param keyframes       keyframes
   * @param new_keyframes   newly registered keyframes
   * @param graph_slam      pose graph
   */
  std::vector<Loop::Ptr> detect(const std::vector<KeyFrame::Ptr>& keyframes, const std::deque<KeyFrame::Ptr>& new_keyframes, GraphSLAM& graph_slam) 
  {
    std::vector<Loop::Ptr> detected_loops;
    for(const auto& new_keyframe : new_keyframes) {
      auto candidates = find_candidates(keyframes, new_keyframe);
      auto loop = matching(candidates, new_keyframe, graph_slam);
      if(loop) {
        detected_loops.push_back(loop);
      }
    }

    return detected_loops;
  }

  double get_distance_thresh() const {
    return distance_thresh;
  }

private:
  /**
   * @brief find loop candidates. A detected loop begins at one of #keyframes and ends at #new_keyframe
   * @param keyframes      candidate keyframes of loop start
   * @param new_keyframe   loop end keyframe
   * @return loop candidates
   */
  std::vector<KeyFrame::Ptr> find_candidates(const std::vector<KeyFrame::Ptr>& keyframes, const KeyFrame::Ptr& new_keyframe) const {
    // too close to the last registered loop edge
    if(new_keyframe->accum_distance - last_edge_accum_distance < distance_from_last_edge_thresh) {
      return std::vector<KeyFrame::Ptr>();
    }

    std::vector<KeyFrame::Ptr> candidates;
    candidates.reserve(32);

    for(const auto& k : keyframes) {
      // traveled distance between keyframes is too small
      if(new_keyframe->accum_distance - k->accum_distance < accum_distance_thresh) {
        continue;
      }

      const auto& pos1 = k->node->estimate().translation();
      const auto& pos2 = new_keyframe->node->estimate().translation();

      // estimated distance between keyframes is too small
      double dist = (pos1.head<2>() - pos2.head<2>()).norm();
      if(dist > distance_thresh) {
        continue;
      }

      candidates.push_back(k);
    }

    return candidates;
  }

  /**
   * @brief To validate a loop candidate this function applies a scan matching between keyframes consisting the loop. If they are matched well, the loop is added to the pose graph
   * @param candidate_keyframes  candidate keyframes of loop start
   * @param new_keyframe         loop end keyframe
   * @param graph_slam           graph slam
   */
  Loop::Ptr matching(const std::vector<KeyFrame::Ptr>& candidate_keyframes, const KeyFrame::Ptr& new_keyframe,GraphSLAM& graph_slam) {
    if(candidate_keyframes.empty()) {
      return nullptr;
    }

    registration->setInputTarget(new_keyframe->cloud);

    double best_score = std::numeric_limits<double>::max();
    KeyFrame::Ptr best_matched;
    Eigen::Matrix4f relative_pose;

    std::cout << std::endl;
    std::cout << "--- loop detection ---" << std::endl;
    std::cout << "num_candidates: " << candidate_keyframes.size() << std::endl;
    std::cout << "matching" << std::flush;
    auto t1 = ros::Time::now();

    pcl::PointCloud<PointT>::Ptr aligned(new pcl::PointCloud<PointT>());
    for(const auto& candidate : candidate_keyframes) {
      registration->setInputSource(candidate->cloud);
      Eigen::Matrix4f guess = (new_keyframe->node->estimate().inverse() * candidate->node->estimate()).matrix().cast<float>();
      guess(2, 3) = 0.0;
      registration->align(*aligned, guess);
      std::cout << "." << std::flush;

      double score = registration->getFitnessScore(fitness_score_max_range);
      if(!registration->hasConverged() || score > best_score) {
        continue;
      }

      best_score = score;
      best_matched = candidate;
      relative_pose = registration->getFinalTransformation();
    }

    auto t2 = ros::Time::now();
    std::cout << " done" << std::endl;
    std::cout << "best_score: " << boost::format("%.3f") % best_score << "    time: " << boost::format("%.3f") % (t2 - t1).toSec() << "[sec]" << std::endl;

    if(best_score > fitness_score_thresh) {
      std::cout << "loop not found..." << std::endl;
      return nullptr;
    }

    std::cout << "loop found!!" << std::endl;
    std::cout << "relpose: " << relative_pose.block<3, 1>(0, 3) << " - " << Eigen::Quaternionf(relative_pose.block<3, 3>(0, 0)).coeffs().transpose() << std::endl;

    last_edge_accum_distance = new_keyframe->accum_distance;

    return std::make_shared<Loop>(new_keyframe, best_matched, relative_pose);
  }

private:
  double distance_thresh;                 // estimated distance between keyframes consisting a loop must be less than this distance
  double accum_distance_thresh;           // traveled distance between ...
  double distance_from_last_edge_thresh;  // a new loop edge must far from the last one at least this distance

  double fitness_score_max_range;         // maximum allowable distance between corresponding points
  double fitness_score_thresh;            // threshold for scan matching

  double last_edge_accum_distance;

  pcl::Registration<PointT, PointT>::Ptr registration;
};

 
class MapCloudGenerator 
{
public:
  using PointT = pcl::PointXYZ;

  MapCloudGenerator(){};
  ~MapCloudGenerator(){};

  /**
   * @brief generates a map point cloud
   * @param keyframes   snapshots of keyframes
   * @param resolution  resolution of generated map
   * @return generated map point cloud
   */
  pcl::PointCloud<PointT>::Ptr generate(const std::vector<KeyFrameSnapshot::Ptr>& keyframes, double resolution) const
  {
    if(keyframes.empty()) {
     std::cerr << "warning: keyframes empty!!" << std::endl;
     return nullptr;
     }

  pcl::PointCloud<PointT>::Ptr cloud(new pcl::PointCloud<PointT>());
  cloud->reserve(keyframes.front()->cloud->size() * keyframes.size());

  for(const auto& keyframe : keyframes) {
    Eigen::Matrix4f pose = keyframe->pose.matrix().cast<float>();
    for(const auto& src_pt : keyframe->cloud->points) {
      PointT dst_pt;
      dst_pt.getVector4fMap() = pose * src_pt.getVector4fMap();
      //dst_pt.intensity = src_pt.intensity;
      cloud->push_back(dst_pt);
    }
  }

  cloud->width = cloud->size();
  cloud->height = 1;
  cloud->is_dense = false;

  pcl::octree::OctreePointCloud<PointT> octree(resolution);
  octree.setInputCloud(cloud);
  octree.addPointsFromInputCloud();

  pcl::PointCloud<PointT>::Ptr filtered(new pcl::PointCloud<PointT>());
  octree.getOccupiedVoxelCenters(filtered->points);

  filtered->width = filtered->size();
  filtered->height = 1;
  filtered->is_dense = false;

  return filtered;
}

};



class InformationMatrixCalculator 
{
public:
  using PointT = pcl::PointXYZ;

  InformationMatrixCalculator(ros::NodeHandle& nh) 
  {
  use_const_inf_matrix = nh.param<double>("use_const_inf_matrix", false);
  const_stddev_x = nh.param<double>("const_stddev_x", 0.01);
  const_stddev_q = nh.param<double>("const_stddev_q", 0.01);

  var_gain_a = nh.param<double>("var_gain_a", 200.0);
  min_stddev_x = nh.param<double>("min_stddev_x", 0.01);
  max_stddev_x = nh.param<double>("max_stddev_x", 0.05);
  min_stddev_q = nh.param<double>("min_stddev_q", 0.01);
  max_stddev_q = nh.param<double>("max_stddev_q", 0.05);
  fitness_score_thresh = nh.param<double>("fitness_score_thresh", 0.05);
  }
  
  ~InformationMatrixCalculator() {};

double calc_fitness_score(const pcl::PointCloud<PointT>::ConstPtr& cloud1, const pcl::PointCloud<PointT>::ConstPtr& cloud2, const Eigen::Isometry3d& relpose, double max_range = std::numeric_limits<double>::max()) const 
  {
  pcl::search::KdTree<PointT>::Ptr tree_(new pcl::search::KdTree<PointT>());
  tree_->setInputCloud(cloud1);

  double fitness_score = 0.0;

  // Transform the input dataset using the final transformation
  pcl::PointCloud<PointT> input_transformed;
  pcl::transformPointCloud (*cloud2, input_transformed, relpose.cast<float>());

  std::vector<int> nn_indices (1);
  std::vector<float> nn_dists (1);

  // For each point in the source dataset
  int nr = 0;
  for (size_t i = 0; i < input_transformed.points.size (); ++i)
  {
    // Find its nearest neighbor in the target
    tree_->nearestKSearch (input_transformed.points[i], 1, nn_indices, nn_dists);

    // Deal with occlusions (incomplete targets)
    if (nn_dists[0] <= max_range)
    {
      // Add to the fitness score
      fitness_score += nn_dists[0];
      nr++;
    }
  }

  if (nr > 0)
    return (fitness_score / nr);
  else
    return (std::numeric_limits<double>::max ());
} 
  
  Eigen::MatrixXd calc_information_matrix(const pcl::PointCloud<PointT>::ConstPtr& cloud1, const pcl::PointCloud<PointT>::ConstPtr& cloud2, const Eigen::Isometry3d& relpose) const 
  {
   if(use_const_inf_matrix) 
    {
     Eigen::MatrixXd inf = Eigen::MatrixXd::Identity(6, 6);
     inf.topLeftCorner(3, 3).array() /= const_stddev_x;
     inf.bottomRightCorner(3, 3).array() /= const_stddev_q;
     return inf;
    }

  double fitness_score = calc_fitness_score(cloud1, cloud2, relpose);

  double min_var_x = std::pow(min_stddev_x, 2);
  double max_var_x = std::pow(max_stddev_x, 2);
  double min_var_q = std::pow(min_stddev_q, 2);
  double max_var_q = std::pow(max_stddev_q, 2);

  float w_x = weight(var_gain_a, fitness_score_thresh, min_var_x, max_var_x, fitness_score);
  float w_q = weight(var_gain_a, fitness_score_thresh, min_var_q, max_var_q, fitness_score);

  Eigen::MatrixXd inf = Eigen::MatrixXd::Identity(6, 6);
  inf.topLeftCorner(3, 3).array() /= w_x;
  inf.bottomRightCorner(3, 3).array() /= w_q;
  return inf;
}
private:
  double weight(double a, double max_x, double min_y, double max_y, double x) const
  {
    double y = (1.0 - std::exp(-a * x)) / (1.0 - std::exp(-a * max_x));
    return min_y + (max_y - min_y) * y;
  }

private:
  bool use_const_inf_matrix;
  double const_stddev_x;
  double const_stddev_q;

  double var_gain_a;
  double min_stddev_x;
  double max_stddev_x;
  double min_stddev_q;
  double max_stddev_q;
  double fitness_score_thresh;

};


static geometry_msgs::TransformStamped matrix2transform(const ros::Time& stamp, const Eigen::Matrix4f& pose, const std::string& frame_id, const std::string& child_frame_id)
{
  Eigen::Quaternionf quat(pose.block<3, 3>(0, 0));
  quat.normalize();
  geometry_msgs::Quaternion odom_quat;
  odom_quat.w = quat.w();
  odom_quat.x = quat.x();
  odom_quat.y = quat.y();
  odom_quat.z = quat.z();

  geometry_msgs::TransformStamped odom_trans;
  odom_trans.header.stamp = stamp;
  odom_trans.header.frame_id = frame_id;
  odom_trans.child_frame_id = child_frame_id;

  odom_trans.transform.translation.x = pose(0, 3);
  odom_trans.transform.translation.y = pose(1, 3);
  odom_trans.transform.translation.z = pose(2, 3);
  odom_trans.transform.rotation = odom_quat;

  return odom_trans;
}

static Eigen::Isometry3d odom2isometry(const nav_msgs::OdometryConstPtr& odom_msg) 
{
  const auto& orientation = odom_msg->pose.pose.orientation;
  const auto& position = odom_msg->pose.pose.position;

  Eigen::Quaterniond quat;
  quat.w() = orientation.w;
  quat.x() = orientation.x;
  quat.y() = orientation.y;
  quat.z() = orientation.z;

  Eigen::Isometry3d isometry = Eigen::Isometry3d::Identity();
  isometry.linear() = quat.toRotationMatrix();
  isometry.translation() = Eigen::Vector3d(position.x, position.y, position.z);
  return isometry;
}