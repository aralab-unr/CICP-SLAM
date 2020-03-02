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


#include <pcl/features/normal_3d.h>

#include <pcl/registration/icp.h>
#include <pcl/registration/icp_nl.h>
#include <pcl/registration/transforms.h>
#include <pcl/registration/registration.h>

#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/registration/gicp.h>
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

#include <pcl/filters/filter.h>
#include <boost/unordered_map.hpp>
//#include <pcl/filters/impl/uniform_sampling.hpp>
#include <pcl/common/common.h>
//#include <pcl/filters/uniform_sampling.h>
#include <pcl/registration/transformation_estimation_correntropy_svd.h>

#define PI 3.14159265



//#include tf_eigen.h"
using pcl::visualization::PointCloudColorHandlerGenericField;
using pcl::visualization::PointCloudColorHandlerCustom;


//#include "evaluation.h"
//#include "registration.h"
//convenient typedefs
typedef pcl::PointXYZ PointT;
typedef pcl::PointCloud<PointT> PointCloud;
typedef pcl::PointNormal PointNormalT;
typedef pcl::PointCloud<PointNormalT> PointCloudWithNormals;
typedef pcl::PointCloud<pcl::PointXYZ> PointCloud;


pcl::PCLPointCloud2* cloud = new pcl::PCLPointCloud2;

PointCloud::Ptr source (new PointCloud);
PointCloud::Ptr target(new PointCloud);

PointCloud::Ptr all_cloud (new PointCloud);
pcl::PLYWriter ply_writer;

PointCloud result;
 std::string ss;
int frame_count=0;
int file_no=0;
int error_count=0;
float avg_distance=0.0;
int pointcloudoriginalsize=0;
 Eigen::Matrix4f Global_transform = Eigen::Matrix4f::Identity (), pairTransform =Eigen::Matrix4f::Identity (),prev_transform =Eigen::Matrix4f::Identity ();

int snof;
int enof;
std::string frame_id;
ros::Publisher pcl_icp_publisher_;

clock_t start, end,a_start,a_end;
float tot_passthrough=0.0f,tot_voxel_time=0.0f,tot_outlier=0.0f,tot_normals=0.0f,tot_keypoints=0.0f,tot_descriptor=0.0f,tot_sac=0.0f,tot_icp=0.0f,tot_time_taken=0.0f;

float passthrough=0.0f,voxel_time=0.0f,outlier=0.0f,normals=0.0f,keypoints=0.0f,descriptor=0.0f,sac=0.0f,icp_time=0.0f,time_taken=0.0f,iter_time=0.0f;

// This is a tutorial so we can afford having global variables
//our visualizer
pcl::visualization::PCLVisualizer *p;
//its left and right viewports
int vp_1, vp_2,vp_3,vp_4;
int dist_count;
/*******************************GLOBAL VARIABLES**************************************************/
int frame_ctr=0;
int voxel_no_of_divisions=50.0;


ofstream recording;
pcl::StopWatch timer_iter;
pcl::StopWatch timer_component;
pcl::StopWatch timer_total;


PointCloud::Ptr src (new PointCloud);
PointCloud::Ptr tgt (new PointCloud);

//pcl::PointXYZNormals::Ptr src_key_point_normals (new PointCloud);
//pcl::PointXYZNormals::Ptr tgt_key_point_normals (new PointCloud);

//PointCloudWithNormals::Ptr src_point_normals (new PointCloud);
//PointCloudWithNormals::Ptr tgt_point_normals (new PointCloud);

PointCloudWithNormals::Ptr src_with_normals (new PointCloudWithNormals);
PointCloudWithNormals::Ptr tgt_with_normals (new PointCloudWithNormals);

PointCloud::Ptr src_filtered (new PointCloud);
PointCloud::Ptr tgt_filtered (new PointCloud);


Eigen::Matrix4f sac_transformation;
Eigen::Matrix4f refined_T;

PointCloudColorHandlerCustom<PointT> cloud_tgt_h (tgt, 0, 255, 0);
PointCloudColorHandlerCustom<PointT> cloud_src_h (src, 255, 0, 0);


pcl::PassThrough<pcl::PointXYZ> passthrough_filter;

//pcl::UniformSampling<PointT> uniform_sampling;

 pcl::VoxelGrid<pcl::PointXYZ> voxel_filter;
 PointCloud::Ptr src_voxel_filter (new PointCloud);
 PointCloud::Ptr tgt_voxel_filter (new PointCloud);
 float voxel_side_length=0.00f;
 float voxel_side_length_min=0.00f;
 float voxel_side_length_max=0.00f;
 float voxel_change_factor=0.25f;
 float *dist_map;
 int perform_icp=1;
 float voxel_x=0.1f;
 float voxel_y=0.1f;
float  voxel_z=0.1f;
 float voxel_size_x=0.04f;
  float voxel_size_y=0.04f;
  float voxel_size_z=0.04f;

 pcl::StatisticalOutlierRemoval<pcl::PointXYZ> sor;
PointCloud::Ptr src_stat_removal (new PointCloud);
PointCloud::Ptr tgt_stat_removal (new PointCloud);
int statistical_outlier_meank=50;
double statistical_outlier_stddev=1.0;


pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> normal_Estimation;
double normals_radius=0.10;


PointCloud::Ptr src_key (new PointCloud);
PointCloud::Ptr tgt_key (new PointCloud);

pcl::PointCloud<pcl::Normal>::Ptr src_key_normals(new pcl::PointCloud<pcl::Normal>);
pcl::PointCloud<pcl::Normal>::Ptr tgt_key_normals(new pcl::PointCloud<pcl::Normal>);

pcl::PointCloud<pcl::Normal>::Ptr src_normals(new pcl::PointCloud<pcl::Normal>);
pcl::PointCloud<pcl::Normal>::Ptr tgt_normals(new pcl::PointCloud<pcl::Normal>);


pcl::PointCloud<pcl::FPFHSignature33>::Ptr src_key_descriptors(new pcl::PointCloud<pcl::FPFHSignature33>());
pcl::PointCloud<pcl::FPFHSignature33>::Ptr tgt_key_descriptors(new pcl::PointCloud<pcl::FPFHSignature33>());

pcl::PointCloud<pcl::FPFHSignature33>::Ptr src_descriptors(new pcl::PointCloud<pcl::FPFHSignature33>());
pcl::PointCloud<pcl::FPFHSignature33>::Ptr tgt_descriptors(new pcl::PointCloud<pcl::FPFHSignature33>());


double icp_max_corresp_dist=100.0f;
double icp_ransac_outlier_rejection=10.0f;
double icp_transformation_epsilon=1e-8;
int    icp_set_max_iter=1000000;
double icp_euclidean_fitness_epsilon=0.01f;
int    ransac_iter=50;
//typedef pcl::registration::TransformationEstimationPointToPlaneLLS<PointNormalT,PointNormalT> PointToPlane;
//typedef pcl::registration::TransformationEstimationPointToPlaneLLS<PointT,PointT> PointToPlane;


/*******************************END OF GLOBAL VARIABLES***********************************************************/
namespace pcl
{
  template<>
    struct SIFTKeypointFieldSelector<PointXYZ>
    {
      inline float
      operator () (const PointXYZ &p) const
      {
    return p.z;
      }
    };
}



namespace pcl
{
  /** \brief @b UniformSampling assembles a local 3D grid over a given PointCloud, and downsamples + filters the data.
    *
    * Theis class creates a *3D voxel grid* (think about a voxel
    * grid as a set of tiny 3D boxes in space) over the input point cloud data.
    * Then, in each *voxel* (i.e., 3D box), all the points present will be
    * approximated (i.e., *downsampled*) with their centroid. This approach is
    * a bit slower than approximating them with the center of the voxel, but it
    * represents the underlying surface more accurately.
    *
    * \authorAshu
    * \ingroup keypoints
    */
  template <typename PointT>
  class ICP_Voxel_Grid: public Filter<PointT>
  {
    typedef typename Filter<PointT>::PointCloud PointCloud;

    using Filter<PointT>::filter_name_;
    using Filter<PointT>::input_;
    using Filter<PointT>::indices_;
    using Filter<PointT>::getClassName;

    public:
      typedef boost::shared_ptr<ICP_Voxel_Grid<PointT> > Ptr;
      typedef boost::shared_ptr<const ICP_Voxel_Grid<PointT> > ConstPtr;

      /** \brief Empty constructor. */
      ICP_Voxel_Grid() :
        leaves_ (),
        leaf_size_ (Eigen::Vector4f::Zero ()),
        inverse_leaf_size_ (Eigen::Vector4f::Zero ()),
        min_b_ (Eigen::Vector4i::Zero ()),
        max_b_ (Eigen::Vector4i::Zero ()),
        div_b_ (Eigen::Vector4i::Zero ()),
        divb_mul_ (Eigen::Vector4i::Zero ())
       // search_radius_ (0)
      {
        filter_name_ = "ICP_Voxel_Grid";
      }

      /** \brief Destructor. */
      virtual ~ICP_Voxel_Grid ()
      {
        leaves_.clear();
      }

      /** \brief Set the 3D grid leaf size.
        * \param radius the 3D grid leaf size
        */
      virtual inline void
      setLeafSize (float lx,float ly,float lz)
      {
        leaf_size_[0] =lx, leaf_size_[1] = ly ,leaf_size_[2] = lz;
        // Avoid division errors
        if (leaf_size_[3] == 0)
          leaf_size_[3] = 1;
        // Use multiplications instead of divisions
        inverse_leaf_size_ = Eigen::Array4f::Ones () / leaf_size_.array ();
        //search_radius_ = radius;
      }

    //protected:
      /** \brief Simple structure to hold an nD centroid and the number of points in a leaf. */
      struct Leaf
      {
        Leaf () : idx (-1) { }
        int idx;
      };

      /** \brief The 3D grid leaves. */
      boost::unordered_map<size_t, Leaf> leaves_;

      /** \brief The size of a leaf. */
      Eigen::Vector4f leaf_size_;

      /** \brief Internal leaf sizes stored as 1/leaf_size_ for efficiency reasons. */
      Eigen::Array4f inverse_leaf_size_;

      /** \brief The minimum and maximum bin coordinates, the number of divisions, and the division multiplier. */
      Eigen::Vector4i min_b_, max_b_, div_b_, divb_mul_;

      /** \brief The nearest neighbors search radius for each point. */
      //double search_radius_;

      /** \brief Downsample a Point Cloud using a voxelized grid approach
        * \param[out] output the resultant point cloud message
        */
      void applyFilter (PointCloud &output)
      {

          // Has the input dataset been set already?
          if (!input_)
          {
            PCL_WARN ("[pcl::%s::detectKeypoints] No input dataset given!\n", getClassName ().c_str ());
            output.width = output.height = 0;
            output.points.clear ();
            return;
          }

          output.height       = 1;                    // downsampling breaks the organized structure
          output.is_dense     = true;                 // we filter out invalid points

          Eigen::Vector4f min_p, max_p;
          // Get the minimum and maximum dimensions
          pcl::getMinMax3D<PointT>(*input_, min_p, max_p);
//           min_p[0]=-99.99f;
//           min_p[1]=-99.99f;
//           min_p[2]=-99.99f;

//           max_p[0]=99.99f;
//           max_p[1]=99.99f;
//           max_p[2]=99.99f;


          // Compute the minimum and maximum bounding box values
          min_b_[0] = static_cast<int> (floor (min_p[0] * inverse_leaf_size_[0]));
          max_b_[0] = static_cast<int> (floor (max_p[0] * inverse_leaf_size_[0]));
          min_b_[1] = static_cast<int> (floor (min_p[1] * inverse_leaf_size_[1]));
          max_b_[1] = static_cast<int> (floor (max_p[1] * inverse_leaf_size_[1]));
          min_b_[2] = static_cast<int> (floor (min_p[2] * inverse_leaf_size_[2]));
          max_b_[2] = static_cast<int> (floor (max_p[2] * inverse_leaf_size_[2]));

          // Compute the number of divisions needed along all axis
          div_b_ = max_b_ - min_b_ + Eigen::Vector4i::Ones ();
          div_b_[3] = 0;

          // Clear the leaves
          leaves_.clear ();

          // Set up the division multiplier
          divb_mul_ = Eigen::Vector4i (1, div_b_[0], div_b_[0] * div_b_[1], 0);


          // First pass: build a set of leaves with the point index closest to the leaf center
           PCL_INFO("\nFilter process1: %d\n",indices_->size());
          for (size_t cp = 0; cp < indices_->size (); ++cp)
          {
            if (!input_->is_dense)
              // Check if the point is invalid
              if (!pcl_isfinite (input_->points[(*indices_)[cp]].x) ||
                  !pcl_isfinite (input_->points[(*indices_)[cp]].y) ||
                  !pcl_isfinite (input_->points[(*indices_)[cp]].z))
              continue;

            Eigen::Vector4i ijk = Eigen::Vector4i::Zero ();
            ijk[0] = static_cast<int> (floor (input_->points[(*indices_)[cp]].x * inverse_leaf_size_[0]));
            ijk[1] = static_cast<int> (floor (input_->points[(*indices_)[cp]].y * inverse_leaf_size_[1]));
            ijk[2] = static_cast<int> (floor (input_->points[(*indices_)[cp]].z * inverse_leaf_size_[2]));

            // Compute the leaf index
            int idx = (ijk - min_b_).dot (divb_mul_);

            Leaf& leaf = leaves_[idx];
            // First time we initialize the index
            if (leaf.idx == -1)
            {
              //PCL_INFO("\nGot ngative idx");
               leaf.idx = (*indices_)[cp];
               continue;
            }

            // Check to see if this point is closer to the leaf center than the previous one we saved
            float diff_cur   = (input_->points[(*indices_)[cp]].getVector4fMap () - ijk.cast<float> ()).squaredNorm ();
            float diff_prev  = (input_->points[leaf.idx].getVector4fMap ()        - ijk.cast<float> ()).squaredNorm ();

            // If current point is closer, copy its index instead
            if (diff_cur < diff_prev)
              leaf.idx = (*indices_)[cp];
          }
     PCL_INFO("\nFilter process2: %d\n",leaves_.size());
          // Second pass: go over all leaves and copy data
          output.points.resize (leaves_.size ());
          int cp = 0;

          for (typename boost::unordered_map<size_t, Leaf>::const_iterator it = leaves_.begin (); it != leaves_.end (); ++it)
            output.points[cp++] = input_->points[it->second.idx];
          output.width = static_cast<uint32_t> (output.points.size ());
      }
  };
}


//convenient structure to handle our pointclouds
struct PCD
{
    PointCloud::Ptr cloud;
    std::string f_name;

    PCD() : cloud (new PointCloud) {};
};

struct PCDComparator
{
    bool operator () (const PCD& p1, const PCD& p2)
    {
        return (p1.f_name < p2.f_name);
    }
};


// Define a new point representation for < x, y, z, curvature >
class MyPointRepresentation : public pcl::PointRepresentation <PointNormalT>
{
    using pcl::PointRepresentation<PointNormalT>::nr_dimensions_;
public:
    MyPointRepresentation ()
    {
        // Define the number of dimensions
        nr_dimensions_ = 4;
    }

    // Override the copyToFloatArray method to define our feature vector
    virtual void copyToFloatArray (const PointNormalT &p, float * out) const
    {
        // < x, y, z, curvature >
        out[0] = p.x;
        out[1] = p.y;
        out[2] = p.z;
        out[3] = p.curvature;
    }
};


////////////////////////////////////////////////////////////////////////////////
/** \brief Display source and target on the first viewport of the visualizer
 *
 */
void showCloudsLeft(const PointCloud::Ptr cloud_target, const PointCloud::Ptr cloud_source)
{
    p->removePointCloud ("vp1_target");
    p->removePointCloud ("vp1_source");

    PointCloudColorHandlerCustom<PointT> tgt_h (cloud_target, 0, 255, 0);
    PointCloudColorHandlerCustom<PointT> src_h (cloud_source, 255, 0, 0);
    p->addPointCloud (cloud_target, tgt_h, "vp1_target", vp_1);
    p->addPointCloud (cloud_source, src_h, "vp1_source", vp_1);

    PCL_INFO ("Press q to begin the registration.\n");
    // p-> spin();
}


////////////////////////////////////////////////////////////////////////////////
/** \brief Display source and target on the second viewport of the visualizer
 *
 */
void showCloudsRight(const PointCloudWithNormals::Ptr cloud_target, const PointCloudWithNormals::Ptr cloud_source)
{
    p->removePointCloud ("source");
    p->removePointCloud ("target");


    PointCloudColorHandlerGenericField<PointNormalT> tgt_color_handler (cloud_target, "curvature");
    if (!tgt_color_handler.isCapable ())
        PCL_WARN ("Cannot create curvature color handler!");

    PointCloudColorHandlerGenericField<PointNormalT> src_color_handler (cloud_source, "curvature");
    if (!src_color_handler.isCapable ())
        PCL_WARN ("Cannot create curvature color handler!");


    p->addPointCloud (cloud_target, tgt_color_handler, "target", vp_2);
    p->addPointCloud (cloud_source, src_color_handler, "source", vp_2);

    p->spinOnce();
}

////////////////////////////////////////////////////////////////////////////////
/** \brief Load a set of PCD files that we want to register together
  * \param argc the number of arguments (pass from main ())
  * \param argv the actual command line arguments (pass from main ())
  * \param models the resultant vector of point cloud datasets
  */
void loadData (int argc, char **argv, std::vector<PCD, Eigen::aligned_allocator<PCD> > &models)
{
    std::string extension (".pcd");


    // Suppose the first argument is the actual test model
    for (int i = snof;i<=enof ; i++)
    {
        frame_id=std::to_string(i);
        frame_id+=".pcd";

        /*   std::string fname = std::string (argv[i]);
    // Needs to be at least 5: .plot
    if (fname.size () <= extension.size ())
      continue;
    std::transform (fname.begin (), fname.end (), fname.begin (), (int(*)(int))tolower);
   */
        //check that the argument is a pcd file
        // if (fname.compare (fname.size () - extension.size (), extension.size (), extension) == 0)
        {
            // Load the cloud and saves it into the global list of models
            PCD m;
            m.f_name = frame_id;
            pcl::io::loadPCDFile (frame_id, *m.cloud);
            //remove NAN points from the cloud
            std::vector<int> indices;
            pcl::removeNaNFromPointCloud(*m.cloud,*m.cloud, indices);

            models.push_back (m);
        }
    }
}



 void pairAlign (const PointCloud::Ptr cloud_src, const PointCloud::Ptr cloud_tgt, PointCloud::Ptr output, Eigen::Matrix4f &final_transform,  Eigen::Matrix4f &initial_guess)
 {



     frame_ctr++;



   /*                     cloud_src    cloud_tgt
    *                         |           |
    *                      ___|___________|____
   ************************PASSTHROUGH FILTER*****************************
                                                                        */

     timer_component.reset();

    passthrough_filter.setFilterFieldName ("z");
    //passthrough_filter.setFilterLimits (0.0,500.0);

   if(frame_ctr==1)
     {
     passthrough_filter.setInputCloud (cloud_tgt);
     passthrough_filter.filter (*tgt_filtered);
     }
    else
     {
         pcl::copyPointCloud(*src_filtered, *tgt_filtered);
     }


    passthrough_filter.setInputCloud (cloud_src);
    passthrough_filter.filter (*src_filtered);
    passthrough=(float)timer_component.getTimeSeconds();
    ROS_INFO("STEP 2: Pass through filter completed, time_taken=%f secs SOURCE_SIZE=%d (%f %%) \tTARGET_SIZE=%d (%f %%)\n",passthrough,
              (int)src_filtered->size(),((float)src_filtered->size()/(float)pointcloudoriginalsize)*100.0f,(int)tgt_filtered->size(),((float)tgt_filtered->size()/(float)pointcloudoriginalsize)*100.0f);


 /*
  **********************END OF PASSTHROUGH FILTER************************************
                          |                |
                     src_filtered        tgt_filtered
                          |                |
                          |                |
                  ________|________________|___________
  **************************VOXEL GRID FILTERING ******************************************
                                                                                     */

    timer_component.reset();
    /*
     *VoxelGrid filter is created with a leaf size of 2cm, the input data is passed, and the output is computed and stored in cloud_filtered.
     * */

   // voxel_side_length = 0.02f; //2cm
     pcl::VoxelGrid<pcl::PointXYZ> voxel_filter;
    Eigen::Vector4f voxel_leaf_size (voxel_size_x,
                                     voxel_size_y,
                                     voxel_size_z,
                                     0.0f);

     voxel_filter.setLeafSize (voxel_leaf_size);
    // voxel_filter.setMinimumPointsNumberPerVoxel(1);
     if(frame_ctr==1)
     {
     voxel_filter.setInputCloud(cloud_tgt);
     voxel_filter.filter (*tgt_voxel_filter);
     }
     else
     {
             pcl::copyPointCloud<PointT>(*src_voxel_filter, *tgt_voxel_filter);
     }

     voxel_filter.setInputCloud(cloud_src);
     voxel_filter.filter (*src_voxel_filter);



     voxel_time=(float)timer_component.getTimeSeconds();
   ROS_INFO("STEP 3: Voxel grid  completed time_taken=%f secs \n \t \t \t \t \tAVG_DISTANCE=%f \n \t \t \t \t \tvoxel_x=%f  voxel_y=%f voxel_z=%f \n \t \t \t \t \tsrc_size=%d (%f %%) tgt_size=%d (%f %%)\n",
            voxel_time,avg_distance, voxel_size_x,voxel_size_y,voxel_size_z,
            (int)src_voxel_filter->size(),((float)src_voxel_filter->size()/(float)pointcloudoriginalsize)*100.0f,(int)tgt_voxel_filter->size(),((float)tgt_voxel_filter->size()/(float)pointcloudoriginalsize)*100.0f );

   //std::cout<<endl<<voxel_filter.getNrDivisions()<<endl;
   //std::cout<<endl<<voxel_filter.getDivisionMultiplier()<<endl;

 /***********************END OF VOXEL GRID FILTER***********************
                           |                  |
                     src_voxel_filter    tgt_voxel_filter
                           |                  |
                           |                  |
     *****************STATISTICAL OUTLIER REMOVAL*************************************/
//   timer_component.reset();
//    /****
//     * number of neighbors to analyze for each point is set to 50, and the standard deviation multiplier to 1.
//     * all points who have a distance larger than 1 standard deviation of the mean distance to the query point will be marked as outliers and removed
//     *  The output is computed and stored in tgt and src.
//     * */


//     sor.setMeanK (statistical_outlier_meank);
//     sor.setStddevMulThresh (statistical_outlier_stddev);

//     if(frame_ctr==1)
//     {
//     sor.setInputCloud(tgt_voxel_filter);
//     sor.filter (*tgt);
//     }
//     else
//     {
//             pcl::copyPointCloud<PointT>(*src, *tgt);
//     }

//     sor.setInputCloud(src_voxel_filter);
//     sor.filter (*src);



//     end = clock();
//     outlier=timer_component.getTimeSeconds();

//     PCL_INFO("\nStatistical Outlier Removal completed, meank=%d stddev=%lf time_taken=%f src_size=%d tgt_size=%d\nsecs \n",statistical_outlier_meank,statistical_outlier_stddev,outlier,src->size(),tgt->size());

 /***************END OF STATISTICAL OUTLIER REMOVAL************************************/


 /*********************************ESTIMATING NORMALS***************************************/
    // start = clock();

    timer_component.reset();


    pcl::search::KdTree<pcl::PointXYZ>::Ptr src_kdtree(new pcl::search::KdTree<pcl::PointXYZ>);
    pcl::search::KdTree<pcl::PointXYZ>::Ptr tgt_kdtree(new pcl::search::KdTree<pcl::PointXYZ>);


  // Use all neighbors in a sphere of radius 10cm
  normal_Estimation.setRadiusSearch(normals_radius);

  if(frame_ctr==1)
  {
  normal_Estimation.setSearchMethod(tgt_kdtree);
  normal_Estimation.setSearchSurface(tgt_voxel_filter);
  normal_Estimation.setInputCloud(tgt_voxel_filter);
  normal_Estimation.compute(*tgt_normals);

 // normal_Estimation.setSearchMethod(tgt_key_kdtree);
 // normal_Estimation.setSearchSurface(tgt_key);
 // normal_Estimation.setInputCloud(tgt_key);
 // normal_Estimation.compute(*tgt_key_normals);


  }
  else
  {

     pcl::copyPointCloud<pcl::Normal>(*src_normals, *tgt_normals);
     //pcl::copyPointCloud<pcl::Normal>(*src_key_normals, *tgt_key_normals);
  }



  normal_Estimation.setSearchMethod(src_kdtree);
  normal_Estimation.setSearchSurface(src_voxel_filter);
  normal_Estimation.setInputCloud(src_voxel_filter);
  normal_Estimation.compute(*src_normals);


  end = clock();
  normals=(float)timer_component.getTimeSeconds();

  ROS_INFO("STEP 4: Normal Estimation completed: time_taken=%f secs radius=%f \n",normals,normals_radius);

 /********************END OF ESTIMATING NORMALS********************************/





 /*********************************COPY POINT CLOUD WITH NORMALS***************************************************************************/

      pcl::copyPointCloud<PointT, PointNormalT> (*src_voxel_filter,*src_with_normals);
      pcl::copyPointCloud<pcl::Normal, PointNormalT> (*src_normals, *src_with_normals);
      pcl::copyPointCloud<PointT, PointNormalT> (*tgt_voxel_filter,*tgt_with_normals);
      pcl::copyPointCloud<pcl::Normal, PointNormalT> (*tgt_normals, *tgt_with_normals);

     // PCL_INFO("\nsrc_normals=%d tgt_nprmals=%d\n",src_with_normals->size(),tgt_with_normals->size());


 /*********************************END OF COPY POINT CLOUD WITH NORMALS***************************************************************************/


 /**********************PERFORM ICP******************************************************************************************************************/

      timer_component.reset();

     // PCL_INFO("\nICP started\n");
      //pcl::IterativeClosestPointWithNormals<PointNormalT, PointNormalT> icp;

       //PointCloudWithNormals registration_output;
          pcl::PointCloud<pcl::PointXYZ>::Ptr registration_output ( new pcl::PointCloud<pcl::PointXYZ> () );
       pcl::IterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ>::Ptr icp ( new pcl::IterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> () );
       pcl::registration::TransformationEstimationSVD<pcl::PointXYZ, pcl::PointXYZ>::Ptr trans_svd (new pcl::registration::TransformationEstimationSVD<pcl::PointXYZ, pcl::PointXYZ>);
      
      //Set the maximum distance threshold between two correspondent points in source <-> target
      
      //icp.setMaxCorrespondenceDistance (icp_max_corresp_dist);

      //The method considers a point to be an inlier, if the distance between the target data index and the transformed source index is smaller than the given inlier distance threshold.
      //The value is set by default to 0.05m. In this example 10m.
      
      //icp.setRANSACOutlierRejectionThreshold (icp_ransac_outlier_rejection);


      //Set the transformation epsilon (maximum allowable difference between two consecutive transformations) in order
      //for an optimization to be considered as having converged to the final solution.
      
      //icp.setTransformationEpsilon (icp_transformation_epsilon);



      //icp.setMaximumIterations (icp_set_max_iter);

      icp->setMaximumIterations (100);
      icp->setTransformationEstimation (trans_svd);
      //Set the maximum allowed Euclidean error between two consecutive steps in the ICP loop, before the algorithm is considered to have converged
      icp->setEuclideanFitnessEpsilon(icp_euclidean_fitness_epsilon);
      //icp.setRANSACIterations(ransac_iter);

    //icp.setTransformationEstimation(trans_lls);

      //PCL_INFO("\nICP started 2\n");
      icp->setInputSource(src_voxel_filter); // from (1)
      icp->setInputTarget(tgt_voxel_filter);



     // std::cout<<endl<<initial_guess<<endl;

      //Perform ICP with alignment with initial_guess
      icp->align (*registration_output,initial_guess);

      //PCL_INFO("\n ICPc=%d ",registration_output.size());

       final_transform=icp->getFinalTransformation();


        pcl::copyPointCloud(*registration_output,*output);

     //end = clock();
     //tot_icp+=((double)(end-start)/CLOCKS_PER_SEC);

     icp_time=(float)timer_component.getTimeSeconds();

     ROS_INFO("STEP 5: ICP completed, time_taken=%f secs\n",icp_time);


 /**********************END OF PERFORM ICP****************************************************************************************************************/

}



void process_icp(const PointCloud::Ptr cloud_src, const PointCloud::Ptr cloud_tgt)
{
    file_no++;
    std::vector<int> indices;
    PointCloud::Ptr source_no_nan (new PointCloud);
    PointCloud::Ptr target_no_nan(new PointCloud);
    PointCloud::Ptr aligned_cloud(new PointCloud);
    PointCloud::Ptr result(new PointCloud);
    pointcloudoriginalsize=(int)cloud_src->size();
    PCL_INFO("\n****************************************************************************************************************************************\n");
    ROS_INFO("Original Point Cloud Size: SOURCE=%d TARGET=%d\n",(int)cloud_src->size(),(int)cloud_tgt->size());
   // PCL_INFO("\n Orginal before NAN SRC_W=%d SRC_H=%d TGT_w=%d TGT_H=%d \n",cloud_src->width,cloud_src->height,cloud_tgt->width,cloud_tgt->height);
    pcl::removeNaNFromPointCloud(*cloud_src,*source_no_nan, indices);
    pcl::removeNaNFromPointCloud(*cloud_tgt,*target_no_nan, indices);
    ROS_INFO("STEP 1: Point Cloud Size after NaN Removal: SOURCE=%d (%f %%) \t TARGET=%d (%f %%) \n",(int)source_no_nan->size(), (float)((float)(source_no_nan->size())/(float)cloud_src->size())*100.0f,
             (int)target_no_nan->size(),(float)(((float)target_no_nan->size())/(float)cloud_tgt->size())*100.0f);




    /***************************************************Steps************************************************/

    //prev_transform=Eigen::Matrix4f::Identity();
   //  pcl::transformPointCloud (*source, *source_transformed,prev_transform);



           /*******************Pairwise Alignment*****************************
              function_name= pairAlign();

              Declaration:
              void pairAlign (const PointCloud::Ptr cloud_src, const PointCloud::Ptr cloud_tgt, PointCloud::Ptr output, Eigen::Matrix4f &final_transform,Eigen::Matrix4f &initial_guess)

              Parameters:

              source= Input source cloud
              Target= Target cloud
              aligned_cloud= resultant aligned cloud
              pairTransform = resultant pairwise transformation matrix
              prev_transform= Used as initial Guess for the ICP method. See  icp.align(registration_output,initial_guess) in the function pairAlign

             BLOCK DIAGRAM
             ~~~~~~~~~~~~~~
                                                                                                                       -------<--------
                                                                                                                      |                |
                                       _______________________________________________________________________________|_________       |
            Pn.......P2(Source)       |                                                                               |         |      |pair transformation(T1-----Tn)
               ---------------------->|                                                                               |         |--------------------------------->|
                                      |  pairAlign (source_no_nan, target_no_nan,aligned_cloud, pairTransform,prev_transform);  |                                  | GT=I at 1st iteration
                                      |                                                                                         |Aligned Cloud(P21....Pn n-1)      |
               ---------------------->|                                                                                         |----------------------------------| GT=GT*pairT, source---GT--->result
            Pn-1.....P1(Target)       |_________________________________________________________________________________________|                                  |-------------------------------------> save result






      ***********************************************************************************************************/



    timer_iter.reset();


    pairAlign (source_no_nan,target_no_nan,aligned_cloud, pairTransform,prev_transform);

    ROS_INFO("Pairwise alignment completed, time_taken=%f secs", timer_iter.getTimeSeconds());


    PCL_INFO("\n****************************************************************************************************************************************\n");


    prev_transform=pairTransform;


    ss=std::to_string(file_no);
    ss+="_icp_transformation.txt";

//    ofstream myfile;
//    myfile.open (ss);
//    myfile << pairTransform;
//    myfile.close();


     //update the Global transform
     Global_transform = Global_transform * pairTransform;



  //   int ran1=rand()%256;
   //  int ran2=rand()%256;
    // int ran3=rand()%256;

     //transform source cloud into the global transform
      pcl::transformPointCloud (*source, *result, Global_transform);
    //

      /***************Reduce the number of point clouds so that RVIZ has less data to display and visualization is faster
                     You can skip this section  by commenting it out and changing
                        pcl::toROSMsg(*out, pcl2_msg_);  to pcl::toROSMsg(*source, pcl2_msg_);



****************************************************/
      //*all_clo*result; 
       PointCloud::Ptr cloud_filtered (new PointCloud);
      pcl::VoxelGrid<pcl::PointXYZ> voxel_filter;
      float voxel_side_length = 0.04f; //2cm
      Eigen::Vector4f voxel_leaf_size (voxel_side_length,
                                       voxel_side_length,
                                       voxel_side_length,
                                       0.0f);

      voxel_filter.setLeafSize (voxel_leaf_size);


     //voxel filter input cloud
      voxel_filter.setInputCloud(result);


      voxel_filter.filter (*cloud_filtered);

      PointCloud::Ptr out (new PointCloud);
      pcl::StatisticalOutlierRemoval<pcl::PointXYZ> sor;
      sor.setMeanK (50);
      sor.setStddevMulThresh (1.0);

      sor.setInputCloud(cloud_filtered);
      sor.filter (*out);
      //*all_cloud=*all_cloud+**///out;


      /**********************************************************************/



      /****************************Send Point Clouds along with TF********************/
      ros::Time time_stamp = ros::Time::now();

      tf::Transform transform;
      Eigen::Matrix4d global_transformation_(Global_transform.cast<double>());

      Eigen::Affine3d affine(global_transformation_);

      tf::transformEigenToTF(affine, transform);
      static tf::TransformBroadcaster br;

      /*************************************************************************************
      StampedTransform (const tf::Transform &input, const ros::Time &timestamp, const std::string &frame_id, const std::string &child_frame_id)
      Public Attributes

       ros::Time  stamp_
       The timestamp associated with this transform.


       std::string  frame_id_
       The frame_id of the coordinate frame in which this transform is defined.


       std::string  child_frame_id_
       The frame_id of the coordinate frame this transform defines.

      *************************************************************************************/
     br.sendTransform(tf::StampedTransform(transform,time_stamp, "lidar_link",  "GT"));



      sensor_msgs::PointCloud2 pcl2_msg_;
      pcl::toROSMsg(*out, pcl2_msg_);

      pcl2_msg_.header.seq = file_no;
      pcl2_msg_.header.frame_id = "GT";
      pcl2_msg_.header.stamp=time_stamp;

     pcl_icp_publisher_.publish(pcl2_msg_);

      /****************************END OF Send Point Clouds along with tf********************/



}

void get_point_clouds(const sensor_msgs::PointCloud2ConstPtr& msg)
{
    std::cout<<"\nGet point clouds!!";
    frame_count++;  //keep track of count of incoming frames
    pcl::PCLPointCloud2 cloud_original;
    ss=std::to_string(frame_count);
    ss+=".ply";
    pcl_conversions::toPCL(*msg,cloud_original);

   if(frame_count==1)
    {

        pcl::fromPCLPointCloud2(cloud_original,*target);
     // ply_writer.write(ss,*target);

    }
   else if(frame_count==2)
    {
       pcl::fromPCLPointCloud2(cloud_original,*source);
    // pcl::PLYWriter::write(ss,*source);
        ply_writer.write(ss,*source);
       if(perform_icp==1)
          process_icp(source,target);
    }
   else
    {
       pcl::copyPointCloud(*source,*target);
       pcl::fromPCLPointCloud2(cloud_original,*source);
     //ply_writer.write(ss,*source);
       if(perform_icp==1)
          process_icp(source,target);
    }

/****************SAVING TO PCD FILES FOR EVALUATION**********************************/
//  ss=std::to_string(file_id);
//     ss+=".pcd";

// if (pcl::io::savePCDFile<pcl::PointXYZ>(ss,result) != 0)
//    {
//    printf("error reading file");
//    }

//    printf ("\t(%f, %f, %f)\n", pt.x, pt.y, pt.z);

/****************END OF SAVING TO PCD FILES FOR EVALUATION**********************************/





}
void get_distance(const std_msgs::Float32::ConstPtr& msg)
{


    //TODO:change the factor to a non linear function

    /*****Change in voxel size to give approximately fixed number of points*************/
//      voxel_size_x=0.04;          // (2*depth*tan(HV/2)/Number of segments you want)*scaling_factor_for_x_component_of_voxel
//     voxel_size_y=0.04;          // (2*depth*tan(VV/2)/Number of segments you want)*scaling_factor_for_y_component_of_voxel
//     voxel_size_z=0.04;
     if(msg->data==0.000000)
     {
         printf("\nNo data");
         voxel_size_x=0.04f;          // (2*depth*tan(HV/2)/Number of segments you want)*scaling_factor_for_x_component_of_voxel
         voxel_size_y=0.04f;          // (2*depth*tan(VV/2)/Number of segments you want)*scaling_factor_for_y_component_of_voxel
         voxel_size_z=0.04f;
     }
     else
     {


     voxel_size_x=((2.0f*msg->data*0.60086061902f)/(float)voxel_no_of_divisions*1.45f)*1.0f/1.0f;          // (2*depth*tan(HV/2)/Number of segments you want)*scaling_factor_for_x_component_of_voxel
     voxel_size_y=((2.0f*msg->data*0.41421356237f)/(float)voxel_no_of_divisions)*1.0f/1.0f;          // (2*depth*tan(VV/2)/Number of segments you want)*scaling_factor_for_y_component_of_voxel
     voxel_size_z=10*msg->data;
     avg_distance=msg->data;
     }
    /**********************************************************************/




     //printf("\nAVG_DIST=%f voxel_size=%f voxel_x=%f voxel_y=%f ",msg->data,voxel_side_length,voxel_size_x,voxel_size_y);


}

int main(int argc, char** argv)
{

    double dur_secs=5.0;
   ros::init(argc, argv, "sub_pcl");

   ros::param::get("perform_icp",perform_icp);
   //printf("")
   ros::param::get("sleep",dur_secs);
  // ros::param::get("voxel_side_length_min",voxel_side_length_min);
  // ros::param::get("voxel_side_length_max",voxel_side_length_max);
   ros::param::get("voxel_no_of_divisions",voxel_no_of_divisions);
   ros::param::get("voxel_change_factor",voxel_change_factor);
   ros::param::get("statistical_outlier_meank",statistical_outlier_meank);
   ros::param::get("statistical_outlier_stddev",statistical_outlier_stddev);
   ros::param::get("normals_radius",normals_radius);
   ros::param::get("icp_max_corresp_dist",icp_max_corresp_dist);
   ros::param::get("ransac_iter",ransac_iter);
   ros::param::get("icp_ransac_outlier_rejection",icp_ransac_outlier_rejection);
   ros::param::get("icp_transformation_epsilon",icp_transformation_epsilon);
   ros::param::get("icp_set_max_iter",icp_set_max_iter);
   ros::param::get("icp_euclidean_fitness_epsilon",icp_euclidean_fitness_epsilon);






  ros::NodeHandle nh;
  ros::NodeHandle pcl_distance_subs;
  ros::NodeHandle pcl_icp_node;


  ros::Duration(dur_secs).sleep();
  //std::cout<<"\nHi I am up\n";
   ROS_INFO("HI ICP HAS STARTED\n");

   ros::Subscriber sub = nh.subscribe<sensor_msgs::PointCloud2>("/velodyne_points", 10000, get_point_clouds);
   ros::Subscriber distance_subs=pcl_distance_subs.subscribe("/cam0/picoflexx_depth", 10000, get_distance);

 // ros::Subscriber sub_depth = pcl_depth_image.subscribe<sensor_msgs::PointCloud2>("/cam0/picoflexx_depth", 1, get_depth);


  pcl_icp_publisher_ = pcl_icp_node.advertise<sensor_msgs::PointCloud2>("/lidar/pcl/icp", 1000000);
  ros::spin();
}
