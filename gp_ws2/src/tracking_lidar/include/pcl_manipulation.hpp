#include <pcl/features/normal_3d.h>
#include <iostream>
#include <pcl/ModelCoefficients.h>
#include <pcl/filters/project_inliers.h>
#include <pcl/features/moment_of_inertia_estimation.h>
#include <pcl_ros/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/common/eigen.h>
#include <pcl/common/transforms.h>
#include <pcl/io/pcd_io.h>
#include <pcl/filters/passthrough.h>
#include <pcl/search/search.h>
#include <pcl/search/kdtree.h>
#include <pcl/features/normal_3d.h>
#include <pcl/segmentation/region_growing.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/filters/filter.h>
#include <pcl/exceptions.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/filters/radius_outlier_removal.h>
#include <pyramid.hpp>
#include "trackdim.h"
#include <pcl/search/brute_force.h>
#include <visualization_msgs/MarkerArray.h>
#include <init_bb_calib.hpp>
#include "udp_server/status_timestamped.h"
#include <cv_bridge/cv_bridge.h>
#include <opencv2/core/mat.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/plot.hpp>



class PclManipulation {


private:

    	Eigen::Affine3f transform_z;
      float theta;
      pcl::PassThrough <pcl::PointXYZI> pass;
      Eigen::Affine3f transform_3;
      float theta2;
      tracking_lidar::Statusdim init;
      float x_init, y_init, l_init, w_init, psi_init;
      float psi_angle;
    //  float x_est_temp, y_est_temp;







public:


    PclManipulation();


    void rotation_z(pcl::PointCloud<pcl::PointXYZI>::Ptr transformed_cloud, pcl::PointCloud<pcl::PointXYZI>::Ptr cloud);
    void filter_in_y(pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_filtered, pcl::PointCloud<pcl::PointXYZI>::Ptr transformed_cloud);
    void test_point_in_pyramids(pcl::PointCloud<pcl::PointXYZI>::Ptr transformed_cloud1, pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_filtered, pcl::PointCloud<pcl::PointXYZI>::Ptr filterd_pc, pcl::PointCloud<pcl::PointXYZI>::Ptr transformed_filter1, Pyramid *my_pyramid);
    bool extracting_features(pcl::PointCloud<pcl::PointXYZI>::Ptr new_transformed_filter, trackdim *track1, ros::Publisher *bbox3d_pub, geometry_msgs::Quaternion *TR, pcl::PointCloud<pcl::PointXYZI>::Ptr obstcal_cloud, bool track_init);
    int neares_track(visualization_msgs::MarkerArray arr);
    int track_assooc(visualization_msgs::MarkerArray arr, float x, float y);
    void print_cloud(bool track_init, trackdim *track1, bool print_flag, ros::Publisher *bbox3d_pub_KF, udp_server::status_timestamped *cat_status,ros::Publisher *Prediction, ros::Publisher *Estimation, geometry_msgs::Quaternion *TR, Eigen::Quaternionf *qinertia, pcl::PointCloud<pcl::PointXYZI>::Ptr obstcal_cloud, InitBbCalib *ibc, cv_bridge::CvImagePtr cv_ptr, bool colour, ofstream *myfile, ofstream *myfile2, ofstream *myfile3);
    tracking_lidar::Statusdim KF_prediction(trackdim *track, tracking_lidar::Statusdim pre);
    tracking_lidar::Statusdim KF_estimation(trackdim *track, tracking_lidar::Statusdim est);
    int compute_u(InitBbCalib ibc, float centx, float centy, float centz);
    int compute_v(InitBbCalib ibc, float centx, float centy, float centz);
    // void plot_position_pre(tracking_lidar::Statusdim *pre);
    // void plot_position_init(float x, float y, float l, float w);
    void plot_position(tracking_lidar::Statusdim *pre, trackdim *track);



    ~PclManipulation();





};
