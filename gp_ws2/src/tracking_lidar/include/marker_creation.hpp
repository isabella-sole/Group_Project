#include "opencv2/opencv.hpp"
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/video/tracking.hpp>
#include <ros/ros.h>
#include <cmath>
#include <iomanip>
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <sensor_msgs/image_encodings.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/CameraInfo.h>
#include <camera_info_manager/camera_info_manager.h>
#include <pcl/features/normal_3d.h>
#include <iostream>
#include <pcl/ModelCoefficients.h>
#include <pcl/filters/project_inliers.h>
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <visualization_msgs/MarkerArray.h>
#include <pcl/features/moment_of_inertia_estimation.h>
#include <tf/transform_broadcaster.h>
#include <string>
#include <pcl_ros/point_cloud.h>
#include <boost/foreach.hpp>
#include <pcl_conversions/pcl_conversions.h>
#include <velodyne_pointcloud/point_types.h>
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
#include <Eigen/Eigen>
#include <Eigen/Core>
#include <Eigen/Dense>
#include <Eigen/QR>
#include <rml/RML.h>
#include <Eigen/Geometry>
#include <exception>
#include <pcl/filters/filter.h>
#include <pcl/exceptions.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/extract_clusters.h>
#include <algorithm>
#include <iterator>
#include <fstream>
#include <pcl/filters/radius_outlier_removal.h>

#include "udp_server/status_timestamped.h"
#include "tracking_lidar/Statusdim.h"
#include "trackdim.h"


using namespace Eigen;
using namespace message_filters;
using namespace pcl;



// using namespace std;

class MarkerCreation {


public:

    visualization_msgs::Marker marker;
    //visualization_msgs::MarkerArray bbox_markers;


    MarkerCreation();

    visualization_msgs::MarkerArray creation(Eigen::Quaternionf q, int i, std::vector<float> array_pos, std::vector<double> array_scale, visualization_msgs::MarkerArray bbox_markers);

    //~MarkerCreation();

};
