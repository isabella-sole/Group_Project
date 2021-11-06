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
#include <pyramid.hpp>
#include "udp_server/status_timestamped.h"
#include "tracking_lidar/Statusdim.h"
#include "trackdim.h"
#include <time.h>
#include <unistd.h>
#include <stdio.h>
#include <limits.h>
#include <pcl/common/common_headers.h>
#include <pcl/features/normal_3d.h>
#include <pcl/io/pcd_io.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/console/parse.h>


using namespace Eigen;
using namespace message_filters;
using namespace pcl;

string CAMERA_TOPIC = "pylon_camera_node/image_raw";
string VELODYNE_TOPIC = "velodyne_points";
string CAT_STATUS_TOPIC = "catamaran_status";



ros::Subscriber lidar_sub;
ros::Subscriber cat_sub;
ros::Subscriber cam_sub;
ros::Publisher pub_filterd;
ros::Publisher pub_filterd_inY;
ros::Publisher pub_regions;
ros::Publisher bbox3d_pub;
ros::Publisher bbox3d_pub_KF;
ros::Publisher Prediction;
ros::Publisher Estimation;
ros::Publisher image;




udp_server::status_timestamped cat_status;

cv_bridge::CvImagePtr cv_ptr;

pcl::PointCloud<pcl::PointXYZI>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZI>);


pcl::PointCloud<pcl::PointXYZI>::Ptr old(new pcl::PointCloud<pcl::PointXYZI>);


pcl::PointCloud<pcl::PointXYZI>::Ptr print_old(new pcl::PointCloud<pcl::PointXYZI>);

bool avilableold=false;






struct FirstOrderFilter {
    Eigen::VectorXd prevState_;
    Eigen::VectorXd filteredData_;
    bool isRunning_, initialised_;
    bool enableAngularWrap_;
    int dataSize_;
    double alpha_;

    FirstOrderFilter();

    void Reset();

    void Init(const int dataSize, const double alpha, bool enableAngularWrap);

    Eigen::VectorXd& Filter(const Eigen::VectorXd array);

};



FirstOrderFilter::FirstOrderFilter() {
	 Reset();
 }


void FirstOrderFilter::Reset() {
	isRunning_ = false;
}


void FirstOrderFilter::Init(const int dataSize, const double alpha, bool enableAngularWrap)
{
    dataSize_ = dataSize;
    prevState_.resize(dataSize);
    filteredData_.resize(dataSize);
    alpha_ = alpha;
    enableAngularWrap_ = enableAngularWrap;
    initialised_ = true;
}


Eigen::VectorXd& FirstOrderFilter::Filter(const Eigen::VectorXd array)
{
    // If the filter is not yet running we don't have any previous state so
    // we use the current one as a pseudo-previous state.
    if (initialised_) {
        if (!isRunning_) {
            for (int i = 0; i < dataSize_; ++i) {
                prevState_(i) = array(i);
            }
            isRunning_ = true;
        }

        Eigen::VectorXd input = array;

        for (int i = 0; i < dataSize_; ++i) {

            if (enableAngularWrap_) {

                if ((prevState_(i) - input(i)) > M_PI) {

                    input(i) = input(i) + 2 * M_PI;
                }

                else if ((prevState_(i) - input(i)) < -M_PI) {

                    input(i) = input(i) - 2 * M_PI;
                }
            }
            filteredData_(i) = alpha_ * input(i) + (1 - alpha_) * prevState_(i);

            if (enableAngularWrap_) {

                if (filteredData_(i) > M_PI) {

                    filteredData_(i) = filteredData_(i) - 2 * M_PI;

                }
                 else if (filteredData_(i) < -M_PI) {

                    filteredData_(i) = filteredData_(i) + 2 * M_PI;
                }
            }

            prevState_(i) = filteredData_(i);
        }

        return filteredData_;
    }
     else {

        std::cout << "Error: FirstOrderFilter::Init(dataSize, alpha) was not called!\n"
                  << "Exiting..." << std::endl;
        exit(EXIT_FAILURE);

    }
}



void status_callback(const udp_server::status_timestampedConstPtr& status){

	cat_status=*status;
	ROS_INFO_STREAM("cat info received at " << cat_status.header.stamp.toSec());
}



void callback(const sensor_msgs::ImageConstPtr& img_ptr,const sensor_msgs::PointCloud2ConstPtr& pc_ptr)
{

	ROS_INFO_STREAM("Camera info received at " << img_ptr->header.stamp.toSec());
	ROS_INFO_STREAM("Velodyne scan received at " << pc_ptr->header.stamp.toSec());

	cv_ptr = cv_bridge::toCvCopy(img_ptr, sensor_msgs::image_encodings::BGR8);


	pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_p (new pcl::PointCloud<pcl::PointXYZI>);
  pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_t (new pcl::PointCloud<pcl::PointXYZI>);
		//pcl::fromROSMsg( *pc_ptr, *cloud);
	//uncomment to use old cloud point too

	 pcl::fromROSMsg( *pc_ptr, *cloud_p);
	 pcl::fromROSMsg( *pc_ptr, *cloud_t);

	 cout << cloud_p->points.size() << endl;
	 if (avilableold==true)
   {
    // copyPointCloud(*cloud_p, *print_old);

    copyPointCloud(*old, *print_old);


     *cloud_p+=*old;
	 }

   else {

     copyPointCloud(*cloud_t, *print_old);


   }


	 old->points.clear();
	 copyPointCloud(*cloud_t, *old);
	 avilableold=true;

	 cout << cloud_p->points.size() << endl;
	 copyPointCloud(*cloud_p,*cloud );



}


Quaternionf compute_quaternion(FirstOrderFilter anglefilter, int alfa){

    Quaternionf quaternion = Eigen::AngleAxisf(anglefilter.filteredData_(0),
					Eigen::Vector3f::UnitZ())
					* Eigen::AngleAxisf(alfa*anglefilter.filteredData_(2),
							Eigen::Vector3f::UnitY())
					* Eigen::AngleAxisf(alfa*anglefilter.filteredData_(1),
						Eigen::Vector3f::UnitX());
    return quaternion;
}
