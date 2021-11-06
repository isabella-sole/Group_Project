#ifndef PYRAMID_H
#define PYRAMID_H

#include <Eigen/Geometry>
#include <Eigen/Eigen>
#include <Eigen/Core>
#include <Eigen/Dense>
#include <Eigen/QR>
#include <velodyne_pointcloud/point_types.h>
#include <pcl_ros/point_cloud.h>



struct Pyramid {

  Eigen::Vector4d p3d1;
  Eigen::Vector4d p3d2;
  Eigen::Vector4d p3d3;
  Eigen::Vector4d p3d4;

  Eigen::VectorXd p1;
  Eigen::VectorXd p2;
  Eigen::VectorXd p3;
  Eigen::VectorXd p4;

};



bool TestPointInPyramid (const pcl::PointXYZI  pcpoint, Pyramid *pyramid );




Pyramid create_pyramids(Pyramid pyramid, Eigen::MatrixXd  bounding_box, Eigen::MatrixXd invcalib);

#endif
