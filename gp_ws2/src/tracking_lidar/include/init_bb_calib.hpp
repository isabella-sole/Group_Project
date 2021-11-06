#include <ros/ros.h>
#include <iostream>
#include <string>
#include <Eigen/Eigen>
#include <Eigen/Core>
#include <Eigen/Dense>
#include <Eigen/QR>
#include <Eigen/Geometry>

using namespace std;

class InitBbCalib {

private:

    string prefix;
    string calib_prefix;
    int r, c;

    string prefix2;
    string bb_prefix;
    int r2, c2;


public:

    Eigen::MatrixXd calib;
    Eigen::MatrixXd  bounding_box;

    InitBbCalib();

    void init_calib_matrix(ros::NodeHandle nh);
    void init_bb_matrix(ros::NodeHandle nh);

    ~InitBbCalib();


};
