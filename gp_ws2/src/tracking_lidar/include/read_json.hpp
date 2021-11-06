#include <ros/ros.h>
#include <iostream>
#include <string>
#include <fstream>

#include <Eigen/Eigen>
#include <Eigen/Core>
#include <Eigen/Dense>
#include <Eigen/QR>
#include <Eigen/Geometry>

#include <unistd.h>
#include <stdio.h>
#include <limits.h>

#include <json/json.h>
//#include <json/reader.h>
//#include <json/value.h>
//#include <json/tool.h>

#include <utility>
#include <cstdio>
#include <cassert>
#include <cstring>
#include <stdexcept>

#include <experimental/filesystem>

/*
#include <boost/property_tree/ptree.hpp>
#include <boost/property_tree/json_parser.hpp>*/

using namespace std;

class ReadJson {

    private:

        string prefix;
        string path;

        int image_id;
        int category_id;
        std::vector<float> bbox;
        int id;
        int score;

        std::vector<int> image_id_vector;
        std::vector<int>  category_id_vector;
        std::vector<std::vector<float>> bbox_vector;
        std::vector<int>  id_vector;
        std::vector<int>  score_vector;

        int r2, c2;


    public:

        Eigen::MatrixXd  bounding_box;

        ReadJson();

        void read_Json_bb(ros::NodeHandle nh);
        void compute_vertices_bb(ros::NodeHandle nh);
        // void read_Json_bb(const char* filename);

        ~ReadJson();


};