#include <marker_creation.hpp>

MarkerCreation::MarkerCreation() {

//inizializzazione dei parametri che uso nelle seguenti funzioni


}


visualization_msgs::MarkerArray  MarkerCreation::creation(Eigen::Quaternionf q, int i, std::vector<float> array_pos, std::vector<double> array_scale, visualization_msgs::MarkerArray bbox_markers)  {

    marker.header.frame_id = "inertia";     //passa marker, array_pos, rotational_matrix_OBB, bbox_markers
    marker.header.stamp = ros::Time();
    marker.type = visualization_msgs::Marker::CUBE;
    marker.action = visualization_msgs::Marker::ADD;
    marker.id = i;
    marker.lifetime = ros::Duration(1);

    marker.pose.position.x = array_pos[0];
    marker.pose.position.y = array_pos[1];
    marker.pose.position.z = array_pos[2];


    marker.pose.orientation.x = q.x();
    marker.pose.orientation.y = q.y();
    marker.pose.orientation.z = q.z();
    marker.pose.orientation.w = q.w();


    marker.scale.x = array_scale[0];
    marker.scale.y = array_scale[1];
    marker.scale.z = array_scale[2];

    marker.color.a = 0.2;
    marker.color.r = 0.0;
    marker.color.g = 1.0;
    marker.color.b = 0;

    bbox_markers.markers.push_back(marker);


   return bbox_markers;

  }

//MarkerCreation::~MarkerCreation() {

  //inizializzazione dei parametri che uso nelle seguenti funzioni


//}
