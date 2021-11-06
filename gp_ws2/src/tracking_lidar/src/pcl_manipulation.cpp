#include <pcl_manipulation.hpp>
#include <marker_creation.hpp>
#include <ros/ros.h>
#include <Eigen/Geometry>
#include <write_log.hpp>
#include <Eigen/Eigen>




PclManipulation::PclManipulation() {

//inizializzazione dei parametri che uso nelle seguenti funzioni
  transform_z = Eigen::Affine3f::Identity();
  theta = 90 * (M_PI / 180);
  transform_3 = Eigen::Affine3f::Identity();
  theta2 = -90 * (M_PI / 180);



}



void  PclManipulation::rotation_z(pcl::PointCloud<pcl::PointXYZI>::Ptr transformed_cloud, pcl::PointCloud<pcl::PointXYZI>::Ptr cloud)
{

  transform_z.translation() << 0.0, 0.0, 0.0;
  // 90 degree rotation in z to allign axis betzeen matlab and pc
  transform_z.rotate(Eigen::AngleAxisf(theta, Eigen::Vector3f::UnitZ()));
  pcl::transformPointCloud(*cloud, *transformed_cloud, transform_z);

}


void PclManipulation::filter_in_y(pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_filtered, pcl::PointCloud<pcl::PointXYZI>::Ptr transformed_cloud)
{

  pass.setInputCloud(transformed_cloud);
  pass.setFilterFieldName("y");
  pass.setFilterLimits(0.0, 1000);
  pass.filter(*cloud_filtered);

}


void PclManipulation::test_point_in_pyramids(pcl::PointCloud<pcl::PointXYZI>::Ptr transformed_cloud1, pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_filtered, pcl::PointCloud<pcl::PointXYZI>::Ptr filterd_pc, pcl::PointCloud<pcl::PointXYZI>::Ptr transformed_filter1, Pyramid *my_pyramid)
{

  transform_3.translation() << 0.0, 0.0, 0.0;
  transform_3.rotate(Eigen::AngleAxisf(theta2, Eigen::Vector3f::UnitZ()));
  pcl::transformPointCloud(*cloud_filtered, *transformed_cloud1, transform_3);

  // filter point cloud inside the pyramids
  for (std::size_t i = 0; i <= transformed_cloud1->size(); ++i) {

    if (TestPointInPyramid(transformed_cloud1->points[i], my_pyramid)) {
        filterd_pc->push_back(transformed_cloud1->points[i]);
      }
    }

    pcl::transformPointCloud(*filterd_pc, *transformed_filter1, transform_z);


}




bool PclManipulation::extracting_features(pcl::PointCloud<pcl::PointXYZI>::Ptr new_transformed_filter, trackdim *track1, ros::Publisher *bbox3d_pub, geometry_msgs::Quaternion *TR, pcl::PointCloud<pcl::PointXYZI>::Ptr obstcal_cloud, bool track_init){


//  bool track_init = false;
  if (new_transformed_filter->size() > 40) {
    new_transformed_filter->is_dense = false; // to be able to remove nan
    // remove nan values from point cloud
    pcl::PointCloud<pcl::PointXYZI>::Ptr transformed_filter(new pcl::PointCloud<pcl::PointXYZI>());
    std::vector<int> indices;
    pcl::removeNaNFromPointCloud(*new_transformed_filter, *transformed_filter, indices);

    // eclddiqn clustring
  //				pcl::search::Search<pcl::PointXYZI>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZI>);
    pcl::search::Search<pcl::PointXYZI>::Ptr tree(new pcl::search::BruteForce<pcl::PointXYZI>);
    std::vector<pcl::PointIndices> clusters;
    pcl::EuclideanClusterExtraction < pcl::PointXYZI > ec;
    /*
      Default: 2.5 100 700
      Prova 1: 1000 10 1000
      Prova 2: 1000 2 10000

    */
    ec.setClusterTolerance(2); // in meter  //default 2.5.
    ec.setMinClusterSize(35); //default 100
    ec.setMaxClusterSize(200); //default 700
    ec.setSearchMethod(tree);
    ec.setInputCloud(transformed_filter);
    ec.extract(clusters);
    cout << "cluster size" << clusters.size() << endl;
    if (clusters.size() > 0) {

      visualization_msgs::MarkerArray bbox_markers;

      // extract centroid and bounding box dimension for all clusters
      for (size_t i = 0; i < clusters.size(); ++i) {

        pcl::PointIndices inliers = clusters[i];
        pcl::PointCloud<pcl::PointXYZI>::Ptr objectPC(new pcl::PointCloud<pcl::PointXYZI>());
        pcl::MomentOfInertiaEstimation < pcl::PointXYZI > feature_extractor;
        pcl::copyPointCloud(*transformed_filter, clusters[i],*objectPC);

        feature_extractor.setInputCloud(objectPC);
        feature_extractor.compute();
        std::vector<float> moment_of_inertia;
        std::vector<float> eccentricity;
        // AABB axis-alligned bounding box (alligned with the axis of coord sys)
        // in this case we use  the orientation of the global ref frame (camera o lidar??)
        pcl::PointXYZI min_point_AABB; // min cioe punto davanti(z), in basso(y), a sx(x)
        pcl::PointXYZI max_point_AABB; // max cioe punto dietro(z), in alto(y), a dx(x)
        // OOB: oriented bounding box (a local or arbitrary bb is used)
        // in this case we use  the orientation of the global ref frame
        pcl::PointXYZI min_point_OBB;
        pcl::PointXYZI max_point_OBB;
        pcl::PointXYZI position_OBB;
        Eigen::Matrix3f rotational_matrix_OBB;
        Eigen::Matrix3f rotation90Z;
        rotation90Z << 0, -1, 0, 1, 0, 0, 0, 0, 1;
        float major_value, middle_value, minor_value;
        Eigen::Vector3f major_vector, middle_vector, minor_vector;
        Eigen::Vector3f mass_center;
        //std::cout << "QUI: " << typeof(mass_center[0]) << std::endl;
        feature_extractor.getMomentOfInertia(moment_of_inertia);
        feature_extractor.getEccentricity(eccentricity);
        feature_extractor.getAABB(min_point_AABB,
            max_point_AABB);
        feature_extractor.getOBB(min_point_OBB, max_point_OBB,
            position_OBB, rotational_matrix_OBB);
        feature_extractor.getEigenValues(major_value,
            middle_value, minor_value);
        feature_extractor.getEigenVectors(major_vector,
            middle_vector, minor_vector);
        feature_extractor.getMassCenter(mass_center);

        psi_angle = atan2(major_vector(1), major_vector(0))*(180.0/3.14);

        cout<<"psi_angle:"<<psi_angle<<endl;



        //NEW markers creation
        // mass center position
        std::vector<float> mass_center_position;
        // or double mass_center_array [3];
        // quando passi vettore alla funzione se usi double metti :  double mass_center_array[]


        for (int j = 0; j < 3; j++)
        {
            mass_center_position.push_back(mass_center[j]);
        }

        Eigen::Quaternionf q(rotational_matrix_OBB );


        // mass center scale
        std::vector<double> mass_center_scale;
        mass_center_scale.push_back(max_point_AABB.x - min_point_AABB.x);
        mass_center_scale.push_back(max_point_AABB.y - min_point_AABB.y);
        mass_center_scale.push_back(max_point_AABB.z - min_point_AABB.z);

        MarkerCreation mar;

        bbox_markers = mar.creation(q, i, mass_center_position, mass_center_scale, bbox_markers);

      }

      // publish clustrues as box markers for better visualization
      bbox3d_pub->publish(bbox_markers);

      float x;
      float y;
      float l;
      float w;

      if (track_init == false) {
        int id = neares_track(bbox_markers);
        //cout<<"nearset track id = " <<id<<endl;
        x = bbox_markers.markers[id].pose.position.x;
        y = bbox_markers.markers[id].pose.position.y;
        l = bbox_markers.markers[id].scale.x;
        w = bbox_markers.markers[id].scale.y;
    //    plot_position_init(x, y, l, w);

        *TR=bbox_markers.markers[id].pose.orientation;


    //    tracking_lidar::Statusdim init;
        init.x = x;
        init.y = y;
        init.vx = 0;
        init.vy = 0;
        init.l=l;
        init.w=w;
        init.psi= 10;
        *track1 = trackdim(init);
        track_init = true;


      }

      if (track_init) {

        int id = track_assooc(bbox_markers, track1->prediction.at<float>(0), track1->prediction.at<float>(1));


        if (id != 100) {

          float x = bbox_markers.markers[id].pose.position.x;
          float y = bbox_markers.markers[id].pose.position.y;
          float l= bbox_markers.markers[id].scale.x;
          float w=  bbox_markers.markers[id].scale.y;
          *TR=bbox_markers.markers[id].pose.orientation;

          x_init = x;
          y_init = y;
          l_init = l;
          w_init = w;




      //   plot_position_init(x, y, l, w);




          pcl::copyPointCloud(*transformed_filter, clusters[id], *obstcal_cloud);
          track1->set_measurment(x,y,w,l, psi_angle);
        }
      }
    }
  }


return track_init;
}



void PclManipulation::print_cloud(bool track_init, trackdim *track1, bool print_flag, ros::Publisher *bbox3d_pub_KF, udp_server::status_timestamped *cat_status, ros::Publisher *Prediction, ros::Publisher *Estimation, geometry_msgs::Quaternion *TR, Eigen::Quaternionf *qinertia, pcl::PointCloud<pcl::PointXYZI>::Ptr obstcal_cloud, InitBbCalib *ibc, cv_bridge::CvImagePtr cv_ptr, bool colour, ofstream *myfile, ofstream *myfile2, ofstream *myfile3)
{
  if ((track_init == true)) {


    tracking_lidar::Statusdim pre;

    pre = KF_prediction(track1, pre);


    track1->mahalanobis_distance();

    tracking_lidar::Statusdim est;

    est = KF_estimation(track1, est);

    *myfile<<est.l;
    *myfile<<"\n";

    *myfile2<<est.w;
    *myfile2<<"\n";

    *myfile3<<cat_status->longitude;
    *myfile3<<"  ";
    *myfile3<<cat_status->latitude;
    *myfile3<<"  ";
    *myfile3<<est.x;
    *myfile3<<"  ";
    *myfile3<<est.y;
    *myfile3<<"\n";











    // write on txt
    WriteLog Log_Track;
    string myfile="log_track";
    Log_Track.write_log_track(myfile, pre, est, *cat_status );

    Prediction->publish(pre);
    Estimation->publish(est);


    //markers creation

    Eigen::Quaternionf TR_quat(TR->w, TR->x, TR->y, TR->z);

    visualization_msgs::MarkerArray bbox_markers_KF;
    std::vector<float> marker_KF_position;

    marker_KF_position.push_back(est.x);
    marker_KF_position.push_back(est.y);
    marker_KF_position.push_back(0);


    std::vector<double> marker_KF_scale;

    marker_KF_scale.push_back(est.l);
    marker_KF_scale.push_back(est.w);
    marker_KF_scale.push_back(0);


    MarkerCreation mar_KF;
    bbox_markers_KF = mar_KF.creation(TR_quat, 0, marker_KF_position, marker_KF_scale, bbox_markers_KF);
    bbox3d_pub_KF->publish(bbox_markers_KF);

    float z = -1;

    pcl::PointCloud<pcl::PointXYZ>::Ptr centriod(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointXYZ c;

    c.x = est.x;
    c.y = est.y;
    c.z = 0;

    centriod->push_back(c);
    Eigen::Affine3f transform_inertialT = Eigen::Affine3f::Identity();
    transform_inertialT.translation() << 0.0, 0.0, 1.0;
    transform_inertialT.rotate(*qinertia);
    Eigen::Affine3f transform_inertial=transform_inertialT.inverse();

    pcl::transformPointCloud(*centriod, *centriod, transform_inertial);

    float cent_x = centriod->points[0].x;
    float cent_y = centriod->points[0].y;
    float cent_z = centriod->points[0].z;

    WriteLog LogPos;
    string myfile1="log_pos";
    LogPos.write_log_pos(myfile1, cent_x, cent_y);


    if (obstcal_cloud->size() > 0) {

      pcl::transformPointCloud(*obstcal_cloud, *obstcal_cloud, transform_inertial);


    if(print_flag)
    {
      for (std::size_t i = 0; i <= obstcal_cloud->size(); ++i) {

          float centx = obstcal_cloud->points[i].x;
          float centy = obstcal_cloud->points[i].y;
          float centz = obstcal_cloud->points[i].z;


          int u =compute_u(*ibc, centx, centy, centz);
          int v =compute_v(*ibc, centx, centy, centz);


          if (colour ==0)
            cv::drawMarker(cv_ptr->image, cv::Point(u+40, (v/10)+400), (255,0,255), MARKER_CROSS, 10, 2, 8);
          else if (colour ==1)
            cv::drawMarker(cv_ptr->image, cv::Point(u+40, (v/10)+400), (255,0,0), MARKER_CROSS, 10, 2, 8);


         //Default (255, 0, 255), MARKER_CROSS, 10, 2, 8)


        }

        //PLOT A CROSS
        // int cent_u =compute_u(*ibc, cent_x, cent_y, cent_z);
        // int cent_v =compute_v(*ibc, cent_x, cent_y, cent_z);
        // cv::drawMarker(cv_ptr->image, cv::Point(cent_u, cent_v), (0, 0, 255), MARKER_CROSS, 20, 4, 8);


//PLOT A CROSS ON THE KF PUTPUT->SET CORRECT VALUES FOR THE TUNING PARAMETERS
      //   cout<<"*****************x_est_temp:"<<x_est_temp<<endl;
      //   cout<<"*****************y_est_temp:"<<y_est_temp<<endl;
      //
      //
      //
      //   int x_est_temp2 = static_cast<int>(x_est_temp);
      //   int y_est_temp2 = static_cast<int>(y_est_temp);
      //
      //   cout<<"x_est_temp2:"<<x_est_temp2<<endl;
      //   cout<<"y_est_temp2:"<<y_est_temp2<<endl;
      //
      //   int cent_u2 =compute_u(*ibc, x_est_temp2, y_est_temp2, 0);
      //   int cent_v2 =compute_v(*ibc, x_est_temp2, y_est_temp2, 0);
      //
      //   cout<<"*****************cent_u2:"<<cent_u2<<endl;
      //   cout<<"*****************cent_v2:"<<cent_v2<<endl;
      //
      // //  cv::drawMarker(cv_ptr->image, cv::Point(cent_u2, cent_v2), (0, 0, 255), MARKER_CROSS, 20, 4, 8);
      //   cv::drawMarker(cv_ptr->image, cv::Point((x_est_temp2*(-10))+400, (y_est_temp2*(-10))+400), (255, 0, 0), MARKER_CROSS, 20, 4, 8);


       //


      }




    }

    cout<<"lost count = "<<track1->lost_count<<endl;
      if (track1->lost_count>=10){
      track_init =false;
      }
  }
}



int PclManipulation::neares_track(visualization_msgs::MarkerArray arr)
{
	std::vector<float> dist;

	for (int i= 0; i < arr.markers.size();i++) {

		 dist.push_back(sqrt(pow(arr.markers[i].pose.position.x,2)+pow(arr.markers[i].pose.position.y,2)+pow(arr.markers[i].pose.position.z,2)));

	}


	int minElementIndex = std::min_element(dist.begin(),dist.end()) - dist.begin();


	return minElementIndex;

}



int PclManipulation::track_assooc(visualization_msgs::MarkerArray arr, float x, float y){

  std::vector<float> dist;

  for (int i= 0; i < arr.markers.size();i++) {

    dist.push_back(sqrt(pow(arr.markers[i].pose.position.x-x,2)+pow(arr.markers[i].pose.position.y-y,2)));
  }


  int assoc_track = std::min_element(dist.begin(),dist.end()) - dist.begin();

  cout<<"neaerts track distance =" <<dist.at(assoc_track) << endl;

  if (dist.at(assoc_track)>5) {
    assoc_track=100;
  }


  cout<<"neaerts track distance id =" << assoc_track<< endl;
  return assoc_track;


}



 tracking_lidar::Statusdim PclManipulation::KF_prediction(trackdim *track, tracking_lidar::Statusdim pre)   {


      track->predict();
      cout << " KF. gain" << track->KF.gain << endl;
      cout << "prediction" << track->prediction << endl;


      pre.x = track->prediction.at<float>(0);
			pre.y = track->prediction.at<float>(1);
			pre.w = track->prediction.at<float>(2);
			pre.l = track->prediction.at<float>(3);
			pre.vx = track->prediction.at<float>(4);
      pre.vy = track->prediction.at<float>(5);
			pre.psi = track->prediction.at<float>(6);


  //    plot_position(&pre, track);




      return pre;
}



tracking_lidar::Statusdim PclManipulation::KF_estimation(trackdim *track, tracking_lidar::Statusdim est)   {


      track->estimate();
      cout << " KF. gain" << track->KF.gain << endl;
			cout << "estimation" << track->estimation << endl;

      est.x = track->estimation.at<float>(0);
      est.y = track->estimation.at<float>(1);
      est.w = track->estimation.at<float>(2);
      est.l = track->estimation.at<float>(3);
      est.vx = track->estimation.at<float>(4);
      est.vy = track->estimation.at<float>(5);
      est.psi = track->estimation.at<float>(6);

      // x_est_temp = est.x;
      // y_est_temp = est.y;


      plot_position(&est, track);


      return est;
}



int PclManipulation::compute_u(InitBbCalib ibc, float centx, float centy, float centz){
    int u = (ibc.calib(0, 0) * centx + ibc.calib(0, 1) * centy
		+ ibc.calib(0, 2) * centz + ibc.calib(0, 3) * 1)
		/ (ibc.calib(2, 0) * centx + ibc.calib(2, 1) * centy
		+ ibc.calib(2, 2) * centz + ibc.calib(2, 3) * 1);

    return u;

}

int PclManipulation::compute_v(InitBbCalib ibc, float centx, float centy, float centz)
{
    int v = (ibc.calib(1, 0) * centx + ibc.calib(1, 1) * centy
			+ ibc.calib(1, 2) * centz + ibc.calib(1, 3) * 1)
			/ (ibc.calib(2, 0) * centx + ibc.calib(2, 1) * centy
			+ ibc.calib(2, 2) * centz + ibc.calib(2, 3) * 1);
    return v;

}


//  PLOT POSITION  FROM CLUSTER DIMENSION (Before Kalman Filter)  DOPPIO

void PclManipulation::plot_position(tracking_lidar::Statusdim *pre, trackdim *track)
{


int center_init = 500;
int center_pre = 500;

cv::Mat image(center_init*2.5,center_init*2.5, CV_8UC3, cv::Scalar(0,0,0));

cv::Point centerCircle(center_init, center_init);

cv::Point centerRect(center_init - 20*x_init,center_init + 20*y_init);
cv::Point centerRect2(center_pre - 20*pre->x,center_pre + 20*pre->y);

cv::Size sizeRect(10*l_init, 10*w_init);
cv::Size sizeRect2(10*pre->l, 10*pre->w);



int radiusCircle = 5;
// int radiusCircle2 = 10;
cv::Scalar colorCircle(255,0,0);


cv::circle(image, centerCircle, radiusCircle, colorCircle, CV_FILLED);
// cv::rectangle(image,pointRect1,pointRect2,cv::Scalar(0,255,0));
// cv::rectangle(image,pointRect1_pre,pointRect2_pre,cv::Scalar(0,0,255));

cv::RotatedRect rRect= cv::RotatedRect(centerRect, sizeRect, (float)pre->psi);
cv::RotatedRect rRect2= cv::RotatedRect(centerRect2, sizeRect2, (float)pre->psi);

cv::Point2f vertices[4];
cv::Point2f vertices2[4];

rRect.points(vertices);
rRect2.points(vertices2);

for(int i=0; i<4; i++)
{
  cv::line(image, vertices[i], vertices[(i+1)%4], cv::Scalar(0,255,0));
  cv::line(image, vertices2[i], vertices2[(i+1)%4], cv::Scalar(0,0,255));

}

cv::namedWindow( "Display window", cv::WINDOW_AUTOSIZE );
cv::imshow( "Display window", image );

cout<<"x_init:"<<x_init<<endl;
cout<<"y_init:"<<y_init<<endl;
cout<<"l_int:"<<l_init<<endl;
cout<<"w_init:"<<w_init<<endl;

cout<<"x_pre:"<<pre->x<<endl;
cout<<"y_pre:"<<pre->y<<endl;
cout<<"l_pre:"<<pre->l<<endl;
cout<<"w_pre:"<<pre->w<<endl;
cv::waitKey(1);
}






PclManipulation::~PclManipulation() {

}
