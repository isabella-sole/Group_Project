#include <track_dim_node.h>
#include <marker_creation.hpp>
#include <write_log.hpp>
#include<pcl_manipulation.hpp>
#include <chrono>
#include <thread>
#include <pcl/search/brute_force.h>
#include <iostream>
#include <fstream>




int main(int argc, char** argv) {

	ofstream myfile, myfile2, myfile3;
	myfile.open ("/home/isabella/Desktop/gp_ws2/t_w.txt");
	myfile2.open ("/home/isabella/Desktop/gp_ws2/t_l.txt");
  myfile3.open ("/home/isabella/Desktop/gp_ws2/catPos_obsPos.txt");
	ros::init(argc, argv, "project_to_lidar");

	myfile<<"l\n";
	myfile2<<"w\n";
	myfile3<<"x_cat	y_cat	est_x	est_y\n";




	trackdim track1;
	trackdim track2;
	trackdim track3;

	ros::NodeHandle nh;
	ros::Rate loop_rate(10);

	FirstOrderFilter anglefilter;

	tf::TransformBroadcaster br;
	tf::Transform transform;
	//lidar_sub = nh.subscribe<sensor_msgs::PointCloud2>(VELODYNE_TOPIC,1,lidar_callback);
	cat_sub=nh.subscribe<udp_server::status_timestamped>(CAT_STATUS_TOPIC,1,status_callback);
	//cam_sub = nh.subscribe<sensor_msgs::Image>(CAMERA_TOPIC,1,cam_callback);
	pub_filterd = nh.advertise < sensor_msgs::PointCloud2 > ("pub_filterd", 10);
	pub_filterd_inY=nh.advertise < sensor_msgs::PointCloud2 > ("pub_filterd_inY", 10);
	pub_regions=nh.advertise < sensor_msgs::PointCloud2 > ("pub_region", 10);
	bbox3d_pub =nh.advertise<visualization_msgs::MarkerArray>("vz",100);
	bbox3d_pub_KF =nh.advertise<visualization_msgs::MarkerArray>("KF",100);
	Prediction= nh.advertise<tracking_lidar::Statusdim>("predicition", 10);
	Estimation= nh.advertise<tracking_lidar::Statusdim>("estimation", 10);
	image = nh.advertise<sensor_msgs::Image>("image", 1);


	message_filters::Subscriber < sensor_msgs::Image > img_sub(nh, CAMERA_TOPIC,1);
	message_filters::Subscriber < sensor_msgs::PointCloud2 > cloud_sub(nh, VELODYNE_TOPIC,1);


	typedef sync_policies::ApproximateTime<sensor_msgs::Image,sensor_msgs::PointCloud2> MySyncPolicy;
	Synchronizer<MySyncPolicy> sync(MySyncPolicy(10), img_sub, cloud_sub);
	sync.registerCallback(boost::bind(&callback, _1, _2));

	geometry_msgs::Quaternion TR;

	ROS_INFO_STREAM("Reading Camera and lidar from topics");

	// kalman filter
	bool track_init = false;
	bool track_init2 = false;
	bool track_init3 = false;



  InitBbCalib ibc; // create this object to retrieve calibration matrix and boundin box
	ibc.init_calib_matrix(nh);  // set the calibration matrix from a configuration filter
  rml::RegularizationData regularizationData;
	Eigen::MatrixXd invcalib =rml::RegularizedPseudoInverse(ibc.calib,regularizationData);

	ibc.init_bb_matrix(nh);  // set the bb matrix from a configuration filter


	// Create pyramids
  Pyramid my_pyramid;

	my_pyramid = create_pyramids(my_pyramid, ibc.bounding_box, invcalib);


	while (ros::ok()) {

		Quaternionf qinertia;
		Quaternionf stablization;
		ros::spinOnce();
		loop_rate.sleep();
		bool track_update = false;


		anglefilter.Init(3, .1, false);
		Eigen::VectorXd anglevec(3);
		anglevec << cat_status.orientation.yaw, cat_status.orientation.pitch, cat_status.orientation.roll;

		pcl::PointCloud<pcl::PointXYZI>::Ptr obstcal_cloud(new pcl::PointCloud<pcl::PointXYZI>());
		pcl::PointCloud<pcl::PointXYZI>::Ptr obstcal_cloud2(new pcl::PointCloud<pcl::PointXYZI>());
		pcl::PointCloud<pcl::PointXYZI>::Ptr obstcal_cloud3(new pcl::PointCloud<pcl::PointXYZI>());

		PclManipulation pclman;
		PclManipulation pclman2;
		PclManipulation pclman3;

		//if ((cloud->size() > 200)) {
		cout<<cloud->size()<<endl;

		if ((cloud->size() > 100)) {


			anglefilter.Filter(anglevec);

			cout << anglefilter.filteredData_ << endl;
			// anglefilter.filteredData_(0)=cat_status.orientation.yaw;
			anglefilter.filteredData_(1)=cat_status.orientation.pitch;
			anglefilter.filteredData_(2)=cat_status.orientation.roll;

			cout << cat_status.orientation.yaw << ","
					<< cat_status.orientation.pitch << ","
					<< cat_status.orientation.roll << endl;


			// roation in Z
			pcl::PointCloud<pcl::PointXYZI>::Ptr transformed_cloud(new pcl::PointCloud<pcl::PointXYZI>());
			pcl::PointCloud<pcl::PointXYZI>::Ptr transformed_cloud2(new pcl::PointCloud<pcl::PointXYZI>());
			pcl::PointCloud<pcl::PointXYZI>::Ptr transformed_cloud3(new pcl::PointCloud<pcl::PointXYZI>());
			pclman.rotation_z(transformed_cloud, cloud);
			pclman2.rotation_z(transformed_cloud2, old);
			pclman3.rotation_z(transformed_cloud3, print_old);



			//filter point cloud in y direction and publish filtered data
			pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_filtered(new pcl::PointCloud<pcl::PointXYZI>);
			pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_filtered2(new pcl::PointCloud<pcl::PointXYZI>);
			pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_filtered3(new pcl::PointCloud<pcl::PointXYZI>);
			pclman.filter_in_y(cloud_filtered, transformed_cloud);
			pclman2.filter_in_y(cloud_filtered2, transformed_cloud2);
			pclman3.filter_in_y(cloud_filtered3, transformed_cloud3);


			// publish filtered data
      sensor_msgs::PointCloud2 filterd_rosY;
			pcl::toROSMsg(*cloud_filtered, filterd_rosY);
			filterd_rosY.header.frame_id = "velodyne";
			pub_filterd_inY.publish(filterd_rosY);

			//back to old coordinate to test points in pyramids: filter point cloud inside the pyramids and rotate point cloud again
			pcl::PointCloud<pcl::PointXYZI>::Ptr transformed_cloud1(new pcl::PointCloud<pcl::PointXYZI>());
			pcl::PointCloud<pcl::PointXYZI>::Ptr filterd_pc(new pcl::PointCloud<pcl::PointXYZI>);
			pcl::PointCloud<pcl::PointXYZI>::Ptr transformed_filter1(new pcl::PointCloud<pcl::PointXYZI>());

			pcl::PointCloud<pcl::PointXYZI>::Ptr transformed_cloud2_old(new pcl::PointCloud<pcl::PointXYZI>());
			pcl::PointCloud<pcl::PointXYZI>::Ptr filterd_pc2_old(new pcl::PointCloud<pcl::PointXYZI>);
			pcl::PointCloud<pcl::PointXYZI>::Ptr transformed_filter2_old(new pcl::PointCloud<pcl::PointXYZI>());

			pcl::PointCloud<pcl::PointXYZI>::Ptr transformed_cloud3_old(new pcl::PointCloud<pcl::PointXYZI>());
			pcl::PointCloud<pcl::PointXYZI>::Ptr filterd_pc3_old(new pcl::PointCloud<pcl::PointXYZI>);
			pcl::PointCloud<pcl::PointXYZI>::Ptr transformed_filter3_old(new pcl::PointCloud<pcl::PointXYZI>());

			pclman.test_point_in_pyramids(transformed_cloud1, cloud_filtered, filterd_pc, transformed_filter1, &my_pyramid);
			pclman2.test_point_in_pyramids(transformed_cloud2_old, cloud_filtered2, filterd_pc2_old, transformed_filter2_old, &my_pyramid);
			pclman3.test_point_in_pyramids(transformed_cloud3_old, cloud_filtered3, filterd_pc3_old, transformed_filter3_old, &my_pyramid);



			qinertia=compute_quaternion(anglefilter,1);
			stablization=compute_quaternion(anglefilter,-1);
			qinertia=qinertia.inverse();

			// Create this quaternion from roll/pitch/yaw (in radians)
			transform.setOrigin(tf::Vector3(0.0, 0.0, 0.0));
			transform.setRotation(tf::Quaternion(qinertia.x(), qinertia.y(), qinertia.z(), qinertia.w()));
			br.sendTransform(tf::StampedTransform(transform, ros::Time::now(),"velodyne", "inertia"));


			Eigen::Affine3f transform_inertial = Eigen::Affine3f::Identity();
			transform_inertial.translation() << 0.0, 0.0, 0.0;
			transform_inertial.rotate(qinertia);
			pcl::transformPointCloud(*transformed_filter1, *transformed_filter1, transform_inertial);
			pcl::transformPointCloud(*transformed_filter2_old, *transformed_filter2_old, transform_inertial);
			pcl::transformPointCloud(*transformed_filter3_old, *transformed_filter3_old, transform_inertial);





			// trasform to ros message and publish
			sensor_msgs::PointCloud2 filterd_ros;
			pcl::toROSMsg(*transformed_filter1, filterd_ros);
			filterd_ros.header.frame_id = "inertia";
			pub_filterd.publish(filterd_ros);

			pcl::ModelCoefficients::Ptr coefficients( new pcl::ModelCoefficients());
			coefficients->values.resize(4);
			coefficients->values[0] = coefficients->values[1] = 0;
			coefficients->values[2] = -1;
			coefficients->values[3] = 0;

			// Create the filtering object
			pcl::PointCloud<pcl::PointXYZI>::Ptr transformed_filter12( new pcl::PointCloud<pcl::PointXYZI>());
			pcl::ProjectInliers < pcl::PointXYZI > proj;
			proj.setModelType(pcl::SACMODEL_PLANE);
			proj.setInputCloud(transformed_filter1);
			proj.setModelCoefficients(coefficients);
			proj.filter(*transformed_filter12);

			pcl::PointCloud<pcl::PointXYZI>::Ptr transformed_filter22_old( new pcl::PointCloud<pcl::PointXYZI>());
			pcl::ProjectInliers < pcl::PointXYZI > proj2;
			proj2.setModelType(pcl::SACMODEL_PLANE);
			proj2.setInputCloud(transformed_filter2_old);
			proj2.setModelCoefficients(coefficients);
			proj2.filter(*transformed_filter22_old);

			pcl::PointCloud<pcl::PointXYZI>::Ptr transformed_filter32_old( new pcl::PointCloud<pcl::PointXYZI>());
			pcl::ProjectInliers < pcl::PointXYZI > proj3;
			proj3.setModelType(pcl::SACMODEL_PLANE);
			proj3.setInputCloud(transformed_filter3_old);
			proj3.setModelCoefficients(coefficients);
			proj3.filter(*transformed_filter32_old);

			sensor_msgs::PointCloud2 proj_ros;
			pcl::toROSMsg(*transformed_filter12, filterd_ros);
			filterd_ros.header.frame_id = "inertia";
			pub_regions.publish(filterd_ros);

	    // creat inertial frame
			track_init = pclman.extracting_features(transformed_filter12, &track1, &bbox3d_pub, &TR, obstcal_cloud, track_init);
			track_init2 = pclman2.extracting_features(transformed_filter22_old, &track2, &bbox3d_pub, &TR, obstcal_cloud2, track_init2);
			track_init3 = pclman3.extracting_features(transformed_filter32_old, &track3, &bbox3d_pub, &TR, obstcal_cloud3, track_init3);
		}


		// pclman.print_cloud(track_init, &track1, 0, &bbox3d_pub_KF, &cat_status, &Prediction, &Estimation, &TR, &qinertia, obstcal_cloud, &ibc, cv_ptr,0);
		// pclman2.print_cloud(track_init2, &track2, 1, &bbox3d_pub_KF, &cat_status, &Prediction, &Estimation, &TR, &qinertia, obstcal_cloud2, &ibc, cv_ptr, 0);
		// pclman3.print_cloud(track_init3, &track3, 1, &bbox3d_pub_KF, &cat_status, &Prediction, &Estimation, &TR, &qinertia, obstcal_cloud3, &ibc, cv_ptr, 1);


//to save file .txt
	//	pclman.print_cloud(track_init, &track1, 1, &bbox3d_pub_KF, &cat_status, &Prediction, &Estimation, &TR, &qinertia, obstcal_cloud, &ibc, cv_ptr,0, &myfile, &myfile2, &myfile3);
		 pclman2.print_cloud(track_init2, &track2, 1, &bbox3d_pub_KF, &cat_status, &Prediction, &Estimation, &TR, &qinertia, obstcal_cloud2, &ibc, cv_ptr, 0, &myfile, &myfile2, &myfile3);
		// pclman3.print_cloud(track_init3, &track3, 1, &bbox3d_pub_KF, &cat_status, &Prediction, &Estimation, &TR, &qinertia, obstcal_cloud3, &ibc, cv_ptr, 1);



		if(track_init)
			image.publish(cv_ptr->toImageMsg());


	}


	ros::spin();
	//myfile.close();
	// myfile1.close();
	myfile.close();
	myfile2.close();
	myfile3.close();
	return EXIT_SUCCESS;
}
