/*
 * track.h
 *      Author: mina
 */

#ifndef TRACKDIM_H_
#include "tracking_lidar/Statusdim.h"
#include "opencv2/opencv.hpp"
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/video/tracking.hpp>
#include <opencv2/video/video.hpp>
#include <ros/ros.h>
#include <cmath>
#define TRACKDIM_H_

using namespace cv;
using namespace std;
using namespace ros;



class trackdim {
public:

	bool new_measure_avilabe;
	int lost_count;
	cv::KalmanFilter KF;
	cv::Mat measurement;
	cv::Mat prediction;
	cv::Mat previous_prediction;
	cv::Mat estimation;
	float dt=.1;
	double mahalanobis;
	void set_measurment(float, float , float, float, float);
	void  predict();
	virtual ~trackdim();
	void estimate();
	trackdim();
	trackdim(tracking_lidar::Statusdim);
	void mahalanobis_distance();




};

#endif /* TRACK_H_ */
