/*
 * trackdim.cpp
 *      Author: mina
 */

#include "trackdim.h"

trackdim::trackdim(): KF(7,5,0,CV_32F), measurement(5,1,CV_32F), prediction (7,1,CV_32F), previous_prediction (7,1,CV_32F), estimation (7,1,CV_32F){

}
trackdim::trackdim(tracking_lidar::Statusdim init) : KF(7,5,0,CV_32F), measurement(5,1,CV_32F), prediction (7,1,CV_32F), previous_prediction (7,1,CV_32F), estimation (7,1,CV_32F){


		KF.transitionMatrix = (Mat_<float>(7, 7) << 1,0,0,0,dt,0,0,  0,1,0,0,0,dt,0,  0,0,1,0,0,0,0,  0,0,0,1,0,0,0,  0,0,0,0,1,0,0,  0,0,0,0,0,1,0, 0,0,0,0,0,0,1 );
    KF.measurementMatrix = cv::Mat::zeros(5,7,CV_32F);
    KF.measurementMatrix.at<float>(0)=1.0f;
    KF.measurementMatrix.at<float>(8)=1.0f;
    KF.measurementMatrix.at<float>(16)=1.0f;
		KF.measurementMatrix.at<float>(24)=1.0f;
		KF.measurementMatrix.at<float>(34)=1.0f;
    setIdentity(KF.processNoiseCov, Scalar::all(1e-2)); //settata per dare incertezza su modello che sto usando: anche se uso modello a velocità cost con l e w fissi ci metto incertezza piccola
    setIdentity(KF.measurementNoiseCov, Scalar::all(10));
		setIdentity(KF.errorCovPost, Scalar::all(.1)); // incertezza iniziale
		lost_count = 0;


 // intial condion
    KF.statePost.at<float>(0) = init.x;
		KF.statePost.at<float>(1) = init.y;
		KF.statePost.at<float>(2) = init.w;
		KF.statePost.at<float>(3) = init.l;
    KF.statePost.at<float>(4) = init.vx;
		KF.statePost.at<float>(5) = init.vy;
		KF.statePost.at<float>(6) = init.psi;



		prediction=KF.predict();


}


void trackdim::set_measurment(float x, float y,float w, float l, float psi){

	measurement.at<float>(0) = x;
	measurement.at<float>(1) = y;
	measurement.at<float>(4) = psi;
	if (w<0)
	{
		w=2;
	}

	if (l>(w+(.3*w)))
	{
	measurement.at<float>(2) = w;
    measurement.at<float>(3) = l;
	}

	if (l<=w)
	{
	measurement.at<float>(2) = prediction.at<float>(2);
    measurement.at<float>(3) = prediction.at<float>(3);
	}

 if (abs(psi)>110)
 {
	 measurement.at<float>(4)=prediction.at<float>(6);

 }

    new_measure_avilabe= true;
	lost_count=0;
}


void trackdim::predict(){

  prediction=KF.predict();


}

void trackdim::estimate(){

	if (new_measure_avilabe) {
		estimation= KF.correct(measurement);
	}

	else if(!new_measure_avilabe){
	     	measurement.at<float>(0)=prediction.at<float>(0);
			measurement.at<float>(1)=prediction.at<float>(1);

            measurement.at<float>(2)=prediction.at<float>(2);
						measurement.at<float>(3)=prediction.at<float>(3);
						measurement.at<float>(4)=prediction.at<float>(6);
						estimation= KF.correct(measurement);
						lost_count++;
	}

	new_measure_avilabe= false;
}


void trackdim::mahalanobis_distance()
{

 cv::Mat first_term = (KF.measurementMatrix*prediction-measurement);
 cv::Mat second_term = (KF.measurementMatrix*KF.errorCovPre*(KF.measurementMatrix).t()+KF.measurementNoiseCov);


 cv::Mat product = (first_term).t()*second_term.inv()*first_term;
 mahalanobis =sqrt(product.at<double>(0));
 cout<<"^^^^^^^^^^^^^^^^^^^mahalanobis:"<<mahalanobis<<endl;



}




trackdim::~trackdim() {
	// TODO Auto-generated destructor stub
}
