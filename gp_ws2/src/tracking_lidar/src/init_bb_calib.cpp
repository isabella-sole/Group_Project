#include<init_bb_calib.hpp>



InitBbCalib::InitBbCalib() {

  prefix = "trackin_lidar/";
  calib_prefix = "calib_";
  prefix2 = "trackin_lidar/";
  bb_prefix = "bb_";
}


void InitBbCalib::init_calib_matrix (ros::NodeHandle nh) {

  nh.getParam(prefix+"calib_row", r);
	nh.getParam(prefix+"calib_coloumn", c);

  calib.resize(r,c);

	for(int i=0 ; i<r; i++) {
		for (int j=0;  j<c ; j++) {

				string calibname = prefix+calib_prefix+to_string(i)+to_string(j);
				nh.getParam(calibname, calib(i,j));

		}

  }

}


void InitBbCalib::init_bb_matrix (ros::NodeHandle nh) {

  nh.getParam(prefix2+"bb_row", r2);
	nh.getParam(prefix2+"bb_coloumn", c2);

 bounding_box.resize(r2,c2);
 cout<<"r2"<<r2<<endl;
 cout<<"c2"<<c2<<endl;



	for(int i=0 ; i<r2; i++) {

		for (int j=0;  j<c2 ; j++) {

				string bbname = prefix2+bb_prefix+to_string(i)+to_string(j);
				nh.getParam(bbname, bounding_box(i,j));

		}

	}

}



InitBbCalib::~InitBbCalib() {
}
