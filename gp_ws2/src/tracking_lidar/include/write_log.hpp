#include <ros/ros.h>
#include <iostream>
#include <fstream>
#include <string>
#include "tracking_lidar/Statusdim.h"
#include "trackdim.h"
#include "udp_server/status_timestamped.h"



using namespace std;

class WriteLog {

private:
  string myfilename;
 
 // what else

public:
    
    

    WriteLog();
    // functions
 
    // function for log_track
    
   void write_log_track(string file_name, tracking_lidar::Statusdim pre, tracking_lidar::Statusdim est, udp_server::status_timestamped cat_status);
 
    // function for log_pose
 
    void write_log_pos(string file_name, float centx, float centy);
    

    ~WriteLog();


};
