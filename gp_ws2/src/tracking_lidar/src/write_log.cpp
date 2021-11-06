#include <write_log.hpp>


WriteLog::WriteLog() {
 // how to initialize
}

/* HOW TO WRITE DATA ON TXT FILE

Libraries to be included:
 #include <iostream>
 #include <string>
 #include <fstream>

- define the type of data to be written into the file
 
- Open/create the new file for writing
 //open file for writing
 ofstream fw("c:\\demos\\CPlusPlusSampleFile.txt", std::ofstream::out);

- Write the data
 //check if file was successfully opened for writing
 if (fw.is_open())
 {
  //store array contents to text file
  for (int i = 0; i < arraySize; i++) {
  fw << vehiclesList[i] << "\n";
 }
 fw.close();
 }
 else cout << "Problem with opening file";
 
- Close the file
 //close the file after the writing operation is completed
 fw.close();
 
*/

/***************+ LOG TRACK *******************/
 void WriteLog::write_log_track(string file_name, tracking_lidar::Statusdim pre, tracking_lidar::Statusdim est,udp_server::status_timestamped cat_status){
 //void WriteLog::write_log_track(string name_file, tracking_lidar::Statusdim pre, tracking_lidar::Statusdim est, udp_server::status_timestamped cat_status){
 
 //check if file was successfully opened for writing
 // da fare inserendo controllo if
 
  ofstream myfile(file_name+".txt"); // in mina era myfile.open ("../log_track.txt");
 
 // mina writing
 myfile << "prediction                          estimation .\n";
 myfile << "x,y,vx,vy,                          x,y,vx,vy  .\n";
 
 // ***** WRITE ON LOG_TRACK *****

 myfile << pre.x << "," << pre.y << "," << pre.vx << "," << pre.vy
   << "," << est.x << "," << est.y << "," << est.vx << ","
   << est.vy << "," << cat_status.latitude << ","
   << cat_status.longitude << "," << cat_status.altitude << ","
   << cat_status.speed[0] << "," << cat_status.speed[1] << ","
   << cat_status.orientation.yaw << ","
   << cat_status.orientation.pitch << ","
   << cat_status.orientation.roll <<","
   << est.w <<","<< est.l <<std::setprecision(12)<<endl; 
 
 myfile.close();
 
}



void WriteLog::write_log_pos(string file_name,float centx, float centy){
 
 //check if file was successfully opened for writing
 // da fare inserendo controllo if
 ofstream myfile(file_name+".txt"); // in mina era myfile.open ("../log_track.txt");
 
 // ***** WRITE ON LOG_POS *****
 myfile <<  centx <<","<<centy<<std::setprecision(12)<<endl;
 
 myfile.close();
}


WriteLog::~WriteLog() {
}
