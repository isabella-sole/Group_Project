#include <read_json.hpp>

ReadJson::ReadJson() {
    prefix = "trackin_lidar/";
}



 void ReadJson::read_Json_bb(ros::NodeHandle nh){

     
     // open file json
     std::ifstream in;

     nh.getParam(prefix+"path", path); // take path from configuration file

    //cout << path << endl;
    in.open(path);
    // in.open("/home/mina/catkin_ws2/src/tracking_lidar/json_files/record2_test.json");

    if( !in.is_open())
    {
        cout << "Error opening files \n"<<endl;
        return;
    }

    Json::Reader reader; // to parse json
    Json::Value root; // can represent any type

    if(reader.parse(in, root))
    {
        cout << root.size() << endl;


        for(int i = 0; i < root.size(); i++){
            std::vector<float> bbox;
            
            // read root node information
            image_id_vector.push_back(root[i]["image_id"].asInt());

            category_id_vector.push_back(root[i]["category_id"].asInt());

            id_vector.push_back(root[i]["id"].asInt());

            score_vector.push_back(root[i]["score"].asInt());

            
            for(unsigned int j=0; j < root[i]["bbox"].size(); j++)
            {
                
                float bbox_value = root[i]["bbox"][j].asFloat();
                bbox.push_back(bbox_value); 

                //cout << "Bbox " << bbox[j] << endl;
                
            }
            bbox_vector.push_back(bbox);

           /*
            cout << "Image ID " << image_id_vector[i] << endl;
            cout << "Category ID " << category_id_vector[i] << endl;
            cout << "ID " << id_vector[i] << endl;
            cout << "Score " << score_vector[i] << endl;
            cout << endl;

            cout << "Print matrix containing BBOX vectors" << endl;
            
            for (int i = 0; i < bbox_vector.size(); i++) { 
                for (int j = 0; j < bbox_vector[i].size(); j++) 
                    cout << bbox_vector[i][j] << " "; 
                cout << endl; 
            } */

        }  
    }

    in.close();

 }

 void ReadJson::compute_vertices_bb(ros::NodeHandle nh){

     // bbox = [x_ver, y_ver, larghezza, lunghezza ]
    nh.getParam(prefix+"bb_row", r2);
	nh.getParam(prefix+"bb_coloumn", c2);

    bounding_box.resize(r2,c2);


    for(int i = 0; i < bbox_vector.size(); i++){

        std::vector<float> bbox_i = bbox_vector[i];

        std::map<std::string, float> mapOfMarks = {
     
            {"bb_30" , bbox_i[0]},
            {"bb_31" , bbox_i[1]},

            { "bb_10" ,  bbox_i[0]+bbox_i[2]},
            { "bb_11" , bbox_i[1]-bbox_i[3]},

            { "bb_20" ,  bbox_i[0]+bbox_i[2]},
            { "bb_21" , bbox_i[1]},

            { "bb_00" ,  bbox_i[0]},
            { "bb_01" , bbox_i[1]-bbox_i[3]}

        };

        
        for(int k=0 ; k<r2; k++) {
            for (int j=0;  j<c2 ; j++) {
                string bbname = "bb_"+to_string(k)+to_string(j);

                bounding_box(k,j)= mapOfMarks[bbname];
                    
                    cout << "Kudret("<<k<<j<<") = "<<bounding_box(k,j)<< endl;
            }
        } 
    }

 }

ReadJson::~ReadJson() {


}