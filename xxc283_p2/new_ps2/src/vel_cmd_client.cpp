//example ROS client:
// first run: rosrun example_ROS_service example_ROS_service
// then start this node:  rosrun example_ROS_service example_ROS_client



#include <ros/ros.h>
#include <new_ps2/ServiceMsg.h>
#include <iostream>
#include <string>
#include <std_msgs/Float64.h>
#include <math.h>
using namespace std;

int main(int argc, char **argv) {
    ros::init(argc, argv, "vel_cmd_client");
    ros::NodeHandle n;
    ros::ServiceClient client = n.serviceClient<new_ps2::ServiceMsg>("vel_cmd_service");
    new_ps2::ServiceMsg srv;
    //bool found_on_list = false;
    //string in_name;
    std_msgs::Float64 in_frequency;
    std_msgs::Float64 in_amplitude;
    while (ros::ok()) {
        cout<<endl;
        cout << "enter the frequency"<< endl;
        cin>>in_frequency.data;
        cout << "enter the amplitude"<< endl;
        cin>>in_amplitude.data;
        srv.request.frequency = in_frequency.data; 
        srv.request.amplitude = in_amplitude.data;
       if (client.call(srv)) {
            
                cout << "Secceed to call the service"<< endl;
                
        } else {
            ROS_ERROR("Failed to call service");
         return 1;
        }
    }
    return 0;
}
