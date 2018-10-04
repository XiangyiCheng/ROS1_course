#include <ros/ros.h>
#include <new_ps2/ServiceMsg.h>
#include <iostream>
#include <string>
#include <std_msgs/Float64.h>
#include <math.h>

   std_msgs::Float64 g_time;
   std_msgs::Float64 g_amplitude;
   std_msgs::Float64 g_frequency;
   std_msgs::Float64 vel_cmd;
using namespace std;

bool callbackcommandedvelocity(new_ps2::ServiceMsgRequest& request, new_ps2::ServiceMsgResponse& response)
{
    ROS_INFO("callback callback activated");

       
       g_amplitude.data=request.amplitude;
       g_frequency.data=request.frequency;
       

       return g_amplitude.data;
       return g_frequency.data;

}

int main(int argc, char **argv) {
    ros::init(argc, argv, "vel_cmd_service"); // name of this node will be "vel_cmd_publisher"
    ros::NodeHandle n; // two lines to create a publisher object that can talk to ROS
    ros::Publisher my_publisher_object = n.advertise<std_msgs::Float64>("vel_cmd", 1);
    ros::ServiceServer service = n.advertiseService("vel_cmd_service", callbackcommandedvelocity);
    //"vel_cmd" is the name of the topic to which we will publish
    // the "1" argument says to use a buffer size of 1; could make larger, if expect network backups
    
  //  std_msgs::Float64 sin_vel; //create a variable of type "sin_vel", 
    // as defined in: /opt/ros/indigo/share/std_msgs
    // any message published on a ROS topic must have a pre-defined format, 
    // so subscribers know how to interpret the serialized data transmission
   

   double dt_cmd_pub =0.1;
   double sample_rate = 1.0 / dt_cmd_pub; // compute the corresponding update frequency 
   ros::Rate naptime(sample_rate);
  

   g_time.data=0.0;
   g_amplitude.data=0.0;
   g_frequency.data=0.0;
    // do work here in infinite loop (desired for this example), but terminate if detect ROS has faulted
    while (ros::ok()) 
    {
        // this loop has no sleep timer, and thus it will consume excessive CPU time
        // expect one core to be 100% dedicated (wastefully) to this small task
        g_time.data = g_time.data + 0.01; //increment by 0.01 each iteration
        vel_cmd.data=g_amplitude.data*sin(2*3.1415926*g_frequency.data*g_time.data);
        ROS_INFO("velocity command = %f", vel_cmd.data);
        my_publisher_object.publish(vel_cmd); // publish the value--of type Float64-- 
        //to the topic "topic1vel_cmd"
	//the next line will cause the loop to sleep for the balance of the desired period 
        // to achieve the specified loop frequency 
        ros::spinOnce();
	naptime.sleep(); 
    }
    return 0;
}
