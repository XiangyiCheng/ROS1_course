#include <ros/ros.h>
#include <std_msgs/Float64.h>
#include <math.h>

int main(int argc, char **argv) {
    ros::init(argc, argv, "vel_publisher1"); // name of this node will be "vel_cmd_publisher"
    ros::NodeHandle n; // two lines to create a publisher object that can talk to ROS
    ros::Publisher my_publisher_object = n.advertise<std_msgs::Float64>("/Three_DOF_robot/joint1_position_controller/command", 1);
    ros::Publisher my_publisher_object2 = n.advertise<std_msgs::Float64>("/Three_DOF_robot/joint2_position_controller/command", 1);
    ros::Publisher my_publisher_object3 = n.advertise<std_msgs::Float64>("/Three_DOF_robot/joint3_position_controller/command", 1);
    //"vel_cmd" is the name of the topic to which we will publish
    // the "1" argument says to use a buffer size of 1; could make larger, if expect network backups
    
   
    // as defined in: /opt/ros/indigo/share/std_msgs
    // any message published on a ROS topic must have a pre-defined format, 
    // so subscribers know how to interpret the serialized data transmission
   std_msgs::Float64 time;
   std_msgs::Float64 amplitude;
   std_msgs::Float64 frequency;
   std_msgs::Float64 amplitude2;
   std_msgs::Float64 frequency2;
   std_msgs::Float64 amplitude3;
   std_msgs::Float64 frequency3;
   std_msgs::Float64 pos_cmd;
   std_msgs::Float64 pos_cmd2;
   std_msgs::Float64 pos_cmd3;


   double dt_cmd_pub =0.1;
   double sample_rate = 1.0 / dt_cmd_pub; // compute the corresponding update frequency 
   ros::Rate naptime(sample_rate);
  

   time.data=0.0;
   amplitude.data=1;
   frequency.data=1;
   amplitude2.data=2;
   frequency2.data=2;
   amplitude3.data=3;
   frequency3.data=3;
    // do work here in infinite loop (desired for this example), but terminate if detect ROS has faulted
    while (ros::ok()) 
    {
        // this loop has no sleep timer, and thus it will consume excessive CPU time
        // expect one core to be 100% dedicated (wastefully) to this small task
        time.data = time.data + 0.01; //increment by 0.01 each iteration
        pos_cmd.data=amplitude.data*sin(2*3.1415926*frequency.data*time.data);
        pos_cmd2.data=amplitude2.data*sin(2*3.1415926*frequency2.data*time.data);
        pos_cmd3.data=amplitude3.data*sin(2*3.1415926*frequency3.data*time.data);
        my_publisher_object.publish(pos_cmd); // publish the value--of type Float64-- 
        my_publisher_object2.publish(pos_cmd2);
        my_publisher_object3.publish(pos_cmd3);
        //to the topic "topic1vel_cmd"
	//the next line will cause the loop to sleep for the balance of the desired period 
        // to achieve the specified loop frequency 
	naptime.sleep(); 
    }
}
