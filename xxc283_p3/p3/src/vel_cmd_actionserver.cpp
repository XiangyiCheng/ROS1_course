#include <ros/ros.h>
#include <std_msgs/Float64.h>
#include <math.h>
#include <actionlib/server/simple_action_server.h>
//the following #include refers to the "action" message defined for this package
// The action message can be found in: .../example_action_server/action/demo.action
// Automated header generation creates multiple headers for message I/O
// These are referred to by the root name (demo) and appended name (Action)
#include<p3/commandAction.h>

class ExampleActionServer {
private:

    ros::NodeHandle nh_;  // we'll need a node handle; get one upon instantiation

    // this class will own a "SimpleActionServer" called "as_".
    // it will communicate using messages defined in example_action_server/action/demo.action
    // the type "demoAction" is auto-generated from our name "demo" and generic name "Action"
    actionlib::SimpleActionServer<p3::commandAction> as_;
    
    // here are some message types to communicate with our client(s)
    p3::commandGoal goal_; // goal message, received from client
    p3::commandResult result_; // put results here, to be sent back to the client when done w/ goal
    p3::commandFeedback feedback_; // not used in this example; 
    // would need to use: as_.publishFeedback(feedback_); to send incremental feedback to the client



public:
    ExampleActionServer(); //define the body of the constructor outside of class definition

    ~ExampleActionServer(void) {
    }
    // Action Interface
    void executeCB(const actionlib::SimpleActionServer<p3::commandAction>::GoalConstPtr& goal);
};

//implementation of the constructor:
// member initialization list describes how to initialize member as_
// member as_ will get instantiated with specified node-handle, name by which this server will be known,
//  a pointer to the function to be executed upon receipt of a goal.
//  
// Syntax of naming the function to be invoked: get a pointer to the function, called executeCB, which is a member method
// of our class exampleActionServer.  Since this is a class method, we need to tell boost::bind that it is a class member,
// using the "this" keyword.  the _1 argument says that our executeCB takes one argument
// the final argument  "false" says don't start the server yet.  (We'll do this in the constructor)

ExampleActionServer::ExampleActionServer() :
   as_(nh_, "example_action", boost::bind(&ExampleActionServer::executeCB, this, _1),false) 
// in the above initialization, we name the server "example_action"
//  clients will need to refer to this name to connect with this server
{
    ROS_INFO("in constructor of exampleActionServer...");
    // do any other desired initializations here...specific to your implementation

    as_.start(); //start the server running
}

void ExampleActionServer::executeCB(const actionlib::SimpleActionServer<p3::commandAction>::GoalConstPtr& goal) {
   ros::NodeHandle n;
   ros::Publisher my_publisher_object = n.advertise<std_msgs::Float64>("vel_cmd", 1);
   std_msgs::Float64 time;
   std_msgs::Float64 in_amplitude;
   std_msgs::Float64 in_frequency;
   std_msgs::Float64 in_cycle_number;
   std_msgs::Float64 end_time;
   std_msgs::Float64 vel_cmd;

   double dt_cmd_pub =0.1;
   double sample_rate = 1.0 / dt_cmd_pub; // compute the corresponding update frequency 
   ros::Rate naptime(sample_rate);
  

   time.data=0.0;
   in_amplitude.data=goal->amplitude;
   in_frequency.data=goal->frequency;
   in_cycle_number.data=goal->cycle_number;
   end_time.data=in_cycle_number.data/in_frequency.data;

 while (time.data<=end_time.data) 
{
   time.data = time.data + 0.01; //increment by 0.01 each iteration
  // ROS_INFO("goal amplitude is: %d", goal->amplitude);
  // ROS_INFO("goal frequency is: %d", goal->frequency);

   vel_cmd.data=in_amplitude.data*sin(2*3.1415926*in_frequency.data*time.data);
   ROS_INFO("velocity command = %f", vel_cmd.data);
   result_.vel_cmd_value= vel_cmd.data;
   feedback_.fdbk = vel_cmd.data; // populate feedback message with current countdown value
   as_.publishFeedback(feedback_); // send feedback to the action client that requested this goal
   naptime.sleep(); 
   my_publisher_object.publish(vel_cmd);
}
   result_.vel_cmd_value = 0.0; //value should be zero, if completed countdown
   as_.setSucceeded(result_); 
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "vel_cmd_actionserver"); 
     // two lines to create a publisher object that can talk to ROS
    
    ExampleActionServer as_object; 
    ROS_INFO("going into spin");
    //"vel_cmd" is the name of the topic to which we will publish
    // the "1" argument says to use a buffer size of 1; could make larger, if expect network backups
    
   
    // as defined in: /opt/ros/indigo/share/std_msgs
    // any message published on a ROS topic must have a pre-defined format, 
    // so subscribers know how to interpret the serialized data transmission
  
    // do work here in infinite loop (desired for this example), but terminate if detect ROS has faulted
   
        ros::spin(); 
        // this loop has no sleep timer, and thus it will consume excessive CPU time
        // expect one core to be 100% dedicated (wastefully) to this small task
        
    
 return 0;
}
