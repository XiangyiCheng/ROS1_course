//example ROS client:
// first run: rosrun example_ROS_service example_ROS_service
// then start this node:  rosrun example_ROS_service example_ROS_client



#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h>
#include <iostream>
#include <string>
#include <std_msgs/Float64.h>
#include <math.h>
#include <p3/commandAction.h>
using namespace std;
bool g_goal_active = false; 
float g_result_output = 0.0;
float g_fdbk = 0.0;

void doneCb(const actionlib::SimpleClientGoalState& state,
        const p3::commandResultConstPtr& result) {
 //   ROS_INFO(" doneCb: server responded with state [%s]", state.toString().c_str());
    ROS_INFO("got result vel_cmd = %f",result->vel_cmd_value);
    ROS_INFO("goal succeed");
    g_result_output= result->vel_cmd_value;
    g_goal_active=false;
}

void feedbackCb(const p3::commandFeedbackConstPtr& fdbk_msg) {
    ROS_INFO("feedback status (commanded velocity) = %f",fdbk_msg->fdbk);
    g_fdbk = fdbk_msg->fdbk; //make status available to "main()"
}

void activeCb()
{
  ROS_INFO("Goal just went active");
  g_goal_active=true; //let main() know that the server responded that this goal is in process
}


int main(int argc, char **argv) {
    ros::init(argc, argv, "vel_cmd_actionclient");

     p3::commandGoal goal; 
     actionlib::SimpleActionClient<p3::commandAction> action_client("example_action", true);
        
    std_msgs::Float64 get_frequency;
    std_msgs::Float64 get_amplitude;
    std_msgs::Float64 get_cycle_number;
    ROS_INFO("waiting for server: ");
    bool server_exists = action_client.waitForServer(ros::Duration(7.0)); // wait for up to 5 seconds
        // something odd in above: does not seem to wait for 5 seconds, but returns rapidly if server not running
        //bool server_exists = action_client.waitForServer(); //wait forever

    if (!server_exists)
       {
            ROS_WARN("could not connect to server; halting");
            return 0; // bail out; optionally, could print a warning message and retry
        }
        
       
        ROS_INFO("connected to action server"); 
    while (true)
    {
        cout<<endl;
        cout << "enter the frequency"<< endl;
        cin>>get_frequency.data;
        cout << "enter the amplitude"<< endl;
        cin>>get_amplitude.data;
        cout << "enter the cycle number"<< endl;
        cin>>get_cycle_number.data;
        goal.frequency = get_frequency.data; 
        goal.amplitude = get_amplitude.data;
        goal.cycle_number = get_cycle_number.data;
        action_client.sendGoal(goal,&doneCb,&activeCb,&feedbackCb);

    }
    return 0;
}
