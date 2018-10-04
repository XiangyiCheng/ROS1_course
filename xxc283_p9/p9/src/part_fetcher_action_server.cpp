#include<ros/ros.h>
#include <actionlib/client/simple_action_client.h>
#include <actionlib/server/simple_action_server.h>
#include <actionlib/client/terminal_state.h>
#include <object_grabber/object_grabberAction.h>
#include<part_fetcher/PartFetcherAction.h>
#include <Eigen/Eigen>
#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <xform_utils/xform_utils.h>
#include <object_manipulation_properties/object_ID_codes.h>
#include<generic_gripper_services/genericGripperInterface.h>

using namespace std;
XformUtils xformUtils; //type conversion utilities

geometry_msgs::PoseStamped g_object_pickup_poseStamped;
geometry_msgs::PoseStamped g_object_dropoff_poseStamped;
int g_object_ID;
bool g_received_order=false;

class PartFetcherActionServer {
private:

    ros::NodeHandle nh_;  // we'll need a node handle; get one upon instantiation

    // this class will own a "SimpleActionServer" called "as_".
    // it will communicate using messages defined in example_action_server/action/demo.action
    // the type "demoAction" is auto-generated from our name "demo" and generic name "Action"
    actionlib::SimpleActionServer<part_fetcher::PartFetcherAction> as_;
    
    // here are some message types to communicate with our client(s)
    part_fetcher::PartFetcherGoal goal_; // goal message, received from client
    part_fetcher::PartFetcherResult result_; // put results here, to be sent back to the client when done w/ goal
    part_fetcher::PartFetcherFeedback feedback_; // not used in this example; 
    // would need to use: as_.publishFeedback(feedback_); to send incremental feedback to the client



public:
    PartFetcherActionServer(); //define the body of the constructor outside of class definition

    ~PartFetcherActionServer(void) {
    }
    // Action Interface
    int executeCB(const actionlib::SimpleActionServer<part_fetcher::PartFetcherAction>::GoalConstPtr& goal);
};


PartFetcherActionServer::PartFetcherActionServer() :
   as_(nh_, "part_fetcher", boost::bind(&PartFetcherActionServer::executeCB, this, _1),false) 
{
    ROS_INFO("in constructor of PartFetcherActionServer...");
    // do any other desired initializations here...specific to your implementation

    as_.start(); //start the server running
}

int g_object_grabber_return_code;
void objectGrabberDoneCb(const actionlib::SimpleClientGoalState& state,
        const object_grabber::object_grabberResultConstPtr& result) {
    ROS_INFO(" objectGrabberDoneCb: server responded with state [%s]", state.toString().c_str());
    g_object_grabber_return_code = result->return_code;
    ROS_INFO("got result output = %d; ", g_object_grabber_return_code);
}


int PartFetcherActionServer::executeCB(const actionlib::SimpleActionServer<part_fetcher::PartFetcherAction>::GoalConstPtr& goal) {
    
     g_object_pickup_poseStamped = goal->object_frame;
     g_object_dropoff_poseStamped = goal->desired_frame;
     g_object_ID = goal->object_id;
     ROS_INFO("requested fetch of part_id = %d from location ",g_object_ID);
     xformUtils.printStampedPose(g_object_pickup_poseStamped);
     ROS_INFO("requested dropoff at pose: ");
     xformUtils.printStampedPose(g_object_dropoff_poseStamped);
     g_received_order=true; //set flag that new order has been received   
 
    actionlib::SimpleActionClient<object_grabber::object_grabberAction> object_grabber_ac("object_grabber_action_service", true);
ROS_INFO("waiting for server: ");
    bool server_exists = false;
    while ((!server_exists)&&(ros::ok())) {
        server_exists = object_grabber_ac.waitForServer(ros::Duration(0.5)); // 
        ros::spinOnce();
        ros::Duration(0.5).sleep();
        ROS_INFO("retrying...");
    }
    ROS_INFO("connected to object_grabber action server"); 

bool finished_before_timeout;
object_grabber::object_grabberGoal object_grabber_goal;
    ROS_INFO("sending test code: ");
    object_grabber_goal.action_code = object_grabber::object_grabberGoal::TEST_CODE;
    object_grabber_ac.sendGoal(object_grabber_goal, &objectGrabberDoneCb);
    finished_before_timeout = object_grabber_ac.waitForResult(ros::Duration(30.0));
    if (!finished_before_timeout) {
        ROS_WARN("giving up waiting on result ");
        return 1;
    }

 //move to waiting pose
    ROS_INFO("sending command to move to waiting pose");
    object_grabber_goal.action_code = object_grabber::object_grabberGoal::MOVE_TO_WAITING_POSE;
    object_grabber_ac.sendGoal(object_grabber_goal, &objectGrabberDoneCb);
    finished_before_timeout = object_grabber_ac.waitForResult(ros::Duration(30.0));
    if (!finished_before_timeout) {
        ROS_WARN("giving up waiting on result ");
        return 1;
    }

ROS_INFO("sending a grab-object command");
    object_grabber_goal.action_code = object_grabber::object_grabberGoal::GRAB_OBJECT; //specify the action to be performed 
    object_grabber_goal.object_id = g_object_ID; // specify the object to manipulate                
    object_grabber_goal.object_frame = g_object_pickup_poseStamped; //and the object's current pose
    object_grabber_goal.grasp_option = object_grabber::object_grabberGoal::DEFAULT_GRASP_STRATEGY; //from above
    object_grabber_goal.speed_factor = 1.0;
    ROS_INFO("sending goal to grab object: ");
    object_grabber_ac.sendGoal(object_grabber_goal, &objectGrabberDoneCb);
    ROS_INFO("waiting on result");
    finished_before_timeout = object_grabber_ac.waitForResult(ros::Duration(30.0));

    if (!finished_before_timeout) {
        ROS_WARN("giving up waiting on result ");
        return 1;
    }

 //move to waiting pose
    ROS_INFO("sending command to move to waiting pose");
    object_grabber_goal.action_code = object_grabber::object_grabberGoal::MOVE_TO_WAITING_POSE;
    object_grabber_ac.sendGoal(object_grabber_goal, &objectGrabberDoneCb);
    finished_before_timeout = object_grabber_ac.waitForResult(ros::Duration(30.0));
    if (!finished_before_timeout) {
        ROS_WARN("giving up waiting on result ");
        return 1;
    }
    
  ROS_INFO("sending a dropoff-object command");
    object_grabber_goal.action_code = object_grabber::object_grabberGoal::DROPOFF_OBJECT; //specify the action to be performed 
    object_grabber_goal.object_id = g_object_ID; // specify the object to manipulate                
    object_grabber_goal.object_frame = g_object_dropoff_poseStamped; //and the object's current pose
    object_grabber_goal.grasp_option = object_grabber::object_grabberGoal::DEFAULT_GRASP_STRATEGY; //from above
    object_grabber_goal.speed_factor = 1.0;
    ROS_INFO("sending goal to dropoff object: ");
    object_grabber_ac.sendGoal(object_grabber_goal, &objectGrabberDoneCb);
    ROS_INFO("waiting on result");
    finished_before_timeout = object_grabber_ac.waitForResult(ros::Duration(30.0));

    if (!finished_before_timeout) {
        ROS_WARN("giving up waiting on result ");
        return 1;
    }

    result_.rtn_code=0; 
    as_.setSucceeded(result_); 
   
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "part_fetcher_action_server1"); // name this node 

    ROS_INFO("instantiating the part-fetcher action server: ");

    PartFetcherActionServer as_object; 
    ROS_INFO("going into spin");

    while (ros::ok()) {
        ros::spinOnce(); 
        ros::Duration(0.1).sleep();
        if (g_received_order) {
         ROS_WARN("main: received new order; should act on it!");
         g_received_order=false;
        }
    }

    return 0;
}

