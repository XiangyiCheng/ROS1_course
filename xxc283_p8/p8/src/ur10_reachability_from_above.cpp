// reachability_from_above.cpp
// wsn, September 2016
// compute reachability, w/ z_tool_des pointing down;
// distance from gripper frame to flange frame is 0.1577
// cafe table is 0.79m above ground plane

// w/ Baxter on pedestal, torso is 0.93m above ground plane
// therefore, expect when fingers touch table, flange is at 0.79+0.1577 
// i.e., 0.0177m above torso
// search for reachability for flange over x_range = [0.4,1] , y_range= [-1,1] at z_range =[0,0.1]

#include <ur_fk_ik/ur_kin.h> 
#include <fstream>
using namespace std;
typedef Eigen::Matrix<double, 6, 1> Vectorq6x1;
//typedef Eigen::Matrix<double, 7, 1> Vectorq7x1;

int main(int argc, char **argv) {
    ros::init(argc, argv, "ur_reachability");
    Eigen::Vector3d p;
    Eigen::Vector3d n_des,t_des,b_des;
    Vectorq6x1 q_in;
    q_in<<0,0,0,0,0,0;

    UR10FwdSolver ur10fwdsolver;
    UR10IkSolver  ur10ikslover;
    //Baxter_fwd_solver baxter_fwd_solver;
    //Baxter_IK_solver baxter_ik_solver;

    b_des<<0,0,-1; //tool flange pointing down
    n_des<<0,0,1; //x-axis pointing forward...arbitrary
    t_des = b_des.cross(n_des); //consistent right-hand frame
    
    Eigen::Matrix3d R_des;
    R_des.col(0) = n_des;
    R_des.col(1) = t_des;
    R_des.col(2) = b_des;
    

    //std::vector<Vectorq7x1> q_ik_solns;

 vector<Eigen::VectorXd> q_ik_solns;
    Vectorq6x1 qsoln;

    Eigen::Affine3d a_tool_des; // expressed in DH frame  
    a_tool_des.linear() = R_des;
    //a_tool_des.translation() << x_des,0,0;
    double x_des,y_des,z_des;
    double x_min = -1.5;
    double x_max = 1.5;
    double y_min = -1.5;
    double y_max = 1.5;

    double arm_base =1.099893;
    double z_ground = 0.0-arm_base;
    double z_bin = 0.724275-arm_base;
    double z_tray = 0.950316-arm_base;
    double z_conveyor = 0.903960-arm_base;
    double z_AGV = 0.750201-arm_base;
    double dx = 0.05;
    double dy = 0.05;
    double height[5]= {z_ground,z_bin,z_tray,z_conveyor,z_AGV};
    Eigen::Vector3d p_des;
    int nsolns;
    std::vector<Eigen::Vector3d> reachable1, reachable2,reachable3, reachable4,reachable5;
       
for (int i=0;i<5;i++)
{
        for (double x_des = x_min;x_des<x_max;x_des+=dx) {
            for (double y_des = y_min; y_des<y_max; y_des+=dy) {
                p_des[0] = x_des;
                p_des[1] = y_des;
                p_des[2] = height[i]; // test grasp pose; //z_high; //test approach pose
                a_tool_des.translation() = p_des;

                nsolns=ur10ikslover.ik_solve(a_tool_des,q_ik_solns);
                //nsolns = baxter_ik_solver.ik_solve_approx_wrt_torso(a_tool_des, q_solns);
             // nsolns=0;
                if (nsolns>0) { //test grasp pose:
                        ROS_INFO("soln at x,y = %f, %f",p_des[0],p_des[1]);
if (i==0){
reachable1.push_back(p_des);
}

if (i==1)
{
reachable2.push_back(p_des);
}
if (i==2)
{
reachable3.push_back(p_des);
}
if (i==3)
{
reachable4.push_back(p_des);
}
if (i==4)
{
reachable5.push_back(p_des);
}

/*
switch (i){
case '0': 
reachable1.push_back(p_des);
break;
case '1': 
reachable2.push_back(p_des);
break;
case '2': 
reachable3.push_back(p_des);
break;
case '3': 
reachable4.push_back(p_des);
break;
case '4': 
reachable5.push_back(p_des);
break;
}
 */                       
                }
               
                
            }
        }
}
    ROS_INFO("saving the results...");
nsolns = reachable1.size();
    ofstream outfile;
    outfile.open("ur_reachable1_x_y"); 
    for (int i=0;i<nsolns;i++) {
        p_des = reachable1[i];
        outfile<<p_des[0]<<", "<<p_des[1]<<endl;
    }
    outfile.close();

    nsolns = reachable2.size();
    ofstream outfile2;
    outfile2.open("ur_reachable2_x_y");
    for (int i=0;i<nsolns;i++) {
        p_des = reachable2[i];
        outfile2<<p_des[0]<<", "<<p_des[1]<<endl;
    }
    outfile2.close();

    nsolns = reachable3.size();
    ofstream outfile3;
    outfile3.open("ur_reachable3_x_y");
    for (int i=0;i<nsolns;i++) {
        p_des = reachable3[i];
        outfile3<<p_des[0]<<", "<<p_des[1]<<endl;
    }
    outfile3.close();

    nsolns = reachable4.size();
    ofstream outfile4;
    outfile4.open("ur_reachable4_x_y");
    for (int i=0;i<nsolns;i++) {
        p_des = reachable4[i];
        outfile4<<p_des[0]<<", "<<p_des[1]<<endl;
    }
    outfile4.close();

    nsolns = reachable5.size();
    ofstream outfile5;
    outfile5.open("ur_reachable5_x_y");
    for (int i=0;i<nsolns;i++) {
        p_des = reachable5[i];
        outfile5<<p_des[0]<<", "<<p_des[1]<<endl;
    }
    outfile5.close();
   
    return 0;
}
