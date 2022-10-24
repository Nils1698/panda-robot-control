#include <ros/ros.h>
#include <stdlib.h>

#include <franka_msgs/FrankaState.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <sstream>
#include "std_msgs/String.h"

class PosePubHardcoded
{
    private:
        ros::NodeHandle nh;
        ros::Subscriber sub;
        ros::Subscriber subM;
        ros::Publisher pub;
        ros::Publisher pubM;

    public:
        int task = 0;
        std::string actionForController;


        PosePubHardcoded()
        {
            sub = nh.subscribe("/franka_state_controller/franka_states", 1000,&PosePubHardcoded::msgCallback, this);
            subM = nh.subscribe("/action2controller", 1000,&PosePubHardcoded::msgCallbackGripper, this);
            //pub = nh.advertise<geometry_msgs::PoseWithCovarianceStamped>("turtle1/cmd_vel", 10);
            pub = nh.advertise<geometry_msgs::PoseWithCovarianceStamped>("/desktop/target_pose_cov_stmp_panda_link0", 10);
            pubM = nh.advertise<std_msgs::String>("/action2gripper", 1);
        }

        void msgCallback(const franka_msgs::FrankaState& msg)
        {
            //Pick up from x: 0.45 y: 0.0 z: 0.286
            float currX = msg.O_T_EE[12];
            float currY = msg.O_T_EE[13];
            float currZ = msg.O_T_EE[14];

            //Wanted x,y,z pose
            float wantX;
            float wantY;
            float wantZ;
            //ROS_INFO("%s\n", actionForController.c_str());

            switch(task) {
                case 0:
                    pubPose(0.6, 0.0, 0.6, 0.0, 0.0, 1.0, 0.0);
                    if(isPoseReached(currX, currY, currZ, 0.6, 0.0, 0.6))
                    {
                        ROS_INFO("Task 0 finished.");
                        task = 1;
                    }
                    break;
                case 1:
                    pubPose(0.45, 0.0, 0.6, 0.0, 0.0, 1.0, 0.0);
                    if(isPoseReached(currX, currY, currZ, 0.45, 0.0, 0.6))
                    {
                        ROS_INFO("Task 1 finished.");
                        task = 2;
                    }
                    break;
                case 2:
                    pubPose(0.45, 0.0, 0.270, 0.0, 0.0, 1.0, 0.0);
                    if(isPoseReached(currX, currY, currZ, 0.45, 0.0, 0.270))
                    {
                        ROS_INFO("Task 2 finished.");
                        task = 3;
                    }
                    break;
                case 3:
                    pubMsg("Ready to grasp");
                    if (actionForController == "Object Grasped") {
                        ROS_INFO("Task 3 finished.");
                        task = 4;
                    }
                    break;
                case 4:
                    pubPose(0.45, 0.0, 0.40, 0.0, 0.0, 1.0, 0.0);
                    if(isPoseReached(currX, currY, currZ, 0.45, 0.0, 0.40))
                    {
                        ROS_INFO("Task 4 finished.");
                        task = 5;
                    }
                    break;
                case 5:
                    pubPose(0.45, 0.3, 0.40, 0.0, 0.0, 1.0, 0.0);
                    if(isPoseReached(currX, currY, currZ, 0.45, 0.3, 0.40))
                    {
                        ROS_INFO("Task 5 finished.");
                        task = 6;
                    }
                    break;
                case 6:
                    pubPose(0.45, 0.4, 0.4, 0.0, 0.0, 1.0, 0.0);
                    if(isPoseReached(currX, currY, currZ, 0.45, 0.4, 0.4))
                    {
                        ROS_INFO("Task 6 finished.");
                        task = 7;
                    }
                    break;
                case 7:
                    pubPose(0.45, 0.4, 0.27, 0.0, 0.0, 1.0, 0.0);
                    if(isPoseReached(currX, currY, currZ, 0.45, 0.4, 0.27))
                    {
                        ROS_INFO("Task 7 finished.");
                        task = 8;
                    }
                    break;
                case 8:
                    pubMsg("Open gripper");
                    if (actionForController == "Gripper opened") {
                        ROS_INFO("Task 8 finished.");
                        task = 999;
                    }
                    break;
                default:
                    pubMsg("Good job gripper... We did it!");
                    pubPose(0.6, 0.0, 0.6, 0.0, 0.0, 1.0, 0.0);
            }


        }

        bool isPoseReached(float currX, float currY, float currZ, float wantX, float wantY, float wantZ)
        {
            float allowedError = 0.03;
            if((currX < wantX+allowedError && currX > wantX-allowedError) && (currY < wantY+allowedError && currY > wantY-allowedError) && (currZ < wantZ+allowedError && currZ > wantZ-allowedError))
            {
                return true;
            } else 
            {
                return false;
            }
        }

        void pubMsg(std::string action) 
        {
            //Initialization
            std_msgs::String act;
            
            act.data = action;
            
            //Publish Hardcoded pose
            pubM.publish(act);
        }

        void pubPose(double px, double py, double pz, double ox, double oy, double oz, double ow) 
        {
            //Initialization
            geometry_msgs::PoseWithCovarianceStamped msg;
            
            msg.pose.pose.position.x  = px;
            msg.pose.pose.position.y  = py;
            msg.pose.pose.position.z  = pz;
            msg.pose.pose.orientation.x = ox;
            msg.pose.pose.orientation.y = oy;
            msg.pose.pose.orientation.z = oz;
            
            //ROS_INFO_STREAM("Sending random velocity command:"<<" linear="<<msg.pose.pose.position.x<<" angular="<<msg.pose.pose.orientation.x);

            //Publish Hardcoded pose
            pub.publish(msg);
        }

        void msgCallbackGripper(const std_msgs::String& act)
        {
            actionForController = act.data;
            //Pick up from x: 0.45 y: 0.0 z: 0.286
            //ROS_INFO("%s\n", act.data.c_str());

        }

};

int main(int argc, char**argv)
{
    ros::init(argc, argv, "publish_hardcoded_coordinates");
    PosePubHardcoded pph;
    ros::spin(); // Run until interupted 
    return 0;
};