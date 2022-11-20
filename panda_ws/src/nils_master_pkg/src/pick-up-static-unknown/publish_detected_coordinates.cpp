#include <ros/ros.h>
#include <stdlib.h>
#include<unistd.h>
#include <franka_msgs/FrankaState.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <tf2/LinearMath/Quaternion.h>
#include <sstream>
#include "std_msgs/String.h"
#include <math.h>

class PosePubDetected
{
    private:
        ros::NodeHandle nh;
        ros::Subscriber sub;
        ros::Subscriber subM;
        ros::Subscriber subG;
        ros::Subscriber subCoor;
        ros::Publisher pub;
        ros::Publisher pubM;
        ros::Publisher pubG;

    public:
        int task = 0;
        int numObj = 0;

        //Wanted x,y,z pose
        float wantX;
        float wantY;
        float wantZ;
        float wantOZ;
        float gox;
        float goy;
        float goz;  
        bool coorSaved = false;
        std::string cameraToController;
        std::string gripperToController;
        tf2::Quaternion myQuaternion;

        PosePubDetected()
        {
            sub = nh.subscribe("/franka_state_controller/franka_states", 1000,&PosePubDetected::msgCallback, this);
            subM = nh.subscribe("/camera2controller", 1000,&PosePubDetected::msgCallbackCamera, this);
            subG = nh.subscribe("/gripper2controller", 1000,&PosePubDetected::msgCallbackGripper, this);
            subCoor = nh.subscribe("/objCoordinates", 1000,&PosePubDetected::msgCallbackCoor, this);
            //pub = nh.advertise<geometry_msgs::PoseWithCovarianceStamped>("turtle1/cmd_vel", 10);
            pub = nh.advertise<geometry_msgs::PoseWithCovarianceStamped>("/desktop/target_pose_cov_stmp_panda_link0", 10);
            pubM = nh.advertise<std_msgs::String>("/controller2camera", 10);
            pubG = nh.advertise<std_msgs::String>("/controller2gripper", 10);
        }

        void msgCallback(const franka_msgs::FrankaState& msg)
        {
            //Pick up from x: 0.45 y: 0.0 z: 0.286
            float currX = msg.O_T_EE[12];
            float currY = msg.O_T_EE[13];
            float currZ = msg.O_T_EE[14];

            float dropPos = 0.0 + 0.1 * numObj;

            //In case the object gets lost
            if (gripperToController == "Object lost") {
                task = 105;
            }
            

            switch(task) {
                case 0:
                    pubGripper("Are you ready?");
                    pubMsg("Are you ready?");
                    myQuaternion.setRPY( 0, 0, M_PI);
                    if ((cameraToController == "Ready!") && (gripperToController == "Ready!")) {
                        pubPose(0.2, 0.0, 0.7, myQuaternion.getX(), myQuaternion.getY(), myQuaternion.getZ(), myQuaternion.getW());//myQuaternion.getX(), myQuaternion.getY(), myQuaternion.getZ(), myQuaternion.getW());
                        if(isPoseReached(currX, currY, currZ, 0.2, 0.0, 0.7))
                        {
                            ROS_INFO("All nodes ready.");
                            ROS_INFO("Task 1 finished.");
                            task = 1;
                        }
                        break;
                    }
                    break;
                case 1:
                    pubMsg("Awaiting coordinates");
                    if (cameraToController == "No objects visible"){
                        pubPose(0.2, 0.0, 0.7, myQuaternion.getX(), myQuaternion.getY(), myQuaternion.getZ(), myQuaternion.getW()); //0.0, 0.0, 1.0, 0.0);
                        task = 1;
                    } 
                    else if (cameraToController == "Coordinates are being published...") {
                        ROS_WARN("Going to x=%f y=%f z=%f oz=%f", wantX, wantY, wantZ, wantOZ);
                        coorSaved = true;
                        myQuaternion.setRPY( 0, 0, wantOZ);
                        gox = wantX;
                        goy = wantY;
                        goz = wantZ;  
                        task = 2;
                    }
                    break;
                case 2:
                    if (!coorSaved) {
                        task = 1;
                    } else {
                        pubPose(gox, goy, 0.5, myQuaternion.getX(), myQuaternion.getY(), myQuaternion.getZ(), myQuaternion.getW());
                        if(isPoseReached(currX, currY, currZ, gox, goy, 0.5))
                        {
                            ROS_INFO("Task 2 finished.");
                            coorSaved = false;
                            task = 3;
                        }
                    }
                    break;
                case 3:
                    pubPose(gox, goy, 0.4, myQuaternion.getX(), myQuaternion.getY(), myQuaternion.getZ(), myQuaternion.getW());
                    if(isPoseReached(currX, currY, currZ, gox, goy, 0.4))
                    {
                        ROS_INFO("Task 3 finished.");
                        task = 4;
                    }
                    break;
                case 4:
                    pubPose(gox, goy, 0.3, myQuaternion.getX(), myQuaternion.getY(), myQuaternion.getZ(), myQuaternion.getW());
                    if(isPoseReached(currX, currY, currZ, gox, goy, 0.3))
                    {
                        ROS_INFO("Task 4 finished.");
                        task = 5;
                    }
                    break;
                case 5:
                    pubPose(gox, goy, 0.27, myQuaternion.getX(), myQuaternion.getY(), myQuaternion.getZ(), myQuaternion.getW());
                    if(isPoseReached(currX, currY, currZ, gox, goy, 0.27))
                    {
                        ROS_INFO("Task 5 finished.");
                        task = 6;
                    }
                    break;
                case 6:
                    pubGripper("Ready to grasp");
                    if (gripperToController == "Object Grasped") {
                        ROS_INFO("Task 6 finished.");
                        task = 7;
                    }
                    break;
                case 7:
                    pubGripper("Object grasped");
                    pubPose(gox, goy, 0.4, myQuaternion.getX(), myQuaternion.getY(), myQuaternion.getZ(), myQuaternion.getW());
                    if(isPoseReached(currX, currY, currZ, gox, goy, 0.4))
                    {
                        myQuaternion.setRPY( 0, 0, M_PI);
                        ROS_INFO("Task 7 finished.");
                        task = 8;
                    }
                    break;
                case 8:
                    pubGripper("Object grasped");
                    pubPose(0.7, dropPos, 0.4, myQuaternion.getX(), myQuaternion.getY(), myQuaternion.getZ(), myQuaternion.getW());
                    if(isPoseReached(currX, currY, currZ, 0.7, dropPos, 0.4))
                    {
                        ROS_INFO("Task 8 finished.");
                        task = 9;
                    }
                    break;
                case 9:
                    pubGripper("Object grasped");
                    pubPose(0.7, dropPos, 0.22, myQuaternion.getX(), myQuaternion.getY(), myQuaternion.getZ(), myQuaternion.getW());
                    if(isPoseReached(currX, currY, currZ, 0.7, dropPos, 0.22))
                    {
                        ROS_INFO("Task 9 finished.");
                        task = 10;
                    }
                    break;
                case 10:
                    pubGripper("Open gripper");
                    if (gripperToController == "Gripper opened") {
                        numObj = numObj + 1;
                        ROS_INFO("Task 10 finished.");
                        task = 11;
                    }
                    break;
                case 105:
                    pubGripper("Open gripper");
                    if (gripperToController == "Gripper opened") {
                        myQuaternion.setRPY( 0, 0, M_PI);
                        ROS_INFO("Task 105 finished.");
                        task = 115;
                    }
                    break;
                case 11:
                    pubPose(0.7, 0.0, 0.5, myQuaternion.getX(), myQuaternion.getY(), myQuaternion.getZ(), myQuaternion.getW());
                    if(isPoseReached(currX, currY, currZ, 0.7, 0.0, 0.5))
                    {
                        ROS_INFO("Task 11 finished.");
                        task = 12;
                    }
                    break;
                case 115:
                    pubPose(0.5, 0.0, 0.7, myQuaternion.getX(), myQuaternion.getY(), myQuaternion.getZ(), myQuaternion.getW());
                    if(isPoseReached(currX, currY, currZ, 0.5, 0.0, 0.7))
                    {
                        ROS_INFO("Task 115 finished.");
                        task = 12;
                    }
                    break;
                case 12:
                    pubPose(0.2, 0.0, 0.7, myQuaternion.getX(), myQuaternion.getY(), myQuaternion.getZ(), myQuaternion.getW());
                    if(isPoseReached(currX, currY, currZ, 0.2, 0.0, 0.7))
                    {
                        ROS_INFO("Task 12 finished.");
                        usleep(200000);
                        task = 1;
                    }
                    break;
                default:
                    pubPose(0.2, 0.0, 0.6, myQuaternion.getX(), myQuaternion.getY(), myQuaternion.getZ(), myQuaternion.getW());
            }


        }

        bool isPoseReached(float currX, float currY, float currZ, float wantX, float wantY, float wantZ)
        {
            float allowedError = 0.01;
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

        void pubGripper(std::string actionGripper) 
        {
            //Initialization
            std_msgs::String actG;
            
            actG.data = actionGripper;
            
            //Publish Hardcoded pose
            pubG.publish(actG);
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
            msg.pose.pose.orientation.w = ow; //THIS IS THE REASON
            
            //ROS_INFO_STREAM("Sending random velocity command:"<<" linear="<<msg.pose.pose.position.x<<" angular="<<msg.pose.pose.orientation.x);

            //Publish Hardcoded pose
            pub.publish(msg);
        }

        void msgCallbackCamera(const std_msgs::String& act)
        {
            cameraToController = act.data;
        }

        void msgCallbackGripper(const std_msgs::String& actG)
        {
            gripperToController = actG.data;
        }

        void msgCallbackCoor(const geometry_msgs::Twist& msgCoor)
        {
            //Transformation to World coordinates
            wantX = msgCoor.linear.x + 0.363;
            wantY = (msgCoor.linear.y - 0.45) * 1.05;
            wantZ = msgCoor.linear.z + 0.27; 
            wantOZ = msgCoor.angular.z + M_PI;        
        }

};

int main(int argc, char**argv)
{
    ros::init(argc, argv, "publish_detected_coordinates");
    PosePubDetected ppd;
    ros::spin(); // Run until interupted 
    return 0;
};