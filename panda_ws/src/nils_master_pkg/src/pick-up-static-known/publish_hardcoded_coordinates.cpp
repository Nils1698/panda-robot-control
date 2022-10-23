#include <ros/ros.h>
#include <stdlib.h>

#include <geometry_msgs/PoseWithCovarianceStamped.h>

class PosePubHardcoded
{
    private:
        ros::NodeHandle nh;
        ros::Publisher pub;

    public:
        PosePubHardcoded()
        {

            pub = nh.advertise<geometry_msgs::PoseWithCovarianceStamped>("turtle1/cmd_vel", 10);

            //Begin of loop
            ros::Rate loop_rate(10);
            while(ros::ok)
                {
                    pubPose(1.2, 0.0, 0.1, 0.0, 0.0, 0.0, 1.0);
                    ros::spinOnce();        
                    loop_rate.sleep(); 
                }
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
};

int main(int argc, char**argv)
{
    ros::init(argc, argv, "publish_hardcoded_coordinates");
    PosePubHardcoded pph;
    ros::spin(); // Run until interupted 
    return 0;
};