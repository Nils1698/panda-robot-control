#include <memory>
#include <string>
#include <vector>
#include <ros/ros.h>
#include <std_msgs/Int64.h>
#include <std_srvs/SetBool.h>
// #include <Eigen/Core>
#include <Eigen/Core>
#include <unsupported/Eigen/MatrixFunctions>
#include <std_msgs/Float64MultiArray.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <geometry_msgs/PoseWithCovariance.h>

#include <predictor/PredictedPoses.h>

#include <string>
#include <iostream>
#include <sstream>

class Predictor {

    private:
        // Kalman Filter Initialization parameters
        const static int inputs     = 0;   // Predictor inputs
        const static int outputs    = 3;   // System outputs
        const static int horizon    = 60;  // Prediction horizon
        const static int pred_order = 2;   // Prediction order                             // position and velocity, for now. In future acceleration and jerk may also be predicted.
        const static int dimensions = 3;   // Number of axis along which we are predicting // We are predicting both along the x, y & z axis. We are, for now, not predicting the atitude states from the quaternion.
        const static int states     = 6;   // Predictor states                             // dimensions * pred_order 
        const double     Ts_pred    = 0.1; // Time between prediction generations
        const double     Ts_meas    = 0.2; // Time between measurements                    // As of right now 2022-03-18, there is no certainty in the frequency or regularity of the measurement. Last time "$ rostopic hz /target_pose_measurement_stamped" was used the frequency varied between 2 adn 5 hz. There may be a bottleneck in one of the loops in a node somewhere.
        bool new_measurement_acquired   = false;
        bool measurement_happended_at_last_iteration = false;
        bool first_measurement_callback = false;

        Eigen::Matrix<double, 3, 3> sigma_sqrt_max; // oxo | True Maximum sensor noise amplitude
        Eigen::Matrix<double, 3, 3> sigma_sqrt;     // oxo | True Variance of the measurement noise
        Eigen::Matrix<double, 3, 3> V2;             // oxo | Measurement Uncertainty
        Eigen::Matrix<double, 3, 1> y;              // ox1 | Measurement signal from subscription
        Eigen::Matrix<double, 4, 1> y_orientation;  // 4x1 | Measurement signal from subscription
        Eigen::Matrix<double, 6, 6> V1;             // ixi | State Uncertainty
        
        Eigen::Matrix<double, 6, 6> F;      // sxs TODO: If I find a way to have the dimensions of F be modular, replace the constants inserted here.
        Eigen::Matrix<double, 6, 6> F_N;    // sxs
        Eigen::Matrix<double, 6, 6> F_Ntsm; // sxs
        Eigen::Matrix<double, 6, 6> Qm;     // sxs Qm = Q⁻(k) and Qp = Q⁺(k)
        Eigen::Matrix<double, 6, 6> Qp;     // sxs Qm = Q⁻(k) and Qp = Q⁺(k)
        Eigen::Matrix<double, 6, 6> I6;     // sxs Identity matrix
        Eigen::Matrix<double, 6, 3> L;      // sxo xhat is the 
        Eigen::Matrix<double, 3, 6> C;      // oxs Model output matrix
        Eigen::Matrix<double, 6, 1> xhat;   // sx1
        Eigen::Matrix<double, 6, 1> G;      // sxi This one is zero and should be removed for optimization
        
        Eigen::Matrix<double,Eigen::Dynamic, Eigen::Dynamic> F_star, Qm_star;
        Eigen::VectorXd xhat_pred;
        // Publisher and subscriber handles
        ros::Publisher pub;
        ros::Subscriber measurement_subscriber;
        ros::ServiceServer reset_service;
        predictor::PredictedPoses predicted_poses; // prediction message for publishing
        std::vector<geometry_msgs::PoseWithCovariance> poses_states;
        // std_msgs::Float64MultiArray prediction;

        std::stringstream buffer; // My debugging buffer string for printing


    public:
    Predictor(ros::NodeHandle *nh)
    {
        
        F               <<  1,  Ts_pred,    0,  0,          0,  0,          // x-axis position
                            0,  1,          0,  0,          0,  0,          // x-axis velocity
                            0,  0,          1,  Ts_pred,    0,  0,          // y-axis position
                            0,  0,          0,  1,          0,  0,          // y-axis velocity
                            0,  0,          0,  0,          1,  Ts_pred,    // z-axis position
                            0,  0,          0,  0,          0,  1;          // z-axis velocity
        G               =   Eigen::Matrix<double, states, 1>::Zero(); // zero vector | unused
        C               <<  1,  0,  0,  0,  0,  0, // x-axis position
                            0,  0,  1,  0,  0,  0, // y-axis position
                            0,  0,  0,  0,  1,  0; // z-axis position
        // sigma_sqrt_max  <<  0.03,   0.001,  0.001,
        //                     0.001,  0.03,   0.001,
        //                     0.001,  0.001,  0.03;
        sigma_sqrt_max  <<  0.03,  0.00,  0.00,
                            0.00,  0.03,  0.00,
                            0.00,  0.00,  0.03;
        sigma_sqrt      =  sigma_sqrt_max/3;
        // buffer << sigma_sqrt << std::endl;
        // ROS_INFO("sigma_sqrt: \n%s",buffer.str().c_str());
        // buffer.str("");
        // V2              =  sigma_sqrt; // For AIC
        V2              =  sigma_sqrt*1.5; // For MRAC, Also works for AIC
        // buffer << V2 << std::endl;
        // ROS_INFO("V2: \n%s",buffer.str().c_str());
        // buffer.str("");
        V1              <<  F*F.transpose()*Ts_pred; // Unused TODO: Well, it should be used :p
        buffer << V1 << std::endl;
        ROS_INFO("V1: \n%s",buffer.str().c_str());
        buffer.str("");
        // buffer << C << std::endl;
        // ROS_INFO("C: \n%s",buffer.str().c_str());
        // buffer.str(""); 
        Qm.setIdentity(); // Qm set to identity matrix
        Qp.setIdentity(); // Qp set to identity matrix

        // buffer << Qp << std::endl;
        // ROS_INFO("Qp: \n%s",buffer.str().c_str());
        // buffer.str("");

        // buffer << Qm << std::endl;
        // ROS_INFO("Qm: \n%s",buffer.str().c_str());
        // buffer.str("");
        I6.setIdentity(); // Initialize 6x6 Identity matrix

        y               =   Eigen::Matrix<double, outputs, 1>::Zero();
        y_orientation   =   Eigen::Matrix<double, 4      , 1>::Zero();
        L               =   Eigen::Matrix<double, states,  outputs>::Zero();
        Qm_star = Eigen::Matrix<double, horizon*states, states>::Zero(); // Qm_star initialization
        for(int i = 0;i<horizon;++i){
            Qm_star.block(i*states,0,states,states) = Qm;
        }
        // F_star initialization:
        F_star = Eigen::Matrix<double, horizon*states, states>::Zero(); // F_star is the matrix that generates the prediction vector xhat_pred based on xhat.
        Eigen::MatrixPower< Eigen::Matrix<double, states, states>> Fpow(F);
        for(int i = 0;i<horizon;++i){
            F_star.block(i*states,0,states,states) = Fpow(i+1);
        }
        F_N    = Fpow(horizon); // Reinitialization matrix for F_Ntsm whenever a new measurement has been acquired.
        F_Ntsm = F_N; // Initialization of future state transition matrices for further step ahead predictions than initially aniticipated by "horizon". This parameter as of 2022-03-18 is assumed only to be used for calculating Qm in further step ahead predictions.

        xhat_pred = Eigen::Matrix<double, horizon*states,1>::Zero(); // Initialize diemension of xhat_pred
        xhat_pred.block(0,0,states,1) << 0.3, 0.0, 0.6, 0.0, 0.0, 0.0; //<< 4, 3.0, -5, 0.5, 10, -0.4;  // Initialize assumptions about first measurement of xhat_pred

        // buffer << xhat_pred.transpose() << std::endl; // Test initialization of xhat_pred
        // ROS_INFO("xhat_pred: \n%s",buffer.str().c_str());
        // buffer.str("");

        // Publisher, Subscriber & Service initialization
        pub = nh->advertise<predictor::PredictedPoses>("/predicted_poses", 10);   
        measurement_subscriber = nh->subscribe("/target_pose_measurement_stamped", 1000, 
            &Predictor::callback_measurement, this);
        reset_service = nh->advertiseService("/reset_kalman_filter", 
            &Predictor::callback_reset_kalman_filter, this);
        
        predicted_poses.layout.dim.push_back(std_msgs::MultiArrayDimension()); // Increase dimension of predicted pose matrix by 1, to 1.
        predicted_poses.layout.dim.push_back(std_msgs::MultiArrayDimension()); // Increase dimension of predicted pose matrix by 1, to 2.
        predicted_poses.layout.dim[0].label  = "rows, prediction step";
        predicted_poses.layout.dim[0].size   = horizon;
        predicted_poses.layout.dim[0].stride = horizon*pred_order;
        predicted_poses.layout.dim[1].label  = "columns, prediction order"; // xp,yp,zp,xv,yv,zv
        predicted_poses.layout.dim[1].size   = pred_order;
        predicted_poses.layout.dim[1].stride = pred_order;
        predicted_poses.poses.push_back(geometry_msgs::PoseWithCovariance()); // This line has a compilation error. Do I even need to "push_back" as this array?
        predicted_poses.poses.resize(pred_order*horizon);

        // buffer << predicted_poses.poses.size() << std::endl;
        // ROS_INFO("predicted_poses.poses.size(): \n%s",buffer.str().c_str());
        // buffer.str("");
        // ROS_INFO("The Predictor object was Constructed");
    }

    predictor::PredictedPoses convert_prediction_to_message(/*xhat_pred*/){ // Is this variable inclusion redundant or unnecessary?
        // Array dimensions should have been handled in the constructor, so I should be able to just load in the header and poses.
        // predicted_poses.
        for (int h = 0; h < horizon; ++h){
            for (int p = 0; p < pred_order; ++p){
                predicted_poses.poses[h*pred_order + p].pose.position.x     = xhat_pred[h*states + p + 0 * pred_order];
                predicted_poses.poses[h*pred_order + p].pose.position.y     = xhat_pred[h*states + p + 1 * pred_order];
                predicted_poses.poses[h*pred_order + p].pose.position.z     = xhat_pred[h*states + p + 2 * pred_order];
                predicted_poses.poses[h*pred_order + p].pose.orientation.x  = y_orientation(0,0);
                predicted_poses.poses[h*pred_order + p].pose.orientation.y  = y_orientation(1,0);
                predicted_poses.poses[h*pred_order + p].pose.orientation.z  = y_orientation(2,0);
                predicted_poses.poses[h*pred_order + p].pose.orientation.w  = y_orientation(3,0);
                predicted_poses.poses[h*pred_order + p].covariance[0]      = Qm_star(h * states + p  + 0 * pred_order, p + 0 * pred_order); // In use
                predicted_poses.poses[h*pred_order + p].covariance[1]      = Qm_star(h * states + p  + 0 * pred_order, p + 1 * pred_order);
                predicted_poses.poses[h*pred_order + p].covariance[2]      = Qm_star(h * states + p  + 0 * pred_order, p + 2 * pred_order);
                predicted_poses.poses[h*pred_order + p].covariance[6]      = predicted_poses.poses[h*pred_order + p].covariance[1];
                predicted_poses.poses[h*pred_order + p].covariance[7]      = Qm_star(h * states + p  + 1 * pred_order, p + 1 * pred_order); // In use
                predicted_poses.poses[h*pred_order + p].covariance[8]      = Qm_star(h * states + p  + 1 * pred_order, p + 2 * pred_order);
                predicted_poses.poses[h*pred_order + p].covariance[12]     = predicted_poses.poses[h*pred_order + p].covariance[2];
                predicted_poses.poses[h*pred_order + p].covariance[13]     = predicted_poses.poses[h*pred_order + p].covariance[8];
                predicted_poses.poses[h*pred_order + p].covariance[14]     = Qm_star(h * states + p  + 2 * pred_order, p + 2 * pred_order); // In use
                
            }
        }
        // predicted_poses.poses[0].covariance[0]      = Qp(0,0); //Qm_star(h * states + p  + 0 * pred_order, p + 0 * pred_order); // In use
        // // predicted_poses.poses[0].covariance[1]      = Qp(0,0); //Qm_star(h * states + p  + 0 * pred_order, p + 1 * pred_order);
        // // predicted_poses.poses[0].covariance[2]      = Qp(0,0); //Qm_star(h * states + p  + 0 * pred_order, p + 2 * pred_order);
        // // predicted_poses.poses[0].covariance[6]      = Qp(0,0); //predicted_poses.poses[h*pred_order + p].covariance[1];
        // predicted_poses.poses[0].covariance[7]      = Qp(2,2); //Qm_star(h * states + p  + 1 * pred_order, p + 1 * pred_order); // In use
        // // predicted_poses.poses[0].covariance[8]      = Qp(0,0); //Qm_star(h * states + p  + 1 * pred_order, p + 2 * pred_order);
        // // predicted_poses.poses[0].covariance[12]     = Qp(0,0); //predicted_poses.poses[h*pred_order + p].covariance[2];
        // // predicted_poses.poses[0].covariance[13]     = Qp(0,0); //predicted_poses.poses[h*pred_order + p].covariance[8];
        // predicted_poses.poses[0].covariance[14]     = Qp(4,4); //Qm_star(h * states + p  + 2 * pred_order, p + 2 * pred_order); // In use
        // buffer << Qm_star.block(0,0,states,states) << std::endl;
        // ROS_INFO("Qm_star.block(0,0,6,6)\n%s",buffer.str().c_str());
        // buffer.str("");
        return predicted_poses;
    }
    
    void set_Qm_star(){ //Eigen::Matrix
        for (int  i = 0 ; i < horizon; ++i){
            Qm_star.block(states*i,0,states,states) = 
                  F_star.block(states*i,0,states,states)
                * Qp
                * F_star.block(states*i,0,states,states).transpose()
                + V1
                ;

        }
            
        // buffer << Qm_star << std::endl;
        // ROS_INFO("Qm_star: \n%s",buffer.str().c_str());
        // buffer.str("");
        return;
    }

    void predictive_kalman_filter(){  
        // I have to go with the asynchronous kalman filter
        // ROS_INFO("The kalman filter function has been called");
        ros::Rate r(1/Ts_pred); //acquire time and processing time // Answer, ros:Rate() enables this publication rate to be maintained.
        // verify the constantness of processing time
        // If it's reliable have PKF rate be integer multiple
        // If it't not reliable 
        // ros::WallTime t_start = ros::WallTime::now(), t_end = ros::WallTime::now(); // Used by silvia
        // ros::Duration dur();
        // Funciton to check if frequency is okay: "Rate-something"
        // int time_since_prediction = 0;
            // For Debugging
        // xhat_pred   = F_star * xhat_pred.block(0,0,states,1);
        // buffer << xhat_pred << std::endl;
        // ROS_INFO("xhat_pred: \n%s",buffer.str().c_str());
        // buffer.str("");

        // buffer << xhat_pred.block(0,0,states,1).transpose() << std::endl;
        // ROS_INFO("xhat_pred.block(0,0,states,1): \n%s",buffer.str().c_str());
        // buffer.str("");
    
        // ROS_INFO("Test of Qm_star BEGIN");
        // buffer << Qp << std::endl;
        // ROS_INFO("Qp: \n%s",buffer.str().c_str());
        // buffer.str("");
        // buffer << F_star << std::endl;
        // ROS_INFO("F_star: \n%s",buffer.str().c_str());
        // buffer.str("");
        // buffer << Qm_star << std::endl;
        // ROS_INFO("Qm_star before set_Qm_star(): \n%s",buffer.str().c_str());
        // buffer.str("");
        // set_Qm_star();
        // buffer << Qm_star << std::endl;
        // ROS_INFO("Qm_star after set_Qm_star(): \n%s",buffer.str().c_str());
        // buffer.str("");
        // ROS_INFO("Test of Qm_star END");
        
        // buffer << I6 << std::endl;
        // ROS_INFO("I6: \n%s",buffer.str().c_str());
        // buffer.str("");

        while(ros::ok()){
            // ROS_INFO("in first while loop");
            if(first_measurement_callback){
                // ROS_INFO("First measurement");
                while(ros::ok()/* && sleeptime okay*/){
                    // ROS_INFO("Kalman Started");
                    if(new_measurement_acquired){   /// new measurement:
                        
                        // ROS_INFO("Kalman new meas gain update");
                        xhat_pred = F_star * xhat_pred.block(0,0,states,1); // xhat_pred.block(0,0,states,1) is the state for time k-1
                        set_Qm_star(); // Only the block diagonal is computed and stacked in a horizon*states x states matrix // should only be computed when a new measurement is acruired
                        // Qm          = Qm_star.block(0,0,states,states); // F*Qp*F.transpose();
                        L = Qm_star.block(0,0,states,states) * C.transpose()  *  (C*Qm_star.block(0,0,states,states) * C.transpose() + V2).inverse(); // "Qm" from Qm_star has to be the one from the timestep prior to when the last measurement was acquired
                        // buffer << L << std::endl;                    // Den har kun ikke-nul værdier i (0,0),(0,1),(1,2),(1,3),(2,4),(2,5) // præcis som forventet givet Q
                        // ROS_INFO("L: \n%s",buffer.str().c_str());
                        // buffer.str("");
                        // xhat(-count)
                        xhat_pred.block(0,0,states,1)                   = xhat_pred.block(0,0,states,1) + L * (y - C * xhat_pred.block(0,0,states,1));   // Correct states based on measurement
                        xhat_pred.block(states,0,states*(horizon-1),1)  = F_star.block(0,0,states*(horizon-1),states) * xhat_pred.block(0,0,states,1); // Compute remaining predictions
                        // ROS_INFO("measurement y = %f",y(0,0));
                        // xhat_pred.block(0,0,states,1) += L * (y - C * xhat_pred.block(0,0,states,1));   // Correct states based on measurement
                        Qp = (I6 - L*C) * Qm_star.block(0,0,states,states); // Covaraiance update from Measurement Update
                        F_Ntsm = F_N; // F_Ntsm is reset seen as a new measurement has occured.
                        new_measurement_acquired = false;
                        
                        // ROS_INFO("Kalman new meas: ");
                        measurement_happended_at_last_iteration = true;
                    } else if(measurement_happended_at_last_iteration){
                        xhat_pred                                       = F_star * xhat_pred.block(0,0,states,1); // xhat_pred.block(0,0,states,1) is the state for time k-1
                        set_Qm_star(); 
                        Qp = Qm_star.block(0,0,states,states);
                        measurement_happended_at_last_iteration = false;
                    } else {                        /// no new measurement, and also not at the previous iteration:
                        xhat_pred.block(0,0,states*(horizon-1),1)       = xhat_pred.block(states,0,states*(horizon-1),1); // shift old predictions one step backwards //WARNING: this overrides the previous timestep state prediction, measurement or correction.
                        xhat_pred.block(states*(horizon-1),0,states,1)  = F_star.block(0,0,states,states) * xhat_pred.block(states*(horizon-1),0,states,1); // horizon-1 step prediction (Replace last state prediction)
                        
                        Qm_star.block(0,0,states*(horizon-1),states) = Qm_star.block(states,0,states*(horizon-1),states);    // Shift Qm by one time step
                        Qm_star.block(states*(horizon-1),0,states,states) = F_Ntsm * Qp * F_Ntsm.transpose();          // Override the last 2x2 matrix in Qm_star
                        F_Ntsm = F_Ntsm * F; // should be updated after use as it's initialized to F^horizon and not F^(horizon-1)
                        // Calculate the horizon-1 prediction and Qm and Qm_star matrices
                        // buffer << xhat_pred << std::endl;
                        // ROS_INFO("xhat_pred: \n%s",buffer.str().c_str());
                        // buffer.str("");
                        
                        
                        Qp = Qm_star.block(0,0,states,states);
                        // ROS_INFO("We heard position x was: ");
                        // ROS_INFO("We heard position x was: ");
                        // ROS_INFO("Kalman old meas");  
                    }

                    // buffer << xhat_pred.block(0,0,states,1).transpose() << std::endl;
                    // ROS_INFO("xhat_pred.block(0,0,states,1): \n%s",buffer.str().c_str());
                    // buffer.str("");
                    // buffer << Qm_star.block(0,0,states,states) << std::endl;
                    // ROS_INFO("Qm_star.block(0,0,states,states): \n%s",buffer.str().c_str());
                    // buffer.str("");
                    // buffer << Qp << std::endl;
                    // ROS_INFO("Qp: \n%s",buffer.str().c_str());
                    // buffer.str("");
                    pub.publish(convert_prediction_to_message(/*xhat_pred*/)); // now publish new prediction vector
                    // ros::spinOnce(); // Check if new measurements are available // location 2 of ros::spinOnce()

                    // Qm_star (2*60 x 2*60) = F_star (2*60 x 2) * Qp * F_star.transpose(); // When making "all" Qm calculations in one prediction, make sure to pick the right one. The one corresponding to right before the measurement is acquired in terms of time step. "predicting to 11, but measurement is at 10, pick Qm calculated for right before 10"
                    // Here is last moment at which a new measurement must be acquired to be used in this prediction cycle.
                    
                    // ROS_INFO("Kalman Ran Through");
                    ros::spinOnce(); // Check if new measurements are available // location 1 of ros::spinOnce()

                    r.sleep();
                }
            }
            ros::spinOnce();
        }
    }

    void callback_measurement(const geometry_msgs::PoseWithCovarianceStamped& msg) {
        predicted_poses.header = msg.header; // Copy header
        y(0,0) = msg.pose.pose.position.x; // x position
        y(1,0) = msg.pose.pose.position.y; // y position
        y(2,0) = msg.pose.pose.position.z; // z position
        // ROS_INFO("We heard position x was: %f",y(0,0));
        // ROS_INFO("We heard position y was: %f",y(1,0));
        // ROS_INFO("We heard position z was: %f",y(2,0));
        y_orientation(0,0) = msg.pose.pose.orientation.x;
        y_orientation(1,0) = msg.pose.pose.orientation.y;
        y_orientation(2,0) = msg.pose.pose.orientation.z;
        y_orientation(3,0) = msg.pose.pose.orientation.w;
        // y(0,0) += 10.0; // Add 10 to the measurement artificially
        new_measurement_acquired = true;
        first_measurement_callback = true;
    }

    bool callback_reset_kalman_filter(std_srvs::SetBool::Request &req, 
                                std_srvs::SetBool::Response &res)
    {
        if (req.data) {
            res.success = true;
            res.message = "Counter has been successfully reset";
        }
        else {
            res.success = false;
            res.message = "Counter has not been reset";
        }

        return true;
    }
};

int main (int argc, char **argv)
{
    ros::init(argc, argv, "kalman_predictor");
    ros::NodeHandle nh;
    Predictor nc = Predictor(&nh);
    nc.predictive_kalman_filter();

    ros::spin();
}
