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


// #define states 2
// #define inputs 0
// #define outputs 1

class Predictor {

    private:
    // Kalman Filter Initialization parameters
    // Integer varaibles
    /// states:         Predictor states
    /// inputs:         Predictor inputs
    /// outputs:        System outputs
    /// horizon:        Prediction horizon
    /// Ts_pred:        Time between prediction generations
    /// Ts_meas:        Time between measurements
    const static int states = 2, inputs = 0, outputs = 1, horizon = 60;
    const double Ts_pred = 0.1, Ts_meas = 0.2; // As of right now 2022-03-18, there is no certainty in the frequency or regularity of the measurement. Last time "$ rostopic hz /target_pose_measurement_stamped" was used the frequency varied between 2 adn 5 hz. There may be a bottleneck in one of the loops in a node somewhere.
    bool new_measurement_acquired   = false;
    bool measurement_happended_at_last_iteration = false;
    bool first_measurement_callback = false;
    // Double variables
    /// sigma_sqrt_max: True Maximum sensor noise amplitude
    /// sigma_sqrt:     True Variance of the measurement noise
    /// V1:             State Uncertainty
    /// V2:             Measurement Uncertainty
    /// y:              Measurement signal from subscription
    Eigen::Matrix<double, 1, 1> sigma_sqrt_max, sigma_sqrt, V1, V2, y;

    // Matrix variables
    Eigen::Matrix<double, 2, 2> F; // TODO: If I find a way to have the dimensions of F be modular, replace the constants inserted here.
    Eigen::Matrix<double, 2, 2> F_Ntsm; //
    Eigen::Matrix<double, 2, 1> G; // This one is zero and should be removed for optimization
    Eigen::Matrix<double, 1, 2> C; // Model output matrix
    Eigen::Matrix<double, 2, 2> Qm; //Qm = Q⁻(k) and Qp = Q⁺(k)
    Eigen::Matrix<double, 2, 2> Qp; //Qm = Q⁻(k) and Qp = Q⁺(k)
    Eigen::Matrix<double, 2, 1> xhat, L; // xhat is the 
    Eigen::Matrix<double, 2, 2> I2;      // 2x2 Identity matrix
    int counter;
    Eigen::Matrix<double,Eigen::Dynamic, Eigen::Dynamic> F_star, Qm_star;
    // Eigen::Matrix<double,Eigen::Dynamic, Eigen::Dynamic> xhat_pred;
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
        sigma_sqrt_max  << 0.03;
        sigma_sqrt      << sigma_sqrt_max;
        V2              << 0.01;
        V1              << 0;
        F               << 1, Ts_pred, 0, 1;
        G               << 0, 0;
        C               << 1, 0;
        Qm = Qm.Identity();// = Eigen::Matrix<double,2,2>::Identity();//<< 1, Ts_pred, 0, 1;
        Qp = Qp.Identity();// << 1, Ts_pred, 0, 1;

        buffer << Qp << std::endl;
        ROS_INFO("Qp: %s",buffer.str().c_str());
        buffer.str("");

        buffer << Qm << std::endl;
        ROS_INFO("Qm: %s",buffer.str().c_str());
        buffer.str("");
        I2.setIdentity();
        xhat            << 4, 3.0;
        y               << 0;
        L               << 0, 0;
        Qm_star = Eigen::Matrix<double, horizon*states, states>::Zero(); // Qm_star initialization
        for(int i = 0;i<horizon;++i){
            Qm_star.block(i*states,0,states,states) = Qm;
        }
        // F_star initialization:
        F_star = Eigen::Matrix<double, horizon*states, states>::Zero(); // F_star is the matrix that generates the prediction vector xhat_pred based on xhat.
        Eigen::MatrixPower< Eigen::Matrix<double, states, states> > Fpow(F);
        for(int i = 0;i<horizon;++i){
            F_star.block(i*states,0,states,states) = Fpow(i+1);
        }
        F_Ntsm = Fpow(horizon); // Initialization of future state transition matrices for further step ahead predictions than initially aniticipated by "horizon". This parameter as of 2022-03-18 is assumed only to be used for calculating Qm in further step ahead predictions.
        // // Test print of initialization of F_star:
        // std::stringstream buffer; 
        // buffer << "F*\n" << F_star << std::endl;
        // ROS_INFO("%s",buffer.str().c_str());
        xhat_pred = Eigen::Matrix<double, horizon*states,1>::Zero();
        xhat_pred.block(0,0,states,1) << 0.0,0.0; // 15.0, 0.001234;
        counter = 10;



        pub = nh->advertise<predictor::PredictedPoses>("/predicted_poses1d", 10);   
        // pub = nh->advertise<std_msgs::Float64MultiArray>("/reference_vector", 10); 

        measurement_subscriber = nh->subscribe("/target_pose_measurement_stamped", 1000, 
            &Predictor::callback_measurement, this);
        reset_service = nh->advertiseService("/reset_kalman_filter", 
            &Predictor::callback_reset_kalman_filter, this);
        
        predicted_poses.layout.dim.push_back(std_msgs::MultiArrayDimension()); // Increase dimension of predicted pose matrix by 1, to 1.
        predicted_poses.layout.dim.push_back(std_msgs::MultiArrayDimension()); // Increase dimension of predicted pose matrix by 1, to 2.
        predicted_poses.layout.dim[0].label  = "rows, prediction step";
        predicted_poses.layout.dim[0].size   = horizon;
        predicted_poses.layout.dim[0].stride = horizon*states;
        predicted_poses.layout.dim[1].label  = "columns, prediction order [position, velocity]"; // xp,yp,zp,xv,yv,zv
        predicted_poses.layout.dim[1].size   = states;
        predicted_poses.layout.dim[1].stride = states;
        predicted_poses.poses.push_back(geometry_msgs::PoseWithCovariance()); // This line has a compilation error. Do I even need to "push_back" as this array?
        // poses_states = std::vector<geometry_msgs::PoseWithCovariance> 
        predicted_poses.poses.resize(states*horizon);
        // poses_states = (horizon*states);

        ROS_INFO("The Predictor object was Constructed");
        // predictive_kalman_filter(); // This has an "infinite" loop so this can't possibly be the correct way to do this!!! // Hypothesis was correct, this is not the way to do it. It doesn't work and doesn't ever come past this function. 
        
        // prediction.layout.dim.push_back(std_msgs::MultiArrayDimension());
        // prediction.layout.dim.push_back(std_msgs::MultiArrayDimension());
        // prediction.layout.dim[0].label  = "rows";
        // prediction.layout.dim[0].size   = horizon;
        // prediction.layout.dim[0].stride = horizon*states;
        // prediction.layout.dim[1].label  = "columns";
        // prediction.layout.dim[1].size   = states;
        // prediction.layout.dim[1].stride = states;
        // prediction.layout.data_offset   = 0;
    }

    predictor::PredictedPoses convert_prediction_to_message(/*xhat_pred*/){ // Is this variable inclusion redundant or unnecessary?
        // Array dimensions should have been handled in the constructor, so I should be able to just load in the header and poses.
        // predicted_poses.
        for (int h = 0; h < horizon; ++h){
            for (int s = 0; s < states; ++s){
                predicted_poses.poses[h*states + s].pose.position.x = xhat_pred[h*states + s];
            }
            // ROS_INFO("prediction converted: 40x = %s",buffer.c_str());
        }
        // for
        // ROS_INFO("prediction converted: 40x = %f",predicted_poses.poses[40].pose.position.x);
        return predicted_poses;
    }
    
    void set_Qm_star(){ //Eigen::Matrix
        for (int  i = 0 ; i < horizon; ++i){
            Qm_star.block(states*i,0,states,states) = 
                  F_star.block(states*i,0,states,states)
                * Qp
                * F_star.block(states*i,0,states,states).transpose();
        }
        return;
    }

    void predictive_kalman_filter(){  
        // I have to go with the asynchronous kalman filter
        ROS_INFO("The kalman filter function has been called");
        ros::Rate r(1/Ts_pred); //acquire time and processing time // Answer, ros:Rate() enables this publication rate to be maintained.
        // verify the constantness of processing time
        // If it's reliable have PKF rate be integer multiple
        // If it't not reliable 
        // ros::WallTime t_start = ros::WallTime::now(), t_end = ros::WallTime::now(); // Used by silvia
        // ros::Duration dur();
        // Funciton to check if frequency is okay: "Rate-something"
        int time_since_prediction = 0;
        // For Debugging
        // xhat_pred   = F_star * xhat_pred.block(0,0,states,1);
        // buffer << xhat_pred << std::endl;
        // ROS_INFO("xhat_pred: \n%s",buffer.str().c_str());
        // buffer.str("");

        buffer << xhat_pred.block(0,0,states,1).transpose() << std::endl;
        ROS_INFO("xhat_pred.block(0,0,states,1): \n%s",buffer.str().c_str());
        buffer.str("");
    
        ROS_INFO("Test of Qm_star BEGIN");
        buffer << Qp << std::endl;
        ROS_INFO("Qp: \n%s",buffer.str().c_str());
        buffer.str("");
        buffer << F_star << std::endl;
        ROS_INFO("F_star: \n%s",buffer.str().c_str());
        buffer.str("");
        buffer << Qm_star << std::endl;
        ROS_INFO("Qm_star before set_Qm_star(): \n%s",buffer.str().c_str());
        buffer.str("");
        set_Qm_star();
        buffer << Qm_star << std::endl;
        ROS_INFO("Qm_star after set_Qm_star(): \n%s",buffer.str().c_str());
        buffer.str("");
        ROS_INFO("Test of Qm_star END");

        while(ros::ok()){
        // ROS_INFO("in first while loop");
        if(first_measurement_callback){
        ROS_INFO("First measurement");
        while(ros::ok()/* && sleeptime okay*/){
            // if(new_measurement_acquired){ 
            // }
            // ROS_INFO("Kalman Started");
            // timing correction
            // xhat = F * xhat;
            // xhat_pred = F_star * xhat;
            if(new_measurement_acquired){   /// new measurement:
                
                ROS_INFO("Kalman new meas gain update");
                xhat_pred                                       = F_star * xhat_pred.block(0,0,states,1); // xhat_pred.block(0,0,states,1) is the state for time k-1
                set_Qm_star(); // Only the block diagonal is computed and stacked in a horizon*states x states matrix // should only be computed when a new measurement is acruired
                // Qm          = Qm_star.block(0,0,states,states); // F*Qp*F.transpose();
                L                                               = Qm_star.block(0,0,states,states) * C.transpose()  *  (C*Qm_star.block(0,0,states,states) * C.transpose() + V2).inverse(); // "Qm" from Qm_star has to be the one from the timestep prior to when the last measurement was acquired
                
                // xhat(-count)
                xhat_pred.block(0,0,states,1)                   = xhat_pred.block(0,0,states,1) + L * (y - C * xhat_pred.block(0,0,states,1));   // Correct states based on measurement
                xhat_pred.block(states,0,states*(horizon-1),1)  = F_star.block(0,0,states*(horizon-1),states) * xhat_pred.block(0,0,states,1); // Compute remaining predictions
                // ROS_INFO("measurement y = %f",y(0,0));
                // xhat_pred.block(0,0,states,1) += L * (y - C * xhat_pred.block(0,0,states,1));   // Correct states based on measurement
                Qp                                              = (I2 - L*C) * Qm_star.block(0,0,states,states); // Covaraiance update from Measurement Update
                
                new_measurement_acquired = false;
                
                ROS_INFO("Kalman new meas: ");
                measurement_happended_at_last_iteration = true;
            } else if(measurement_happended_at_last_iteration){
                xhat_pred                                       = F_star * xhat_pred.block(0,0,states,1); // xhat_pred.block(0,0,states,1) is the state for time k-1
                set_Qm_star(); 
                Qp = Qm_star.block(0,0,states,states);
                measurement_happended_at_last_iteration = false;
            } else {                        /// no new measurement, and also not at the previous iteration:
                xhat_pred.block(0,0,states*(horizon-1),1)       = xhat_pred.block(states,0,states*(horizon-1),1); // shift old predictions one step backwards //WARNING: this overrides the previous timestep state prediction, measurement or correction.
                xhat_pred.block(states*(horizon-1),0,states,1)  = F_star.block(0,0,states,states) * xhat_pred.block(states*(horizon-1),0,states,1); // horizon-1 step prediction (Replace last state prediction)
                
                Qm_star.block(0,0,states*(horizon-1),states) = Qm_star.block(2,0,states*(horizon-1),states);    // Shift Qm by one time step
                Qm_star.block(states*(horizon-1),0,states,states) = F_Ntsm * Qp * F_Ntsm.transpose();          // Override the last 2x2 matrix in Qm_star
                F_Ntsm = F_Ntsm * F; // should be updated after use as it's initialized to F^horizon and not F^(horizon-1)
                // Calculate the horizon-1 prediction and Qm and Qm_star matrices
                // buffer << xhat_pred << std::endl;
                // ROS_INFO("xhat_pred: \n%s",buffer.str().c_str());
                // buffer.str("");
                
                
                Qp = Qm_star.block(0,0,states,states);
                ROS_INFO("We heard position x was: ");
                ROS_INFO("We heard position x was: ");
                ROS_INFO("Kalman old meas");  
            }

            buffer << xhat_pred.block(0,0,states,1).transpose() << std::endl;
            ROS_INFO("xhat_pred.block(0,0,states,1): \n%s",buffer.str().c_str());
            buffer.str("");
            buffer << Qm_star.block(0,0,states,states) << std::endl;
            ROS_INFO("Qm_star.block(0,0,states,states): \n%s",buffer.str().c_str());
            buffer.str("");
            buffer << Qp << std::endl;
            ROS_INFO("Qp: \n%s",buffer.str().c_str());
            buffer.str("");
            pub.publish(convert_prediction_to_message(/*xhat_pred*/)); // now publish new prediction vector
            // ros::spinOnce(); // Check if new measurements are available // location 2 of ros::spinOnce()

            // Qm_star (2*60 x 2*60) = F_star (2*60 x 2) * Qp * F_star.transpose(); // When making "all" Qm calculations in one prediction, make sure to pick the right one. The one corresponding to right before the measurement is acquired in terms of time step. "predicting to 11, but measurement is at 10, pick Qm calculated for right before 10"
            // Here is last moment at which a new measurement must be acquired to be used in this prediction cycle.
            
            // if(new_measurement_acquired){
            //     new_measurement_acquired = false;
            //     L       = Qm * C.transpose()  *  (C*Qm * C.transpose() + V2).inverse(); // "Qm" from Qm_star has to be the one from the timestep prior to when the last measurement was acquired
            //     // xhat(-count)
            //     xhat    = xhat + L * (y - C * xhat);   // Correct states based on measurement
            //     Qp      = (I2 - L*C) * Qm;                   // Covaraiance update from Measurement Update
            //     // count = 0
            // }
            // ROS_INFO("hej %d %f",F_star_y,sigma_sqrt);
            // When should I call the publishing of xhat_pred?
            // count++

            ++time_since_prediction;
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
        // y(0,0) = msg.pose.pose.position.y; // y position
        // y(0,0) = msg.pose.pose.position.z; // z position
        ROS_INFO("We heard position x was: %f",y(0,0));
        // y(0,0) += 10.0; // Add 10 to the measurement artificially
        new_measurement_acquired = true;
        first_measurement_callback = true;
    }

    bool callback_reset_kalman_filter(std_srvs::SetBool::Request &req, 
                                std_srvs::SetBool::Response &res)
    {
        if (req.data) {
            counter = 0;
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
    ros::init(argc, argv, "number_counter");
    ros::NodeHandle nh;
    Predictor nc = Predictor(&nh);
    nc.predictive_kalman_filter();

    ros::spin();
}