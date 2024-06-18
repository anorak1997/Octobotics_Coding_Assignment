// #include <ros/ros.h>
// #include <inverted_controller/ControlForce.h>
// #include <cmath> // For sin function

// int main(int argc, char** argv) {
//     // Initialize the ROS node
//     ros::init(argc, argv, "control_force_publisher");
//     ros::NodeHandle nh;

//     // Create a publisher object
//     ros::Publisher force_pub = nh.advertise<inverted_controller::ControlForce>("/inverted_pendulum/control_force", 10);

//     // Define the message
//     inverted_controller::ControlForce msg;

//     // Define the frequency of publishing
//     ros::Rate loop_rate(100); // 100 Hz

//     // Variables for sine wave generation
//     double amplitude = 10.0;
//     double frequency = 0.1;
//     double time = 0.0;
//     double time_step = 0.01; // 100 Hz -> 0.01 s

//     while (ros::ok()) {
//         // Generate sinusoidal force
//         msg.force = amplitude * sin(2 * M_PI * frequency * time);

//         // Publish the message
//         force_pub.publish(msg);

//         // Update time
//         time += time_step;

//         // Update amplitude and frequency for variation
//         amplitude = 10.0 + 5.0 * sin(0.1 * time); // Slowly vary amplitude between 5 and 15
//         frequency = 0.1 + 0.05 * sin(0.05 * time); // Slowly vary frequency between 0.05 and 0.15

//         // Spin and sleep to maintain the loop rate
//         ros::spinOnce();
//         loop_rate.sleep();
//     }

//     return 0;
// }


// Currently being used


// #include <ros/ros.h>
// #include <cmath>
// #include <iostream>
// #include <inverted_pendulum_sim/CurrentState.h>
// #include <inverted_controller/ControlForce.h>

// class PIDController {
// public:
//     PIDController() :
//         kp_(100.0), ki_(0.0), kd_(50.0),
//         max_force_(50.0), min_force_(-50.0),
//         error_sum_(0.0), last_error_(0.0),
//         dt_(1.0/100.0), // control loop rate 100 Hz
//         target_theta_(M_PI / 2) // Target theta (upright position)
//     {
//         // Initialize ROS components
//         nh_.param<double>("/inverted_pendulum/kp", kp_, 50.0);
//         nh_.param<double>("/inverted_pendulum/ki", ki_, 0.0);
//         nh_.param<double>("/inverted_pendulum/kd", kd_, 10.0);
//         nh_.param<double>("/inverted_pendulum/max_force", max_force_, 50.0);
//         nh_.param<double>("/inverted_pendulum/min_force", min_force_, -50.0);
//         nh_.param<double>("/inverted_pendulum/dt", dt_, 1.0/100.0);
//         nh_.param<double>("/inverted_pendulum/target_theta", target_theta_, M_PI / 2);

//         current_state_sub_ = nh_.subscribe("/inverted_pendulum/current_state", 10, &PIDController::currentStateCallback, this);
//         control_force_pub_ = nh_.advertise<inverted_controller::ControlForce>("/inverted_pendulum/control_force", 10);
//     }

//     void currentStateCallback(const inverted_pendulum_sim::CurrentState::ConstPtr& msg) {
//         double current_theta = msg->curr_theta;
//         double current_theta_dot = msg->curr_theta_dot;

//         // Compute error
//         double error = target_theta_ - current_theta;

//         // Integral term
//         error_sum_ += error * dt_;

//         // Derivative term
//         double error_diff = (error - last_error_) / dt_;
//         last_error_ = error;

//         // Compute control force
//         double force = kp_ * error + ki_ * error_sum_ + kd_ * error_diff;

//         // Apply limits to the control force
//         if (force > max_force_) {
//             force = max_force_;
//         } else if (force < min_force_) {
//             force = min_force_;
//         }

//         // Publish control force
//         inverted_controller::ControlForce control_msg;
//         control_msg.force = force;
//         control_force_pub_.publish(control_msg);
//     }

//     void run() {
//         ros::Rate rate(1.0 / dt_);
//         while (ros::ok()) {
//             ros::spinOnce();
//             rate.sleep();
//         }
//     }

// private:
//     ros::NodeHandle nh_;
//     ros::Subscriber current_state_sub_;
//     ros::Publisher control_force_pub_;

//     // PID parameters
//     double kp_;
//     double ki_;
//     double kd_;

//     // Force limits
//     double max_force_;
//     double min_force_;

//     // PID variables
//     double error_sum_;
//     double last_error_;

//     // Control loop parameters
//     double dt_;

//     // Target angle for the pendulum
//     double target_theta_;
// };

// int main(int argc, char **argv) {
//     ros::init(argc, argv, "pid_controller");
//     PIDController controller;
//     controller.run();
//     return 0;
// }















// #include <ros/ros.h>
// #include <cmath>
// #include <iostream>
// #include <inverted_pendulum_sim/CurrentState.h>
// #include <inverted_controller/ControlForce.h>

// class PIDController {
// public:
//     PIDController() :
//         kp_theta_(200.0), ki_theta_(0.0), kd_theta_(50.0),
//         kp_x_(5.0), ki_x_(0.0), kd_x_(1.0),
//         max_force_(50.0), min_force_(-50.0),
//         error_sum_theta_(0.0), last_error_theta_(0.0),
//         error_sum_x_(0.0), last_error_x_(0.0),
//         dt_(1.0/100.0), // control loop rate 100 Hz
//         target_theta_(M_PI), // Target theta (upright position)
//         target_x_(0.0) // Target x position
//     {
//         // Initialize ROS components
//         nh_.param<double>("/inverted_pendulum/kp_theta", kp_theta_, 200.0);
//         nh_.param<double>("/inverted_pendulum/ki_theta", ki_theta_, 0.0);
//         nh_.param<double>("/inverted_pendulum/kd_theta", kd_theta_, 50.0);
//         nh_.param<double>("/inverted_pendulum/kp_x", kp_x_, 5.0);
//         nh_.param<double>("/inverted_pendulum/ki_x", ki_x_, 0.0);
//         nh_.param<double>("/inverted_pendulum/kd_x", kd_x_, 1.0);
//         nh_.param<double>("/inverted_pendulum/max_force", max_force_, 50.0);
//         nh_.param<double>("/inverted_pendulum/min_force", min_force_, -50.0);
//         nh_.param<double>("/inverted_pendulum/dt", dt_, 1.0/100.0);

//         current_state_sub_ = nh_.subscribe("/inverted_pendulum/current_state", 10, &PIDController::currentStateCallback, this);
//         control_force_pub_ = nh_.advertise<inverted_controller::ControlForce>("/inverted_pendulum/control_force", 10);
//     }

//     void currentStateCallback(const inverted_pendulum_sim::CurrentState::ConstPtr& msg) {
//         double current_x = msg->curr_x;
//         double current_x_dot = msg->curr_x_dot;
//         double current_theta = msg->curr_theta;
//         double current_theta_dot = msg->curr_theta_dot;

//         // Compute error for theta
//         double error_theta = target_theta_ - current_theta;

//         // Integral term for theta
//         error_sum_theta_ += error_theta * dt_;

//         // Derivative term for theta
//         double error_diff_theta = (error_theta - last_error_theta_) / dt_;
//         last_error_theta_ = error_theta;

//         // Compute control force for theta
//         double force_theta = kp_theta_ * error_theta + ki_theta_ * error_sum_theta_ + kd_theta_ * error_diff_theta;

//         // Compute error for x
//         double error_x = target_x_ - current_x;

//         // Integral term for x
//         error_sum_x_ += error_x * dt_;

//         // Derivative term for x
//         double error_diff_x = (error_x - last_error_x_) / dt_;
//         last_error_x_ = error_x;

//         // Compute control force for x
//         double force_x = kp_x_ * error_x + ki_x_ * error_sum_x_ + kd_x_ * error_diff_x;

//         // Combine forces
//         double force = force_theta + force_x;

//         // Apply limits to the control force
//         if (force > max_force_) {
//             force = max_force_;
//         } else if (force < min_force_) {
//             force = min_force_;
//         }

//         // Publish control force
//         inverted_controller::ControlForce control_msg;
//         control_msg.force = force;
//         control_force_pub_.publish(control_msg);
//     }

//     void run() {
//         ros::Rate rate(1.0 / dt_);
//         while (ros::ok()) {
//             ros::spinOnce();
//             rate.sleep();
//         }
//     }

// private:
//     ros::NodeHandle nh_;
//     ros::Subscriber current_state_sub_;
//     ros::Publisher control_force_pub_;

//     // PID parameters for theta (pendulum angle)
//     double kp_theta_;
//     double ki_theta_;
//     double kd_theta_;

//     // PID parameters for x (cart position)
//     double kp_x_;
//     double ki_x_;
//     double kd_x_;

//     // Force limits
//     double max_force_;
//     double min_force_;

//     // PID variables for theta
//     double error_sum_theta_;
//     double last_error_theta_;

//     // PID variables for x
//     double error_sum_x_;
//     double last_error_x_;

//     // Control loop parameters
//     double dt_;

//     // Target angles and positions
//     double target_theta_; // Target theta (upright position)
//     double target_x_; // Target x position
// };

// int main(int argc, char **argv) {
//     ros::init(argc, argv, "pid_controller");
//     PIDController controller;
//     controller.run();
//     return 0;
// }
















// #include <ros/ros.h>
// #include <cmath>
// #include <iostream>
// #include <inverted_pendulum_sim/CurrentState.h>
// #include <inverted_controller/ControlForce.h>

// class PIDController {
// public:
//     PIDController() :
//         kp_theta_(200.0), ki_theta_(0.0), kd_theta_(50.0),
//         kp_x_(5.0), ki_x_(0.0), kd_x_(1.0),
//         max_force_(50.0), min_force_(-50.0),
//         error_sum_theta_(0.0), last_error_theta_(0.0),
//         error_sum_x_(0.0), last_error_x_(0.0),
//         dt_(1.0/100.0), // control loop rate 100 Hz
//         target_theta_up_(M_PI / 2), // Target theta (upright position)
//         target_theta_zero_(M_PI / 2), // Target theta (zero position)
//         target_x_up_(2.0), // Target x position (center)
//         target_x_zero_(0.0) // Target x position (zero)
//     {
//         // Initialize ROS components
//         nh_.param<double>("/inverted_pendulum/kp_theta", kp_theta_, 200.0);
//         nh_.param<double>("/inverted_pendulum/ki_theta", ki_theta_, -10.0);
//         nh_.param<double>("/inverted_pendulum/kd_theta", kd_theta_, 50.0);
//         nh_.param<double>("/inverted_pendulum/kp_x", kp_x_, 5.0);
//         nh_.param<double>("/inverted_pendulum/ki_x", ki_x_, 0.0);
//         nh_.param<double>("/inverted_pendulum/kd_x", kd_x_, 1.0);
//         nh_.param<double>("/inverted_pendulum/max_force", max_force_, 50.0);
//         nh_.param<double>("/inverted_pendulum/min_force", min_force_, -50.0);
//         nh_.param<double>("/inverted_pendulum/dt", dt_, 1.0/100.0);

//         current_state_sub_ = nh_.subscribe("/inverted_pendulum/current_state", 10, &PIDController::currentStateCallback, this);
//         control_force_pub_ = nh_.advertise<inverted_controller::ControlForce>("/inverted_pendulum/control_force", 10);

//         // Initial target positions
//         target_theta_ = target_theta_up_;
//         target_x_ = target_x_up_;
//     }

//     void currentStateCallback(const inverted_pendulum_sim::CurrentState::ConstPtr& msg) {
//         double current_x = msg->curr_x;
//         double current_x_dot = msg->curr_x_dot;
//         double current_theta = msg->curr_theta;
//         double current_theta_dot = msg->curr_theta_dot;

//         // Compute error for theta
//         double error_theta = target_theta_ - current_theta;

//         // Integral term for theta
//         error_sum_theta_ += error_theta * dt_;

//         // Derivative term for theta
//         double error_diff_theta = (error_theta - last_error_theta_) / dt_;
//         last_error_theta_ = error_theta;

//         // Compute control force for theta
//         double force_theta = kp_theta_ * error_theta + ki_theta_ * error_sum_theta_ + kd_theta_ * error_diff_theta;

//         // Compute error for x
//         double error_x = target_x_ - current_x;

//         // Integral term for x
//         error_sum_x_ += error_x * dt_;

//         // Derivative term for x
//         double error_diff_x = (error_x - last_error_x_) / dt_;
//         last_error_x_ = error_x;

//         // Compute control force for x
//         double force_x = kp_x_ * error_x + ki_x_ * error_sum_x_ + kd_x_ * error_diff_x;

//         // Combine forces
//         double force = force_theta + force_x;

//         // Apply limits to the control force
//         if (force > max_force_) {
//             force = max_force_;
//         } else if (force < min_force_) {
//             force = min_force_;
//         }

//         // Publish control force
//         inverted_controller::ControlForce control_msg;
//         control_msg.force = force;
//         control_force_pub_.publish(control_msg);
//     }

//     void run() {
//         ros::Rate rate(1.0 / dt_);
//         while (ros::ok()) {
//             ros::spinOnce();
//             rate.sleep();
//         }
//     }

// private:
//     ros::NodeHandle nh_;
//     ros::Subscriber current_state_sub_;
//     ros::Publisher control_force_pub_;

//     // PID parameters for theta (pendulum angle)
//     double kp_theta_;
//     double ki_theta_;
//     double kd_theta_;

//     // PID parameters for x (cart position)
//     double kp_x_;
//     double ki_x_;
//     double kd_x_;

//     // Force limits
//     double max_force_;
//     double min_force_;

//     // PID variables for theta
//     double error_sum_theta_;
//     double last_error_theta_;

//     // PID variables for x
//     double error_sum_x_;
//     double last_error_x_;

//     // Control loop parameters
//     double dt_;

//     // Target angles and positions
//     double target_theta_up_; // Target theta (upright position)
//     double target_theta_zero_; // Target theta (zero position)
//     double target_x_up_; // Target x position (center)
//     double target_x_zero_; // Target x position (zero)

//     double target_theta_; // Current target theta
//     double target_x_; // Current target x
// };

// int main(int argc, char **argv) {
//     ros::init(argc, argv, "pid_controller");
//     PIDController controller;
//     controller.run();
//     return 0;
// }




#include <ros/ros.h>
#include <cmath>
#include <iostream>
#include <inverted_pendulum_sim/CurrentState.h>
#include <inverted_pendulum_sim/ControlForce.h>

class PIDController {
public:
    PIDController() :
        kp_theta_(25.0), ki_theta_(0.01), kd_theta_(5.0),
        kp_x_(100.0), ki_x_(5.0), kd_x_(0.55),
        max_force_(50.0), min_force_(-50.0),
        error_sum_theta_(0.1), last_error_theta_(0.1),
        error_sum_x_(0.0), last_error_x_(50.0),
        dt_(1.0/100.0), // control loop rate 100 Hz
        target_theta_(M_PI), // Target theta (vertical position)
        target_x_(0.0) // Target x position
    {
        // Initialize ROS components
        nh_.param<double>("/inverted_pendulum/kp_theta", kp_theta_, 25.0);
        nh_.param<double>("/inverted_pendulum/ki_theta", ki_theta_, 0.01);
        nh_.param<double>("/inverted_pendulum/kd_theta", kd_theta_, 5.0);
        nh_.param<double>("/inverted_pendulum/kp_x", kp_x_, 100.0);
        nh_.param<double>("/inverted_pendulum/ki_x", ki_x_, 5.0);
        nh_.param<double>("/inverted_pendulum/kd_x", kd_x_, 0.55);
        nh_.param<double>("/inverted_pendulum/max_force", max_force_, 50.0);
        nh_.param<double>("/inverted_pendulum/min_force", min_force_, -50.0);
        nh_.param<double>("/inverted_pendulum/dt", dt_, 1.0/100.0);

        current_state_sub_ = nh_.subscribe("/inverted_pendulum/current_state", 10, &PIDController::currentStateCallback, this);
        control_force_pub_ = nh_.advertise<inverted_pendulum_sim::ControlForce>("/inverted_pendulum/control_force", 10);
    }

    void currentStateCallback(const inverted_pendulum_sim::CurrentState::ConstPtr& msg) {
        double current_x = msg->curr_x;
        double current_x_dot = msg->curr_x_dot;
        double current_theta = msg->curr_theta;
        double current_theta_dot = msg->curr_theta_dot;

        // Compute error for theta
        double error_theta = target_theta_ - current_theta;

        // Integral term for theta
        error_sum_theta_ += error_theta * dt_;

        // Derivative term for theta
        double error_diff_theta = (error_theta - last_error_theta_) / dt_;
        last_error_theta_ = error_theta;

        // Compute control force for theta
        double force_theta = kp_theta_ * error_theta + ki_theta_ * error_sum_theta_ + kd_theta_ * error_diff_theta;

        // Compute error for x
        double error_x = target_x_ - current_x;

        // Integral term for x
        error_sum_x_ += error_x * dt_;

        // Derivative term for x
        double error_diff_x = (error_x - last_error_x_) / dt_;
        last_error_x_ = error_x;

        // Compute control force for x
        double force_x = kp_x_ * error_x + ki_x_ * error_sum_x_ + kd_x_ * error_diff_x;

        // Combine forces
        double force = force_theta + force_x;

        // Apply limits to the control force
        if (force > max_force_) {
            force = max_force_;
        } else if (force < min_force_) {
            force = min_force_;
        }

        // Publish control force
        inverted_pendulum_sim::ControlForce control_msg;
        control_msg.force = force;
        control_force_pub_.publish(control_msg);
    }

    void run() {
        ros::Rate rate(1.0 / dt_);
        while (ros::ok()) {
            ros::spinOnce();
            rate.sleep();
        }
    }

private:
    ros::NodeHandle nh_;
    ros::Subscriber current_state_sub_;
    ros::Publisher control_force_pub_;

    // PID parameters for theta (pendulum angle)
    double kp_theta_;
    double ki_theta_;
    double kd_theta_;

    // PID parameters for x (cart position)
    double kp_x_;
    double ki_x_;
    double kd_x_;

    // Force limits
    double max_force_;
    double min_force_;

    // PID variables for theta
    double error_sum_theta_;
    double last_error_theta_;

    // PID variables for x
    double error_sum_x_;
    double last_error_x_;

    // Control loop parameters
    double dt_;

    // Target angles and positions
    double target_theta_; // Target theta (vertical position)
    double target_x_; // Target x position
};

int main(int argc, char **argv) {
    ros::init(argc, argv, "pid_controller");
    PIDController controller;
    controller.run();
    return 0;
}
