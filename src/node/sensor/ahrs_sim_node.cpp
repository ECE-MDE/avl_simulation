//==============================================================================
// Autonomous Vehicle Library
//
// Description: ROS node to simulate microstrain AHRS data from true simulation
//              data.
//
// Servers:     None
//
// Clients:     None
//
// Publishers:  device/ahrs (avl_devices/AhrsMsg)
//
// Subscribers: /sim/theta_n_b (geometry_msgs/Vector3)
//              /sim/w_ib_b (geometry_msgs/Vector3)
//              /sim/f_ib_b (geometry_msgs/Vector3)
//==============================================================================

// Base node class
#include <avl_core/node.h>

// Util functions
#include <avl_core/util/math.h>
#include <avl_core/util/matrix.h>

// Standard sensor error model
#include <avl_simulation/sensor_model.h>

// ROS message includes
#include <avl_msgs/VehicleStateMsg.h>
#include <avl_msgs/AhrsMsg.h>
using namespace avl_msgs;

// Eigen library
#include <Eigen/Dense>
using namespace Eigen;

//==============================================================================
//                              NODE DEFINITION
//==============================================================================

class AhrsSimNode : public Node
{

public:

    //--------------------------------------------------------------------------
    // Name:        AhrsSimNode constructor
    //--------------------------------------------------------------------------
    AhrsSimNode(int argc, char **argv) : Node(argc, argv)
    {

    }

private:

    // Publisher for simulated AHRS data
    ros::Publisher ahrs_pub;

    // Subscriber for vehicle state
    ros::Subscriber state_sub;

    // Sensor error models for the AHRS measurements
    SensorModel att_err;
    SensorModel gyro_err;
    SensorModel accel_err;
    SensorModel mag_err;

    // Vehicle state variables
    Vector3d theta_n_b;
    Vector3d w_ib_b;
    Vector3d f_ib_b;
    Vector3d m_b;

    // Timer for sensor output rate
    ros::Timer iterate_timer;

    // Flag indicating whether the vehicle state has been initialized. Sensor
    // will not be iterated until vehicle state is initialized
    bool state_initialized = false;

    // Total magnetic field strength in Gauss from config file
    double mag_field_strength;

private:

    //--------------------------------------------------------------------------
    // Name:        state_msg_callback
    // Description: Called when a message is received on the topic.
    // Arguments:   - message: Message received on the topic.
    //--------------------------------------------------------------------------
    void state_msg_callback(const VehicleStateMsg message)
    {

        state_initialized = true;
        theta_n_b(0) = message.roll;
        theta_n_b(1) = message.pitch;
        theta_n_b(2) = message.yaw;
        w_ib_b(0) = message.w_ib_b_x;
        w_ib_b(1) = message.w_ib_b_y;
        w_ib_b(2) = message.w_ib_b_z;
        f_ib_b(0) = message.f_ib_b_x;
        f_ib_b(1) = message.f_ib_b_y;
        f_ib_b(2) = message.f_ib_b_z;

        // Get the magnetic field measurement by rotating the magnetic field
        // vector in the navigation frame to the body frame
        Matrix3d C_n_b = avl::euler_to_matrix(theta_n_b);
        Vector3d m_n = {mag_field_strength, 0.0, 0.0};
        m_b = C_n_b*m_n;

    }

    //--------------------------------------------------------------------------
    // Name:        iterate_timer_callback
    // Description: Called when the sensor iteration timer expires.
    // Arguments:   - event: ROS timer event structure.
    //--------------------------------------------------------------------------
    void iterate_timer_callback(const ros::TimerEvent& event)
    {

        // Only iterate if initialized with initial message values
        if (state_initialized)
        {

            // Add sensor noise
            Vector3d theta_n_b_meas = att_err.add_error(theta_n_b);
            Vector3d w_ib_b_meas = gyro_err.add_error(w_ib_b);
            Vector3d f_ib_b_meas = accel_err.add_error(f_ib_b);
            Vector3d m_b_meas = mag_err.add_error(m_b);

            // Create and publish the simulated AHRS message
            AhrsMsg ahrs_msg;
            ahrs_msg.theta.x = theta_n_b_meas(0);
            ahrs_msg.theta.y = theta_n_b_meas(1);
            ahrs_msg.theta.z = avl::wrap_to_2pi(theta_n_b_meas(2));
            ahrs_msg.w.x = w_ib_b_meas(0);
            ahrs_msg.w.y = w_ib_b_meas(1);
            ahrs_msg.w.z = w_ib_b_meas(2);
            ahrs_msg.a.x = f_ib_b_meas(0);
            ahrs_msg.a.y = f_ib_b_meas(1);
            ahrs_msg.a.z = f_ib_b_meas(2);
            ahrs_msg.mag.x = m_b_meas(0);
            ahrs_msg.mag.y = m_b_meas(1);
            ahrs_msg.mag.z = m_b_meas(2);
            ahrs_pub.publish(ahrs_msg);

            // Log the data
            log_data("[theta_n_b] %.9f %.9f %.9f",
                ahrs_msg.theta.x, ahrs_msg.theta.y, ahrs_msg.theta.z);
            log_data("[f_ib_b] %.9f %.9f %.9f",
                ahrs_msg.a.x, ahrs_msg.a.y, ahrs_msg.a.z);
            log_data("[w_ib_b] %.9f %.9f %.9f",
                ahrs_msg.w.x, ahrs_msg.w.y, ahrs_msg.w.z);
            log_data("[m_b] %.9f %.9f %.9f",
                ahrs_msg.mag.x, ahrs_msg.mag.y, ahrs_msg.mag.z);

        }

    }

    //--------------------------------------------------------------------------
    // Name:        init
    // Description: Initializes the node. Called when the node is started.
    //--------------------------------------------------------------------------
    void init()
    {

        // Add data log headers
        add_data_header("[theta_n_b] roll pitch yaw");
        add_data_header("[theta_n_b] rad rad rad");
        add_data_header("[f_ib_b] f_{ib,x}^b f_{ib,y}^b f_{ib,z}^b");
        add_data_header("[f_ib_b] m/s^2 m/s^2 m/s^2");
        add_data_header("[w_ib_b] \\omega_{ib,x}^b \\omega_{ib,y}^b \\omega_{ib,z}^b");
        add_data_header("[w_ib_b] rad/s rad/s rad/s");
        add_data_header("[m_b] m_x m_y m_z");
        add_data_header("[m_b] gauss gauss gauss");

        // Get the total magnetic field strength from the config file
        mag_field_strength = get_param<double>("~mag_field_strength");

        // Configure the sensor error models
        att_err = SensorModel::from_config_file("att_err");
        gyro_err = SensorModel::from_config_file("gyro_err");
        accel_err = SensorModel::from_config_file("accel_err");
        mag_err = SensorModel::from_config_file("mag_err");

        // Set up the publishers and subscribers
        ahrs_pub = node_handle->advertise<AhrsMsg>("device/ahrs", 1);
        state_sub = node_handle->subscribe("sim/state", 1,
            &AhrsSimNode::state_msg_callback, this);

        // Set up the iteration timer. Creating the timer also starts it
        double dt = 1.0/get_param<float>("~iteration_rate");
        iterate_timer = node_handle->createTimer(ros::Duration(dt),
            &AhrsSimNode::iterate_timer_callback, this);

    }

    //--------------------------------------------------------------------------
    // Name:        run
    // Description: Main node loop. Called after the node is initialized.
    //--------------------------------------------------------------------------
    void run()
    {
        ros::spin();
    }

};

//==============================================================================
//                                  MAIN
//==============================================================================
int main(int argc, char **argv)
{
    AhrsSimNode node(argc, argv);
    node.start();
    return 0;
}
