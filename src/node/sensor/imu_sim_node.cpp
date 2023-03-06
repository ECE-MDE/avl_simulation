//==============================================================================
// Autonomous Vehicle Library
//
// Description: ROS node to simulate microstrain IMU data from true simulation
//              data.
//
// Servers:     None
//
// Clients:     None
//
// Publishers:  device/imu (avl_devices/ImuMsg)
//
// Subscribers: /sim/state (avl_simulation/VehicleStateMsg)
//==============================================================================

// Base node class
#include <avl_core/node.h>

// Util functions
#include <avl_core/util/math.h>

// Standard sensor error model
#include <avl_simulation/sensor_model.h>

// ROS message includes
#include <avl_msgs/VehicleStateMsg.h>
#include <avl_msgs/ImuMsg.h>
using namespace avl_msgs;

// Eigen library
#include <Eigen/Dense>
using namespace Eigen;

//==============================================================================
//                              NODE DEFINITION
//==============================================================================

class ImuSimNode : public Node
{

public:

    //--------------------------------------------------------------------------
    // Name:        ImuSimNode constructor
    //--------------------------------------------------------------------------
    ImuSimNode(int argc, char **argv) : Node(argc, argv)
    {

    }

private:

    // Publisher for simulated AHRS data
    ros::Publisher imu_pub;

    // Subscriber for vehicle state
    ros::Subscriber state_sub;

    // Sensor error models for the gyroscope and accelerometer measurements
    SensorModel gyro_err;
    SensorModel accel_err;

    // IMU measurements
    Vector3d w_ib_b;
    Vector3d f_ib_b;

    // Timer for sensor output rate
    ros::Timer iterate_timer;
    double dt;

    // Flag indicating whether the vehicle state has been initialized. Sensor
    // will not be iterated until vehicle state is initialized
    bool state_initialized = false;

private:

    //--------------------------------------------------------------------------
    // Name:        state_msg_callback
    // Description: Called when a message is received on the topic.
    // Arguments:   - message: Message received on the topic.
    //--------------------------------------------------------------------------
    void state_msg_callback(const VehicleStateMsg message)
    {
        state_initialized = true;
        w_ib_b(0) = message.w_ib_b_x;
        w_ib_b(1) = message.w_ib_b_y;
        w_ib_b(2) = message.w_ib_b_z;
        f_ib_b(0) = message.f_ib_b_x;
        f_ib_b(1) = message.f_ib_b_y;
        f_ib_b(2) = message.f_ib_b_z;
    }

    //--------------------------------------------------------------------------
    // Name:        iterate_timer_callback
    // Description: Called when the sensor iteration timer expires.
    // Arguments:   - event: ROS timer event structure.
    //--------------------------------------------------------------------------
    void iterate_timer_callback(const ros::TimerEvent& event)
    {

        if (state_initialized)
        {

            // Add sensor noise
            Vector3d w_ib_b_meas = gyro_err.add_error(w_ib_b);
            Vector3d f_ib_b_meas = accel_err.add_error(f_ib_b);

            // Create and publish the simulated IMU message
            ImuMsg imu_msg;
            imu_msg.dt = dt;
            imu_msg.angular_velocity.x = w_ib_b_meas(0);
            imu_msg.angular_velocity.y = w_ib_b_meas(1);
            imu_msg.angular_velocity.z = w_ib_b_meas(2);
            imu_msg.linear_acceleration.x = f_ib_b_meas(0);
            imu_msg.linear_acceleration.y = f_ib_b_meas(1);
            imu_msg.linear_acceleration.z = f_ib_b_meas(2);
            imu_msg.temperature = 20.0;
            imu_msg.valid = true;
            imu_pub.publish(imu_msg);

            // Log the data
            log_data("[w_ib_b] %.9f %.9f %.9f",
                w_ib_b_meas(0),
                w_ib_b_meas(1),
                w_ib_b_meas(2));

            log_data("[f_ib_b] %.9f %.9f %.9f",
                f_ib_b_meas(0),
                f_ib_b_meas(1),
                f_ib_b_meas(2));

        }

    }

    //--------------------------------------------------------------------------
    // Name:        init
    // Description: Initializes the node. Called when the node is started.
    //--------------------------------------------------------------------------
    void init()
    {

        // Add data log headers
        add_data_header("[f_ib_b] f_{ib,x}^b f_{ib,y}^b f_{ib,z}^b");
        add_data_header("[f_ib_b] m/s^2 m/s^2 m/s^2");
        add_data_header("[w_ib_b] \\omega_{ib,x}^b \\omega_{ib,y}^b \\omega_{ib,z}^b");
        add_data_header("[w_ib_b] rad/s rad/s rad/s");

        // Configure the sensor error models
        gyro_err = SensorModel::from_config_file("gyro_err");
        accel_err = SensorModel::from_config_file("accel_err");

        // Set up the publishers and subscribers
        imu_pub  = node_handle->advertise<ImuMsg>("device/imu", 1);
        state_sub = node_handle->subscribe("sim/state", 1,
            &ImuSimNode::state_msg_callback, this);

        // Set up the iteration timer. Creating the timer also starts it
        dt = 1.0/get_param<float>("~iteration_rate");
        iterate_timer = node_handle->createTimer(ros::Duration(dt),
            &ImuSimNode::iterate_timer_callback, this);

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
    ImuSimNode node(argc, argv);
    node.start();
    return 0;
}
