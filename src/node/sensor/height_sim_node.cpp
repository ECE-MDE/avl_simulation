//==============================================================================
// Autonomous Vehicle Library
//
// Description: ROS node to simulate simple height sensor data from true
//              simulation data assuming a flat seafloor at a given altitude.
//
// Servers:     None
//
// Clients:     None
//
// Publishers:  device/height (std_msgs/Float64)
//
// Subscribers: /sim/state (avl_simulation/VehicleStateMsg)
//==============================================================================

// Node base class
#include <avl_core/node.h>

// Standard sensor error model
#include <avl_simulation/sensor_model.h>

// ROS message includes
#include <avl_msgs/VehicleStateMsg.h>
#include <std_msgs/Float64.h>
using namespace avl_msgs;

// Eigen library
#include <Eigen/Dense>
using namespace Eigen;

//==============================================================================
//                              NODE DEFINITION
//==============================================================================

class HeightSimNode : public Node
{

public:

    //--------------------------------------------------------------------------
    // Name:        HeightSimNode constructor
    //--------------------------------------------------------------------------
    HeightSimNode(int argc, char **argv) : Node(argc, argv)
    {

    }

private:

    // Publisher for simulated height data
    ros::Publisher height_pub;

    // Subscriber for vehicle state
    ros::Subscriber state_sub;

    // Sensor error model for the height measurement
    SensorModel height_err;

    // Vehicle state variables
    double alt;
    double alt_seafloor;

    // Timer for sensor output rate
    ros::Timer iterate_timer;

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
        alt = message.alt;
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

            // Calculate the true height sensor measurement value
            VectorXd height(1);
            height << alt - alt_seafloor;

            // Add height sensor noise to the true value
            VectorXd height_meas = height_err.add_error(height);

            // Create and publish the simulated height sensor message
            std_msgs::Float64 height_msg;
            height_msg.data = height_meas(0);
            height_pub.publish(height_msg);

            log_data("[height] %.9f", height_msg.data);

        }

    }

    //--------------------------------------------------------------------------
    // Name:        init
    // Description: Initializes the node. Called when the node is started.
    //--------------------------------------------------------------------------
    void init()
    {

        // Add data log headers
        add_data_header("[height] height");
        add_data_header("[height] m");

        // Get the altitude of the seafloor from the config file
        alt_seafloor = get_param<double>("~alt_seafloor");

        // Configure the sensor error model
        height_err = SensorModel::from_config_file("height_err");

        // Set up the publishers and subscribers
        height_pub = node_handle->advertise<std_msgs::Float64>("device/height", 1);
        state_sub = node_handle->subscribe("sim/state", 1,
            &HeightSimNode::state_msg_callback, this);

        // Set up the iteration timer. Creating the timer also starts it
        double dt = 1.0/get_param<float>("~iteration_rate");
        iterate_timer = node_handle->createTimer(ros::Duration(dt),
            &HeightSimNode::iterate_timer_callback, this);

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
    HeightSimNode node(argc, argv);
    node.start();
    return 0;
}
