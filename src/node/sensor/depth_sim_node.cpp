//==============================================================================
// Autonomous Vehicle Library
//
// Description: ROS node to simulate simple depth sensor data from true
//              simulation data.
//
// Servers:     None
//
// Clients:     None
//
// Publishers:  device/depth (std_msgs/Float64)
//
// Subscribers: /sim/p_b (geometry_msgs/Vector3)
//==============================================================================

// Node base class
#include <avl_core/node.h>

// Standard sensor error model
#include <avl_simulation/sensor_model.h>

// ROS message includes
#include <avl_msgs/VehicleStateMsg.h>
#include <std_msgs/Float64.h>
#include <std_msgs/Bool.h>
using namespace avl_msgs;

// Eigen library
#include <Eigen/Dense>
using namespace Eigen;

//==============================================================================
//                              NODE DEFINITION
//==============================================================================

class DepthSimNode : public Node
{

public:

    //--------------------------------------------------------------------------
    // Name:        DepthSimNode constructor
    //--------------------------------------------------------------------------
    DepthSimNode(int argc, char **argv) : Node(argc, argv)
    {

    }

private:

    // Publisher for simulated depth data
    ros::Publisher depth_pub;

    // Subscriber for vehicle state
    ros::Subscriber state_sub;

    // Sensor error model for the depth measurement
    SensorModel depth_err;

    // Vehicle state variables
    double alt;
    double alt_sealevel;

    // Timer for sensor output rate
    ros::Timer iterate_timer;

    // Flag indicating whether the vehicle state has been initialized. Sensor
    // will not be iterated until vehicle state is initialized
    bool state_initialized = false;

    // Flags for fault gen
    bool fault_zero = false;
    bool fault_hold = false;
    bool fault_noisy = false;

    // Subscriber for fault gen state
    ros::Subscriber fault_zero_sub;
    ros::Subscriber fault_hold_sub;
    ros::Subscriber fault_noisy_sub;

    // Sensor error model for fault
    SensorModel fault_depth_err;
    VectorXd depth_meas;

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

            // Calculate the true depth sensor measurement value
            VectorXd depth(1);
            depth << alt_sealevel - alt;

            if(!fault_hold) {
                if(fault_noisy) {
                    depth_meas = fault_depth_err.add_error(depth);
                } else {
                    depth_meas = depth_err.add_error(depth);
                }
                if(fault_zero) {
                    depth_meas(0) = 0;
                }
            }

            // Create and publish the simulated depth sensor message
            std_msgs::Float64 depth_msg;
            depth_msg.data = depth_meas(0);
            depth_pub.publish(depth_msg);

            log_data("[depth] %.9f", depth_msg.data);

        }

    }

    //--------------------------------------------------------------------------
    // Name:        init
    // Description: Initializes the node. Called when the node is started.
    //--------------------------------------------------------------------------
    void init()
    {

        // Add data log headers
        add_data_header("[depth] depth");
        add_data_header("[depth] m");

        // Get the altitude of sealevel from the config file
        alt_sealevel = get_param<double>("~alt_sealevel");

        // Configure the sensor error model
        depth_err = SensorModel::from_config_file("depth_err");
        fault_depth_err = SensorModel::from_config_file("fault_depth_err");

        // Set up the publishers and subscribers
        depth_pub = node_handle->advertise<std_msgs::Float64>("device/depth", 1);
        state_sub = node_handle->subscribe("sim/state", 1,
            &DepthSimNode::state_msg_callback, this);

        fault_noisy_sub = node_handle->subscribe<std_msgs::Bool, const std_msgs::Bool &>("fault_gen/depth_sensor_noisy", 1, 
            [&](auto msg){fault_noisy = msg.data;});
        fault_zero_sub = node_handle->subscribe<std_msgs::Bool, const std_msgs::Bool &>("fault_gen/depth_sensor_zero", 1, 
            [&](auto msg){fault_zero = msg.data;});
        fault_hold_sub = node_handle->subscribe<std_msgs::Bool, const std_msgs::Bool &>("fault_gen/depth_sensor_hold", 1, 
            [&](auto msg){fault_hold = msg.data;});

        // Set up the iteration timer. Creating the timer also starts it
        double dt = 1.0/get_param<float>("~iteration_rate");
        iterate_timer = node_handle->createTimer(ros::Duration(dt),
            &DepthSimNode::iterate_timer_callback, this);

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
    DepthSimNode node(argc, argv);
    node.start();
    return 0;
}
