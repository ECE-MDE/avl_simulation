//==============================================================================
// Autonomous Vehicle Library
//
// Description: ROS node to simulate simple RPM sensor data from true
//              simulation data.
//
// Servers:     None
//
// Clients:     None
//
// Publishers:  device/rpm (std_msgs/Float64)
//
// Subscribers: /sim/state (avl_simulation/VehicleStateMsg)
//==============================================================================

// Node base class
#include <avl_core/node.h>

// Utility functions
#include <avl_core/util/math.h>

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

class RpmSimNode : public Node
{

public:

    //--------------------------------------------------------------------------
    // Name:        RpmSimNode constructor
    //--------------------------------------------------------------------------
    RpmSimNode(int argc, char **argv) : Node(argc, argv)
    {

    }

private:

    // Publisher for simulated RPM sensor data
    ros::Publisher rpm_pub;

    // Subscriber for vehicle state data
    ros::Subscriber state_sub;

    // Sensor error model for the RPM measurement
    SensorModel rpm_err;

    // RPM measurement
    double rpm;

    // Timer for sensor output rate
    ros::Timer iterate_timer;
    ros::Duration iterate_timer_duration;

    // Flag indicating whether an initial RPM message has been received.
    // We want to receive an initial message before iterating
    bool rpm_initialized = false;

private:

    //--------------------------------------------------------------------------
    // Name:        state_msg_callback
    // Description: Called when a message is received on the topic.
    // Arguments:   - message: Message received on the topic.
    //--------------------------------------------------------------------------
    void state_msg_callback(const VehicleStateMsg message)
    {
        rpm_initialized = true;
        rpm = message.rpm;
    }

    //--------------------------------------------------------------------------
    // Name:        iterate_timer_callback
    // Description: Called when the sensor iteration timer expires.
    // Arguments:   - event: ROS timer event structure.
    //--------------------------------------------------------------------------
    void iterate_timer_callback(const ros::TimerEvent& event)
    {
        if (rpm_initialized)
        {

            // Add sensor noise
            VectorXd rpm_vect(1);
            rpm_vect << rpm;
            VectorXd rpm_meas = rpm_err.add_error(rpm_vect);

            // Create and publish the simulated RPM sensor message
            std_msgs::Float64 rpm_msg;
            rpm_msg.data = avl::clamp(rpm_meas(0), 0.0, 5000.0);
            rpm_pub.publish(rpm_msg);

            log_data("[rpm] %.9f", rpm_msg.data);

        }
    }

    //--------------------------------------------------------------------------
    // Name:        init
    // Description: Initializes the node. Called when the node is started.
    //--------------------------------------------------------------------------
    void init()
    {

        // Add data log headers
        add_data_header("[rpm] RPM");
        add_data_header("[rpm] RPM");

        // Configure the sensor error model
        rpm_err = SensorModel::from_config_file("rpm_err");

        // Set up the publishers and subscribers
        rpm_pub = node_handle->advertise<std_msgs::Float64>("device/rpm", 1);
        state_sub = node_handle->subscribe("sim/state", 1,
            &RpmSimNode::state_msg_callback, this);

        // Set up the iteration timer. Creating the timer also starts it
        double dt = 1.0/get_param<float>("~iteration_rate");
        iterate_timer_duration = ros::Duration(dt);
        iterate_timer = node_handle->createTimer(iterate_timer_duration,
            &RpmSimNode::iterate_timer_callback, this);

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
    RpmSimNode node(argc, argv);
    node.start();
    return 0;
}
