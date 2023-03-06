//==============================================================================
// Autonomous Vehicle Library
//
// Description: Dynamics node base class that implements an iteration timer and
//              an iterate callback to be implemented by the user. The iterate
//              function provides dynamics inputs as arguments and requires that
//              the child class returns dynamics output message calculated from
//              the inputs.
//
// Servers:     None
//
// Clients:     None
//
// Publishers:  /sim/dynamics (avl_simulation/VehicleDynamicsOutputMsg)
//
// Subscribers: /sim/state (avl_simulation/VehicleStateMsg)
//              /sim/fins (avl_devices/FinsMsg)
//              /sim/rpm (std_msgs/Float64)
//==============================================================================

// Base node class
#include <avl_core/node.h>
#include <avl_core/util/math.h>
#include <avl_core/util/vector.h>
#include <avl_core/util/matrix.h>

// ROS message includes
#include <avl_devices/FinsMsg.h>
#include <std_msgs/Float64.h>
#include <avl_simulation/VehicleStateMsg.h>
#include <avl_simulation/VehicleDynamicsOutputMsg.h>
using namespace avl_devices;
using namespace std_msgs;
using namespace avl_simulation;

// Eigen library
#include <Eigen/Dense>
using namespace Eigen;

//==============================================================================
//                              NODE DEFINITION
//==============================================================================

class DynamicsNode : public Node
{

public:

    //--------------------------------------------------------------------------
    // Name:        DynamicsNode constructor
    //--------------------------------------------------------------------------
    DynamicsNode(int argc, char **argv) : Node(argc, argv)
    {

    }

    //--------------------------------------------------------------------------
    // Name:        iterate
    // Description: Called every time the dynamics node iterates. This function
    //              should use the dynamics inputs given in the function
    //              arguments to calculate and return a dynamics output message.
    // Arguments:   - vehicle_state: Vehicle state input.
    //              - fins_state: Fins state input.
    //              - dt: Time step in seconds.
    // Returns:     Dynamics output message calculated from the inputs.
    //--------------------------------------------------------------------------
    virtual VehicleDynamicsOutputMsg iterate(VehicleStateMsg vehicle_state,
        FinsMsg fins_state, double rpm, double dt) = 0;

private:

    // Publishers for simulation data
    ros::Publisher dynamics_pub;

    // Subscribers for input data
    ros::Subscriber state_sub;
    ros::Subscriber fins_sub;
    ros::Subscriber rpm_sub;

    // Input values for motor percent and fin angles
    VehicleStateMsg vehicle_state;
    FinsMsg fins_state;
    double rpm = 0.0;

    // Timer for dynamics iterations
    double dt;
    ros::Timer iterate_timer;

private:

    //--------------------------------------------------------------------------
    // Name:        state_msg_callback
    // Description: Called when a message is received on the topic.
    // Arguments:   - message: message received on the topic
    //--------------------------------------------------------------------------
    void state_msg_callback(const avl_simulation::VehicleStateMsg& message)
    {
        iterate_timer.start();
        vehicle_state = message;
    }

    //--------------------------------------------------------------------------
    // Name:        fins_msg_callback
    // Description: Called when a message is received on the topic.
    // Arguments:   - message: message received on the topic
    //--------------------------------------------------------------------------
    void fins_msg_callback(const avl_devices::FinsMsg& message)
    {
        fins_state = message;
    }

    //--------------------------------------------------------------------------
    // Name:        rpm_msg_callback
    // Description: Called when a message is received on the topic.
    // Arguments:   - message: message received on the topic
    //--------------------------------------------------------------------------
    void rpm_msg_callback(const std_msgs::Float64& message)
    {
        rpm = message.data;
    }

    //--------------------------------------------------------------------------
    // Name:        iterate_timer_callback
    // Description: Called when the iteration timer expires. Handles the
    //              iteration of the AUV state and publishes the results.
    // Arguments:   - event: ROS timer event structure
    //--------------------------------------------------------------------------
    void iterate_timer_callback(const ros::TimerEvent& event)
    {

        VehicleDynamicsOutputMsg dynamics_msg = iterate(vehicle_state,
            fins_state, rpm, dt);
        dynamics_pub.publish(dynamics_msg);

        log_data("[omega_nb_b] %+.9f %+.9f %+.9f",
            dynamics_msg.omega_nb_b_x,
            dynamics_msg.omega_nb_b_y,
            dynamics_msg.omega_nb_b_z);

        log_data("[f_nb_b] %+.9f %+.9f %+.9f",
            dynamics_msg.f_nb_b_x,
            dynamics_msg.f_nb_b_y,
            dynamics_msg.f_nb_b_z);

    }

    //--------------------------------------------------------------------------
    // Name:        run
    // Description: Main node loop. Called after the node is initialized.
    //--------------------------------------------------------------------------
    void run() final
    {

        // Log data headers
        log_data("[f_nb_b] f_{nb,x}^b f_{nb,y}^b f_{nb,z}^b");
        log_data("[f_nb_b] m/s^2 m/s^2 m/s^2");
        log_data("[omega_nb_b] \\omega_{nb,x}^b \\omega_{nb,y}^b \\omega_{nb,z}^b");
        log_data("[omega_nb_b] deg/s deg/s deg/s");

        // Publisher for dyanmics output
        dynamics_pub = node_handle->advertise<VehicleDynamicsOutputMsg>(
            "/sim/dynamics", 1);

        // Subscribers for input data
        state_sub = node_handle->subscribe("sim/state", 1,
            &DynamicsNode::state_msg_callback, this);
        fins_sub = node_handle->subscribe("sim/fins", 1,
            &DynamicsNode::fins_msg_callback, this);
        rpm_sub = node_handle->subscribe("sim/rpm", 1,
            &DynamicsNode::rpm_msg_callback, this);

        // Set up the iteration timer. Creating the timer also starts it
        dt = 1.0/get_param<float>("~iteration_rate");
        iterate_timer = node_handle->createTimer(ros::Duration(dt),
            &DynamicsNode::iterate_timer_callback, this);
        iterate_timer.stop();

        ros::Rate spin_rate(1000);
        while (ros::ok())
        {
            ros::spinOnce();
            spin_rate.sleep();
        }
    }

};
