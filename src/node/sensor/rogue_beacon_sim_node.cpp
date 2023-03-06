//==============================================================================
// Autonomous Vehicle Library
//
// Description: ROS node to simulate range measurements from a statis beacon.
//
// Servers:     None
//
// Clients:     None
//
// Publishers:  nav/range (avl_navigation/RangeMsg)
//
// Subscribers: /sim/p_b (geometry_msgs/Vector3)
//==============================================================================

// Node base class
#include <avl_core/node.h>

// Utility functions
#include <avl_core/util/math.h>
#include <avl_navigation/algorithm/inertial_nav.h>

// Standard sensor error model
#include <avl_simulation/sensor_model.h>

// ROS message includes
#include <avl_msgs/VehicleStateMsg.h>
#include <avl_msgs/RangeMsg.h>
using namespace avl_msgs;

// Eigen library
#include <Eigen/Dense>
using namespace Eigen;

//==============================================================================
//                              NODE DEFINITION
//==============================================================================

class RogueBeaconSimNode : public Node
{

public:

    //--------------------------------------------------------------------------
    // Name:        RogueBeaconSimNode constructor
    //--------------------------------------------------------------------------
    RogueBeaconSimNode(int argc, char **argv) : Node(argc, argv)
    {

    }

private:

    // Publisher for simulated range measurements
    ros::Publisher range_pub;

    // Subscriber for vehicle state
    ros::Subscriber state_sub;

    // Sensor error model for the range measurements
    SensorModel range_err;

    // Vehicle and beacon positions in radians and meters
    Vector3d p_b;
    Vector3d p_beacon0;
    Vector3d v_beacon;
    Vector3d p_beacon;

    // Timer for sensor output rate
    double dt;
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
    void state_msg_callback(VehicleStateMsg message)
    {
        state_initialized = true;
        p_b(0) = message.lat;
        p_b(1) = message.lon;
        p_b(2) = message.alt;
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

            // Calculate the range from the beacon to the vehicle
            VectorXd range(1);
            range << linear_dist(p_beacon, p_b, true);

            // Add sensor noise
            VectorXd range_meas = range_err.add_error(range);

            // Create and publish the simulated range measurement message
            RangeMsg range_msg;
            range_msg.lat = p_beacon0(0);
            range_msg.lon = p_beacon0(1);
            range_msg.alt = p_beacon0(2);
            range_msg.range = range_meas(0);
            range_pub.publish(range_msg);

            log_data("[position] %f %f %f %f %f %f",
                p_beacon0(0),
                p_beacon0(1),
                p_beacon0(2),
                p_beacon(0),
                p_beacon(1),
                p_beacon(2));

            log_data("[velocity] v_N v_E v_D",
                v_beacon(0),
                v_beacon(1),
                v_beacon(2));

            log_data("[range] %f", range_msg.range);

            // Move the beacon according to its velocity

            double R_E = 6371000.0;
            double vN = v_beacon(0);
            double vE = v_beacon(1);
            double vD = v_beacon(2);
            double lat = avl::deg_to_rad(p_beacon(0));
            double lon = avl::deg_to_rad(p_beacon(1));
            double alt = p_beacon(2);

            // Calculate geodetic coordinates rate of change
            double lat_dot = 1 / (R_E + alt)*vN;
            double lon_dot = 1 / ((R_E + alt)*cos(lat))*vE;
            double alt_dot = -1 * vD;

            // Integrate geodetic coordinates rate of change
            lat = lat + lat_dot*dt;
            lon = lon + lon_dot*dt;
            alt = alt + alt_dot*dt;

            p_beacon(0) = avl::rad_to_deg(lat);
            p_beacon(1) = avl::rad_to_deg(lon);
            p_beacon(2) = alt;

        }
    }

    //--------------------------------------------------------------------------
    // Name:        init
    // Description: Initializes the node. Called when the node is started.
    //--------------------------------------------------------------------------
    void init()
    {

        // Add data log headers
        add_data_header("[position] lat0 lon0 alt0 lat lon alt");
        add_data_header("[position] deg deg m deg deg m");
        add_data_header("[velocity] v_N v_E v_D");
        add_data_header("[velocity] m/s m/s m/s");
        add_data_header("[range] range");
        add_data_header("[range] m");

        // Configure the sensor error model
        range_err = SensorModel::from_config_file("range_err");

        // Set up the publishers and subscribers
        range_pub = node_handle->advertise<RangeMsg>("nav/range", 1);
        state_sub = node_handle->subscribe("sim/state", 1,
            &RogueBeaconSimNode::state_msg_callback, this);

        // Get the beacon configuration from the config file
        p_beacon0(0) = get_param<double>("~beacon_lat");
        p_beacon0(1) = get_param<double>("~beacon_lon");
        p_beacon0(2) = get_param<double>("~beacon_alt");
        p_beacon(0) = p_beacon0(0);
        p_beacon(1) = p_beacon0(1);
        p_beacon(2) = p_beacon0(2);
        v_beacon(0) = get_param<double>("~v_beacon_N");
        v_beacon(1) = get_param<double>("~v_beacon_E");
        v_beacon(2) = get_param<double>("~v_beacon_D");

        // Set up the iteration timer. Creating the timer also starts it
        dt = 1.0/get_param<float>("~iteration_rate");
        iterate_timer = node_handle->createTimer(ros::Duration(dt),
            &RogueBeaconSimNode::iterate_timer_callback, this);

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
    RogueBeaconSimNode node(argc, argv);
    node.start();
    return 0;
}
