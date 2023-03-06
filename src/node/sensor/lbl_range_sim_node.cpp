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

class LblRangeSimNode : public Node
{

public:

    //--------------------------------------------------------------------------
    // Name:        LblRangeSimNode constructor
    //--------------------------------------------------------------------------
    LblRangeSimNode(int argc, char **argv) : Node(argc, argv)
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
    std::vector<Vector3d> p_beacon;

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

            // Generate a and publish range measurement for each beacon
            for (size_t i = 0; i < p_beacon.size(); i++)
            {

                // Calculate the range from the beacon to the vehicle
                VectorXd range(1);
                range << linear_dist(p_beacon.at(i), p_b);

                // Add sensor noise
                VectorXd range_meas = range_err.add_error(range);

                // Create and publish the simulated range measurement messages
                RangeMsg range_msg;
                range_msg.lat = p_beacon.at(i)(0);
                range_msg.lon = p_beacon.at(i)(1);
                range_msg.alt = p_beacon.at(i)(2);
                range_msg.range = range_meas(0);
                range_pub.publish(range_msg);

                log_data("[beacon_%d] %f %f %f %.3f",
                    i,
                    range_msg.lat,
                    range_msg.lon,
                    range_msg.alt,
                    range_msg.range);

            }

        }
    }

    //--------------------------------------------------------------------------
    // Name:        init
    // Description: Initializes the node. Called when the node is started.
    //--------------------------------------------------------------------------
    void init()
    {

        // Configure the sensor error model
        range_err = SensorModel::from_config_file("range_err");

        // Get the beacon positions from the config file
        auto beacon_lats = get_param<std::vector<double>>("~beacon_lats");
        auto beacon_lons = get_param<std::vector<double>>("~beacon_lons");
        auto beacon_alts = get_param<std::vector<double>>("~beacon_alts");

        // Ensure there are the same number of lats, lons, and alts
        if (beacon_lats.size() != beacon_lons.size() ||
            beacon_lats.size() != beacon_alts.size())
        {
            throw std::runtime_error("beacon lats, lons, and alts vectors must"
                " be the same size");
        }

        // Add data log headers for each beacon
        for (size_t i = 0; i < beacon_lats.size(); i++)
        {
            std::string tag = "[beacon_" + std::to_string(i) + "]";
            add_data_header(tag + " lat lon alt range");
            add_data_header(tag + " deg deg m m");
        }

        // Put the beacon positions into Eigen vectors
        for (size_t i = 0; i < beacon_lats.size(); i++)
        {
            Vector3d pos;
            pos(0) = avl::deg_to_rad(beacon_lats.at(i));
            pos(1) = avl::deg_to_rad(beacon_lons.at(i));
            pos(2) = beacon_alts.at(i);
            p_beacon.push_back(pos);
        }

        // Set up the publishers and subscribers
        range_pub = node_handle->advertise<RangeMsg>("nav/range", 1);
        state_sub = node_handle->subscribe("sim/state", 1,
            &LblRangeSimNode::state_msg_callback, this);

        // Set up the iteration timer. Creating the timer also starts it
        double dt = 1.0/get_param<float>("~iteration_rate");
        iterate_timer = node_handle->createTimer(ros::Duration(dt),
            &LblRangeSimNode::iterate_timer_callback, this);

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
    LblRangeSimNode node(argc, argv);
    node.start();
    return 0;
}
