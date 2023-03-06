//==============================================================================
// Autonomous Vehicle Library
//
// Description: ROS node to simulate simple GPS position data from true
//              simulation data.
//
// Servers:     None
//
// Clients:     None
//
// Publishers:  device/gps
//
// Subscribers: /sim/p_b (geometry_msgs/Vector3)
//==============================================================================

// Device node base class
#include <avl_devices/device_node.h>

// Standard sensor error model
#include <avl_simulation/sensor_model.h>

// ROS message includes
#include <avl_msgs/VehicleStateMsg.h>
#include <avl_msgs/GpsMsg.h>
using namespace avl_msgs;

// Eigen library
#include <Eigen/Dense>
using namespace Eigen;

// Util functions
#include <avl_core/util/geo.h>

//==============================================================================
//                              NODE DEFINITION
//==============================================================================

class GpsSimNode : public DeviceNode
{

public:

    //--------------------------------------------------------------------------
    // Name:        GpsSimNode constructor
    //--------------------------------------------------------------------------
    GpsSimNode(int argc, char **argv) : DeviceNode(argc, argv)
    {

    }

private:

    // Publisher for simulated GPS data
    ros::Publisher gps_pub;

    // Subscriber for vehicle state
    ros::Subscriber state_sub;

    // Sensor error model for the position measurement
    SensorModel pos_err;

    // Vehicle state variables
    Vector3d p_b;
    Vector3d v_eb_b;
    double yaw;

    // Timer for sensor output rate
    ros::Timer iterate_timer;

    // Flag indicating whether the vehicle state has been initialized. Sensor
    // will not be iterated until vehicle state is initialized
    bool state_initialized = false;

    // Altitude of sealevel in meters and GPS dropout depth from config file.
    // Used to cause GPS to lose lock at the specified depth
    double alt_sealevel;
    double gps_dropout_depth;

private:

    //--------------------------------------------------------------------------
    // Name:        state_msg_callback
    // Description: Called when a message is received on the topic.
    // Arguments:   - message: Message received on the topic.
    //--------------------------------------------------------------------------
    void state_msg_callback(const VehicleStateMsg message)
    {
        state_initialized = true;
        p_b(0) =    message.lat;
        p_b(1) =    message.lon;
        p_b(2) =    message.alt;
        v_eb_b(0) = message.v_eb_b_x;
        v_eb_b(1) = message.v_eb_b_y;
        v_eb_b(2) = message.v_eb_b_z;
        yaw = message.yaw;
    }

    //--------------------------------------------------------------------------
    // Name:        get_device_parameters
    // Description: Called when a DEVICE packet is requested from the node or
    //              when a DEVICE packet is needed to be published to the FSD
    //              or BSD interface. This function should be implemented by the
    //              device child class to create and return a parameter list.
    // Returns:     Parameter list containing parameters to be added to the
    //              device packet that will be transmitted.
    //--------------------------------------------------------------------------
    avl::ParameterList get_device_parameters()
    {

        avl::ParameterList params;
        params.add(Parameter("LAT",         p_b(0)));
        params.add(Parameter("LON",         p_b(1)));
        params.add(Parameter("ALT",         p_b(2)));
        params.add(Parameter("FIX QUALITY", 0));
        params.add(Parameter("NUM SATS",    6));
        return params;

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

            // If altitude is below sealevel and GPS lock underwater is
            // disabled, there should be no GPS lock
            GpsMsg gps_msg;
            if (p_b(2) < (alt_sealevel - gps_dropout_depth))
            {

                // Create and publish the simulated GPS message
                gps_msg.lat = NAN;
                gps_msg.lon = NAN;
                gps_msg.alt = NAN;
                gps_msg.ground_speed = NAN;
                gps_msg.track_angle = NAN;
                gps_msg.fix_quality = 0;
                gps_msg.num_sats = 0;
                gps_msg.hdop = 0;
                gps_pub.publish(gps_msg);

            }
            else
            {

                // Calculate ground speed and track angle
                double vN = v_eb_b(0)*cos(yaw) + v_eb_b(1)*sin(yaw);
                double vE = v_eb_b(0)*sin(yaw) - v_eb_b(1)*cos(yaw);

                // Add sensor noise
                Vector3d p_b_meas = pos_err.add_error(p_b);

                // Create and publish the simulated GPS message
                gps_msg.lat = p_b_meas(0);
                gps_msg.lon = p_b_meas(1);
                gps_msg.alt = p_b_meas(2);
                gps_msg.ground_speed = sqrt(vN*vN + vE*vE);
                gps_msg.track_angle = atan2(vE, vN);
                gps_msg.fix_quality = 1;
                gps_msg.num_sats = 10; // Arbitrary number
                gps_msg.hdop = 1; // Arbitrary number
                gps_pub.publish(gps_msg);

            }

            log_data("[gps] %.9f %.9f %.1f %.4f %.4f %d %d %d",
                gps_msg.lat,
                gps_msg.lon,
                gps_msg.alt,
                gps_msg.ground_speed,
                gps_msg.track_angle,
                gps_msg.fix_quality,
                gps_msg.num_sats,
                gps_msg.hdop);

        }
    }

    //--------------------------------------------------------------------------
    // Name:        init
    // Description: Initializes the node. Called when the node is started.
    //--------------------------------------------------------------------------
    void init()
    {

        // Add data log headers
        add_data_header("[gps] lat lon alt ground\\_speed track\\_angle fix\\_quality num\\_sats hdop");
        add_data_header("[gps] rad rad m m/s rad NA num NA");

        // Get the altitude of sealevel and GPS dropout depth from config file
        alt_sealevel = get_param<double>("~alt_sealevel");
        gps_dropout_depth = get_param<double>("~gps_dropout_depth");

        // Configure the sensor error models
        pos_err = SensorModel::from_config_file("pos_err");

        // Set up the publishers and subscribers
        gps_pub  = node_handle->advertise<GpsMsg>("device/gps", 1);
        state_sub = node_handle->subscribe("sim/state", 1,
            &GpsSimNode::state_msg_callback, this);

        // Set up the iteration timer. Creating the timer also starts it
        double dt = 1.0/get_param<float>("~iteration_rate");
        iterate_timer = node_handle->createTimer(ros::Duration(dt),
            &GpsSimNode::iterate_timer_callback, this);

        // Set device name and default DEVICE packet output rates
        set_device_name("Simulated GPS");
        set_device_packet_output_rate(1.0, INTERFACE_FSD);
        set_device_packet_output_rate(1.0, INTERFACE_BSD);

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
    GpsSimNode node(argc, argv);
    node.start();
    return 0;
}
