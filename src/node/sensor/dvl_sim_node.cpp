//==============================================================================
// Autonomous Vehicle Library
//
// Description: ROS node to simulate simple DVL velocity data from true
//              simulation data.
//
// Servers:     None
//
// Clients:     None
//
// Publishers:  device/velocity (geometry_msgs/Vector3)
//
// Subscribers: /sim/v_eb_b (geometry_msgs/Vector3)
//==============================================================================

// Device node base class
#include <avl_devices/device_node.h>

// Standard sensor error model
#include <avl_simulation/sensor_model.h>

// ROS message includes
#include <avl_msgs/VehicleStateMsg.h>
#include <avl_msgs/PathfinderDvlMsg.h>
#include <geometry_msgs/Vector3.h>
#include <std_msgs/Float64MultiArray.h>
#include <std_msgs/Float64.h>
using namespace avl_msgs;

// Util includes
#include <avl_core/util/math.h>

// Eigen library
#include <Eigen/Dense>
using namespace Eigen;

//==============================================================================
//                              NODE DEFINITION
//==============================================================================

class DvlSimNode : public DeviceNode
{

public:

    //--------------------------------------------------------------------------
    // Name:        DvlSimNode constructor
    //--------------------------------------------------------------------------
    DvlSimNode(int argc, char **argv) : DeviceNode(argc, argv)
    {

    }

private:

    // Publisher for simulated velocity data
    ros::Publisher vel_pub;
    ros::Publisher dvl_pub;

    // Publisher for the height generated in the Unreal simulation
    ros::Publisher height_pub;

    // Subscriber for vehicle state
    ros::Subscriber state_sub;

    // Subscriber for Unreal AUV DVL
    ros::Subscriber unreal_dvl_sub;

    // Sensor error model for the velocity measurement
    SensorModel vel_err;

    // Vehicle state variables
    Vector3d v_eb_b;

    // Pathfinder DVL has 4 beams from which we estimate the height
    VectorXd h_vec;

    // lat, lon, alt and roll, pitch, yaw, vectors
    Vector3d geo_vec;
    Vector3d euler_vec;
    Vector3d meas_geo_vec;
    Vector3d meas_euler_vec;

    // Other values measured by the DVL
    double salinity; // ppt
    double temperature; // C
    double depth; // m
    double sound_speed; // m/s

    // Janus angle of the dvl beams
    double janus_angle;

    // Timer for sensor output rate
    ros::Timer iterate_timer;

    // Flag indicating whether the vehicle state has been initialized. Sensor
    // will not be iterated until vehicle state is initialized
    bool state_initialized = false;

private:

    //--------------------------------------------------------------------------
    // Name:        state_msg_callback
    // Description: Called when a message is received on the topic.
    // Arguments:   - Message: message received on the topic.
    //--------------------------------------------------------------------------
    void state_msg_callback(const VehicleStateMsg message)
    {
        state_initialized = true;
        v_eb_b(0) = message.v_eb_b_x;
        v_eb_b(1) = message.v_eb_b_y;
        v_eb_b(2) = message.v_eb_b_z;

        // Save the most recent states
        geo_vec(0) = message.lat;
        geo_vec(1) = message.lon;
        geo_vec(2) = message.alt;
        euler_vec(0) = message.roll;
        euler_vec(1) = message.pitch;
        euler_vec(2) = message.yaw;
    }

    //--------------------------------------------------------------------------
    // Name:        dvl_msg_callback
    // Description: Called when a message is received on the topic.
    // Arguments:   - message: Message received on the topic.
    //--------------------------------------------------------------------------
    void unreal_dvl_msg_callback(const std_msgs::Float64MultiArray message)
    {

        // Cosine of the janus angle
        double c_j = std::cos(avl::deg_to_rad(janus_angle));
        // double pressure = std::stod(split_line.at(1));
        double h1 = c_j*message.data.at(0);
        double h2 = c_j*message.data.at(1);
        double h3 = c_j*message.data.at(2);
        double h4 = c_j*message.data.at(3);

        // Calculate altitude from the 4 slant ranges per page 35 of the
        // Pathfinder DVL Operation Manual
        double h = (h1*h2) / (h1+h2) + (h3*h4) / (h3+h4);

        // Gather the height values for device packet updates
        h_vec(0) = h1;
        h_vec(1) = h2;
        h_vec(2) = h3;
        h_vec(3) = h4;
        h_vec(4) = h;
        meas_geo_vec = geo_vec;
        meas_euler_vec = euler_vec;

        // Create and publish a height message
        std_msgs::Float64 height_msg;
        height_msg.data = h;
        height_pub.publish(height_msg);

        log_data("[height] %f %f %f %f %f", h, h1, h2, h3, h4);

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
        params.add(Parameter("Vx", v_eb_b(0)));
        params.add(Parameter("Vy", v_eb_b(1)));
        params.add(Parameter("Vz", v_eb_b(2)));
        params.add(Parameter("H1", h_vec(0)));
        params.add(Parameter("H2", h_vec(1)));
        params.add(Parameter("H3", h_vec(2)));
        params.add(Parameter("H4", h_vec(3)));
        params.add(Parameter("H", h_vec(4)));
        params.add(Parameter("SALINITY", salinity));
        params.add(Parameter("TEMPERATURE", temperature));
        params.add(Parameter("DEPTH", depth));
        params.add(Parameter("SOUND_SPEED", sound_speed));
        params.add(Parameter("LAT", meas_geo_vec(0)));
        params.add(Parameter("LON", meas_geo_vec(1)));
        params.add(Parameter("ALT", meas_geo_vec(2)));
        params.add(Parameter("ROLL", meas_euler_vec(0)));
        params.add(Parameter("PITCH", meas_euler_vec(1)));
        params.add(Parameter("YAW", meas_euler_vec(2)));
        params.add(Parameter("JANUS", 30.0));

        return params;

    }

    //--------------------------------------------------------------------------
    // Name:        iterate_timer_callback
    // Description: Called when the sensor iteration timer expires.
    // Arguments:   - event: ROS timer event structure
    //--------------------------------------------------------------------------
    void iterate_timer_callback(const ros::TimerEvent& event)
    {

        if (state_initialized)
        {

            // Add sensor noise
            Vector3d v_eb_b_meas = vel_err.add_error(v_eb_b);

            // Create and publish the simulated velocity message
            geometry_msgs::Vector3 vel_msg;
            vel_msg.x = v_eb_b_meas(0);
            vel_msg.y = v_eb_b_meas(1);
            vel_msg.z = v_eb_b_meas(2);
            vel_pub.publish(vel_msg);

            // Create and publish the simulated dvl message
            avl_msgs::PathfinderDvlMsg dvl_msg;
            dvl_msg.vx = v_eb_b_meas(0);
            dvl_msg.vy = v_eb_b_meas(1);
            dvl_msg.vz = v_eb_b_meas(2);
            dvl_msg.ve = 0.0;
            dvl_msg.valid = true;
            dvl_pub.publish(dvl_msg);

            log_data("[v_eb_b] %.9f %.9f %.9f",
                vel_msg.x, vel_msg.y, vel_msg.z);

        }

    }

    //--------------------------------------------------------------------------
    // Name:        init
    // Description: Initializes the node. Called when the node is started.
    //--------------------------------------------------------------------------
    void init()
    {

        // Add data log headers
        add_data_header("[v_eb_b] v_x v_y v_z");
        add_data_header("[v_eb_b] m/s m/s /ms");

        // Configure the sensor error models
        vel_err = SensorModel::from_config_file("vel_err");

        // Set the Janus angle of the dvl beams
        janus_angle = get_param<double>("~janus_angle");

        // Set up the publishers and subscribers
        vel_pub = node_handle->advertise<geometry_msgs::Vector3>("device/velocity", 1);
        dvl_pub = node_handle->advertise<avl_msgs::PathfinderDvlMsg>("device/dvl", 1);
        height_pub = node_handle->advertise<std_msgs::Float64>("device/height", 1);
        state_sub = node_handle->subscribe("sim/state", 1,
            &DvlSimNode::state_msg_callback, this);

        // Set up the AVL unreal dvl data subscriber
        unreal_dvl_sub = node_handle->subscribe("unreal/dvl", 100,
            &DvlSimNode::unreal_dvl_msg_callback, this);

        // Set up the iteration timer. Creating the timer also starts it
        double dt = 1.0/get_param<float>("~iteration_rate");
        iterate_timer = node_handle->createTimer(ros::Duration(dt),
            &DvlSimNode::iterate_timer_callback, this);

        // Set the size of the height vector to match the number of beams
        // on the pathfinder DVL + 1 for the height estimate
        h_vec.resize(5);

        // Set device name and default DEVICE packet output rates
        set_device_name("Simulated DVL");
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
    DvlSimNode node(argc, argv);
    node.start();
    return 0;
}
