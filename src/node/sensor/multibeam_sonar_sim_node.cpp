//==============================================================================
// Autonomous Vehicle Library
//
// Description: ROS node to simulate multibeam sonar data using a height map.
//
// Servers:     None
//
// Clients:     None
//
// Publishers:  device/multibeam (avl_devices/MultibeamMsg)
//
// Subscribers: /sim/state (avl_simulation/VehicleStateMsg)
//==============================================================================

// Node base class
#include <avl_core/node.h>

// Inertial nav kinematics equations
#include <avl_navigation/algorithm/inertial_nav.h>

// Standard sensor error model
#include <avl_simulation/sensor_model.h>

// Terrain map class
#include <avl_navigation/algorithm/terrain_map.h>

// ROS message includes
#include <avl_msgs/VehicleStateMsg.h>
#include <avl_msgs/MultibeamMsg.h>
using namespace avl_msgs;

//==============================================================================
//                              NODE DEFINITION
//==============================================================================

class MultibeamSonarSimNode : public Node
{

public:

    //--------------------------------------------------------------------------
    // Name:        MultibeamSonarSimNode constructor
    //--------------------------------------------------------------------------
    MultibeamSonarSimNode(int argc, char **argv) : Node(argc, argv)
    {

    }

private:

    // Publisher for simulated multibeam data
    ros::Publisher multibeam_pub;

    // Subscriber for vehicle state
    ros::Subscriber state_sub;

    // Sensor error model for the multibeam measurement
    SensorModel multibeam_err;

    // Vehicle state variables
    Vector3d p_b;
    Vector3d theta_n_b;

    // Terrain map containing terrain altitude data
    TerrainMap map;

    // Sonar configuration
    int num_beams;
    double swath_width;
    MatrixXd p_beams_b;

    // Timer for sensor output rate
    ros::Timer iterate_timer;

    // Flag indicating whether the vehicle state has been initialized. Sensor
    // will not be iterated until vehicle state is initialized
    bool state_initialized = false;

private:

    //------------------------------------------------------------------------------
    // Name:        h_sonar
    // Description: Nonlinear measurement equation for a multibeam sonar measuring
    //              range of multiple beams. Designed for use as an update equation
    //              for a UKF. NOTE: ASsumes beams query straight down from a
    //              position in the body frame. Ray tracing is future work.
    // Arguments:   - x: Navigation state vector (15 x 1).
    //              - n: Sensor noise vector (1 x 1).
    //              - p_beams_b: Beam positions in the body frame in meters.
    //              - map: Pointer to terrain map to query.
    // Returns :    Beam range measurements generated from state vector (1 x 1).
    //------------------------------------------------------------------------------
    VectorXd h_sonar(VectorXd x, VectorXd n, MatrixXd p_beams_b, TerrainMap* map)
    {

        // Transform body frame beam positions to the navigation frame. Use only yaw
        Vector3d theta_n_b = {0.0, 0.0, x(2)};
        MatrixXd p_b = x.segment(6, 3);
        MatrixXd p_beams_n = transform_pos_b_to_n(p_beams_b, theta_n_b, p_b);

        // Calculate the range for each beam from the depth map
        VectorXd ranges(p_beams_n.cols());
        for (int i = 0; i < p_beams_n.cols(); i++)
        {
            double alt = map->alt(p_beams_n(0,i), p_beams_n(1,i));
            ranges(i) = p_beams_n(2,i) - alt;
        }

        return ranges + n;

    }

    //--------------------------------------------------------------------------
    // Name:        state_msg_callback
    // Description: Called when a message is received on the topic.
    // Arguments:   - message: Message received on the topic.
    //--------------------------------------------------------------------------
    void state_msg_callback(const VehicleStateMsg msg)
    {
        p_b = { msg.lat, msg.lon, msg.alt };
        theta_n_b = { msg.roll, msg.pitch, msg.yaw };
        state_initialized = true;
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

            // Use the sonar measurement function to generate the range
            // measurements
            VectorXd x = VectorXd::Zero(9);
            x.segment(0,3) = theta_n_b;
            x.segment(6,3) = p_b;
            VectorXd n = VectorXd::Zero(num_beams);
            VectorXd ranges = h_sonar(x, n, p_beams_b, &map);

            // Transform body frame beam positions to the navigation frame
            // for logging purposes
            theta_n_b = {0.0, 0.0, theta_n_b(2)};
            MatrixXd p_beams_n = transform_pos_b_to_n(p_beams_b, theta_n_b, p_b);

            // Add sensor noise to the true value
            ranges = multibeam_err.add_error(ranges);

            // Create and publish the simulated multibeam message
            MultibeamMsg multibeam_msg;
            multibeam_msg.x = avl::to_std_vector(p_beams_b.row(0));
            multibeam_msg.y = avl::to_std_vector(p_beams_b.row(1));
            multibeam_msg.range = avl::to_std_vector(ranges);
            multibeam_pub.publish(multibeam_msg);

            // Log the data
            std::stringstream ss;
            ss.precision(9);
            ss << "[multibeam] ";
            for (int i = 0; i < num_beams; i++)
                ss << " " << p_beams_n(0,i)
                   << " " << p_beams_n(1,i)
                   << " " << ranges(i);
            log_data(ss.str());

        }
    }

    //--------------------------------------------------------------------------
    // Name:        init
    // Description: Initializes the node. Called when the node is started.
    //--------------------------------------------------------------------------
    void init()
    {

        // Load the terrain map
        map.load(get_param<std::string>("~terrain_map_filepath"));

        // Get the beam configuration
        num_beams = get_param<int>("~num_beams");
        swath_width = get_param<double>("~swath_width");
        p_beams_b = MatrixXd::Zero(3, num_beams);
        p_beams_b.row(1) = avl::linspace(-swath_width/2.0,
                                          swath_width/2.0,
                                          num_beams);

        // Add data log headers
        std::stringstream names_ss;
        names_ss << "[multibeam]";
        for (int i = 0; i < num_beams; i++)
            names_ss << " lat" << i << " lon" << i << " height" << i;

        std::stringstream units_ss;
        units_ss << "[multibeam]";
        for (int i = 0; i < num_beams; i++)
            units_ss << " rad rad m";

        add_data_header(names_ss.str());
        add_data_header(units_ss.str());

        // Configure the sensor error model
        SensorConfig config;
        config.N = num_beams;
        config.N = num_beams;
        config.M = MatrixXd::Identity(num_beams, num_beams);
        config.b = VectorXd::Zero(num_beams);
        config.cov = get_param<double>("~cov")*VectorXd::Ones(num_beams);
        multibeam_err = SensorModel(config);

        // Set up the publishers and subscribers
        multibeam_pub = node_handle->advertise<MultibeamMsg>(
            "device/multibeam", 1);
        state_sub = node_handle->subscribe("sim/state", 1,
            &MultibeamSonarSimNode::state_msg_callback, this);

        // Set up the iteration timer. Creating the timer also starts it
        double dt = 1.0/get_param<float>("~iteration_rate");
        iterate_timer = node_handle->createTimer(ros::Duration(dt),
            &MultibeamSonarSimNode::iterate_timer_callback, this);

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
    MultibeamSonarSimNode node(argc, argv);
    node.start();
    return 0;
}
