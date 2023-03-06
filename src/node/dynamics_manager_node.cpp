//==============================================================================
// Autonomous Vehicle Library
//
// Description: Manages vehicle dynamics calculations.
//
// Servers:     None
//
// Clients:     /sim/dynamics (avl_msgs/VehicleDynamicsSrv)
//
// Publishers:  /sim/state (avl_msgs/VehicleStateMsg)
//
// Subscribers: device/fins (avl_msgs/FinsMsg)
//              device/motor (std_msgs/Float64)
//==============================================================================

// Node base class
#include <avl_core/node.h>

// Util functions
#include <avl_core/util/math.h>
#include <avl_core/util/vector.h>
#include <avl_core/util/matrix.h>

// Inertial nav kinematics equations
#include <avl_navigation/algorithm/inertial_nav.h>

// ROS message includes
#include <avl_msgs/VehicleDynamicsSrv.h>
#include <avl_msgs/VehicleStateMsg.h>
#include <avl_msgs/FinsMsg.h>
#include <std_msgs/Float64.h>
using namespace avl_msgs;

// Declare matrix/vector types
typedef Matrix<double, 6, 6> Matrix6d;
typedef Matrix<double, 6, 1> Vector6d;

//==============================================================================
//                              NODE DEFINITION
//==============================================================================

class DynamicsManagerNode : public Node
{

public:

    //--------------------------------------------------------------------------
    // Name:        DynamicsManagerNode constructor
    //--------------------------------------------------------------------------
    DynamicsManagerNode(int argc, char **argv) : Node(argc, argv) { }

private:

    // Timer for dynamics iterations
    double dt;
    ros::Timer iterate_timer;

    // Vehicle dynamics service client for requesting dynamics iteration
    ros::ServiceClient dynamics_client;

    // Subscriber for fin angles and corresponding rudder and elevator angles in
    // radians
    ros::Subscriber fins_sub;
    double delta_r;
    double delta_e;

    // Subscriber for motor speed and corresponding speed as a percentage of
    // maximum throttle
    ros::Subscriber motor_sub;
    double throttle;

    // Vector indicating which axes of nu should be locked to their initial
    // values
    std::vector<bool> nu_lock;

    // Vehicle state vectors
    Vector6d nu;
    Vector6d eta;

    // Curvilinear position and origin point for cartesian position
    Vector3d p_b0;
    Vector3d p_b;

    // Publisher for state data
    ros::Publisher vehicle_state_pub;

private:

    //--------------------------------------------------------------------------
    // Name:        S
    // Description: Creates a 3x3 skew-symmetric matrix from a 3x1 vector
    // according to Equation 2.6 from Fossen.
    // Arguments:   - x: 3x1 vector to make skew-symmetric matrix
    // Returns :    Skew-symmetric matrix
    //--------------------------------------------------------------------------
    Matrix3d S(Vector3d x)
    {
        Matrix3d X;
        X <<   0, -x(2), x(1),
             x(2),   0, -x(0),
            -x(1), x(0),   0;
        return X;
    }

    //--------------------------------------------------------------------------
    // Name:        J_1
    // Description: Calculates the 3x3 linear velocity transform matrix defined
    //              by Equation 2.11 from Fossen.
    // Arguments:   - eta: Position and attitude vector (6x1).
    // Returns:     Linear velocity transform matrix (3x3).
    //--------------------------------------------------------------------------
    Matrix3d J_1(Vector6d eta)
    {

        double phi =   eta(3);
        double theta = eta(4);
        double psi =   eta(5);

        Matrix3d J1x;
        J1x << 1,     0,        0,
               0,  cos(phi), sin(phi),
               0, -sin(phi), cos(phi);

        Matrix3d J1y;
        J1y << cos(theta),  0, -sin(theta),
                   0,       1,     0,
               sin(theta),  0,  cos(theta);

        Matrix3d J1z;
        J1z <<  cos(psi),   sin(psi), 0,
               -sin(psi),   cos(psi), 0,
                   0,          0,     1;

        return (J1x*J1y*J1z).transpose();

    }

    //--------------------------------------------------------------------------
    // Name:        J_2
    // Description: Calculates the 3x3 angular velocity transform matrix defined
    //              by Equation 2.14 from Fossen.
    // Arguments:   - eta: Position and attitude vector (6x1).
    // Returns:     Angular velocity transform matrix (3x3).
    //--------------------------------------------------------------------------
    Matrix3d J_2(Vector6d eta)
    {
        double phi = eta(3);
        double theta = eta(4);
        Matrix3d mat;
        mat << 1.0 ,  sin(phi)*tan(theta),  cos(phi)*tan(theta),
               0.0 ,  cos(phi),            -sin(phi),
               0.0 ,  sin(phi)/cos(theta),  cos(phi)/cos(theta);
        return mat;
    }

    //--------------------------------------------------------------------------
    // Name:        J
    // Description: Calculates the 6x6 combined linear and angular velocity
    //              transform matrix defined by Equation 2.15 from Fossen.
    // Arguments:   - eta: Position and attitude vector (6x1).
    // Returns:     Linear and angular velocity transform matrix (6x6).
    //--------------------------------------------------------------------------
    Matrix6d J(Vector6d eta)
    {
        Matrix6d mat = Matrix6d::Zero();
        mat.topLeftCorner(3,3) = J_1(eta);
        mat.bottomRightCorner(3,3) = J_2(eta);
        return mat;
    }

    //--------------------------------------------------------------------------
    // Name:        fins_msg_callback
    // Description: Called when a message is received on the topic.
    // Arguments:   - message: message received on the topic
    //--------------------------------------------------------------------------
    void fins_msg_callback(const FinsMsg& message)
    {
        delta_e = message.port;
        delta_r = message.top;
    }

    //--------------------------------------------------------------------------
    // Name:        motor_msg_callback
    // Description: Called when a message is received on the topic.
    // Arguments:   - message: message received on the topic
    //--------------------------------------------------------------------------
    void motor_msg_callback(const std_msgs::Float64& message)
    {
        throttle = message.data;
    }

    //--------------------------------------------------------------------------
    // Name:        iterate_timer_callback
    // Description: Called when the iteration timer expires. Handles the
    //              iteration of the AUV state and publishes the results.
    // Arguments:   - event: ROS timer event structure
    //--------------------------------------------------------------------------
    void iterate_timer_callback(const ros::TimerEvent& event)
    {

        // Do not try to iterate dynamics if there is no dynamics server
        // connected
        if (!dynamics_client.exists())
            return;

        // ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
        // Construct the vehicle dynamics service request
        VehicleDynamicsSrv srv;

        srv.request.u = nu(0);
        srv.request.v = nu(1);
        srv.request.w = nu(2);
        srv.request.p = nu(3);
        srv.request.q = nu(4);
        srv.request.r = nu(5);

        srv.request.x =     eta(0);
        srv.request.y =     eta(1);
        srv.request.z =     eta(2);
        srv.request.phi =   eta(3);
        srv.request.theta = eta(4);
        srv.request.psi =   eta(5);

        srv.request.throttle = throttle;
        srv.request.delta_r =  delta_r;
        srv.request.delta_e =  delta_e;

        srv.request.dt = dt;

        // Call the dynamics service to get nu_dot
        dynamics_client.call(srv);

        /// Assemble the nu_dot response into a vector
        Vector6d nu_dot;
        nu_dot(0) = srv.response.u_dot;
        nu_dot(1) = srv.response.v_dot;
        nu_dot(2) = srv.response.w_dot;
        nu_dot(3) = srv.response.p_dot;
        nu_dot(4) = srv.response.q_dot;
        nu_dot(5) = srv.response.r_dot;

        // ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
        // Integrate nu_dot

        // Lock axes indicated in the config file parameter
        for (size_t i = 0; i < nu_lock.size(); i++)
            if (nu_lock.at(i))
                nu_dot(i) = 0.0;

        nu += nu_dot * dt;

        log_data("[nu] %f %f %f %f %f %f",
            nu(0), nu(1), nu(2), nu(3), nu(4), nu(5));

        log_data("[nu_dot] %f %f %f %f %f %f",
            nu_dot(0), nu_dot(1), nu_dot(2),
            nu_dot(3), nu_dot(4), nu_dot(5));

        // ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
        // Calculate and integrate eta_dot

        Vector6d eta_dot = J(eta)*nu;
        eta += eta_dot * dt;

        log_data("[eta] %f %f %f %f %f %f",
            eta(0), eta(1), eta(2), eta(3), eta(4), eta(5));

        log_data("[eta_dot] %f %f %f %f %f %f",
            eta_dot(0), eta_dot(1), eta_dot(2),
            eta_dot(3), eta_dot(4), eta_dot(5));

        // ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
        // Dynamics outputs are relative to an earth-fixed tangent frame
        // denoted by superscript l. Since the tangent frame frame is non-moving
        // and non-rotating relative to the ECEF frame, the velocity,
        // acceleration and angular rate relative to the tangent frame is the
        // same as the velocity, acceleration and angular rate relative to the
        // ECEF frame (Groves Section 2.5.4). In other words,
        //     v_lb_b = v_eb_b
        //     a_lb_b = a_eb_b
        //     w_lb_b = w_eb_b
        // However, the angular rate and resulting Euler angles will instead be
        // treated as if they were relative to the navigation frame that is
        // centered on the vehicle and aligned with the local north, east, and
        // down direction so that vehicle movement is mapped to the surface of
        // the earth instead of the tangent frame

        // ECEF-relative acceleration and velocity from dynamics
        Vector3d a_eb_b = nu_dot.head(3);
        Vector3d v_eb_b = nu.head(3);

        // Navigation-frame-relative angular rate, Euler angles, and rotation
        // matricies from dynamics
        Vector3d w_nb_b = nu.tail(3);
        Vector3d theta_n_b = eta.tail(3);
        Matrix3d C_b_n = J_1(eta);
        Matrix3d C_n_b = C_b_n.transpose();

        // Rate of change of velocity required for later calculation of f_ib_b.
        // Note the cross product that arises from the derivative of the
        // velocity
        Vector3d vdot_eb_b = a_eb_b + w_nb_b.cross(v_eb_b);

        // Since the navigation frame and the body frame share the same origin,
        // only a rotation is required to represent the body frame velocity and
        // rate of change of velocity in the navigation frame
        Vector3d v_eb_n = C_b_n*v_eb_b;
        Vector3d vdot_eb_n = C_b_n*vdot_eb_b;

        // Calculate earth rotation rate and transport rate (and their skew
        // symmetrix forms) required for calculation of w_ib_b and a_ib_b
        Vector3d w_ie_n = earth_rotation_rate(p_b);
        Vector3d w_en_n = transport_rate(p_b, v_eb_n);
        Matrix3d W_ie_n = avl::skew(w_ie_n);
        Matrix3d W_en_n = avl::skew(w_en_n);

        // Calculate acceleration due to gravity required for calculation
        // of f_ib_b
        Vector3d g_b_n = gravitational_acceleration(p_b);

        // Calculate the angular rate and acceleration relative to the inertial
        // frame, as measured by an IMU
        Vector3d w_ib_b = C_n_b*(w_ie_n + w_en_n) + w_nb_b;
        Vector3d f_ib_b = C_n_b*(vdot_eb_n - g_b_n + (W_en_n + 2*W_ie_n)*v_eb_n);

        // Calculate cartesian to curvilinear transformation matrix
        Matrix3d T_rn_p = cartesian_to_curvilinear(p_b);

        // Integrate the curvilinear position
        p_b += T_rn_p*v_eb_n*dt;

        // ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
        // Publish a vehicle state message

        VehicleStateMsg state_msg;
        state_msg.rpm =      srv.response.rpm;
        state_msg.roll =     theta_n_b(0);
        state_msg.pitch =    theta_n_b(1);
        state_msg.yaw =      theta_n_b(2);
        state_msg.v_eb_b_x = v_eb_b(0);
        state_msg.v_eb_b_y = v_eb_b(1);
        state_msg.v_eb_b_z = v_eb_b(2);
        state_msg.lat =      p_b(0);
        state_msg.lon =      p_b(1);
        state_msg.alt =      p_b(2);
        state_msg.lat0 =     p_b0(0);
        state_msg.lon0 =     p_b0(1);
        state_msg.alt0 =     p_b0(2);
        state_msg.r_b_n =    eta(0);
        state_msg.r_b_e =    eta(1);
        state_msg.r_b_d =    eta(2);
        state_msg.w_ib_b_x = w_ib_b(0);
        state_msg.w_ib_b_y = w_ib_b(1);
        state_msg.w_ib_b_z = w_ib_b(2);
        state_msg.f_ib_b_x = f_ib_b(0);
        state_msg.f_ib_b_y = f_ib_b(1);
        state_msg.f_ib_b_z = f_ib_b(2);
        vehicle_state_pub.publish(state_msg);

        // ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
        // Log data

        log_data("[p_b] %.9f %.9f %.2f %.9f %.9f %.2f",
            p_b0(0), p_b0(1), p_b0(2),
            p_b(0), p_b(1), p_b(2));

        log_data("[w_ib_b] %.6f %.6f %.6f",
            w_ib_b(0), w_ib_b(1), w_ib_b(2));

        log_data("[f_ib_b] %.6f %.6f %.6f",
            f_ib_b(0), f_ib_b(1), f_ib_b(2));

    }

    //--------------------------------------------------------------------------
    // Name:        init
    // Description: Initializes the node. Called when the node is started.
    //--------------------------------------------------------------------------
    void init()
    {

        // Add data log headers
        add_data_header("[nu] u v w p q r");
        add_data_header("[nu] m/s m/s m/s rad/s rad/s rad/s");
        add_data_header("[nu_dot] u\\_dot v\\_dot w\\_dot p\\_dot q\\_dot r\\_dot");
        add_data_header("[nu_dot] m/s^2 m/s^2 m/s^2 rad/s^2 rad/s^2 rad/s^2");
        add_data_header("[eta] x y z \\phi \\theta \\psi");
        add_data_header("[eta] m m m rad rad rad");
        add_data_header("[eta_dot] x\\_dot y\\_dot z\\_dot \\phi\\_dot \\theta\\_dot \\psi\\_dot");
        add_data_header("[eta_dot] m/s m/s m/s rad/s rad/s rad/s");
        add_data_header("[p_b] lat0 lon0 alt0 lat lon alt");
        add_data_header("[p_b] rad rad m rad rad m");
        add_data_header("[w_ib_b] \\omega_{ib,x}^b \\omega_{ib,y}^b \\omega_{ib,z}^b");
        add_data_header("[w_ib_b] rad/s rad/s rad/s");
        add_data_header("[f_ib_b] f_{ib,x}^b f_{ib,y}^b f_{ib,z}^b");
        add_data_header("[f_ib_b] m/s^2 m/s^2 m/s^2");

        // Get axis lock from config file
        nu_lock = get_param<std::vector<bool>>("~nu_lock");

        // Set up the vehicle dynamics service client
        dynamics_client =
            node_handle->serviceClient<VehicleDynamicsSrv>("sim/dynamics");

        // Set up the fin angle and motor speed subscribers
        fins_sub = node_handle->subscribe("device/fins", 100,
            &DynamicsManagerNode::fins_msg_callback, this);
        motor_sub = node_handle->subscribe("device/motor", 100,
            &DynamicsManagerNode::motor_msg_callback, this);

        // Set up the vehicle state publisher
        vehicle_state_pub =
            node_handle->advertise<VehicleStateMsg>("sim/state", 1);

        // Get the initial state from the config file
        auto initial_att = get_param<std::vector<double>>("~initial_att");
        auto initial_vel = get_param<std::vector<double>>("~initial_vel");
        auto initial_pos = get_param<std::vector<double>>("~initial_pos");

        // Set the initial states
        nu(0) = initial_vel.at(0);
        nu(1) = initial_vel.at(1);
        nu(2) = initial_vel.at(2);
        nu(3) = 0.0;
        nu(4) = 0.0;
        nu(5) = 0.0;

        eta(0) = 0.0;
        eta(1) = 0.0;
        eta(2) = 0.0;
        eta(3) = avl::deg_to_rad(initial_att.at(0));
        eta(4) = avl::deg_to_rad(initial_att.at(1));
        eta(5) = avl::deg_to_rad(initial_att.at(2));

        p_b0(0) = avl::deg_to_rad(initial_pos.at(0));
        p_b0(1) = avl::deg_to_rad(initial_pos.at(1));
        p_b0(2) = initial_pos.at(2);
        p_b = p_b0;

        // Set up the iteration timer. Creating the timer also starts it
        dt = 1.0/get_param<float>("~iteration_rate");
        iterate_timer = node_handle->createTimer(ros::Duration(dt),
            &DynamicsManagerNode::iterate_timer_callback, this);

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
    DynamicsManagerNode node(argc, argv);
    node.start();
    return 0;
}
