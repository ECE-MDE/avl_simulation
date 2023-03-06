//==============================================================================
// Autonomous Vehicle Library
//
// Description: Implements a vehicle dynamics server that calculates nu_dot
//              from an input nu, eta, and actuator states.
//
// Servers:     /sim/dynamics (avl_msgs/VehicleDynamicsSrv)
//
// Clients:     None
//
// Publishers:  None
//
// Subscribers: None
//
//==============================================================================

// Node base class
#include <avl_core/node.h>

// Util functions
#include <avl_core/util/math.h>
#include <avl_core/util/vector.h>
#include <avl_core/util/matrix.h>

// ROS messages
#include <avl_msgs/VehicleDynamicsSrv.h>
using namespace avl_msgs;

// Eigen library
#include <Eigen/Dense>
using namespace Eigen;

// Declare matrix/vector types
typedef Matrix<double, 6, 6> Matrix6d;
typedef Matrix<double, 6, 1> Vector6d;

//==============================================================================
//                              NODE DEFINITION
//==============================================================================

class AuvDynamicsNode : public Node
{

public:

    //--------------------------------------------------------------------------
    // Name:        AuvDynamicsNode constructor
    //--------------------------------------------------------------------------
    AuvDynamicsNode(int argc, char **argv) : Node(argc, argv)
    {

    }

private:

    // Server for responding to dynamics service requests
    ros::ServiceServer dynamics_server;

    // Acceleration due to gravity
    const double g = 9.8066;

    // Vehicle parameters from config file
    double m;
    double B_p;
    Matrix3d I_O;
    Vector3d r_G;
    Vector3d r_B;

    // Maximum RPM and RPM to velocity slope and intercept from config file
    double max_rpm;
    double slope;
    double inter;

    // Water current velocity from config file
    Vector3d v_curr;

    // Drag terms from config file
    double X_uu;
    double Y_vv, Y_rr, Y_uv, Y_ur;
    double Z_ww, Z_qq, Z_uw, Z_uq;
    double K_vv, K_pp, K_uv, K_up;
    double M_ww, M_qq, M_uw, M_uq;
    double N_vv, N_rr, N_uv, N_ur;

    // Added mass Coriolis terms from config file
    double X_udot, Y_udot, Z_udot, K_udot, M_udot, N_udot;
    double X_vdot, Y_vdot, Z_vdot, K_vdot, M_vdot, N_vdot;
    double X_wdot, Y_wdot, Z_wdot, K_wdot, M_wdot, N_wdot;
    double Y_pdot, X_pdot, Z_pdot, K_pdot, M_pdot, N_pdot;
    double X_qdot, Y_qdot, Z_qdot, K_qdot, M_qdot, N_qdot;
    double X_rdot, Y_rdot, Z_rdot, K_rdot, M_rdot, N_rdot;

    // Control terms from config file
    double Y_uudr;
    double Z_uude;
    double N_uudr;
    double M_uude;

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
    // Name:        D
    // Description: Calculates the 6x6 Drag matrix as a function of the linear
    //              and angular rates nu.
    // Arguments:   - nu: Linear and angular rate vector (6x1).
    // Returns:    Drag matrix (6x6).
    //--------------------------------------------------------------------------
    Matrix6d D(Vector6d nu)
    {

        // Components of nu
        double u = nu(0);
        double v = nu(1);
        double w = nu(2);
        double p = nu(3);
        double q = nu(4);
        double r = nu(5);

        // Construct the matrix
        Matrix6d mat = Matrix6d::Zero();
    	mat(0, 0) = X_uu * abs(u);
    	mat(1, 1) = Y_vv * abs(v) + Y_uv * u;
    	mat(2, 2) = Z_ww * abs(w) + Z_uw * u;
    	mat(3, 3) = K_pp * abs(p) + K_up * u;
    	mat(4, 4) = M_qq * abs(q) + M_uq * u;
    	mat(5, 5) = N_rr * abs(r) + N_ur * u;
    	mat(1, 5) = Y_rr * abs(r) + Y_ur * u;
    	mat(2, 4) = Z_qq * abs(q) + Z_uq * u;
        mat(3, 1) = K_vv * abs(v) + K_uv * u;
    	mat(4, 2) = M_ww * abs(w) + M_uw * u;
    	mat(5, 1) = N_vv * abs(v) + N_uv * u;

        return mat;

    }

    //--------------------------------------------------------------------------
    // Name:        M_RB
    // Description: Calculates the 6x6 rigid body matrix using Equation 2.91
    //              from Fossen.
    // Returns:     Rigid body matrix (6x6).
    //--------------------------------------------------------------------------
    Matrix6d M_RB()
    {
        Matrix6d mat = Matrix6d::Zero();
        mat.topLeftCorner(3,3) =      m*Matrix3d::Identity();
        mat.topRightCorner(3,3) =    -m*S(r_G);
        mat.bottomLeftCorner(3,3) =   m*S(r_G);
        mat.bottomRightCorner(3,3) =  I_O;
        return mat;
    }

    //--------------------------------------------------------------------------
    // Name:        C_RB
    // Description: Calculates the 6x6 rigid body Coriolis matrix as a function
    //              of the linear and angular rates nu using Equation 2.99 from
    //              Fossen.
    // Arguments:   - nu: Linear and angular rate vector (6x1).
    // Returns:     Rigid body Coriolis matrix (6x6).
    //--------------------------------------------------------------------------
    Matrix6d C_RB(Vector6d nu)
    {

        // Components of nu
        double u = nu(0);
        double v = nu(1);
        double w = nu(2);
        double p = nu(3);
        double q = nu(4);
        double r = nu(5);

        // Components of r_G
        double x_G = r_G(0);
        double y_G = r_G(1);
        double z_G = r_G(2);

        // Components of I_O
        double I_xx = I_O(0,0);
        double I_yy = I_O(1,1);
        double I_zz = I_O(2,2);
        double I_xy = I_O(0,1);
        double I_xz = I_O(0,2);
        double I_yz = I_O(1,2);

        // Construct the matrix
        Matrix6d mat;
        mat  <<   0,                  0,                   0,                    m*(y_G*q + z_G*r),      -m*(x_G*q - w),          -m*(x_G*r + v),
                  0,                  0,                   0,                   -m*(y_G*p + w),           m*(z_G*r + x_G*p),      -m*(y_G*r - u),
                  0,                  0,                   0,                   -m*(z_G*p - v),          -m*(z_G*q + u),           m*(x_G*p + y_G*q),
                 -m*(y_G*q + z_G*r),  m*(y_G*p + 0),       m*(z_G*p - 0),        0,                      (I_zz*r-I_xz*p-I_yz*q),  (I_yz*r+I_xy*p-I_yy*q),
                  m*(x_G*q - 0),     -m*(z_G*r + x_G*p),   m*(z_G*q + 0),       (I_yz*q+I_xz*p-I_zz*r),   0,                      (I_xx*p-I_xz*r-I_xy*q),
                  m*(x_G*r + 0),      m*(y_G*r - 0),      -m*(x_G*p + y_G*q),   (I_yy*q-I_yz*r-I_xy*p),  (I_xz*r+I_xy*q-I_xx*p),   0;

        return mat;

    }

    //--------------------------------------------------------------------------
    // Name:        M_A
    // Description: Calculates the 6x6 added mass matrix.
    // Returns:     Added mass matrix (6x6).
    //--------------------------------------------------------------------------
    Matrix6d M_A()
    {
        Matrix6d mat;
        mat << X_udot, X_vdot, X_wdot, X_pdot, X_qdot, X_rdot,
               Y_udot, Y_vdot, Y_wdot, Y_pdot, Y_qdot, Y_rdot,
               Z_udot, Z_vdot, Z_wdot, Z_pdot, Z_qdot, Z_rdot,
               K_udot, K_vdot, K_wdot, K_pdot, K_qdot, K_rdot,
               M_udot, M_vdot, M_wdot, M_pdot, M_qdot, M_rdot,
               N_udot, N_vdot, N_wdot, N_pdot, N_qdot, N_rdot;
        return mat;
    }

    //--------------------------------------------------------------------------
    // Name:        C_A
    // Description: Calculates the 6x6 added mass Coriolis matrix as a function
    //              of the linear and angular rates nu using Equation 2.125 from
    //              Fossen.
    // Arguments:   - nu: Linear and angular rate vector (6x1).
    // Returns:     Added mass Coriolis matrix (6x6).
    //--------------------------------------------------------------------------
    Matrix6d C_A(Vector6d nu)
    {

        // Get components of nu for ease of notation
        double u = nu(0);
        double v = nu(1);
        double w = nu(2);
        double p = nu(3);
        double q = nu(4);
        double r = nu(5);

        // Construct the matrix
        Matrix6d mat;
        mat   <<   0,                               0,                             0,                             0,                           Z_udot*u+0*Z_wdot*w+0*Z_qdot*q, -0*Y_vdot*v-0*Y_rdot*r-Y_pdot*p,
                   0,                               0,                             0,                            -Z_wdot*w-Z_qdot*q-Z_udot*u,  0,                               0*X_udot*u+X_wdot*w+X_qdot*q,
                   0,                               0,                             0,                             Y_vdot*v+Y_rdot*r+Y_pdot*p, -0*X_udot*u-X_wdot*w-X_qdot*q,    0,
                   0,                               Z_udot*u+Z_wdot*w+Z_qdot*q,   -Y_vdot*v-Y_rdot*r-Y_pdot*p,    0,                           N_vdot*v+N_pdot*p+N_rdot*r,     -M_qdot*q-M_wdot*w-M_udot*u,
                  -Z_udot*u-0*Z_wdot*w-0*Z_qdot*q,  0,                             0*X_udot*u+X_wdot*w+X_qdot*q, -N_rdot*r-N_vdot*v-N_pdot*p,  0,                               K_vdot*v+K_pdot*p+K_rdot*r,
                   0*Y_vdot*v+Y_pdot*p+0*Y_rdot*r, -0*X_udot*u-X_wdot*w-X_qdot*q,  0,                             M_udot*u+M_wdot*w+M_qdot*q, -K_vdot*v-K_pdot*p-K_rdot*r,      0;
        return -mat;

    }

    //--------------------------------------------------------------------------
    // Name:        G
    // Description: Calculates the 6x1 restoring forces vector defined by
    //              Equation 2.167 from Fossen.
    // Arguments:   - eta: Position and attitude vector (6x1).
    // Returns:     Restoring forces vector (6x1).
    //--------------------------------------------------------------------------
    Vector6d G(Vector6d eta)
    {

        // Calculate vehicle weight and buoyancy force using the equations in
        // the paragraph above Equation 2.166 in Fossen except we will use a
        // buoyancy percentage instead of volume. Buoyancy only applies when
        // below the surface.
        double W = m*g;
        double B;
        if (eta(2) > 0.0)
            B = (1 + B_p/100.0) * W;
        else
            B = W;

        // Calculate the body-fixed weight and buoyancy force vectors using
        // Equation 2.166 in Fossen
        Vector3d f_G = J_1(eta).transpose() * Vector3d({0.0, 0.0, W});
        Vector3d f_B = -J_1(eta).transpose() * Vector3d({0.0, 0.0, B});

        // Calculate the restoring forces vector using Equation 2.167 in Fossen
        Vector6d vec = Vector6d::Zero();
        vec.head(3) = f_G + f_B;
        vec.tail(3) = r_G.cross(f_G) + r_B.cross(f_B);

        return vec;

    }

    //--------------------------------------------------------------------------
    // Name:        nu_dot
    // Description: Calculates the 6x1 vector containing the linear and angular
    //              velocity rates of change solved from Equation 2.172 in
    //              Fossen.
    // Arguments:   - nu: Linear and angular rate vector (6x1).
    //              - eta: Position and attitude vector (6x1).
    //              - tau: Control input vector (6x1).
    // Returns:     Linear and angular rates of change vector (6x1).
    //--------------------------------------------------------------------------
    Vector6d nu_dot(Vector6d nu, Vector6d eta, Vector6d tau)
    {

        // Adjust nu to include water current velocity as per Equations 3.106
        // through 3.111 in Fossen
        Vector6d nu_c = Vector6d::Zero();
        nu_c.head(3) = J_1(eta).transpose() * v_curr;
        nu = nu - nu_c;

        return (M_RB() - M_A()).inverse() *
            (-C_RB(nu)*nu + C_A(nu)*nu + D(nu)*nu + G(eta) + tau);

    }

    //--------------------------------------------------------------------------
    // Name:        dynamics_srv_callback
    // Description: Called when the dynamics service is called.
    // Arguments:   - req: request received on the service
    //              - res: response to the service request
    // Returns:     True if service succeeded, false if it failed and was not
    //              responded to.
    //--------------------------------------------------------------------------
    bool dynamics_srv_callback(VehicleDynamicsSrv::Request& req,
                               VehicleDynamicsSrv::Response& res)
    {

        // ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
        // Construct nu and eta from the request data

        Vector6d nu;
        nu << req.u, req.v, req.w, req.p, req.q, req.r;

        Vector6d eta;
        eta << req.x, req.y, req.z, req.phi, req.theta, req.psi;

        log_data("[nu] %f %f %f %f %f %f",
            nu(0), nu(1), nu(2), nu(3), nu(4), nu(5));

        log_data("[eta] %f %f %f %f %f %f",
            eta(0), eta(1), eta(2), eta(3), eta(4), eta(5));

        // ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
        // Construct the control input vector tau

        // Calculate RPM from throttle
        double rpm = avl::linear_scale(req.throttle,
            0.0, 100.0, 0.0, max_rpm);

        // Calculate velocity and thrust from RPM to velocity equation and
        // hydrodynamic cpefficients
        double vx = slope*rpm + inter;
        double thrust = -1.0*X_uu*vx*vx;

        // Construct the input vector tau. Rudder angle must be negated due to
        // Dr. Stilwell's inconsistent rudder angle convention.

        // Calculate water-relative nu per Equations 3.106 through 3.111 in
        // Fossen
        Vector6d nu_c = Vector6d::Zero();
        nu_c.head(3) = J_1(eta).transpose() * v_curr;
        Vector6d nu_wat = nu - nu_c;

        double u = nu_wat(0);
        Vector6d tau;
        tau << thrust,
               Y_uudr*u*abs(u)*-req.delta_r,
               Z_uude*u*abs(u)*req.delta_e,
               0.0,
               M_uude*u*abs(u)*req.delta_e,
               N_uudr*u*abs(u)*-req.delta_r;

       log_data("[tau] %f %f %f %f %f %f",
           tau(0), tau(1), tau(2), tau(3), tau(4), tau(5));

        // ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
        // Calculate nu_dot

        res.rpm = rpm;

        Vector6d nudot = nu_dot(nu, eta, tau);
        res.u_dot = nudot(0);
        res.v_dot = nudot(1);
        res.w_dot = nudot(2);
        res.p_dot = nudot(3);
        res.q_dot = nudot(4);
        res.r_dot = nudot(5);

        log_data("[nu_dot] %f %f %f %f %f %f",
            nudot(0), nudot(1), nudot(2), nudot(3), nudot(4), nudot(5));

        return true;

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
        add_data_header("[eta] x y z \\phi \\theta \\psi");
        add_data_header("[eta] m m m rad rad rad");
        add_data_header("[tau] thrust \\tau_1 \\tau_2 \\tau_3 \\tau_4 \\tau_5");
        add_data_header("[tau] N ? ? ? ? ?");
        add_data_header("[nu_dot] u\\_dot v\\_dot w\\_dot p\\_dot q\\_dot r\\_dot");
        add_data_header("[nu_dot] m/s^2 m/s^2 m/s^2 rad/s^2 rad/s^2 rad/s^2");

        // Get the dynamic model parameters from the config file
        auto I_O_vect = get_param<std::vector<double>>("~I_O");
        auto r_G_vect = get_param<std::vector<double>>("~r_G");
        auto r_B_vect = get_param<std::vector<double>>("~r_B");

        I_O = Map<Matrix3d>(I_O_vect.data()).transpose();
        r_G = Map<Vector3d>(r_G_vect.data());
        r_B = Map<Vector3d>(r_B_vect.data());
        m = get_param<double>("~m");
        B_p = get_param<double>("~B_p");

        max_rpm = get_param<double>("~max_rpm");
        slope = get_param<double>("~slope");
        inter = get_param<double>("~inter");

        auto current_vel_vect = get_param<std::vector<double>>("~current_vel");
        v_curr = Map<Vector3d>(current_vel_vect.data());

        X_uu = get_param<double>("~X_uu");
        Y_vv = get_param<double>("~Y_vv");
        Y_rr = get_param<double>("~Y_rr");
        Y_uv = get_param<double>("~Y_uv");
        Y_ur = get_param<double>("~Y_ur");
        Z_ww = get_param<double>("~Z_ww");
        Z_qq = get_param<double>("~Z_qq");
        Z_uw = get_param<double>("~Z_uw");
        Z_uq = get_param<double>("~Z_uq");
        K_vv = get_param<double>("~K_vv");
        K_pp = get_param<double>("~K_pp");
        K_uv = get_param<double>("~K_uv");
        K_up = get_param<double>("~K_up");
        M_ww = get_param<double>("~M_ww");
        M_qq = get_param<double>("~M_qq");
        M_uw = get_param<double>("~M_uw");
        M_uq = get_param<double>("~M_uq");
        N_vv = get_param<double>("~N_vv");
        N_rr = get_param<double>("~N_rr");
        N_uv = get_param<double>("~N_uv");
        N_ur = get_param<double>("~N_ur");

        X_udot = get_param<double>("~X_udot");
        Y_udot = get_param<double>("~Y_udot");
        Z_udot = get_param<double>("~Z_udot");
        K_udot = get_param<double>("~K_udot");
        M_udot = get_param<double>("~M_udot");
        N_udot = get_param<double>("~N_udot");
        X_vdot = get_param<double>("~X_vdot");
        Y_vdot = get_param<double>("~Y_vdot");
        Z_vdot = get_param<double>("~Z_vdot");
        K_vdot = get_param<double>("~K_vdot");
        M_vdot = get_param<double>("~M_vdot");
        N_vdot = get_param<double>("~N_vdot");
        X_wdot = get_param<double>("~X_wdot");
        Y_wdot = get_param<double>("~Y_wdot");
        Z_wdot = get_param<double>("~Z_wdot");
        K_wdot = get_param<double>("~K_wdot");
        M_wdot = get_param<double>("~M_wdot");
        N_wdot = get_param<double>("~N_wdot");
        Y_pdot = get_param<double>("~Y_pdot");
        X_pdot = get_param<double>("~X_pdot");
        Z_pdot = get_param<double>("~Z_pdot");
        K_pdot = get_param<double>("~K_pdot");
        M_pdot = get_param<double>("~M_pdot");
        N_pdot = get_param<double>("~N_pdot");
        X_qdot = get_param<double>("~X_qdot");
        Y_qdot = get_param<double>("~Y_qdot");
        Z_qdot = get_param<double>("~Z_qdot");
        K_qdot = get_param<double>("~K_qdot");
        M_qdot = get_param<double>("~M_qdot");
        N_qdot = get_param<double>("~N_qdot");
        X_rdot = get_param<double>("~X_rdot");
        Y_rdot = get_param<double>("~Y_rdot");
        Z_rdot = get_param<double>("~Z_rdot");
        K_rdot = get_param<double>("~K_rdot");
        M_rdot = get_param<double>("~M_rdot");
        N_rdot = get_param<double>("~N_rdot");

        Y_uudr = get_param<double>("~Y_uudr");
        Z_uude = get_param<double>("~Z_uude");
        N_uudr = get_param<double>("~N_uudr");
        M_uude = get_param<double>("~M_uude");

        // Set up the dynamics service server
        dynamics_server = node_handle->advertiseService("sim/dynamics",
            &AuvDynamicsNode::dynamics_srv_callback, this);

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
    AuvDynamicsNode node(argc, argv);
    node.start();
    return 0;
}
