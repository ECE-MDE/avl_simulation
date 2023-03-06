//==============================================================================
// Autonomous Vehicle Library
//
// Description: Implements a basic sensor model with sensor errors for a sensor
//              with an arbitrary number of outputs. Sensor model includes
//              Gaussian sensor noise, axis misalignment, and bias errors.
//==============================================================================

#include <avl_simulation/sensor_model.h>

// Exceptions
#include <stdexcept>

// Util functions
#include <avl_core/util/ros.h>

//==============================================================================
//                              CLASS DEFINITION
//==============================================================================

//------------------------------------------------------------------------------
// Name:        from_config_file
// Description: Constructs a sensor model from config file parameters.
// Arguments:   - name: Name of parameter containing sensor error settings.
// Returns:     Sensor model configured from the config file settings.
//------------------------------------------------------------------------------
SensorModel SensorModel::from_config_file(std::string name)
{

    // Get the config file parameters
    int N = avl::get_param<int>("~" + name + "/N");
    doubles_t M_vect =   avl::get_param<doubles_t>("~" + name + "/M");
    doubles_t b_vect =   avl::get_param<doubles_t>("~" + name + "/b");
    doubles_t cov_vect = avl::get_param<doubles_t>("~" + name + "/cov");

    // Construct the config struct
    SensorConfig config;
    config.N = N;
    config.M =   Eigen::Map<MatrixXd>(M_vect.data(), N, N).transpose();
    config.b =   Eigen::Map<VectorXd>(b_vect.data(), N);
    config.cov = Eigen::Map<VectorXd>(cov_vect.data(), N);

    return SensorModel(config);

}

//--------------------------------------------------------------------------
// Name:        SensorModel constructor
// Description: SensorModel constructor.
//--------------------------------------------------------------------------
SensorModel::SensorModel()
{
    SensorModel(SensorConfig());
}

//------------------------------------------------------------------------------
// Name:        SensorModel constructor
// Description: SensorModel constructor. Sets the number of sensor axes.
// Arguments:   - config: Sensor error model configuration struct.
//------------------------------------------------------------------------------
SensorModel::SensorModel(SensorConfig config)
{

    this->config = config;

    // Verify misalignment/scale factor matrix size
    if (config.M.rows() != config.N || config.M.cols() != config.N)
        throw std::runtime_error("misalignment and scale factor matrix must "
            "be size (N x N)");

    // Verify bias vector size
    if (config.b.size() != config.N)
        throw std::runtime_error("bias vector must be size (N x 1)");

    // Verify covariance vector size
    if (config.cov.size() != config.N)
        throw std::runtime_error("noise cov vector must be size (N x 1)");

    // Create a normal distribution for each sensor axis with zero mean and
    // stddev from the config covariance
    noise_dists.clear();
    for (int i = 0; i < config.N; i++)
    {
        auto dist = std::normal_distribution<double>(0.0, sqrt(config.cov(i)));
        noise_dists.push_back(dist);
    }

    // Generate a seed for the random number generator from the current time
    generator.seed(system_clock::now().time_since_epoch().count());

}

//------------------------------------------------------------------------------
// Name:        SensorModel destructor
// Description: Default virtual destructor.
//------------------------------------------------------------------------------
SensorModel::~SensorModel()
{

}

//------------------------------------------------------------------------------
// Name:        add_error
// Description: Adds the sensor error model terms to the true sensor
//              measurement value, inclusing noise.
// Arguments:   - x: Vector of true sensor values to add error to.
// Returns:     Vector with sensor errors added.
//------------------------------------------------------------------------------
VectorXd SensorModel::add_error(const VectorXd& x)
{

    // Verify vector size
    if (x.size() != config.N)
        throw std::runtime_error("add_error: vector must be size (N x 1)");

    // Generate the vector of noise samples
    doubles_t noise_vect;
    for (int i = 0; i < config.N; i++)
        noise_vect.push_back(noise_dists.at(i)(generator));
    Eigen::Map<VectorXd> noise(noise_vect.data(), config.N);

    // Add the various error sources to the true value
    return config.M * (x + config.b) + noise;

}
