//==============================================================================
// Autonomous Vehicle Library
//
// Description: Implements a basic sensor model with sensor errors for a sensor
//              with an arbitrary number of outputs. Sensor model includes
//              Gaussian sensor noise, axis misalignment, and bias errors.
//==============================================================================

#ifndef SENSOR_MODEL_H
#define SENSOR_MODEL_H

// Random number generation
#include <random>
#include <chrono>
using namespace std::chrono;

// Eigen matrices and vectors
#include <Eigen/Core>
using Eigen::VectorXd;
using Eigen::MatrixXd;

// Alias for double vector
typedef std::vector<double> doubles_t;

//==============================================================================
//                              STRUCT DEFINITION
//==============================================================================

// Struct containing info about the sensor error
typedef struct SensorConfig
{

    // Number of sensor axes
    int N = 1;

    // Scale factor and misalignment matrix
    MatrixXd M = MatrixXd::Identity(1,1);

    // Bias vector
    VectorXd b = VectorXd::Zero(1);

    // Noise covariance vector
    VectorXd cov = VectorXd::Zero(1);

} SensorConfig;

//==============================================================================
//                              CLASS DECLARATION
//==============================================================================

class SensorModel
{

public:

    //--------------------------------------------------------------------------
    // Name:        from_config_file
    // Description: Constructs a sensor model from config file parameters.
    // Arguments:   - name: Name of parameter containing sensor error settings.
    // Returns:     Sensor model configured from the config file settings.
    //--------------------------------------------------------------------------
    static SensorModel from_config_file(std::string name);

public:


    //--------------------------------------------------------------------------
    // Name:        SensorModel constructor
    // Description: SensorModel constructor.
    //--------------------------------------------------------------------------
    SensorModel();

    //--------------------------------------------------------------------------
    // Name:        SensorModel constructor
    // Description: SensorModel constructor. Sets the number of sensor axes.
    // Arguments:   - config: Sensor error model configuration struct.
    //--------------------------------------------------------------------------
    SensorModel(SensorConfig config);

    //--------------------------------------------------------------------------
    // Name:        SensorModel destructor
    // Description: Default virtual destructor.
    //--------------------------------------------------------------------------
    virtual ~SensorModel();

    //--------------------------------------------------------------------------
    // Name:        add_error
    // Description: Adds the sensor error model terms to the true sensor
    //              measurement value, inclusing noise.
    // Arguments:   - x: Vector of true sensor values to add error to.
    // Returns:     Vector with sensor errors added.
    //--------------------------------------------------------------------------
    VectorXd add_error(const VectorXd& x);

private:

    // Sensor error configuration
    SensorConfig config;

    // Random number generator
    // see http://www.cplusplus.com/reference/random/
    std::default_random_engine generator;

    // Vector containing the noise distribution for each axis
    std::vector<std::normal_distribution<double>> noise_dists;

};

#endif // SENSOR_MODEL_H
