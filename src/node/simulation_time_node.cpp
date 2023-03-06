//==============================================================================
// Autonomous Vehicle Library
//
// Description: Manages simulation time. Responds to commands to set simulation
//              time multiplier.
//
// Servers:     None
//
// Clients:     None
//
// Publishers:  /clock (rosgraph_msgs/Clock)
//
// Subscribers: None
//==============================================================================

// Node base class
#include <avl_core/node.h>

// Command handler class
#include <avl_comms/command_handler.h>
using namespace avl;

// ROS message includes
#include <rosgraph_msgs/Clock.h>

//==============================================================================
//                             SIMULATION COMMANDS
//==============================================================================

// Enum listing simulation speed multipliers
enum SimCommand
{
    SIM_1X,
    SIM_2X,
    SIM_5X,
    SIM_10X
};

//==============================================================================
//                              NODE DEFINITION
//==============================================================================

class SimulationTimeNode : public Node
{

public:

    //--------------------------------------------------------------------------
    // Name:        SimulationTimeNode constructor
    //--------------------------------------------------------------------------
    SimulationTimeNode(int argc, char **argv) : Node(argc, argv) { }

private:

    // Publisher for ROS simulation time
    ros::Publisher clock_pub;

    // Wall timer for iterating clock ticks
    ros::WallTimer clock_timer;

    // Clock tick rate in Hz from config file
    double clock_rate;

    // Clock time in seconds
    double clock_time = 0.0;

    // Time multiplier. 1.0 is real time, 2.0 is double real time, etc.
    volatile double time_multiplier;

    // Command handler for handling incoming COMMAND packets
    CommandHandler command_handler;

private:

    //--------------------------------------------------------------------------
    // Name:        command_callback
    // Description: Called when a COMMAND packet is received by the comms
    //              architecture.
    // Arguments:   - channel: Channel that the packet was received through.
    //              - interface: Interface that the packet was received from.
    //              - command_name: Command name of the received command.
    //              - params: Command parameter list.
    //              - result: Should be set to indicate if the response to the
    //                packet is a success or failure.
    //              - data: Should be set to contain data as a response to the
    //                packet or indicate the reason for a failure.
    // Returns:     True if the packet was responded to, false if it was not.
    //--------------------------------------------------------------------------
    bool command_callback(CommsChannel channel, CommsInterface interface,
        std::string command_name, ParameterList params,
        bool& result, std::vector<uint8_t>& data)
    {

        // Handle SIMULATION commands
        if (command_name == "SIMULATION")
        {

            switch (params.get("COMMAND").to_enum<SimCommand>())
            {
                case SIM_1X:  time_multiplier = 1.0; break;
                case SIM_2X:  time_multiplier = 2.0; break;
                case SIM_5X:  time_multiplier = 5.0; break;
                case SIM_10X: time_multiplier = 10.0; break;
            }

            result = true;
            return true;

        }

        return false;

    }

    //--------------------------------------------------------------------------
    // Name:        init
    // Description: Initializes the node. Called when the node is started.
    //--------------------------------------------------------------------------
    void init()
    {

        // Get the default time multiplier and clock rate from the config file
        time_multiplier = get_param<double>("~default_sim_time_multiplier");
        clock_rate = get_param<double>("~clock_rate");

        // Configure the clock publisher
        clock_pub = node_handle->advertise<rosgraph_msgs::Clock>("/clock", 1);

        // Set the command handler's callback
        command_handler.set_callback(
            &SimulationTimeNode::command_callback, this);

    }

    //--------------------------------------------------------------------------
    // Name:        run
    // Description: Main node loop. Called after the node is initialized.
    //--------------------------------------------------------------------------
    void run()
    {

        // Async spinner for handling command callback
        ros::AsyncSpinner spinner(1);
        spinner.start();

        // Main loop for publishing clock messages
        ros::WallRate rate(clock_rate);
        while (ros::ok())
        {
            clock_time += 1.0 / clock_rate * time_multiplier;
            rosgraph_msgs::Clock clock_msg;
            ros::Time time(clock_time);
            clock_msg.clock = time;
            clock_pub.publish(clock_msg);
            rate.sleep();
        }

    }

};

//==============================================================================
//                                  MAIN
//==============================================================================
int main(int argc, char **argv)
{
    SimulationTimeNode node(argc, argv);
    node.start();
    return 0;
}
