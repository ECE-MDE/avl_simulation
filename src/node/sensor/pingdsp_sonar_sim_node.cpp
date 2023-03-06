//==============================================================================
// Autonomous Vehicle Library
//
// Description: A node that listens for sonar data published by the AVL
//              Unreal Engine simulation and logs it.
//
// Servers:     None
//
// Clients:     None
//
// Publishers:  None
//
// Subscribers: /sim/sonar (std_msgs/Float32MultiArray)
//==============================================================================

// Node base class
#include <avl_core/node.h>

// ROS message includes
#include <std_msgs/Float32MultiArray.h>

//==============================================================================
//                              NODE DEFINITION
//==============================================================================

class PingdspSonarSimNode : public Node
{

public:

    //--------------------------------------------------------------------------
    // Name:        PingdspSonarSimNode constructor
    //--------------------------------------------------------------------------
    PingdspSonarSimNode(int argc, char **argv) : Node(argc, argv)
    {

    }

private:

    // Subscriber for simulated sonar data
    ros::Subscriber sonar_sub;

private:

    //--------------------------------------------------------------------------
    // Name:        sonar_msg_callback
    // Description: Called when a message is received on the topic.
    // Arguments:   - message: Message received on the topic.
    //--------------------------------------------------------------------------
    void sonar_msg_callback(const std_msgs::Float32MultiArray message)
    {

        // Since there are an unknown number of elements, loop throught all
        // elements and append them to a string stream with a space between them
        std::stringstream message_string_stream;
        for (const float& elem : message.data)
            message_string_stream << elem << " ";

        log_data("[sonar] %s", message_string_stream.str().c_str());

    }

    //--------------------------------------------------------------------------
    // Name:        init
    // Description: Initializes the node. Called when the node is started.
    //--------------------------------------------------------------------------
    void init()
    {

        // Add data log headers
        add_data_header("[sonar] x y z");
        add_data_header("[sonar] m m m");

        // Set up the AVL unreal sonar data subscriber
        sonar_sub = node_handle->subscribe("unreal/sonar", 100,
            &PingdspSonarSimNode::sonar_msg_callback, this);

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
    PingdspSonarSimNode node(argc, argv);
    node.start();
    return 0;
}
