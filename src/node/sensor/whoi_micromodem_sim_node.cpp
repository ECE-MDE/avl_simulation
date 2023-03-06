//==============================================================================
// Autonomous Vehicle Library
//
// Description: ROS node to control a WHOI micromodem. Handles the sending and
//              receiving of acoustic data to and from the modem. A modem can
//              take  the following actions:
//
// Servers:     device/whoi/downlink_tx (avl_msgs/ByteArraySrv)
//              device/whoi/ping_transponders (std_srvs/Trigger)
//
// Clients:     None
//
// Publishers:  device/whoi/status (avl_msgs/WhoiStatusMsg)
//              device/whoi/downlink_rx (avl_msgs/WhoiDataMsg)
//              device/whoi/transponders_rx (avl_msgs/WhoiTranspondersMsg)
//
// Subscribers: None
//==============================================================================


// Base node class
#include <avl_core/node.h>
#include <avl_comms/comms.h>

// AVL binary packet protocol
#include <avl_core/protocol/avl.h>
using namespace avl;

// Utility functions
#include <avl_core/util/string.h>
#include <avl_core/util/time.h>

// ROS messages
#include <avl_msgs/ByteArraySrv.h>
#include <std_srvs/Trigger.h>
#include <avl_msgs/WhoiStatusMsg.h>
#include <avl_msgs/WhoiDataMsg.h>
#include <avl_msgs/WhoiTranspondersMsg.h>
using namespace avl_msgs;
using namespace std_srvs;

// UDP socket class for sending packets
#include <avl_asio/udp_socket.h>

// WHOI Micromodem command protocol
#include <avl_devices/protocol/whoi_micromodem.h>

//==============================================================================
//                              NODE DEFINITION
//==============================================================================

class WhoiMicromodemSimNode : public Node
{

public:

    //--------------------------------------------------------------------------
    // Name:        WhoiMicromodemSimNode constructor
    //--------------------------------------------------------------------------
    WhoiMicromodemSimNode(int argc, char **argv) : Node(argc, argv)
    {

    }

private:

    // WHOI micromodem ID
    const int MODEM_ID = 0;

    // UDP socket used to send and receive mesasges and the multicast address
    // and port that they are passed through
    UdpSocket socket;
    std::string udp_address;
    int udp_tx_port;
    int udp_rx_port;

    // Service servers and publishers
    ros::ServiceServer downlink_tx_server;
    ros::ServiceServer ping_transponders_server;
    ros::Publisher status_pub;
    ros::Publisher downlink_rx_pub;
    ros::Publisher transponders_rx_pub;

    // Most recent time of arrival measurement
    double time_of_arrival;

    // Flag indicating that an acoustic transmission is in progress. The modem
    // cannot start a new transmission while already transmitting.
    bool transmit_in_progress = false;
    bool lbl_ping_in_progress = false;

    // Flag indicating whether the modem has synced its clock to the GPS time
    // and GPS PPS signal
    bool synced_to_gps = false;

    // Transmit buffer to hold bytes that will be sent to the modem when it
    // requests data to be transmitted
    std::vector<uint8_t> tx_buffer;

private:

    //--------------------------------------------------------------------------
    // Name:        acoustic_tx_srv_callback
    // Description: Service callback called when the micromodem downlink service
    //              is called to transmit data.
    // Arguments:   - request: Request message received from the client.
    //              - response: Response message to be returned to the client.
    // Returns:     True if service succeeded, false if it failed and was not
    //              responded to.
    //--------------------------------------------------------------------------
    bool downlink_tx_srv_callback(ByteArraySrv::Request& request,
                                  ByteArraySrv::Response& response)
    {

        // Cannot downlink while a transmit is already in progress
        if (transmit_in_progress || lbl_ping_in_progress)
        {
            response.success = false;
            response.message = "transmit already in progress";
            return true;
        }

        // Load the transmit buffer
        tx_buffer = request.data;

        // Initiate a modem transmit cycle. The modem will then send a request
        // for binary data to transmit, which will be handled in the serial
        // read callback
        transmit_in_progress = true;
        transmit_udp(tx_buffer);
        transmit_in_progress = false;

        // Return a success response
        response.success = true;
        response.message = "success";
        return true;

    }

    //--------------------------------------------------------------------------
    // Name:        ping_transponders_srv_callback
    // Description: Service callback called when the ping transponders service
    //              is called.
    // Arguments:   - request: Request message received from the client.
    //              - response: Response message to be returned to the client.
    // Returns:     True if service succeeded, false if it failed and was not
    //              responded to.
    //--------------------------------------------------------------------------
    bool ping_transponders_srv_callback(Trigger::Request& request,
                                        Trigger::Response& response)
    {

        // Cannot ping transponders while a transmit is already in progress
        if (transmit_in_progress || lbl_ping_in_progress)
        {
            response.success = false;
            response.message = "transmit already in progress";
            return true;
        }

        // Initiate the transponder ping
        lbl_ping_in_progress = true;

        //TODO: I'm not sure what to do with this in simulation
        lbl_ping_in_progress = false;

        // Return a success response
        response.success = true;
        response.message = "success";
        return true;

    }

    //--------------------------------------------------------------------------
    // Name:        packet_callback
    // Description: Called when the read match condition is met.
    // Arguments:   - data: data read by match condition
    //--------------------------------------------------------------------------
    void packet_callback(Packet packet)
    {

        switch (packet.get_descriptor())
        {

            case PASSTHROUGH_PACKET:
            {

                // Convert to a passthrough packet
                PassthroughPacket passthrough_packet(packet.get_bytes());
                PacketHeader header = packet.get_header();
                PassthroughMessage message = passthrough_packet.get_passthrough_message();

                // Ignore messages from self
                if(header.source_id == avl::get_vehicle_id())
                    return;

                // Publish a WHOI downlinnk RX message with the received data and
                // time of arrival
                WhoiDataMsg downlink_rx_msg;
                downlink_rx_msg.time_of_arrival = get_epoch_time();
                downlink_rx_msg.data = message.data;
                downlink_rx_pub.publish(downlink_rx_msg);
                break;

            } // case HEARTBEAT_PACKET

            default:
            {

            }

        } // switch

    }

    //--------------------------------------------------------------------------
    // Name:        transmit_udp
    // Description: Embeds data in a passthrough message and broadcasts over udp
    // Arguments:   - data: data to transmit
    //--------------------------------------------------------------------------
    void transmit_udp(std::vector<uint8_t> data)
    {
        // Header
        avl::PacketHeader header;
        header.timestamp = get_epoch_time_nanoseconds();
        header.timeout = 0;
        header.source_id = avl::get_vehicle_id();
        header.destination_id = 0;

        // Format the message
        //              0       uint8   current_waypoint
        //              1-end   uint8   visited_waypoints
        avl::PassthroughMessage message;
        message.target_id = 0x00;
        message.channel = CHANNEL_ACOUSTIC;
        message.interface = INTERFACE_FSD;
        message.data = data;

        // Packet
        avl::PassthroughPacket passthrough_packet(header, message);

        // Should add a delay here that scales with the packet size
        socket.send_to(udp_address, udp_tx_port, passthrough_packet.get_bytes());
    }

    //--------------------------------------------------------------------------
    // Name:        init
    // Description: Initializes the node. Called when the node is started.
    //--------------------------------------------------------------------------
    void init()
    {

        // Get the config file settings
        udp_address = avl::get_param<std::string>("~udp_address");
        udp_tx_port = avl::get_param<int>("~udp_tx_port");
        udp_rx_port = avl::get_param<int>("~udp_rx_port");

        // Open the UDP socket
        socket.set_read_timeout(0);
        socket.set_packet_callback(&WhoiMicromodemSimNode::packet_callback, this);
        socket.open(udp_rx_port, udp_address);


        // Reset transmit flags
        transmit_in_progress = false;
        lbl_ping_in_progress = false;

        // Set up the service servers and publishers
        downlink_tx_server = node_handle->advertiseService(
            "device/whoi/downlink_tx",
            &WhoiMicromodemSimNode::downlink_tx_srv_callback, this);
        ping_transponders_server = node_handle->advertiseService(
            "device/whoi/ping_transponders",
            &WhoiMicromodemSimNode::ping_transponders_srv_callback, this);

        status_pub = node_handle->advertise<WhoiStatusMsg>(
            "device/whoi/status", 1, true);
        downlink_rx_pub = node_handle->advertise<WhoiDataMsg>(
            "device/whoi/downlink_rx", 1);
        transponders_rx_pub = node_handle->advertise<WhoiTranspondersMsg>(
            "device/whoi/transponders_rx", 1);

        // Publish an initial status message for the status topic
        WhoiStatusMsg status_msg;
        status_msg.synced_to_gps = synced_to_gps;
        status_pub.publish(status_msg);

    }

    //--------------------------------------------------------------------------
    // Name:        run
    // Description: Main node loop. Called after the node is initialized.
    //--------------------------------------------------------------------------
    void run()
    {

        ros::Rate spin_rate(1000);
        while (ros::ok())
        {
            socket.spin_once();
            ros::spinOnce();
            spin_rate.sleep();
        }

    }

};

//==============================================================================
//                                  MAIN
//==============================================================================

int main(int argc, char **argv)
{
    WhoiMicromodemSimNode node(argc, argv);
    node.start();
    return 0;
}
