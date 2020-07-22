// monitor for handling raw tcp packets, especially an abort message
// created by matt waltz
// email matthewwaltzis@gmail.com with any questions

#include "MOOS/libMOOS/App/MOOSApp.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <fcntl.h>
#include <sys/ioctl.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>

#define PABORT_VERSION 1.0 // version information
#define PACKET_SIZE 128 // receiving packet size

class pAbortModule : public CMOOSApp {

    // ---------------------------------------------------------------------------------
    // runs on start of program
    // prints info and gets data from the mission file
    // ---------------------------------------------------------------------------------
    bool OnStartUp(void) {
        MOOSTrace(MOOSFormat("pAbort v%3.2f\n", PABORT_VERSION) + "Written by Matt Waltz\n\n");

        // grab the tcp port number from the mission file
        if (!m_MissionReader.GetConfigurationParam("TCP_PORT", port)) {
            return MOOSFail("Failed to read required mission file parameters\n", "");
        }

        // set the application status connection
        ReadyTCPConnection();
        return true;
    }

    // ---------------------------------------------------------------------------------
    // runs on connection to the moosdb server
    // set the variables controlling the return statuses
    // ---------------------------------------------------------------------------------
    bool OnConnectToServer() {
        return Register("RETURN", 0.0) | Register("DEPLOY", 0.0);
    }

    // ---------------------------------------------------------------------------------
    // registers the main abort variables and opens/configures the comm socket
    // ---------------------------------------------------------------------------------
    bool ReadyTCPConnection() {
        int on = 1;

        // we are now connected to the moosdb
        // let's check for an acceptable tcp connection

        // initialize the socket
        if ((listen_sd = socket(AF_INET, SOCK_STREAM, 0)) < 0) {
            return MOOSFail("Failed to create socket.\n", "");
        }

        // allow reuse of the socket address
        if (setsockopt(listen_sd, SOL_SOCKET, SO_REUSEADDR, (char*)&on, sizeof(on)) < 0) {
            return MOOSFail("Failed to option socket.\n", "");
        }

        // don't allow the socket to block
        if (ioctl(listen_sd, FIONBIO, (char *)&on) < 0) {
            return MOOSFail("Failed to non-block socket.\n", "");
        }

        // set up the address inforamtion
        memset(&addr, 0, sizeof(addr));
        addr.sin_family = AF_INET;
        addr.sin_addr.s_addr = INADDR_ANY;
        addr.sin_port = htons(port);

        MOOSTrace("Sever port: %d\n\n", port);

        // bind the requested port to the ip socket
        if (bind(listen_sd, (struct sockaddr*)&addr, sizeof(addr)) < 0) {
            return MOOSFail("Failed to bind socket.\n", "");
        }

        // listen for up to 32 connection pendings
        if (listen(listen_sd, 32) < 0) {
            return MOOSFail("Failed to listen on socket.\n", "");
        }

        // zero the master file descriptor
        FD_ZERO(&master_set);
        max_sd = listen_sd;
        FD_SET(listen_sd, &master_set);

        return true;
    }

    // ---------------------------------------------------------------------------------
    // called every now and then
    // checks if a tcp packet is received over ethernet
    // ---------------------------------------------------------------------------------
    bool Iterate() {
        fd_set working_set;
        struct timeval timeout;
        int rc;

        // set the timeout to 5000 microseconds for polling a connection
        memcpy(&working_set, &master_set, sizeof(master_set));
        timeout.tv_sec  = 0;
        timeout.tv_usec = 5000;

        // check if we have anything available
        rc = select(max_sd + 1, &working_set, NULL, NULL, &timeout);
        if (rc < 0) {
            return MOOSFail("Failed to select().\n", "");
        }

        // iterate over available descriptors
        int desc_ready = rc;
        for (int fd = 0; fd <= max_sd && desc_ready > 0; fd++) {
            if (FD_ISSET(fd, &working_set)) {
                desc_ready--;

                // is the current file descriptor the listening one?
                if (fd == listen_sd) {
                    do {
                        // accept the new connection
                        new_sd = accept(listen_sd, NULL, NULL);
                        if (new_sd < 0) {
                            if (errno != EWOULDBLOCK) {
                                return MOOSFail("Failed to accept().\n", "");
                            }
                            break;
                        }

                        MOOSTrace("New incoming connection configured.\n");
                        FD_SET(new_sd, &master_set);
                        if (new_sd > max_sd) { max_sd = new_sd; }
                    } while (new_sd != -1);
                } else {
                    bool close_connection = false;

                    do {
                        packet_t packet;
                        memset(&packet, 0, sizeof(packet));

                        rc = recv(fd, &packet, sizeof(packet), 0);

                        // break out if error occured
                        if (rc < 0) {
                            if (errno != EWOULDBLOCK) {
                                MOOSTrace("Failed to receive.");
                                close_connection = true;
                            }
                            break;
                        }

                        // check for a timeout
                        if (rc == 0) {
                            MOOSTrace("Connection closed.\n");
                            close_connection = true;
                            break;
                        }

                        // handle the received tcp packet data
                        if (!handle_received_packet(&packet, fd, rc)) {
                            MOOSTrace("Error processing packet.\n");
                            close_connection = true;
                            break;
                        }
                    } while(true);

                    // a closed connection needs cleanup
                    if (close_connection) {
                        close(fd);
                        FD_CLR(fd, &master_set);
                        if (fd == max_sd) {
                            while ((FD_ISSET(max_sd, &master_set) == false)) { max_sd--; }
                        }
                    }
                }
            }
        }

        return true;
    }

    // ---------------------------------------------------------------------------------
    // handles messages from the moosdb
    // ---------------------------------------------------------------------------------
    bool OnNewMail(MOOSMSG_LIST &Mail) {
        MOOSMSG_LIST::iterator q;
        for (q = Mail.begin(); q != Mail.end(); q++) {
            q->Trace();
        }
        return true;
    }

private:

    enum packet_types {
        TYPE_GENERIC = 0,     // 0 Generic packet type
        //TYPE_GENERIC_LOG,     // 1
        TYPE_SENSOR = 2,      // 2 Sensor readings
        //TYPE_SENSOR_LOG,      // 3
        //TYPE_CONTROLS,        // 4 SCOTTY servo positions and motor speed
        //TYPE_CONTROLS_LOG,    // 5
        TYPE_COMMAND = 6,     // 6 Command from one controller to another
        //TYPE_COMMAND_LOG,     // 7
        //TYPE_LBL_POS,         // 8 Position information from Woods Hole Modem or GPS
        //TYPE_LBL_POS_LOG,     // 9
        //TYPE_WHM_MSG,         // 10 Message received from the WHOI acoustic modem
        //TYPE_WHM_MSG_LOG,     // 11
        //TYPE_GPGGA,           // 12 GPS info
        //TYPE_GPGGA_LOG,       // 13
        //TYPE_BUOY_POS,        // 14 LBL buoy positions sent from Uhura to Kirk.
        //TYPE_BUOY_POS_LOG,    // 15
        //TYPE_KALMAN,          // 16 Kalman filter info
        //TYPE_KALMAN_LOG,      // 17
        //TYPE_IMU,             // 18 "IMU" message contents from the Archangel IMU
        //TYPE_IMU_LOG,         // 19
        //TYPE_TELEMETRY,       // 20 Telemetry packet sent from Kirk to DSP
        //TYPE_TELEMETRY_LOG,   // 21
        //TYPE_AHRS,            // 22 "AHRS" messages from the Archangel IMU
        //TYPE_AHRS_LOG,        // 23
        //TYPE_SYNCH_RANGE,     // 24 Synchronous navigation range
        //TYPE_SYNCH_RANGE_LOG, // 25
        //TYPE_KALMSHIP,        // 26 Used for logging of the ship EKF
        //TYPE_KALMSHIP_LOG,    // 27
        //TYPE_DEPTH,           // 28 Used for high rate depth sensor logging
        //TYPE_DEPTH_LOG,       // 29
        NUM_PACKET_TYPEIDS
    };

    enum command_types {
        CMD_SET_TRIM,           /* 0(0x00) Request */
        CMD_ABORT,              /* 1(0x01) Abort/stop current mission */
        CMD_START_MISSION,      /* 2(0x02) Start running a new mission
                                   params[0]: Mission number to run
								   params[1]: Run number;
								   params[2]: Vehicle position ID in formation
								   params[3]: Vehicle ID;
								   params[4]: Leader flag (1 = vehicle is the leader; 0 = not leader)
								   params[5]: LBL nav ping cycle ID
								   params[6]: Day
								   params[7]: Month
								   params[8]: Year */
        CMD_REQUEST_SENSORS = 21
    };

    // header packet
    typedef struct {
        uint16_t type;            /**< Identifies the type of packet */
        uint16_t source;          /**< Device ID of the packet's sender */
        uint16_t dest;            /**< Device ID of the packet's intended destination */
        uint16_t reserved;        /**< Reserved for future use */
        uint32_t ms;              /**< Millisecond time stamp */
    } packet_header_t;

    // generic packet
    typedef struct {
        packet_header_t header;
        char payload[PACKET_SIZE - sizeof(packet_header_t)]; /**< Raw payload Bytes */
    } packet_t;

    // command packet
    typedef struct {
        packet_header_t header;   /**< Packet header */
        int16_t command;          /**< Command ID from command_types */
        int16_t params[10];       /**< Command parameters */
        char padding[PACKET_SIZE - (sizeof(packet_header_t) + 11*sizeof(int16_t))];
    } packet_command_t;

    // sensor packet
    typedef struct {
        packet_header_t header;
        char payload[PACKET_SIZE - sizeof(packet_header_t)]; /**< Raw payload Bytes */
    } packet_sensor_t;

    // ---------------------------------------------------------------------------------
    // handles any incomming packet
    // ---------------------------------------------------------------------------------
    bool handle_received_packet(packet_t *packet, int fd, int len) {
        bool rc = true;

        // print out the received data just for kicks
        MOOSTrace("  ");
        for (int j = 0; j < len; j++) {
            if (j > 0) { MOOSTrace(":"); }
            MOOSTrace(MOOSFormat("%02x", ((uint8_t*)packet)[j]));
        }
        MOOSTrace("\n");

        // case statement to handle packet types
        switch (packet->header.type) {
            case TYPE_GENERIC:
                MOOSTrace("Received generic packet.\n");
                break;
            case TYPE_COMMAND:
                MOOSTrace("Received command packet.\n");
                rc = handle_command_packet((packet_command_t*)packet, fd);
                break;
            default:
                MOOSTrace("Received packe.\n");
                break;
        }
        return rc;
    }

    // ---------------------------------------------------------------------------------
    // handles a command packet
    // currently only start/abort/sensor packets
    // ---------------------------------------------------------------------------------
    bool handle_command_packet(packet_command_t *packet, int fd) {
        packet_sensor_t sensor_packet;
        bool rc = true;

        // check command rule
        switch (packet->command) {
            case CMD_ABORT:
                MOOSTrace("  ABORT packet detected, triggering return sequence...\n");
                rc = Notify("RETURN", 1.0) | Notify("DEPLOY", 0.0);
                break;
            case CMD_START_MISSION:
                MOOSTrace("  START_MISSION packet detected, initiating mission...\n");
                rc = Notify("RETURN", 0.0) | Notify("DEPLOY", 1.0);
                break;
            case CMD_REQUEST_SENSORS:
                MOOSTrace("  REQUEST_SENSORS packet detected, sending sensor data...\n");
                memset(&sensor_packet, 0, sizeof(sensor_packet));
                sensor_packet.header.type = TYPE_SENSOR;

                // add sensor payload here as necessary

                if ((send(fd, &sensor_packet, sizeof(sensor_packet), 0) < 0)) {
                    MOOSTrace("Sending Failed.");
                    rc = false;
                }
                break;
            default:
                break;
        }
        return rc;
    }

    // private class members
    int listen_sd, max_sd, new_sd;
    struct sockaddr_in addr;
    fd_set master_set;
    int port;
};
