/*
 * Desc: Driver for the SICK LMS151 unit
 * Author: Nico Blodow and Radu Bogdan Rusu
 * Modified by: Kasper Vinther
 * Modified by: Jack Hargreaves to work with ROS
 * Date: 24/02/2012
 */

#ifndef LMS151_COLA_H_
#define LMS151_COLA_H_

#include <sys/types.h>
#include <queue>
#include <netinet/in.h>
#include <iostream>
#include <string>

#include <stdint.h>

#include <boost/thread/mutex.hpp>

#include "ScanData.h"
#include "ScanDataBuilder.h"

#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>

////////////////////////////////////////////////////////////////////////////////
// Maths stuff

#ifndef M_PI
#define M_PI        3.14159265358979323846
#endif

// Convert degrees to radians
#ifndef DTOR
#define DTOR(d) ((d) * M_PI / 180)
#endif

typedef struct lms151_cola_configuration {
    /** Scan resolution [rad].  */
    float resolution;
    /** Scanning frequency [Hz] */
    float scanning_frequency;
} lms151_cola_configuration;

////////////////////////////////////////////////////////////////////////////////
class lms100_cola {
public:

    static const uint32_t MAX_QUEUE_SIZE = 2;
    static const char STX_BYTE = 0x02;
    static const char ETX_BYTE = 0x03;
    static const float MM_TO_M_RATIO = 0.001f;
    static const uint32_t NANOSECONDS_IN_SECOND = 1000000;

    lms100_cola(const char* host, size_t port, size_t debug_mode);
    lms100_cola(const char* host, size_t port, size_t debug_mode, size_t expected_data_count);
    ~lms100_cola() {}

    // Creates socket, connects
    int Connect();
    int Disconnect();

    // Configuration parameters
    int SetResolutionAndFrequency(float freq, float ang_res, float angle_start,
            float angle_range);
    int ConfigureScanDataOutput();

    int StartMeasurement(bool intensity = true);
    bool ReadMeasurement(sensor_msgs::LaserScan& laser_scan);
    int StopMeasurement();

    int SetUserLevel(int8_t userlevel, const char* password);

    int TerminateConfiguration();

    int SendCommand(const char* cmd);
    int ReadResult();
    // for "Variables", Commands that only reply with one Answer message
    int ReadAnswer();
    // for "Procedures", Commands that reply with localBuffer, sizeof(localBuffer)a Confirmation message and an Answer message
    int ReadConfirmationAndAnswer();

    lms151_cola_configuration GetConfiguration();

    // main thread to run to get data
    void Run();

    void Shutdown();

    inline bool IsRunning();

private:

    static const uint32_t RECV_BUFFER_SIZE = 4096;
    static const uint32_t SEND_BUFFER_SIZE = 1024;
    static const int CMD_BUFFER_SIZE = 255;

    // the amount of data points expected in message from the laser scanner
    size_t expected_data_count;

    boost::mutex mutex;

    // assembles frame to be sent
    int assemblecommand(unsigned char* command, int len);

    const char* hostname;
    int sockfd, portno, n;
    struct sockaddr_in serv_addr;
    struct hostent *server;

    // Internal Parameters:
    int verbose;
    lms151_cola_configuration configuration;

    // for reading:
    uint32_t bufferContents;
    char buffer[RECV_BUFFER_SIZE];
    uint16_t index_k;
    unsigned int bufferlength;

    // for sending:
    char command[SEND_BUFFER_SIZE];
    int commandlength;
    std::deque<ScanData> MeasurementQueue;

    // used to indicate if the driver should continue running or shutdown
    bool running;

    // Safely reads an element from the queue into the buffer
    // returns false if queue is empty
    bool ReadFromQueueTo(ScanData& data);

    // Safely writes an element to the queue
    // returns false if the queue was full
    bool WriteToQueue(ScanData& element);

    // Reads data off of the socket and writes it to the element
    // returns false if there were any errors? Maybe throw exception?
    bool ReadDataFromLaserScanner(ScanData& element);
};

#endif /* LMS151_COLA_H_ */
