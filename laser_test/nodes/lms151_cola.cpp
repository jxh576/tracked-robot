/*
 * Desc: Driver for the SICK LMS151 unit
 * Author: Nico Blodow and Radu Bogdan Rusu
 * Modified by: Kasper Vinther
 * Modified by: Jack Hargreaves to work with ROS
 * Date: 24/02/2012
 *
 * WARNING: This driver only supports 18 hours of continuous operation before its timestamp roles over
 */

#include <sys/socket.h>
#include <netdb.h>
#include <unistd.h>

#include "lms151_cola.h"

#include <string.h>
#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <vector>

#include <boost/thread/mutex.hpp>

#include "player_laser_data.h"
#include "ScanData.h"
#include "ScanDataBuilder.h"

////////////////////////////////////////////////////////////////////////////////
// Constructor.
lms100_cola::lms100_cola(const char* host, size_t port, size_t debug_mode,
        size_t expected_data_count) :
    expected_data_count(expected_data_count), hostname(host), portno(port),
            verbose(debug_mode), bufferContents(0) {

    memset(command, 0, SEND_BUFFER_SIZE);
    memset(buffer, 0, RECV_BUFFER_SIZE);
    ROS_DEBUG("Constructed cola protocol object\n");

}

lms100_cola::lms100_cola(const char* host, size_t port, size_t debug_mode) :
    expected_data_count(600), hostname(host), portno(port),
            verbose(debug_mode), bufferContents(0) {

    memset(command, 0, SEND_BUFFER_SIZE);
    memset(buffer, 0, RECV_BUFFER_SIZE);
    ROS_DEBUG("Constructed cola protocol object\n");

}

////////////////////////////////////////////////////////////////////////////////
// Connect to the LMS100 unit using hostname:portno
// Returns 0 if connection was successful, -1 otherwise
int lms100_cola::Connect() {
    // Create a socket
    sockfd = socket(AF_INET, SOCK_STREAM, 0);
    if (sockfd < 0)
        return (-1);

    // Get the network host entry
    server = gethostbyname((const char *) hostname);
    if (server == NULL)
        return (-1);

    // Fill in the sockaddr_in structure values
    memset((char *) &serv_addr, 0, sizeof(serv_addr));
    serv_addr.sin_family = AF_INET;
    serv_addr.sin_port = htons(portno);
    memcpy((char *) &serv_addr.sin_addr.s_addr, (char *) server->h_addr,
            server->h_length);

    ROS_DEBUG("attempt to connect");

    // Attempt to connect
    if (connect(sockfd, (const sockaddr*) &serv_addr, sizeof(serv_addr)) < 0)
        return (-1);

    return (0);
}

////////////////////////////////////////////////////////////////////////////////
// Disconnect from the LMS100 unit
// Returns 0 if connection was successful, -1 otherwise
int lms100_cola::Disconnect() {
    return (close(sockfd));
}

////////////////////////////////////////////////////////////////////////////////
// Get the current laser unit configuration and return it into Player format
player_laser_config lms100_cola::GetConfiguration() {
    player_laser_config_t cfg;
    cfg = Configuration;
    return cfg;
}

////////////////////////////////////////////////////////////////////////////////
// Configure scan data output (this function does not work yet! The LMS100
// answers with error code "sFA 8")
int lms100_cola::ConfigureScanDataOutput() {
    char cmd[SEND_BUFFER_SIZE];
    uint16_t OutputChannel = 1;
    bool Remission = 0;
    int Resolution = 1;
    int Unit = 0;
    uint16_t Encoder = 0;
    bool Position = 0;
    bool DeviceName = 0;
    bool Comment = 0;
    bool Time = 0;
    uint16_t Outputinterval = 1;
    snprintf(cmd, SEND_BUFFER_SIZE,
            "sWN LMDscandatacfg %d %d %d %d %d %d %d %d %d %d", OutputChannel,
            Remission, Resolution, Unit, Encoder, Position, DeviceName,
            Comment, Time, Outputinterval);

    SendCommand(cmd);

    if (ReadAnswer() != 0)
        return (-1);

    return (0);
}

////////////////////////////////////////////////////////////////////////////////
// Set the desired userlevel by logging in with the appropriate password
int lms100_cola::SetUserLevel(int8_t userlevel, const char* password) {
    char cmd[SEND_BUFFER_SIZE];
    snprintf(cmd, SEND_BUFFER_SIZE, "sMN SetAccessMode %d %s", userlevel,
            password);
    SendCommand(cmd);

    return (ReadAnswer());
}

////////////////////////////////////////////////////////////////////////////////
// Terminate configuration and change back to userlevel 0
int lms100_cola::TerminateConfiguration() {
    const char* cmd = "sMN Run";
    SendCommand(cmd);

    return (ReadAnswer());
}

////////////////////////////////////////////////////////////////////////////////
// Set both resolution and frequency without going to a higher user level (?)
int lms100_cola::SetResolutionAndFrequency(float freq, float ang_res,
        float angle_start, float angle_range) {
    char cmd[SEND_BUFFER_SIZE];
    uint32_t frequ = (uint32_t) (freq * 100);
    uint8_t NumberSegments = 1;
    uint32_t ang_resu = (uint32_t) (ang_res * 10000);
    int32_t angle_star = (int32_t) ((angle_start + 45) * 10000); //Don't know why +45 is added
    int32_t angle_rang = (int32_t) ((angle_range + 45) * 10000);
    snprintf(cmd, SEND_BUFFER_SIZE, "sMN mLMPsetscancfg +%d +%d +%d %d +%d",
            frequ, NumberSegments, ang_resu, angle_star, angle_rang);
    SendCommand(cmd);

    int error = ReadAnswer();

    // If no error, parse the results
    if (error == 0) {
        strtok((char*) buffer, " ");
        strtok(NULL, " ");
        int ErrorCode = strtol(strtok(NULL, " "), NULL, 16);
        long int sf = strtol(strtok(NULL, " "), NULL, 16);
        long int re = strtol(strtok(NULL, " "), NULL, 16);

        if ((ErrorCode != 0) && (verbose))
            ROS_WARN("Warning: got an error code %d", ErrorCode);

        memcpy(&Configuration.scanning_frequency, &sf, sizeof(uint32_t));
        memcpy(&Configuration.resolution, &re, sizeof(uint32_t));

        ROS_INFO("Measured value quality is: %ld [2500-5000]",
                strtol (strtok (NULL, " "), NULL, 16));
    }

    return (error);
}

////////////////////////////////////////////////////////////////////////////////
// Start a measurement for both distance and intensity or just distance.
int lms100_cola::StartMeasurement(bool intensity) {
    char cmd[SEND_BUFFER_SIZE];
    snprintf(cmd, SEND_BUFFER_SIZE, "sMN LMCstartmeas");
    SendCommand(cmd);
    ReadAnswer();

    //Status of the LMS must be ASCII 7 which means it is ready for measurement
    ROS_INFO("Waiting for LMS to be ready...");
    snprintf(cmd, SEND_BUFFER_SIZE, "sRN STlms");
    do {
        SendCommand(cmd);
        buffer[10] = 0;
        n = read(sockfd, buffer, 1);

        if (n < 0)
            return (-1);

        if (buffer[0] != 0x02) {
            ROS_INFO("Warning: expected STX!");
            n = read(sockfd, buffer, 3000);
            return (-1);
        }
        n = read(sockfd, buffer, 3000);
    } while (buffer[10] != 55);
    ROS_INFO("LMS ready\n");
    ROS_INFO("buffer contents: %s", buffer);

    // Start continous measurement value output
    int startmeasure = 1;
    snprintf(cmd, SEND_BUFFER_SIZE, "sEN LMDscandata %d", startmeasure);
    SendCommand(cmd);
    ReadAnswer();

    return (0);
}


////////////////////////////////////////////////////////////////////////////////
// Read a measurement, returns true if there was a measurement to read
bool lms100_cola::ReadMeasurement(sensor_msgs::LaserScan& laser_scan) {
    // TODO: fix me! -- i do not add data to laser_scan

    ScanData scan_data;
    if (!ReadFromQueueTo(scan_data)) {
        return false;
    }

    std::cout << "power up time: " << scan_data.time_since_startup << std::endl;
    std::cout << "tranmission time: " << scan_data.time_of_transmission << std::endl;

    laser_scan.header.seq = scan_data.scan_counter;
    laser_scan.header.stamp = ros::Time(scan_data.time_since_startup / NANOSECONDS_IN_SECOND,
                scan_data.time_since_startup % NANOSECONDS_IN_SECOND);
    laser_scan.header.frame_id = 1; // TODO: I have no idea if this is right or not. Makes it part of global frame

    float f = *((float*)(&scan_data.start_angle));
    ROS_INFO("start angle: %i, start_angle as float: %f, steps: %i", scan_data.start_angle, f, scan_data.steps);

    laser_scan.angle_increment = DTOR(scan_data.steps / 10000.f);
    laser_scan.angle_min = DTOR(scan_data.start_angle / 10000.f); //DTOR(config.min_angle);
    laser_scan.angle_max = laser_scan.angle_min + (laser_scan.angle_increment * (scan_data.amount_of_data - 1));

    laser_scan.range_max = 50; // according to manual for lms 151
    laser_scan.range_min = 0.5;

    // manual states that measurement frequency is in units of 100ths of Hz
    // eg. 2500 -> 25Hz
    laser_scan.time_increment = 0.01f / scan_data.measurement_frequency;
    // manual states that scan frequency is:
    // "Frequency between two separate measurements in 100 Hz"
    // eg. 2500 -> 25Hz (I hope)
    laser_scan.scan_time = 100.f / scan_data.scan_frequency;

    laser_scan.ranges.reserve(scan_data.data.size());

    for (std::vector<uint16_t>::iterator iter = scan_data.data.begin(),
            end = scan_data.data.end(); iter != end; ++iter) {

        // ignore scan_data.scale_factor as we expect to always be 1
        laser_scan.ranges.push_back((*iter) * MM_TO_M_RATIO);
    }

    return true;

//    // copy range data from the buffer into player_data struct.
//    // only values between 0 and 180 degrees are used
//    // In player_data it is -90 to 90 since we scan in the x-direction
//    // JWB 9/5/11 modified to return all readings
//    if (Configuration.resolution == 0.5) {
//        player_data.ranges_count = (meas_header.NumberData);
//        player_data.ranges = new float[player_data.ranges_count];
//        //   for (int i = 0; i < 90 ; i++){
//        //     message_cutter ();
//        //   }
//        ROS_DEBUG("min angle: %f  max angle: %f resolution  %f Scale %u", player_data.min_angle , player_data.max_angle , Configuration.resolution, meas_header.ScalingFactor );
//
//        for (int i = 0; i < (meas_header.NumberData - 1); i++) {
//            uint16_t val = message_cutter();
//            player_data.ranges[i] = (((float) val) * mm_to_m);
//            if (player_data.ranges[i] < 0) {
//                ROS_WARN("[%i] dist: %f  measured %u scale %u ", i, player_data.ranges[i], val, meas_header.ScalingFactor );
//            } else {
//                ROS_DEBUG("[%i] dist: %f  measured %u scale %u ", i, player_data.ranges[i], val, meas_header.ScalingFactor );
//            }
//        }
//    }
//
//    if (Configuration.resolution == 0.25) {
//        player_data.ranges_count = (meas_header.NumberData - 361);
//        player_data.ranges = new float[player_data.ranges_count];
//        for (unsigned int i = 0; i < 180; i++) {
//            message_cutter();
//        }
//        for (unsigned int i = 0; i < player_data.ranges_count; i++) {
//            uint16_t val = message_cutter();
//            player_data.ranges[i] = (((float) val) * mm_to_m);
//
//            ROS_DEBUG("[%i] dist: %f  measured %u scale %u", i, player_data.ranges[i], val, meas_header.ScalingFactor );
//        }
//    }
//
//    //return (player_data);
//    return false;
}

bool lms100_cola::ReadFromQueueTo(ScanData& data) {
    boost::mutex::scoped_lock lock(mutex);
    if (MeasurementQueue.empty()) {
        return false;
    }
    data = MeasurementQueue.front();
    MeasurementQueue.pop_front();
    return true;
}

bool lms100_cola::WriteToQueue(ScanData& element) {
    boost::mutex::scoped_lock lock(mutex);
    if (MeasurementQueue.size() == MAX_QUEUE_SIZE) {
        // queue is filling up to quickly -- purge it
        MeasurementQueue.clear();
    }
    MeasurementQueue.push_back(element);
    return true;
}

////////////////////////////////////////////////////////////////////////////////
// Stop a measurement
int lms100_cola::StopMeasurement() {
    char cmd[SEND_BUFFER_SIZE];
    snprintf(cmd, SEND_BUFFER_SIZE, "sMN LMCstopmeas");//"sMN mLRstopdata"
    SendCommand(cmd);
    return (ReadAnswer());
}

////////////////////////////////////////////////////////////////////////////////
// Send a command to the laser unit. Returns -1 on error.
int lms100_cola::SendCommand(const char* cmd) {
    if (strncmp((const char*) cmd, "sRN STlms", 9) != 0)
        ROS_DEBUG("Sent: \"%s\"", cmd);
    assemblecommand((unsigned char *) cmd, strlen(cmd));

    n = write(sockfd, command, commandlength);

    if (n < 0)
        return (-1);

    return (0);
}

////////////////////////////////////////////////////////////////////////////////
// Read a result from the laser unit.
int lms100_cola::ReadResult() {
    memset(buffer, 0, RECV_BUFFER_SIZE);
    n = read(sockfd, buffer, 1);

    if (n < 0)
        return (-1);

    if (buffer[0] != 0x02) {
        if (verbose)
            printf("> E: expected STX!\n");
        n = read(sockfd, buffer, RECV_BUFFER_SIZE);
        return (-1);
    }

    // Put the message in the buffer
    n = read(sockfd, buffer, RECV_BUFFER_SIZE);

    if ((buffer[0] != 0x20))
        ROS_DEBUG("Received: \"%s\"", buffer);

    // Check for error
    if (strncmp((const char*) buffer, "sFA", 3) == 0) {
        strtok((char*) buffer, " ");
        ROS_WARN("E: Got an error message with code 0x%s", buffer); //strtok (NULL, " "));
        return (-1);
    }

    if (buffer[0] == 's')
        return (0);
    else if (buffer[0] == 0x20)
        return (ReadResult());

    return (0);
}

////////////////////////////////////////////////////////////////////////////////
// Read an answer from the laser unit
int lms100_cola::ReadAnswer() {
    return ReadResult();
}

////////////////////////////////////////////////////////////////////////////////
// Read a confirmation and an answer from the laser unit
int lms100_cola::ReadConfirmationAndAnswer() {
    ReadResult();
    if (buffer[0] == 's' && buffer[1] == 'F' && buffer[2] == 'A')
        return (-1);
    else {
        return ReadResult();
    }
}

////////////////////////////////////////////////////////////////////////////////
// adds STX and ETX to command to be sent
int lms100_cola::assemblecommand(unsigned char* cmd, int len) {
    int index = 0;

    command[0] = 0x02; // Messages start with 1 STX

    for (index = 0; index < len; index++) {
        command[index + 1] = cmd[index];
    }

    command[1 + len] = 0x03; // Messages end with 1 ETX

    commandlength = 2 + len;
    return commandlength;
}

////////////////////////////////////////////////////////////////////////////////
// Cuts out data from the buffer and converts from string to integer.
uint64_t lms100_cola::message_cutter() {
    uint8_t counter = 0;
    uint64_t output = 0;

    // Data is seperated with white space (hex 0x20 or decimal 32)
    while (buffer[index_k] != 32) {
        index_k += 1;
        counter += 1;
    }

    // Create a string
    char streng[counter + 1];
    for (int i = 0; i < counter; i++) {
        streng[i] = buffer[index_k - counter + i];
    }
    streng[counter] = '\0';

    // Convert the string to an integer
    output = (uint64_t) strtol(streng, NULL, 16);

    // Increment index_k to point to the next data in the message
    index_k += 1;

    return output;
}

bool lms100_cola::ReadDataFromLaserScanner(ScanData& element) {

    if (bufferContents != 0 && buffer[0] != STX_BYTE) {
        ROS_FATAL("was expecting stx_byte, found: %c", buffer[0]);
        Shutdown();
    }

    if (bufferContents == RECV_BUFFER_SIZE) {
        ROS_FATAL("buffer full and no room for more data");
        Shutdown();
    }

    ScanDataBuilder builder(expected_data_count);

    while (true) {
        int n = read(sockfd, buffer + bufferContents, RECV_BUFFER_SIZE
                - bufferContents - 1);

        bufferContents += n;
        buffer[bufferContents] = '\0'; // null terminate buffer to prevent reading old content by accident

        char* temp_buf = buffer;
        size_t temp_size = bufferContents;

        bool result = builder.Parse(temp_buf, temp_size);

        // move remaining content to front of buffer
        memmove(buffer, temp_buf, temp_size);
        bufferContents = temp_size;
        buffer[bufferContents] = '\0';

        if (result) {
            break;
        }
    }

    element = builder.GetData();

    return true;
}

void lms100_cola::Shutdown() {
    this->running = false;
}

inline bool lms100_cola::IsRunning() {
    return this->running;
}

void lms100_cola::Run() {

    if (Connect() != 0) {
        ROS_FATAL("Failed to connect to laser scanner");
        return;
    }
    ROS_DEBUG("low level thread started");
    this->StartMeasurement();

    ScanData element;

    while (IsRunning()) {
        // if enough data to get a scan data response then add to queue
//        ROS_INFO("\n\nread data from scanner");
        if (ReadDataFromLaserScanner(element)) {
            // append to queue
//            ROS_INFO("write data to queue");
            WriteToQueue(element);
        }
//        ROS_INFO("end of main loop");
    }

    ROS_DEBUG("low level thread finished");

    this->StopMeasurement();
    this->Disconnect();

}

