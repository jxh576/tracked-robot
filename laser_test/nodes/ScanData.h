/*
 * ScanData.h
 *
 *  Created on: 28 Feb 2012
 *      Author: Jack Hargreaves
 */

#include <stdint.h>
#include <vector>

#ifndef SCANDATA_H_
#define SCANDATA_H_

struct Encoder {

    // if amount_of_encoders is zero then other parameters are available
    uint16_t amount_of_encoders;
    uint16_t encoder_position;
    uint16_t encoder_speed;

};

class ScanData {
public:
    ScanData();
    ScanData(size_t expected_data_count);
    virtual ~ScanData();

    char command_type[4];
    char command[12];
    uint16_t version_number;
    uint16_t device_number;
    uint32_t serial_number;
    uint16_t device_status[2];
    uint16_t telegram_counter;
    uint16_t scan_counter;
    // in nano seconds
    uint32_t time_since_startup;
    // in nano seconds
    uint32_t time_of_transmission;
    uint8_t status_of_digital_inputs[2];
    uint8_t status_of_digital_outputs[2];
    uint16_t reserved_byte;
    // frequency of scan in 100ths of Hz (eg. 2500 -> 25Hz)
    uint32_t scan_frequency;
    // frequency of individual scan measurements in 100s of Hz (eg. 25 -> 2500Hz)
    uint32_t measurement_frequency;
    struct Encoder encoders;
    uint16_t amount_of_16_bit_channels;
    char content[6];

    // either 1 or 2 -- for lms151 it is 1
    float scale_factor;
    // always 0 for lms
    float scale_factor_offset;

    // starting angle in 10,000th of degrees
    uint32_t start_angle;
    // angular step in 10,000th of degrees
    uint16_t steps;
    uint16_t amount_of_data;
    // ranges in mm
    std::vector<uint16_t> data;
};

#endif /* SCANDATA_H_ */
