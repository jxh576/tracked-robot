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
    uint32_t time_since_startup;
    uint32_t time_of_transmission;
    uint8_t status_of_digital_inputs[2];
    uint8_t status_of_digital_outputs[2];
    uint16_t reserved_byte;
    uint32_t scan_frequency;
    uint32_t measurement_frequency;
    struct Encoder encoders;
    uint16_t amount_of_16_bit_channels;
    char content[6];

    float scale_factor;
    float scale_factor_offset;

    uint32_t start_angle;
    uint16_t steps;
    uint16_t amount_of_data;
    // ranges in mm
    std::vector<uint16_t> data;
};

#endif /* SCANDATA_H_ */
