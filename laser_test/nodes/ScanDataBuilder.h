/*
 * ScanDataBuilder.h
 *
 *  Created on: 28 Feb 2012
 *      Author: Jack Hargreaves
 */

#include <stdint.h>
#include "ScanData.h"
#include <vector>

#ifndef SCANDATABUILDER_H_
#define SCANDATABUILDER_H_

class ScanDataBuilder {
public:
    ScanDataBuilder();
    ScanDataBuilder(size_t expected_data_count);
    virtual ~ScanDataBuilder();

    // attempts to parse as much data as possible
    // returns true if the ScanData object is ready, false otherwise (ie builder needs more data)
    // buffer and size are modified to reflect how much data was read
    bool Parse(char*& buffer, size_t& size);
    ScanData GetData();

private:

    char irrelevant_data[15];
    size_t irrelevant_data_len;

    ScanData data;
    size_t method_index;

    // returns the number of characters until the next space
    int32_t get_token_length(char* src, size_t src_size);

    // these methods will attempt to parse the data from the buffer and
    // advance the buffer if they succeed (returning true)
    // otherwise they will leave src alone and return false
    bool get_next_string(char*& src, size_t& src_size, char* dest, size_t dest_size);
    bool get_next_uint8(char*& src, size_t& src_size, uint8_t& i);
    bool get_next_uint16(char*& src, size_t& src_size, uint16_t& i);
    bool get_next_uint32(char*& src, size_t& src_size, uint32_t& i);
    bool get_next_uint64(char*& src, size_t& src_size, uint64_t& i);
    bool get_next_float(char*& src, size_t& src_size, float& f);

    bool parse_command_type(char*& buffer, size_t& size, ScanData& data);
    bool parse_command(char*& buffer, size_t& size, ScanData& data);
    bool parse_version_number(char*& buffer, size_t& size, ScanData& data);
    bool parse_device_number(char*& buffer, size_t& size, ScanData& data);
    bool parse_serial_number(char*& buffer, size_t& size, ScanData& data);

    bool parse_device_status(char*& buffer, size_t& size, ScanData& data);
    bool parse_telegram_counter(char*& buffer, size_t& size, ScanData& data);
    bool parse_scan_counter(char*& buffer, size_t& size, ScanData& data);
    bool parse_time_since_startup(char*& buffer, size_t& size, ScanData& data);
    bool parse_time_of_transmission(char*& buffer, size_t& size, ScanData& data);

    bool parse_status_of_digital_inputs(char*& buffer, size_t& size, ScanData& data);
    bool parse_status_of_digital_outputs(char*& buffer, size_t& size, ScanData& data);
    bool parse_reserved_byte(char*& buffer, size_t& size, ScanData& data);
    bool parse_scan_frequency(char*& buffer, size_t& size, ScanData& data);
    bool parse_measurement_frequency(char*& buffer, size_t& size, ScanData& data);

    bool parse_encoders(char*& buffer, size_t& size, ScanData& data);

    bool parse_amount_of_16_bit_channels(char*& buffer, size_t& size, ScanData& data);
    bool parse_content(char*& buffer, size_t& size, ScanData& data);
    bool parse_scale_factor(char*& buffer, size_t& size, ScanData& data);
    bool parse_scale_factor_offset(char*& buffer, size_t& size, ScanData& data);
    bool parse_start_angle(char*& buffer, size_t& size, ScanData& data);
    bool parse_steps(char*& buffer, size_t& size, ScanData& data);
    bool parse_amount_of_data(char*& buffer, size_t& size, ScanData& data);
    bool parse_data(char*& buffer, size_t& size, ScanData& data);

    // checks that end of message is as expected (6 zeros)
    bool parse_irrelevant_data(char*& buffer, size_t& size);

};

#endif /* SCANDATABUILDER_H_ */
