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
    // buffer is changed to the first unused bit of data and size decreased by how much data was consumed
    bool Parse(char*& buffer, size_t& size);
    ScanData GetData();


private:

    static const char* irrelevant_data;
    static const size_t irrelevant_data_len;

    ScanData data;
public:
    int32_t method_index;
private:

    // this method will attempt to parse the data from the buffer and
    // advance the buffer if they succeed (returning true)
    // otherwise they will leave src alone and return false
    template <class T>
    bool get_next(char*& src, size_t& src_size, T& datum);
    template <class T>
    bool get_next_multiple_times(char*& src, size_t& src_size, T*& data, size_t data_size);
    bool get_next_string(char*& src, size_t& src_size, char* dest, size_t dest_size);
    // will parse one byte, checking that it is the same as the byte parameter
    bool parse_byte(char*& buffer, size_t& size, char byte);

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
