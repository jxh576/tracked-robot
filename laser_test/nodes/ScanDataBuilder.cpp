/*
 * ScanDataBuilder.cpp
 *
 *  Created on: 28 Feb 2012
 *      Author: Jack Hargreaves
 */



#include "lms151_cola.h"
#include "ScanDataBuilder.h"
#include <string.h>
#include <stdlib.h>
#include <iostream>
#include <stdio.h>

const char* ScanDataBuilder::irrelevant_data = "0 0 0 0 0 0";

const size_t ScanDataBuilder::irrelevant_data_len = strlen(ScanDataBuilder::irrelevant_data);

#define GET_NEXT_COMPILE_TIME_ASSERT(predicate) \
    typedef char __dummy[2*!!(predicate)-1]; // template class is too big! will cause unexpected results

// note an array (var[]) must be used and not a pointer (*var)
#define ARRAY_LENGTH(array) \
        (sizeof(array) / sizeof(*array))

ScanDataBuilder::ScanDataBuilder() :
        data(), method_index(-1) {
}

ScanDataBuilder::ScanDataBuilder(size_t expected_data_count) :
        data(expected_data_count), method_index(-1) {
}

ScanDataBuilder::~ScanDataBuilder() {
}

ScanData ScanDataBuilder::GetData() {
    return this->data;
}

template <class T>
bool ScanDataBuilder::get_next(char*& src, size_t& src_size, T& datum) {

    GET_NEXT_COMPILE_TIME_ASSERT(sizeof(T) <= sizeof(uint64_t));

    uint64_t as_unsigned_int;
    char* token_end;
    as_unsigned_int = strtoul(src, &token_end, 16);

    if (*token_end != ' ') {
        return false;
    }

//#ifndef NDEBUG
//    // does not work as tokens do not represent leading 0 bits
//    assert((token_end-src) / 2.0 == sizeof(T));
//#endif

    memcpy(&datum, &as_unsigned_int, sizeof(T));
    src_size -= (token_end - src) + 1;
    src = token_end + 1;
    return true;
}

template <class T>
bool ScanDataBuilder::get_next_multiple_times(char*& src, size_t& src_size, T*& data,
            size_t data_size) {

    char* original_src = src;
    size_t original_src_size = src_size;

    T temp_data[data_size];

    for (size_t i = 0; i < data_size; ++i) {
        if(!get_next<T>(src, src_size, temp_data[i])) {
            src = original_src;
            src_size = original_src_size;
            return false;
        }
    }

    memcpy(data, temp_data, data_size);
    return true;
}

bool ScanDataBuilder::get_next_string(char*& src, size_t& src_size, char* dest,
        size_t dest_size) {

    char* token_end = (char*) memchr(src, ' ', src_size);

    if (token_end == NULL) {
        return false;
    }

    if ((token_end - src) >= dest_size) {
        ROS_WARN("unexpected message format \"%s\"", src);
        return false;
    }

    memcpy(dest, src, dest_size - 1);
    dest[dest_size - 1] = '\0';

    src_size -= (token_end - src) + 1;
    src = token_end + 1;

    return true;
}

bool ScanDataBuilder::parse_byte(char*& buffer, size_t& size, char byte) {
    if (*buffer == byte) {
        ++buffer;
        --size;
        return true;
    } else {
        return false;
    }
}

bool ScanDataBuilder::Parse(char*& buffer, size_t& size) {

    char* prev_buf = NULL;
    bool success;
    while (method_index <= 25) {
//        std::cout << "method index: " << method_index <<std::endl;
        prev_buf = buffer;
        switch (method_index) {
        case -1:
            success = parse_byte(buffer, size, lms100_cola::STX_BYTE);
            break;
        case 0:
            success = parse_command_type(buffer, size, data);
            break;
        case 1:
            success = parse_command(buffer, size, data);
            break;
        case 2:
            success = parse_version_number(buffer, size, data);
            break;
        case 3:
            success = parse_device_number(buffer, size, data);
            break;
        case 4:
            success = parse_serial_number(buffer, size, data);
            break;
        case 5:
            success = parse_device_status(buffer, size, data);
            break;
        case 6:
            success = parse_telegram_counter(buffer, size, data);
            break;
        case 7:
            success = parse_scan_counter(buffer, size, data);
            break;
        case 8:
            success = parse_time_since_startup(buffer, size, data);
            break;
        case 9:
            success = parse_time_of_transmission(buffer, size, data);
            break;
        case 10:
            success = parse_status_of_digital_inputs(buffer, size, data);
            break;
        case 11:
            success = parse_status_of_digital_outputs(buffer, size, data);
            break;
        case 12:
            success = parse_reserved_byte(buffer, size, data);
            break;
        case 13:
            success = parse_scan_frequency(buffer, size, data);
            break;
        case 14:
            success = parse_measurement_frequency(buffer, size, data);
            break;
        case 15:
            success = parse_encoders(buffer, size, data);
            break;
        case 16:
            success = parse_amount_of_16_bit_channels(buffer, size, data);
            break;
        case 17:
            success = parse_content(buffer, size, data);
            break;
        case 18:
            success = parse_scale_factor(buffer, size, data);
            break;
        case 19:
            success = parse_scale_factor_offset(buffer, size, data);
            break;
        case 20:
            success = parse_start_angle(buffer, size, data);
            break;
        case 21:
            success = parse_steps(buffer, size, data);
            break;
        case 22:
            success = parse_amount_of_data(buffer, size, data);
            break;
        case 23:
            success = parse_data(buffer, size, data);
            break;
        case 24:
            success = parse_irrelevant_data(buffer, size);
            break;
        case 25:
            success = parse_byte(buffer, size, lms100_cola::ETX_BYTE);
            break;
        default:
            return false;
        }

        // method didn't have enough data to parse
        if (!success) {
            return false;
        }

        method_index += 1;
    }

    return true;
}

bool ScanDataBuilder::parse_command_type(char*& buffer, size_t& size,
        ScanData& data) {
    return get_next_string(buffer, size, data.command_type,
            sizeof(data.command_type));
}

bool ScanDataBuilder::parse_command(char* &buffer, size_t& size, ScanData& data) {
    return get_next_string(buffer, size, data.command, sizeof(data.command));
}

bool ScanDataBuilder::parse_version_number(char*& buffer, size_t& size,
        ScanData& data) {
    return get_next<uint16_t>(buffer, size, data.version_number);
}

bool ScanDataBuilder::parse_device_number(char*& buffer, size_t& size,
        ScanData& data) {
    return get_next<uint16_t>(buffer, size, data.device_number);
}

bool ScanDataBuilder::parse_serial_number(char*& buffer, size_t& size,
        ScanData& data) {
    return get_next<uint32_t>(buffer, size, data.serial_number);
}

bool ScanDataBuilder::parse_device_status(char*& buffer, size_t& size,
        ScanData& data) {
    uint16_t* temp = data.device_status;
    return get_next_multiple_times<uint16_t> (buffer, size, temp,
            ARRAY_LENGTH(data.device_status));
}

bool ScanDataBuilder::parse_telegram_counter(char*& buffer, size_t& size,
        ScanData& data) {
    return get_next<uint16_t>(buffer, size, data.telegram_counter);

}

bool ScanDataBuilder::ScanDataBuilder::parse_scan_counter(char*& buffer,
        size_t& size, ScanData& data) {
    return get_next<uint16_t>(buffer, size, data.scan_counter);
}

bool ScanDataBuilder::parse_time_since_startup(char*& buffer, size_t& size,
        ScanData& data) {
    return get_next<uint32_t>(buffer, size, data.time_since_startup);
}

bool ScanDataBuilder::parse_time_of_transmission(char*& buffer, size_t& size,
        ScanData& data) {
    return get_next<uint32_t>(buffer, size, data.time_of_transmission);
}

bool ScanDataBuilder::parse_status_of_digital_inputs(char*& buffer,
        size_t& size, ScanData& data) {
    uint8_t* temp = data.status_of_digital_inputs;
    return get_next_multiple_times<uint8_t> (buffer, size, temp,
            ARRAY_LENGTH(data.status_of_digital_inputs));
}

bool ScanDataBuilder::parse_status_of_digital_outputs(char*& buffer,
        size_t& size, ScanData& data) {
    uint8_t* temp = data.status_of_digital_outputs;
    return get_next_multiple_times<uint8_t> (buffer, size, temp,
            ARRAY_LENGTH(data.status_of_digital_outputs));
}

bool ScanDataBuilder::parse_reserved_byte(char*& buffer, size_t& size,
        ScanData& data) {
    return get_next<uint16_t>(buffer, size, data.reserved_byte);
}

bool ScanDataBuilder::parse_scan_frequency(char*& buffer, size_t& size,
        ScanData& data) {
    return get_next<uint32_t>(buffer, size, data.scan_frequency);
}
bool ScanDataBuilder::parse_measurement_frequency(char*& buffer, size_t& size,
        ScanData& data) {
    return get_next<uint32_t>(buffer, size, data.measurement_frequency);
}

bool ScanDataBuilder::parse_encoders(char*& buffer, size_t& size,
        ScanData& data) {

    char* original_buffer = buffer;
    size_t original_size = size;

    struct Encoder encoder;

    if (!get_next<uint16_t>(buffer, size, encoder.amount_of_encoders)) {
        buffer = original_buffer;
        size = original_size;
        return false;
    }

    if (encoder.amount_of_encoders == 0) {
        data.encoders = encoder;
        return true;
    }

    if (!get_next<uint16_t>(buffer, size, encoder.encoder_position)) {
        buffer = original_buffer;
        size = original_size;
        return false;
    }

    if (!get_next<uint16_t>(buffer, size, encoder.encoder_speed)) {
        buffer = original_buffer;
        size = original_size;
        return false;
    }

    data.encoders = encoder;
    return true;

}

bool ScanDataBuilder::parse_amount_of_16_bit_channels(char*& buffer,
        size_t& size, ScanData& data) {
    return get_next<uint16_t>(buffer, size, data.amount_of_16_bit_channels);
}

bool ScanDataBuilder::parse_content(char*& buffer, size_t& size, ScanData& data) {
    return get_next_string(buffer, size, data.content, sizeof(data.content));
}

bool ScanDataBuilder::parse_scale_factor(char*& buffer, size_t& size,
        ScanData& data) {
    return get_next<float>(buffer, size, data.scale_factor);
}

bool ScanDataBuilder::parse_scale_factor_offset(char*& buffer, size_t& size,
        ScanData& data) {
    return get_next<float>(buffer, size, data.scale_factor_offset);
}

bool ScanDataBuilder::parse_start_angle(char*& buffer, size_t& size,
        ScanData& data) {
    return get_next<int32_t>(buffer, size, data.start_angle);
}

bool ScanDataBuilder::parse_steps(char*& buffer, size_t& size, ScanData& data) {
    return get_next<uint16_t>(buffer, size, data.steps);
}

bool ScanDataBuilder::parse_amount_of_data(char*& buffer, size_t& size,
        ScanData& data) {
    return get_next<uint16_t>(buffer, size, data.amount_of_data);
}

bool ScanDataBuilder::parse_data(char*& buffer, size_t& size, ScanData& data) {
    while (data.data.size() < data.amount_of_data) {
        uint16_t datum;
        if (!get_next<uint16_t>(buffer, size, datum)) {
            return false;
        }
        data.data.push_back(datum);
    }
    return true;
}

bool ScanDataBuilder::parse_irrelevant_data(char*& buffer, size_t& size) {

    if (size >= irrelevant_data_len && memcmp(buffer, irrelevant_data, irrelevant_data_len) == 0) {
        buffer += irrelevant_data_len;
        size -= irrelevant_data_len;
        return true;
    }
    return false;
}

//int main() {
//    char
//            packet_template[] =
//                    "%csSN LMDscandata 1 1 A037B9 0 0 C1C3 C1C6 8B621C37 8B627603 0 0 7 0 0 1388 168 0 1 DIST1 3F800000 00000000 FFF92230 1388 21D 293 272 274 273 28D 294 2A3 2AA 29A 29B 29D 28F 28D 295 282 261 278 289 281 274 27B 27F 26A 26E 271 25E 259 267 260 261 25A 258 25D 251 258 251 254 259 259 25A 25B 25C 266 267 25E 25F 266 272 273 275 284 282 29A 2A0 2B7 2BC 2DD 2E9 310 33C 349 36E 38A 3B6 3E4 409 433 451 461 47B 48A 496 488 491 49E 4A6 49C 49A 46B 48F 4AB 4BC 499 48B 2C5 23F 290 4AF 4B0 4A9 4BC 4AB 4B8 3 3 24C 200 21D 358 3 3 3 3 3 3 3 3 3 3 3 3 3 3 3 520 51F 502 4E9 4C0 4A5 496 476 464 44A 430 41D 404 3FC 3E7 3C8 3 367 25D 24F 285 25D 264 260 39F 531 6CD 30D 2AA 372 66A 65A 64B 637 626 616 60C 605 5EF 5D6 5CF 5B4 59C 58F 58A 58F 56F 563 553 550 544 3 3 3 50E 50E 4FC 4F8 4EC 4DD 4D4 4C7 4DB 0 3 4AA 4A2 4A0 3 0 0 0 0 3 3 1537 155B 0 3 0 3 3 3 3 3 BC9 BD1 BE1 BD3 BCF BCD 3 399 38A B8D B86 B87 1B5C DD0 1CC0 DBE DCA 1CFE E2B E30 F0C 1093 10F2 11CF 10A1 1093 11C9 139A 2A26 29C0 1390 1379 2975 29F4 29A4 2799 26DA 26D2 26D2 26C1 26DF 2731 3A2C 3901 D32 D79 12CD 1888 187D 1873 187C 16A8 153D 150C 1494 1485 146A 1489 1499 149B 14BE 1615 15D8 14F8 2C38 2C37 2C40 2C4C 2C52 2C42 2C3F 1D42 1CF5 1CAA 1CB8 1CDE 1CB7 1774 1DB7 1FAB 1FE1 1FF3 1FD9 1BF9 2DB5 2922 2728 25F6 24B8 1494 1412 218C 21B5 20FA 1E5A 1E62 1DF4 1D2B 1C5E 1B9F 1B14 DA5 178B 1723 1664 15E5 BAA B7F 3 3 3 3 3 3 3 A13 9F0 A11 A2C 9F6 869 856 858 86A 879 836 7F4 830 868 863 819 791 798 77D 77B 76A 772 772 762 769 77E 7D2 7E9 7EB 7D6 7EC 7E6 7E4 7EB 7E4 7CA 7D5 7E3 7FA 8A2 8A6 82C 79B 7A2 774 760 707 6DA 6DC 6DB 6D6 65E 3 3CD 3CE 3 3 3E5 3E3 3E5 3E9 3F5 3 5C0 64D 640 646 63A 632 61C 61B 615 615 605 5F6 5F5 5EC 5E5 5F0 5E9 5E4 5D0 5D8 5C7 5C5 5C2 5B9 5BB 5BC 5BE 5BE 5B7 5A5 59D 59E 58D 59D 58D 58D 581 58D 57D 587 58E 58C 584 577 579 560 56D 563 55E 55D 570 56E 56D 571 55C 552 563 560 561 55A 577 558 55C 567 571 573 55D 563 556 565 571 563 560 569 56B 576 56C 57A 578 58A 57B 564 5B5 88E 8D2 8CC 8D2 665 5AA 62E 76B 877 86E 859 85A 85A 837 82F 812 7F6 7E0 7DE 7E9 7E6 7E8 7EC 7F2 7D8 812 7F5 7F1 7F9 7FE 80F 850 856 84E 83C 861 855 877 878 88A 87D 87C 880 8A4 8B5 8C1 8CE 8B7 8C5 8A2 8A8 8ED 8C0 89E 8BE 8C4 8C5 8D0 8EF 3 3 3 3 3 0 0 0 0 0 6D0 68B 689 67C 636 5F0 5D4 5D7 3 5DB 3 3 0 0 0 0 0 0%c";
//
//    char packet[sizeof(packet_template)];
//    sprintf(packet, packet_template,lms100_cola::STX_BYTE, lms100_cola::ETX_BYTE);
//    size_t expected_amount_of_data = 541;
//
//    ScanDataBuilder builder(expected_amount_of_data);
//    char* temp_packet = packet;
//    size_t size = sizeof(packet);
//    std::cout << "size = " << size << std::endl;
//    bool ready = builder.Parse(temp_packet, size);
//    std::cout << "ready = " << ready << std::endl;
//    ScanData data = builder.GetData();
//    if (ready) {
//        std::cout << "scan frequency: " << data.scan_frequency << std::endl;
//        std::cout << "measurement frequency: " << data.measurement_frequency
//                << std::endl;
//        std::cout << "scale factor: " << data.scale_factor << std::endl;
//        std::cout << "scale factor offset: " << data.scale_factor_offset
//                << std::endl;
//        std::cout << "start angle: " << data.start_angle << std::endl;
//        std::cout << "steps: " << data.steps << std::endl;
//        std::cout << "amount of data: " << data.amount_of_data << std::endl;
//    } else {
//        std::cout << "error parsing data" << std::endl <<
//                "method index: " << builder.method_index <<std::endl;
//        std::cout << "src: " << temp_packet << std::endl;
//    }
//
////    char src_data[] = "F F 3dcccccd ";
////
////    char* buffer = src_data;
////    size_t remaining = strlen(buffer);
////
////    ScanDataBuilder builder;
////
////    uint8_t uint = 0;
////    int8_t sint = 0;
////    float f = 0;
////
////    builder.get_next<uint8_t>(buffer, remaining, uint);
////    builder.get_next<int8_t>(buffer, remaining, sint);
////    builder.get_next<float>(buffer, remaining, f);
////
////    printf("remaining is 0 -- %s\n", remaining == 0 ? "true" : "false");
////    printf("buffer ptr is '\\0' -- %s\n", *buffer == '\0' ? "true" : "false");
////    printf("uint = %u\n", uint);
////    printf("sint = %i\n", sint);
////    printf("f = %f\n", f);
//
//
//    return 0;
//}
