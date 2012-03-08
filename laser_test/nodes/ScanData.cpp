/*
 * ScanData.cpp
 *
 *  Created on: 28 Feb 2012
 *      Author: Jack Hargreaves
 */

#include "ScanData.h"

ScanData::ScanData() {
}

ScanData::ScanData(size_t expected_data_count) {
    data.reserve(expected_data_count);
}

ScanData::~ScanData() {
}
