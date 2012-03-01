/*
 * player_laser_data.h
 *
 *  Created on: 24 Feb 2012
 */

#ifndef PLAYER_LASER_DATA_H_
#define PLAYER_LASER_DATA_H_

typedef struct player_laser_data
{
  /** Start and end angles for the laser scan [rad].  */
  float min_angle;
  /** Start and end angles for the laser scan [rad].  */
  float max_angle;
  /** Angular resolution [rad].  */
  float resolution;
  /** Maximum range [m]. */
  float max_range;
  /** Number of range readings.  */
  uint32_t ranges_count;
  /** Range readings [m]. */
  float *ranges;
  /** Number of intensity readings */
  uint32_t intensity_count;
  /** Intensity readings. */
  uint8_t *intensity;
  /** A unique, increasing, ID for the scan */
  uint32_t id;
} player_laser_data_t;

#endif /* PLAYER_LASER_DATA_H_ */
