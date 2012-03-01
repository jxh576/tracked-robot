/*
 * player_laser_config.h
 *
 *  Created on: 24 Feb 2012
 */

#ifndef PLAYER_LASER_CONFIG_H_
#define PLAYER_LASER_CONFIG_H_


typedef struct player_laser_config
{
  /** Start and end angles for the laser scan [rad].*/
  float min_angle;
  /** Start and end angles for the laser scan [rad].*/
  float max_angle;
  /** Scan resolution [rad].  */
  float resolution;
  /** Maximum range [m] */
  float max_range;
  /** Range Resolution [m] */
  float range_res;
  /** Enable reflection intensity data. */
  uint8_t  intensity;
  /** Scanning frequency [Hz] */
  float scanning_frequency;
} player_laser_config_t;

#endif /* PLAYER_LASER_CONFIG_H_ */
