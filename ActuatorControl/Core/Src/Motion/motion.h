#ifndef MOTION_H
#define MOTION_H

/**
 * @brief Initialize motion path using default values
 */
void motion_init(void);

/**
 * @brief Return joint angles at the time t
 * @param t time in seconds
 * @param output joint angles values
 */
void motion_get(float t, float output[3]);

/**
 * @brief Set custom step high
 * @param h step high
 */
void motion_set_h(float h);

/**
 * @brief Set custom step high offset (minimum high)
 * @param h offset
 */
void motion_set_hoffset(float h);

/**
 * @brief Set custom distance from body
 * @param d distance
 */
void motion_set_d(float d);

#endif
