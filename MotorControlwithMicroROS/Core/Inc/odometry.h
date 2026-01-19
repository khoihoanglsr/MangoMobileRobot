/*
 * odometry.h
 *
 *  Created on: Dec 2, 2025
 *      Author: khoi2
 */

#ifndef INC_ODOMETRY_H_
#define INC_ODOMETRY_H_

#include <stdint.h>

// Pose robot
extern volatile float odom_x;
extern volatile float odom_y;
extern volatile float odom_theta;

// Khởi tạo hệ odometry
void Odometry_Init(void);

// Gọi hàm này mỗi chu kỳ (vd: 10ms)
void Odometry_Update(void);

#endif /* INC_ODOMETRY_H_ */
