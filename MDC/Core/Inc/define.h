
/*
 * define.h
 *
 *  Created on: Aug 7, 2024
 *      Author: ryuto
 */

#ifndef INC_DEFINE_H_
#define INC_DEFINE_H_

//制御周波数
const int RATE = 50;
const float DT = 1/((float)RATE);
//エンコーダーの分解能
const int RESOLUTION = 4096;
//エンコーダーが1回転するときにタイヤは何回転するか
const float ENC_TO_TIRE = 10.0f/28.0f;

//duty比100%となるduty_cnt
const uint32_t MAX_DUTY_CNT = 7999;

typedef struct {
	float kp;
	float ki;

	float integral_l;
	float integral_r;
}PIController;

#endif /* INC_DEFINE_H_ */
