/*
 * initial.h
 *
 *  Created on: Dec 4, 2024
 *      Author: Owner
 */

#ifndef INC_INITIAL_H_
#define INC_INITIAL_H_

#include "main.h"

extern UART_HandleTypeDef hlpuart1;
extern TIM_HandleTypeDef htim2;
extern TIM_HandleTypeDef htim3;
extern float receive_data;
extern int push_servo_flog,motor_flog,angel_floag,step_angle,face_detection,caliblation;
extern int Gun_Angle;


void Motor(int16_t MotorL,int16_t MotorR);
void setServoAngle(TIM_HandleTypeDef *htim, uint32_t Channel, uint16_t angle);
void set_Stepping_Motor(int16_t angle);
void stepping_motor(int16_t direction);
void UART_Receive(void);
void data_analysis(float data);
int limit_switche_1(void);
int limit_switche_2(void);
void push_marshmallow(uint8_t flag);
void init(void);
void buzzer(int flag);
void step_Initialization();
#endif /* INC_INITIAL_H_ */
