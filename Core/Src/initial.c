
/*
 * initial.c
 *
 *  Created on: Dec 4, 2024
 *      Author: Owner
 */
#include "initial.h"
#include "main.h"
#include <stdio.h>
#include <stdint.h>
#include <string.h> // Include for memcpy
#include <stdlib.h>

#define MIN_PULSE_WIDTH_TICKS 25  // 0.5msに対応 (0度)
#define MAX_PULSE_WIDTH_TICKS 125 // 2.5msに対応 (180度)
#define Stepping_Motor_step 800

float receive_data =0.135;
int push_servo_flog,motor_flog,angel_floag,step_angle,face_detection,caliblation;
int Gun_Angle =0;

//uint8_t buffer[sizeof(float)];
uint8_t buffer[4];

void Motor(int16_t MotorL,int16_t MotorR){
	  __HAL_TIM_SET_COMPARE(&htim3,TIM_CHANNEL_2,MotorL);
	  __HAL_TIM_SET_COMPARE(&htim2,TIM_CHANNEL_2,MotorR);
}
void setServoAngle(TIM_HandleTypeDef *htim, uint32_t Channel, uint16_t angle) {
    // 角度の範囲を制限 (0〜180度)
    if (angle > 180) {
        angle = 180;
    }

    // パルス幅 (ticks) を計算
    uint32_t pulseWidthTicks = MIN_PULSE_WIDTH_TICKS +
                               ((angle * (MAX_PULSE_WIDTH_TICKS - MIN_PULSE_WIDTH_TICKS)) / 180);

    // PWMデューティ比を設定
    __HAL_TIM_SET_COMPARE(htim, Channel, pulseWidthTicks);
}
void set_Stepping_Motor(int16_t angle){

	if(angle < 10) angle = 10;
	if(angle > 175 ) angle = 175;


	int deviation = Gun_Angle-angle;

	if(deviation ==0)return;

	if (deviation < 0 ) {
		deviation = deviation*-1;
		HAL_GPIO_WritePin(GPIOA, GPIO_PIN_6, GPIO_PIN_SET);
	}
	else HAL_GPIO_WritePin(GPIOA, GPIO_PIN_6, GPIO_PIN_RESET);

	int step= ((float)Stepping_Motor_step/360)*deviation;
	//int step= 800;
	 HAL_GPIO_WritePin(GPIOA, GPIO_PIN_7, GPIO_PIN_SET);



	for(int i=0;i<step;i++){
		HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_SET);
		HAL_Delay(3);
		HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_RESET);
		HAL_Delay(3);
//		if(limit_switche_1()){
//			Gun_Angle=0;
//			break;
//		}
//		if(limit_switche_2()){
//			Gun_Angle=180;
//			break;
//		}
	}
	Gun_Angle = angle;

	angel_floag=0;
	 //HAL_GPIO_WritePin(GPIOA, GPIO_PIN_7, GPIO_PIN_RESET);
}
void UART_Receive(void){
    // バッファを初期化
    memset(buffer, 0, sizeof(buffer));

    // UART受信
    if (HAL_UART_Receive(&hlpuart1, buffer, sizeof(buffer), 800) == HAL_OK) {

        memcpy(&receive_data, buffer, sizeof(float));


        printf("%f", receive_data);

        data_analysis(receive_data);
    } else {
        printf("-1");
    }
}
void data_analysis(float data){
	//data= data+0.135;
	int num = data;
	 int digits[10];
	 int count = 0;

	while (num > 0) {
		digits[count] = num % 10; // 最下位桁を取得
		num /= 10;               // 次の桁に移動
		count++;
	}

	step_angle = (data-(int)data)*1000;
	angel_floag = digits[0];
	motor_flog = digits[1];
	push_servo_flog = digits[2];
	face_detection = digits[3];
	caliblation = digits[4];

}
int limit_switche_1(void){
	return (HAL_GPIO_ReadPin(GPIOB,GPIO_PIN_0)==0);
}
int limit_switche_2(void){
	return (HAL_GPIO_ReadPin(GPIOC,GPIO_PIN_1)==0);
}
void push_marshmallow(uint8_t flag){
	if(flag)setServoAngle(&htim3, TIM_CHANNEL_1,50);
	else setServoAngle(&htim3, TIM_CHANNEL_1,120);
}
void init(void){
	while(1){
		if(limit_switche_1()){
			Gun_Angle=0;
			break;
		}
		if(limit_switche_2()){
			Gun_Angle=180;
			break;
		}

	}

}
void buzzer(int flag){
	if(flag)HAL_GPIO_WritePin(GPIOA, GPIO_PIN_0, GPIO_PIN_SET);
	else HAL_GPIO_WritePin(GPIOA, GPIO_PIN_0, GPIO_PIN_RESET);
}
void step_Initialization(){
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_6, GPIO_PIN_SET);
	while(1){
		HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_SET);
		HAL_Delay(3);
		HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_RESET);
		HAL_Delay(3);
		if(limit_switche_2()){
			Gun_Angle=180;
			caliblation=0;
			set_Stepping_Motor(90);
			return;
			break;

		}
	}
}
