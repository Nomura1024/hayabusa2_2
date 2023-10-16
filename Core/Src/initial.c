/*
 * initial.c
 *
 *  Created on: 2021/11/02
 *      Author: Owner
 */
#include "initial.h"
#include"flash.h"
#include "main.h"
#include <AQM0802.h>
#include <infor.h>
#include "stdio.h"
#include "math.h"

TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;
TIM_HandleTypeDef htim8;
TIM_HandleTypeDef htim6;
TIM_HandleTypeDef htim4;
ADC_HandleTypeDef hadc1;
DMA_HandleTypeDef hdma_adc1;

uint16_t analog[SENSOR_NUMBER];
int16_t Average_speed;
uint8_t maker_right;
uint8_t maker_left;
uint8_t maker_flag=0;
uint8_t cross_flag=0;
float tread = 100;
void MX_TIM6_Init(void);
void init(){
	if(HAL_ADC_Start_DMA(&hadc1, (uint32_t *) analog, SENSOR_NUMBER) != HAL_OK){
	  Error_Handler();
	}

	if (HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_2) != HAL_OK){
		  Error_Handler();
	}
	if (HAL_TIM_PWM_Start(&htim8, TIM_CHANNEL_3) != HAL_OK){
	      Error_Handler();
	}

	if (HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_3) != HAL_OK){
		  Error_Handler();
	}
	if (HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_4) != HAL_OK){
		  Error_Handler();
	}
	lcd_init();
	lcd_clear();
	lcd_locate(0,0);
	lcd_printf("OK");

	HAL_TIM_Encoder_Start(&htim4,TIM_CHANNEL_ALL);
	HAL_TIM_Encoder_Start(&htim3,TIM_CHANNEL_ALL);
	HAL_TIM_PWM_Start(&htim1,TIM_CHANNEL_ALL);

	HAL_TIM_PWM_Start(&htim8,TIM_CHANNEL_ALL);
	HAL_TIM_PWM_Start(&htim2,TIM_CHANNEL_ALL);

	Flash_load();
	for(int z=0;z<SENSOR_NUMBER;z++){
		ADC_dif[z] = work_ram[z];
		ADC_Small[z]=work_ram[z+SENSOR_NUMBER];
	}
	flashcheck();
	Average_speed=work_ram[26];
	Kp = work_ram[27];
	Ki = work_ram[28];
	Kd = work_ram[29];

	ang_average = work_ram[30]/1000;
	if(ang_average>=50) ang_average = (ang_average*-1)/100;
	Accm = work_ram[31];
	Decm = work_ram[32];

	TIM4 -> CNT = 32768;
	TIM3 -> CNT = 32768;

}
void LED(uint8_t x){
	switch(x){
		case 1:
			HAL_GPIO_WritePin(GPIOB, GPIO_PIN_14, GPIO_PIN_RESET);
			HAL_GPIO_WritePin(GPIOB, GPIO_PIN_13, GPIO_PIN_SET);
			HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12, GPIO_PIN_SET);
			break;
		case 2:
			HAL_GPIO_WritePin(GPIOB, GPIO_PIN_14, GPIO_PIN_SET);
			HAL_GPIO_WritePin(GPIOB, GPIO_PIN_13, GPIO_PIN_RESET);
			HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12, GPIO_PIN_SET);
			break;
		case 3:
			HAL_GPIO_WritePin(GPIOB, GPIO_PIN_14, GPIO_PIN_RESET);
			HAL_GPIO_WritePin(GPIOB, GPIO_PIN_13, GPIO_PIN_RESET);
			HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12, GPIO_PIN_SET);
			break;
		case 4:
			HAL_GPIO_WritePin(GPIOB, GPIO_PIN_14, GPIO_PIN_SET);
			HAL_GPIO_WritePin(GPIOB, GPIO_PIN_13, GPIO_PIN_SET);
			HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12, GPIO_PIN_RESET);
			break;
		case 5:
			HAL_GPIO_WritePin(GPIOB, GPIO_PIN_14, GPIO_PIN_RESET);
			HAL_GPIO_WritePin(GPIOB, GPIO_PIN_13, GPIO_PIN_SET);
			HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12, GPIO_PIN_RESET);
			break;
		case 6:
			HAL_GPIO_WritePin(GPIOB, GPIO_PIN_14, GPIO_PIN_SET);
			HAL_GPIO_WritePin(GPIOB, GPIO_PIN_13, GPIO_PIN_RESET);
			HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12, GPIO_PIN_RESET);
			break;
		case 7:
			HAL_GPIO_WritePin(GPIOB, GPIO_PIN_14, GPIO_PIN_RESET);
			HAL_GPIO_WritePin(GPIOB, GPIO_PIN_13, GPIO_PIN_RESET);
			HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12, GPIO_PIN_RESET);
			break;
		case 8:
			HAL_GPIO_WritePin(GPIOB, GPIO_PIN_14, GPIO_PIN_SET);
			HAL_GPIO_WritePin(GPIOB, GPIO_PIN_13, GPIO_PIN_SET);
			HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12, GPIO_PIN_SET);
			break;

	}

}
void LED2(uint8_t x){
	switch(x){
		case 1:
			HAL_GPIO_WritePin(GPIOB, GPIO_PIN_5, GPIO_PIN_RESET);
			HAL_GPIO_WritePin(GPIOA, GPIO_PIN_15, GPIO_PIN_SET);
			HAL_GPIO_WritePin(GPIOB, GPIO_PIN_4, GPIO_PIN_SET);
			break;
		case 2:
			HAL_GPIO_WritePin(GPIOB, GPIO_PIN_5, GPIO_PIN_SET);
			HAL_GPIO_WritePin(GPIOA, GPIO_PIN_15, GPIO_PIN_RESET);
			HAL_GPIO_WritePin(GPIOB, GPIO_PIN_4, GPIO_PIN_SET);
			break;
		case 3:
			HAL_GPIO_WritePin(GPIOB, GPIO_PIN_5, GPIO_PIN_RESET);
			HAL_GPIO_WritePin(GPIOA, GPIO_PIN_15, GPIO_PIN_RESET);
			HAL_GPIO_WritePin(GPIOB, GPIO_PIN_4, GPIO_PIN_SET);
			break;
		case 4:
			HAL_GPIO_WritePin(GPIOB, GPIO_PIN_5, GPIO_PIN_SET);
			HAL_GPIO_WritePin(GPIOA, GPIO_PIN_15, GPIO_PIN_SET);
			HAL_GPIO_WritePin(GPIOB, GPIO_PIN_4, GPIO_PIN_RESET);
			break;
		case 5:
			HAL_GPIO_WritePin(GPIOB, GPIO_PIN_5, GPIO_PIN_RESET);
			HAL_GPIO_WritePin(GPIOA, GPIO_PIN_15, GPIO_PIN_SET);
			HAL_GPIO_WritePin(GPIOB, GPIO_PIN_4, GPIO_PIN_RESET);
			break;
		case 6:
			HAL_GPIO_WritePin(GPIOB, GPIO_PIN_5, GPIO_PIN_SET);
			HAL_GPIO_WritePin(GPIOA, GPIO_PIN_15, GPIO_PIN_RESET);
			HAL_GPIO_WritePin(GPIOB, GPIO_PIN_4, GPIO_PIN_RESET);
			break;
		case 7:
			HAL_GPIO_WritePin(GPIOB, GPIO_PIN_5, GPIO_PIN_RESET);
			HAL_GPIO_WritePin(GPIOA, GPIO_PIN_15, GPIO_PIN_RESET);
			HAL_GPIO_WritePin(GPIOB, GPIO_PIN_4, GPIO_PIN_RESET);
			break;
		case 8:
			HAL_GPIO_WritePin(GPIOB, GPIO_PIN_5, GPIO_PIN_SET);
			HAL_GPIO_WritePin(GPIOA, GPIO_PIN_15, GPIO_PIN_SET);
			HAL_GPIO_WritePin(GPIOB, GPIO_PIN_4, GPIO_PIN_SET);
			break;

	}

}
void error(){
//	__HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_4, ESC_MIN);
//	__HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, ESC_MIN);
	__HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_3, 0);
	__HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_4, 0);
	HAL_TIM_Base_Stop_IT(&htim6);

	while(1){
		Motor(0,0);
		LED(1);
	}
}
void stop(){
	Average_speed = 1000;
	while(1){
		Average_speed = Average_speed-200;
		//if(driv_flag=1)Average_speed = Average_speed-100;
		HAL_Delay(50);
		if(Average_speed<10)break;
	}
	//HAL_Delay(10);
	Average_speed = 0;
	__HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_3, 0);
	__HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_4, 0);
//	__HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_4, ESC_MIN);
//	__HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, ESC_MIN);

	HAL_Delay(700);
	HAL_TIM_Base_Stop_IT(&htim6);
	Motor(0,0);
	LED(6);


}
void Motor(int16_t MotorL,int16_t MotorR)
{
//	if(MotorL >= 0 ){
//		if (MotorL >= 2000) MotorL = 2000;
//		__HAL_TIM_SET_COMPARE(&htim8, TIM_CHANNEL_3, MotorL);
//		__HAL_TIM_SET_COMPARE(&htim8, TIM_CHANNEL_4, 0);
//	}else if(MotorL <= 0){
//		MotorL = -MotorL;
//		if (MotorL >= 2000) MotorL = 2000;
//		__HAL_TIM_SET_COMPARE(&htim8, TIM_CHANNEL_3, 0);
//		__HAL_TIM_SET_COMPARE(&htim8, TIM_CHANNEL_4, MotorL);
//	}
//
//	if(MotorR >= 0 ){
//		if (MotorR >= 2000) MotorR = 2000;
//		__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, 0);
//		__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, MotorR);
//	}else if(MotorR <= 0){
//		MotorR = -MotorR;
//		if (MotorR >= 2000) MotorR = 2000;
//		__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, MotorR);
//		__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, 0);
//
//	}
	if(MotorL > 0 ){
		HAL_GPIO_WritePin(GPIOC, GPIO_PIN_9, GPIO_PIN_SET);


	}else if(MotorL < 0){
		HAL_GPIO_WritePin(GPIOC, GPIO_PIN_9, GPIO_PIN_RESET);
		MotorL = -MotorL;
	}
	if(MotorR > 0 ){
		HAL_GPIO_WritePin(GPIOA, GPIO_PIN_8, GPIO_PIN_RESET);

	}else if(MotorR < 0){
		HAL_GPIO_WritePin(GPIOA, GPIO_PIN_8, GPIO_PIN_SET);
		MotorR = -MotorR;

	}
	if (MotorR > 2000) MotorR = 2000;
	if (MotorL > 2000) MotorL = 2000;

	__HAL_TIM_SET_COMPARE(&htim8, TIM_CHANNEL_3, MotorL);
	__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, MotorR);


}
void sidemaker(){
if(HAL_GPIO_ReadPin(GPIOC,GPIO_PIN_15) ==0) maker_right = true;
	else maker_right = false;

	if(HAL_GPIO_ReadPin(GPIOB,GPIO_PIN_2) ==0 ) maker_left = true;
	else maker_left = false;

	if(log_flag){
	  if(cross_line) cross_flag++;
	  if(cross_flag>=11) {
		  cross_line=0;
		  cross_flag=0;
	  }
		  log_flag= false;
	  }
}
void fan_pressure(float L,float R){
	int fanL=0;
	int fanR=0;
	if(L>120)L=150;
	if(R>120)R=150;

	__HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_3, L);
	__HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_4, R);
}
void driv_fan(float speed_now){
	float fan=0;
	float fan_Lim=0;
	static float speed_buff=0;

//	if (speed_now >= speed_buff)speed_now=3800;
//	speed_buff = speed_now;

	if(speed_now<=3800)speed_now=2010;
	if(speed_now>=5000)speed_now=5000;

	fan = (100*(speed_now-2000))/3000;
	if(fan>20)fan=20;

	fan_Lim = (19*(100-fan))/100;

	fan_pressure(round(fan_Lim),round(fan_Lim));
}
void flashcheck(){
	if(work_ram[28] != 0){
		  work_ram[26]=1800;
		  work_ram[27]=200;
		  work_ram[28]=0;
		  work_ram[29]=42;
		  work_ram[31]=17;
		  work_ram[32]=4;
		  work_ram[33] = 2000;
		  work_ram[34] = 3000;
		  work_ram[35] = 3500;
		  work_ram[36] = 4500;
		  work_ram[37] = 4500;
		  work_ram[38] = 7000;
		  Flash_store();
	}
}
uint8_t switch_cheack(){
	return HAL_GPIO_ReadPin(GPIOC,GPIO_PIN_13);

}
uint8_t switch_cheack2(){
	return HAL_GPIO_ReadPin(GPIOB,GPIO_PIN_15);

}
int mode_Selection(uint8_t cheak){
	int8_t i=1;
	TIM1 -> CNT = 32768;
	LED(1);
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_12, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_11, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_10, GPIO_PIN_RESET);
	while (switch_cheack()){

		if(TIM4 -> CNT>32768+2000){
			i++;
			if(i>=8)i=1;
			LED(i);
			TIM4 -> CNT=32768;
		}
		if(TIM4 -> CNT<32768-2000){
			i--;
			if(i<=0)i=7;
			LED(i);
			TIM4 -> CNT=32768;
		}
	}
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_12, GPIO_PIN_SET);
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_11, GPIO_PIN_SET);
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_10, GPIO_PIN_SET);
	return i;
}
