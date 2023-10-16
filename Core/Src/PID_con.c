/*
 * PID_con.c
 *
 *  Created on: Mar 16, 2022
 *      Author: Owner
 */
#include"PID_con.h"
#include"main.h"
#include <sensor_ADC.h>
#include <initial.h>
#include <infor.h>
#include "stdio.h"
#include <Driv_log.h>
#include "math.h"
#include<IMU_cale.h>
uint16_t Kp;
uint16_t  Ki;
uint16_t  Kd ;
extern float speedget ;

//float speedval_I=0 ;
inline float Line_Motor(){
	static float sensvalBuf;
	static int sensval_I = 0;
	static float sensval_IBuf ;
	float sensval_D =0;
	float sensval;//偏差


	sensval = senseGet();//hensa

    sensval_I = sensval_I + (float)sensval*T;
    sensval_IBuf = sensval_IBuf + sensval;

    if(sensval_I >= 1000000) sensval_I = 1000000;
    if(sensval_I <= (-1000000)) sensval_I = (-1000000);

    sensval_D = (sensvalBuf - sensval)/T;
    sensvalBuf = sensval;

	//return ((sensval* (float)Kp*0.001)+(sensval_I*(float)Ki*0.0001)-(sensval_D*(float)Kd*0.0001));
	return ((sensval* 1000)+(sensval_I*(float)Ki*0.0001)-(sensval_D*(float)Kd*0.0000))*(ang*10);


}
inline float speed_get(){
	//static float speedbuffg=0;
	float speedget_L=0;
	float speedget_R=0;
	float load_L,load_R=0;
	float load=0;
	int L=0;
	int R=0;
//	static int i =0;
	L = TIM3 -> CNT -32768;
	TIM3 -> CNT = 32768;

	R = TIM4 -> CNT -32768;;
	TIM4 -> CNT = 32768;





	//val=((float)L+(float)R)/2;
//	speedget_L = (((24.1/4096)*(float)L)/T);
//	speedget_R = (((24.1/4096)*(float)R)/T);
//	speedget_L = (((24.1/4096)*(float)L)/T);
//	speedget_R = (((24.1/4096)*(float)R)/T);
//	speedget_L = (((23/4096)*(float)L)/T);
//	speedget_R = (((23/4096)*(float)R)/T);
	load_L = (25.3/2048)*(float)L;
	load_R = (25.3/2048)*(float)R;
	speedget_L = load_L/T;
	speedget_R = load_R/T;
//	if(load_L <= 0) load_L=0;
//	if(load_R <= 0) load_R=0;
	load = (load_R+load_L)/2.0f;
	//load = (((24.1/4096)*(float)L)+((24.1/4096)*(float)R))/2;
	speedget = (speedget_R+speedget_L)/2;
	//speedbuffg += speedget*T;
	if(driv_flag==1&& maker_flag==1){

		first_driv(load);
		first_cale();
	}
	if(driv_flag==2 && maker_flag==1)second_driv(load);

	return speedget;
}
inline void Speed_Motor(){
	int16_t MotorL=0,MotorR=0;

	float speedval;
	float val_v = 0;
	float val_k=0;
	float se=0;
	static int speedval_I=0 ;
	static float speed_buff=0;
	float val= 0;
	float sKp = 2.5;//1.8
	float sKi= 28;//20
	//float lKi= 20;

	speedval = (float)Average_speed - speed_get();
//	speedval = 0- speed_get();

	if(speed_buff != Average_speed && speed_buff > Average_speed && speedval_I > 0)speedval_I=0;
	if(speed_buff != Average_speed && speed_buff < Average_speed && speedval_I < 0)speedval_I=0;
	speed_buff = Average_speed;
	//if(speedval <= 10.0 && speedval >= (-10.0))speedval_I=0;
	speedval_I = speedval_I + ((float)speedval*T);

	//if(speedval <= 10.0 && speedval >= (-10.0))speedval_I=0;

	if(speedval_I >= 100000) speedval_I = 100000;
	if(speedval_I <= (-100000)) speedval_I = (-100000);

	//val_v = (speedval* sKp)+((float)speedval_I*sKi);
	val_k  = Line_Motor();
//	if(val_v >= 1400 && val_k >= 2000) val_v =1400;
//	if(val_v <= -1400 && val_k <= -2000) val_v = -1400;
	if(val_v >= 2000) val_v =2000;
	if(val_v <= -2000) val_v = -2000;
	if(val_k >= 2000){
		val_k =2000;
		//val_v = 0;
	}
	if(val_k <= -2000){
		val_k = -2000;
		//val_v = 0;
	}
//	if(val_k >= 2000) val_v = 0;
//	if(val_k <= -2000) val_v = 0;
	MotorL = roundf(val_v + val_k);
	MotorR =roundf( val_v - val_k);
	if(MotorL>=2000  ){
		se = MotorL-2000;
		MotorL = (val_v-se)+val_k;
		MotorR = (val_v-se)-val_k;
	}
	else if(MotorR>=2000){
		se = MotorR-2000;
		MotorL = (val_v-se)+val_k;
		MotorR = (val_v-se)-val_k;
	}
	else if(MotorL<=-2000 ){
		se = MotorL+2000;
		MotorL = (val_v-se)+val_k;
		MotorR = (val_v-se)-val_k;
	}
	else if(MotorR<=-2000 ){
		se = MotorR+2000;
		MotorL = (val_v-se)+val_k;
		MotorR = (val_v-se)-val_k;
	}
	if(speedval>=2000){
		MotorL=MotorL+200;
		MotorR=MotorR+200;
	}
//	if(MotorL-MotorR <= 2000) fan_pressure(14.5,15.0);
//	else fan_pressure(14.5,14.5);
//	if(MotorR-MotorL <= 2000) fan_pressure(15.0,14.5);
//	else fan_pressure(14.5,14.5);
	Motor(MotorL,MotorR);


}
