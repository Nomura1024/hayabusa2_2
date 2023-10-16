/*
 * sensor_ADC.c
 *
 *  Created on: 2022/03/16
 *      Author: Owner
 */
#include "main.h"
#include "initial.h"
#include <sensor_ADC.h>
#include <AQM0802.h>
#include <flash.h>
#include "stdio.h"
#include "string.h"
#include "initial.h"
#include "infor.h"


uint16_t ADC_Small[SENSOR_NUMBER];
uint16_t ADC_dif[SENSOR_NUMBER];
uint8_t cross_line=0;

extern uint16_t work_ram[BACKUP_FLASH_SECTOR_SIZE] __attribute__ ((aligned(4)));
void ADCinit(){
	Flash_load();
	lcd_printf("ADCinit");
	HAL_Delay(100);
	uint16_t ADC_Max[SENSOR_NUMBER]={0};
	uint16_t i;

	for(int z=0;z<SENSOR_NUMBER;z++){
		ADC_Small[z]=10000;
	}
	LED2(1);

	i = 0;
	while (switch_cheack2())
	{
		if(analog[i] > ADC_Max[i]){
			ADC_Max[i] = analog[i];
		}
		if(analog[i] < ADC_Small[i]){
			ADC_Small[i] = analog[i];
		}
		i++;
		if(i == SENSOR_NUMBER){
			i=0;
		}
		LED(2);
		LED2(2);

	}
	for(int z=0;z<SENSOR_NUMBER;z++){
		ADC_dif[z] = ADC_Max[z]-ADC_Small[z];
		work_ram[z] = ADC_dif[z];
		work_ram[z+SENSOR_NUMBER] = ADC_Small[z];
	}
	Flash_store();
	for(int z=0;z<SENSOR_NUMBER;z++){
		printf("%d,%d\r\n",z,ADC_Small[z]);
	}
//	for(int z=0;z<SENSOR_NUMBER;z++){
//		printf("%d,%d\r\n",z,ADC_Max[z]);
//	}
	LED2(3);

}
inline float senseGet(){
	float sensL=0;
	float sensR=0;
	float sensval=0;
	uint32_t sens[SENSOR_NUMBER];
	float sensRatio[SENSOR_NUMBER];
	static float sens_buff[SENSOR_NUMBER];
	static uint8_t error_count=0;
	uint8_t black= 0;

//	for(int i=0; i<SENSOR_NUMBER; i++){
//		sens[i] = analog[i];
//		if(sens[i] >=  (ADC_Small[i]+ADC_dif[i])){
//			sens[i] =  (ADC_Small[i]+ADC_dif[i]);
//		}
//		if(sens[i] <= ADC_Small[i]){
//			sens[i] = ADC_Small[i] ;
//		}
//	}
	sensval=0;
	for(int i=0; i<SENSOR_NUMBER; i++){
		sens[i] = analog[i];
		if(sens[i] >=  (ADC_Small[i]+ADC_dif[i])){
			//ADC_dif[i] = sens[i]-ADC_Small[i];
			sens[i] =  (ADC_Small[i]+ADC_dif[i]);
		}
		if(sens[i] <= ADC_Small[i]){
			sens[i] = ADC_Small[i] ;
		}
		sensRatio[i] = (1000.0f/(float)ADC_dif[i])*((float)(sens[i]-(float)ADC_Small[i]));
//		sensRatio[i] = (sensRatio_buff[i]+sensRatio[i])/2;
//		sensRatio_buff[i]=sensRatio[i];
		sensval = sensval+sensRatio[i];
		if(sensRatio[i] >= 600) black++;
//		if(i<=5)sensL += sensRatio[i];
//		if(i>=7)sensR += sensRatio[i];
	}
	sensL = sensRatio[5]+sensRatio[4]*1.4+sensRatio[3]*1.8+sensRatio[2]*2.2+sensRatio[1]*2.6+sensRatio[0]*3.0;
	sensR = sensRatio[7]+sensRatio[8]*1.4+sensRatio[9]*1.8+sensRatio[10]*2.2+sensRatio[11]*2.6+sensRatio[12]*3.0;

//	for(int i=0;i<=5;i++){
//		sensL += sensRatio[i];
//	}
//	for(int i=7;i<=12;i++){
//		sensR += sensRatio[i];
//	}
	//if ((sensL+sensR)/12<=400){
//	if ((sensRatio[0]+sensRatio[12])/2<=500 /*|| (sensRatio[1]+sensRatio[11])/2 <= 650*/) {
//	if ((sensRatio[0]<=800 && sensRatio[12]<=800) && (sensRatio[1]<=800 && sensRatio[11]<=800)) {
	if ((sensRatio[0]<=600 || sensRatio[1]<=500) && (sensRatio[11]<=500 || sensRatio[12]<=600) && sensRatio[6]<=800) {
		cross_line=true;
		cross_flag=0;
	}
//	for(int i=0; i<SENSOR_NUMBER; i++){
//		if(sensRatio[i] >= 600) black++;
//	}

	//if((sensL-sensR) >=  )
//	if ((sensRatio[6]>=900) && (sensRatio[5])>=900 && (sensRatio[7])>=900) error_count++;
	if(black >= 13){
		error_count++;
		black=0;
	}
	else {
		error_count=0;
		black=0;
	}
	//if ((sensRatio[0]<=900 && sensRatio[12]<=900)) error_count++;
	//if ((sensRatio[6])>=700) error_count++;
//	else if ((sensRatio[6])<=700)error_count=0;
	if(error_count>110)error();
	return (sensL-sensR)/sensval;
	//return sensRatio[6];

}
