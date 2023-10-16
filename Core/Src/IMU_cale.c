/*
 * IMU_cale.c
 *
 *  Created on: 2021/11/17
 *      Author: Owner
 */
#include"main.h"
#include <initial.h>
#include<IMU_cale.h>
#include <flash.h>
#include "stdio.h"
#include <infor.h>
#include <ICM_20648.h>
#define BACKUP_FLASH_SECTOR_SIZE    1024*16
#define PI 3.141592
#define R_IMU 1 //0.5
#define NOMUSANNO "*"

extern  uint16_t work_ram[BACKUP_FLASH_SECTOR_SIZE] __attribute__ ((aligned(4)));
 char _backup_flash_start;
float ang_average=0;

float ang;
void off_angle(){
	float average;
	int i;
	Flash_load();
	IMU_init();
	//HAL_Delay(1500);

	for(i=0;i<=1000;i++){
		read_gyro_data();
		average = average+zg;
		HAL_Delay(1);
	}
	LED(4);
	ang_average = average/1000;
	if(average<=0) average = -average*100;
	work_ram[30]= average;
	//if(average>=50) average=-average/10;


//	printf("%f\r\n",ang_average);
//	printf("%d\r\n",work_ram[30]);
	Flash_store();
}
inline float calc_angle(){
	float omega_z=0;
	static float pre_zg;
	float R_IMU2 = 0.1;
	//float angle;
	read_gyro_data();
	omega_z = (((float)zg-ang_average) / 16.4) * PI / 180;
	//angle = angle+ (omega_z * 0.01);
	//if(driv_flag==2)omega_z= ((R_IMU2)*(omega_z) + (1.0 - (R_IMU2))* (pre_zg));
	omega_z= ((R_IMU)*(omega_z) + (1.0 - (R_IMU))* (pre_zg)); // lowpath filter
	pre_zg =  omega_z;
	//ang = ang+(omega_z*T);
	ang = omega_z;
	return omega_z;
}


