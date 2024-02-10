/*
 * Driv_log.c
 *
 *  Created on: 2022/03/28
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
#include"PID_con.h"
#include "Flash_F405.h"
#include<IMU_cale.h>
#include <Driv_log.h>
#include "math.h"
#include "stdlib.h"
uint32_t callog_adress;
uint32_t loadlog_adress;
uint32_t plan_velo_adress;
uint32_t actual__velo_adress;
uint32_t actual__callog_adress;
uint32_t side_adress;
float load_log=0;
uint16_t co=0;
uint8_t log_flag=0;
uint8_t log2_flag=0;
uint8_t straight_flag=0;
float secondsp[6100] = {0};
float second_load[6100] = {0};
uint8_t correc_maker = 0;
uint32_t log_count=0;
uint32_t log_count_buff;
int count_num=0;
uint8_t second_soeed=0;
uint16_t correction[200]= {0};
uint8_t cross_maker=0;
extern float test;
void log_init (){
	actual__callog_adress = start_adress_sector6;
	callog_adress = start_adress_sector7;
	actual__velo_adress = start_adress_sector8;
	loadlog_adress = start_adress_sector9;
	side_adress = start_adress_sector10;
	plan_velo_adress=start_adress_sector11;
	log_count=0;
}
void speed_sloow(int a,int b,float speed){
	for(int i=a;i<=b;i++){
		secondsp[i] = speed;
	}
}
void log_write(float u){
	  FLASH_Write_Word_F(callog_adress,u);
	  FLASH_Write_Word_F(loadlog_adress,load_log);
	  callog_adress+= 0x04;
	  loadlog_adress+= 0x04;
}
inline void first_cale(){
	static float cale_val=0;
	static float cale_buff=0;
	static uint16_t num=0;
	float cale=0;
	static int frp=0;
	static int frap=0;
	static int d=39;
	static uint16_t coi=0;
	num++;
	cale_val += calc_angle()*T;

	if(log2_flag) {
		coi++;
		cale = cale_val;

		log_write(cale_val);
		cale_val=0;
		if((cale_buff-cale)<=0.05 && (cale_buff-cale)>=-0.05 ) frp++;
		cale_buff = cale;
		if(frp >= 50 || cross_line){
			HAL_GPIO_WritePin(GPIOA, GPIO_PIN_12, GPIO_PIN_RESET);
			HAL_GPIO_WritePin(GPIOA, GPIO_PIN_11, GPIO_PIN_RESET);
			HAL_GPIO_WritePin(GPIOA, GPIO_PIN_10, GPIO_PIN_RESET);
			straight_flag = true;
			frp=30;
			frap =0;
		}
//		if(cale>=0.05 && cale<=-0.05 ) frap++;
//		if(frap >= 20){
//			straight_flag = false;
//			frap = 0;
//			frp=0;
//		}

		log2_flag=0;
		//cale_val=0;
		num=0;
		co++;
	}

//	if(straight_flag==1  && correc_maker==1){
	//if(coi>=30)HAL_GPIO_WritePin(GPIOC, GPIO_PIN_8, GPIO_PIN_SET);
	if(correc_maker==1 && straight_flag==1 ){
//	if(correc_maker==1){
		FLASH_Write_Word_S(side_adress,log_count_buff);
		HAL_GPIO_WritePin(GPIOA, GPIO_PIN_12, GPIO_PIN_SET);
		HAL_GPIO_WritePin(GPIOA, GPIO_PIN_11, GPIO_PIN_SET);
		HAL_GPIO_WritePin(GPIOA, GPIO_PIN_10, GPIO_PIN_SET);
		side_adress+= 0x04;
		straight_flag=false;
		correc_maker = false;
		coi=0;
		frp=0;
	}
	if(correc_maker==1){
		straight_flag = false;
		frp= 0;
		//coi=0;
	}
	if(cross_maker){
		HAL_GPIO_WritePin(GPIOA, GPIO_PIN_12, GPIO_PIN_SET);
		HAL_GPIO_WritePin(GPIOA, GPIO_PIN_11, GPIO_PIN_SET);
		HAL_GPIO_WritePin(GPIOA, GPIO_PIN_10, GPIO_PIN_SET);
		work_ram[d] = log_count_buff;
		cross_maker = false;
		d++;
	}


}
inline void first_driv(float log){
	static float logbuff=0;
	static float buff=0;
	//	logbuff = logbuff + log;

		logbuff += log;
		load_log =logbuff;
	//	if(logbuff>=10.000){
	//		log_flag = 1;
	//		log2_flag=1;
	//		load_log += logbuff-buff;
	//		logbuff = logbuff-10.0;
	//		buff = logbuff;
	//		log_count++;
	//
	//	}
		if((logbuff-buff)>=10.000){
			log_flag = 1;
			log2_flag=1;

			buff = logbuff;

			//logbuff = 0;
			//buff = logbuff;
			log_count++;

		}
}
int logsecond_Speed(double h){
	int spee=2500;
	float ratio=0;
	if(h<0)h=-h;
	switch(second_soeed){
	//h = Driving_log[6100+c]/(Driving_log[c]*0.01);


//	if(h < 100) spee = work_ram[37];
//	else if(h < 300)  spee = work_ram[36];
//	else if(h < 500)  spee = work_ram[35];
//	else if(h < 800)  spee = work_ram[34];
//	else if(h < 1000) spee = work_ram[33];
//	else  spee = work_ram[32];
//	case 0://bluee
//		h=h-100;
//		if(h<100) h=0;
//		if(h>1000) h=1000;
//		spee= (5500*h)/1000;
//		spee=spee+2500;
//		if(h < 100) spee = 1700;
//		else if(h < 300)  spee = 2000;
//		else if(h < 500)  spee = 2500;
//		else if(h < 800)  spee = 3500;
//		else if(h < 1000) spee = 4000;
//		else spee = 5500;
	//	break;
	case 1://blue
//		if(h < 100) spee = 1800;
//		else if(h < 300)  spee = 2400;
//		else if(h < 500)  spee = 2700;
//		else if(h < 800)  spee = 3800;
//		else if(h < 1000) spee = 4000;
//		else spee = 6000;
		h=h-100;
		if(h<100) h=0;
		if(h>1000) h=1000;
		spee= (6500*h)/1000;
		spee=spee+2800;

		break;
	case 2://green
		h=h-100;
		if(h<100) h=0;
		if(h>1000) h=1000;
		spee= (6800*h)/1000;
		spee=spee+2800;
//		if(h < 100) spee = 2000;
//		else if(h < 300)  spee = 2400;
//		else if(h < 500)  spee = 2700;
//		else if(h < 800)  spee = 3800;
//		else if(h < 1000) spee = 4000;
//		else spee = 6000;
		break;

	case 3://light blue
		h=h-100;
		if(h<100) h=0;
		if(h>1000) h=1000;
		spee= (7000*h)/1000;
		spee=spee+2800;

		break;

	case 4://red
		h=h-100;
		if(h<100) h=0;
		if(h>1000) h=1000;
		spee= (6500*h)/1000;
		spee=spee+3000;
		break;
	case 5://pink
		h=h-100;
		if(h<100) h=0;
		if(h>1000) h=1000;
		spee= (7000*h)/1000;
		spee=spee+3000;
		break;
	case 6://yellow
		h=h-100;
		if(h<100) h=0;
		if(h>1000) h=1000;
		spee= (7000*h)/1000;
		spee=spee+3500;
		break;
	case 7://white
		h=h-100;
		if(h<100) h=0;
		if(h>1000) h=1000;
		spee= (7000*h)/1000;
		spee=spee+4000;
		break;
	}

//	if(h < 100) spee = 1400;
//		else if(h < 300)  spee = 1600;
//		else if(h < 500)  spee = 1800;
//		else if(h < 800)  spee = 2100;
//		else if(h < 1000) spee = 4100;
//		else if(1000 < h)spee = 4600;
	return spee;

}
inline void log_Cal(){
	uint i=0;
	uint d=0;
	uint l=0;
	uint o=0;
	uint h=0;
	uint32_t callog;
	float Ca,Lo;
	float Lo_buff=0;
	float sd=0;
	float sd_buff=0;
	float sd_cou=0;
	float sd_slo=0;
	int cao=0;
	float fu=0;
	float buff=10000;


	o=39;
//	callog_adress = start_adress_sector7;
//	while(1){
//		Ca=*(float*)callog_adress;
//		if(isnan(Ca) != 0)break;
//		if(work_ram[d]==0)break;
//		l++;
//		if(l==work_ram[d]){
//			callog = callog_adress;
//			for(int j=0; j<=10;j++){
//				FLASH_Write_Word_F(callog,0.001);
//				callog -= 0x04;
//			}
//			d++;
//		}
//		callog_adress+= 0x04;
//	}

	callog_adress = start_adress_sector7;
	loadlog_adress = start_adress_sector9;
	while(1){
		Lo=*(float*)loadlog_adress-Lo_buff;
		Ca=*(float*)callog_adress;

		if(isnan(Ca) != 0)break;
		Lo_buff += Lo;
		if(i==work_ram[o]){
			for(int j=i; j>=i-10;j--){
				secondsp[j]=logsecond_Speed(1500);
			}
			o++;
//		}else secondsp[i]= logsecond_Speed(Lo/(Ca*(Lo/(double)work_ram[26])));
		}else secondsp[i]= logsecond_Speed(Lo/Ca);
//		loada[i] = loada[i-1]+Lo;
		second_load[i] = Lo;
		callog_adress+= 0x04;
		loadlog_adress+= 0x04;
		LED(2);
		i++;

	}

	for(int s=i-1;s>=0;s--){
		if(buff < secondsp[s]){
			if(buff>=2500 && secondsp[s]>=5000){
				for( h=0 ;h<=5;h++){
					if(secondsp[s-h] < buff)break;
					secondsp[s-h] = buff;

				}
			}else if(buff>=2500){
				for( h=0 ;h<=5;h++){
					if(secondsp[s-h] < buff)break;
					secondsp[s-h] = buff;

				}
			}else {//if(buff<=3000 && secondsp[s]>=5000){
				for( h=0 ;h<=5;h++){
					if(secondsp[s-h] < buff)break;
					secondsp[s-h] = buff;

				}
			}
			//else{
//				for( h=0 ;h<=20;h++){
//					if(secondsp[s-h] < buff)break;
//					secondsp[s-h] = buff;
//
//				}
//			}
			s=s-h;
		}
		buff = secondsp[s];
	}


	for(int s=i-2;s>=0;s--){
		Lo=second_load[s];
	//	Lo_buff -= Lo;
//		printf("%lf\r\n",Lo);
//		HAL_Delay(10);
		if((secondsp[s-1] - secondsp[s])/(Lo/(double)work_ram[26])>Decm)secondsp[s-1]=Decm*Lo+secondsp[s];
	}

	for(int s=1;s<=i;s++){
		Lo=second_load[s];
		//Lo_buff += Lo;
		if((secondsp[s] - secondsp[s-1])/(Lo/(double)work_ram[26])>Accm)secondsp[s]=Accm*Lo+secondsp[s-1];

	}
	for(int s=0;s<=i;s++){
		if(secondsp[s+1]>=secondsp[s]){
			if(sd_buff==0)sd = secondsp[s];
			sd_buff++;
			sd_slo= 0;

		}
		if(secondsp[s+1]<=secondsp[s]){

			if(sd_buff <= 5 && sd_slo==0) speed_sloow(s-sd_buff,s,sd);
			sd_slo++;
			sd_buff= 0;

		}
	}
	for(int s=i;s>=0;s--){
		if(secondsp[s-1]>=secondsp[s]){
			if(sd_buff==0)sd = secondsp[s];
			sd_buff++;
			sd_slo= 0;

		}
		if(secondsp[s-1]<=secondsp[s]){

			if(sd_buff <=  5 && sd_slo==0) speed_sloow(s,s+sd_buff,sd);
			sd_slo++;
			sd_buff= 0;

		}
	}
//	for(int s=i;s>=0;s--){
//		if(buff < secondsp[s]){
//			for( h=0 ;h<=30;h++){
//				if(secondsp[s-h] < buff)break;
//				secondsp[s-h] = buff;
//
//			}
//			s=s-h;
//		}
//		buff = secondsp[s];
//	}
	count_num = i;
//	for(int s=0;s<i;s++){
//		mon_speed = (float)secondsp[s];
//		//printf("%d,",s);
//		printf("%f\n\r",mon_speed);
//		HAL_Delay(10);
//		count_num = s;
//	}
	i=0;
	loadlog_adress = start_adress_sector9;
	while(1){
		Lo=*(float*)loadlog_adress;
		if(isnan(Lo) != 0)break;
		second_load[i] = Lo;
		loadlog_adress+= 0x04;
		//printf("%lf\r\n",Lo);
		LED(2);
		i++;
		//HAL_Delay(500);

	}

	side_adress = start_adress_sector10;
	while(1){

		cao = *(uint32_t*)side_adress;
		fu =  *(float*)side_adress;
		if(isnan(fu) != 0)break;
		correction[d] = cao;
		d++;
		side_adress+= 0x04;
	}
}
inline void second_driv(float log){
	static float logbuff=0;
	static float cale_buff=0;
	static int i=0;
	static  int hu=0;
	static float logbuff2=0;
	static float cale_val=0;
	static uint16_t num=0;
	static uint16_t corr = 0;
	float cale=0;
	static int frp=0;
	static int frap = 0;
	static int cao=0;
	int cao_d=0;
	int cao_v=10000;
	int carr_d=0;
	int cou_buff=0;
	static uint16_t coi=0;
	static int cros=39;

	//float fu=0;
	static uint d=0;
	if(i<=0)corr=correction[0];
	num++;
	cale_val += calc_angle()*T;

	logbuff = logbuff+ log;
	logbuff2 = logbuff2 + log;
	if(logbuff2>=10.000){
		log_flag = 1;
		cale = cale_val;
		if((cale_buff-cale)<=0.05 && (cale_buff-cale)>=-0.05 )frp++;//0.005
		cale_buff = cale;
		cale_val=0;
		logbuff2=0;
	}
	//test = logbuff;
	//driv_fan(speedget);
	if(straight_flag){
		HAL_GPIO_WritePin(GPIOA, GPIO_PIN_12, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(GPIOA, GPIO_PIN_11, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(GPIOA, GPIO_PIN_10, GPIO_PIN_RESET);
	}
	//else HAL_GPIO_WritePin(GPIOC, GPIO_PIN_8, GPIO_PIN_RESET);
	if(logbuff >= second_load[i]){
		if(logbuff > second_load[i+1]) i++;

		coi++;
		if(secondsp[i]<2500 && count_num > i && maker_flag==1)secondsp[i]=2500;
		if(count_num-100 < i || i <= 100){
			goal=1;
			if(secondsp[i]<2500)secondsp[i]=2500;
		}
		else goal=0;

		Average_speed = secondsp[i];
		co++;
		i++;
		cao++;
		FLASH_Write_Word_F(actual__callog_adress,cale_val);
		FLASH_Write_Word_F(actual__velo_adress,Average_speed);
		FLASH_Write_Word_F(plan_velo_adress,speedget);
		plan_velo_adress += 0x04;
		actual__velo_adress += 0x04;
		actual__callog_adress+= 0x04;

//		if((corr-20) == i) straight_flag = true;
//		if(corr <= i){
//			//straight_flag = false;
//			corr=correction[hu+2];
//		}

		//if((second_load[i]-logbuff)>=10.00)i--;
//		cale = cale_val;
		//cale = calc_angle();
//		if((cale_buff-cale)<=0.001 && (cale_buff-cale)>=-0.001 ){
//			frp++;
//			//if(abs((corr-i) <= 20) && cao >= 30) straight_flag = true;
//			//if(corr <= i)corr=correction[d+1];
//		}

//		cale_buff = cale;
		if(frp >= 40 || cross_line){
			straight_flag = true;
			frp=30;
			frap = 0;
		}

		//cale_val=0;
		num=0;
	}
//	if(correc_maker==1 ){
	if(correc_maker==1  && straight_flag == 1){

		hu=0;
		while(1){
			cao = correction[hu];
			if(correction[hu]==0 && hu > 0)break;
			cao_d = abs(i-cao);

			//if(cao_d<30)break;
			if (cao_v > cao_d){
				cao_v = cao_d;
				cou_buff = cao;
				carr_d=hu;
			}
			hu++;

		}
		if(abs(i-cou_buff)<=30 /*|| (cao >= 500 && abs(i-cou_buff)<=500)*/){

			HAL_GPIO_WritePin(GPIOA, GPIO_PIN_12, GPIO_PIN_SET);
			HAL_GPIO_WritePin(GPIOA, GPIO_PIN_11, GPIO_PIN_SET);
			HAL_GPIO_WritePin(GPIOA, GPIO_PIN_10, GPIO_PIN_SET);
			i = cou_buff;
			logbuff=second_load[i];
			hu = carr_d;
			d= carr_d+1;
			corr=correction[d];
			cao=0;
			coi=0;




		}
//		d++;
//		corr=correction[d];
		straight_flag=false;
		correc_maker = false;
		frp=0;
	}
	if(cross_maker){
		HAL_GPIO_WritePin(GPIOA, GPIO_PIN_12, GPIO_PIN_SET);
		HAL_GPIO_WritePin(GPIOA, GPIO_PIN_11, GPIO_PIN_SET);
		HAL_GPIO_WritePin(GPIOA, GPIO_PIN_10, GPIO_PIN_SET);
		i = work_ram[cros];
		logbuff=second_load[i];
		cross_maker = false;
		cros++;
	}
	if(correc_maker==1){
		frp=0;
		coi=0;
		straight_flag = false;
		correc_maker = false;

	}

}

void log_play(void){
	float Ca,Lo,SP,PLsp,PLca;
	float Lo_buff=0;
	int count=0;
	second_soeed=2;
	log_Cal();
	log_init ();
	while(1){
		PLca =*(float*)actual__callog_adress;
		PLsp =*(float*)actual__velo_adress;
		SP =*(float*)plan_velo_adress;
		Lo=*(float*)loadlog_adress-Lo_buff;
		Ca=*(float*)callog_adress;
		if(isnan(Ca) != 0)break;
		Lo_buff += Lo;
		if(secondsp[count]<=1800)secondsp[count]=2000;
		printf("%lf,%lf,%lf,%lf,%lf,%lf\n\r",Lo,Ca,secondsp[count],SP,PLsp,PLca);
		count++;
		//HAL_Delay(1);
		callog_adress+= 0x04;
		loadlog_adress+= 0x04;
		plan_velo_adress+= 0x04;
		actual__velo_adress+= 0x04;
		actual__callog_adress+= 0x04;
		//plan_velo_adress+= 0x04;
	}

}
