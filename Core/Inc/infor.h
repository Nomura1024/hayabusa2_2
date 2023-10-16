/*
 * infor.h
 *
 *  Created on: 2022/03/16
 *      Author: Owner
 */

#ifndef INC_INFOR_H_
#define INC_INFOR_H_

#define T 0.001
#define SENSOR_NUMBER 13
#define BACKUP_FLASH_SECTOR_NUM     FLASH_SECTOR_1
#define BACKUP_FLASH_SECTOR_SIZE    1024*16
#define ESC_MAX 		3527	//84[us]
#define ESC_MIN			1763	//42[us]




extern uint16_t Accm;
extern uint16_t Decm;
extern uint16_t analog[SENSOR_NUMBER];
extern uint16_t ADC_dif[SENSOR_NUMBER];
extern uint16_t ADC_Small[SENSOR_NUMBER];
extern uint16_t work_ram[BACKUP_FLASH_SECTOR_SIZE] __attribute__ ((aligned(4)));
extern char _backup_flash_start;
extern int16_t Average_speed;
extern uint16_t  Kp;
extern uint16_t  Ki;
extern uint16_t  Kd ;
extern uint8_t maker_right;
extern uint8_t maker_left;
extern uint8_t log_flag;
extern uint8_t log2_flag;
extern uint8_t cross_line;
extern float ang_average;
extern uint8_t maker_flag;
extern uint8_t cross_flag;

extern uint32_t callog_adress;
extern uint32_t loadlog_adress;
extern uint32_t plan_velo_adress;
extern uint32_t side_adress;
extern float load_log;
extern uint16_t co;
extern uint16_t driv_flag;
extern float mon_speed;
extern uint8_t straight_flag;
extern uint8_t correc_maker;
extern uint32_t log_count;
extern float second_load[6100];
extern uint8_t goal;
extern uint8_t second_soeed;
extern uint8_t cross_maker;
extern float speedget;
extern float ang;
#endif /* INC_INFOR_H_ */
