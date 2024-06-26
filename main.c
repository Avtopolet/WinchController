/*
 * BMSBattery S series motor controllers firmware
 *
 * Copyright (C) Casainho, 2017.
 *
 * Released under the GPL License, Version 3
 */

#include <stdint.h>
#include <stdio.h>
#include "stm8s.h"
#include "gpio.h"
#include "stm8s_itc.h"
#include "stm8s_gpio.h"
#include "interrupts.h"
#include "stm8s_tim2.h"
#include "motor.h"
#include "main.h"
#include "uart.h"
#include "adc.h"
#include "brake.h"
#include "cruise_control.h"
#include "timers.h"
#include "pwm.h"
#include "PAS.h"
#include "SPEED.h"
//#include "update_setpoint.h"
#include "ACAsetPoint.h"
#include "config.h"
#include "display.h"
#include "display_kingmeter.h"
#include "ACAcontrollerState.h"
#include "BOdisplay.h"
#include "ACAeeprom.h"
#include "ACAcommons.h"

//uint16_t ui16_LPF_angle_adjust = 0;
//uint16_t ui16_LPF_angle_adjust_temp = 0;

uint16_t ui16_log1 = 0;
uint8_t ui8_slowloop_flag = 0;
uint8_t ui8_veryslowloop_counter = 0;
uint8_t ui8_ultraslowloop_counter = 0;
uint16_t ui16_log2 = 0;
uint8_t ui8_log = 0;
uint8_t ui8_i = 0; //counter for ... next loop

float float_kv = 0;
float float_R = 0;
uint8_t a = 0; //loop counter

static int16_t i16_deziAmps;

//Start UART RX block

uint8_t ui8_rx_buffer1[17]; // modbus ascii with max 8 bytes payload (array including padding) // modbus rtu uses only 11 bytes
uint8_t ui8_rx_converted_buffer1[7]; // for decoded ascii values
uint8_t ui8_rx_buffer_counter1 = 0;
//End UART RX block

uint16_t ui16_motor_speed_erps_old=0;
uint8_t average_erps[10];
uint8_t erps_counter=0;
uint8_t ui8_value_old=0;
uint8_t DC_value=0;
uint8_t DC_direction=0;
uint8_t speed_correction=0;
uint8_t DC_brake=0; // DC - duty cycle

uint8_t control_counter=0;

int avg_erps=0;
int erps_target=0;
int setpoint_correction;
int erps_correction_flag=0;
int erps_correction_target=0;
int erps_must_be = 0;
int erps_factor = 0;


/////////////////////////////////////////////////////////////////////////////////////////////
//// Functions prototypes///
uint8_t hex2int(uint8_t ch) {
	if (ch >= 0x30 && ch <= 0x39)
		return ch - 0x30;
	if (ch >= 0x41 && ch <= 0x46)
		return ch - 0x41 + 10;
	if (ch >= 0x61 && ch <= 0x66)
		return ch - 0x61 + 10;
	return 0;
}
uint8_t readRtu() {
	//uart_fill_rx_packet_buffer(ui8_rx_buffer1, 11, &ui8_rx_buffer_counter1);
	
	if (ui8_rx_buffer_counter1 > 3) {
		printf("Command error!!!");
		ui8_rx_buffer_counter1 = 0;
		return 0;
	}
	uart_fill_rx_packet_buffer(ui8_rx_buffer1, 12, &ui8_rx_buffer_counter1);

	if (ui8_rx_buffer_counter1 == 4) {
		ui8_value = (hex2int(ui8_rx_buffer1[0])<< 4)+ hex2int(ui8_rx_buffer1[1]);
		DC_brake  = (hex2int(ui8_rx_buffer1[2])<< 4)+ hex2int(ui8_rx_buffer1[3]);
		/*ui8_rx_converted_buffer1[0] = ui8_rx_buffer1[0];
		ui8_rx_converted_buffer1[1] = ui8_rx_buffer1[1];
		ui8_rx_converted_buffer1[2] = ui8_rx_buffer1[2];
		ui8_rx_converted_buffer1[3] = ui8_rx_buffer1[3];
		ui8_rx_converted_buffer1[4] = ui8_rx_buffer1[4];
		ui8_rx_converted_buffer1[5] = ui8_rx_buffer1[5];
		ui8_rx_converted_buffer1[6] = ui8_rx_buffer1[6];*/
		// allow fetching of new data
		ui8_rx_buffer_counter1 = 0;
		return 1;
	}
	return 0;
}

uint8_t readUart() {
	return readRtu();
}

// main -- start of firmware and main loop
int main(void);

//With SDCC, interrupt service routine function prototypes must be placed in the file that contains main ()
//in order for an vector for the interrupt to be placed in the the interrupt vector space.  It's acceptable
//to place the function prototype in a header file as long as the header file is included in the file that
//contains main ().  SDCC will not generate any warnings or errors if this is not done, but the vector will
//not be in place so the ISR will not be executed when the interrupt occurs.

//Calling a function from interrupt not always works, SDCC manual says to avoid it. Maybe the best is to put
//all the code inside the interrupt

// Local VS global variables
// Sometimes I got the following error when compiling the firmware: motor.asm:750: Error: <r> relocation error
// and the solution was to avoid using local variables and define them as global instead

// Brake signal interrupt
void EXTI_PORTA_IRQHandler(void) __interrupt(EXTI_PORTA_IRQHANDLER);
// Speed signal interrupt
void EXTI_PORTC_IRQHandler(void) __interrupt(EXTI_PORTC_IRQHANDLER);
// PAS signal interrupt
void EXTI_PORTD_IRQHandler(void) __interrupt(EXTI_PORTD_IRQHANDLER);

// Timer1/PWM period interrupt
void TIM1_UPD_OVF_TRG_BRK_IRQHandler(void) __interrupt(TIM1_UPD_OVF_TRG_BRK_IRQHANDLER);

// Timer2/slow control loop
void TIM2_UPD_OVF_TRG_BRK_IRQHandler(void) __interrupt(TIM2_UPD_OVF_TRG_BRK_IRQHANDLER);

// UART2 receivce handler
void UART2_IRQHandler(void) __interrupt(UART2_IRQHANDLER);


/////////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////////

int main(void) {
	ui8_value=0;
	//set clock at the max 16MHz
	CLK_HSIPrescalerConfig(CLK_PRESCALER_HSIDIV1);

	gpio_init();
	debug_pin_init();
	debug_pin_set();
	
	brake_init();
	
	while (brake_is_set()); // hold here while brake is pressed -- this is a protection for development
	//debug_pin_init();
	light_pin_init();
	timer2_init();
	uart_init();
	eeprom_init();
	controllerstate_init();
	initErpsRatio();
	pwm_init();
	hall_sensor_init();
	adc_init();
	PAS_init();
	SPEED_init();
	aca_setpoint_init();
#if (defined (DISPLAY_TYPE) && defined (DISPLAY_TYPE_KINGMETER)) || defined DISPLAY_TYPE_KT_LCD3 || defined BLUOSEC
	display_init();
#endif

	//  ITC_SetSoftwarePriority (ITC_IRQ_TIM1_OVF, ITC_PRIORITYLEVEL_2);

	enableInterrupts();

	watchdog_init(); //init watchdog after enabling interrupt to have fast loop running already

#if (SVM_TABLE == SVM)
	TIM1_SetCompare1(126 << 1);
	TIM1_SetCompare2(126 << 1);
	TIM1_SetCompare3(126 << 1);
#elif (SVM_TABLE == SINE) || (SVM_TABLE == SINE_SVM)
	TIM1_SetCompare1(126 << 2);
	TIM1_SetCompare2(126 << 2);
	TIM1_SetCompare3(126 << 2);
#endif

	hall_sensors_read_and_action(); // needed to start the motor
	//printf("Back in Main.c\n");

	for (a = 0; a < NUMBER_OF_PAS_MAGS; a++) {// array init
		ui16_torque[a] = 0;
	}
#ifdef DIAGNOSTICS
	printf("System initialized\r\n");
#endif
	while (1) {

		uart_send_if_avail();

		updateSpeeds();
		updatePasStatus();

#if (defined (DISPLAY_TYPE) && defined (DISPLAY_TYPE_KINGMETER)) || defined DISPLAY_TYPE_KT_LCD3 || defined BLUOSEC
		display_update();
#endif

		// scheduled update of setpoint and duty cycle (slow loop, 50 Hz)
		if (ui8_slowloop_flag) {
			//printf("MainSlowLoop\n");
		    debug_pin_set();
			ui8_slowloop_flag = 0; //reset flag for slow loop
			ui8_veryslowloop_counter++; // increase counter for very slow loop

			checkPasInActivity();
			updateRequestedTorque(); //now calculates tq for sensor as well
			updateSlowLoopStates();
			updateX4();
			updateLight();
			ui16_setpoint = (uint16_t) aca_setpoint(ui16_time_ticks_between_pas_interrupt, ui16_setpoint); //update setpoint

			//#define DO_CRUISE_CONTROL 1
#if DO_CRUISE_CONTROL == 1
			ui16_setpoint = cruise_control(ui16_setpoint);
#endif

			erps_must_be=ui8_value/2;
			if (erps_counter<10) {
			average_erps[erps_counter]=ui16_motor_speed_erps;
			erps_counter++;} else {
				for (int i =0; i<10;i++) {
				avg_erps += average_erps[i];	
				}
				avg_erps=avg_erps/10;
				//printf("avgerps %u \r\n",avg_erps);
				ui16_motor_speed_erps_old=avg_erps;
				erps_counter=0;
				avg_erps=0;}

			if (ui8_value != ui8_value_old) {DC_value=ui8_value;}
			
			if(ui16_motor_speed_erps_old-ui16_motor_speed_erps>1 && ui16_motor_speed_erps_old !=0 && (ui16_motor_speed_erps_old-ui16_motor_speed_erps)<65500 ) {
				
				printf("ERPS DOWN!!! -- before %u -- now %u -- difference %u -- setpoint %u -- erps_must_be %u -- erps_factor %u \r\n",ui16_motor_speed_erps_old,ui16_motor_speed_erps,ui16_motor_speed_erps_old-ui16_motor_speed_erps,ui16_setpoint,erps_must_be,erps_factor);
				
				if (ui8_value == ui8_value_old && ui8_value !=0 ) {
					
					erps_factor=ui8_value/erps_must_be;
					if(erps_factor>=2){erps_correction_flag =1;}
					}
					if (ui8_value ==0) {setpoint_correction=0;}


			}
			//printf("ERPS -- before %u -- now %u -- difference %u \r\n",ui16_motor_speed_erps_old,ui16_motor_speed_erps,ui16_motor_speed_erps_old-ui16_motor_speed_erps);
			//
			ui16_motor_speed_erps_old = ui16_motor_speed_erps;
			ui8_value_old=ui8_value;
			
			if(speed_correction == 1) {
			
			if (erps_must_be+3>ui16_motor_speed_erps && erps_must_be>0 && ui8_value != 0) {
				DC_value=DC_value+1;
				erps_correction_flag = 0;
				printf("Speed CORRECTED UP >> %u \r\n", ui8_value);}
			
			if(erps_must_be-3<ui16_motor_speed_erps && ui8_value != 0) {
				DC_value=DC_value-1;
				
				erps_correction_flag = 0;
				
				printf("Speed CORRECTED DOWN <<  %u \r\n",ui8_value);
			}

									}

			/*if (erps_must_be+3>ui16_motor_speed_erps && erps_must_be>0 && ui8_value != 0) {
				ui8_value=ui8_value + 1;
				printf("Setpoint CORRECTED UP>> %u \r\n", ui8_value);}

			if (erps_must_be-3<ui16_motor_speed_erps && erps_must_be>0 && ui8_value != 0) {
				ui8_value=ui8_value - 1;
				printf("Setpoint CORRECTED DOWN>> %u \r\n", ui8_value);}*/	
			
			
			//if (ui16_motor_speed_erps>0) {ui16_setpoint=ui16_setpoint + setpoint_correction;}
			
			//pwm_set_duty_cycle((uint8_t) ui16_setpoint);
			//printf("SetDutyCycle\r\n");
			if (speed_correction==1) {pwm_set_duty_cycle (DC_value, DC_direction);} //DC_value
			if (speed_correction==0) {pwm_set_duty_cycle (DC_brake, DC_direction);}
			ui16_motor_speed_erps_old = ui16_motor_speed_erps;
			ui8_value_old=ui8_value;

			
			
		

			
			
			/****************************************************************************/
			//very slow loop for communication
			if (ui8_veryslowloop_counter > 5) {

				ui8_ultraslowloop_counter++;
				ui8_veryslowloop_counter = 0;

				if (ui8_ultraslowloop_counter > 10) {
					ui8_ultraslowloop_counter = 0;
					control_counter ++;
					ui8_uptime++;
				}

				if (control_counter == 1) {ui8_value; DC_direction=0; speed_correction=1;}
				if (control_counter == 2) {ui8_value; DC_direction=0; control_counter=0;speed_correction=0; }

#ifdef DIAGNOSTICS
				
				//uint32_torquesensorCalibration=80;
				//printf("%u,%u, %u, %u, %u, %u\r\n", ui16_control_state, (uint16_t) uint32_current_target, PAS_is_active, ui16_BatteryCurrent, ui16_sum_torque, (uint16_t)uint32_torquesensorCalibration);
				//if(readRtu()>0){readRtu(); printf("RX buffer %u, %u, %u\r\n",ui8_rx_buffer[0],ui8_rx_buffer[1],ui8_rx_buffer[2]);}	
				//printf("Fillbufer %u, counter %u\r\n", readRtu(), ui8_rx_buffer_counter1); 
				if (readRtu()>0) {
					//printf("RX buffer %u, %u, %u, %u, %u, %u, %u, counter %u\r\n",ui8_rx_buffer1[0],ui8_rx_buffer1[1],ui8_rx_buffer1[2],ui8_rx_buffer1[3],ui8_rx_buffer1[4],ui8_rx_buffer1[5],ui8_rx_buffer1[6],ui8_rx_buffer_counter1);
					printf("RX buffer %u, %u, %u, counter %u, value %u, current target %u\r\n",ui8_rx_buffer1[0],ui8_rx_buffer1[1],ui8_rx_buffer1[2],ui8_rx_buffer_counter1,ui8_value,(uint16_t) uint32_current_target);
					}

				
				//printf("erps %d, motorstate %d, cyclecountertotal %d\r\n", ui16_motor_speed_erps, ui8_possible_motor_state|ui8_dynamic_motor_state, ui16_PWM_cycles_counter_total);

				//printf("cheatstate, %d, km/h %lu, Voltage, %d, setpoint %d, erps %d, current %d, correction_value, %d\r\n", ui8_offroad_state, ui32_speed_sensor_rpks, ui8_BatteryVoltage, ui16_setpoint, ui16_motor_speed_erps, ui16_BatteryCurrent, ui8_position_correction_value);

				//printf("ui8_value %u, Voltage, %d, setpoint %d, erps %d, current >> %d, CorVal, %d - CurrTGT%u \r\n",  ui8_value, ui8_BatteryVoltage, ui16_setpoint, ui16_motor_speed_erps, ui16_BatteryCurrent, ui8_position_correction_value,(uint16_t) uint32_current_target);
				printf("DC_value %u, erps %d, erps_must_be %u, setspeed %d, DC_brake %u, \r\n", DC_value, ui16_motor_speed_erps, erps_must_be,ui8_value,DC_brake);

				/*for(a = 0; a < 6; a++) {			// sum up array content
						 putchar(uint8_t_hall_case[a]);
						 }
				putchar(ui16_ADC_iq_current>>2);
				putchar(ui8_position_correction_value);
				putchar(255);*/
				// printf("%d, %d, %d, %d, %d, %d\r\n", (uint16_t) uint8_t_hall_case[0], (uint16_t)uint8_t_hall_case[1],(uint16_t) uint8_t_hall_case[2],(uint16_t) uint8_t_hall_case[3], (uint16_t)uint8_t_hall_case[4], (uint16_t)uint8_t_hall_case[5]);
				//printf("%d, %d, %d, %d, %d, %d, %d,\r\n", ui8_position_correction_value, ui16_BatteryCurrent, ui16_setpoint, ui8_regen_throttle, ui16_motor_speed_erps, ui16_ADC_iq_current>>2,ui16_adc_read_battery_voltage());


				//printf("correction angle %d, Current %d, Voltage %d, sumtorque %d, setpoint %d, km/h %lu\n",ui8_position_correction_value, i16_deziAmps, ui8_BatteryVoltage, ui16_sum_throttle, ui16_setpoint, ui32_speed_sensor_rpks);
#endif
			}//end of very slow loop

			debug_pin_reset();
		}// end of slow loop
	}// end of while(1) loop
}


