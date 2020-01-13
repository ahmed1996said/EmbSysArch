/*----------------------------------------------------------------------------
 *----------------------------------------------------------------------------*/
#include <MKL25Z4.H>
#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <string.h>
#include "gpio_defs.h"
#include "LEDs.h"
#include "i2c.h"
#include "mma8451.h"
#include "delay.h"
#include "RTCS.h"

#define FLASH_DELAY 2
#define ACC_SENSITIVITY 90
#define TASK_MOTION_SENSOR_FREQ_HZ (50) 
#define BLOCKING 1
#define FSM_MODE 2
extern int lock_detect;
extern volatile I2C_MESSAGE_T g_I2C_Msg;
unsigned int test=0;

void Init_Debug_Signals(void) {
	SIM->SCGC5 |= SIM_SCGC5_PORTB_MASK;

	PORTB->PCR[DBG_0] |= PORT_PCR_MUX(1);          
	PORTB->PCR[DBG_1] |= PORT_PCR_MUX(1);          
	PORTB->PCR[DBG_2] |= PORT_PCR_MUX(1);          
	PORTB->PCR[DBG_3] |= PORT_PCR_MUX(1);          
	PORTB->PCR[DBG_4] |= PORT_PCR_MUX(1);          
	PORTB->PCR[DBG_5] |= PORT_PCR_MUX(1);          
	PORTB->PCR[DBG_6] |= PORT_PCR_MUX(1);          
	PORTB->PCR[DBG_7] |= PORT_PCR_MUX(1);          

	PTB->PDDR |= MASK(DBG_0) | MASK(DBG_1) | MASK(DBG_2) | MASK(DBG_3) | MASK(DBG_4) | MASK(DBG_5) | MASK(DBG_6) | MASK(DBG_7);
	PTB->PCOR = MASK(DBG_0) | MASK(DBG_1) | MASK(DBG_2) | MASK(DBG_3) | MASK(DBG_4) | MASK(DBG_5) | MASK(DBG_6) | MASK(DBG_7);

}


void Init_Config_Signals(void) {
	SIM->SCGC5 |= SIM_SCGC5_PORTE_MASK;

	PORTE->PCR[CONFIG1_POS] &= ~PORT_PCR_MUX_MASK;          
	PORTE->PCR[CONFIG1_POS] |= PORT_PCR_MUX(1);          
	PTE->PDDR &= ~MASK(CONFIG1_POS);
	
	PORTE->PCR[CONFIG1_POS] |= PORT_PCR_PS_MASK;
	PORTE->PCR[CONFIG1_POS] |= PORT_PCR_PE_MASK;
	
	
	

	PORTE->PCR[CONFIG2_POS] &= ~PORT_PCR_MUX_MASK;          
	PORTE->PCR[CONFIG2_POS] |= PORT_PCR_MUX(1);          
	PTE->PDDR &= ~MASK(CONFIG2_POS);

	PORTE->PCR[CONFIG3_POS] &= ~PORT_PCR_MUX_MASK;          
	PORTE->PCR[CONFIG3_POS] |= PORT_PCR_MUX(1);          
	PTE->PDDR &= ~MASK(CONFIG3_POS);	//input
	

	
}





void Task_I2C_Server_FSM(void){
	
	static enum{S1,S2,S3,S4,S5,S6,S7,S8,S9,S10,S11} next_state = S1;
	static uint8_t dummy, num_bytes_read=0, is_last_read=0;
	static uint8_t Dev_adx, Reg_adx,Data_count;

			
	SET_BIT(DEBUG_I2C_CODE);	
	//--- FSM----
	switch(next_state){

			case S1:			
					if(g_I2C_Msg.Status == IDLE && g_I2C_Msg.Command==READ)
					{
						g_I2C_Msg.Status = READING;
						//save shared variables into local ones
						Data_count = g_I2C_Msg.Data_count;
						Reg_adx = g_I2C_Msg.Reg_adx;
						Dev_adx = g_I2C_Msg.Dev_adx;
						g_I2C_Msg.Command = NONE;		//reset
						I2C_TRAN;													//	set to transmit mode		
						SET_BIT(DEBUG_MSG_ON_BUS);	
						I2C_M_START;											     //	send start		
						I2C0->D = Dev_adx;					//	send dev address (write)
						lock_detect = 0;						
						next_state = S2;
						//SET_BIT(DEBUG_I2C_BUSY_WAIT);
				}
					else{
					//stay in this state until IDLE AND READ set
					}
						break;						
			case S2:
								  SET_BIT(DEBUG_I2C_BUSY_WAIT);
									if( ( I2C0->S & I2C_S_IICIF_MASK ) == 0 ) {
									}
									else{
									 I2C0->S |= I2C_S_IICIF_MASK;
										next_state = S3;
									}
									  CLEAR_BIT(DEBUG_I2C_BUSY_WAIT);									
									break;
			
			case S3:						
									I2C0->D = Reg_adx;				//	send register address	
									next_state = S4;
									break;
			
			case S4:
									SET_BIT(DEBUG_I2C_BUSY_WAIT);
									if( ( I2C0->S & I2C_S_IICIF_MASK ) == 0 )  {
									}
									else{
									 I2C0->S |= I2C_S_IICIF_MASK;										
										next_state = S5;
									}
									 CLEAR_BIT(DEBUG_I2C_BUSY_WAIT);									
									break;
			case S5:			
								I2C_M_RSTART;											//	repeated start									
								I2C0->D = Dev_adx | 0x01 ;				//	send dev address (read)							
								next_state = S6;
								break;
		
			case S6:
								SET_BIT(DEBUG_I2C_BUSY_WAIT);
								if(  ( I2C0->S & I2C_S_IICIF_MASK ) == 0 ) {
								}
								else{
									 I2C0->S |= I2C_S_IICIF_MASK;									
									next_state = S7;
								}
          			CLEAR_BIT(DEBUG_I2C_BUSY_WAIT);
								break;
	
			case S7:
								I2C_REC;													//	set to receive mode		
								next_state = S8;	
								break;		
			
			case S8:			
								if(num_bytes_read<Data_count){		
									is_last_read = (num_bytes_read == g_I2C_Msg.Data_count-1)? 1: 0;
									if (is_last_read){
										NACK;													// tell HW to send NACK after read							
									} else {
										ACK;													// tell HW to send ACK after read								
										}
								
							  dummy = I2C0->D;								//	dummy read
								next_state = S9;	
								}	
								else
									 next_state = S11;		
								break;
								
			case S9:
				  			SET_BIT(DEBUG_I2C_BUSY_WAIT);										
								if( ( I2C0->S & I2C_S_IICIF_MASK ) == 0 )  {
								}
								else{
									 I2C0->S |= I2C_S_IICIF_MASK;									
									 next_state = S10;
								}
								CLEAR_BIT(DEBUG_I2C_BUSY_WAIT);
								break;
								
			case S10:
								if (is_last_read){
								I2C_M_STOP;										//	send stop	
								CLEAR_BIT(DEBUG_MSG_ON_BUS);	
								}					
								g_I2C_Msg.Data[num_bytes_read++] = I2C0->D; //	read data	
								next_state = S8;
								break;
									
			case S11:
								g_I2C_Msg.Status = READ_COMPLETE;
								next_state = S1;
								num_bytes_read = 0;
								break;
			
			default: 
								next_state = S1;
								g_I2C_Msg.Status = IDLE;
								break;
			
	///---------------------------------------------
		}
				
			CLEAR_BIT(DEBUG_I2C_CODE);
	
}




void Task_Motion_Sensor_FSM(void){

	static int16_t prev_acc_X=0, prev_acc_Y=0, prev_acc_Z=0;
	static int16_t acc_X=0, acc_Y=0, acc_Z=0;
	static uint8_t rf, gf, bf;
	static int i;
//	static uint8_t data[6];
	static int16_t temp[3];
	static enum{S1,S2,S3} n_state = S1;

	
	SET_BIT(DEBUG_TASK_MOTION_SENSOR);
	
		switch(n_state){
							
				case S1:
					RTCS_Set_Task_Period(Task_Motion_Sensor_FSM, 1,0);				
					if(g_I2C_Msg.Status == IDLE){
							g_I2C_Msg.Dev_adx = MMA_ADDR;
							g_I2C_Msg.Reg_adx = REG_XHI;
							g_I2C_Msg.Data_count = 6;
							g_I2C_Msg.Command = READ;
						n_state = S2;
					}
					break;
					
				case S2:
					if(g_I2C_Msg.Status == READ_COMPLETE){
						//can read data now
						
						for ( i=0; i<3; i++ ) {
							temp[i] = (int16_t) ((g_I2C_Msg.Data[2*i]<<8) | g_I2C_Msg.Data[2*i+1]);
						}
						// Align for 14 bits
						acc_X = temp[0]/4;
						acc_Y = temp[1]/4;
						acc_Z = temp[2]/4;
						

						rf = abs(prev_acc_X - acc_X) > ACC_SENSITIVITY ? 1 : 0;
						gf = abs(prev_acc_Y - acc_Y) > ACC_SENSITIVITY ? 1 : 0;
						bf = abs(prev_acc_Z - acc_Z) > ACC_SENSITIVITY ? 1 : 0;

						Control_RGB_LEDs(rf, gf, bf);

  				 // Delay(FLASH_DELAY);
						
						RTCS_Set_Task_Period(Task_Motion_Sensor_FSM, 1,0);
						n_state = S3;
					}
			else 
				n_state = S2;
			break;
			
				case S3:
						Control_RGB_LEDs(0, 0, 0);	
					//	Delay(FLASH_DELAY*2);		
					  RTCS_Set_Task_Period(Task_Motion_Sensor_FSM, 9,0);	
						prev_acc_X = acc_X;
						prev_acc_Y = acc_Y;
						prev_acc_Z = acc_Z;
						// change status to IDLE
						g_I2C_Msg.Status = IDLE;
						n_state = S1;
				break;
			
			default:
						n_state = S1;
						break;
						
}
	CLEAR_BIT(DEBUG_TASK_MOTION_SENSOR);

}



void Task_Motion_Sensor(void) {
	
	
	static int16_t prev_acc_X=0, prev_acc_Y=0, prev_acc_Z=0;
	int16_t acc_X=0, acc_Y=0, acc_Z=0;
	uint8_t rf, gf, bf;
	
	SET_BIT(DEBUG_TASK_MOTION_SENSOR);
	
	read_full_xyz(&acc_X, &acc_Y, &acc_Z);

	rf = abs(prev_acc_X - acc_X) > ACC_SENSITIVITY ? 1 : 0;
	gf = abs(prev_acc_Y - acc_Y) > ACC_SENSITIVITY ? 1 : 0;
	bf = abs(prev_acc_Z - acc_Z) > ACC_SENSITIVITY ? 1 : 0;

	Control_RGB_LEDs(rf, gf, bf);
	Delay(FLASH_DELAY);
	Control_RGB_LEDs(0, 0, 0);							
	Delay(FLASH_DELAY*2);		

	prev_acc_X = acc_X;
	prev_acc_Y = acc_Y;
	prev_acc_Z = acc_Z;
	
	CLEAR_BIT(DEBUG_TASK_MOTION_SENSOR);
}



/*----------------------------------------------------------------------------
  MAIN function
 *----------------------------------------------------------------------------*/
int main (void) {
	int MODE;

	Init_RGB_LEDs();
	Init_Debug_Signals();
	Init_Config_Signals();

	
	if(PTE->PDIR & MASK(CONFIG1_POS))
		MODE = BLOCKING;
	else
		MODE = FSM_MODE;
		
	Control_RGB_LEDs(1, 1, 0);								/* yellow: starting up */
	i2c_init();																/* init i2c	*/
	Delay(200);
	
	if (!init_mma()) {												/* init mma peripheral */
		Control_RGB_LEDs(1, 0, 0);							/* Light red error LED */
		while (1)																/* not able to initialize mma */
			;
	}
	Control_RGB_LEDs(0, 0, 0);							
	
	if(MODE== BLOCKING){	
		
			RTCS_Init(SCHED_FREQ_HZ);
			RTCS_Add_Task(Task_Motion_Sensor, 0, TICKS(TASK_MOTION_SENSOR_FREQ_HZ)); // Run periodically
			RTCS_Run_Scheduler();
	}
	
	if(MODE == FSM_MODE){

			RTCS_Init(500);			
			RTCS_Add_Task(Task_I2C_Server_FSM, 0, 1); // Run periodically
			RTCS_Add_Task(Task_Motion_Sensor_FSM, 1, 1);
			RTCS_Run_Scheduler();
		
	}
	
	
}

