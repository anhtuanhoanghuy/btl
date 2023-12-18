#include <MKL46Z4.h>


volatile uint32_t msTicks = 0;
volatile uint8_t status = 0x00;
int count = 0;



void init_LED(void) {
	SIM->SCGC5 |= SIM_SCGC5_PORTD_MASK; //Enable Clock to PORT D of Green LED
	PORTD->PCR[5] = PORT_PCR_MUX(1u);	// Config PORT D to GPIO
	PTD->PDDR |= 1<<5;	//set PTD5 to Output
	PTD->PSOR = 1<<5; //set bit 5 to 1
	
	SIM->SCGC5 |= SIM_SCGC5_PORTE_MASK; //Enable Clock to port E of Red LED
	PORTE->PCR[29] = PORT_PCR_MUX(1u);;	// Config PORT E to GPIO
	PTE->PDDR |= 1<<29; //set PTE29 to Output
	PTE->PSOR = 1<<29; //set bit 29 to 1
}
void green_LED_ON(void) {
	PTD->PCOR = 1<<5; //clear bit 5 to 0
}
void green_LED_OFF(void) {
	PTD->PSOR = 1<<5; //set bit 5 to 1
}
void red_LED_ON(void) {
	PTE->PCOR = 1<<29; //clear bit 29 to 0
}
void red_LED_OFF(void) {
	PTE->PSOR = 1<<29; //set bit 29 to 1
}

void init_switch(void) {
	SIM->SCGC5 |= SIM_SCGC5_PORTC_MASK; //Enable Clock to PORT C of Switch 1 and Switch 3
	PORTC->PCR[3] = PORT_PCR_MUX(1u)|PORT_PCR_PE_MASK|PORT_PCR_PS_MASK|PORT_PCR_IRQC(0xA); // Config Pin 3 PORT C to GPIO, enable pull-up resistor and set falling edge interupt
	PTC->PDDR &= ~(1u<<3); //set PTC3 to Input
	PORTC->PCR[12] = PORT_PCR_MUX(1u)|PORT_PCR_PE_MASK|PORT_PCR_PS_MASK|PORT_PCR_IRQC(0xA); // Config Pin 12 PORT C to GPIO, enable pull-up resistor and set falling edge interupt
	PTC->PDDR &= ~(1u<<12);	//set PTC12 to Input
	NVIC_ClearPendingIRQ(31);	//Clear NVIC any pending interrupts on PORTC_D
	NVIC_EnableIRQ(31); // Enable NVIC interrupts source for PORTC_D module
}

void PORTC_PORTD_IRQHandler(void) {

	if ((PTC->PDIR & (1<<3)) == 0) { //Switch 1 is pressed
			PORTC->PCR[3] |= PORT_PCR_ISF_MASK;
			status ^= 0x01; //Status of Switch 1 is saved in bit 0x01
			
		
		
	}
	if ((PTC->PDIR & (1<<12)) == 0) { //Switch 3 is pressed
		PORTC->PCR[12] |= PORT_PCR_ISF_MASK; 
		status ^= 0x02;	//Status of Switch 3 is saved in bit 0x02
		
	}
}


void init_SysTick_interupt(void) {
	SysTick->LOAD |= (SystemCoreClock/1000) -1 ;
	SysTick->CTRL |= SysTick_CTRL_CLKSOURCE_Msk | SysTick_CTRL_TICKINT_Msk | SysTick_CTRL_ENABLE_Msk;
}

void SysTick_Handler (void) {
	msTicks++;
}
			
void delay(uint32_t TICK) {
	while(msTicks < TICK);
	msTicks = 0;
}




int main(void) {
	init_LED();
	init_switch();
	init_SysTick_interupt();
	delay(2);
	while(1) {
	if ((status & 0x01)== 0) { //chua ngoi tren xe
			green_LED_OFF();
			//SegLCD_Set(3,0x01); //OFF
			//delay(100);
			status &= ~(1<<2);
			if((status & 0x02)==0) { //chua ngoi tren xe va chua that day an toan
				red_LED_ON();
			} else { //chua ngoi tren xe ma da that day an toan 
				red_LED_OFF();
			}
		} else {	//Switch 1 nhan -> da ngoi tren xe 00000001
				if((status & 0x04) == 0) {
					//SegLCD_Set(4,4); //IN
					delay(1);
					count++;
					if (count < 5) {
						 if ((status & 0x02) != 0) { //Switch 1 nhan & switch 3 k nhan
									red_LED_OFF();
									green_LED_ON();
									//SegLCD_Set(4,4); //OK
									count = 0;
									status |= (1<<2);
						 }
					} else {
						 count = 0;
						 status |= (1<<2);            
						}
				} else {
						if ((status & 0x02) == 0) { //Switch 1 nhan & switch 3 khong nhan
							//SegLCD_Set(4,4); //IN
							green_LED_OFF();
							red_LED_ON();
							delay(1);
							red_LED_OFF();
							delay(1);
						} else {//Switch 1 nhan & switch 3 nhan
							red_LED_OFF();
							green_LED_ON();
							//SegLCD_Set(4,4); //OK
						}
				}
			}
		}
	}