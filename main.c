#include "MKL46Z4.h"
#include "board.h"
#include "pin_mux.h"
#include "fsl_device_registers.h"
#include "fsl_debug_console.h"

#define GREEN_LED_PIN (1U << 5)
#define RED_LED_PIN (1U << 29)
#define SW1_PIN (1U << 3)
#define SW2_PIN (1U << 12)

volatile int32_t msTicks = 0; 
volatile int32_t sw2Pressed = 0; 
volatile int32_t sw1Pressed = 0;
void InitLed() {
      SIM->SCGC5 |= SIM_SCGC5_PORTE_MASK;
    PORTE->PCR[29] = PORT_PCR_MUX(1u);
    PTE->PDDR |= RED_LED_PIN;
    PTE->PSOR |= RED_LED_PIN;

    SIM->SCGC5 |= SIM_SCGC5_PORTD_MASK;
    PORTD->PCR[5] = PORT_PCR_MUX(1u);
    PTD->PDDR |= GREEN_LED_PIN;
    PTD->PSOR |= GREEN_LED_PIN;
}

void InitSwitches() {
	    SIM->SCGC5 |= SIM_SCGC5_PORTC_MASK;

   PORTC->PCR[3] = PORT_PCR_MUX(1) | PORT_PCR_PE_MASK | PORT_PCR_PS_MASK | PORT_PCR_IRQC(0xA);
    PTC->PDDR &= ~SW1_PIN;

    PORTC->PCR[12] = PORT_PCR_MUX(1) | PORT_PCR_PE_MASK | PORT_PCR_PS_MASK | PORT_PCR_IRQC(0xA);
    PTC->PDDR &= ~SW2_PIN;

    NVIC_EnableIRQ(PORTC_PORTD_IRQn);
}

void PORTC_PORTD_IRQHandler(void) {
    if (!(PTC->PDIR & SW1_PIN)) {
        sw1Pressed++;
        sw1Pressed = sw1Pressed % 2;
        msTicks = 0;
    
        PORTC->PCR[3] |= PORT_PCR_ISF_MASK;
    }
    if (!(PTC->PDIR & SW2_PIN)) {
        sw2Pressed++;
        sw2Pressed = sw2Pressed % 2;
        msTicks = 0;
      
        PORTC->PCR[12] |= PORT_PCR_ISF_MASK;
    }
}


void init_systick_interrupt() {
   SysTick->LOAD = SystemCoreClock / 1000; 
	SysTick->CTRL = SysTick_CTRL_CLKSOURCE_Msk |SysTick_CTRL_TICKINT_Msk |SysTick_CTRL_ENABLE_Msk;
}

void SysTick_Handler(void) { 
    msTicks++; 
}

void Delay(uint32_t TICK) {
    uint32_t startTicks = msTicks; 
    while ((msTicks - startTicks) < TICK);
}
enum State {
    CASE_1,//KHACH NGOI VAO CHO
    CASE_2,//KHACH DA NGOI MOT THOI GIAN MA KHONG THAT DAY
    CASE_3,//KHI NHAN DUOC THONG BAO KHACH THAT DAY
	  CASE_4,//KHI KHAC LEN XE VA THAT DAY LUON
    DEFAULT_STATE
};

int main(void) {
	  enum State currentState = DEFAULT_STATE  ;
	  BOARD_InitPins();
		BOARD_BootClockRUN();
		BOARD_InitDebugConsole();			
		SegLCD_Init();		
    SystemInit();
    InitLed();
    InitSwitches();
    init_systick_interrupt();
	bool seatOccupied = false;
bool beltFastened = false;
uint32_t timeSeatOccupied = 0;
		SegLCD_DisplayDecimal(0000);
    while (1) {
     
			  if ((sw1Pressed == 1) && (msTicks < 3000)) {
        if (sw2Pressed == 0) {
            currentState = CASE_1;
        } else {
            currentState = CASE_3;
        }
    } else if ((sw1Pressed == 1) && (msTicks >= 3000)) {
        if (sw2Pressed == 0) {
            currentState = CASE_2;
        } else {
            currentState = CASE_4;
        }
    }
				
				
				switch (currentState) {
            case CASE_1:
                PTD->PCOR |= GREEN_LED_PIN; 
                SegLCD_DisplayDecimal(0001);
                break;
            case CASE_2:
                PTD->PSOR |= GREEN_LED_PIN; 
                PTE->PCOR |= RED_LED_PIN; 
                SegLCD_DisplayDecimal(1111);
                break;
						case CASE_4:
                PTD->PCOR |= GREEN_LED_PIN; 
                PTE->PSOR |= RED_LED_PIN;  
                SegLCD_DisplayDecimal(1001);
                break;
            case CASE_3:
             
                SegLCD_DisplayDecimal(1001);
                break;
            default:
                PTE->PSOR |= RED_LED_PIN;
                PTD->PSOR |= GREEN_LED_PIN;
                break;
        }
			}
		}