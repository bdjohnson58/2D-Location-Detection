//includes for the project
#include "stm32F30x.h"
#include "STM32f3_discovery.h"
#include "stm32f30x_gpio.h"
#include  "stm32f30x_i2c.h"
#include "main.h"
#include  "stdio.h"
#include  "string.h"
#include "uart_com_main.h"


RCC_ClocksTypeDef 			RCC_Clocks;
GPIO_InitTypeDef        GPIO_InitStructure;


void    IO_Init(void);
void    I2C_init(void);
void    I2C2_init(void);
void    USART_Config(void);

void  LCD_write(int,int, char);
void  LCD_clear(void);
void  LCD_contrast(int);                // Contrast level = 1..50
void  LCD_backlight(int);               // Backlight level = 1..8
void Delay(uint32_t nTime);

unsigned  int   tick_count = 0;
unsigned int   distance = 0;
unsigned int t_stop = 0;
char strDisp[20] ;  
volatile int     ButtonPressed = 0;
volatile int     SignalIn = 0;
volatile int     RFSignalIn = 0;
static __IO uint32_t TimingDelay;

char        message[16];
char        message2[16];
char        ready_msg[] =   "    Waiting...  ";
char        signal_msg[] =  "Signal Recieved!";
char        sent_msg[] =    "   Ultrasonic   ";
char        sent_msg2[] =   "Signal Recieved!";

//RFT variables
int RecievedData(void);
uint16_t TxBuffer = 0xFF;
uint16_t RxBuffer;
int j;
uint16_t tmp;
uint16_t cmp = 0x000F;
uint8_t Serial_GetByte(USART_TypeDef *USARTx);


int main(void) {

    int   i, match = 0;
    
    IO_Init();
		USART_Config();
           
    I2C2_init();  
    while(I2C_GetFlagStatus(I2C2, I2C_ISR_BUSY) != RESET);
	
		LCD_contrast(50);
		LCD_backlight(8);
		
		LCD_clear();
		
		SysTick_Config(SystemCoreClock / 1000);      /* Configure SysTick to generate an interrupt every millisecond */
			
    while(1){
			for (i=0; i < strlen(ready_msg); i++)  //Display "Waiting"
        LCD_write(0, i, ready_msg[i]);                 
			
			while(match == 0){     //wait for RF signal to to tell it to start clock
									
				if(GPIO_ReadInputDataBit(GPIOD, GPIO_Pin_11)!=RESET){ //Wait for ERIC to detect inbound Signal through carrier detection (PD11)
					if(Serial_GetByte(USART3)==0xFF)
						match = 1;
				}
			}
					
			
			tick_count = 0;
			
			while (!ButtonPressed && GPIO_ReadInputDataBit(GPIOD, GPIO_Pin_10)==RESET ){   //while US signal has not been recieved, display counting clock
						sprintf(message, "time(ms) = %5i", tick_count);
						for (i=0; i < strlen(message); i++)
								LCD_write(0, i, message[i]);				// Display number on 1st line of display
			}
			
			
			t_stop = tick_count; //store time when US Signal is recieved
      ButtonPressed = 0;    
			
      LCD_clear();
			
			    
			distance = t_stop * 1.12533;    //calculate distance in ft (1.12533 ft/ms) .
			
			for (i=0; i < strlen(sent_msg); i++)  //Display "Ultrasonic Signal Recieved"
        LCD_write(0, i, sent_msg[i]);  
			
			for (i=0; i < strlen(sent_msg); i++)  
        LCD_write(1, i, sent_msg2[i]);  
			
			Delay(1000);   // Display Message for 1 second
			
			LCD_clear();
			
			sprintf(message, "t(ms) = %5i", t_stop);
			sprintf(message2, "d(ft)  = %5i", distance);
			for (i=0; i < strlen(message); i++){   //Display amount of time it took and distance 
          LCD_write(0, i, message[i]);				// Display number on 1st line of display
          LCD_write(1, i, message2[i]); 			
			}
					
			while (!ButtonPressed); //wait for button press and start over when hit
      ButtonPressed = 0;
			tick_count = 0;
			LCD_clear();
			
			
		}



}

//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~       


void IO_Init() {
  
		EXTI_InitTypeDef   EXTI_InitStructure;
		GPIO_InitTypeDef   GPIO_InitStructure;
		NVIC_InitTypeDef   NVIC_InitStructure;
			 
			
		/* SysTick end of count event each 1ms */
		RCC_GetClocksFreq(&RCC_Clocks);
		SysTick_Config(RCC_Clocks.HCLK_Frequency / 1000);

		/* Enable GPIOA clock */
		RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOA, ENABLE);

		/* Configure PA0 pin as input floating */
		GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN;
		GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
		GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0;
		GPIO_Init(GPIOA, &GPIO_InitStructure);

		/* Enable SYSCFG clock */
		RCC_APB2PeriphClockCmd(RCC_APB2Periph_SYSCFG, ENABLE);

		/* Connect EXTI0 Line to PA0 pin */
		SYSCFG_EXTILineConfig(EXTI_PortSourceGPIOA, EXTI_PinSource0);

		/* Configure EXTI0 line */
		EXTI_InitStructure.EXTI_Line = EXTI_Line0;
		EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;
		EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Rising;
		EXTI_InitStructure.EXTI_LineCmd = ENABLE;
		EXTI_Init(&EXTI_InitStructure);

		/* Enable and set EXTI0 Interrupt to the lowest priority */
		NVIC_InitStructure.NVIC_IRQChannel = EXTI0_IRQn;
		NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0x0F;
		NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0x0F;
		NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
		NVIC_Init(&NVIC_InitStructure);
		
		// External Interrupt 1 for recieveing Ultrasonic Signal PD10 **********
		
		/* Enable GPIOD clock */
		RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOD, ENABLE);
		
		/* Configure PD10 pin as input floating */
		GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN;
		GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_DOWN;
		GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10;
		GPIO_Init(GPIOD, &GPIO_InitStructure);

		/* Enable SYSCFG clock */
		RCC_APB2PeriphClockCmd(RCC_APB2Periph_SYSCFG, ENABLE);

		
		// External Interrupt 3 for recieveing RF Signal PD11 **********
		
		/* Enable GPIOD clock */
		RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOD, ENABLE);
		
		/* Configure PD11 pin as input floating */
		GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN;
		GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_DOWN;
		GPIO_InitStructure.GPIO_Pin = GPIO_Pin_11;
		GPIO_Init(GPIOD, &GPIO_InitStructure);

		/* Enable SYSCFG clock */
		RCC_APB2PeriphClockCmd(RCC_APB2Periph_SYSCFG, ENABLE);

		/* Connect EXTI3 Line to PD11 pin */
		SYSCFG_EXTILineConfig(EXTI_PortSourceGPIOD, EXTI_PinSource11);

		/* Configure EXTI3 line */
		EXTI_InitStructure.EXTI_Line = EXTI_Line3;
		EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;
		EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Rising;
		EXTI_InitStructure.EXTI_LineCmd = ENABLE;
		EXTI_Init(&EXTI_InitStructure);

		/* Enable and set EXTI3 Interrupt to the 3nd lowest priority */
		NVIC_InitStructure.NVIC_IRQChannel = EXTI3_IRQn;
		NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0x0D;
		NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0x0D;
		NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
		NVIC_Init(&NVIC_InitStructure);

}

/* UART Config to set RF transciever   */
static void USART_Config(void)
{
	USART_InitTypeDef USART_InitStructure;
  GPIO_InitTypeDef GPIO_InitStructure;
 
	/* Enable USART clock */
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART3, ENABLE);
	
  /* Enable GPIO clock */
  RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOB, ENABLE);
  
  /* Configure USART Tx Rx CTS RTS as alternate function push-pull */
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10;
  GPIO_Init(GPIOB, &GPIO_InitStructure);
  
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_11 | GPIO_Pin_13 | GPIO_Pin_14;
  GPIO_Init(GPIOB, &GPIO_InitStructure);

	/* Connect PB10 to USARTx_Tx */
  GPIO_PinAFConfig(GPIOB, GPIO_PinSource10, GPIO_AF_7);
  
  /* Connect PB11 to USARTx_Rx */
  GPIO_PinAFConfig(GPIOB, GPIO_PinSource11, GPIO_AF_7);
	
	 /* Connect PB13 to USARTx_CTS*/
  GPIO_PinAFConfig(GPIOB, GPIO_PinSource13, GPIO_AF_7);
	
		 /* Connect PB14 to USARTx_RTS*/
  GPIO_PinAFConfig(GPIOB, GPIO_PinSource14, GPIO_AF_7);
	
	/* USARTx configured as follow:
        - BaudRate = 115200 baud  
        - Word Length = 8 Bits
        - One Stop Bit
        - No parity
        - Hardware flow control CTS enabled 
        - Receive and transmit enabled
  */
	
  USART_InitStructure.USART_BaudRate = 9600;
  USART_InitStructure.USART_WordLength = USART_WordLength_8b;
  USART_InitStructure.USART_StopBits = USART_StopBits_1;
  USART_InitStructure.USART_Parity = USART_Parity_No;
  USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
  USART_InitStructure.USART_Mode = USART_Mode_Rx;
	
	USART_Init(USART3, &USART_InitStructure);
	USART_Cmd(USART3, ENABLE);
	
}

/**
**===========================================================================
**
**  Abstract: Get_Byte (blocking mode)
**
**===========================================================================
*/
 
uint8_t Serial_GetByte(USART_TypeDef *USARTx)
{
    while (USART_GetFlagStatus(USARTx, USART_FLAG_RXNE) == RESET);
    return USART_ReceiveData(USARTx);
}

int RecievedData(void)
{
	// Wait until host is ready to recieve data from transciever
	while (USART_GetFlagStatus(USART3, USART_FLAG_BUSY) == SET)
	{}
	
	j=0;

  /* Loop until read data register is empty */
  while (USART_GetFlagStatus(USART3, USART_FLAG_RXNE) == RESET)
  {
			tmp = USART_ReceiveData(USART3);	
	}
	
	if(tmp != cmp)
		 return 0;
	else
		 return 1;
}

void I2C2_init(void)
{
		GPIO_InitTypeDef GPIO_InitStructure;
		I2C_InitTypeDef  I2C_InitStructure;

		RCC_I2CCLKConfig(RCC_I2C2CLK_SYSCLK);  

		RCC_APB1PeriphClockCmd(RCC_APB1Periph_I2C2, ENABLE);  

		RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOA, ENABLE);
		GPIO_PinAFConfig(GPIOA, GPIO_PinSource9, GPIO_AF_4);
		GPIO_PinAFConfig(GPIOA, GPIO_PinSource10, GPIO_AF_4);
		GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
		GPIO_InitStructure.GPIO_OType = GPIO_OType_OD;
		GPIO_InitStructure.GPIO_PuPd  = GPIO_PuPd_NOPULL;
		GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;

		GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9;
		GPIO_Init(GPIOA, &GPIO_InitStructure);

		GPIO_InitStructure.GPIO_Pin =  GPIO_Pin_10;
		GPIO_Init(GPIOA, &GPIO_InitStructure);

		I2C_DeInit(I2C2);
		I2C_InitStructure.I2C_Mode = I2C_Mode_I2C;  
		I2C_InitStructure.I2C_AnalogFilter = I2C_AnalogFilter_Enable;
		I2C_InitStructure.I2C_DigitalFilter = 0x00;
		I2C_InitStructure.I2C_Ack = I2C_Ack_Enable;
		I2C_InitStructure.I2C_AcknowledgedAddress = I2C_AcknowledgedAddress_7bit;
		I2C_InitStructure.I2C_Timing = 0xC062121F; 

		I2C_Init(I2C2, &I2C_InitStructure);  
		I2C_Cmd(I2C2, ENABLE);
}



void  LCD_write(int row, int col, char data) {
          
		// Move to sepcified row, col
			
		I2C_TransferHandling(I2C2, 0x50 , 3, I2C_SoftEnd_Mode, I2C_Generate_Start_Write);

		while(I2C_GetFlagStatus(I2C2, I2C_ISR_TXIS) == RESET);

		I2C_SendData(I2C2, 0xFE);
					 
		while(I2C_GetFlagStatus(I2C2, I2C_ISR_TXIS) == RESET);

		I2C_SendData(I2C2, 0x45);

		while(I2C_GetFlagStatus(I2C2, I2C_ISR_TXIS) == RESET);        
		if (!row)               // if row == 0
				I2C_SendData(I2C2, col);
		 else                  // else row asumed to be 1
				I2C_SendData(I2C2, (0x40 + col));       
		 
		I2C_TransferHandling(I2C2, 0x50 , 1, I2C_SoftEnd_Mode, I2C_Generate_Start_Write);
			
		while(I2C_GetFlagStatus(I2C2, I2C_ISR_TXIS) == RESET);                        
		I2C_SendData(I2C2, data);

} 


//
//      Set LCD Contrast - Level should be 1..50 (Seems to work best if > 35)
//

void  LCD_contrast(int level) {
     
		I2C_TransferHandling(I2C2, 0x50 , 3, I2C_SoftEnd_Mode, I2C_Generate_Start_Write);

		while(I2C_GetFlagStatus(I2C2, I2C_ISR_TXIS) == RESET);

		I2C_SendData(I2C2, 0xFE);
					 
		while(I2C_GetFlagStatus(I2C2, I2C_ISR_TXIS) == RESET);

		I2C_SendData(I2C2, 0x52);

		while(I2C_GetFlagStatus(I2C2, I2C_ISR_TXIS) == RESET);

		I2C_SendData(I2C2, level); 

		Delay(20);
}         

//
//      Set LCD Backlight - Level should be 1..8 (Seems to work best if > 1)
//

void  LCD_backlight(int level) {
  

		I2C_TransferHandling(I2C2, 0x50 , 3, I2C_SoftEnd_Mode, I2C_Generate_Start_Write);

		while(I2C_GetFlagStatus(I2C2, I2C_ISR_TXIS) == RESET);

		I2C_SendData(I2C2, 0xFE);
				 
		while(I2C_GetFlagStatus(I2C2, I2C_ISR_TXIS) == RESET);

		I2C_SendData(I2C2, 0x53);

		while(I2C_GetFlagStatus(I2C2, I2C_ISR_TXIS) == RESET);

		I2C_SendData(I2C2, level);
			
		Delay(20);
}         


void  LCD_clear() {
       
		I2C_TransferHandling(I2C2, 0x50 , 2, I2C_SoftEnd_Mode, I2C_Generate_Start_Write);

		while(I2C_GetFlagStatus(I2C2, I2C_ISR_TXIS) == RESET);

		I2C_SendData(I2C2, 0xFE);
					 
		while(I2C_GetFlagStatus(I2C2, I2C_ISR_TXIS) == RESET);

		I2C_SendData(I2C2, 0x51);

		Delay(20);
}         

/**
  * @brief  Inserts a delay time.
  * @param  nTime: specifies the delay time length, in milliseconds.
  * @retval None
  */
void Delay(uint32_t nTime)
{ 
  TimingDelay = nTime;

  while(TimingDelay != 0);
}

/**
  * @brief  Decrements the TimingDelay variable.
  * @param  None
  * @retval None
  */
void TimingDelay_Decrement(void)
{
  if (TimingDelay != 0x00)
  { 
    TimingDelay--;
  }
  
}


void SysTick_Handler(void)
{
     TimingDelay_Decrement();
     
     tick_count++;
}


void EXTI0_IRQHandler(void)
{

	if ((EXTI_GetITStatus(USER_BUTTON_EXTI_LINE) == SET)&&(STM_EVAL_PBGetState(BUTTON_USER) != RESET))
  {
     
    ButtonPressed = 1;
		
    /* Clear the EXTI line 0 pending bit */
    EXTI_ClearITPendingBit(USER_BUTTON_EXTI_LINE);
  }    

}

void EXTI3_IRQHandler(void)
{

	if ((EXTI_GetITStatus(EXTI_Line3) == SET)&&(GPIO_ReadInputDataBit(GPIOD, GPIO_Pin_11) == SET))
  {
     
    RFSignalIn = 1;
		
    /* Clear the EXTI line 3 pending bit */
    EXTI_ClearITPendingBit(EXTI_Line3);
  }    

}
