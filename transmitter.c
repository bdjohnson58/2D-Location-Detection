//includes for the project
#include "stm32f30x.h"
#include "STM32f3_discovery.h"
#include "main.h"
#include  "stdio.h"
#include  "string.h"
	
RCC_ClocksTypeDef 			RCC_Clocks;
GPIO_InitTypeDef        GPIO_InitStructure;

void    IO_Init(void);
static void USART_Config(void);
void TransmitData(void);
	
/*void    I2C_init(void);
void    I2C2_init(void); */

int InitPwmSignal(int pwm_freq);
void InitPwmGpio(void);
void Serial_PutByte(USART_TypeDef *USARTx, uint8_t byte);
void Serial_SendPacket(USART_TypeDef *USARTx, uint8_t *data, uint16_t length);
void Serial_PutString(USART_TypeDef *USARTx, char *p_string);

/*void  LCD_write(int,int, char) ;
void  LCD_clear(void);
void  LCD_contrast(int);                // Contrast level = 1..50
void  LCD_backlight(int);     */          // Backlight level = 1..8
void Delay(uint32_t nTime);

unsigned  int   tick_count = 0;
unsigned int   distance = 0;
unsigned int t_stop = 0;
char strDisp[20] ;  
volatile int     ButtonPressed = 0;
static __IO uint32_t TimingDelay;
uint8_t ch;
int   i;

/*char        message[16];
char        message2[16];
char        ready_msg[] = " Press to Send  ";
char        sent_msg[] =  "   Ultrasonic   ";
char        sent_msg2[] = "  Signal Sent!  "; */


int main(void) {

    
		int pulse_width = 25;
		int pwm_freq = 40000;
    
    IO_Init();
		InitPwmGpio();
		InitPwmSignal(pwm_freq);
		USART_Config();
           
    /*I2C2_init(); 	
    while(I2C_GetFlagStatus(I2C2, I2C_ISR_BUSY) != RESET);
	
		LCD_contrast(50);
		LCD_backlight(8);
		
		LCD_clear();*/
	
    while(1){
			
			TIM_SetCompare1(TIM3, 0);
			/*for (i=0; i < strlen(ready_msg); i++)  //Display "Press to Send"
        LCD_write(0, i, ready_msg[i]);*/

			while (!ButtonPressed);                //Wait for button press
			ButtonPressed = 0;
			
			//TransmitData();
			//while (USART_GetFlagStatus(USART3, USART_FLAG_CTS) == SET){}	
			Serial_PutByte(USART3, 0xFF);
			TIM_SetCompare1(TIM3, pulse_width);    //Set w pulse width at 25 = 50% duty cycle  (send signal)
			GPIOE->ODR = 0xFF00;
			
			Delay(1000);  			         // Send US signal for 1 second
			
			TIM_SetCompare1(TIM3, 0); //Set pulse to 0    (Stop sending signal)
			
			GPIOE->ODR = 0x0000; // 
			
			//LCD_clear();
			
			/*for (i=0; i < strlen(sent_msg); i++)  //Display "Ultrasonic Signal Sent"
        LCD_write(0, i, sent_msg[i]);  
			
			for (i=0; i < strlen(sent_msg); i++)  
        LCD_write(1, i, sent_msg2[i]);  */
			
			//Delay(2000);   // Display Message for 2 seconds
			
			//LCD_clear();
			
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


		/* GPIOE Periph clock enable */
		RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOE, ENABLE);

		/* Configure PE14 and PE15 in output pushpull mode */
		GPIO_InitStructure.GPIO_Pin = GPIO_Pin_15 | GPIO_Pin_14 | GPIO_Pin_13 | GPIO_Pin_12
			| GPIO_Pin_11| GPIO_Pin_10| GPIO_Pin_9| GPIO_Pin_8;
		GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;              
		GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
		GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
		GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;             //GPIO_PuPd_NOPULL
		GPIO_Init(GPIOE, &GPIO_InitStructure);

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

}



/*void I2C2_init(void)
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
}*/


void InitPwmGpio()
{
     
    GPIO_InitTypeDef GPIO_InitStructure;
    RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOA, ENABLE);
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF; // Use the alternative pin functions 
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;  // GPIO speed - has nothing to do with the timer timing 
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP; // Push-pull 
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_DOWN; // Setup pull-down resistors 
    GPIO_Init(GPIOA, &GPIO_InitStructure);
 
    // Connect the timer output to the pins
 
    GPIO_PinAFConfig(GPIOA, GPIO_PinSource6, GPIO_AF_2); // TIM3_CH1 -> A6 

}

int InitPwmSignal(int pwm_freq)
{
    TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
    TIM_OCInitTypeDef  TIM_OCInitStructure;

    double ms_pulses;
    
     //Calculates the timing. This is common for all channels 
    int clk = 72e6; 		// 72MHz -> system core clock. Default on the stm32f3 discovery
    //int clk = 36e6; 		// (APB1 max) 36MHz. Default on the stm32f3 discovery
	
    int tim_freq = 2e6; 	// in Hz (2MHz) Base frequency of the pwm timer
	
    int prescaler = ( (clk / tim_freq) - 1 );
 
    // Calculate the period for a given pwm frequency
    int pwm_period = tim_freq / pwm_freq;       // 2MHz / 40KHz = 50 
                             
 
    // Calculate a number of pulses per millisecond.
    ms_pulses = (float)pwm_period / ( 1000.0 / pwm_freq );  
 
 
    //  Enable the TIM4 peripheral
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3 , ENABLE );
 
    // Setup the timing and configure the TIM4 timer
    TIM_TimeBaseStructInit(& TIM_TimeBaseStructure);
    TIM_TimeBaseStructure.TIM_Prescaler = prescaler;
    TIM_TimeBaseStructure.TIM_Period = pwm_period - 1;
    TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1; 
    TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up ;
    TIM_TimeBaseInit(TIM3, &TIM_TimeBaseStructure);
 
 
    // Initialize the OC for PWM
    TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;
    TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
    TIM_OCInitStructure.TIM_Pulse = (int)(ms_pulses*10); 	// preset pulse width 1 ms
    TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High; 	// Pulse polarity
 
    // Setup channels 
    // Channel 1 (PE2)
    TIM_OC1Init(TIM3, &TIM_OCInitStructure);
    TIM_OC1PreloadConfig(TIM3, TIM_OCPreload_Enable);

 
    // Start the timer 
    TIM_Cmd(TIM3 , ENABLE);
 
    return pwm_period;
} 

/* UART Config to set RF transciever   */
static void USART_Config(void)
{
	USART_InitTypeDef USART_InitStructure;
  GPIO_InitTypeDef GPIO_InitStructure;
	
	/* Enable USART clock */
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART3, ENABLE);
	
  /* Enable GPIO clock */
  RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOD, ENABLE);
  
  /* Configure USART Tx Rx CTS and RTS as alternate function push-pull */
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_8 | GPIO_Pin_9 | GPIO_Pin_11 | GPIO_Pin_12;
  GPIO_Init(GPIOD, &GPIO_InitStructure);
	
	/* Connect PD8 to USARTx_Tx */
  GPIO_PinAFConfig(GPIOD, GPIO_PinSource8, GPIO_AF_7);
  
  /* Connect PD9 to USARTx_Rx */
  GPIO_PinAFConfig(GPIOD, GPIO_PinSource9, GPIO_AF_7);
	
	/* Connect PD11 to USARTx_CTS*/
  GPIO_PinAFConfig(GPIOD, GPIO_PinSource11, GPIO_AF_7);
	
  /* Connect PD12 to USARTx_RTS*/
  GPIO_PinAFConfig(GPIOD, GPIO_PinSource12, GPIO_AF_7);
	
	/* USARTx configured as follow:
        - BaudRate = 115200 baud  
        - Word Length = 8 Bits
        - One Stop Bit
        - No parity
        - Hardware flow control RTS and CTS enabled 
        - Receive and transmit enabled
  */
  USART_InitStructure.USART_BaudRate = 9600;
  USART_InitStructure.USART_WordLength = USART_WordLength_8b;
  USART_InitStructure.USART_StopBits = USART_StopBits_1;
  USART_InitStructure.USART_Parity = USART_Parity_No;
  USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
  USART_InitStructure.USART_Mode = USART_Mode_Tx;
	
	USART_Init(USART3, &USART_InitStructure);
	USART_Cmd(USART3, ENABLE);
	

}

/**
**===========================================================================
**
**  Abstract: Send_Byte (blocking mode)
**
**===========================================================================
*/
void Serial_PutByte(USART_TypeDef *USARTx, uint8_t byte)
{
    while (USART_GetFlagStatus(USARTx, USART_FLAG_TXE) == RESET);
    USART_SendData(USARTx, byte);
}
/**
**===========================================================================
**
**  Abstract: SendPacket
**
**===========================================================================
*/
void Serial_SendPacket(USART_TypeDef *USARTx, uint8_t *data, uint16_t length)
{
    uint16_t i;
    i = 0;
    while (i < length)
    {
        Serial_PutByte(USARTx, data[i]);
        i++;
    }
}
/**
**===========================================================================
**
**  Abstract: SendString
**
**===========================================================================
*/
void Serial_PutString(USART_TypeDef *USARTx, char *p_string)
{
    uint16_t length = 0;
 
    while (p_string[length] != '\0')
    {
        Serial_PutByte(USARTx, p_string[length]);
        length++;
    }
}

void TransmitData(void)
{

	while (USART_GetFlagStatus(USART3, USART_FLAG_CTS) == SET){}	
	Serial_PutString(USART3, "Start Clock\r\n");
	
}





/*void  LCD_write(int row, int col, char data) {
          
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
}  */    

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
		
		//GPIOE->ODR = 0xFF00;
     
    ButtonPressed = 1;
		
    /* Clear the EXTI line 0 pending bit */
    EXTI_ClearITPendingBit(USER_BUTTON_EXTI_LINE);
  }    

}

