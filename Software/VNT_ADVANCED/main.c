
/*  4 ventilation battery rotation

    To test the functionality 

	Set `test coil flag` in the Modbus coils no. 3.
	This is the one that modify the time of the ventilation block sorting
	from 10 hours to 30 seconds, so we can test the ventilation block rotation,
	and it will also enable the update of the oil temperature value from
	the third Modbus Hoding Register
	
		TEST_COIL_FLAG = getCoil(3);

	If flag is set then we use oil temperature from third Modbus register
	so we can test ventilation steps (watch out, the ventilation blocks change is at 30 seconds !!!)
		if (TEST_COIL_FLAG == 1)
			OILTEMP = (float)(readHoldingRegister(3));
*/

// If we need to store signed values we use 2's complement
// int16_t val = -100;
// uint16_t number = (uint16_t) val


#include "stm32f10x.h"
#include "main.h"
#include "mbutils.h"
#include "mb.h"


volatile uint8_t sec = 0;
volatile uint8_t min = 0;
volatile uint8_t hour = 0;

volatile uint32_t TimingDelay;

volatile uint8_t GR[4];
volatile uint8_t GR1_STATE_FLAG;
volatile uint8_t GR2_STATE_FLAG;
volatile uint8_t GR3_STATE_FLAG;
volatile uint8_t GR4_STATE_FLAG;
volatile uint8_t TEST_COIL_FLAG = 0;

volatile uint8_t Modbus_Request_Flag;

volatile uint8_t dayChangeFlag = 0;



#define GR1ON    GPIO_SetBits(GPIOB, GPIO_Pin_5)
#define GR1OFF   GPIO_ResetBits(GPIOB, GPIO_Pin_5)

#define GR2ON    GPIO_SetBits(GPIOB, GPIO_Pin_6)
#define GR2OFF   GPIO_ResetBits(GPIOB, GPIO_Pin_6)

#define GR3ON    GPIO_SetBits(GPIOB, GPIO_Pin_7)
#define GR3OFF   GPIO_ResetBits(GPIOB, GPIO_Pin_7)

#define GR4ON    GPIO_SetBits(GPIOB, GPIO_Pin_8)
#define GR4OFF   GPIO_ResetBits(GPIOB, GPIO_Pin_8)


// dec -> hex, hex -> dec
#define TO_HEX(i) (i <= 9 ? '0' + i : 'A' - 10 + i)
#define TO_DEC(i) (i <= '9'? i - '0': i - 'A' + 10)
#define CHECK_BIT(var,pos) ((var) & (1<<(pos)))

void Delay(__IO uint32_t nTime);

u16 usRegHoldingBuf[100+1]; // 0..99 holding registers
u8  usRegCoilBuf[64/8+1];   // 1..64  coils


void USART3_Putch(unsigned char ch);
void USART3_Print(char s[]);
void USART3_Print_Int(int number);
void USART3_Init(void);

void START_VENTILATION_BLOCK(uint8_t number);
void STOP_VENTILATION_BLOCK(uint8_t number);

void ADC_Configuration(void);
u16 readADC1(u8 channel);

void writeCoil(uint8_t coil_index, uint8_t state)
{
    uint8_t coil_offset=coil_index/8;
    if (state == 1)
        usRegCoilBuf[coil_offset] |= (1<<(coil_index%8));
    else usRegCoilBuf[coil_offset] &= ~(1<<(coil_index%8));
}

uint8_t getCoil(uint8_t coil_index)
{
    uint8_t coil_byte=usRegCoilBuf[coil_index/8];
    if (coil_byte & (1<<(coil_index%8))) return 1;
    else return 0;
}

void writeHoldingRegister(uint8_t reg_index, uint16_t reg_val)
{
    usRegHoldingBuf[reg_index] = reg_val;
}

uint16_t readHoldingRegister(uint8_t reg_index)
{
    return usRegHoldingBuf[reg_index];
}




int main(void)
{
    GPIO_InitTypeDef GPIO_InitStructure;

    uint8_t data_gathering_active_flag;
    int8_t working_counter;

    uint8_t co_index, hr_index;

    uint8_t i;

    uint16_t adc_value;
    float voltage, current;

    uint8_t GR1, GR2, GR3, GR4;
    float STEP1_START, STEP1_STOP;
    float STEP2_START, STEP2_STOP;
    float STEP3_START, STEP3_STOP;
    float STEP4_START, STEP4_STOP;
    float OILTEMP;

    uint16_t SIMULATED_OIL_TEMPERATURE;

    // initialize variables used for ventilation blocks order
    GR[0] = 1;
    GR[1] = 2;
    GR[2] = 3;
    GR[3] = 4;

    // reset ventilation blocks state variables
    GR1_STATE_FLAG = 0;
    GR2_STATE_FLAG = 0;
    GR3_STATE_FLAG = 0;
    GR4_STATE_FLAG = 0;

    // ventilation steps initialization
    STEP1_START = 55.0;
    STEP2_START = 65.0;
    STEP3_START = 75.0;
    STEP4_START = 85.0;

    STEP1_STOP = 50.0;
    STEP2_STOP = 60.0;
    STEP3_STOP = 70.0;
    STEP4_STOP = 80.0;


    /*
     * Pozitia vectorilor de intrerupere a fost setata in fisierul system_stm32f10x.c la linia 128
     */
    /* Set the Vector Table base adress at 0x8004000 */
    // NVIC_SetVectorTable(NVIC_VectTab_FLASH, 0x4000);

    /* Setup SysTick Timer for 1 msec interrupts.
       ------------------------------------------
    1. The SysTick_Config() function is a CMSIS function which configure:
       - The SysTick Reload register with value passed as function parameter.
       - Configure the SysTick IRQ priority to the lowest value (0x0F).
       - Reset the SysTick Counter register.
       - Configure the SysTick Counter clock source to be Core Clock Source (HCLK).
       - Enable the SysTick Interrupt.
       - Start the SysTick Counter.

    2. You can change the SysTick Clock source to be HCLK_Div8 by calling the
       SysTick_CLKSourceConfig(SysTick_CLKSource_HCLK_Div8) just after the
       SysTick_Config() function call. The SysTick_CLKSourceConfig() is defined
       inside the misc.c file.

    3. You can change the SysTick IRQ priority by calling the
       NVIC_SetPriority(SysTick_IRQn,...) just after the SysTick_Config() function
       call. The NVIC_SetPriority() is defined inside the core_cm3.h file.

    4. To adjust the SysTick time base, use the following formula:

         Reload Value = SysTick Counter Clock (Hz) x  Desired Time base (s)

       - Reload Value is the parameter to be passed for SysTick_Config() function
       - Reload Value should not exceed 0xFFFFFF

			 SystemCoreClock / 1000 = 72000
    */
    if(SysTick_Config(72000))
    {
        /* Capture error */
        while (1);
    }


    /************************************************************
    *   RTC configuration
    *************************************************************/
    dayChangeFlag = 0;
    RTC_Configuration();
    BKP_WriteBackupRegister(BKP_DR1, 0xA5A5);
    /* NVIC configuration */
    NVIC_Configuration();
    /* Clear reset flags */
    RCC_ClearFlag();

    /************************************************************
    *   init PB0 & PB1 led
    *************************************************************/
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB , ENABLE);
    GPIO_StructInit(&GPIO_InitStructure);
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_14 | GPIO_Pin_1 | GPIO_Pin_0;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
    GPIO_Init(GPIOB, &GPIO_InitStructure);

    /************************************************************
    *   Init PB5, PB6, PB7, PB8 as controls for relays K1, K2, K3, K4
    *************************************************************/
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB , ENABLE);
    GPIO_StructInit(&GPIO_InitStructure);
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_5 | GPIO_Pin_6 | GPIO_Pin_7 | GPIO_Pin_8;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_OD;
    GPIO_Init(GPIOB, &GPIO_InitStructure);

    // Make a test of ventilation blocks each time we start

    // All OFF
    GR1OFF;
    GR2OFF;
    GR3OFF;
    GR4OFF;
    Delay(4000);

    // Start one by one
    GR1ON;
    Delay(4000);
    GR2ON;
    Delay(4000);
    GR3ON;
    Delay(4000);
    GR4ON;
    Delay(4000);

    // All OFF
    GR1OFF;
    GR2OFF;
    GR3OFF;
    GR4OFF;

    /*************************************************************
    *   Init USART3 peripheral
    *************************************************************/
    USART3_Init();
    Delay(500);
    USART3_Print("START");

    /* PCLK2 is the APB2 clock */
    /* ADCCLK = PCLK2/6 = 72/6 = 12MHz*/
    RCC_ADCCLKConfig(RCC_PCLK2_Div6);
    Delay(500);
    ADC_Configuration();

    // Initialize protocol stack in RTU mode for a slave with address 10 = 0x0A
	// MB_RTU, Device ID: 1, USART portL: 1 (este configurat in portserial.h, Baud rate: 19200, Parity: NONE)
    eMBInit(MB_RTU, 6, 1, 19200, MB_PAR_NONE);
	// Enable the Modbus Protocol Stack.
    eMBEnable();



    co_index = 1;
    hr_index = 2;
    working_counter = 0;
    data_gathering_active_flag = 0;

    while(1) {
        eMBPoll();

        // we have 2 modes:   1.send ID (data_gathering_active_flag = 0)
        //                    2.data gathering (data_gathering_active_flag = 1)
        // "if then else" is like this so when flag is active execute only "if" condition
        if (data_gathering_active_flag == 1) {

        	// we execute this block each loop if flag is active
            // but only for time set below using TimingDelay variable (100 ms)


            // if we have timeout condition deactivate this block and
            if (TimingDelay == 0) {
                data_gathering_active_flag = 0;
            }


        } else {
        	// activate data gathering mode
            data_gathering_active_flag = 1;

            // Blink led so we can see main loop working
        	GPIOB->ODR ^= GPIO_Pin_1;


        	// The ADC value is on 12 bits(4095), the pin is PA3
        	adc_value = 0;
        	// use average of 10 values for stability
            for (i = 0; i < 10; i++) adc_value += readADC1(ADC_Channel_3);
            // convert to voltage
            voltage = (adc_value/10.0) * (3.3/4095.0);
            // current in mA !!!
            current = (voltage / 120.0) * 1000.0;


            // store oil temperature input current to modbus as integer with
            // with 1 decimal precision
    		writeHoldingRegister(2, (uint16_t)(current * 10.0));


    		// temperature probe is from 0 .. 150 degrees
    		if ((current - 4.0) > 0.0)
    		    OILTEMP = (((current - 4.0) * 150.0) / 16.0);
    		else
				OILTEMP = 0.0;


            // how to compute the temperature value using 4-20mA current
    		// x = 20 => (((x-4)*150)/16) = 150
    	    // x = 4 =>  (((x-4)*150)/16) = 0


    		// update test coil flag from the third modbus coil register
    		// this is the one that modify the time of the ventilation block sorting
    		// from 10 hours to 30 seconds, so we can test the ventilation block rotation
    		// also it will enable the update of the oil temperature value from
    		// the third modbus holding register
    	    TEST_COIL_FLAG = getCoil(3);


    		// if flag is set then we use oil temperature from third modbus register
    	    // so we can test ventilation steps (watch out, the ventilation blocks change is at 30 seconds !!!)
    		if (TEST_COIL_FLAG == 1)
    			OILTEMP = (float)(readHoldingRegister(3));


    		// set the temporary ventilation block variables
			// if start
    		if (OILTEMP > STEP1_START) GR1 = 1;
			if (OILTEMP > STEP2_START) GR2 = 1;
			if (OILTEMP > STEP3_START) GR3 = 1;
			if (OILTEMP > STEP4_START) GR4 = 1;
			// if stop
			if (OILTEMP < STEP4_STOP) GR4 = 0;
			if (OILTEMP < STEP3_STOP) GR3 = 0;
			if (OILTEMP < STEP2_STOP) GR2 = 0;
			if (OILTEMP < STEP1_STOP) GR1 = 0;


            // activate time counters according to active ventilation block
			if (GR1 == 1) GR1_STATE_FLAG = 1;
            else GR1_STATE_FLAG = 0;
			if (GR2 == 1) GR2_STATE_FLAG = 1;
			else GR2_STATE_FLAG = 0;
			if (GR3 == 1) GR3_STATE_FLAG = 1;
			else GR3_STATE_FLAG = 0;
			if (GR4 == 1) GR4_STATE_FLAG = 1;
			else GR4_STATE_FLAG = 0;

			// avoid starting stopping ventilation blocks near
			// the moment when we sort ventilation blocks working time
            if ((sec != 59)&&(sec != 0)&&(sec != 1))
            {
				// activate relays based on refreshed state variables
				if (GR1 == 1) {
					START_VENTILATION_BLOCK(1);
				} else {
					STOP_VENTILATION_BLOCK(1);
				}
				if (GR2 == 1)
				{
					START_VENTILATION_BLOCK(2);
				} else {
					STOP_VENTILATION_BLOCK(2);
				}
				if (GR3 == 1)
				{
					START_VENTILATION_BLOCK(3);
				} else {
					STOP_VENTILATION_BLOCK(3);
				}
				if (GR4 == 1)
				{
					START_VENTILATION_BLOCK(4);
				} else {
					STOP_VENTILATION_BLOCK(4);
				}
            }

			if (working_counter++ == 10) working_counter = 0;
			writeHoldingRegister(3, working_counter);


            // set timeout
            TimingDelay = 500;

        }
    }
}


void START_VENTILATION_BLOCK(uint8_t number)
{
	// works only for good ventilation block number 1,2,3,4
	// the array is 0,1,2,3, that's why we do -1
    if ((number >= 1)&&(number <= 4)) {
        if (GR[number-1] == 1) GR1ON;
        if (GR[number-1] == 2) GR2ON;
        if (GR[number-1] == 3) GR3ON;
        if (GR[number-1] == 4) GR4ON;
	}
}

void STOP_VENTILATION_BLOCK(uint8_t number)
{
	// works only for good ventilation block number 1,2,3,4
	// the array is 0,1,2,3, that's why we do -1
	if ((number >= 1)&&(number <= 4)) {
        if (GR[number-1] == 1) GR1OFF;
        if (GR[number-1] == 2) GR2OFF;
        if (GR[number-1] == 3) GR3OFF;
        if (GR[number-1] == 4) GR4OFF;
	}
}

void Delay(volatile uint32_t nTime)
{
    TimingDelay = nTime;

    while(TimingDelay != 0);
}



void USART3_Putch(unsigned char ch)
{
	USART_SendData( USART3, ch);

	// Wait until the end of transmision
	while( USART_GetFlagStatus( USART3, USART_FLAG_TC) == RESET){}
}


void USART3_Print(char s[])
{
    int i=0;

    while( i < 64)
	{
	    if( s[i] == '\0') break;
        USART3_Putch( s[i++]);
    }
}


void USART3_Print_Int(int number)
{
	unsigned char s[5], i=1, j=0;

    if( number < 0)
    {
    	USART3_Putch( '-');
		number = -number;
	}

    while( number >= 10)
    {
	    s[i++] = number % 10;
	    number /= 10;
    }
    s[i] = number;
    j = i;
    for( i=j; i>0; i--) USART3_Putch( '0' + s[i]);
}


void USART3_Init(void)
{
    GPIO_InitTypeDef GPIO_InitStructure;
    USART_InitTypeDef USART_InitStructure;

    RCC_APB2PeriphClockCmd( RCC_APB2Periph_GPIOB,ENABLE);
	  RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO, ENABLE);
	  RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART3, ENABLE);

    // Configure USART3 Tx (PB.10) as alternate function push-pull
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
    GPIO_Init(GPIOB, &GPIO_InitStructure);

	  // Configure USART3 Rx (PB.11) as input floating
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_11;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOA, &GPIO_InitStructure);

	  // USART3 configuration
    USART_InitStructure.USART_BaudRate = 19200;
    USART_InitStructure.USART_WordLength = USART_WordLength_8b;
    USART_InitStructure.USART_StopBits = USART_StopBits_1;
    USART_InitStructure.USART_Parity = USART_Parity_No;
    USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
    USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;

    USART_Init(USART3, &USART_InitStructure);

    //USART_ITConfig(USART1, USART_IT_RXNE, ENABLE);
    //USART_ITConfig(USART1, USART_IT_TXE, ENABLE);
    USART_Cmd(USART3, ENABLE);
}


void ADC_Configuration(void)
{
    ADC_InitTypeDef ADC_InitStructure;

    /* PCLK2 is the APB2 clock */
    /* ADCCLK = PCLK2/6 = 72/6 = 12MHz*/
    RCC_ADCCLKConfig(RCC_PCLK2_Div6);

    /* Enable ADC1 clock so that we can talk to it */
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_ADC1, ENABLE);

    /* Put everything back to power-on defaults */
    ADC_DeInit(ADC1);

    /* ADC1 Configuration ------------------------------------------------------*/
    /* ADC1 and ADC2 operate independently */
    ADC_InitStructure.ADC_Mode = ADC_Mode_Independent;

    /* Disable the scan conversion so we do one at a time */
    ADC_InitStructure.ADC_ScanConvMode = DISABLE;

    /* Don't do contimuous conversions - do them on demand */
    ADC_InitStructure.ADC_ContinuousConvMode = DISABLE;

    /* Start conversin by software, not an external trigger */
    ADC_InitStructure.ADC_ExternalTrigConv = ADC_ExternalTrigConv_None;

    /* Conversions are 12 bit - put them in the lower 12 bits of the result */
    ADC_InitStructure.ADC_DataAlign = ADC_DataAlign_Right;

    /* Say how many channels would be used by the sequencer */
    ADC_InitStructure.ADC_NbrOfChannel = 1;

    /* Now do the setup */
    ADC_Init(ADC1, &ADC_InitStructure);

    /* Enable ADC1 */
    ADC_Cmd(ADC1, ENABLE);

    /* Enable ADC1 reset calibaration register */
    ADC_ResetCalibration(ADC1);

    /* Check the end of ADC1 reset calibration register */
    while(ADC_GetResetCalibrationStatus(ADC1));

    /* Start ADC1 calibaration */
    ADC_StartCalibration(ADC1);

    /* Check the end of ADC1 calibration */
    while(ADC_GetCalibrationStatus(ADC1));
}


u16 readADC1(u8 channel)
{
    /* Start the conversion */
    ADC_RegularChannelConfig(ADC1, channel, 1, ADC_SampleTime_1Cycles5);
    /* Wait until conversion completion */
    ADC_SoftwareStartConvCmd(ADC1, ENABLE);
    /* Get the conversion value */
    while(ADC_GetFlagStatus(ADC1, ADC_FLAG_EOC) == RESET);
    return ADC_GetConversionValue(ADC1);
}


void RTC_Configuration(void)
{
	/* Enable PWR and BKP clocks */
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_PWR | RCC_APB1Periph_BKP, ENABLE);

	/* Allow access to BKP Domain */
	PWR_BackupAccessCmd(ENABLE);

	/* Reset Backup Domain */
	BKP_DeInit();

	/* Enable LSE */
	RCC_LSEConfig(RCC_LSE_ON);
	/* Wait till LSE is ready */
	while (RCC_GetFlagStatus(RCC_FLAG_LSERDY) == RESET)
	{}

	/* Select LSE as RTC Clock Source */
	RCC_RTCCLKConfig(RCC_RTCCLKSource_LSE);

	/* Enable RTC Clock */
	RCC_RTCCLKCmd(ENABLE);

	/* Wait for RTC registers synchronization */
	RTC_WaitForSynchro();

	/* Wait until last write operation on RTC registers has finished */
	RTC_WaitForLastTask();

	/* Enable the RTC Second */
	RTC_ITConfig(RTC_IT_SEC, ENABLE);

	/* Wait until last write operation on RTC registers has finished */
	RTC_WaitForLastTask();

	/* Set RTC prescaler: set RTC period to 1sec */
	RTC_SetPrescaler(32767); /* RTC period = RTCCLK/RTC_PR = (32.768 KHz)/(32767+1) */

	/* Wait until last write operation on RTC registers has finished */
	RTC_WaitForLastTask();
}

void NVIC_Configuration(void)
{
	NVIC_InitTypeDef NVIC_InitStructure;

	/* Configure one bit for preemption priority */
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_1);

	/* Enable the RTC Interrupt */
	NVIC_InitStructure.NVIC_IRQChannel = RTC_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);
}


/*
 *     VARIANTA SIMPLA FARA PARTEA DE SORTARE
 *
			// activate relays based on refreshed state variables
			if (GR1 == 1) {
				GR1ON;
			} else {
				GR1OFF;
			}
			if (GR2 == 1)
			{
				GR2ON;
			} else {
                GR2OFF;
			}
			if (GR3 == 1)
			{
				GR3ON;
			} else {
				GR3OFF;
			}
			if (GR4 == 1)
			{
				GR4ON;
			} else {
				GR4OFF;
			}

 */




/*
    					//   We use this to reset hours counter
						//   We need to write the number 12358 to holding register 99
                        if (usRegHoldingBuf[99] == 12358)
						{
		    				//   Reset control code from modbus server
							daysCounter = 0;
  							writeHoldingRegister(98, 0);
                            writeHoldingRegister(99, 0);
		    				//   Reset value in flash
                            FLASH_ClearFlag(FLASH_FLAG_BSY | FLASH_FLAG_EOP | FLASH_FLAG_PGERR | FLASH_FLAG_WRPRTERR);
                            FLASH_Unlock();
                            FLASH_ErasePage(FLASH_ADR);   // we must erase the entire page before we can write
		                    FLASH_ProgramWord(FLASH_ADR, daysCounter);
                            FLASH_Lock();
						}

 */


/*
 * Lines used to test float values in the modbus server
 *
 * get's the float from position 3 and store it at position index + 4 = 7
 *
 * writeFloat2HoldingRegisters(3, 0.4956); // store a float in registers 2 & 3
 * float t = get_float(3);
 */
//writeFloat2HoldingRegisters(3, 0.4956); // store a float in registers 2 & 3
//writeFloat2HoldingRegisters(7, 49.315);  // store a float in registers 6 & 7



/* METODA DE DELAY us BAZATA PE DWT, PRIN DOUA METODE
 * UNA EXPLICATA BINE SI UNA OPTIMIZATA !!!
#if DWT_DELAY_NEWBIE

 // If you are a newbie and see magic in DWT_Delay, consider this more
 // illustrative function, where you explicitly determine a counter
 // value when delay should stop while keeping things in bounds of uint32.

void DWT_Delay(uint32_t us) // microseconds
{
    uint32_t startTick  = DWT->CYCCNT,
             targetTick = DWT->CYCCNT + us * (SystemCoreClock/1000000);

    // Must check if target tick is out of bounds and overflowed
    if (targetTick > startTick) {
        // Not overflowed
        while (DWT->CYCCNT < targetTick);
    } else {
        // Overflowed
        while (DWT->CYCCNT > startTick || DWT->CYCCNT < targetTick);
    }
}
#else

 // Delay routine itself.
 // Time is in microseconds (1/1000000th of a second), not to be
 // confused with millisecond (1/1000th).
 //
 // No need to check an overflow. Let it just tick :)
 //
 // @param uint32_t us  Number of microseconds to delay for

void DWT_Delay(uint32_t us) // microseconds
{
    uint32_t startTick = DWT->CYCCNT,
             delayTicks = us * (SystemCoreClock/1000000);

    while (DWT->CYCCNT - startTick < delayTicks);
}
*/





/*
// METODA DE DELAY BAZATA PE ASSEMBLER SI LOOP
#define delayUS_ASM(us) do {\
	asm volatile (	"MOV R0,%[loops]\n\t"\
			"1: \n\t"\
			"SUB R0, #1\n\t"\
			"CMP R0, #0\n\t"\
			"BNE 1b \n\t" : : [loops] "r" (16*us) : "memory"\
		      );\
} while(0)
*/





/*  VARIANTA VECHE FARA SEMNALIZARE DE EROARE SI MAI COMPLICATA
void DS18B20_PIN_Input (void)
{
    GPIO_InitTypeDef GPIO_InitStructure;

	GPIO_StructInit(&GPIO_InitStructure);
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_5;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING; // We have externall pull-up
	GPIO_Init(GPIOB, &GPIO_InitStructure);
}

void DS18B20_PIN_Output (void)
{
    GPIO_InitTypeDef GPIO_InitStructure;

    GPIO_StructInit(&GPIO_InitStructure);
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_5;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
    GPIO_Init(GPIOB, &GPIO_InitStructure);
}

uint8_t DS18B20_Init (void)
{
	DS18B20_PIN_Output();

	GPIOB->BRR = GPIO_Pin_5;   // pull the pin low

	DWT_Delay(480);   // delay according to datasheet

	DS18B20_PIN_Input();

	DWT_Delay(80);   // delay according to datasheet

	if (!(GPIOB->IDR & GPIO_Pin_5))   // if the pin is low i.e the presence pulse is there
	{
		DWT_Delay(400);
		return 0;
	} else {
        DWT_Delay(400);
		return 1;
	}
}

void DS18B20_Write (uint8_t data)
{
	DS18B20_PIN_Output();

	for (int i=0; i<8; i++) {
		if ((data & (1<<i)) != 0) {  // if the bit is high
			// write 1
			DS18B20_PIN_Output();
			GPIOB->BRR = GPIO_Pin_5;   // pull the pin low
			DWT_Delay(5);
			DS18B20_PIN_Input();
			DWT_Delay(55);
		} else {  // if the bit is low
			// write 0
			DS18B20_PIN_Output();
			GPIOB->BRR = GPIO_Pin_5;   // pull the pin low
			DWT_Delay(55);
			DS18B20_PIN_Input();
			DWT_Delay(5);
		}
	}
}

uint8_t DS18B20_Read (void)
{
	uint8_t value=0;
	DS18B20_PIN_Input();

	for (int i=0;i<8;i++)
	{
		DS18B20_PIN_Output();
		GPIOB->BRR = GPIO_Pin_5;   // pull the pin low
		DWT_Delay(2);

		DS18B20_PIN_Input();
		if (GPIOB->IDR & GPIO_Pin_5)  // if the pin is HIGH
		{
			value |= 1<<i;  // read = 1
		}
		DWT_Delay(60);
	}
	return value;
}

uint16_t DS18B20_GetTemperature(void)
{
	uint8_t check=2, temp_l=0, temp_h=0;
	uint16_t temp=0;

    check = DS18B20_Init();
    DS18B20_Write(0xCC);  // skip ROM
    DS18B20_Write(0x44);  // convert t
    DWT_Delay(800);

    DS18B20_Init();
    DS18B20_Write(0xCC);  // skip ROM
    DS18B20_Write(0xBE);  // Read Scratchpad

	temp_l = DS18B20_Read();
	temp_h = DS18B20_Read();
	return (temp_h<<8)|temp_l;

}
 */
