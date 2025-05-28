
#include <stdint.h>
#include <stdbool.h>
#include "hw_memmap.h"
#include "debug.h"
#include "gpio.h"
#include "hw_i2c.h"
#include "hw_types.h"
#include "i2c.h"
#include "pin_map.h"
#include "sysctl.h"
#include "systick.h"
#include "interrupt.h"
#include "uart.h"
#include "hw_ints.h"
#include "pwm.h"
#include "util.c"
#include <stdio.h>
#include <string.h>
#include <stdlib.h> // For atoi

#define SYSTICK_FREQUENCY		100		
#define	I2C_FLASHTIME				500			
#define LONG_PRESS_TIME			300			
#define SHORT_PRESS_TIME			30
#define BeepPeriod	16000
//*****************************************************************************
//
//I2C GPIO chip address and resigster define
//
//*****************************************************************************
#define TCA6424_I2CADDR 					0x22
#define PCA9557_I2CADDR						0x18

#define PCA9557_INPUT							0x00
#define	PCA9557_OUTPUT						0x01
#define PCA9557_POLINVERT					0x02
#define PCA9557_CONFIG						0x03

#define TCA6424_CONFIG_PORT0			0x0c
#define TCA6424_CONFIG_PORT1			0x0d
#define TCA6424_CONFIG_PORT2			0x0e

#define TCA6424_INPUT_PORT0				0x00
#define TCA6424_INPUT_PORT1				0x01
#define TCA6424_INPUT_PORT2				0x02

#define TCA6424_OUTPUT_PORT0			0x04
#define TCA6424_OUTPUT_PORT1			0x05
#define TCA6424_OUTPUT_PORT2			0x06


enum State{
    STATE_INIT,
    STATE_FLOWING,
    STATE_DISABLED,
    STATE_SET_DATE,
    STATE_SET_TIME,
    STATE_SET_ALARM,
    STATE_ERROR
};
enum Direction{
    DIR_LEFT,
    DIR_RIGHT
};
enum KeyControl{
		KEY_FUNC_SHORT,
		KEY_SHIFT_SHORT,
		KEY_ADD_SHORT,
		KEY_SAVE_SHORT,
		KEY_FUNC_LONG,
		KEY_SHIFT_LONG,
		KEY_ADD_LONG,
		KEY_SAVE_LONG
};

void 				DeviceInit(void);
void        StartProcess(void);
void 				Delay(uint32_t value);
void 				S800_GPIO_Init(void);
uint8_t 		I2C0_WriteByte(uint8_t DevAddr, uint8_t RegAddr, uint8_t WriteData);
uint8_t 		I2C0_ReadByte(uint8_t DevAddr, uint8_t RegAddr);
void        UARTStringPut(const char *);
void				S800_I2C0_Init(void);
void 				S800_UART_Init(void);
void    		S800_PWM_Init(void);
void 				handleCommand(const char *);


void        updateTime(DateTime *time);
void        DetectKeyPress(void);


void  			Seg7DisplayUpdateOne(uint8_t data, uint8_t position);
void        Seg7DisplayUpdateAll(uint8_t* data);
void        Seg7DisplayBlocked(uint16_t milliseconds);
void        Seg7DisableAll(void);
void        Seg7EnableAll(void);


void        Interval_10ms_Handler(void);
void        Interval_100ms_Handler(void);
void        Interval_1s_Handler(void);
void        Interval_500ms_Handler(void);

void        ExecState(void);
void        ExecFlowingState(void);
void        ExecDisabledState(void);
void        ExecSetDateState(void);
void        ExecSetTimeState(void);
void        ExecSetAlarmState(void);
void        ExecErrorState(void);

void        KeyPressHandler(void);
void        KeyFunctionExecute();


uint8_t CursorPosition = 0; // 光标位置
uint8_t PrevStateBeforeDisabled = STATE_INIT; // 用于记录禁用前的状态
uint8_t CurrentState = STATE_INIT;
uint16_t InitStateTimeMilliSeconds = 1000;
uint8_t key_code = 0x0ff;

uint8_t PWMFlag = 0;
uint8_t alarmBeepingFlag = 0, alarmBeepCount = 0;
uint8_t alarmEnabled = 1;
// 全局变量
DateTime    currentDateTime = {2025, 12, 12, 12, 12, 30};
DateTime    alarmTime = {0,0,0,12, 12, 40};
DateTime    dateTemp  = {0, 0, 0, 0, 0, 0};
DateTime    timeTemp  = {0, 0, 0, 0, 0, 0};
//systick software counter define
uint32_t ui32SysClock;
volatile uint16_t systick_10ms_couter,systick_100ms_counter,systick_500ms_counter,systick_1s_counter;
volatile uint8_t	systick_10ms_status,systick_100ms_status,systick_500ms_status,systick_1s_status;


volatile uint8_t result,cnt,key_value,gpio_status;
volatile uint16_t key_counter[8] = {0,0,0,0,0,0,0,0}; // 用于记录按键计数

volatile uint16_t    i2c_flash_cnt;
volatile uint8_t rightshift = 0x01;
uint8_t seg7[] = {0x3f,0x06,0x5b,0x4f,0x66,0x6d,0x7d,0x07,0x7f,0x6f,0x77,0x7c,0x39,0x5e,0x079,0x71,0x5c};
uint8_t Seg7Shining[8] = {0,0,0,0,0,0,0,0};
uint8_t Seg7Enable[8] = {1,1,1,1,1,1,1,1}; // 数码管使能状态

uint8_t RxEndFlag = 0;
unsigned char RxBuf[256];

unsigned char DateTimeString[32] = "2025-05-26 16:48:00 "; // 用于存储实际的日期-时间的字符串
unsigned char TimeStringToSet[16] = "16:48:00"; // 用于存储要显示的字符串
unsigned char DateStringToSet[16] = "2025-05-26"; // 用于存储要显示的日期字符串

uint8_t Seg7Display[8] = {0x3f, 0x06, 0x5b, 0x4f, 0x66, 0x6d, 0x7d, 0x07}; // 数码管显示的数字

uint8_t flow_direction = DIR_RIGHT; // 滚动方向
uint8_t flow_interval = 2; // 滚动间隔
uint8_t flow_cnt = 0; // 滚动计数器
uint8_t flow_index = 0; // 当前显示的字串起始索引

int main(void)
{

    DeviceInit();

    while (1)
    {
			
        ExecState();
        
        key_value = I2C0_ReadByte(TCA6424_I2CADDR,TCA6424_INPUT_PORT0);
        I2C0_WriteByte(PCA9557_I2CADDR, PCA9557_OUTPUT, ~key_value);


        if (systick_10ms_status)
        {
            Interval_10ms_Handler();
        }
        if (systick_100ms_status)
        {
            Interval_100ms_Handler();
        }
        if (systick_500ms_status)
        {
            Interval_500ms_Handler();
        }
        if (systick_1s_status) {
            Interval_1s_Handler();
        }
	}



}
void DeviceInit(void)
{
    ui32SysClock = SysCtlClockFreqSet((SYSCTL_XTAL_16MHZ |SYSCTL_OSC_INT | SYSCTL_USE_PLL |SYSCTL_CFG_VCO_480), 16000000);
    S800_GPIO_Init();
    S800_I2C0_Init();
    S800_UART_Init();	
    S800_PWM_Init();
    SysTickPeriodSet(ui32SysClock/SYSTICK_FREQUENCY);
    UARTIntEnable(UART0_BASE, UART_INT_RX | UART_INT_RT);
    //SysTick中断优先级默认就是0（最高），所以不需要特别设置
    //UART0的优先级我们已经设置为1，所以现在SysTick的优先级高于UART0
    IntEnable(INT_UART0);
    SysTickEnable();
    SysTickIntEnable();
    IntMasterEnable();   
}

void Delay(uint32_t value)
{
	uint32_t ui32Loop;
	for(ui32Loop = 0; ui32Loop < value; ui32Loop++){};
}


void UARTStringPut(const char *cMessage)
{
    while(*cMessage != '\0') {
        // Wait if UART is busy
        while(UARTBusy(UART0_BASE)) {};
        // Send character
        UARTCharPut(UART0_BASE, *(cMessage++));
    }
    
    // Ensure the last character is transmitted
    while(UARTBusy(UART0_BASE)) {};
}

void S800_UART_Init(void)
{
    SysCtlPeripheralEnable(SYSCTL_PERIPH_UART0);
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOA);                        //Enable PortA
    while(!SysCtlPeripheralReady(SYSCTL_PERIPH_GPIOA));            //Wait for the GPIO moduleA ready

    GPIOPinConfigure(GPIO_PA0_U0RX);                        // Set GPIO A0 and A1 as UART pins.
    GPIOPinConfigure(GPIO_PA1_U0TX);                

    GPIOPinTypeUART(GPIO_PORTA_BASE, GPIO_PIN_0 | GPIO_PIN_1);

    // Configure the UART for 115,200, 8-N-1 operation.
    UARTConfigSetExpClk(UART0_BASE, ui32SysClock,115200,(UART_CONFIG_WLEN_8 | UART_CONFIG_STOP_ONE |UART_CONFIG_PAR_NONE));
    UARTStringPut("\r\nHello, world!\r\n");
    
    // 设置UART0中断优先级为1（数值越小优先级越高）
    IntPrioritySet(INT_UART0, 1);
}

void S800_GPIO_Init(void)
{
	SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOF);						//Enable PortF
	while(!SysCtlPeripheralReady(SYSCTL_PERIPH_GPIOF));			//Wait for the GPIO moduleF ready
	SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOJ);						//Enable PortJ	
	while(!SysCtlPeripheralReady(SYSCTL_PERIPH_GPIOJ));			//Wait for the GPIO moduleJ ready	
	SysCtlPeripheralEnable(SYSCTL_PERIPH_GPION);						//Enable PortN	
	while(!SysCtlPeripheralReady(SYSCTL_PERIPH_GPION));			//Wait for the GPIO moduleN ready		
	
  GPIOPinTypeGPIOOutput(GPIO_PORTF_BASE, GPIO_PIN_0);			//Set PF0 as Output pin
  GPIOPinTypeGPIOOutput(GPIO_PORTN_BASE, GPIO_PIN_0);			//Set PN0 as Output pin

	GPIOPinTypeGPIOInput(GPIO_PORTJ_BASE,GPIO_PIN_0 | GPIO_PIN_1);//Set the PJ0,PJ1 as input pin
	GPIOPadConfigSet(GPIO_PORTJ_BASE,GPIO_PIN_0 | GPIO_PIN_1,GPIO_STRENGTH_2MA,GPIO_PIN_TYPE_STD_WPU);
}

void S800_I2C0_Init(void)
{
	uint8_t result;
  SysCtlPeripheralEnable(SYSCTL_PERIPH_I2C0);//初始化i2c模块
  SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOB);//使用I2C模块0，引脚配置为I2C0SCL--PB2、I2C0SDA--PB3
	GPIOPinConfigure(GPIO_PB2_I2C0SCL);//配置PB2为I2C0SCL
  GPIOPinConfigure(GPIO_PB3_I2C0SDA);//配置PB3为I2C0SDA
  GPIOPinTypeI2CSCL(GPIO_PORTB_BASE, GPIO_PIN_2);//I2C将GPIO_PIN_2用作SCL
  GPIOPinTypeI2C(GPIO_PORTB_BASE, GPIO_PIN_3);//I2C将GPIO_PIN_3用作SDA

	I2CMasterInitExpClk(I2C0_BASE,ui32SysClock, true);										//config I2C0 400k
	I2CMasterEnable(I2C0_BASE);	

	result = I2C0_WriteByte(TCA6424_I2CADDR,TCA6424_CONFIG_PORT0,0x0ff);		//config port 0 as input
	result = I2C0_WriteByte(TCA6424_I2CADDR,TCA6424_CONFIG_PORT1,0x0);			//config port 1 as output
	result = I2C0_WriteByte(TCA6424_I2CADDR,TCA6424_CONFIG_PORT2,0x0);			//config port 2 as output 

	result = I2C0_WriteByte(PCA9557_I2CADDR,PCA9557_CONFIG,0x00);					//config port as output
	result = I2C0_WriteByte(PCA9557_I2CADDR,PCA9557_OUTPUT,0x0ff);				//turn off the LED1-8
	
}

void S800_PWM_Init(void)
{
	SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOK);						//Enable PortK
	while(!SysCtlPeripheralReady(SYSCTL_PERIPH_GPIOK)){}			//Wait for the GPIO moduleK ready
	
	
  GPIOPinTypeGPIOOutput(GPIO_PORTK_BASE, GPIO_PIN_5);			//Set PK5 as Output pin
	GPIOPinConfigure(GPIO_PK5_M0PWM7);
	GPIOPinTypePWM(GPIO_PORTK_BASE, GPIO_PIN_5);		
  SysCtlPeripheralEnable(SYSCTL_PERIPH_PWM0);					  //使能PWM0模块
	while(!SysCtlPeripheralReady(SYSCTL_PERIPH_PWM0)) { } // Wait for the PWM0 module to be ready.
	PWMClockSet(PWM0_BASE, PWM_SYSCLK_DIV_1);             //设置PWM0时钟预分频为1分频

	PWMGenConfigure(PWM0_BASE,									//配置PWM发生器为递减技术模式，异步更新
					PWM_GEN_3,									//注意：使用的PWM发生器的号码与输出PWM的引脚有关
					PWM_GEN_MODE_DOWN |
					PWM_GEN_MODE_NO_SYNC);
	PWMGenPeriodSet(PWM0_BASE,PWM_GEN_3,BeepPeriod);		//设置PWM0发生器的周期为BeepPeriod
	PWMPulseWidthSet(PWM0_BASE,PWM_OUT_7,BeepPeriod/4);	//设置初PWM0输出宽度
	PWMOutputState(PWM0_BASE,PWM_OUT_7_BIT,true);				//使能PWM输出
	//PWMGenEnable(PWM0_BASE,PWM_GEN_3);							//使能PWM发生器
	
}

uint8_t I2C0_WriteByte(uint8_t DevAddr, uint8_t RegAddr, uint8_t WriteData)
{
	uint8_t rop;
	while(I2CMasterBusy(I2C0_BASE)){};//如果I2C0模块忙，等待
		//
	I2CMasterSlaveAddrSet(I2C0_BASE, DevAddr, false);
		//设置主机要放到总线上的从机地址。false表示主机写从机，true表示主机读从机
		
	I2CMasterDataPut(I2C0_BASE, RegAddr);//主机写设备寄存器地址
	I2CMasterControl(I2C0_BASE, I2C_MASTER_CMD_BURST_SEND_START);//执行重复写入操作
	while(I2CMasterBusy(I2C0_BASE)){};
		
	rop = (uint8_t)I2CMasterErr(I2C0_BASE);//调试用

	I2CMasterDataPut(I2C0_BASE, WriteData);
	I2CMasterControl(I2C0_BASE, I2C_MASTER_CMD_BURST_SEND_FINISH);//执行重复写入操作并结束
	while(I2CMasterBusy(I2C0_BASE)){};

	rop = (uint8_t)I2CMasterErr(I2C0_BASE);//调试用

	return rop;//返回错误类型，无错返回0
}

uint8_t I2C0_ReadByte(uint8_t DevAddr, uint8_t RegAddr)
{
	uint8_t value,rop;
	while(I2CMasterBusy(I2C0_BASE)){};	
	I2CMasterSlaveAddrSet(I2C0_BASE, DevAddr, false);
	I2CMasterDataPut(I2C0_BASE, RegAddr);
//	I2CMasterControl(I2C0_BASE, I2C_MASTER_CMD_BURST_SEND_START);		
	I2CMasterControl(I2C0_BASE,I2C_MASTER_CMD_SINGLE_SEND);//执行单词写入操作
	while(I2CMasterBusBusy(I2C0_BASE));
	rop = (uint8_t)I2CMasterErr(I2C0_BASE);
	Delay(1);
	//receive data
	I2CMasterSlaveAddrSet(I2C0_BASE, DevAddr, true);//设置从机地址
	I2CMasterControl(I2C0_BASE,I2C_MASTER_CMD_SINGLE_RECEIVE);//执行单次读操作
	while(I2CMasterBusBusy(I2C0_BASE));
	value=I2CMasterDataGet(I2C0_BASE);//获取读取的数据
		Delay(1);
	return value;
}

/*
	Corresponding to the startup_TM4C129.s vector table systick interrupt program name
*/
void SysTick_Handler(void)
{
		if (key_code ^ 0x0ff){
				KeyFunctionExecute();
				key_code = 0x0ff;
		}
    if (systick_1s_counter    != 0)
        systick_1s_counter--;
    else
    {
        systick_1s_counter	= SYSTICK_FREQUENCY;
        systick_1s_status = 1;  // 设置更新标志
				currentDateTime.second++;
				updateTime(&currentDateTime);
    
    // 更新数码管显示
    if (CurrentState == STATE_FLOWING)
    {
				formatDateTime(DateTimeString,&currentDateTime);
        FlowingStringToSeg7(DateTimeString, Seg7Display, strlen(DateTimeString));
    }
    }
    // 原有的systick计数器更新
    if (systick_100ms_counter	!= 0)
        systick_100ms_counter--;
    else
    {
        systick_100ms_counter	= SYSTICK_FREQUENCY/10;
        systick_100ms_status 	= 1;
    }

    if (systick_10ms_couter	!= 0)
        systick_10ms_couter--;
    else
    {
        systick_10ms_couter		= SYSTICK_FREQUENCY/100;
        systick_10ms_status 	= 1;
    }
    if (systick_500ms_counter != 0)
        systick_500ms_counter--;
    else
    {
        systick_500ms_counter = SYSTICK_FREQUENCY/2; // Reset counter for 500ms
        systick_500ms_status = 1; // Set update flag
    }
}

void Interval_10ms_Handler(void)
{
    // 10ms间隔处理逻辑
    systick_10ms_status	= 0;
		KeyPressHandler();

    //if (++gpio_flash_cnt >= GPIO_FLASHTIME/10)
    //{
    //    gpio_flash_cnt = 0;
    //    if (gpio_status)
    //        GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_0,GPIO_PIN_0 );
    //    else
    //        GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_0,0);
    //    gpio_status	= !gpio_status;
    //
    //}
    if (RxEndFlag){
        RxEndFlag = 0;

        GPIOPinWrite(GPIO_PORTN_BASE, GPIO_PIN_0,0);	
    }
}
void Interval_100ms_Handler(void)
{
    systick_100ms_status	= 0;
    if (CurrentState == STATE_FLOWING){
        if (flow_direction == DIR_LEFT) {
            // 向左滚动
            flow_cnt++;
            if (flow_cnt >= flow_interval) {
                flow_cnt = 0;
                flow_index++;
                if (flow_index >= strlen((char*)DateTimeString)) {
                    flow_index = 0; // 重置索引
                }
            } 
        } else {
            // 向右滚动
            flow_cnt++;
            if (flow_cnt >= flow_interval) {
                flow_cnt = 0;
                if (flow_index == 0) {
                    flow_index = strlen((char*)DateTimeString) - 1; // 从末尾开始
                } else {
                    flow_index--;
                }
            }
        }
        FlowingStringToSeg7((char*)DateTimeString, Seg7Display, strlen((char*)DateTimeString));
    }
}
void Interval_500ms_Handler(void)
{
		uint8_t i;
    systick_500ms_status = 0;
    for (i = 0; i < 8; i++){
        if (Seg7Shining[i]){
            Seg7Enable[i] = !Seg7Enable[i]; // 切换使能状态
        }
    }
		    // 如果闹钟正在响，执行闹钟响铃逻辑
    if (alarmBeepingFlag) {
        if (alarmBeepCount > 0) {
            alarmBeepCount--;
            if (PWMFlag) {
                PWMOutputState(PWM0_BASE, PWM_OUT_7_BIT, true); // 开启PWM输出
                PWMGenEnable(PWM0_BASE, PWM_GEN_3); // 启用PWM发生器
								PWMFlag = 0;
            } else {
                PWMOutputState(PWM0_BASE, PWM_OUT_7_BIT, false); // 关闭PWM输出
                PWMGenDisable(PWM0_BASE, PWM_GEN_3); // 禁用PWM发生器
								PWMFlag = 1;
            }
            if (alarmBeepCount == 0) {
                alarmBeepingFlag = 0; // 闹钟结束
            }
        }
    }
}
void Interval_1s_Handler(void)
{
    systick_1s_status = 0;
    // if time is equal to alarm time, set beeping flag
    if (alarmEnabled && currentDateTime.second == alarmTime.second
    && currentDateTime.minute == alarmTime.minute
    && currentDateTime.hour == alarmTime.hour) {
        alarmBeepingFlag = 1; 
        alarmBeepCount = 6;
        PWMFlag = 1;
    } 

}
void UART0_Handler(void)
{
    static uint8_t buffer[256] = {0};  
    static uint8_t bufferIndex = 0;    
    uint32_t ulStatus;
    
    ulStatus = UARTIntStatus(UART0_BASE, true);
    UARTIntClear(UART0_BASE, ulStatus);
    GPIOPinWrite(GPIO_PORTN_BASE, GPIO_PIN_0,1);
    
    // Read new characters
    while(UARTCharsAvail(UART0_BASE) && bufferIndex < 255) {
        char ch = UARTCharGetNonBlocking(UART0_BASE);
        if (ch == '\r' || ch == '\n') {
            if (bufferIndex > 0) {
                buffer[bufferIndex] = '\0';
                
                strcpy((char*)RxBuf, (char*)buffer);
                UARTStringPut("Received: ");
                UARTStringPut((char*)buffer);
                UARTStringPut("\r\n");
                
                handleCommand((char*)buffer);
                bufferIndex = 0;
                memset(buffer, 0, sizeof(buffer));
            }
        }
        else {
            buffer[bufferIndex++] = ch;
        }
    }
    
    RxEndFlag = 1;
}

void Seg7DisplayBlocked(uint16_t milliseconds){
    uint8_t i;
    uint16_t timer = 0;
    while (timer * 10 < milliseconds){
        if (++i2c_flash_cnt		>= I2C_FLASHTIME)
        {
            i2c_flash_cnt = 0;
            result = I2C0_WriteByte(TCA6424_I2CADDR,TCA6424_OUTPUT_PORT2,0);	//write port 2
            if (Seg7Enable[cnt])
            {
                result = I2C0_WriteByte(TCA6424_I2CADDR,TCA6424_OUTPUT_PORT1,Seg7Display[cnt]);	//write port 1 				
                result = I2C0_WriteByte(TCA6424_I2CADDR,TCA6424_OUTPUT_PORT2,rightshift);	//write port 2
            }

            cnt++;
            rightshift= rightshift<<1;

            if (cnt >= 0x8)
            {
                rightshift= 0x01;
                cnt = 0;
            }
        }
        if (systick_10ms_status)
        {
            systick_10ms_status = 0;
            timer++;
        }
    }
}


void Seg7DisableAll(void)
{
    uint8_t i;
    for (i = 0; i < 8; i++) {
        Seg7Enable[i] = 0; // 禁用所有数码管
        while (++i2c_flash_cnt	< I2C_FLASHTIME){}
        i2c_flash_cnt = 0;
        result = I2C0_WriteByte(TCA6424_I2CADDR,TCA6424_OUTPUT_PORT2,0);	//write port 2

        result = I2C0_WriteByte(TCA6424_I2CADDR,TCA6424_OUTPUT_PORT1,0x00);	//write port 1 				
    }
    PrevStateBeforeDisabled = CurrentState; // 记录当前状态
    //CurrentState = STATE_DISABLED;
}
void Seg7EnableAll(void)
{
    uint8_t i;
    for (i = 0; i < 8; i++) {
        Seg7Enable[i] = 1; // 启用所有数码管
    }
    CurrentState = PrevStateBeforeDisabled; // 恢复之前的状态
}
void StartProcess(void)
{
    result = I2C0_WriteByte(PCA9557_I2CADDR,PCA9557_OUTPUT,0x00); // Turn on all LEDs
    StringToSeg7("31910697", Seg7Display);
    Seg7DisplayBlocked(InitStateTimeMilliSeconds/2);
    result = I2C0_WriteByte(PCA9557_I2CADDR,PCA9557_OUTPUT,0x0ff); // Turn off all LEDs
		Seg7DisableAll();
    Delay(1000000); // 等待
		Seg7EnableAll();
    result = I2C0_WriteByte(PCA9557_I2CADDR,PCA9557_OUTPUT,0x00); // Turn on all LEDs
	  StringToSeg7("LYC  V1", Seg7Display);
		Seg7DisplayBlocked(InitStateTimeMilliSeconds/2); 
    result = I2C0_WriteByte(PCA9557_I2CADDR,PCA9557_OUTPUT,0x0ff); // Turn off all LEDs
    CurrentState = STATE_FLOWING; // 进入流播状态
}

void ExecState(void)
{
    switch (CurrentState) {
        case STATE_INIT:
			StartProcess();
            break;
        case STATE_FLOWING:
            ExecFlowingState();
            break;
        case STATE_DISABLED:
            ExecDisabledState();
            break;
        case STATE_SET_DATE:
            ExecSetDateState();
            break;
        case STATE_SET_TIME:
            ExecSetTimeState();
            break;
        case STATE_SET_ALARM:
            ExecSetAlarmState();
            break;
        case STATE_ERROR:
            ExecErrorState();
            break;
        default:
            break;
    }
}
void ExecFlowingState(void)
{
    ++ i2c_flash_cnt;
    if (i2c_flash_cnt >= I2C_FLASHTIME / 100)
    {

        i2c_flash_cnt = 0;
			result = I2C0_WriteByte(TCA6424_I2CADDR,TCA6424_OUTPUT_PORT2,0);	//write port 2
        if (Seg7Enable[cnt])
        {
            result = I2C0_WriteByte(TCA6424_I2CADDR,TCA6424_OUTPUT_PORT1,Seg7Display[cnt]);	//write port 1 				
            result = I2C0_WriteByte(TCA6424_I2CADDR,TCA6424_OUTPUT_PORT2,rightshift);	//write port 2
        }
        result = I2C0_WriteByte(TCA6424_I2CADDR,TCA6424_OUTPUT_PORT0,key_value);            
        cnt++;
        rightshift= rightshift<<1;
        if (cnt >= 8)
        {
            rightshift= 0x01;
            cnt = 0;
        }
    }
}
void ExecDisabledState(void)
{
    ++i2c_flash_cnt;
    if (i2c_flash_cnt >= I2C_FLASHTIME / 100)
    {
        i2c_flash_cnt = 0;
        result = I2C0_WriteByte(TCA6424_I2CADDR,TCA6424_OUTPUT_PORT2,0);	//write port 2
        result = I2C0_WriteByte(TCA6424_I2CADDR,TCA6424_OUTPUT_PORT1,0);	//write port 1 				
        result = I2C0_WriteByte(TCA6424_I2CADDR,TCA6424_OUTPUT_PORT2,0);	//write port 2
        result = I2C0_WriteByte(TCA6424_I2CADDR,TCA6424_OUTPUT_PORT0,key_value);   
    }         
}
void ExecSetDateState(void)
{
    ++ i2c_flash_cnt;
    if (i2c_flash_cnt >= I2C_FLASHTIME / 100)
    {

        i2c_flash_cnt = 0;
			result = I2C0_WriteByte(TCA6424_I2CADDR,TCA6424_OUTPUT_PORT2,0);	//write port 2
        if (Seg7Enable[cnt])
        {
            result = I2C0_WriteByte(TCA6424_I2CADDR,TCA6424_OUTPUT_PORT1,Seg7Display[cnt]);	//write port 1 				
            result = I2C0_WriteByte(TCA6424_I2CADDR,TCA6424_OUTPUT_PORT2,rightshift);	//write port 2
        }
        result = I2C0_WriteByte(TCA6424_I2CADDR,TCA6424_OUTPUT_PORT0,key_value);            
        cnt++;
        rightshift= rightshift<<1;
        if (cnt >= 8)
        {
            rightshift= 0x01;
            cnt = 0;
        }
    }
}
void ExecSetTimeState(void)
{
    ++ i2c_flash_cnt;
    if (i2c_flash_cnt >= I2C_FLASHTIME / 100)
    {

        i2c_flash_cnt = 0;
            result = I2C0_WriteByte(TCA6424_I2CADDR,TCA6424_OUTPUT_PORT2,0);	//write port 2
        if (Seg7Enable[cnt])
        {
            result = I2C0_WriteByte(TCA6424_I2CADDR,TCA6424_OUTPUT_PORT1,Seg7Display[cnt]);	//write port 1 				
            result = I2C0_WriteByte(TCA6424_I2CADDR,TCA6424_OUTPUT_PORT2,rightshift);	//write port 2
        }
        result = I2C0_WriteByte(TCA6424_I2CADDR,TCA6424_OUTPUT_PORT0,key_value);            
        cnt++;
        rightshift= rightshift<<1;
        if (cnt >= 8)
        {
            rightshift= 0x01;
            cnt = 0;
        }
    }
}
void ExecSetAlarmState(void)
{
    ++ i2c_flash_cnt;
    if (i2c_flash_cnt >= I2C_FLASHTIME / 100)
    {

        i2c_flash_cnt = 0;
            result = I2C0_WriteByte(TCA6424_I2CADDR,TCA6424_OUTPUT_PORT2,0);	//write port 2
        if (Seg7Enable[cnt])
        {
            result = I2C0_WriteByte(TCA6424_I2CADDR,TCA6424_OUTPUT_PORT1,Seg7Display[cnt]);	//write port 1 				
            result = I2C0_WriteByte(TCA6424_I2CADDR,TCA6424_OUTPUT_PORT2,rightshift);	//write port 2
        }
        result = I2C0_WriteByte(TCA6424_I2CADDR,TCA6424_OUTPUT_PORT0,key_value);            
        cnt++;
        rightshift= rightshift<<1;
        if (cnt >= 8)
        {
            rightshift= 0x01;
            cnt = 0;
        }
    }
}
void ExecErrorState(void)
{
    ++ i2c_flash_cnt;
    if (i2c_flash_cnt >= I2C_FLASHTIME / 100)
    {

        i2c_flash_cnt = 0;
            result = I2C0_WriteByte(TCA6424_I2CADDR,TCA6424_OUTPUT_PORT2,0);	//write port 2
        if (Seg7Enable[cnt])
        {
            result = I2C0_WriteByte(TCA6424_I2CADDR,TCA6424_OUTPUT_PORT1,0xff);	//write port 1 				
            result = I2C0_WriteByte(TCA6424_I2CADDR,TCA6424_OUTPUT_PORT2,rightshift);	//write port 2
        }
        result = I2C0_WriteByte(TCA6424_I2CADDR,TCA6424_OUTPUT_PORT0,key_value);            
        cnt++;
        rightshift= rightshift<<1;
        if (cnt >= 8)
        {
            rightshift= 0x01;
            cnt = 0;
        }
    }
}
void KeyPressHandler(void)
{
    uint8_t i = 0;
    for (i = 0; i < 8; i++) {
        if (key_counter[i] * 10 >= LONG_PRESS_TIME) { // 长按超过300ms
            key_code = KEY_FUNC_SHORT + i + 4; // 设置按键代码
            key_counter[i] = 0; // 重置计数器
            return;
        }
        if (~key_value & (1 << i)) {
            key_counter[i]++;
        } else {
            if (key_counter[i] * 10 >= LONG_PRESS_TIME) { // 长按超过300ms
                key_code = KEY_FUNC_SHORT + i + 4; // 设置按键代码
            } else if (key_counter[i] * 10>= SHORT_PRESS_TIME ) { // 短按
                key_code = KEY_FUNC_SHORT + i; // 设置按键代码
            }
            key_counter[i] = 0; // 重置计数器
						
        }
    }
}
void KeyFunctionExecute()
{
    switch (key_code) {
        case KEY_FUNC_LONG:
                    CurrentState = STATE_FLOWING; // 返回流播状态
					Seg7DisableAll();
                    memset(Seg7Shining, 0, sizeof(Seg7Shining)); // 清除闪烁状态
					Seg7EnableAll();
                    break;
        case KEY_FUNC_SHORT://在日期，时间，闹钟设定间切换。长按保存退出。
            switch (CurrentState) {
                case STATE_FLOWING:
                    CurrentState = STATE_SET_DATE; // 切换到设置日期状态
                    CursorPosition = 0; // 重置光标位置
                    Seg7DisableAll();
                    dateTemp = currentDateTime; // 保存当前日期时间
                    sprintf((char*)DateStringToSet, "%04d-%02d-%02d", dateTemp.year, dateTemp.month, dateTemp.day);
										memset(Seg7Shining, 0, sizeof(Seg7Shining)); // 清除闪烁状态
                    StringToSeg7(DateStringToSet, Seg7Display);
                    Seg7Shining[0] = 1; // 闪烁第一个数码管
                    Seg7Shining[1] = 1; // 闪烁第二个数码管
                    Seg7Shining[2] = 1; // 闪烁第三个数码管
                    Seg7Shining[3] = 1; // 闪烁第四个数码管
                    Seg7EnableAll(); // 启用数码管显示
                    break;
                case STATE_SET_DATE:
                    CurrentState = STATE_SET_TIME; // 切换到设置时间状态
                    CursorPosition = 0; // 重置光标位置
                    Seg7DisableAll();
                    timeTemp = currentDateTime; // 保存当前时间
                    sprintf((char*)TimeStringToSet, "%02d:%02d:%02d", timeTemp.hour, timeTemp.minute, timeTemp.second);
                    StringToSeg7(TimeStringToSet, Seg7Display);
                    memset(Seg7Shining, 0, sizeof(Seg7Shining)); // 清除闪烁状态
                    Seg7Shining[0] = 1;
                    Seg7Shining[1] = 1; 
                    Seg7EnableAll(); // 启用数码管显示
                    break;
                case STATE_SET_TIME:
                    CurrentState = STATE_SET_ALARM; // 切换到设置闹钟状态
                    CursorPosition = 0; // 重置光标位置
                    Seg7DisableAll();
                    timeTemp = alarmTime; // 保存当前闹钟
                    sprintf((char*)TimeStringToSet, "%02d:%02d:%02d", timeTemp.hour, timeTemp.minute, timeTemp.second);
                    StringToSeg7(TimeStringToSet, Seg7Display);
                    memset(Seg7Shining, 0, sizeof(Seg7Shining)); // 清除闪烁状态
                    Seg7Shining[0] = 1;
                    Seg7Shining[1] = 1; 
                    Seg7EnableAll(); // 启用数码管显示
                    break;
                case STATE_SET_ALARM:
                    CurrentState = STATE_FLOWING; // 返回流播状态
										Seg7DisableAll();
                    memset(Seg7Shining, 0, sizeof(Seg7Shining)); // 清除闪烁状态
										Seg7EnableAll();
                    break;
                default:
                    break;
                }
            break;
        case KEY_SHIFT_SHORT:
					
				case KEY_SHIFT_LONG:
            Seg7DisableAll();
            switch (CurrentState) {
                case STATE_SET_DATE:
                    // 光标向右移动
                    if (CursorPosition == 0){
                        Seg7Shining[0] = Seg7Shining[1] =  Seg7Shining[2] = Seg7Shining[3] = 0;
                        Seg7Shining[4] = Seg7Shining[5] = 1;
                    }
                    if (CursorPosition == 1){
                        Seg7Shining[4] = Seg7Shining[5] = 0;
                        Seg7Shining[6] = Seg7Shining[7] = 1;
                    }
                    if (CursorPosition == 2){
                        Seg7Shining[0] = Seg7Shining[1] =  Seg7Shining[2] = Seg7Shining[3] = 1;
                        Seg7Shining[6] = Seg7Shining[7] = 0;
                    }
                    CursorPosition = (CursorPosition + 1) % 3;
                    break;
                case STATE_SET_TIME:
                case STATE_SET_ALARM:
                    if (CursorPosition == 0){
                        Seg7Shining[0] = Seg7Shining[1] = 0;
                        Seg7Shining[2] = Seg7Shining[3] = 1;
                    }
                    if (CursorPosition == 1){
                        Seg7Shining[2] = Seg7Shining[3] = 0;
                        Seg7Shining[4] = Seg7Shining[5] = 1;
                    }
                    if (CursorPosition == 2){
                        Seg7Shining[4] = Seg7Shining[5] = 0;
                        Seg7Shining[0] = Seg7Shining[1] = 1;
                    }
                    CursorPosition = (CursorPosition + 1) % 3;
                    break;
                default:
                    break;
            
            }
            Seg7EnableAll(); // 启用数码管显示
            break;
        case KEY_ADD_SHORT:
            switch (CurrentState){
                case STATE_SET_DATE:
                    // 增加日期
                    if (CursorPosition == 0) {
                        dateTemp.year = (dateTemp.year + 1) % 10000; // 年份循环
                    } else if (CursorPosition == 1) {
                        dateTemp.month = (dateTemp.month % 12) + 1; // 月份循环
                    } else if (CursorPosition == 2) {
                        dateTemp.day = (dateTemp.day % getDaysInMonth(dateTemp.year, dateTemp.month)) + 1; // 日循环
                    }
                    sprintf((char*)DateStringToSet, "%04d-%02d-%02d", dateTemp.year, dateTemp.month, dateTemp.day);
                    StringToSeg7(DateStringToSet, Seg7Display);
                    break;
                case STATE_SET_TIME:
									;
                case STATE_SET_ALARM:
                    // 增加时间
                    if (CursorPosition == 0) {
                        timeTemp.hour = (timeTemp.hour + 1) % 24;
                    } else if (CursorPosition == 1) {
                        timeTemp.minute = (timeTemp.minute + 1) % 60; // 分钟循环
                    } else if (CursorPosition == 2) {
                        timeTemp.second = (timeTemp.second + 1) % 60; // 秒钟循环
                    }
                    sprintf((char*)TimeStringToSet, "%02d:%02d:%02d", timeTemp.hour, timeTemp.minute, timeTemp.second);
                    StringToSeg7(TimeStringToSet, Seg7Display);
                    break;
                default:
                    break;
            }
						break;
        case KEY_ADD_LONG:  
                switch (CurrentState){
                case STATE_SET_DATE:
                    // 增加日期
                    if (CursorPosition == 0) {
                        dateTemp.year = (dateTemp.year + 10) % 10000; // 年份循环
                    } else if (CursorPosition == 1) {
                        dateTemp.month = ((dateTemp.month+9) % 12) + 1; // 月份循环
                    } else if (CursorPosition == 2) {
                        dateTemp.day = (dateTemp.day + 9) % getDaysInMonth(dateTemp.year, dateTemp.month) + 1; // 日循环
                    }
                    sprintf((char*)DateStringToSet, "%04d-%02d-%02d", dateTemp.year, dateTemp.month, dateTemp.day);
                    StringToSeg7(DateStringToSet, Seg7Display);
                    break;
                case STATE_SET_TIME:
                case STATE_SET_ALARM:
                    // 增加时间
                    if (CursorPosition == 0) {
                        timeTemp.hour = (timeTemp.hour + 10) % 24;
                    } else if (CursorPosition == 1) {
                        timeTemp.minute = (timeTemp.minute + 10) % 60; // 分钟循环
                    } else if (CursorPosition == 2) {
                        timeTemp.second = (timeTemp.second + 10) % 60; // 秒钟循环
                    }
                    sprintf((char*)TimeStringToSet, "%02d:%02d:%02d", timeTemp.hour, timeTemp.minute, timeTemp.second);
                    StringToSeg7(TimeStringToSet, Seg7Display);
                    break;
                default:
                    break;
            }
            break;
        case KEY_SAVE_SHORT:
        case KEY_SAVE_LONG:
            if (CurrentState == STATE_SET_DATE) {
                CurrentState = STATE_FLOWING; // 返回流播状态
                Seg7DisableAll();
                // 保存日期设置
                currentDateTime = dateTemp;
                memset(Seg7Shining, 0, sizeof(Seg7Shining)); // 清除闪烁状态
                Seg7EnableAll();
            } else if (CurrentState == STATE_SET_TIME) {
                CurrentState = STATE_FLOWING; // 返回流播状态
                                            // 保存时间设置
                Seg7DisableAll();
                currentDateTime = timeTemp;
                memset(Seg7Shining, 0, sizeof(Seg7Shining)); // 清除闪烁状态
                Seg7EnableAll();
            } else if (CurrentState == STATE_SET_ALARM) {
                CurrentState = STATE_FLOWING; // 返回流播状态
                Seg7DisableAll();
                alarmEnabled = 1; // 启用闹钟
                // 保存闹钟设置
                alarmTime = timeTemp;
                memset(Seg7Shining, 0, sizeof(Seg7Shining)); // 清除闪烁状态
                CurrentState = STATE_FLOWING;                // 返回流播状态
                Seg7EnableAll();
            }
            break;
        default:
            break;

    }
		key_code = 0;
}
// 处理时间命令
void handleCommand(const char *cmd) {
    char cmd_copy[64];
    char response[64];
    char *p;
    char *token;
    char *parts[10];
    int part_count = 0;
    int i;         

    strncpy(cmd_copy, cmd, sizeof(cmd_copy) - 1);
    cmd_copy[sizeof(cmd_copy) - 1] = '\0';

    /* Remove trailing newline or carriage return characters */
    p = cmd_copy;
    while (*p) {
        if (*p == '\r' || *p == '\n') {
            *p = '\0';
            break;
        }
        p++;
    }

    to_upper(cmd_copy);

    /* --- PARSING LOGIC FIXED FOR C89 --- */
    /* First call to strtok requires the string */
    token = strtok(cmd_copy, " :");
    while (token != NULL && part_count < 10) {
        parts[part_count++] = token;
        /* Subsequent calls must use NULL */
        token = strtok(NULL, " :");
    }

    if (part_count == 0) {
        UARTStringPut("ERROR: Invalid command\r\n");
        return;
    }

    if (strcmp(parts[0], "*RST") == 0) {
        makeDateTime(2025, 12, 12, 12, 12, 30, &currentDateTime); /* 设置默认日期时间 */
        alarmEnabled = 0; /* 禁用闹钟 */
        if (CurrentState == STATE_DISABLED) {
            Seg7EnableAll(); // 恢复数码管显示
        }
        flow_direction = DIR_RIGHT;
        UARTStringPut("System Reset OK\r\n");
    } else if (strcmp(parts[0], "*SET") == 0 && part_count > 1) {
        if (strcmp(parts[1], "DATE") == 0) {
            for (i = 2; i < part_count; i += 2) {
                if (i + 1 < part_count) {
                    if (strcmp(parts[i], "YEAR") == 0) currentDateTime.year = atoi(parts[i + 1]);
                    else if (strcmp(parts[i], "MONTH") == 0) currentDateTime.month = atoi(parts[i + 1]);
                    else if (strcmp(parts[i], "DATE") == 0) currentDateTime.day = atoi(parts[i + 1]);
                }
            }
            UARTStringPut("SET DATE OK\r\n");
        } else if (strcmp(parts[1], "TIME") == 0) {
            for (i = 2; i < part_count; i += 2) {
                if (i + 1 < part_count) {
                    if (strcmp(parts[i], "HOUR") == 0) currentDateTime.hour = atoi(parts[i + 1]);
                    else if (strcmp(parts[i], "MINUTE") == 0) currentDateTime.minute = atoi(parts[i + 1]);
                    else if (strcmp(parts[i], "SECOND") == 0) currentDateTime.second = atoi(parts[i + 1]);
                }
            }
            UARTStringPut("SET TIME OK\r\n");
        } else if (strcmp(parts[1], "ALARM") == 0) {
            for (i = 2; i < part_count; i += 2) {
                if (i + 1 < part_count) {
                    if (strcmp(parts[i], "HOUR") == 0) alarmTime.hour = atoi(parts[i + 1]);
                    else if (strcmp(parts[i], "MINUTE") == 0) alarmTime.minute = atoi(parts[i + 1]);
                    else if (strcmp(parts[i], "SECOND") == 0) alarmTime.second = atoi(parts[i + 1]);
                }
								alarmEnabled = true;
            }
            UARTStringPut("SET ALARM OK\r\n");
        } else if (strcmp(parts[1], "DISPLAY") == 0 && part_count > 2) {
            if (strcmp(parts[2], "ON") == 0) {
								if (CurrentState == STATE_DISABLED)
									Seg7EnableAll(); // 启用数码管显示
            }
            else if (strcmp(parts[2], "OFF") == 0)
            {
                Seg7DisableAll();
								CurrentState = STATE_DISABLED;
            }
            UARTStringPut("SET DISPLAY OK\r\n");
        } else if (strcmp(parts[1], "FORMAT") == 0 && part_count > 2) {
            if (strcmp(parts[2], "LEFT") == 0) flow_direction = DIR_LEFT;
            else if (strcmp(parts[2], "RIGHT") == 0) flow_direction = DIR_RIGHT;
            UARTStringPut("SET FORMAT OK\r\n");
						
        } else {
            UARTStringPut("ERROR: Invalid SET command\r\n");
        }
    } else if (strcmp(parts[0], "*GET") == 0 && part_count > 1) {
        if (strcmp(parts[1], "DATE") == 0) {
            snprintf(response, sizeof(response), "DATE: %04d.%02d.%02d\r\n", currentDateTime.year, currentDateTime.month, currentDateTime.day);
            UARTStringPut(response);
        } else if (strcmp(parts[1], "TIME") == 0) {
            snprintf(response, sizeof(response), "TIME: %02d:%02d:%02d\r\n", currentDateTime.hour, currentDateTime.minute, currentDateTime.second);
            UARTStringPut(response);
        } else if (strcmp(parts[1], "ALARM") == 0) {
            snprintf(response, sizeof(response), "ALARM: %02d:%02d:%02d\r\n", alarmTime.hour, alarmTime.minute, alarmTime.second);
            UARTStringPut(response);
        } else if (strcmp(parts[1], "DISPLAY") == 0) {
            UARTStringPut(CurrentState==STATE_DISABLED ? "DISPLAY: OFF\r\n" : "DISPLAY: ON\r\n");
        } else if (strcmp(parts[1], "FORMAT") == 0) {
            UARTStringPut(flow_direction == DIR_LEFT ? "FORMAT: LEFT\r\n" : "FORMAT: RIGHT\r\n");
        } else {
            UARTStringPut("ERROR: Invalid GET command\r\n");
        }
    } else {
        UARTStringPut("ERROR: Unknown command\r\n");
    }
}