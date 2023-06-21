// Motor Control
// UART RX -> Recive Safety Speed. (1sec / 30 ~ 35)
// High Torque Servo Motor Control PWM Range ( 9850 ~ 9550 )
// MCU -> STM32F407VET6 ( Cortex M4 )

// S-COOTER _ DETECTED DEPTH  CAMERA + AHRS + GPS
#include <stdio.h>
#include "stm32f4xx.h"
#include "led.h"
#include "delay.h"
#include "switch.h"
#include "UART.h"
#include <string.h>
#include <stdlib.h>

void parsingData_limit_speed(void);
void USART1_IRQHandler(void);
void TIM2_SET(void);
void TIM2_IRQHandler(void);
void HALLSENSORS_SET(void);
void RELAY_SET(void);
void NVIC_TIM2_Configuration(void);
void PWM_SET(void);
void Board_Set(void);

// UART1 : ON : PC->TEST -> RX(Fix Speed)
// UART2 : ON : JETSON NANO -> RX(Fix Speed)


#define TIMER_PRESCALER_FREQ        1000000             // timer 입력 클력 1MHz
#define PWM_FREQ                    50                // PWM 반복 주기 1ms = 1kHz
#define PWM_WIDTH                   (TIMER_PRESCALER_FREQ / PWM_FREQ)
#define DUTY_IDLE                   (PWM_WIDTH / 2)     // 50% duty
#define DUTY_MAX                    (PWM_WIDTH - 1)     // 최대 duty 값.

#define BRAKE_WAITING 9900
// #define BRAKE_MAX 150 // setp 3
#define BRAKE_MAX 300 // step 6

 int brake_value = 9900;

// brake 9850 : waiting brake state
// brake 9550 : max brake state
// brake 9700 : middle brake state

 //for(int i=9850; i>9550; i--)

char buffer_read_speed[100] = {0};
uint8_t bStartPacket_speed = 0;
uint8_t pos_packet_speed = 0;

int before_bit = 0 ;
int current_bit = 0 ;
int thick_count = 0;
int period_count = 0;
int prescaler_count= 0;
int count_calculation = 0;
int run_time = 0;



int target_speed = 0;
char buffer[100] = {0};

int current_speed = 15; // 실시간 속도 측정 구간

void parsingData_limit_speed()
{         //. $0,0,0,0#
   static char tokenizer[] = ",*";
   static char * tok = NULL;
   static char temp_packets[10][100] = {0};    
   int count = 0;   
   tok = strtok(buffer_read_speed,tokenizer);   

   while(tok != NULL)
   {
      strcpy(temp_packets[count],tok);
      tok = strtok(NULL,tokenizer);
      count++;
   }

   count=0;   
   target_speed = atoi(temp_packets[0]);       
   sprintf(buffer,"★ [ TS : %dkm/H]\n",target_speed);
   
   
   
   LED_D2_TOGGLE();   
   
   if(target_speed < current_speed)
   {          
     brake_value = 9900 - ((((current_speed - target_speed) /(double) 25) * 300)) ;
     // range step 20;     
     
     sprintf(buffer, "brake value = %4d\n",brake_value);
     UART1Print(buffer );
     
     TIM_SetCompare4(TIM3, (uint16_t) brake_value); //close     
   }  
   else if(target_speed >= current_speed)
   {
     brake_value = 9900;
     TIM_SetCompare4(TIM3, (uint16_t) brake_value); //close
   }
      
   UART1Print(buffer);
}

void USART1_IRQHandler(void) // PC TEST
{
  LED_D2_TOGGLE();
   char c;   
   if (USART_GetITStatus(USART1, USART_IT_RXNE) == SET)
   {
      c = USART_ReceiveData(USART1);
      if(c == '$')
      {      //패킷 시작
         bStartPacket_speed = 1;
      }
      if(c != '$' && bStartPacket_speed)
      {
         buffer_read_speed[pos_packet_speed] = c;
         pos_packet_speed++;
      }

      if(c == '#') 
      {         //문자열 끝
         bStartPacket_speed = 0;
         buffer_read_speed[pos_packet_speed] = '\0';
         parsingData_limit_speed();
         pos_packet_speed= 0;
         memset(buffer_read_speed,0,100);
      }                 
   }
}

void TIM2_SET(void)
{
  TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure;
  
  RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2, ENABLE);
  
  TIM_TimeBaseStructure.TIM_Period = 1000;
  TIM_TimeBaseStructure.TIM_Prescaler = 10;
  TIM_TimeBaseStructure.TIM_ClockDivision = 0;
  TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
  TIM_TimeBaseInit(TIM2, &TIM_TimeBaseStructure);
  TIM_ITConfig(TIM2, TIM_IT_Update, ENABLE);
  TIM_Cmd(TIM2,ENABLE);
}

void TIM2_IRQHandler(void) {
  LED_D3_TOGGLE();
  period_count++;
  
  if(TIM_GetITStatus(TIM2, TIM_IT_Update) == SET)
  {    
    TIM_ClearITPendingBit(TIM2, TIM_IT_Update);
    
    // 킥보드 속도 확인 //
     int hall_a=0, hall_b=0, hall_c=0;
    
    hall_a = GPIO_ReadInputDataBit(GPIOE, GPIO_Pin_9);
    hall_b = GPIO_ReadInputDataBit(GPIOE, GPIO_Pin_11);
    hall_c = GPIO_ReadInputDataBit(GPIOE, GPIO_Pin_13);
    
    current_bit = hall_a + (hall_b*2)+ (hall_c*4);         
    period_count++;
    //current_bit = before_bit;        
    
    if(before_bit != current_bit && current_bit > 0) thick_count++;
    before_bit = current_bit;
  }
  
  if(period_count>1000) 
  {
    prescaler_count++;
    period_count = 0;
  }   
  
  if(prescaler_count>10)
  {    
    run_time++;    
    
    current_speed = (thick_count * 0.008695652173913043) * 3.6;
    sprintf(buffer, "◎ CS : %2dkm/H [Thick : %3d ][RUN : %2dm %2ds]\n", current_speed, thick_count, run_time/60, run_time%60, current_bit);
    UART1Print(buffer); // 테스트 작업후 해당 문구 주석 해제
     //sprintf(buffer, "brake value = %4d\n",brake_value);
    // UART1Print(buffer );
    thick_count = 0;
    prescaler_count = 0;
  }
}
 
void HALLSENSORS_SET(void)
{
  

    GPIO_InitTypeDef GPIO_InitStructure;
    
    
    
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOE, ENABLE);
    
    //GPIO_PinAFConfig(GPIOE, GPIO_PinSource9|GPIO_PinSource11|GPIO_PinSource13, GPIO_AF_CAN1);
    
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9;
    
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN;
    
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
    
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
    
    GPIO_Init(GPIOE, &GPIO_InitStructure);        
    
    
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOE, ENABLE);
    
    //GPIO_PinAFConfig(GPIOE, GPIO_PinSource9|GPIO_PinSource11|GPIO_PinSource13, GPIO_AF_CAN1);
    
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_11;
    
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN;
    
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
    
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
    
    GPIO_Init(GPIOE, &GPIO_InitStructure);        
    
    
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOE, ENABLE);
    
    //GPIO_PinAFConfig(GPIOE, GPIO_PinSource9|GPIO_PinSource11|GPIO_PinSource13, GPIO_AF_CAN1);
    
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_13;
    
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN;
    
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
    
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
    
    GPIO_Init(GPIOE, &GPIO_InitStructure);        
}

void RELAY_SET(void)
{

    GPIO_InitTypeDef GPIO_InitStructure;
    
    
     GPIO_InitTypeDef GPIO_INitStructure;
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);
    GPIO_PinAFConfig(GPIOB, GPIO_PinSource5, GPIO_AF_CAN1);
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_5;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOB, &GPIO_InitStructure);    
    GPIO_WriteBit(GPIOB,GPIO_Pin_5,1); // low state
}

void NVIC_TIM2_Configuration(void)
{
  NVIC_InitTypeDef NVIC_InitStructure;  
  NVIC_InitStructure.NVIC_IRQChannel = TIM2_IRQn;
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority= 0;
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
  NVIC_Init(&NVIC_InitStructure);  
  
  RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2, ENABLE);
}
    

void PWM_SET(void)
{
    uint16_t PrescalerValue;
    TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure;
    TIM_OCInitTypeDef TIM_OCInitStructure;
    GPIO_InitTypeDef GPIO_InitStructure;
    
   RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3, ENABLE);
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB, ENABLE);
    // GPIO PIN B Seriese SET
    GPIO_PinAFConfig(GPIOB, GPIO_PinSource1, GPIO_AF_TIM3);
    // PWM GIPIO PIN_B1 SET
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_1;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
    
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOB, &GPIO_InitStructure); 

    // SYSTEM CLOCK SET
    /* Compute the prescaler value */
    SystemCoreClockUpdate();
    PrescalerValue = (uint16_t) (SystemCoreClock / 2 / TIMER_PRESCALER_FREQ) - 1;   // timer base counter에 1MHz 입력
    
    // PWM TIMER    
    /* Time base configuration */
    TIM_TimeBaseStructure.TIM_Period = TIMER_PRESCALER_FREQ / PWM_FREQ - 1;     // 1kHz timer
    TIM_TimeBaseStructure.TIM_Prescaler = PrescalerValue;
    TIM_TimeBaseStructure.TIM_ClockDivision = 0;
    TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
    TIM_TimeBaseInit(TIM3, &TIM_TimeBaseStructure);
    // PWM 구현
    TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;
    TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
    TIM_OCInitStructure.TIM_Pulse = DUTY_IDLE;       // default 50%
    TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;
    TIM_OC4Init(TIM3, &TIM_OCInitStructure);
    TIM_OC4PreloadConfig(TIM3, TIM_OCPreload_Disable);
    /* TIM3 enable counter */
    TIM_Cmd(TIM3, ENABLE);
}

void Board_Set(void)
{
    LED_INIT();    
    
    LED_D2_ON();
    LED_D3_ON();    
    
    SW_0_INIT();
    SW_1_INIT();   

    init_USART1();    
    
    TIM2_SET();
    
    HALLSENSORS_SET();
    
    RELAY_SET();
    
    PWM_SET();
    
    NVIC_TIM2_Configuration();
    
    // PWM GIPIO PIN_B1 SET
}

// pwm duty 9850 ~ 9550
// step 0 ~ 6

int main()
{
    Board_Set();
    while(1) 
    { 
    }
}


// Max step 6 : 9550