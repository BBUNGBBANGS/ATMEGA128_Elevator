/*
 * ATMEGA128_Elevator.c
 *
 * Created: 2022-11-10 오후 2:50:58
 * Author : 서지수
 */ 
#define F_CPU 16000000UL
#include <avr/io.h>
#include <stdlib.h>
#include <avr/interrupt.h>
#include <util/delay.h>

/* 7 segment LED PORTD.0~3 (High), PORTC.0-7(A-G,DP)
*
*            A
*          F   B
*            G
*          E   C
*            D       DP
*/
#define _BIT(x) (1 << (x))
#define LED_A  _BIT(0)
#define LED_B  _BIT(1)
#define LED_C  _BIT(2)
#define LED_D  _BIT(3)
#define LED_E  _BIT(4)
#define LED_F  _BIT(5)
#define LED_G  _BIT(6)
#define LED_DP _BIT(7)

uint8_t Segment_Num[10] = 
{
	LED_G | LED_DP,                                       		/* 0 */ 
	LED_A | LED_D | LED_E | LED_F | LED_G | LED_DP,				/* 1 */ 			
	LED_C | LED_F | LED_DP,                                     /* 2 */ 
	LED_E | LED_F | LED_DP,						                /* 3 */ 
	LED_A | LED_D | LED_E | LED_DP,        						/* 4 */ 	
	LED_B | LED_E | LED_DP,                                     /* 5 */ 
	LED_B | LED_DP,                                             /* 6 */ 
	LED_D | LED_E | LED_F| LED_G | LED_DP,						/* 7 */
	LED_DP,		                                                /* 8 */ 
	LED_E | LED_DP                              				/* 9 */ 
};

#define ELEVATOR_POSITION_FIRST         (1)
#define ELEVATOR_POSITION_SECOND        (2)
#define ELEVATOR_POSITION_MOVING        (8)

#define ELEVATOR_DOOR_CLOSE     (1)
#define ELEVATOR_DOOR_OPEN      (2)

#define LED_Y0      _BIT(0) 
#define LED_Y1      _BIT(1)      
#define LED_Y2      _BIT(2)
#define LED_Y3      _BIT(3)
#define LED_Y4      _BIT(4)
#define LED_Y5      _BIT(5)
#define LED_Y6      _BIT(6)
#define LED_Y7      _BIT(7)

uint16_t INT4_timer_100ms,INT5_timer_100ms,INT6_timer_100ms,INT7_timer_100ms;
uint8_t INT4_Status,INT5_Status,INT6_Status,INT7_Status;
uint8_t INT4_Status_ISR,INT5_Status_ISR,INT6_Status_ISR,INT7_Status_ISR;
uint8_t INT4_Status_Old,INT5_Status_Old,INT6_Status_Old,INT7_Status_Old;
uint16_t INT4_Count,INT5_Count,INT6_Count,INT7_Count;

uint8_t Elevator_Position = ELEVATOR_POSITION_FIRST;
uint8_t Elevator_Door_Status = ELEVATOR_DOOR_CLOSE;
uint16_t Elevator_Moving_Time_100ms,Elevator_Door_Time_100ms,Segment_Timer_100ms;

static void Port_Init(void);
static void External_ISR_Init(void);
static void Timer1_Init(void);
static void Elevator_Control(void);

int main(void)
{
    Port_Init();
    External_ISR_Init();
    Timer1_Init();

    while (1) 
    {
        Elevator_Control();
    }
}

static void Port_Init(void)
{
    /* LED 연결 포트 출력 설정 */
    DDRA = 0xFF; 
    /* LED 초기 출력 OFF 설정(HIGH) */
    PORTA = 0xFF; 

    /* 7-Segment Data 포트 출력 설정 */
    DDRC = 0x7F; 
    /* 7-Segment Data 출력 OFF 설정 (HIGH) */
    PORTC = 0x7F; 

    /* 7-Segment Select 포트 출력 설정 */
    DDRD = 0x0F; 
    /* 7-Segment Select 출력 OFF 설정 (LOW) */
    PORTD = 0x00; 

    /* Switch 포트 출력 설정 */
    DDRE = 0x00; 
    /* Switch 핀 Pull-Up 설정(HIGH) */
    PORTE = 0xF0;
}

static void External_ISR_Init(void)
{
    /* INT4,5,6,7 Riging Edge Interrupt 설정 */
    EICRB = 0xFF;    
    /* INT4,5,6,7 Enable */
    EIMSK = 0xF0; 
}

static void Timer1_Init(void)
{
    /* Timer 1 Overflow interrupt enable */
	TIMSK = 0x04; 
	TCCR1A = 0x00;
    /* Timer 1 Normal모드, 256 Prescale 설정 */    
	TCCR1B = 0x04; 
    /* Timer 1 Counter 설정, 1/16*256*(65536-59286) : 100[ms] */    
	TCNT1 =  59286; 
    /* Global Interrupt Enable */    
	SREG = 0x80; 
}

static void Elevator_Control(void)
{
    static uint8_t Elevator_Position_Old;
    
    /* 엘레베이터 위치 Control */
    switch(Elevator_Position)
    {
        case ELEVATOR_POSITION_FIRST : 
            /* LED OFF (Y2 ,Y3) */
            PORTA |= (LED_Y2 | LED_Y3);   
            /* 엘리베이터 위치 정보 이전값 기억 */
            Elevator_Position_Old = Elevator_Position;
            /* SW6이 눌린 경우 진입 */
            if(INT4_Status == 1)
            {
                /* LED ON (Y0) */
                PORTA &= (uint8_t)(~LED_Y0); 
                /* 엘리베이터 위치 정보 이동중으로 변경 */                
                Elevator_Position =  ELEVATOR_POSITION_MOVING;
                INT4_Status = 0;
            }
            /* SW7이 눌린 경우 진입 */            
            if(INT5_Status == 1)
            {
                /* LED ON (Y1) */
                PORTA &= (uint8_t)(~LED_Y1); 
                /* 엘리베이터 위치 정보 이동중으로 변경 */         
                Elevator_Position =  ELEVATOR_POSITION_MOVING;
                INT5_Status = 0;
            }
            /* SW9이 눌린 경우 진입 */
            if(INT7_Status == 1)
            {
                /* 엘리베이터 문 열기 */         
                Elevator_Door_Status = ELEVATOR_DOOR_OPEN;
                /* 엘리베이터 문 열림 시간 리셋 */         
                Elevator_Door_Time_100ms = 0; 
                INT7_Status = 0;
            }
            /* 7-Segment Q0 출력 (1) */
            PORTD = 0x01;
            PORTC = Segment_Num[ELEVATOR_POSITION_FIRST];
        break;
        case ELEVATOR_POSITION_SECOND : 
            /* LED OFF (Y0 ,Y1) */
            PORTA |= (LED_Y0 | LED_Y1);   
            /* 엘리베이터 위치 정보 이전값 기억 */
            Elevator_Position_Old = Elevator_Position;
            /* SW6이 눌린 경우 진입 */
            if(INT4_Status == 1)
            {
                /* 엘리베이터 문 열기 */         
                Elevator_Door_Status = ELEVATOR_DOOR_OPEN;
                /* 엘리베이터 문 열림 시간 리셋 */         
                Elevator_Door_Time_100ms = 0; 
                INT4_Status = 0;
            }
            /* SW8이 눌린 경우 진입 */      
            if(INT6_Status == 1)
            {               
                 /* LED ON (Y2) */
                PORTA &= (uint8_t)(~LED_Y2); 
                /* 엘리베이터 위치 정보 이동중으로 변경 */    
                Elevator_Position = ELEVATOR_POSITION_MOVING;
                INT6_Status = 0;
            }
            /* SW9이 눌린 경우 진입 */      
            if(INT7_Status == 1)
            {                
                /* LED ON (Y3) */
                PORTA &= (uint8_t)(~LED_Y3);  
                /* 엘리베이터 위치 정보 이동중으로 변경 */    
                Elevator_Position = ELEVATOR_POSITION_MOVING;
                INT7_Status = 0;
            }
            /* 7-Segment Q0 출력 (2) */
            PORTD = 0x01;
            PORTC = Segment_Num[ELEVATOR_POSITION_SECOND];
        break;
        case ELEVATOR_POSITION_MOVING : 
            /* 엘리베이터 이동 시간 10[s] 이후 위치 정보 변경 */
            if(Elevator_Moving_Time_100ms>=100)
            {
                /* 엘리베이터 위치 정보 이전값이 1층 일 경우 진입 */
                if(Elevator_Position_Old == ELEVATOR_POSITION_FIRST)
                {
                    /* 엘리베이터 위치 정보 2층으로 변경 */
                    Elevator_Position = ELEVATOR_POSITION_SECOND;  
                    /* 엘리베이터 목표 층 도착했으므로 문 열림 */ 
                    Elevator_Door_Status = ELEVATOR_DOOR_OPEN;           
                }                
                /* 엘리베이터 위치 정보 이전값이 2층 일 경우 진입 */
                if(Elevator_Position_Old == ELEVATOR_POSITION_SECOND)
                {                    
                    /* 엘리베이터 위치 정보 1층으로 변경 */
                    Elevator_Position = ELEVATOR_POSITION_FIRST;    
                    /* 엘리베이터 목표 층 도착했으므로 문 열림 */ 
                    Elevator_Door_Status = ELEVATOR_DOOR_OPEN;           
                }
                Elevator_Moving_Time_100ms = 0;
            }
            /* 7-Segment Q0 출력 깜빡임 (8) */
            if(Segment_Timer_100ms>=10)
            {
                /* 7-Segment Q0 선택 */
                PORTD = 0x01;
                /* 7-Segment 출력 OFF */
                PORTC = 0xFF;           
                /* Segment Timer가 2[s] 일 경우 Tiemr Reset */ 
                if(Segment_Timer_100ms>=20)     
                {
                    Segment_Timer_100ms = 0;
                }     
            }
            else
            {
                /* 7-Segment Q0 선택 */
                PORTD = 0x01;                
                /* 7-Segment 8 출력 */
                PORTC = Segment_Num[ELEVATOR_POSITION_MOVING];     
            }

        break;
    }

    /* 엘리베이터 문 상태 Control */
    if(Elevator_Door_Status == ELEVATOR_DOOR_OPEN)
    {
        /* LED ON (Y4,Y7) */
        PORTA &= (uint8_t)(~(LED_Y4|LED_Y7));    
        /* LED OFF (Y5,Y6) */
        PORTA |= (LED_Y5|LED_Y6); 
    }
    else
    {
        /* LED ON (Y4,Y5,Y6,Y7) */
        PORTA &= (uint8_t)(~(LED_Y4|LED_Y5|LED_Y6|LED_Y7));
    }
}


ISR(TIMER1_OVF_vect)
{
    /* Switch 중복 입력 방지를 위해 300[ms]의 Glitch Filter Time을 가짐 */
    if(INT4_Status_ISR == 1)
    {
        INT4_timer_100ms++;
        if(INT4_timer_100ms >= 3)
        {
            if((INT4_Status_ISR == 1)&&(INT4_Status_Old == 0))
            {
                INT4_Status = 1;
                INT4_Count++;
                INT4_Status_ISR = 0;
            }
            INT4_Status_Old = INT4_Status_ISR;
            INT4_timer_100ms = 0;
        }
    }
    if(INT5_Status_ISR == 1)
    {
        INT5_timer_100ms++;
        if(INT5_timer_100ms >= 3)
        {
            if((INT5_Status_ISR == 1)&&(INT5_Status_Old == 0))
            {
                INT5_Status = 1;
                INT5_Count++;
                INT5_Status_ISR = 0;
            }
            INT5_Status_Old = INT5_Status_ISR;
            INT5_timer_100ms = 0;
        }
    }
    if(INT6_Status_ISR == 1)
    {
        INT6_timer_100ms++;
        if(INT6_timer_100ms >= 3)
        {
            if((INT6_Status_ISR == 1)&&(INT6_Status_Old == 0))
            {
                INT6_Status = 1;
                INT6_Count++;
                INT6_Status_ISR = 0;
            }
            INT6_Status_Old = INT6_Status_ISR;
            INT6_timer_100ms = 0;
        }
    }
    if(INT7_Status_ISR == 1)
    {
        INT7_timer_100ms++;
        if(INT7_timer_100ms >= 3)
        {
            if((INT7_Status_ISR == 1)&&(INT7_Status_Old == 0))
            {
                INT7_Status = 1;
                INT7_Count++;
                INT7_Status_ISR = 0;
            }
            INT7_Status_Old = INT7_Status_ISR;
            INT7_timer_100ms = 0;
        }
    }

    if(Elevator_Position == ELEVATOR_POSITION_MOVING)
    {
        Elevator_Moving_Time_100ms++;
        Segment_Timer_100ms++;
    }

    if(Elevator_Door_Status == ELEVATOR_DOOR_OPEN)
    {
        Elevator_Door_Time_100ms++;
        if(Elevator_Door_Time_100ms >= 50)
        {
            Elevator_Door_Status = ELEVATOR_DOOR_CLOSE;
            Elevator_Door_Time_100ms = 0;
        }
    }
    
    /* Timer 1 Counter 설정, 1/16*256*(65536-59286) : 100[ms] */    
    TCNT1 = 59286; 
}

ISR(INT4_vect)
{
    /* 엘리베이터 위치가 1층일 경우에만 SW 입력 받음 */
    if(Elevator_Position == ELEVATOR_POSITION_FIRST)
    {
        INT4_Status_ISR = 1;
    }
}

ISR(INT5_vect)
{
    /* 엘리베이터 위치가 1층일 경우에만 SW 입력 받음 */
    if(Elevator_Position == ELEVATOR_POSITION_FIRST)
    {
        INT5_Status_ISR = 1;
    }
}

ISR(INT6_vect)
{
    /* 엘리베이터 위치가 2층일 경우에만 SW 입력 받음 */
    if(Elevator_Position == ELEVATOR_POSITION_SECOND)
    {
        INT6_Status_ISR = 1;
    }
}

ISR(INT7_vect)
{
    /* 엘리베이터 위치가 2층일 경우에만 SW 입력 받음 */
    if(Elevator_Position == ELEVATOR_POSITION_SECOND)
    {
        INT7_Status_ISR = 1;
    }
}
