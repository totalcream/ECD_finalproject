#include <avr/io.h>
#include <util/delay.h>
#include <stdio.h>
#include <stdlib.h>


// 타이머 카운터 관련 매크로
#define TCA_CTRLA   TCA0.SPLIT.CTRLA
#define TCA_CTRLB   TCA0.SPLIT.CTRLB
#define TCA_CTRLD   TCA0.SPLIT.CTRLD
#define TCA_LPER    TCA0.SPLIT.LPER
#define TCA_HPER    TCA0.SPLIT.HPER
#define TCA_LCMP0   TCA0.SPLIT.LCMP0
#define TCA_HCMP0   TCA0.SPLIT.HCMP0
#define TCA_LCMP1   TCA0.SPLIT.LCMP1
#define TCA_HCMP1   TCA0.SPLIT.HCMP1

// 터치패널의 X-, X+, Y+, Y- 핀 번호
// PORTD에서 사용
#define XL PIN4_bm // PD4
#define XR PIN5_bm // PD5
#define YU PIN6_bm // PD6
#define YD PIN7_bm // PD7

// 터치패널의 ADC 입력을 받을 핀 번호
#define X_READ ADC_MUXPOS_AIN6_gc
#define Y_READ ADC_MUXPOS_AIN5_gc

// 보드 관련 매크로
// #define F_CPU 8000000UL
#define F_CPU 16000000UL

// USART 통신 관련 매크로
#define BAUD_RATE 9600
#define USART0_BAUD_RATE(BAUD_RATE) ((float)(F_CPU * 64 / (16 * (float)BAUD_RATE)) + 0.5)

// 서보모터 제어 관련 매크로
#define MAX_ANGLE 25
#define X_OFFSET -8
#define Y_OFFSET 0

// PID 관련 제어 매크로
#define DELTA_T 0.001
#define MAX_INTEGRAL 100.0  // Integral windup limit
#define FILTER_ALPHA 0.2

// 터치패널의 상하좌우 최소 최대 값 그리고 중앙 값
#define X_LEFT 170
#define X_RIGHT 740
#define X_MID ((X_RIGHT+ X_LEFT) / 2)
#define Y_LEFT 80
#define Y_RIGHT 800
#define Y_MID ((Y_RIGHT+ Y_LEFT) / 2)

// DC모터 제어 핀 정의
#define MOTOR_EN1 PIN4_bm  // Enable pin for motor 1 (PF4)
#define MOTOR_EN2 PIN5_bm  // Enable pin for motor 2 (PF5)
#define MOTOR_IN1 PIN2_bm  // Input 1 for motor 1 (PA2)
#define MOTOR_IN2 PIN3_bm  // Input 2 for motor 1 (PA3)
#define MOTOR_IN3 PIN4_bm  // Input 1 for motor 2 (PA4)
#define MOTOR_IN4 PIN5_bm  // Input 2 for motor 2 (PA5)
#define MOTOR_IN5 PIN0_bm  // Input 1 for motor 3 (PF0)
#define MOTOR_IN6 PIN1_bm  // Input 2 for motor 3 (PF1)
#define MOTOR_IN7 PIN2_bm  // Input 1 for motor 4 (PF2)
#define MOTOR_IN8 PIN3_bm  // Input 2 for motor 4 (PF3)

// 초음파 센서 핀 정의
#define TRIGGER_PIN1 PIN0_bm  // Trigger pin (PE0)
#define ECHO_PIN1 PIN1_bm     // Echo pin (PE1)
#define TRIGGER_PIN2 PIN2_bm  // Trigger pin (PE2)
#define ECHO_PIN2 PIN3_bm     // Echo pin (PE3)
#define TIME_OUT 20000        // 초음파센서 타임아웃 시간(us)

// 모터 구조체 정의
typedef struct {
    uint8_t in1;
    uint8_t in2;
    volatile uint8_t* out;
} Motor;

// 앞바퀴 및 뒷바퀴 모터 정의
Motor frontWheels1 = {MOTOR_IN1, MOTOR_IN2, &PORTA.OUT};
Motor rearWheels1 = {MOTOR_IN3, MOTOR_IN4, &PORTA.OUT};
Motor rearWheels2 = {MOTOR_IN5, MOTOR_IN6, &PORTF.OUT};
Motor frontWheels2 = {MOTOR_IN7, MOTOR_IN8, &PORTF.OUT};
int step = 220;
// 모터 전진 함수
void motorForward(Motor motor) {
    *(motor.out) |= motor.in1;
    *(motor.out) &= ~motor.in2;
}

// 모터 후진 함수
void motorBackward(Motor motor) {
    *(motor.out) |= motor.in2;
    *(motor.out) &= ~motor.in1;
}

// 모터 정지 함수
void motorStop(Motor motor) {
    *(motor.out) &= ~(motor.in1 | motor.in2);
}

void motorForwardAll() {
  motorForward(frontWheels1);
  motorForward(rearWheels1);
  motorForward(rearWheels2);
  motorForward(frontWheels2);
}

void motorStopAll() {
  motorStop(frontWheels1);
  motorStop(rearWheels1);
  motorStop(rearWheels2);
  motorStop(frontWheels2);
}

void motorBackwardAll() {
  motorBackward(frontWheels1);
  motorBackward(rearWheels1);
  motorBackward(rearWheels2);
  motorBackward(frontWheels2);
}

uint16_t f_distance = 0;
uint16_t r_distance = 0;
volatile uint8_t motorState = 0;
volatile uint8_t motorCounter = 0;
volatile uint8_t errorFlag = 0; // 이 변수로 이상 여부를 체크합니다.
bool isfront = false;
bool isrear = false;
// true is front, false is rear
bool goingdir = false;
ISR(TCB0_INT_vect) {
    motorCounter++;
    if (motorCounter >= 50) { // 100ms (assuming 10ms timer interval)
      motorCounter = 0;
      f_distance = measureDistance(&PORTE.OUT, TRIGGER_PIN1, &PORTE.IN, ECHO_PIN1);
      r_distance = measureDistance(&PORTE.OUT, TRIGGER_PIN2, &PORTE.IN, ECHO_PIN2);
      // 전방에 10 ~ 50 사이에 물체가 있을 경우
      if (f_distance > 10 & f_distance < 50)
        // 앞에 있다고 생각
        isfront = true;
      else
        // 아니면 없음
        isfront = false;
      // 후방에 10 ~ 50 사이에 물체가 있을 경우
      if (r_distance > 10 & r_distance < 50)
        // 뒤에 있다고 생각
        isrear = true;
      else
        // 아니면 없음
        isrear = false;
      
      
      // 앞에도 뒤에도 없을 경우
      if (!isfront && !isrear) {
        motorStopAll();
      }
      // 앞에는 있고 뒤에는 없을 경우
      else if (isfront & !isrear) {
        if (f_distance > 10 & f_distance < 50) {
          if (f_distance > 10 & f_distance < 25)
            TCB1.CCMPH = 170;
          else {
              TCB1.CCMPH = 220;
          }
          motorForwardAll();
        }
        else {
          motorStopAll();
        }
      }
      // 뒤에는 있고 앞에는 없는 경우
      else if (isrear & !isfront) {
        if (r_distance > 10 & r_distance < 50) {
          if (r_distance > 10 & r_distance < 25)
            TCB1.CCMPH = 170;
          else {
            TCB1.CCMPH = 220;
          }
        motorBackwardAll();
        }

        else {
          motorStopAll();
        }
      }
      // 앞에도 있고 뒤에도 있는 경우
      else {
        if (f_distance > r_distance) {
          if (f_distance > 10 & f_distance < 25)
            TCB1.CCMPH = 170;
          else {
            TCB1.CCMPH = 220;
          }
          motorForwardAll();
        }
        else if (f_distance < r_distance) {
          if (f_distance > 10 & f_distance < 25)
            TCB1.CCMPH = 170;
          else {
            TCB1.CCMPH = 220;
          }
          motorBackwardAll();
        }
        else {
          motorStopAll();
        }
      }
    }
    TCB0.INTFLAGS = TCB_CAPT_bm; // Clear the interrupt flag
}

// 초음파 센서 거리 측정 함수
uint16_t measureDistance(volatile uint8_t* triggerPort, uint8_t triggerPin, volatile uint8_t* echoPort, uint8_t echoPin) {
    uint32_t count = 0;
    uint32_t timeout = TIME_OUT; // 30ms timeout for the echo

    // Trigger pin을 LOW로 설정하고 잠시 대기
    *triggerPort &= ~triggerPin;
    _delay_us(2);

    // Trigger pin을 HIGH로 설정하고 10µs 대기
    *triggerPort |= triggerPin;
    _delay_us(10);

    // Trigger pin을 다시 LOW로 설정
    *triggerPort &= ~triggerPin;

    // Echo pin이 HIGH가 될 때까지 대기 (타임아웃 포함)
    while (!(*echoPort & echoPin)) {
        if (timeout-- == 0) return 0; // 타임아웃
    }

    timeout = TIME_OUT; // 타임아웃 리셋
    // Echo pin이 LOW가 될 때까지 시간 측정 (타임아웃 포함)
    while (*echoPort & echoPin) {
        if (timeout-- == 0) return 0; // 타임아웃
        count++;
        _delay_us(1); // 1µs 대기
    }

    // 측정된 시간(왕복 거리)을 거리로 변환
    uint16_t distance = count / 29; // 사운드 속도를 고려하여 거리 계산 (cm)

    return distance;
}

// PID제어에 사용할 구조체
struct PID {
    float Kp;
    float Ki;
    float Kd;
    float previous_error;
    float integral;
};

// PID 목표 위치 값
float setpoint_x = X_MID; // Setpoint (middle of the X range)
float setpoint_y = Y_MID; // Setpoint (middle of the Y range)
// float setpoint_y = 500.0; // Setpoint (middle of the Y range)

// 터치패널에서 읽어온 위치 값
uint16_t x, y;
// 터치패널에서 읽어온 이전의 값
uint16_t pre_x, pre_y;
int angle1 = 0;
int angle2 = 0;

// USART 관련 함수
void USART1_init(void);
void USART1_Transmit(unsigned char data);
unsigned char USART1_Receive(void);

void USART1_init(void)
{
    PORTC.DIR |= (1 << PIN0_bp);  // Set TX (PC0) as output
    PORTC.DIR &= ~(1 << PIN1_bp); // Set RX (PC1) as input

    USART1.BAUD = (uint16_t)USART0_BAUD_RATE(BAUD_RATE);
    USART1.CTRLB |= USART_TXEN_bm | USART_RXEN_bm; // Enable TX and RX
}

void USART1_Transmit(unsigned char data)
{
    while (!(USART1.STATUS & USART_DREIF_bm)); // Wait until data register is empty
    USART1.TXDATAL = data;
}

unsigned char USART1_Receive(void)
{
    while (!(USART1.STATUS & USART_RXCIF_bm)); // Wait until data is received
    return USART1.RXDATAL;
}

void USART1_Transmit_String(const char* str)
{
    while (*str)
    {
        USART1_Transmit(*str++);
    }
}

void TCA_reset() {
    TCA_CTRLA = 0;
    TCA0.SPLIT.CTRLESET = TCA_SPLIT_CMD_RESET_gc;
    TCA0.SPLIT.LCNT = 0;
    TCA0.SPLIT.HCNT = 0;
    TCA0.SPLIT.LCMP0 = 0;
    TCA0.SPLIT.HCMP0 = 0;
    TCA0.SPLIT.LCMP1 = 0;
    TCA0.SPLIT.HCMP1 = 0;
}

void TCA_init() {
    // Use PORTMUX to route TCA0 outputs to PORTD
    PORTMUX.TCAROUTEA = PORTMUX_TCA0_PORTD_gc; // PD0 for WO0, PD1 for WO1

    
    // Set period for low byte and high byte
    TCA_LPER = 1250; // 20ms / (128us) = 156.25 ticks
    TCA_HPER = 1250;

    // // Enable TCA0 with DIV1024 prescaler
    // TCA_CTRLA = TCA_SPLIT_CLKSEL_DIV1024_gc | TCA_SPLIT_ENABLE_bm;

    // // Enable split mode and set waveform output mode
    // TCA_CTRLB = TCA_SPLIT_LCMP0EN_bm | TCA_SPLIT_LCMP1EN_bm | TCA_SPLIT_SPLITM_bm;
    TCA_CTRLA = B00001101;
    TCA_CTRLB = B01110111;
    TCA_CTRLD = B00000001;
}

void PORT_init() {
    PORTD.DIRSET = PIN0_bm | PIN1_bm; // Set PD0 and PD1 as output
    PORTD.OUT &= ~(PIN0_bm | PIN1_bm); // Clear PD0 and PD1 output
}

int move_Servo(int angle, bool is_x) {
    if (is_x) {
      if (angle < -MAX_ANGLE+ 5) angle = - MAX_ANGLE - 5;
      else if (angle > MAX_ANGLE + 5) angle = MAX_ANGLE + 5;
    }
    else {
      if (angle < -MAX_ANGLE) angle = -MAX_ANGLE;
      else if (angle > MAX_ANGLE) angle = MAX_ANGLE;
    }

    // Map angle to pulse width in terms of timer ticks
    int mov = map(angle, -90, 90, 32, 157); // 1ms (8 ticks) to 2ms (16 ticks)
    return mov;
}

void ADC_init(void)
{
  // 의미불명의 코드
  PORTD.PIN4CTRL &= ~PORT_ISC_gm;
  PORTD.PIN5CTRL &= ~PORT_ISC_gm;
  PORTD.PIN6CTRL &= ~PORT_ISC_gm;
  PORTD.PIN7CTRL &= ~PORT_ISC_gm;

  PORTD.PIN4CTRL |= PORT_ISC_INPUT_DISABLE_gc; // PF0 interrupt and digital input buffer disabled
  PORTD.PIN5CTRL |= PORT_ISC_INPUT_DISABLE_gc; // PF1 interrupt and digital input buffer disabled
  PORTD.PIN6CTRL |= PORT_ISC_INPUT_DISABLE_gc; // PF2 interrupt and digital input buffer disabled
  PORTD.PIN7CTRL |= PORT_ISC_INPUT_DISABLE_gc; // PF3 interrupt and digital input buffer disabled

  PORTD.PIN4CTRL &= ~PORT_PULLUPEN_bm;
  PORTD.PIN5CTRL &= ~PORT_PULLUPEN_bm;
  PORTD.PIN6CTRL &= ~PORT_PULLUPEN_bm;
  PORTD.PIN7CTRL &= ~PORT_PULLUPEN_bm;

  // Enable ADC, set prescaler to 64 (16MHz / 128 = 62.5kHz ADC clock)
  ADC0.CTRLA = ADC_ENABLE_bm | ADC_RESSEL_10BIT_gc; // Enable ADC and set resolution to 10-bit
  ADC0.CTRLC = ADC_PRESC_DIV128_gc | ADC_REFSEL_VDDREF_gc; // Set prescaler and VDD as reference
}

void ADC_start(void) {
  ADC0.COMMAND |= ADC_STCONV_bm;
}

uint16_t ADC_read(uint8_t channel)
{
    ADC0.MUXPOS = channel; // Select ADC channel
    ADC0.COMMAND = ADC_STCONV_bm; // Start conversion
    // while (!(ADC0.COMMAND) & ADC_STCONV_bm); // Wait for conversion to complete
    while ( !(ADC0.INTFLAGS & ADC_RESRDY_bm) )
    ADC0.INTFLAGS = ADC_RESRDY_bm; // Clear result ready flag
    return ADC0.RES; // Return ADC result
}

void read_touch_position() {
  // 
  pre_x = x;
  pre_y = y;
  // Configure Y-axis as outputs and X-axis as inputs
  PORTD.DIRSET = XL | XR; // outputs
  PORTD.DIRCLR = YU | YD; // inputs

  // Set Y- (PD5) to GND and Y+ (PD4) to VCC
  PORTD.OUTSET = XL;
  PORTD.OUTCLR = XR;

  // Small delay for voltage stabilization
  _delay_ms(1);

  if (ADC0.INTFLAGS & ADC_RESRDY_bm) {
    x = ADC_read(X_READ);
  }

  if (x < 10) {
    x = pre_x;
  }
  // Read X-axis
  //  x = ADC_read(ADC_MUXPOS_AIN5_gc); // Read PD6 (AIN6)
  _delay_ms(5);

  // Configure X-axis (PD6, PD7) as outputs and Y-axis (PD4, PD5) as inputs
  PORTD.DIRSET = YU | YD; // PD6, PD7 as outputs
  PORTD.DIRCLR = XL | XR; // PD4, PD5 as inputs

  // Set X- (PD6) to GND and X+ (PD7) to VCC
  PORTD.OUTSET = YU;
  PORTD.OUTCLR = YD;

  // Small delay for voltage stabilization
  _delay_ms(1);

  if (ADC0.INTFLAGS & ADC_RESRDY_bm) {
    y = ADC_read(Y_READ);
  }

  if (y < 10) {
    y = pre_y;
    // return 0;
  }
  _delay_ms(5);

}

// 현재는 사용 안함
float calculate_velocity(uint16_t current_value, uint16_t previous_value, float delta_t) {
    return (current_value - previous_value) / delta_t;
}

float calculate_pid(struct PID *pid, float setpoint, float measured_value, float velocity)
{
    float error = (setpoint - measured_value);
    pid->integral += error * pid->Ki;

    // Integral windup protection
    if (pid->integral > MAX_INTEGRAL)
        pid->integral = MAX_INTEGRAL;
    else if (pid->integral < -MAX_INTEGRAL)
        pid->integral = -MAX_INTEGRAL;

    float derivative = (error - pid->previous_error) / DELTA_T;
    // float derivative = -velocity;
    pid->previous_error = error;
    return (pid->Kp * error) + (pid->integral) + (pid->Kd * derivative);
}

void setup() {

  // 모터 제어 핀을 출력으로 설정
  PORTA.DIRSET = MOTOR_IN1 | MOTOR_IN2 | MOTOR_IN3 | MOTOR_IN4;
  PORTF.DIRSET = MOTOR_EN1 | MOTOR_EN2 | MOTOR_IN5 | MOTOR_IN6 | MOTOR_IN7 | MOTOR_IN8;

  // 모터 제어 핀 초기화 (LOW)
  PORTA.OUTCLR = MOTOR_IN1 | MOTOR_IN2 | MOTOR_IN3 | MOTOR_IN4;
  PORTF.OUTCLR = MOTOR_EN1 | MOTOR_EN2 | MOTOR_IN5 | MOTOR_IN6 | MOTOR_IN7 | MOTOR_IN8;

  // 초음파 센서 핀을 출력 및 입력으로 설정
  PORTE.DIRSET = TRIGGER_PIN1;
  PORTE.DIRCLR = ECHO_PIN1;
  PORTE.DIRSET = TRIGGER_PIN2;
  PORTE.DIRCLR = ECHO_PIN2;

  // TCB0 설정 (10ms 주기)
  TCB0.CTRLA = TCB_CLKSEL_CLKDIV2_gc | TCB_ENABLE_bm; // CLK_PER/2
  TCB0.CTRLB = TCB_CNTMODE_INT_gc; // Periodic Interrupt mode
  TCB0.CCMP = 16000; // 10ms 주기 (assuming 16MHz clock, 2 prescaler)
  TCB0.INTCTRL = TCB_CAPT_bm; // Enable capture interrupt

  PORTMUX.TCBROUTEA |= PORTMUX_TCB1_bm; // TCB1을 PF5로 라우팅
  PORTF.DIRSET = MOTOR_EN2; // PF5를 출력으로 설정
  TCB1.CTRLA = TCB_CLKSEL_CLKDIV2_gc | TCB_ENABLE_bm; // 분주율 1로 설정
  TCB1.CTRLB = TCB_CNTMODE_PWM8_gc | TCB_CCMPEN_bm; // 8비트 PWM 모드 및 CCMPEN 활성화
  TCB1.CCMP = 255; // 초기 듀티 사이클 설정 (50%)

  // TCA
  TCA_reset();
  TCA_init();

  // USART
  PORT_init();
  USART1_init();

  // ADC
  ADC_init();
  ADC_start();

  TCA_LCMP0 = move_Servo(0, true); // Initialize servo 1 to 0 degrees
  TCA_LCMP1 = move_Servo(0, false); // Initialize servo 2 to 0 degrees

  sei(); // 전역 인터럽트 허용
}


// PID 제어기 초기화
struct PID pid_x = {3.0, 0.007, 0.02, 0.0, 0.0}; // PID parameters for X axis
struct PID pid_y = {6.55, 0.0065, 0.0525, 0.0, 0.0}; // PID parameters for Y axis


void loop() {
  int pi_x = 0;
  int pi_y = 0;
  char buffer[150];
  unsigned long t_time;

  // // front sensor
  // f_distance = measureDistance(&PORTE.OUT, TRIGGER_PIN1, &PORTE.IN, ECHO_PIN1);
  // // some delay
  // _delay_ms(1);
  // // back sensor
  // r_distance = measureDistance(&PORTE.OUT, TRIGGER_PIN2, &PORTE.IN, ECHO_PIN2);
  
  // 터치패널 위 물체 위치 구하기
  read_touch_position();
  _delay_ms(1);

  float velocity_y = calculate_velocity(y, pre_y, DELTA_T);

  // TODO : cal_pid_angle_x and y 값 구하기
  pi_x = (int)calculate_pid(&pid_x, setpoint_x, x, velocity_y);
  pi_y = (int)calculate_pid(&pid_y, setpoint_y, y, velocity_y);

  if(pi_x < -4000) pi_x = -4000;
  else if(pi_x > 4000) pi_x = 4000;

  if(pi_y < -4000) pi_y = -4000;
  else if(pi_y > 4000) pi_y = 4000;

  angle1 = map(pi_x, -4800, 4800, -30, 30);
  angle2 = map(pi_y, -8000, 8000, -25, 25);

  // 수평을 맞추기 위해서 X는 -8도 y는 +3도 offset 설정
  angle1 = angle1 + X_OFFSET;
  angle2 = angle2 + Y_OFFSET;    

  // 서보 위치 제어
  TCA_LCMP0 = move_Servo(angle1, true);
  TCA_LCMP1 = move_Servo(angle2, false);

  // 현재 위치를 이전 값으로 저장
  pre_x = x;
  pre_y = y;
  // step++;
  TCB1.CCMPH = step;
  if (step > 255)
    step = 160;
  // Transmit current angles and touch position
  sprintf(buffer, "Angle1: %-5dAngle2: %-5dX: %-5uY: %-5d fr: %-5dre: %-5d \r\n", angle1, angle2, x, y, f_distance, r_distance);
  USART1_Transmit_String(buffer);

  // Wait for 1 ms
  _delay_ms(1);
}
