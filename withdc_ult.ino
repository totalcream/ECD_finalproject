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
#define X_READ ADC_MUXPOS_AIN7_gc
#define Y_READ ADC_MUXPOS_AIN4_gc

// 보드 관련 매크로
// #define F_CPU 8000000UL
#define F_CPU 16000000UL

// USART 통신 관련 매크로
#define BAUD_RATE 9600
#define USART0_BAUD_RATE(BAUD_RATE) ((float)(F_CPU * 64 / (16 * (float)BAUD_RATE)) + 0.5)

// 모터 제어 관련 매크로
#define MAX_ANGLE 25
#define X_OFFSET -8
#define Y_OFFSET 3

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
#define Y_MID (Y_RIGHT+ Y_LEFT) / 2


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

    PORTA.DIRSET = PIN0_bm; // 초음파 센서 트리거
    PORTA.DIRCLR = PIN1_bm; // 초음파 센서 에코

    PORTA.DIRSET = PIN2_bm | PIN3_bm; // 모터1
    PORTF.DIRSET = PIN4_bm;

    PORTA.DIRSET = PIN4_bm | PIN5_bm; // 모터2
    PORTF.DIRSET = PIN5_bm;
}

void trigger_ultrasonic() {
    // Send a 10us pulse to trigger the ultrasonic sensor
    PORTE.OUTSET = PIN0_bm | PIN2_bm;
    _delay_us(10);
    PORTE.OUTCLR = PIN1_bm | PIN3_bm;
}

uint8_t obstacle_detected() {
    trigger_ultrasonic();

    uint8_t det = 0;

    while (!(PORTA.IN & PIN3_bm));
    while (PORTA.IN & PIN3_bm);   
    _delay_ms(10);                

    if (PORTA.IN & PIN3_bm) {
        det = 1;
    }

    while (!(PORTA.IN & PIN1_bm));
    while (PORTA.IN & PIN1_bm);   
    _delay_ms(10);                

    if (PORTA.IN & PIN1_bm) {
        det = 1;
    }
    return det; 
}

int move_Servo(int angle) {
    if (angle < -MAX_ANGLE) angle = -MAX_ANGLE;
    else if (angle > MAX_ANGLE) angle = MAX_ANGLE;

    // Map angle to pulse width in terms of timer ticks
    int mov = map(angle, -90, 90, 32, 157); // 1ms (8 ticks) to 2ms (16 ticks)
    return mov;
}

void TCB_init() {
    // TCB 활성화 및 CLK_PER/2 (16MHz/2 = 8MHz) 사용 설정
    TCB0.CTRLA = TCB_CLKSEL_CLKDIV2_gc;
    TCB0.CTRLB = TCB_CCMPEN_bm | TCB_CNTMODE_PWM8_gc; // 8비트 PWM 모드 활성화
    TCB0.CCMP = 0;
}

void motor_control(uint8_t motor, int8_t speed) {
    if (motor == 1) { // 왼쪽
        if (speed > 0) {
            // Left motor forward
            PORTA.OUTSET = PIN2_bm;
            PORTA.OUTCLR = PIN3_bm;
        } else if (speed < 0) {
            // Left motor backward
            PORTA.OUTCLR = PIN2_bm;
            PORTA.OUTSET = PIN3_bm;
            speed = -speed; 
        } else {
            // Left motor stop
            PORTA.OUTCLR = PIN2_bm | PIN3_bm;
        }
        TCB0.CCMP = speed; 
    } else if (motor == 2) { // 오른쪽
        if (speed > 0) {
            // Right motor forward
            PORTA.OUTSET = PIN4_bm;
            PORTA.OUTCLR = PIN5_bm;
        } else if (speed < 0) {
            // Right motor backward
            PORTA.OUTCLR = PIN4_bm;
            PORTA.OUTSET = PIN5_bm;
            speed = -speed; 
        } else {
            // Right motor stop
            PORTA.OUTCLR = PIN4_bm | PIN5_bm;
        }
        TCB0.CCMP = speed; // Set PWM duty cycle for right motor
    }
}

void motor_rotate_left(uint8_t speed) { // 좌측은 전진, 우측은 후진
    motor_control(1, speed); 
    motor_control(2, -speed); 
}

void motor_rotate_right(uint8_t speed) { // 좌측은 후진, 우측은 전진
    motor_control(1, -speed); 
    motor_control(2, speed); 
}

void ADC_init(void)
{
  // 의미불명의 코드
  PORTD_PIN4CTRL = PORT_ISC_INPUT_DISABLE_gc; // PF0 interrupt and digital input buffer disabled
  PORTD_PIN5CTRL = PORT_ISC_INPUT_DISABLE_gc; // PF1 interrupt and digital input buffer disabled
  PORTD_PIN6CTRL = PORT_ISC_INPUT_DISABLE_gc; // PF2 interrupt and digital input buffer disabled
  PORTD_PIN7CTRL = PORT_ISC_INPUT_DISABLE_gc; // PF3 interrupt and digital input buffer disabled

  // Enable ADC, set prescaler to 64 (16MHz / 128 = 62.5kHz ADC clock)
  ADC0.CTRLA = ADC_ENABLE_bm | ADC_RESSEL_10BIT_gc; // Enable ADC and set resolution to 10-bit
  ADC0.CTRLC = ADC_PRESC_DIV128_gc | ADC_REFSEL_VDDREF_gc; // Set prescaler and VDD as reference
}

void ADC_start(void) {
  ADC0_COMMAND |= ADC_STCONV_bm;
}

uint16_t ADC_read(uint8_t channel)
{
    ADC0.MUXPOS = channel; // Select ADC channel
    ADC0.COMMAND |= ADC_STCONV_bm; // Start conversion
    while (!(ADC0.COMMAND) & ADC_STCONV_bm); // Wait for conversion to complete
    // ADC0.INTFLAGS = ADC_RESRDY_bm; // Clear result ready flag
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

  if (ADC0_INTFLAGS & ADC_RESRDY_bm) {
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
  // PORTD.PIN7CTRL |= PORT_PULLUPEN_bm;

  // Small delay for voltage stabilization
  _delay_ms(1);

  if (ADC0_INTFLAGS & ADC_RESRDY_bm) {
    y = ADC_read(Y_READ);
  }

  if (y < 10) {
    y = pre_y;
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
  // TCA
    TCA_reset();
    TCA_init();

    // USART
    PORT_init();
    USART1_init();

    // ADC
    ADC_init();
    ADC_start();

    TCA_LCMP0 = move_Servo(0); // Initialize servo 1 to 0 degrees
    TCA_LCMP1 = move_Servo(0); // Initialize servo 2 to 0 degrees

    TCB_init();
}


// PID 제어기 초기화
struct PID pid_x = {3, 0.0066, 0.017, 0.0, 0.0}; // PID parameters for X axis
struct PID pid_y = {6.55, 0.0065, 0.0525, 0.0, 0.0}; // PID parameters for Y axis

void loop() {
  static int angle1 = 0;
  static int angle2 = 0;
  int pi_x = 0;
  int pi_y = 0;
  char buffer[150];
  unsigned long t_time;
  
  /*
    Read touch position
    X, Y 범위
    160 <= X <= 715
    80 <= Y <= 800
  */

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

  angle1 = map(pi_x, -4000, 4000, -25, 25);
  angle2 = map(pi_y, -8000, 8000, -25, 25);

  // 수평을 맞추기 위해서 X는 -8도 y는 +3도 offset 설정
  angle1 = -angle1 + X_OFFSET;
  angle2 = angle2 + Y_OFFSET;    

  // 서보 위치 제어
  TCA_LCMP0 = move_Servo(angle1);
  TCA_LCMP1 = move_Servo(angle2);

  // 현재 위치를 이전 값으로 저장
  pre_x = x;
  pre_y = y;

  //example : dc모터랑 초음파센서
  motor_rotate_right(128);

  if (obstacle_detected()) {
      motor_control(1, 0);
      motor_control(2, 0);
      delay(100);

      motor_control(1, -128);
      motor_control(2, -128);
      delay(100);
  } else {
      motor_control(1, 128);
      motor_control(2, 128);
      delay(100);
  }

  // Transmit current angles and touch position
  sprintf(buffer, "Angle1: %-10dAngle2: %-10dX: %-10uY: %-10d pi: %-5d \r\n", angle1, angle2, x, y, pi_x);
  USART1_Transmit_String(buffer);

  // Wait for 1 ms
  _delay_ms(1);
}
