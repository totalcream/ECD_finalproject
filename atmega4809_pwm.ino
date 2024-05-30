#include <avr/io.h>
#include <util/delay.h>
#include <stdio.h>

#define TCA_CTRLA   TCA0.SPLIT.CTRLA
#define TCA_CTRLB   TCA0.SPLIT.CTRLB
#define TCA_CTRLD   TCA0.SPLIT.CTRLD
#define TCA_CTRLESET TCA0.SPLIT.CTRLESET
#define TCA_LPER    TCA0.SPLIT.LPER
#define TCA_HPER    TCA0.SPLIT.HPER
#define TCA_LCMP0   TCA0.SPLIT.LCMP0
#define TCA_HCMP0   TCA0.SPLIT.HCMP0
#define TCA_LCMP1   TCA0.SPLIT.LCMP1
#define TCA_HCMP1   TCA0.SPLIT.HCMP1

#define USART1_BAUD (unsigned int *)(0x0828)
#define USART1_CTRLB (unsigned char *)(0x0826)
#define USART1_STATUS (unsigned char *)(0x0824)
#define USART1_TXDATAL (unsigned char *)(0x0822)
#define USART1_RXDATAL (unsigned char *)(0x0820)
#define USART_DREIF_bm (0x20)
#define USART_RXCIF_bm (0x80)

// #define F_CPU 8000000UL
#define F_CPU 16000000UL
#define BAUD_RATE 9600
#define USART0_BAUD_RATE(BAUD_RATE) ((float)(F_CPU * 64 / (16 * (float)BAUD_RATE)) + 0.5)

void USART1_init(void);
void USART1_Transmit(unsigned char data);
unsigned char USART1_Receive(void);

void USART1_init(void)
{
  PORTC.DIR |= (1 << 0);  // Set TX pin as output
  PORTC.DIR &= ~(1 << 1); // Set RX pin as input

  *USART1_BAUD = (uint16_t)USART0_BAUD_RATE(BAUD_RATE);
  *USART1_CTRLB |= (1 << 3) | (1 << 4); // Enable TX and RX
}

void USART1_Transmit(unsigned char data)
{
  while (!((*USART1_STATUS) & USART_DREIF_bm)); // Wait for Data Register Empty
  *USART1_TXDATAL = data;
}

unsigned char USART1_Receive(void)
{
  while (!((*USART1_STATUS) & USART_RXCIF_bm)); // Wait for Receive Complete
  return *USART1_RXDATAL;
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
  TCA_CTRLESET = TCA_SPLIT_CMD_RESET_gc;
  TCA0.SPLIT.HCNT = 0;
  TCA0.SPLIT.LCNT = 0;
  TCA0.SPLIT.HCMP0 = 0;
  TCA_LCMP0 = 0;
  TCA0.SPLIT.HCMP1 = 0;
  TCA_LCMP1 = 0;
}

void TCA_init() {
  // Use PORTMUX to route TCA0 outputs to PORTD
  PORTMUX.TCAROUTEA = PORTMUX_TCA0_PORTD_gc; // PD0 for WO0, PD1 for WO1
  
  // Set period for low byte and high byte
  TCA_LPER = 156; // 20ms / (32us) = 625 ticks
  TCA_HPER = 156;

  // Enable TCA0 with DIV256 prescaler
  TCA_CTRLA = TCA_SPLIT_CLKSEL_DIV1024_gc | TCA_SPLIT_ENABLE_bm;

  // Enable split mode and set waveform output mode
  TCA_CTRLB = TCA_SPLIT_LCMP0EN_bm | TCA_SPLIT_LCMP1EN_bm | TCA_SPLIT_SPLITM_bm;
}

void PORT_init() {
  PORTD.DIRSET = PIN0_bm | PIN1_bm; // Set PD0 and PD1 as output
  PORTD.OUT &= ~(PIN0_bm | PIN1_bm); // Clear PD0 and PD1 output
}

int move_Servo(int angle) {
  if (angle < -90) angle = -90;
  else if (angle > 90) angle = 90;

  // Map angle to pulse width in terms of timer ticks
  int mov = map(angle, -45, 45, 8, 16); // 1ms (31 ticks) to 2ms (62 ticks)
  return mov;
}

void setup() {
  TCA_reset();
  TCA_init();
  PORT_init();
  USART1_init();
  TCA_LCMP0 = move_Servo(0); // Initialize servo 1 to 0 degrees
  TCA_LCMP1 = move_Servo(0); // Initialize servo 2 to 0 degrees
}

void loop() {
  static int angle1 = 0;
  static int angle2 = 0;
  char buffer[50];
  
  // Move servo 1 to current angle
  TCA_LCMP0 = move_Servo(angle1);
  // Move servo 2 to current angle
  TCA_LCMP1 = move_Servo(angle2);

  // Increment angles
  angle1++;
  angle2--;
  if (angle1 > 90) angle1 = -90;
  if (angle2 < -90) angle2 = 90;

  // Transmit current angles
  sprintf(buffer, "Angle1: %d, Angle2: %d\r\n", angle1, angle2);
  USART1_Transmit_String(buffer);

  // Wait for 1 second
  delay(20);
}