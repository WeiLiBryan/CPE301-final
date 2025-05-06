#include <LiquidCrystal.h>
#include <Stepper.h>
#include <RTClib.h>
#include <string.h>
#include <SimpleDHT.h> 

//   Constants  
#define LCD_RS 12
#define LCD_EN 11
#define LCD_D4 6
#define LCD_D5 5
#define LCD_D6 4
#define LCD_D7 3

#define DHT_PIN 7  

#define REV_STEPS 2038

#define FAN_BIT 0x10  // PORTB4 (Digital Pin 10)

#define GLED_BIT 0x80  // PORTC7 (Pin 30)
#define YLED_BIT 0x20  // PORTC5 (Pin 32)
#define RLED_BIT 0x08  // PORTC3 (Pin 34)
#define BLED_BIT 0x02  // PORTC1 (Pin 36)

#define BTN_START 0x08  // PORTB3 (Pin 11)
#define BTN_RESET 0x04  // PORTB2 (Pin 10)
#define BTN_CTRL 0x02  // PORTB1 (Pin 9)

#define PCI_MASK  0x0C  // PCINT2 and PCINT3

//   UART  
#define RDA 0x80
#define TBE 0x20

//   Register Pointers  
volatile unsigned char *PORTB_PTR = (unsigned char *) 0x25;
volatile unsigned char *DDRB_PTR  = (unsigned char *) 0x24;
volatile unsigned char *PINB_PTR  = (unsigned char *) 0x23;

volatile unsigned char *PORTC_PTR = (unsigned char *) 0x28;
volatile unsigned char *DDRC_PTR  = (unsigned char *) 0x27;

volatile unsigned char* my_ADMUX = (unsigned char*) 0x7C;
volatile unsigned char* my_ADCSRB = (unsigned char*) 0x7B;
volatile unsigned char* my_ADCSRA = (unsigned char*) 0x7A;
volatile unsigned int* my_ADC_DATA = (unsigned int*) 0x78;

volatile unsigned char *PCICR_PTR = (unsigned char *) 0x68;
volatile unsigned char *PCMSK0_PTR = (unsigned char *) 0x6B;

//   UART Registers  
volatile unsigned char *my_UCSR0A = (unsigned char *) 0x00C0;
volatile unsigned char *my_UCSR0B = (unsigned char *) 0x00C1;
volatile unsigned char *my_UCSR0C = (unsigned char *) 0x00C2;
volatile unsigned char *my_UBRR0L = (unsigned char *) 0x00C4;
volatile unsigned char *my_UBRR0H = (unsigned char *) 0x00C5;
volatile unsigned char *my_UDR0   = (unsigned char *) 0x00C6;

//   State Machine  
enum STATE {DISABLED, IDLE, ERROR, RUNNING};
STATE curState = DISABLED, prevState;

LiquidCrystal lcd(LCD_RS, LCD_EN, LCD_D4, LCD_D5, LCD_D6, LCD_D7);
Stepper vent(REV_STEPS, 28, 26, 24, 22);
SimpleDHT11 dht(DHT_PIN);  
RTC_DS3231 rtc;

char lcdText[16], errText[16];
char *stateNames[4] = {"DISABLED", "IDLE", "ERROR", "RUNNING"};
unsigned char ledMap[4] = {YLED_BIT, GLED_BIT, RLED_BIT, BLED_BIT};

const unsigned int tempLimit = 42, waterLimit = 400;
unsigned int waterLevel = 0;

byte temp = 0, humid = 0; 

//   Function Prototypes  
void IO_INIT();
void adc_init();
void U0init(unsigned long baud);
unsigned int adc_read(unsigned char ch);
void LED_UPDATE();
void load_ht();
void log_event(DateTime now);

//   UART FUNCTIONS  
unsigned char U0kbhit()
{
    return *my_UCSR0A & RDA;
}

unsigned char U0getchar()
{
    return *my_UDR0;
}

void U0putchar(unsigned char U0pdata)
{
    while ((*my_UCSR0A & TBE) == 0);
    *my_UDR0 = U0pdata;
}

void U0putstr(unsigned char* my_str)
{
    while (*my_str) U0putchar(*my_str++);
    U0putchar('\n');
}

void U0putint(unsigned int my_int)
{
    if (my_int >= 10) U0putint(my_int / 10);
    U0putchar('0' + (my_int % 10));
}

//   Setup  
void setup()
{
    lcd.begin(16, 2);
    rtc.begin();
    rtc.adjust(DateTime(F(__DATE__), F(__TIME__)));
    vent.setSpeed(2);

    IO_INIT();
    adc_init();
    U0init(19200);
}

//   Main Loop  
void loop()
{
    DateTime now = rtc.now();
    waterLevel = adc_read(0);

    if (now.second() == 0) load_ht();

    lcd.setCursor(0, 1);
    lcd.print(stateNames[curState]);
    LED_UPDATE();

    //   State Machine  
    curState = (curState == IDLE && temp >= tempLimit) ? RUNNING : curState;
    curState = (curState == RUNNING && temp < tempLimit) ? IDLE : curState;
    curState = (curState != DISABLED && waterLevel < waterLimit) ? ERROR : curState;

    if (curState == DISABLED)
    {
        *PORTB_PTR &= ~FAN_BIT;
    }
    else if (curState == IDLE)
    {
        *PORTB_PTR &= ~FAN_BIT;
        lcd.setCursor(0, 0);
        lcd.print(lcdText);
    }
    else if (curState == RUNNING)
    {
        lcd.setCursor(0, 0);
        lcd.print(lcdText);
        *PORTB_PTR |= FAN_BIT;
    }
    else if (curState == ERROR)
    {
        *PORTB_PTR &= ~FAN_BIT;
        snprintf(errText, 16, "Low water!");
        lcd.setCursor(0, 0);
        lcd.print(errText);
    }

    //   Vent Control  
    if ((*PINB_PTR & BTN_CTRL) != 0)
    {
        U0putstr((unsigned char*)"\nVENT MOVED");
        vent.step(1);
    }

    //   Log State Change  
    if (prevState != curState) log_event(now);
    prevState = curState;
}

//   ISR  
ISR(PCINT0_vect)
{
    if ((*PINB_PTR & BTN_RESET) != 0)
    {
        curState = (curState == ERROR) ? IDLE : curState;
    }
    else if ((*PINB_PTR & BTN_START) != 0)
    {
        curState = (curState == DISABLED) ? IDLE : DISABLED;
    }
}

//   Helper Functions  
void load_ht()
{
    int err = dht.read(DHT_PIN, &temp, &humid, NULL);

    if (err != SimpleDHTErrSuccess) {
        snprintf(lcdText, 16, "NO READ TRY AGAIN");
        return;
    }

    snprintf(lcdText, 16, "H:%d T:%dF", humid, temp);
}

void LED_UPDATE()
{
    *PORTC_PTR = ledMap[curState];
}

void log_event(DateTime now)
{
    U0putstr((unsigned char*)"\nSTATE: ");
    U0putstr((unsigned char*)stateNames[prevState]);
    U0putstr((unsigned char*)" -> ");
    U0putstr((unsigned char*)stateNames[curState]);

    U0putstr((unsigned char*)"\nTIME: ");
    U0putint(now.hour());
    U0putchar(':');
    U0putint(now.minute());
    U0putchar(':');
    U0putint(now.second());
    U0putchar('\n');
}

//   IO Initialization  
void IO_INIT()
{
    *DDRC_PTR |= (GLED_BIT | YLED_BIT | RLED_BIT | BLED_BIT);
    *DDRB_PTR |= FAN_BIT;

    *PORTB_PTR |= (BTN_START | BTN_RESET | BTN_CTRL);
    *DDRB_PTR &= ~(BTN_START | BTN_RESET | BTN_CTRL);

    *PCICR_PTR |= 0x01;
    *PCMSK0_PTR |= PCI_MASK;
}

//   ADC  
void adc_init()
{
    *my_ADCSRA |= 0b10000000;
    *my_ADCSRA &= 0b10111111;
    *my_ADCSRA &= 0b11011111;
    *my_ADCSRA &= 0b11111000;

    *my_ADCSRB &= 0b11110111;
    *my_ADCSRB &= 0b11111000;

    *my_ADMUX &= 0b01111111;
    *my_ADMUX |= 0b01000000;
    *my_ADMUX &= 0b11011111;
    *my_ADMUX &= 0b11100000;
}

unsigned int adc_read(unsigned char adc_channel_num)
{
    *my_ADMUX &= 0b11110000;
    *my_ADCSRB &= 0b11110111;

    *my_ADMUX += adc_channel_num;

    *my_ADCSRA |= 0b01000000;

    while ((*my_ADCSRA & 0x40) != 0);

    unsigned int val = (*my_ADC_DATA & 0x03FF);
    return val;
}

//   UART Initialization  
void U0init(unsigned long baud)
{
    unsigned int tbaud = (F_CPU / 16 / baud - 1);
    *my_UBRR0H = (tbaud >> 8);
    *my_UBRR0L = (tbaud & 0xFF);
    *my_UCSR0A = 0x20;
    *my_UCSR0B = 0x18;
    *my_UCSR0C = 0x06;
}
