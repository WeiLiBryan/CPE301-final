#include <LiquidCrystal.h>
#include <SimpleDHT.h>
#include <RTClib.h>
#include <Stepper.h> 

// LCD pins
#define LCD_RS 30
#define LCD_EN 31
#define LCD_D4 32
#define LCD_D5 33
#define LCD_D6 34
#define LCD_D7 35

// Sensor and I/O pins
#define DHT_PIN A0
#define WATER_SENSOR A1
#define FAN_PIN 8

// Stepper motor (vent control)
#define STEPPER_PIN1 10
#define STEPPER_PIN2 11
#define STEPPER_PIN3 12
#define STEPPER_PIN4 13
#define STEPPER_STEPS 2048

// Buttons
#define BTN_START 22
#define BTN_RESET 23
#define BTN_FORCE 24
#define BTN_STEPPER 25

// LEDs
#define LED_RED    50
#define LED_YELLOW 51
#define LED_GREEN  52
#define LED_BLUE   53

// UART registers
volatile unsigned char *myUCSR0A = (unsigned char *)0x00C0;
volatile unsigned char *myUCSR0B = (unsigned char *)0x00C1;
volatile unsigned char *myUCSR0C = (unsigned char *)0x00C2;
volatile unsigned int  *myUBRR0  = (unsigned int *)0x00C4;
volatile unsigned char *myUDR0   = (unsigned char *)0x00C6;

// Devices
LiquidCrystal lcd(LCD_RS, LCD_EN, LCD_D4, LCD_D5, LCD_D6, LCD_D7);
SimpleDHT11 dht(DHT_PIN);
RTC_DS1307 rtc;
Stepper stepper(STEPPER_STEPS, STEPPER_PIN1, STEPPER_PIN2, STEPPER_PIN3, STEPPER_PIN4);

// machine state
enum State {DISABLED, IDLE, RUNNING, ERROR};
State state = DISABLED, prevState = DISABLED;

bool fanOn = false;
bool forceRun = false;
bool stepperMoved = false;

void setup() {
  U0init(9600);
  // rtc.adjust(DateTime(2025, 5, 10, 12, 0, 0)); // time testing
  lcd.begin(16, 2);
  rtc.begin();
  adc_init();
  stepper.setSpeed(10);

  pinMode(FAN_PIN, OUTPUT);
  pinMode(LED_RED, OUTPUT);
  pinMode(LED_GREEN, OUTPUT);
  pinMode(LED_YELLOW, OUTPUT);
  pinMode(LED_BLUE, OUTPUT);

  pinMode(BTN_START, INPUT_PULLUP);
  pinMode(BTN_RESET, INPUT_PULLUP);
  pinMode(BTN_FORCE, INPUT_PULLUP);
  pinMode(BTN_STEPPER, INPUT_PULLUP);

  lcd.print("Ready");
  delay(1000);
  lcd.clear();

  // might try I2C LCD later
  // lcd.begin(20, 4);
}

void updateLEDs(State s) {
  digitalWrite(LED_RED,    s == ERROR);
  digitalWrite(LED_GREEN,  s == IDLE);
  digitalWrite(LED_YELLOW, s == DISABLED);
  digitalWrite(LED_BLUE,   s == RUNNING);
}

// timestamps
void logTimestamp(const char* label) {
  DateTime now = rtc.now();
  Serial.print(label);
  Serial.print(" at ");
  Serial.print(now.month()); Serial.print("/");
  Serial.print(now.day()); Serial.print("/");
  Serial.print(now.year()); Serial.print(" ");
  Serial.print(now.hour()); Serial.print(":");
  Serial.print(now.minute()); Serial.print(":");
  Serial.println(now.second());
}

// lcddisplay
// lcd.setCursor(8, 0); lcd.print("FAN"); // testing alternate position
// lcd.print((fanOn ? "ON " : "OFF"));   // never finalized layout
void updateLCD(byte t, byte h) {
  lcd.setCursor(0, 0);
  lcd.print("T:"); lcd.print(t); lcd.print("C ");
  lcd.print("H:"); lcd.print(h); lcd.print("%");
  lcd.setCursor(0, 1);
  switch (state) {
    case DISABLED: lcd.print("DISABLED        "); break;
    case IDLE:     lcd.print("IDLE            "); break;
    case RUNNING:  lcd.print("RUNNING         "); break;
    case ERROR:    lcd.print("LOW WATER!      "); break;
  }
}

// button function
void handleButtons() {
  if (digitalRead(BTN_START) == LOW) {
    delay(150);
    state = (state == DISABLED) ? IDLE : DISABLED;
    forceRun = false;
    Serial.println("[BTN] Start/Disable");
  }

  if (digitalRead(BTN_RESET) == LOW && state == ERROR) {
    delay(150);
    state = IDLE;
    forceRun = false;
    Serial.println("[BTN] reset");
  }

  if (digitalRead(BTN_FORCE) == LOW) {
    delay(150);
    state = RUNNING;
    forceRun = true;
    Serial.println("[BTN] Forced run");
  }

  if (digitalRead(BTN_STEPPER) == LOW) {
    delay(150);
    stepper.step(50);
    stepperMoved = true;
    Serial.println("[BTN] vent open");
  }
}

// states (1,2,3) (idle, disabled, err)
void logStateChange(State from, State to) {
  Serial.print("[STATE] ");
  Serial.print(from); Serial.print(" -> "); Serial.println(to);
  logTimestamp("State changed");
}

// display snesor values on serial
void printSensors(byte t, byte h, int w) {
  Serial.print("[Sensor] T:"); Serial.print(t);
  Serial.print("C H:"); Serial.print(h);
  Serial.print("% Water:"); Serial.println(w);
}

void loop() {
  handleButtons();

  byte temp = 0, hum = 0;
  int err = dht.read(&temp, &hum, NULL);
  if (err != SimpleDHTErrSuccess) { // built in err handling
    Serial.println("[ERROR] Failed to read humidity n temp");
    return;
  }

  int water = adc_read(WATER_SENSOR);

  // low water = error
  // fanOn = (temp > 38 && hum < 60); // humidity for fan control
  if (!forceRun && state != DISABLED && water < 150 && state != ERROR) {
    state = ERROR;
    fanOn = false;
    Serial.println("[ALERT] Water too low");
  } else if (!forceRun && state != DISABLED && state != ERROR) {
    if (state == IDLE && temp >= 42) state = RUNNING;
    else if (state == RUNNING && temp < 42) state = IDLE;
  }

  digitalWrite(FAN_PIN, state == RUNNING);

  updateLEDs(state);
  updateLCD(temp, hum);

  if (state != prevState) {
    logStateChange(prevState, state);
    prevState = state;
  }

  printSensors(temp, hum, water);

  delay(1000);
}

// UART init (mostly lab 7 code)
void U0init(unsigned long baud) {
  unsigned int tbaud = (16000000UL / 16 / baud - 1);
  *myUCSR0A = 0x20;
  *myUCSR0B = 0x18;
  *myUCSR0C = 0x06;
  *myUBRR0  = tbaud;
}

// UART helpers
unsigned char U0kbhit() { return *myUCSR0A & 0x80; }
unsigned char U0getchar() { return *myUDR0; }
void U0putchar(unsigned char data) {
  while (!(*myUCSR0A & 0x20));
  *myUDR0 = data;
}

// ADC setup
void adc_init() {
  ADCSRA = 0x80;
  ADCSRB = 0x00;
  ADMUX  = 0x40;
}

unsigned int adc_read(unsigned char ch) {
  ADCSRB &= 0xF7;
  ADCSRB |= (ch & 0x08);
  ADMUX &= 0xF8;
  ADMUX |= (ch & 0x07);
  ADCSRA |= 0x40;
  while (ADCSRA & 0x40);
  return ADC;
}
