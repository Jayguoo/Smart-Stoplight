#include <Arduino.h>
#include <TM1637.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "freertos/queue.h"
#include "freertos/timers.h"
#include "driver/gpio.h"

//  Pins 
#define RED1 42
#define YELLOW1 41
#define GREEN1 40
#define RED2 37
#define YELLOW2 38
#define GREEN2 39
#define CROSSWALKLED 36

//  PIR
#define PIRUPDOWN 14       
#define PIRLEFTRIGHT 15    

#define BTNUPDOWN 12
#define BTNLEFTRIGHT 13

// TM1637 pins 
const int CLK = 2;
const int DIO = 4;
TM1637 display(CLK, DIO);

//  FreeRTOS primitives 
SemaphoreHandle_t crosswalkSem;
QueueHandle_t serialQ;            
TimerHandle_t trafficTimer;       
TimerHandle_t displayTimer;       

//  Shared state 
volatile unsigned long globalTime = 0UL; 
volatile int stateTime = 0;              

volatile bool pir1Detected = false;
volatile bool pir2Detected = false; 

volatile bool cwRequested = false; 
volatile int cwCountdown = 0;       

volatile int green1Time = 18; 
volatile int green2Time = 18;

// Debounce variables
volatile unsigned long lastBtn1Time = 0;
volatile unsigned long lastBtn2Time = 0;
const unsigned long DEBOUNCE_DELAY = 200; // 200ms debounce

enum State { INIT, G1, Y1, R1, G2, Y2, R2, CW };
volatile State state = INIT;

//  Helpers
void tmInit() {
  display.init();
  display.set(BRIGHT_TYPICAL); // Set brightness
}

// Show number 
void tmShowNumber(int num) {
  if (num < 0) num = 0;
  if (num > 9999) num = 9999;
  
  int8_t digits[4];
  digits[0] = (num / 1000) % 10;
  digits[1] = (num / 100) % 10;
  digits[2] = (num / 10) % 10;
  digits[3] = num % 10;
  
  display.display(digits);
}

// Clear 
void tmClear() {
  display.clearDisplay();
}

//  Serial gatekeeper queue 
void gatekeeperTask(void* pv) {
  char msg[96];
  while (true) {
    if (xQueueReceive(serialQ, &msg, portMAX_DELAY) == pdTRUE) {
      Serial.println(msg);
    }
  }
}

// copies into queue
void sendSerialMsgFmt(const char* fmt, ...) {
  char buf[96];
  va_list ap;
  va_start(ap, fmt);
  vsnprintf(buf, sizeof(buf), fmt, ap);
  va_end(ap);
  xQueueSend(serialQ, &buf, 0);
}

//  ISR handlers 
void IRAM_ATTR pir1_isr(void* arg) {
  pir1Detected = true;
}
void IRAM_ATTR pir2_isr(void* arg) {
  pir2Detected = true;
}
void IRAM_ATTR btn_up_isr(void* arg) {
  unsigned long currentTime = millis();
  // Debounce check - only trigger if enough time has passed
  if (currentTime - lastBtn1Time > DEBOUNCE_DELAY) {
    lastBtn1Time = currentTime;
    BaseType_t hpw = pdFALSE;
    xSemaphoreGiveFromISR(crosswalkSem, &hpw);
    portYIELD_FROM_ISR(hpw);
  }
}
void IRAM_ATTR btn_left_isr(void* arg) {
  unsigned long currentTime = millis();
  // Debounce check - only trigger if enough time has passed
  if (currentTime - lastBtn2Time > DEBOUNCE_DELAY) {
    lastBtn2Time = currentTime;
    BaseType_t hpw = pdFALSE;
    xSemaphoreGiveFromISR(crosswalkSem, &hpw);
    portYIELD_FROM_ISR(hpw);
  }
}

//  Crosswalk semaphore consumer task 
void crosswalkConsumerTask(void* pv) {
  while (true) {
    if (xSemaphoreTake(crosswalkSem, portMAX_DELAY) == pdTRUE) {
      // Only request if not already requested
      if (!cwRequested) {
        cwRequested = true;
        digitalWrite(CROSSWALKLED, HIGH);
        sendSerialMsgFmt("t=%lu; Crosswalk requested", globalTime);
      }
    }
  }
}

//  FSM logic 
void trafficTimerCallback(TimerHandle_t xTimer) {
  globalTime++;
  stateTime++;

  switch (state) {
    case INIT:
      // both red initially
      digitalWrite(RED1, HIGH); digitalWrite(YELLOW1, LOW); digitalWrite(GREEN1, LOW);
      digitalWrite(RED2, HIGH); digitalWrite(YELLOW2, LOW); digitalWrite(GREEN2, LOW);

      if (stateTime >= 2) {
        state = G1;
        stateTime = 0;
        green1Time = 18;
        sendSerialMsgFmt("t=%lu; Direction 1 goes green", globalTime);
      }
      break;

    case G1:
      // Dir1 green, Dir2 red
      digitalWrite(GREEN1, HIGH); digitalWrite(YELLOW1, LOW); digitalWrite(RED1, LOW);
      digitalWrite(RED2, HIGH); digitalWrite(YELLOW2, LOW); digitalWrite(GREEN2, LOW);

      // PIR detected shorten remaining green of dir1
      if (pir2Detected) {
        int rem = green1Time - stateTime; 
        if (rem > 8) {
          green1Time = stateTime + 8; 
          sendSerialMsgFmt("t=%lu; PIR Dir2 -> shorten Dir1 remaining green to %d s", globalTime, (green1Time - stateTime));
        }
        pir2Detected = false; 
      }

      if (stateTime >= green1Time) {
        state = Y1;
        stateTime = 0;
        sendSerialMsgFmt("t=%lu; Direction 1 goes yellow", globalTime);
      }
      break;

    case Y1:
      digitalWrite(GREEN1, LOW); digitalWrite(YELLOW1, HIGH); digitalWrite(RED1, LOW);
      digitalWrite(RED2, HIGH); digitalWrite(YELLOW2, LOW); digitalWrite(GREEN2, LOW);

      if (stateTime >= 4) {
        state = R1;
        stateTime = 0;
        sendSerialMsgFmt("t=%lu; Direction 1 goes red", globalTime);
      }
      break;

    case R1:
      // both red safety 2s
      digitalWrite(RED1, HIGH); digitalWrite(YELLOW1, LOW); digitalWrite(GREEN1, LOW);
      digitalWrite(RED2, HIGH); digitalWrite(YELLOW2, LOW); digitalWrite(GREEN2, LOW);

      if (stateTime >= 2) {
        if (cwRequested) {
          state = CW;
          stateTime = 0;
          cwCountdown = 20;
          sendSerialMsgFmt("t=%lu; Crosswalk active", globalTime);
        } else {
          state = G2;
          stateTime = 0;
          green2Time = 18;
          sendSerialMsgFmt("t=%lu; Direction 2 goes green", globalTime);
        }
      }
      break;

    case G2:
      // Dir2 green, Dir1 red
      digitalWrite(RED1, HIGH); digitalWrite(YELLOW1, LOW); digitalWrite(GREEN1, LOW);
      digitalWrite(GREEN2, HIGH); digitalWrite(YELLOW2, LOW); digitalWrite(RED2, LOW);

      // PIR detected shorten remaining green of dir2
      if (pir1Detected) {
        int rem = green2Time - stateTime;
        if (rem > 8) {
          green2Time = stateTime + 8;
          sendSerialMsgFmt("t=%lu; PIR Dir1 -> shorten Dir2 remaining green to %d s", globalTime, (green2Time - stateTime));
        }
        pir1Detected = false;
      }

      if (stateTime >= green2Time) {
        state = Y2;
        stateTime = 0;
        sendSerialMsgFmt("t=%lu; Direction 2 goes yellow", globalTime);
      }
      break;

    case Y2:
      // Dir2 yellow 4s
      digitalWrite(RED1, HIGH); digitalWrite(YELLOW1, LOW); digitalWrite(GREEN1, LOW);
      digitalWrite(GREEN2, LOW); digitalWrite(YELLOW2, HIGH); digitalWrite(RED2, LOW);

      if (stateTime >= 4) {
        state = R2;
        stateTime = 0;
        sendSerialMsgFmt("t=%lu; Direction 2 goes red", globalTime);
      }
      break;

    case R2:
      // both red safety 2s
      digitalWrite(RED1, HIGH); digitalWrite(YELLOW1, LOW); digitalWrite(GREEN1, LOW);
      digitalWrite(RED2, HIGH); digitalWrite(YELLOW2, LOW); digitalWrite(GREEN2, LOW);

      if (stateTime >= 2) {
        if (cwRequested) {
          state = CW;
          stateTime = 0;
          cwCountdown = 20;
          sendSerialMsgFmt("t=%lu; Crosswalk active", globalTime);
        } else {
          state = G1;
          stateTime = 0;
          green1Time = 18;
          sendSerialMsgFmt("t=%lu; Direction 1 goes green", globalTime);
        }
      }
      break;

    case CW:
      // both red
      digitalWrite(RED1, HIGH); digitalWrite(YELLOW1, LOW); digitalWrite(GREEN1, LOW);
      digitalWrite(RED2, HIGH); digitalWrite(YELLOW2, LOW); digitalWrite(GREEN2, LOW);

      // Decrement cwCountdown once per second
      if (cwCountdown > 0) cwCountdown--;

      if (stateTime >= 22) { 
        cwRequested = false;
        cwCountdown = 0;
        digitalWrite(CROSSWALKLED, LOW);
        state = G1;
        stateTime = 0;
        green1Time = 18;
        sendSerialMsgFmt("t=%lu; Direction 1 goes green", globalTime);
      }
      break;
  }
}

//  Display timer callback 
void displayTimerCallback(TimerHandle_t xTimer) {
  if (state == CW) {
    // Calculate remaining time based on stateTime
    int remaining = 20 - stateTime;
    if (remaining < 0) remaining = 0;
    tmShowNumber(remaining);
  } else {
    tmClear();
  }
}

//  Setup 
void setup() {
  Serial.begin(115200);
  delay(200);

  //  display
  tmInit();

  //  GPIOs
  pinMode(RED1, OUTPUT); pinMode(YELLOW1, OUTPUT); pinMode(GREEN1, OUTPUT);
  pinMode(RED2, OUTPUT); pinMode(YELLOW2, OUTPUT); pinMode(GREEN2, OUTPUT);
  pinMode(CROSSWALKLED, OUTPUT);
  digitalWrite(CROSSWALKLED, LOW);

  pinMode(PIRUPDOWN, INPUT);
  pinMode(PIRLEFTRIGHT, INPUT);

  pinMode(BTNUPDOWN, INPUT_PULLUP);
  pinMode(BTNLEFTRIGHT, INPUT_PULLUP);

  serialQ = xQueueCreate(10, 96);          
  crosswalkSem = xSemaphoreCreateBinary();

  xTaskCreate(gatekeeperTask, "gatekeeper", 2048, NULL, 3, NULL);

  //  consumer task
  xTaskCreate(crosswalkConsumerTask, "cw_consumer", 2048, NULL, 2, NULL);

  // Set up ISR system - CHANGED TO NEGEDGE (falling edge only)
  gpio_config_t io_conf = {};
  io_conf.intr_type = GPIO_INTR_NEGEDGE;  // Only trigger on button press (falling edge)
  io_conf.mode = GPIO_MODE_INPUT;
  io_conf.pin_bit_mask = (1ULL << BTNUPDOWN) | (1ULL << BTNLEFTRIGHT);
  io_conf.pull_up_en = GPIO_PULLUP_ENABLE;  // Enable internal pullup
  io_conf.pull_down_en = GPIO_PULLDOWN_DISABLE;
  gpio_config(&io_conf);

  // PIR sensors - keep ANYEDGE for motion detection
  gpio_config_t pir_conf = {};
  pir_conf.intr_type = GPIO_INTR_ANYEDGE;
  pir_conf.mode = GPIO_MODE_INPUT;
  pir_conf.pin_bit_mask = (1ULL << PIRUPDOWN) | (1ULL << PIRLEFTRIGHT);
  pir_conf.pull_up_en = GPIO_PULLUP_DISABLE;
  pir_conf.pull_down_en = GPIO_PULLDOWN_DISABLE;
  gpio_config(&pir_conf);

  gpio_install_isr_service(0);
  gpio_isr_handler_add((gpio_num_t)PIRUPDOWN, pir1_isr, NULL);
  gpio_isr_handler_add((gpio_num_t)PIRLEFTRIGHT, pir2_isr, NULL);
  gpio_isr_handler_add((gpio_num_t)BTNUPDOWN, btn_up_isr, NULL);
  gpio_isr_handler_add((gpio_num_t)BTNLEFTRIGHT, btn_left_isr, NULL);

  sendSerialMsgFmt("t=%lu; both direction red", globalTime);

  trafficTimer = xTimerCreate("traffic", pdMS_TO_TICKS(1000), pdTRUE, (void*)0, trafficTimerCallback);
  displayTimer = xTimerCreate("display", pdMS_TO_TICKS(250), pdTRUE, (void*)0, displayTimerCallback);

  // Start timers
  xTimerStart(trafficTimer, 0);
  xTimerStart(displayTimer, 0);

  // Initialize FSM values
  state = INIT;
  stateTime = 0;
  globalTime = 0;
  green1Time = 18;
  green2Time = 18;
}

//  loop 
void loop() {
}