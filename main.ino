/**
 * kegboard-particle
 */

#define REPORT_INTERVAL_MILLIS 1000

#define NUM_METERS 4
#define METER0_PIN D1
#define METER1_PIN D2
#define METER2_PIN D3
#define METER3_PIN D4

// Time of last publication
unsigned long lastReportMillis;

typedef struct {
  volatile unsigned int ticks;
  unsigned int lastPublishedTicks;
} meter_t;

meter_t meters[NUM_METERS];

#define CREATE_METER_ISR(METER_NUM) \
  void meter##METER_NUM##Interrupt(void) { \
    detachInterrupt(METER##METER_NUM##_PIN); \
    meters[METER_NUM].ticks++; \
    attachInterrupt(METER##METER_NUM##_PIN, meter##METER_NUM##Interrupt, FALLING); \
  }

#define SETUP_METER(METER_NUM) \
  do { \
    memset(&meters[METER_NUM], 0, sizeof(meter_t)); \
    pinMode(METER##METER_NUM##_PIN, INPUT_PULLUP); \
    attachInterrupt(METER##METER_NUM##_PIN, meter##METER_NUM##Interrupt, FALLING); \
  } while (false)

CREATE_METER_ISR(0);
CREATE_METER_ISR(1);
CREATE_METER_ISR(2);
CREATE_METER_ISR(3);

//
// Main program
//

int resetMeter(int meterNum) {
  if (meterNum < 0 || meterNum >= NUM_METERS) {
    return -1;
  }
  memset(&meters[meterNum], 0, sizeof(meter_t));
  return 0;
}

int publicResetMeter(String extra) {
  int meterNum = atoi(extra);
  return resetMeter(meterNum);
}

int publicMeterTicks(String extra) {
  int meterNum = atoi(extra);
  if (meterNum < 0 || meterNum >= NUM_METERS) {
    return -1;
  }
  return (int) meters[meterNum].ticks;
}

void setup() {
  Serial.begin(115200);
  Serial.println("start: kegboard-particle online!");

  SETUP_METER(0);
  SETUP_METER(1);
  SETUP_METER(2);
  SETUP_METER(3);

  Particle.function("resetMeter", publicResetMeter);
  Particle.function("meterTicks", publicMeterTicks);
}

void publishStatus() {
  String statusMessage = "";
  bool changed = false;

  for (int i = 0; i < NUM_METERS; i++) {
    meter_t *meter = &meters[i];
    unsigned int ticks = meter->ticks;
    unsigned int delta = ticks - meter->lastPublishedTicks;
    meter->lastPublishedTicks = ticks;

    if (delta > 0) {
      changed = true;
    }

    statusMessage.concat(String::format("meter%i.ticks=%u meter%i.delta=%u ",
        i, ticks, i, delta));
  }

  if (!changed) {
    return;
  }

  Serial.print("kb-status: ");
  Serial.println(statusMessage);
  Particle.publish("kb-status", statusMessage, PRIVATE);
}

void loop() {
  unsigned long now = millis();
  unsigned long delta = now - lastReportMillis;

  if (delta < REPORT_INTERVAL_MILLIS) {
    return;
  }

  lastReportMillis = now;
  publishStatus();
}
