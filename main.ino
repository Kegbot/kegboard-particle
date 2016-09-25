/**
 * kegboard-particle
 *
 * Copyright (c) 2016 Bevbot LLC.
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 */

#include "OneWire.h"
#include "ds1820.h"

#define VERSION "0.1.0"

#define CLOUD_PUBLISH_INTERVAL_MILLIS 1000
#define TCP_PUBLISH_INTERVAL_MILLIS 250
#define CONSOLE_PUBLISH_INTERVAL_MILLIS 250

#define NUM_METERS 4
#define METER0_PIN D1
#define METER1_PIN D2
#define METER2_PIN D3
#define METER3_PIN D4

#define ONEWIRE_PIN D5

#define TCP_SERVER_PORT 8321

TCPServer server = TCPServer(TCP_SERVER_PORT);
TCPClient client;

volatile unsigned int cloudPending;
unsigned long lastCloudPublishMillis;

volatile unsigned int consolePending;
unsigned long lastConsolePublishMillis;

volatile unsigned int tcpPending;
unsigned long lastTcpPublishMillis;

typedef struct {
  volatile unsigned int ticks;
} meter_t;

meter_t meters[NUM_METERS];

OneWire onewire = OneWire(ONEWIRE_PIN);
DS1820Sensor thermoSensor = DS1820Sensor();

#define CREATE_METER_ISR(METER_NUM) \
  void meter##METER_NUM##Interrupt(void) { \
    detachInterrupt(METER##METER_NUM##_PIN); \
    meters[METER_NUM].ticks++; \
    cloudPending = consolePending = tcpPending = 1; \
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

int stepOnewireThermoBus() {
  uint8_t addr[8];
  unsigned long now = millis();

  // Are we already working on a sensor? service it, possibly emitting a a
  // thermo packet.
  if (thermoSensor.Initialized() || thermoSensor.Busy()) {
    if (thermoSensor.Update(now)) {
      char buf[64];
      char nameBuf[17];
      // Just finished conversion
      //writeThermoPacket(&gThermoSensor);
      thermoSensor.GetTempC(buf);
      thermoSensor.GetName(nameBuf);
      if (buf[0] != '\0') {
        Serial.print("THERMO id=");
        Serial.print(nameBuf);
        Serial.print(" temp_c=");
        Serial.println(buf);
      }
      thermoSensor.Reset();
    } else if (thermoSensor.Busy()) {
      // More cycles needed on this sensor
      return 1;
    } else {
      // finished or not started
    }

    // First time, or finished with last sensor; clean up, and look more more
    // devices.
    int more_search = onewire.search(addr);
    if (!more_search) {
      // Bus exhausted; start over
      onewire.reset_search();
      return 0;
    }
    // New sensor. Initialize and start work.
    thermoSensor.Initialize(&onewire, addr);
    thermoSensor.Update(now);
    return 1;
  }

  // First time, or finished with last sensor; clean up, and look more more
  // devices.
  int more_search = onewire.search(addr);
  if (!more_search) {
    // Bus exhausted; start over
    onewire.reset_search();
    return 0;
  }

  // New sensor. Initialize and start work.
  thermoSensor.Initialize(&onewire, addr);
  thermoSensor.Update(now);
  return 1;
}
void setup() {
  Serial.begin(115200);
  Serial.print("start: kegboard-particle online, ip: ");
  Serial.println(WiFi.localIP());

  server.begin();

  SETUP_METER(0);
  SETUP_METER(1);
  SETUP_METER(2);
  SETUP_METER(3);

  Particle.function("resetMeter", publicResetMeter);
  Particle.function("meterTicks", publicMeterTicks);
}

void getStatus(String* statusMessage) {
  for (int i = 0; i < NUM_METERS; i++) {
    meter_t *meter = &meters[i];
    unsigned int ticks = meter->ticks;

    statusMessage->concat(String::format("meter%i.ticks=%u ", i, ticks));
  }
}

void checkForTcpClient() {
  if (!client.connected()) {
    client = server.available();
    if (client.connected()) {
      client.print("info: kegboard-particle device_id=");
      client.print(System.deviceID());
      client.print(" version=");
      client.print(VERSION);
      client.println();
    }
  }
}

void publishCloudStatus() {
  if (!cloudPending) {
    return;
  }
  cloudPending = 0;

  String statusMessage;
  getStatus(&statusMessage);

  bool published = Particle.publish("kb-status", statusMessage, PRIVATE);
  if (published) {
    lastCloudPublishMillis = millis();
  }
}

void publishTcpStatus() {
  if (!tcpPending) {
    return;
  }
  tcpPending = 0;

  if (client.connected()) {
    String statusMessage;
    getStatus(&statusMessage);
    client.print("kb-status: ");
    client.println(statusMessage);
  }
  lastTcpPublishMillis = millis();
}

void publishConsoleStatus() {
  if (!consolePending) {
    return;
  }
  consolePending = 0;

  String statusMessage;
  getStatus(&statusMessage);

  Serial.print("kb-status: ");
  Serial.println(statusMessage);
  lastConsolePublishMillis = millis();
}

void loop() {
  checkForTcpClient();
  stepOnewireThermoBus();

  if (client.connected()) {
    if ((millis() - lastTcpPublishMillis) >= TCP_PUBLISH_INTERVAL_MILLIS) {
      publishTcpStatus();
    }
  } else {
    if ((millis() - lastCloudPublishMillis) >= CLOUD_PUBLISH_INTERVAL_MILLIS) {
      publishCloudStatus();
    }
  }

  if ((millis() - lastConsolePublishMillis) >= CONSOLE_PUBLISH_INTERVAL_MILLIS) {
    publishConsoleStatus();
  }

}
