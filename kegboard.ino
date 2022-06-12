/**
 * Copyright 2016-2020 The Kegbot Project contributors <https://kegbot.org/>
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

SYSTEM_THREAD(ENABLED);

#define VERSION "0.3.0"
#define KEGBOARD_DEBUG 0

#define CLOUD_PUBLISH_INTERVAL_MILLIS 10000
#define TCP_PUBLISH_INTERVAL_MILLIS 250
#define CONSOLE_PUBLISH_INTERVAL_MILLIS 250
#define CLIENT_WATCHDOG_TIMEOUT_MILLIS 30000

#define COMMAND_WATCHDOG_ON "watchdog on"
#define COMMAND_WATCHDOG_OFF "watchdog off"
#define COMMAND_WATCHDOG_KICK "watchdog kick"

#define NUM_METERS 4
#define METER0_PIN D1
#define METER1_PIN D2
#define METER2_PIN D3
#define METER3_PIN D4

#define ONEWIRE_PIN D5

#define TCP_SERVER_PORT 8321
#define TCP_CLIENT_INCOMING_BUFSIZE 256

#include "MDNS.h"
#include "OneWire.h"
#include "ds1820.h"
#include "KegboardPacket.h"

#define FIRMWARE_VERSION 18
#define SERIAL_NUMBER_SIZE_BYTES 30
static uint8_t gSerialNumber[SERIAL_NUMBER_SIZE_BYTES] = { 75,66,45,48,48,48,48,45,48,48,48,48,45,48,48,48,51,49,51,51,55,0 };
// Structure to keep information about this device's uptime.
typedef struct {
  unsigned long uptime_ms;
  unsigned long uptime_days;
  unsigned long last_uptime_ms;
  unsigned long last_meter_event;
  unsigned long last_heartbeat;
} UptimeStat;

static UptimeStat gUptimeStat;

static KegboardPacket gInputPacket;

// Structure that holds the state of incoming serial bytes.
typedef struct {
  uint8_t header_bytes_read;
  uint8_t payload_bytes_remain;
  bool have_packet;
} RxPacketStat;

static RxPacketStat gPacketStat;

#if KEGBOARD_DEBUG
SerialLogHandler logHandler;
#endif

TCPServer server = TCPServer(TCP_SERVER_PORT);
TCPClient client;

char clientBuffer[TCP_CLIENT_INCOMING_BUFSIZE] = { '\0' };
unsigned int clientBufferPos = 0;

mdns::MDNS mdnsImpl;
bool mdnsRunning = false;

volatile unsigned int watchdogEnabled = 0;
unsigned long lastWatchdogKick;

volatile unsigned int meterPending;
volatile unsigned int thermoPending;

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

typedef struct {
  float temp;
  char probe[17];
} temp_t;

temp_t temps[NUM_METERS];

#define CREATE_METER_ISR(METER_NUM) \
  void meter##METER_NUM##Interrupt(void) { \
    detachInterrupt(METER##METER_NUM##_PIN); \
    meters[METER_NUM].ticks++; \
    cloudPending = consolePending = tcpPending = meterPending = 1; \
    attachInterrupt(METER##METER_NUM##_PIN, meter##METER_NUM##Interrupt, FALLING); \
  }

#define SETUP_METER(METER_NUM) \
  do { \
    memset(&meters[METER_NUM], 0, sizeof(meter_t)); \
    pinMode(METER##METER_NUM##_PIN, INPUT_PULLUP); \
    attachInterrupt(METER##METER_NUM##_PIN, meter##METER_NUM##Interrupt, FALLING); \
  } while (false)

#define SETUP_TEMP(TEMP_NUM) \
  do { \
    memset(&temps[TEMP_NUM], 0, sizeof(temp_t)); \
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

void enableClientWatchdog() {
  lastWatchdogKick = millis();
  watchdogEnabled = 1;
}

void disableClientWatchdog() {
  watchdogEnabled = 0;
}

void kickClientWatchdog() {
  if (!watchdogEnabled) {
    enableClientWatchdog();
  }
  lastWatchdogKick = millis();
}

void checkClientWatchdog() {
  if (!watchdogEnabled) {
    return;
  }
  if (!client.connected()) {
    return;
  }
  if ((millis() - lastWatchdogKick) >= CLIENT_WATCHDOG_TIMEOUT_MILLIS) {
    Log.info("Watchdog expired");
    client.println("error: watchdog expired");
    client.stop();
    watchdogEnabled = 0;
  }
}

bool setupMdns() {
  std::vector<String> subServices;
  subServices.push_back("kegboard");

  bool success = mdnsImpl.setHostname("kegboard") &&
    mdnsImpl.addService("tcp", "kegboard", TCP_SERVER_PORT, "Kegboard") &&
    mdnsImpl.begin(true);

  return success;
}

void addTemperature(char* probe, float temp) {
  for (int i = 0; i < NUM_METERS; i++) {
   if (strcmp(temps[i].probe, probe) == 0) {
      temps[i].temp = temp;
      return;
    }
  }
  // New probe ID; Store it.
  for (int i = 0; i < NUM_METERS; i++) {
    if (temps[i].probe[0] == '\0') {
      std::copy(probe, probe+17, temps[i].probe);
      temps[i].temp = temp;
      return;
    }
  }
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
        addTemperature(nameBuf,atof(buf));
        cloudPending = consolePending = tcpPending = thermoPending = 1;
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

  server.begin();

  SETUP_METER(0);
  SETUP_METER(1);
  SETUP_METER(2);
  SETUP_METER(3);

  SETUP_TEMP(0);
  SETUP_TEMP(1);
  SETUP_TEMP(2);
  SETUP_TEMP(3);

  Particle.function("resetMeter", publicResetMeter);
  Particle.function("meterTicks", publicMeterTicks);

  mdnsRunning = setupMdns();

  memset(&gUptimeStat, 0, sizeof(UptimeStat));
  memset(&gPacketStat, 0, sizeof(RxPacketStat));
}

void getStatus(String* statusMessage) {
  for (int i = 0; i < NUM_METERS; i++) {
    meter_t *meter = &meters[i];
    unsigned int ticks = meter->ticks;
    statusMessage->concat(String::format("meter%i.ticks=%u ", i, ticks));
  }
}

void getThermoStatus(String* statusMessage) {
  for (int i = 0; i < NUM_METERS; i++) {
    if (temps[i].probe[0] != '\0') {
      statusMessage->concat(String::format("temp_%s.temp=%f ", temps[i].probe, temps[i].temp));
    }
  }
}

void handleCommand(char *command) {
  Log.info("Handling command: \"%s\" ...", command);
  String commandString = String(command);
  if (commandString.equals(COMMAND_WATCHDOG_ON)) {
    Log.info("Enabling watchdog.");
    enableClientWatchdog();
  } else if (commandString.equals(COMMAND_WATCHDOG_OFF)) {
    Log.info("Disabling watchdog.");
    disableClientWatchdog();
  } else if (commandString.equals(COMMAND_WATCHDOG_KICK)) {
    Log.info("Watchdog kicked.");
    kickClientWatchdog();
  } else {
    Log.info("Unknown command");
  }
}

void serviceCurrentClient() {
  if (!client.connected()) {
    return;
  }

  while (client.available()) {
    char c = client.read();
    if ((int)c == -1) {
      break;
    }

    // Always ignore CRs.
    if (c == '\r') {
      continue;
    }

    // Consume and ignore leading whitespace.
    if (clientBufferPos == 0) {
      if (c == '\n' || c == ' ') {
        continue;
      }
    }

    clientBuffer[clientBufferPos++] = c;

    // We've got a complete command.
    if (c == '\n') {
      break;
    }

    // Haven't received a complete command & about to overflow.
    if (clientBufferPos == (TCP_CLIENT_INCOMING_BUFSIZE - 1)) {
      Log.info("Incoming buffer overflow");
      clientBufferPos = 0;
    }
  }

  if (clientBufferPos > 0 && clientBuffer[clientBufferPos - 1] == '\n') {
    clientBuffer[clientBufferPos - 1] = '\0';
    handleCommand(clientBuffer);
    clientBufferPos = 0;
    clientBuffer[0] = '\0';
  }
}

void checkAndServiceTcp() {
  // Check for a new client.
  if (!client.connected()) {
    client = server.available();
    if (client.connected()) {
      Log.info("TCP client connectd");
      watchdogEnabled = 0;
      client.print("info: kegboard-particle device_id=");
      client.print(System.deviceID());
      client.print(" version=");
      client.print(VERSION);
      client.println();
    }
  }

  if (client.connected()) {
    serviceCurrentClient();
  }
}

void publishCloudStatus() {
  if (!cloudPending) {
    return;
  }
  cloudPending = 0;

  String statusMessage;
  if (meterPending) {
    getStatus(&statusMessage);
    bool published = Particle.publish("kb-status", statusMessage, PRIVATE);
    if (published) {
      lastCloudPublishMillis = millis();
    }
  }
  if (thermoPending) {
    statusMessage = "";
    getThermoStatus(&statusMessage);
    bool published = Particle.publish("kb-thermo", statusMessage, PRIVATE);
    if (published) {
      lastCloudPublishMillis = millis();
    }
  }
}

void publishTcpStatus() {
  if (!tcpPending) {
    return;
  }
  tcpPending = 0;

  if (client.connected()) {
    String statusMessage;
    if (meterPending) {
      getStatus(&statusMessage);
      client.print("kb-status: ");
      client.println(statusMessage);
    }
    if (thermoPending) {
      getThermoStatus(&statusMessage);
      client.print("kb-thermo: ");
      client.println(statusMessage);
    }
  }
  lastTcpPublishMillis = millis();
}

void publishConsoleStatus() {
  if (!consolePending) {
    return;
  }
  consolePending = 0;

  if (meterPending) {
    writeMeterPacket(0);
  }
  if (thermoPending) {
    for (int i = 0; i < NUM_METERS; i++) {
      if (temps[i].probe[0] != '\0') {
        writeThermoPacket(temps[i].probe, temps[i].temp);
      }
    }
  }
  lastConsolePublishMillis = millis();
}

void writeHelloPacket()
{
  int firmware_version = FIRMWARE_VERSION;
  KegboardPacket packet;
  packet.SetType(KBM_HELLO_ID);
  packet.AddTag(KBM_HELLO_TAG_FIRMWARE_VERSION, sizeof(firmware_version), (char*)&firmware_version);
  packet.AddTag(KBM_HELLO_TAG_SERIAL_NUMBER, SERIAL_NUMBER_SIZE_BYTES, (char*)gSerialNumber);
  packet.AddTag(KBM_HELLO_TAG_UPTIME_MILLIS, sizeof(gUptimeStat.uptime_ms), (char*)&gUptimeStat.uptime_ms);
  packet.AddTag(KBM_HELLO_TAG_UPTIME_DAYS, sizeof(gUptimeStat.uptime_days), (char*)&gUptimeStat.uptime_days);
  packet.Print();
}

void writeThermoPacket(char* name, long temp)
{
  char thermo_name[23] = "thermo-";
  strcat(thermo_name, name);
  KegboardPacket packet;
  packet.SetType(KBM_THERMO_READING);
  packet.AddTag(KBM_THERMO_READING_TAG_SENSOR_NAME, 23, thermo_name);
  packet.AddTag(KBM_THERMO_READING_TAG_SENSOR_READING, sizeof(temp), (char*)(&temp));
  packet.Print();
}

void writeMeterPacket(int channel)
{
  char name[5] = "flow";
  meter_t *meter = &meters[channel];
  unsigned int ticks = meter->ticks;

  if (ticks <= 0) {
    return;
  }

  name[4] = 0x30 + channel;
  KegboardPacket packet;
  packet.SetType(KBM_METER_STATUS);
  packet.AddTag(KBM_METER_STATUS_TAG_METER_NAME, 5, name);
  packet.AddTag(KBM_METER_STATUS_TAG_METER_READING, sizeof(ticks), (char*)(&ticks));
  packet.Print();
}

void updateTimekeeping() {
  // TODO(mikey): it would be more efficient to take control of timer0
  unsigned long now = millis();
  gUptimeStat.uptime_ms += now - gUptimeStat.last_uptime_ms;
  gUptimeStat.last_uptime_ms = now;

  if (gUptimeStat.uptime_ms >= MS_PER_DAY) {
    gUptimeStat.uptime_days += 1;
    gUptimeStat.uptime_ms -= MS_PER_DAY;
  }

  if ((now - gUptimeStat.last_heartbeat) > KB_HEARTBEAT_INTERVAL_MS) {
    gUptimeStat.last_heartbeat = now;
    writeHelloPacket();
  }
}

static void readSerialBytes(char *dest_buf, int num_bytes, int offset) {
  while (num_bytes-- != 0) {
    dest_buf[offset++] = Serial.read();
  }
}

void resetInputPacket() {
  memset(&gPacketStat, 0, sizeof(RxPacketStat));
  gInputPacket.Reset();
}

void readIncomingSerialData() {
  char serial_buf[KBSP_PAYLOAD_MAXLEN];
  volatile uint8_t bytes_available = Serial.available();

  // Do not read a new packet if we have one awiting processing.  This should
  // never happen.
  if (gPacketStat.have_packet) {
    return;
  }

  // Look for a new packet.
  if (gPacketStat.header_bytes_read < KBSP_HEADER_PREFIX_LEN) {
    while (bytes_available > 0) {
      char next_char = Serial.read();
      bytes_available -= 1;

      if (next_char == KBSP_PREFIX[gPacketStat.header_bytes_read]) {
        gPacketStat.header_bytes_read++;
        if (gPacketStat.header_bytes_read == KBSP_HEADER_PREFIX_LEN) {
          // Found start of packet, break.
          break;
        }
      } else {
        // Wrong character in prefix; reset framing.
        if (next_char == KBSP_PREFIX[0]) {
          gPacketStat.header_bytes_read = 1;
        } else {
          gPacketStat.header_bytes_read = 0;
        }
      }
    }
  }

  // Read the remainder of the header, if not yet found.
  if (gPacketStat.header_bytes_read < KBSP_HEADER_LEN) {
    if (bytes_available < 4) {
      return;
    }
    gInputPacket.SetType(Serial.read() | (Serial.read() << 8));
    gPacketStat.payload_bytes_remain = Serial.read() | (Serial.read() << 8);
    bytes_available -= 4;
    gPacketStat.header_bytes_read += 4;

    // Check that the 'len' field is not bogus. If it is, throw out the packet
    // and reset.
    if (gPacketStat.payload_bytes_remain > KBSP_PAYLOAD_MAXLEN) {
      goto out_reset;
    }
  }

  // If we haven't yet found a frame, or there are no more bytes to read after
  // finding a frame, bail out.
  if (bytes_available == 0 || (gPacketStat.header_bytes_read < KBSP_HEADER_LEN)) {
    return;
  }

  // TODO(mikey): Just read directly into KegboardPacket.
  if (gPacketStat.payload_bytes_remain) {
    int bytes_to_read = (gPacketStat.payload_bytes_remain >= bytes_available) ?
        bytes_available : gPacketStat.payload_bytes_remain;
    readSerialBytes(serial_buf, bytes_to_read, 0);
    gInputPacket.AppendBytes(serial_buf, bytes_to_read);
    gPacketStat.payload_bytes_remain -= bytes_to_read;
    bytes_available -= bytes_to_read;
  }

  // Need more payload bytes than are now available.
  if (gPacketStat.payload_bytes_remain > 0) {
    return;
  }

  // We have a complete payload. Now grab the footer.
  if (!gPacketStat.have_packet) {
    if (bytes_available < KBSP_FOOTER_LEN) {
      return;
    }
    readSerialBytes(serial_buf, KBSP_FOOTER_LEN, 0);

    // Check CRC

    // Check trailer
    if (strncmp((serial_buf + 2), KBSP_TRAILER, KBSP_FOOTER_TRAILER_LEN)) {
      goto out_reset;
    }
    gPacketStat.have_packet = true;
  }

  // Done!
  return;

out_reset:
  resetInputPacket();
}

void handleInputPacket() {
  if (!gPacketStat.have_packet) {
    return;
  }

  // Process the input packet.
  switch (gInputPacket.GetType()) {
    case KBM_PING:
#if KB_ENABLE_BUZZER
      play_notes(PING_MELODY, KB_PIN_BUZZER);
#endif
      writeHelloPacket();
      break;

    case KBM_SET_OUTPUT: {
      uint8_t id, mode;

      if (!gInputPacket.ReadTag(KBM_SET_OUTPUT_TAG_OUTPUT_ID, &id)
          || !gInputPacket.ReadTag(KBM_SET_OUTPUT_TAG_OUTPUT_MODE, &mode)) {
        break;
      }

      break;
    }

    case KBM_SET_SERIAL_NUMBER: {
      // Serial number can only be set if not already set.
      if (true) {
        break;
      }

      if (gInputPacket.FindTagLength(KBM_SET_SERIAL_NUMBER_TAG_SERIAL) >= SERIAL_NUMBER_SIZE_BYTES) {
        break;
      }

      memset(gSerialNumber, 0, SERIAL_NUMBER_SIZE_BYTES);
      gInputPacket.CopyTagData(KBM_SET_SERIAL_NUMBER_TAG_SERIAL, gSerialNumber);
      writeHelloPacket();

      break;
    }
  }
  resetInputPacket();
}

void loop() {
  updateTimekeeping();
  readIncomingSerialData();
  handleInputPacket();
  checkClientWatchdog();
  checkAndServiceTcp();
  stepOnewireThermoBus();

  if (mdnsRunning) {
    mdnsImpl.processQueries();
  }

  if (client.connected()) {
    if ((millis() - lastTcpPublishMillis) >= TCP_PUBLISH_INTERVAL_MILLIS) {
      publishTcpStatus();
    }
  }
  if ((millis() - lastCloudPublishMillis) >= CLOUD_PUBLISH_INTERVAL_MILLIS) {
    publishCloudStatus();
  }

  if ((millis() - lastConsolePublishMillis) >= CONSOLE_PUBLISH_INTERVAL_MILLIS) {
    publishConsoleStatus();
  }
  meterPending = thermoPending = 0;
}
