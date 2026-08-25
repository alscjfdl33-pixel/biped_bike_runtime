// Read-only DYNAMIXEL position monitor for the OpenRB-150.
//
// This sketch does not enable torque and does not write goal positions or
// configuration values. It only pings IDs 1-6 and reads Present Position.

#include <Dynamixel2Arduino.h>

#define DXL_SERIAL Serial1
#define DEBUG_SERIAL Serial

constexpr int DXL_DIR_PIN = -1;
constexpr uint32_t DXL_BAUDRATE = 1000000;
constexpr float DXL_PROTOCOL_VERSION = 2.0;
constexpr uint32_t PRINT_INTERVAL_MS = 500;
constexpr uint8_t DXL_IDS[] = {1, 2, 3, 4, 5, 6};

Dynamixel2Arduino dxl(DXL_SERIAL, DXL_DIR_PIN);

void setup()
{
  DEBUG_SERIAL.begin(115200);

  // Do not wait forever when the Serial Monitor is closed.
  const uint32_t wait_started_ms = millis();
  while (!DEBUG_SERIAL && (millis() - wait_started_ms < 3000)) {
  }

  dxl.begin(DXL_BAUDRATE);
  dxl.setPortProtocolVersion(DXL_PROTOCOL_VERSION);

  DEBUG_SERIAL.println();
  DEBUG_SERIAL.println("OpenRB-150 read-only position monitor");
  DEBUG_SERIAL.println("DXL: Protocol 2.0, 1000000 bps, IDs 1-6");
  DEBUG_SERIAL.println("No torque or goal-position commands are sent.");

  for (const uint8_t id : DXL_IDS) {
    DEBUG_SERIAL.print("Ping ID ");
    DEBUG_SERIAL.print(id);
    if (dxl.ping(id)) {
      DEBUG_SERIAL.print(": OK, model=");
      DEBUG_SERIAL.println(dxl.getModelNumber(id));
    } else {
      DEBUG_SERIAL.print(": FAILED, error=");
      DEBUG_SERIAL.println(dxl.getLastLibErrCode());
    }
  }
}

void loop()
{
  static uint32_t last_printed_ms = 0;
  const uint32_t now_ms = millis();
  if (now_ms - last_printed_ms < PRINT_INTERVAL_MS) {
    return;
  }
  last_printed_ms = now_ms;

  DEBUG_SERIAL.print("t=");
  DEBUG_SERIAL.print(now_ms);
  DEBUG_SERIAL.println(" ms");

  for (const uint8_t id : DXL_IDS) {
    const float raw_position = dxl.getPresentPosition(id, UNIT_RAW);
    const int32_t raw_error = dxl.getLastLibErrCode();

    DEBUG_SERIAL.print("  ID ");
    DEBUG_SERIAL.print(id);
    if (raw_error != DXL_LIB_OK) {
      DEBUG_SERIAL.print(": READ FAILED, error=");
      DEBUG_SERIAL.println(raw_error);
      continue;
    }

    const float degree_position = dxl.getPresentPosition(id, UNIT_DEGREE);
    const int32_t degree_error = dxl.getLastLibErrCode();
    if (degree_error != DXL_LIB_OK) {
      DEBUG_SERIAL.print(": raw=");
      DEBUG_SERIAL.print(raw_position, 0);
      DEBUG_SERIAL.print(", degree READ FAILED, error=");
      DEBUG_SERIAL.println(degree_error);
      continue;
    }

    DEBUG_SERIAL.print(": raw=");
    DEBUG_SERIAL.print(raw_position, 0);
    DEBUG_SERIAL.print(", degree=");
    DEBUG_SERIAL.println(degree_position, 2);
  }

  DEBUG_SERIAL.println();
}
