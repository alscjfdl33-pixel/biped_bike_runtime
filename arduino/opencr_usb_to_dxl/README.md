#include <Dynamixel2Arduino.h>

// 실제 모터 설정에 맞게 수정
const uint8_t DXL_ID = 5;
const uint32_t DXL_BAUDRATE = 1000000;
const float DXL_PROTOCOL_VERSION = 2.0;

// OpenRB-150의 DYNAMIXEL 포트
Dynamixel2Arduino dxl(Serial1);

void setup()
{
  Serial.begin(115200);
  while (!Serial);

  // OpenRB-150 DYNAMIXEL 전원 활성화
  pinMode(BDPIN_DXL_PWR_EN, OUTPUT);
  digitalWrite(BDPIN_DXL_PWR_EN, HIGH);
  delay(1000);

  // DYNAMIXEL 통신 시작
  dxl.begin(DXL_BAUDRATE);
  dxl.setPortProtocolVersion(DXL_PROTOCOL_VERSION);

  Serial.println("DYNAMIXEL angle monitor");
  Serial.print("ID: ");
  Serial.println(DXL_ID);

  // 통신 확인
  if (!dxl.ping(DXL_ID))
  {
    Serial.println("PING FAILED");
    Serial.print("Library error code: ");
    Serial.println(dxl.getLastLibErrCode());

    while (true)
    {
      delay(1000);
    }
  }

  Serial.println("PING SUCCESS");

  // 반드시 토크를 꺼야 손으로 돌릴 수 있음
  dxl.torqueOff(DXL_ID);

  Serial.println("Torque OFF");
  Serial.println("서보 축을 손으로 천천히 돌려보세요.");
  Serial.println();
}

void loop()
{
  float angle = dxl.getPresentPosition(DXL_ID, UNIT_DEGREE);
  uint16_t error = dxl.getLastLibErrCode();

  if (error == 0)
  {
    Serial.print("Current angle: ");
    Serial.print(angle, 2);
    Serial.println(" deg");
  }
  else
  {
    Serial.print("READ FAILED / Error code: ");
    Serial.println(error);
  }

  delay(200);
}