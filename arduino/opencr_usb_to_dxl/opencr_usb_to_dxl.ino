#include <Dynamixel2Arduino.h>

// ===== 모터 설정 =====
const uint8_t DXL_ID = 5;
const uint32_t DXL_BAUDRATE = 1000000;
const float DXL_PROTOCOL_VERSION = 2.0;

// OpenRB-150의 내장 DYNAMIXEL 포트
Dynamixel2Arduino dxl(Serial1);

void setup()
{
  // PC와 연결되는 USB 시리얼
  Serial.begin(115200);

  // 시리얼 모니터가 열릴 때까지 잠시 대기
  delay(2000);

  // OpenRB의 DYNAMIXEL 전원 켜기
  pinMode(BDPIN_DXL_PWR_EN, OUTPUT);
  digitalWrite(BDPIN_DXL_PWR_EN, HIGH);

  delay(1000);

  // DYNAMIXEL 통신 시작
  dxl.begin(DXL_BAUDRATE);
  dxl.setPortProtocolVersion(DXL_PROTOCOL_VERSION);

  Serial.println();
  Serial.println("=== DYNAMIXEL ANGLE MONITOR ===");
  Serial.print("Motor ID: ");
  Serial.println(DXL_ID);
  Serial.print("Baudrate: ");
  Serial.println(DXL_BAUDRATE);

  // 모터 통신 확인
  if (dxl.ping(DXL_ID))
  {
    Serial.println("PING SUCCESS");

    // 손으로 축을 돌릴 수 있도록 토크 해제
    dxl.torqueOff(DXL_ID);

    Serial.println("Torque OFF");
    Serial.println("서보 축을 손으로 천천히 돌려보세요.");
    Serial.println("--------------------------------");
  }
  else
  {
    Serial.println("PING FAILED");
    Serial.println("ID, Baudrate, 전원, 케이블을 확인하세요.");
  }
}

void loop()
{
  // 현재 위치를 각도 단위로 읽음
  float angle = dxl.getPresentPosition(DXL_ID, UNIT_DEGREE);

  // 통신 성공 여부 확인
  if (dxl.getLastLibErrCode() == 0)
  {
    Serial.print("Current angle: ");
    Serial.print(angle, 2);
    Serial.println(" deg");
  }
  else
  {
    Serial.print("READ FAILED / Library error: ");
    Serial.println(dxl.getLastLibErrCode());
  }

  delay(200);
}