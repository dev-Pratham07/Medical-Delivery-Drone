#include <SPI.h>
#include <MFRC522.h>
#include <Servo.h>

// RFID pins
#define SS_PIN A1
#define RST_PIN 8

MFRC522 rfid(SS_PIN, RST_PIN);

Servo myServo;

// Authorized UID
byte authorizedUID[4] = {0x9D, 0xA4, 0x5F, 0x06};

void setup() {

  Serial.begin(115200);

  SPI.begin();

  rfid.PCD_Init();

  myServo.attach(7);

  // Initial position
  myServo.write(0);

  Serial.println("RFID System Ready");
}

void loop() {

  // Check for card
  if (!rfid.PICC_IsNewCardPresent())
    return;

  // Read card
  if (!rfid.PICC_ReadCardSerial())
    return;

  Serial.print("UID: ");

  for (byte i = 0; i < rfid.uid.size; i++) {

    Serial.print("0x");

    if (rfid.uid.uidByte[i] < 0x10)
      Serial.print("0");

    Serial.print(rfid.uid.uidByte[i], HEX);
    Serial.print(" ");
  }

  Serial.println();

  // Check UID
  bool authorized = true;

  for (byte i = 0; i < 4; i++) {

    if (rfid.uid.uidByte[i] != authorizedUID[i]) {

      authorized = false;
      break;
    }
  }

  // Access granted
  if (authorized) {

    Serial.println("ACCESS GRANTED");

    // Rotate servo to 90°
    myServo.write(90);

    delay(10000);

    // Return to 0°
    myServo.write(0);

    Serial.println("SERVO RESET");
  }
  else {

    Serial.println("ACCESS DENIED");
  }

  // Halt card
  rfid.PICC_HaltA();

  // Stop encryption
  rfid.PCD_StopCrypto1();
}