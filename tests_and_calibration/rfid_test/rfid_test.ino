/*
 * RC522 RFID TEST — Arduino Uno
 *
 * Wiring:
 * --------------------------------
 * RC522 SDA(SS) -> A1
 * RC522 SCK     -> D13
 * RC522 MOSI    -> 11
 * RC522 MISO    -> D12
 * RC522 RST     -> D8
 * RC522 VCC     -> 3.3V
 * RC522 GND     -> GND
 *
 * IMPORTANT:
 * This uses SOFTWARE SPI because
 * your pins are custom.
 */

#include <SPI.h>
#include <MFRC522.h>

// -------------------------------
// RC522 Pins
// -------------------------------

#define RFID_SS   A1
#define RFID_RST  8

#define RFID_MOSI 11
#define RFID_MISO 12
#define RFID_SCK  13

// -------------------------------
// Software SPI
// -------------------------------

MFRC522 mfrc522(
  RFID_SS,
  RFID_RST
);

void setup() {

  Serial.begin(115200);

  // Custom SPI pins
  SPI.begin();

  pinMode(RFID_MOSI, OUTPUT);
  pinMode(RFID_MISO, INPUT);
  pinMode(RFID_SCK, OUTPUT);

  // RC522 init
  mfrc522.PCD_Init();

  Serial.println();
  Serial.println("RC522 READY");
  Serial.println("Tap RFID Card...");
}

void loop() {

  // Detect card
  if (!mfrc522.PICC_IsNewCardPresent()) {
    return;
  }

  // Read card
  if (!mfrc522.PICC_ReadCardSerial()) {
    return;
  }

  Serial.print("CARD UID: ");

  for (byte i = 0; i < mfrc522.uid.size; i++) {

    if (mfrc522.uid.uidByte[i] < 0x10)
      Serial.print("0");

    Serial.print(mfrc522.uid.uidByte[i], HEX);
    Serial.print(" ");
  }

  Serial.println();

  // Halt PICC
  mfrc522.PICC_HaltA();

  delay(1000);
}