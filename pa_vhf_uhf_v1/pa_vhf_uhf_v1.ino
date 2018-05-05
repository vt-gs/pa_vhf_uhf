/*
  VHF/UHF Power Amplifier Control Board
  See link for Eagle schematics and board
  https://github.com/vt-gs/eagle/tree/master/pa_vhf_uhf

  
*/
#include "Arduino.h"
#include <Ethernet2.h>
#include <SPI.h>
#include <Wire.h>
#include <Adafruit_ADS1015.h>
#include <Thanos_INA260.h>


//****Custom Board Pinouts*********************************
//OUTPUT
#define LED_PIN       7
#define VHF_PTT       4
#define UHF_PTT       12
#define PA_FAN        A2
#define VHF_COAX_REL  A0
#define UHF_COAX_REL  A1
#define PA_PWR_REL    13
#define WIZ_CS        11

//INPUT
#define BTN_RED       10
#define BTN_BLK       5
#define ALERT_I       8
#define ALERT_T       9
#define THERMO        0
//*********************************************************************

//****Instantiate Peripheral Objects*********************************
  Adafruit_ADS1115 ads1(0x48);  /* Use this for the 16-bit version */
  Adafruit_ADS1115 ads2(0x49);  /* Use this for the 16-bit version */
  Thanos_INA260 ina260;
  #define lcd Serial1 //LCD connected to Serial1, hope it doesn't conflict with the Current Alert on RXD pin!
  
  // Enter a MAC address and IP address for your controller below.
  // The IP address will be dependent on your local network:
  byte mac[] = { 0xDE, 0xAD, 0xBE, 0xEF, 0xFE, 0xED };
  // assign an IP address for the controller:
  // network configuration.  gateway and subnet are optional.
  //byte mac[] = {0xDE, 0xAD, 0xBE, 0xEF, 0xFE, 0xED};  
  //byte ip[] = {192, 168, 20, 80};  
  IPAddress ip(192, 168, 20, 80);  
  byte gateway[] = {192, 168, 20, 1};
  byte subnet[] = {255, 255, 255, 0};
  unsigned int localPort = 2000;      // local port to listen on
  EthernetServer server(localPort);
//*********************************************************************

//****Global Variable Definition*********************************
  String serIn;   // stores incoming serial commands
  String netIn; //stores incoming network commands
  #define SERIAL_BAUD   9600 
//*********************************************************************

void setup() {
  // initialize digital pin outputs.
  pinMode(LED_PIN, OUTPUT);
  pinMode(VHF_PTT, OUTPUT);
  pinMode(UHF_PTT, OUTPUT);
  pinMode(PA_FAN, OUTPUT);
  pinMode(VHF_COAX_REL, OUTPUT);
  pinMode(UHF_COAX_REL, OUTPUT);
  pinMode(PA_PWR_REL, OUTPUT);
  pinMode(WIZ_CS, OUTPUT);
  
  // start the Ethernet connection and the server:
  Ethernet.init(WIZ_CS);
  Ethernet.begin(mac, ip);
  server.begin();
  
  //Initialize USB Serial
  Serial.begin(SERIAL_BAUD);
  
  //Initialize current Sensor
  ina260.begin(0x40);

  delay(2000);
  Serial.print("IP Address:");
  Serial.println(Ethernet.localIP());
  Serial.print("    Subnet:");
  Serial.println(Ethernet.subnetMask());
  Serial.print("   Gateway:");
  Serial.println(Ethernet.gatewayIP());

  Setup_LCD();
}

// the loop function runs over and over again forever
void loop() {
  // if an incoming client connects, there will be bytes available to read:
  EthernetClient client = server.available();
  //if (client==true && client.available()>0) {
  //  int packetLength = client.available();
    //Serial.print(packetLength);
  //  for (int i = 0; i < packetLength; i++){
  //    char c = client.read();          // gets one byte from the network buffer
  //    netIn += c;
  //  }
  //}
  
  float   pa_temp = getTemp();
  Serial.println(pa_temp);
  getPower();
  Blink(LED_PIN,1000,1); // wait for a second
  delay(1000);
}

void Blink(byte PIN, byte DELAY_MS, byte loops)
    {
      for (byte i=0; i<loops; i++)
      {
        digitalWrite(PIN,HIGH);
        delay(DELAY_MS);
        digitalWrite(PIN,LOW);
        delay(DELAY_MS);
      }
    }

void getPower()
{
  float shuntvoltage = 0;
  float busvoltage = 0;
  float current_mA = 0;
  float loadvoltage = 0;

  shuntvoltage = ina260.getShuntVoltage_mV();
  busvoltage = ina260.getBusVoltage_V();
  current_mA = ina260.getCurrent_mA();
   
  SerialUSB.print("Bus Voltage:   "); SerialUSB.print(busvoltage); SerialUSB.print(" V , \t");
  SerialUSB.print("Shunt Voltage: \t"); SerialUSB.print(shuntvoltage); SerialUSB.print(" mV \t");
  SerialUSB.print("Current:       \t"); SerialUSB.print(current_mA); SerialUSB.println(" mA");

  delay(10); //small delay for the terminal  
}


float getTemp()
{
  //                                                          ADS1015         ADS1115
  //                                                          -------         -------
  // ads.setGain(GAIN_TWOTHIRDS);   // 2/3x gain +/- 6.144V  1 bit = 3mV      0.1875mV (default)
  // ads.setGain(GAIN_ONE);         // 1x gain   +/- 4.096V  1 bit = 2mV      0.125mV
  // ads.setGain(GAIN_TWO);         // 2x gain   +/- 2.048V  1 bit = 1mV      0.0625mV <--USING
  // ads.setGain(GAIN_FOUR);        // 4x gain   +/- 1.024V  1 bit = 0.5mV    0.03125mV
  // ads.setGain(GAIN_EIGHT);       // 8x gain   +/- 0.512V  1 bit = 0.25mV   0.015625mV
  // ads.setGain(GAIN_SIXTEEN);     // 16x gain  +/- 0.256V  1 bit = 0.125mV  0.0078125mV
  ads2.setGain(GAIN_TWO);         // 2x gain   +/- 2.048V  1 bit = 1mV      0.0625mV <--USING
  int16_t adc0;
  adc0 = ads2.readADC_SingleEnded(3);
  //adc0 = ads.getLastConversionResults();
  float volts = adc0 * 0.0625;// - 50;
  Serial.print("volts:");
  Serial.println(volts);
  float V0 = 500;
  float Tc = 10;
  float tempC = (volts - V0)/Tc;
  float tempF = tempC * 9 / 5 + 32;
  return tempC;
}

void lcd_set_contrast(uint8_t contrast){
  // set the contrast, 200 is a good place to start, adjust as desired
  lcd.write(0xFE);
  lcd.write(0x50);
  lcd.write(contrast);
  delay(10);
}

void lcd_set_brightness(uint8_t brightness){
  // set the brightness - we'll max it (255 is max brightness)
  lcd.write(0xFE);
  lcd.write(0x99);
  lcd.write(brightness);
  delay(10);
}

void lcd_clear(){
  // clear screen
  lcd.write(0xFE);
  lcd.write(0x58);
  delay(10);   // we suggest putting delays after each command 
}

void lcd_set_rgb(uint8_t r, uint8_t g, uint8_t b){
  lcd.write(0xFE);
  lcd.write(0xD0);
  lcd.write(r);
  lcd.write(g);
  lcd.write(b);
  delay(10); 
}

void Setup_LCD(){
  lcd.begin(9600);  
      
  // set the size of the display if it isn't 16x2 (you only have to do this once)
  lcd.write(0xFE);
  lcd.write(0xD1);
  lcd.write(16);  // 16 columns
  lcd.write(2);   // 2 rows
  delay(10);       

  // turn off cursors
  lcd.write(0xFE);
  lcd.write(0x4B);
  lcd.write(0xFE);
  lcd.write(0x54);
  delay(10);
 
  lcd_set_contrast(220);
  lcd_set_brightness(255);
  lcd_clear();
  
  // go 'home'
  lcd.write(0xFE);
  lcd.write(0x48);
  delay(10);   // we suggest putting delays after each command 
 
  lcd.println("VHF/UHF Power ");
  lcd.print("Amplifier");

  lcd_set_rgb(255,100,255);
}

