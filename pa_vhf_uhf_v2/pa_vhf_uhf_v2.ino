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
  Adafruit_ADS1115 ads1(0x48);  /* current monitor */
  Adafruit_ADS1115 ads2(0x49);  /* control board */
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
  #define SERIAL_BAUD   9600
  String serIn; // stores incoming serial commands
  String netIn; //stores incoming network commands
  int lcd_state     = 1;  //LCD menu state: 0 = Temps, 1 = DC PWR, 2 = RF Power, 3 = CUSTOM
  int pa_state      = 0;  //PA Deck STATE: 0 = Boot, 1 = RX, 2 = TX_SEQ, 3 = TX, 4 = RX_SEQ, 5 = FAULT
  float i_temp      = 0;  //I monitor temp [C]
  float case_temp   = 0;  //Case temp [C]
  float pa_temp     = 0;  //PA temp [C]
  float shunt_volts = 0;  //INA260 shunt voltage
  float bus_volts   = 0;  //INA260 bus voltage
  float bus_current = 0;  //INA260 bus current
  float pa_fwd      = 0;  //RF Forward Power from Bird Thruline 
  float pa_rev      = 0;  //RF Reverse Power from Bird Thruline

  int thermo        = HIGH;
  int alert_i       = HIGH;
  int alert_t       = HIGH;
  int btn_red_state = HIGH;
  int btn_blk_state = HIGH;
  
  volatile int thermo_state = HIGH; //initialize to high, pullups in place, LOW=thermo engaged 
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

  //initialize digital pin inputs
  pinMode(BTN_RED, INPUT);
  pinMode(BTN_BLK, INPUT);
  pinMode(THERMO, INPUT);
  pinMode(ALERT_I, INPUT);
  pinMode(ALERT_T, INPUT);

  //Setup interrupts
  int irq_thermo = digitalPinToInterrupt(THERMO);
  attachInterrupt(irq_thermo, thermo_isr, CHANGE);
  
  // start the Ethernet connection and the server:
  Ethernet.init(WIZ_CS);
  Ethernet.begin(mac, ip);
  server.begin();
  
  //Initialize USB Serial
  Serial.begin(SERIAL_BAUD);

  //Initialize LCD
  Setup_LCD();
  
  //Initialize current Sensor
  ina260.begin(0x40);

  delay(2000);
  Serial.print("IP Address:");
  Serial.println(Ethernet.localIP());
  Serial.print("    Subnet:");
  Serial.println(Ethernet.subnetMask());
  Serial.print("   Gateway:");
  Serial.println(Ethernet.gatewayIP());
}

// the loop function runs over and over again forever
void loop() {
  
  // if an incoming client connects, there will be bytes available to read:
  EthernetClient client = server.available();
  if (client.available()>0) {
    int packetLength = client.available();
    Serial.print(packetLength); Serial.print(" ");
    for (int i = 0; i < packetLength; i++){
      char c = client.read();          // gets one byte from the network buffer
      netIn += c;
    }
    SerialUSB.println(netIn);
  }
  
  /* ---- GET Temperatures from ADCs---- */
  i_temp    = getADCTemp(ads1, 0);  //INA260 ADC, Imon temp
  case_temp = getADCTemp(ads2, 3);  //Ctrl Brd ADC, Case temp
  pa_temp   = getADCTemp(ads2, 2);  //Ctrl Brd ADC, Case temp

  /* ---- GET DC Power information---- */
  getDCPower(&shunt_volts, &bus_volts, &bus_current);
  
  /* ---- GET RF Power Information ---- */
  getRFPower(&pa_fwd, &pa_rev);
  
  /* ---- GET Digital inputs ---- */
  btn_red_state = digitalRead(BTN_RED);
  if (!btn_red_state){//Red Button Pressed
    
  }
  btn_blk_state = digitalRead(BTN_BLK);
  if (!btn_blk_state){//Black Button Pressed
    
  }
  thermo  = digitalRead(THERMO);
  alert_i = digitalRead(ALERT_I);
  alert_t = digitalRead(ALERT_T);
  

  if (lcd_state == 0){
    updateLcdTemps(&i_temp, &case_temp, &pa_temp);
  }
  else if (lcd_state == 1){
    updateLcdDcPower(&shunt_volts, &bus_volts, &bus_current);
  }
  else if (lcd_state == 2){
    updateLcdRfPower(&pa_fwd, &pa_rev);
  }

  printTlmString();
  //printData();
  
  //Blink(LED_PIN,1000,1); // wait for a second
  delay(500);
  netIn = ""; //reset network input string
}
// --END MAIN LOOP -- //

void printTlmString(){
  SerialUSB.print("$,");
  SerialUSB.print(i_temp); SerialUSB.print(",");
  SerialUSB.print(case_temp); SerialUSB.print(",");
  SerialUSB.print(pa_temp); SerialUSB.print(",");
  SerialUSB.print(shunt_volts); SerialUSB.print(",");
  SerialUSB.print(bus_volts); SerialUSB.print(",");
  SerialUSB.print(bus_current); SerialUSB.print(",");
  SerialUSB.print(pa_fwd); SerialUSB.print(",");
  SerialUSB.print(pa_rev); SerialUSB.print(",");
  SerialUSB.print(btn_red_state); SerialUSB.print(",");
  SerialUSB.print(btn_blk_state); SerialUSB.print(",");
  SerialUSB.print(thermo_state); SerialUSB.print(",");
  SerialUSB.print(alert_i); SerialUSB.print(",");
  SerialUSB.println(alert_t);
  return;
}

void printData(){
  SerialUSB.println("---------------------------------------");
  SerialUSB.print("     Imon Temp [C]: "); SerialUSB.println(i_temp);
  SerialUSB.print("     Case Temp [C]: "); SerialUSB.println(case_temp);
  SerialUSB.print("       PA Temp [C]: "); SerialUSB.println(pa_temp);
  
  SerialUSB.print("Shunt Voltage [mV]: "); SerialUSB.println(shunt_volts);
  SerialUSB.print("  Bus Voltage  [V]: "); SerialUSB.println(bus_volts);
  SerialUSB.print("  Bus Current [mA]: "); SerialUSB.println(bus_current);
  
  SerialUSB.print("     PA RF FWD [V]: "); SerialUSB.println(pa_fwd);
  SerialUSB.print("     PA RF REV [V]: "); SerialUSB.println(pa_rev);

  SerialUSB.print("  Red Button State: "); SerialUSB.println(btn_red_state);
  SerialUSB.print("Black Button State: "); SerialUSB.println(btn_blk_state);
  SerialUSB.print("        Thermostat: "); SerialUSB.print(thermo); SerialUSB.print(" ");SerialUSB.println(thermo_state);
  SerialUSB.print("     Current ALERT: "); SerialUSB.println(alert_i);
  SerialUSB.print("Current Temp ALERT: "); SerialUSB.println(alert_t);
  return;
}

void updateLcdTemps(float* i_temp, float* case_temp, float* pa_temp){
  lcd_clear();
  lcd_set_rgb(0,255,0);
  // go 'home'
  lcd.write(0xFE);
  lcd.write(0x48);
  delay(10);   // we suggest putting delays after each command 
 
  lcd.print("I:"); lcd.print(*i_temp); lcd.print(" ");
  lcd.print("C:"); lcd.println(*case_temp);
  lcd.print("A:"); lcd.print(*pa_temp);
  return;
}

void updateLcdDcPower(float* shunt_volts, float* bus_volts, float* bus_current){
  lcd_clear();
  lcd_set_rgb(0,0,255);
  // go 'home'
  lcd.write(0xFE);
  lcd.write(0x48);
  delay(10);   // we suggest putting delays after each command 
 
  //lcd.print("S:"); lcd.print(*i_temp); lcd.print(" ");
  lcd.print(" V:"); lcd.println(*bus_volts);
  lcd.print("mA:"); lcd.print(*bus_current);
  return;
}

void updateLcdRfPower(float* pa_fwd, float* pa_rev){
  lcd_clear();
  lcd_set_rgb(255,0,0);
  // go 'home'
  lcd.write(0xFE);
  lcd.write(0x48);
  delay(10);   // we suggest putting delays after each command 
 
  //lcd.print("S:"); lcd.print(*i_temp); lcd.print(" ");
  lcd.print("FWD:"); lcd.println(*pa_fwd);
  lcd.print("REV:"); lcd.print(*pa_fwd);
  return;
}



void thermo_isr(){
  thermo_state = digitalRead(THERMO);
  digitalWrite(LED_PIN, !thermo_state);
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

void getDCPower(float* shuntvoltage, float* busvoltage, float* current_mA)
{
  *shuntvoltage = ina260.getShuntVoltage_mV();
  *busvoltage = ina260.getBusVoltage_V();
  *current_mA = ina260.getCurrent_mA();
  delay(10); //small delay for the terminal
  return;  
}

float getADCTemp(Adafruit_ADS1115 adc, int chan)
{
  adc.setGain(GAIN_TWO);         // 2x gain   +/- 2.048V  1 bit = 1mV      0.0625mV <--USING
  int16_t adc_val;
  adc_val = adc.readADC_SingleEnded(chan);
  float volts = adc_val * 0.0625;// - 50;
  float V0 = 500;
  float Tc = 10;
  float tempC = (volts - V0)/Tc;
  return tempC;
}

void getRFPower(float* fwd, float* rev)
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
  int16_t adc_fwd;
  int16_t adc_rev;
  adc_fwd = ads2.readADC_SingleEnded(0);
  adc_rev = ads2.readADC_SingleEnded(1);
  *fwd = adc_fwd * 0.0625;// - 50;
  *rev = adc_rev * 0.0625;// - 50;
  return;
}

/* ----LCD COMMANDS---- */

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
 
  lcd_set_contrast(255);
  lcd_set_brightness(255);
  //lcd_set_rgb(255,0,0);
  lcd_clear();
  
  // go 'home'
  lcd.write(0xFE);
  lcd.write(0x48);
  delay(10);   // we suggest putting delays after each command 
 
  lcd.println("VHF/UHF Power ");
  lcd.print("Amplifier");

  //lcd_set_rgb(255,100,255);
  lcd_set_rgb(0,255,0);
}

