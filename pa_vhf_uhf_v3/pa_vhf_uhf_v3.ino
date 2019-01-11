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
#include <EEPROM.h>


//****Custom Board Pinouts*********************************
//OUTPUT
#define LED_PIN       7
#define VHF_PTT       4
#define UHF_PTT       12
#define PA_FAN        A2
#define VHF_COAX_REL  A0
#define UHF_COAX_REL  A1
#define PA_PWR_REL    A3
#define WIZ_CS        11
#define WIZ_RST       A4

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

// assign MAC and IP Addresses 
byte mac[] = { 0xDE, 0xAD, 0xBE, 0xEF, 0xFE, 0xED }; 
IPAddress ip(192, 168, 20, 80);  
unsigned int localPort = 2000;      // local port to listen on
EthernetServer server(localPort);

//****Global Variable Definition*********************************
#define SERIAL_BAUD   9600
String serIn; //stores incoming serial commands
String netIn; //stores incoming network commands
volatile uint8_t lcd_state = 0;  //LCD menu state: 0 = Temps, 1 = DC PWR, 2 = RF Power, 3 = CUSTOM
volatile uint8_t pa_state  = 0;  //PA Deck STATE: 0 = Boot, 1 = RX, 2 = VHF_TX/UHF_RX, 3 = VHF_RX/UHF_TX, 4 = THERMOSTAT_FAULT

float i_temp      = 0;  //I monitor temp [C]
float case_temp   = 0;  //Case temp [C]
float pa_temp     = 0;  //PA temp [C]
float shunt_volts = 0;  //INA260 shunt voltage
float bus_volts   = 0;  //INA260 bus voltage
float bus_current = 0;  //INA260 bus current
float pa_fwd_mv   = 0;  //RF Forward Power from Bird Thruline [mV] 
float pa_rev_mv   = 0;  //RF Reverse Power from Bird Thruline [mV]
float pa_fwd_pwr  = 0;  //RF Forward Power [Watts], from calibration equations
float pa_rev_pwr  = 0;  //RF Reverse Power [Watts], from calibration equations
float vswr        = 0;  //VSWR

float fault_vswr    = 0;  //retains last vswr fault level
float fault_fwd_pwr = 0;  //retains last vswr forward power level
float fault_rev_pwr = 0;  //retains last vswr fault reverse power level

volatile int thermo = HIGH; //Thermo state, read in ISR, init HIGH, low = engaged
uint8_t alert_i   = HIGH;
int alert_t       = HIGH;
int btn_red_state = HIGH;
int btn_blk_state = HIGH;
int pa_fan        = LOW;
int pa_pwr_rel    = LOW;
int vhf_coax_rel  = LOW;
int uhf_coax_rel  = LOW;
int vhf_ptt       = LOW;
int uhf_ptt       = LOW;
uint8_t manual_fan = LOW;  //Manual Fan override
uint8_t cal_mode  = LOW;   //Calibration mode, ignores VSWR and PA overcurrents

float fan_thresh_hi = 30; //Fan Threshold Hi Temp, above = fan on
float fan_thresh_lo = 28; //Fan Threshold Lo Temp, below = fan off
float current_thresh = 8800; //Overcurrent Threshold, above = PA Overcurrent fault state

//PTT delay time, amount of time between each event
int16_t ptt_delay = 40;

//volatile int thermo_state = HIGH; //initialize to high, pullups in place, LOW=thermo engaged 
//*********************************************************************

uint8_t eeprom_boot_addr = 0;  //EEPROM Memory address to store boot count
uint8_t eeprom_boot_cnt  = 0;  //EEPROM Memory stored boot count, incremented on power cycle.

void setup() {
  eeprom_boot_cnt = EEPROM.read(eeprom_boot_addr);
  if (eeprom_boot_cnt == 255){
    eeprom_boot_cnt=0;//reset counter to rollover and prevent memory corruption.
  }
  else{
    eeprom_boot_cnt++;
  }
  EEPROM.write(eeprom_boot_addr, eeprom_boot_cnt);

  //Initialize USB Serial
  SerialUSB.begin(SERIAL_BAUD);
  
  //Initialize LCD
  lcd.begin(9600); 
  Setup_LCD();
  lcd_clear();
  lcd_home();
  lcd.println("Initializing");
  lcd.print("Peripherals...");
  
  // initialize digital pin outputs.
  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, LOW);
  pinMode(VHF_PTT, OUTPUT);
  digitalWrite(VHF_PTT, LOW);
  pinMode(UHF_PTT, OUTPUT);
  digitalWrite(UHF_PTT, LOW);
  pinMode(PA_FAN, OUTPUT);
  digitalWrite(PA_FAN, LOW);
  pinMode(VHF_COAX_REL, OUTPUT);
  digitalWrite(VHF_COAX_REL, LOW);
  pinMode(UHF_COAX_REL, OUTPUT);
  digitalWrite(UHF_COAX_REL, LOW);
  pinMode(PA_PWR_REL, OUTPUT);
  digitalWrite(PA_PWR_REL, LOW);
  pinMode(WIZ_CS, OUTPUT);

  //initialize digital pin inputs
  pinMode(BTN_RED, INPUT);
  pinMode(BTN_BLK, INPUT);
  pinMode(THERMO, INPUT);
  pinMode(ALERT_I, INPUT);
  pinMode(ALERT_T, INPUT);

  //Attempt some timer interrupts
  //ref:  http://www.instructables.com/id/Arduino-Timer-Interrupts/
  cli();//stop interrupts
  //set timer1 interrupt at 1Hz
  TCCR1A = 0;// set entire TCCR1A register to 0
  TCCR1B = 0;// same for TCCR1B
  TCNT1  = 0;//initialize counter value to 0
  // set compare match register for 0.25hz (4sec) increments
  //OCR1A = 31249;// = (8*10^6) / (1*1024) - 1 (must be <65536)
  OCR1A = 15625;// = (8*10^6) / (1*1024) - 1 (must be <65536)
  // turn on CTC mode
  TCCR1B |= (1 << WGM12);
  // Set CS10 and CS12 bits for 1024 prescaler
  TCCR1B |= (1 << CS12) | (1 << CS10);  
  // enable timer compare interrupt
  TIMSK1 |= (1 << OCIE1A);
  sei();//allow interrupts
  //End of Timer Interrupt Setup

  //Setup interrupts
  int irq_thermo = digitalPinToInterrupt(THERMO);
  attachInterrupt(irq_thermo, thermo_isr, FALLING);

  //Initialize current Sensor
  ina260.begin(0x40);

  delay(1000);
  lcd_clear();
  lcd_home();
  lcd.println("Initializing");
  lcd.print("Ethernet...");
  
  // start the Ethernet connection and the server:
  pinMode(WIZ_RST, OUTPUT);
  digitalWrite(WIZ_RST, HIGH);
  delay(100);
  digitalWrite(WIZ_RST, LOW);
  delay(100);
  digitalWrite(WIZ_RST, HIGH);
  Ethernet.init(WIZ_CS);
  Ethernet.begin(mac, ip);
  delay(1000); //give ethernet time to setup and initialize
  server.begin();
  
  //Initialize PA to Receive State
  pa_state = 1; 

  lcd_clear();
  lcd_home();
  lcd.println("Setup Complete");
  lcd.print("Happy TXing!!!");
  delay(1000);
}

void loop() {
  /* ---- Check for Network Command ---- */
  // if an incoming client connects, there will be bytes available to read:
  EthernetClient client = server.available();
  if (client.available()>0) {
    int packetLength = client.available();
    //Serial.print(packetLength); Serial.print(" ");
    for (int i = 0; i < packetLength; i++){
      char c = client.read();          // gets one byte from the network buffer
      netIn += c;
    }
  }

  if (client.connected() && netIn.length() > 0){
    processNetCommand(&client);
  }

  //SerialUSB.println(MCUSR, HEX);
  
  /* ---- GET Temperatures from ADCs---- */
  i_temp    = getADCTemp(ads1, 0);  //INA260 ADC, Imon temp
  case_temp = getADCTemp(ads2, 3);  //Ctrl Brd ADC, Case temp
  pa_temp   = getADCTemp(ads2, 2);  //Ctrl Brd ADC, PA temp
  if (manual_fan){
    digitalWrite(PA_FAN, HIGH);
  }
  else if (pa_state != 4){//if not in thermo_fault state
    if (!pa_fan & (pa_temp >= fan_thresh_hi)){ //fan is off, pa temp > 30C
      digitalWrite(PA_FAN, HIGH);
    }
    else if (pa_fan & (pa_temp < fan_thresh_lo)){//fan is on, pa_temp < 28C
      digitalWrite(PA_FAN, LOW);
    }
  }

  /* ---- GET DC Power information---- */
  getDCPower(&shunt_volts, &bus_volts, &bus_current);
  if (!cal_mode){
    if (bus_current > current_thresh){ setStateFault(6); } //6 = Overcurrent Fault
  }
  /* ---- GET RF Power Information ---- */
  measRfPowerVolts(&pa_fwd_mv, &pa_rev_mv);
  computeRfPowerWatts();
  computeVswr();
  
  if (!cal_mode){
    if ((pa_fwd_pwr > 1.0) & (vswr > 3) & (pa_state != 5)){ //not already in VSWR Fault State
      fault_vswr = vswr;
      fault_fwd_pwr = pa_fwd_pwr;
      fault_rev_pwr = pa_rev_pwr;
      setStateFault(5); //5 = VSWR Fault
    }
  }

  /* ---- GET Digital inputs ---- */
  thermo  = digitalRead(THERMO);
  alert_i = digitalRead(ALERT_I);
  alert_t = digitalRead(ALERT_T);

  /* ---- Independently read digital outputs ---- */
  pa_fan        = digitalRead(PA_FAN);
  pa_pwr_rel    = digitalRead(PA_PWR_REL);
  vhf_coax_rel  = digitalRead(VHF_COAX_REL);
  uhf_coax_rel  = digitalRead(UHF_COAX_REL);
  vhf_ptt       = digitalRead(VHF_PTT);
  uhf_ptt       = digitalRead(UHF_PTT);

  btn_red_state = digitalRead(BTN_RED);
  btn_blk_state = digitalRead(BTN_BLK);
  if (pa_state != 1){//either in Fault or TX state, goto RX State if BOTH btn pressed
    if ((!btn_red_state) & (!btn_blk_state)){
      setStateRxAll();
      delay(200); //give time for button release
    }
  }
  else if (pa_state == 1){// PA in RX Mode
    //if both buttons are pressed -> likely hold over fron RX transition.
    if (!((!btn_red_state) & (!btn_blk_state))){ //Both buttons are NOT Pressed.
      if (!btn_red_state){//Red pressed = VHF_TX
        setStateVhfTx();
      }
      if (!btn_blk_state){//Black pressed = UHF_TX
        setStateUhfTx();
      }
    }
  }
  
  updateLCD();
  printTlmString();
  //delay(250);
  netIn = ""; //reset network input string
}
// --END MAIN LOOP -- //

void processNetCommand(EthernetClient* client){
  client->print(netIn);
  if (netIn.charAt(0) == 'q'){//query
    sendTlmString(client);
  }
  else if (netIn.charAt(0) == 'r'){//RX_ALL
    if (pa_state != 1){
      setStateRxAll();
    }
  }
  else if (netIn.charAt(0) == 'v'){//VHF_TX
    if (pa_state == 1){
      setStateVhfTx();
    }
  }
  else if (netIn.charAt(0) == 'u'){//UHF_TX
    if (pa_state == 1){
      setStateUhfTx();
    }
  }
  else if (netIn.charAt(0) == 'f'){//MAnual Fan Override
    //manual fan override
    manual_fan = ~manual_fan; //toggle
  }
  else if (netIn.charAt(0) == 'o'){//Toggle Calibration Mode
    //manual fan override
    cal_mode = ~cal_mode; //toggle
  }
  else if (netIn.charAt(0) == 'c'){//clear eeprom reset counter
    EEPROM.write(eeprom_boot_addr, 0); // FIRST Argument is address of register and 2nd is value 
    eeprom_boot_cnt = EEPROM.read(eeprom_boot_addr);
    client->print("EEPROM Cleared: ");
    client->println(eeprom_boot_cnt);
  }
  if (netIn.charAt(0) == 'e'){//eeprom_query
    eeprom_boot_cnt = EEPROM.read(eeprom_boot_addr);
    client->println(eeprom_boot_cnt);
  }
}

void computeRfPowerWatts(){
  if (pa_state == 5){//in VSWR FAULT State
    pa_fwd_pwr = fault_fwd_pwr; //assign retained fault vswr
    pa_rev_pwr = fault_rev_pwr;
    return;
  }
  if (pa_fwd_mv > 25) {
    if (pa_state == 2){//VHF_TX
      //--SN1197302 VHF Conversion Equations, Amateur Band PA --
      //pa_fwd_pwr = 0.00002 * pa_fwd_mv * pa_fwd_mv + 0.0134 * pa_fwd_mv - 0.4473;
      //pa_rev_pwr = 0.000007 * pa_rev_mv * pa_rev_mv + 0.0052 * pa_rev_mv - 0.1905;
      //--SN1107313 VHF Conversion Equations, Federal Band PA -- 
      pa_fwd_pwr = 0.00003 * pa_fwd_mv * pa_fwd_mv + 0.0124 * pa_fwd_mv - 1.0758;
      pa_rev_pwr = 0.00002 * pa_rev_mv * pa_rev_mv + 0.0019 * pa_rev_mv + 0.1684;
    }
    else if (pa_state == 3){//UHF_TX
      //--SN1197302 UHF Conversion Equations, Amateur Band PA --
      //pa_fwd_pwr = 0.00003 * pa_fwd_mv * pa_fwd_mv + 0.0044 * pa_fwd_mv - 0.0164;
      //pa_rev_pwr = 0.000009 * pa_rev_mv * pa_rev_mv + 0.001 * pa_rev_mv + 0.0152;
      //--SN1197302 UHF Conversion Equations, Amateur Band PA --
      pa_fwd_pwr = 0.00003 * pa_fwd_mv * pa_fwd_mv + 0.0038 * pa_fwd_mv - 0.0199;
      pa_rev_pwr = 0.00001 * pa_rev_mv * pa_rev_mv + 0.0015 * pa_rev_mv + 0.0956;
    }  
    if (pa_fwd_pwr < 0){ pa_fwd_pwr = 0; }
    if (pa_rev_pwr < 0){ pa_rev_pwr = 0; }
  }
  else{
      pa_fwd_pwr = 0.0;
      pa_rev_pwr = 0.0;
  }
  return;
}

void computeVswr(){
  if (pa_state == 5){//in VSWR FAULT State
    vswr = fault_vswr; //assign retained fault vswr
    return;
  }
  if ((pa_fwd_pwr <= 0) | (pa_rev_pwr <=0)){
    vswr = 0.0;
  }
  else{
    float a = sqrt(pa_rev_pwr/pa_fwd_pwr);
    if (a == 1.0){
      vswr = 99.0;
    }
    else{
      vswr = (1.0 + a) / (1.0-a);
    }
  }
  return;
}

ISR(TIMER1_COMPA_vect){//timer1 interrupt
  lcd_state = (lcd_state+1) % 4;
}

void setStateUhfTx(){
  digitalWrite(VHF_PTT, LOW);
  digitalWrite(VHF_COAX_REL, LOW);
  delay(ptt_delay);
  digitalWrite(UHF_PTT, HIGH);
  delay(ptt_delay);
  digitalWrite(UHF_COAX_REL, HIGH);
  delay(ptt_delay);
  digitalWrite(PA_PWR_REL, HIGH);
  digitalWrite(PA_FAN, HIGH);
  delay(ptt_delay);
  lcd_state = 3;
  pa_state = 3;
  updateLCD();
}

void setStateVhfTx(){
  digitalWrite(UHF_PTT, LOW);
  digitalWrite(UHF_COAX_REL, LOW);
  delay(ptt_delay);
  digitalWrite(VHF_PTT, HIGH);
  delay(ptt_delay);
  digitalWrite(VHF_COAX_REL, HIGH);
  delay(ptt_delay);
  digitalWrite(PA_PWR_REL, HIGH);
  digitalWrite(PA_FAN, HIGH);
  delay(ptt_delay);
  lcd_state = 3;
  pa_state = 2;
  updateLCD();
}

void setStateRxAll(){
  digitalWrite(PA_PWR_REL, LOW);
  delay(ptt_delay);
  digitalWrite(VHF_COAX_REL, LOW);
  digitalWrite(UHF_COAX_REL, LOW);
  delay(ptt_delay);
  digitalWrite(VHF_PTT, LOW);
  digitalWrite(UHF_PTT, LOW);
  delay(ptt_delay);
  lcd_state = 3;
  pa_state = 1;
  updateLCD();
}

void setStateFault(uint8_t state){
  digitalWrite(PA_FAN, HIGH);
  digitalWrite(PA_PWR_REL, LOW);
  delay(ptt_delay);
  digitalWrite(VHF_COAX_REL, LOW);
  digitalWrite(UHF_COAX_REL, LOW);
  delay(ptt_delay);
  digitalWrite(VHF_PTT, LOW);
  digitalWrite(UHF_PTT, LOW);
  delay(ptt_delay);
  lcd_state = 3;
  pa_state = state;
  updateLCD();
}

void sendTlmString(EthernetClient* server){
  //client->println("TEST");
  server->print("$,");
  server->print(millis()); server->print(",");
  server->print(EEPROM.read(eeprom_boot_addr)); server->print(",");
  if (pa_state == 1){server->print("RX_ALL,");}
  else if (pa_state == 2){server->print("VHF_TX,");}
  else if (pa_state == 3){server->print("UHF_TX,");}
  else if (pa_state == 4){server->print("THERMO_FAULT,");}
  else if (pa_state == 5){server->print("VSWR_FAULT,");}
  else if (pa_state == 6){server->print("OVERCURRENT_FAULT,");}
  else {server->print("FAULT,");}
  server->print(i_temp); server->print(",");
  server->print(case_temp); server->print(",");
  server->print(pa_temp); server->print(",");
  server->print(shunt_volts); server->print(",");
  server->print(bus_volts); server->print(",");
  server->print(bus_current); server->print(",");
  server->print(pa_fwd_mv); server->print(",");
  server->print(pa_rev_mv); server->print(",");
  server->print(pa_fwd_pwr); server->print(",");
  server->print(pa_rev_pwr); server->print(",");
  server->print(vswr); server->print(",");
  server->print(btn_red_state); server->print(",");
  server->print(btn_blk_state); server->print(",");
  server->print(thermo); server->print(",");
  server->print(alert_i); server->print(",");
  server->print(alert_t); server->print(",");
  server->print(pa_fan); server->print(",");
  server->print(pa_pwr_rel); server->print(",");
  server->print(vhf_coax_rel); server->print(",");
  server->print(uhf_coax_rel); server->print(",");
  server->print(vhf_ptt); server->print(",");
  server->print(uhf_ptt); server->print(",");
  server->println(cal_mode);
  return;
}

void printTlmString(){
  SerialUSB.print("$,");
  SerialUSB.print(millis()); SerialUSB.print(",");
  SerialUSB.print(EEPROM.read(eeprom_boot_addr)); SerialUSB.print(",");
  if (pa_state == 1){SerialUSB.print("RX_ALL,");}
  else if (pa_state == 2){SerialUSB.print("VHF_TX,");}
  else if (pa_state == 3){SerialUSB.print("UHF_TX,");}
  else if (pa_state == 4){SerialUSB.print("THERMO_FAULT,");}
  else if (pa_state == 5){SerialUSB.print("VSWR_FAULT,");}
  else if (pa_state == 6){SerialUSB.print("OVERCURRENT_FAULT,");}
  else {SerialUSB.print("FAULT,");}
  SerialUSB.print(i_temp); SerialUSB.print(",");
  SerialUSB.print(case_temp); SerialUSB.print(",");
  SerialUSB.print(pa_temp); SerialUSB.print(",");
  SerialUSB.print(shunt_volts); SerialUSB.print(",");
  SerialUSB.print(bus_volts); SerialUSB.print(",");
  SerialUSB.print(bus_current); SerialUSB.print(",");
  SerialUSB.print(pa_fwd_mv); SerialUSB.print(",");
  SerialUSB.print(pa_rev_mv); SerialUSB.print(",");
  SerialUSB.print(pa_fwd_pwr); SerialUSB.print(",");
  SerialUSB.print(pa_rev_pwr); SerialUSB.print(",");
  SerialUSB.print(vswr); SerialUSB.print(",");
  SerialUSB.print(btn_red_state); SerialUSB.print(",");
  SerialUSB.print(btn_blk_state); SerialUSB.print(",");
  SerialUSB.print(thermo); SerialUSB.print(",");
  SerialUSB.print(alert_i); SerialUSB.print(",");
  SerialUSB.print(alert_t); SerialUSB.print(",");
  SerialUSB.print(pa_fan); SerialUSB.print(",");
  SerialUSB.print(pa_pwr_rel); SerialUSB.print(",");
  SerialUSB.print(vhf_coax_rel); SerialUSB.print(",");
  SerialUSB.print(uhf_coax_rel); SerialUSB.print(",");
  SerialUSB.print(vhf_ptt); SerialUSB.print(",");
  SerialUSB.print(uhf_ptt); SerialUSB.print(",");
  SerialUSB.println(cal_mode);
  return;
}

void updateLCD(){
  if      (pa_state == 1){ lcd_set_rgb(0,255,0); }    //RX_ALL, Green
  else if (pa_state == 2){ lcd_set_rgb(0,128,255); }  //VHF_TX, Light Blue
  else if (pa_state == 3){ lcd_set_rgb(255,0,128);}   //UHF_TX, Light Purple
  else if (pa_state == 4){ lcd_set_rgb(255,0,0);}     //THERMO_FAULT, RED
  else if (pa_state == 5){ lcd_set_rgb(255,0,0);}     //VSWR_FAULT, RED
  else if (pa_state == 6){ lcd_set_rgb(255,0,0);}     //OVERCURRENT_FLT, RED
  else{
    lcd_set_rgb(255,255,255);
    lcd.print("UNKOWN FAULT");
  }
  if      (lcd_state == 0){ updateLcdTemps(); }
  else if (lcd_state == 1){ updateLcdDcPower(); }
  else if (lcd_state == 2){ updateLcdRfPower(); }
  else if (lcd_state == 3){ updateLcdPaState(); }
  return;
}

void updateLcdPaState(){
  lcd_clear();
  lcd_home(); //go to home pos (1,1)

  if ((pa_state == 1) | (pa_state == 2) | (pa_state == 3)){
    lcd.println("PA STATE:"); 
    if      (pa_state == 1){ lcd.print("  RX ALL"); }
    else if (pa_state == 2){ lcd.print("  VHF TX"); }
    else if (pa_state == 3){ lcd.print("  UHF TX"); }
  }
  else{
    lcd.println("PA FAULT:");
    if      (pa_state == 4){ lcd.print("  THERMOSTAT"); }
    else if (pa_state == 5){ lcd.print("  VSWR > 3"); }
    else if (pa_state == 6){ lcd.print("  OVERCURRENT"); }
    else                   { lcd.print("  UNKNOWN"); }
  }
  return;
}

void updateLcdTemps(){
  lcd_clear();
  lcd_home(); //go to home pos (1,1)
 
  lcd.print("I:"); lcd.print(i_temp); lcd.print(" ");
  lcd.print("C:"); lcd.println(case_temp);
  lcd.print("A:"); lcd.print(pa_temp);
  return;
}

void updateLcdDcPower(){
  lcd_clear();
  lcd_home(); //go to home pos (1,1)
 
  //lcd.print("S:"); lcd.print(*i_temp); lcd.print(" ");
  lcd.print(" V:"); lcd.println(bus_volts);
  lcd.print("mA:"); lcd.print(bus_current);
  return;
}

void updateLcdRfPower(){
  lcd_clear();
  lcd_home(); //go to home pos (1,1)
 
  //lcd.print("S:"); lcd.print(*i_temp); lcd.print(" ");
  lcd.print("FWD:"); lcd.println(pa_fwd_pwr);
  lcd.print("REV:"); lcd.print(pa_rev_pwr);

  //lcd_home(); //go to home pos (1,1)
  lcd_set_cursor_pos(1,11);
  lcd.print("VSWR");
  lcd_set_cursor_pos(2,11);
  lcd.print(vswr);
  return;
}

void thermo_isr(){
  //SerialUSB.println("THERMO ISR Fired!!!");
  thermo = digitalRead(THERMO);
  if (!thermo & pa_state != 4){
    setStateFault(4); //4 = Thermostat Fault
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

void measRfPowerVolts(float* fwd, float* rev)
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

void lcd_home(){
  lcd.write(0xFE);
  lcd.write(0x48);
  delay(10);   // we suggest putting delays after each command 
}

void lcd_set_cursor_pos(uint8_t row, uint8_t col){
  lcd.write(0xFE);
  lcd.write(0x47);
  lcd.write(col);
  lcd.write(row);
  delay(10);
}

void Setup_LCD(){
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
  lcd_set_brightness(100);
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

