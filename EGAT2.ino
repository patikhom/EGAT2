/*  Filename     EGAT1.ino
    Description  EGAT Power meter and GSM
    Hardware     EGAT1
    MCU          Atmega328P
    Clock        Internal clock 8 MHz
    Compiler     Arduino 1.8.5
    AVR board    1.6.21
    Engineer     Patikhom Konkeng
    Company      Summation Technology Co., Ltd.
*/


#include <SoftwareSerial.h>
#include "Wire/src/Wire.h"
#include "REG_SDM120.h"
#include "ModbusMaster.h"
#include "OneWire.h"
#include "DallasTemperature.h"
#include "TimerOne.h"

#define ONE_WIRE_BUS_1 3
#define ONE_WIRE_BUS_2 4

#define LED_PWR 7
#define LED_TEMPINBOX 5
#define LED_TEMPOUTBOX 6
#define LED_GSM 13
#define LED_DATA A7
#define REEDSW1 11
#define REEDSW2 12 


OneWire oneWire_in(ONE_WIRE_BUS_1);
OneWire oneWire_out(ONE_WIRE_BUS_2);

DallasTemperature sensor_inhouse(&oneWire_in);
DallasTemperature sensor_outhouse(&oneWire_out);

//String MYID = "002";



//SoftwareSerial mySerial(11, 10);            // RX, TX Pins    // GSM
SoftwareSerial my485(9, 8);            // RX, TX Pins       // Power meter (RS485)

String apn = "internet";                       //APN
String apn_u = "ais";                     //APN-Username
String apn_p = "ais";                     //APN-Password
String url = "http://www.yoururl.xxx/yourphpfile.php";  //URL for HTTP-POST-REQUEST
String data1;   //String for the first Paramter (e.g. Sensor1)
String data2;   //String for the second Paramter (e.g. Sensor2)
String datastr; // all data

boolean REEDSW1F;
unsigned int OPENCOUNT = 0;
unsigned long OPENDURATION = 0;
unsigned long OPENDURATIONTIME = 0;
unsigned int T100US = 0;

float TEMPINBOX, TEMPOUTBOX;
char incomingByte;
String IMEI, ADDR;
unsigned long waitTime;

String inputString = "";         // a string to hold incoming data
boolean stringComplete = false;  // whether the string is complete
boolean SCOMF = false;


ModbusMaster node;

void setup() {
  pinMode(LED_PWR, OUTPUT);
  pinMode(LED_TEMPINBOX, OUTPUT);
  pinMode(LED_TEMPOUTBOX, OUTPUT);
  pinMode(LED_GSM, OUTPUT);

  pinMode(REEDSW1, INPUT_PULLUP);
  pinMode(PB2, INPUT_PULLUP);
  //attachInterrupt(digitalPinToInterrupt(REEDSW1), door_open_count, FALLING);
  

  digitalWrite(LED_PWR, LOW);
  digitalWrite(LED_TEMPINBOX, HIGH);
  digitalWrite(LED_TEMPOUTBOX, HIGH);
  
  digitalWrite(LED_GSM, HIGH);
  delay(200);
  digitalWrite(LED_GSM, LOW);
  delay(200);
  digitalWrite(LED_GSM, HIGH);
  delay(200);
  digitalWrite(LED_GSM, LOW);
  delay(200);
  digitalWrite(LED_GSM, HIGH);
  
  // Open serial communications and wait for port to open:
  Serial.begin(9600);

  sensor_inhouse.begin();
  sensor_outhouse.begin();

  my485.begin(9600);      // Power meter serial comm.
  my485.println("Start...");

  

  delay(5000);
  Serial.println("AT");
  delay(2000);
  //Serial.println("AT+GSN");
  //runsl();

  ADDR = "";
  while(ADDR == "") {
    getIMEI();
    delay(1000);
  }

  //if (digitalRead(REEDSW2) == LOW) selftest();

  
  //my485.println("Go loop");
 
}


void getIMEI() {
  //Serial.println("Getting IMEI...");
  my485.println("Getting IMEI...");
  
  Serial.println("AT+GSN");
  delay(50);
  while (Serial.available()) {
        incomingByte = (char)Serial.read();
        //Serial.print(incomingByte);
        my485.print(incomingByte);
        
        if (incomingByte == 0) {
          // Ignore NULL character
          continue;
        }
       
        
        if (incomingByte == '\n' && IMEI.length() != 17) {
            // Not the line we're looking for
            //Serial.println("Not IMEI");
            my485.println("Not IMEI");
            IMEI = "";
        }
        if (incomingByte == '\n' && IMEI.length() == 17) {    // IMEI --> 867856037547517\r\n
        //if (incomingByte == '\n') {
            // Found the IMEI number
            //my485.println("Found the IMEI number: ");
            IMEI.trim();
            //Serial.println(IMEI);
            my485.println(IMEI);
            ADDR = IMEI.substring(10,15);   // IMEI last 5 digit    --> 867856037547517\r\n --->  47517
            //Serial.println(ADDR);
            my485.println(ADDR);
            break;
        }
        IMEI += incomingByte;   
    }
}

//01 04 00 00 00 02 71 3F // Test 30001
//------------------------------------------------
// Convent 32bit to float
//------------------------------------------------
float HexTofloat(uint32_t x) {
  return (*(float*)&x);
}

uint32_t FloatTohex(float x) {
  return (*(uint32_t*)&x);
}
//------------------------------------------------

float Read_Meter_float(char addr , uint16_t  REG) {
  float i = 0;
  uint8_t j, result;
  uint16_t data[2];
  uint32_t value = 0;
  //node.begin(addr, Serial);
  node.begin(addr, my485);
  result = node.readInputRegisters (REG, 2); ///< Modbus function 0x04 Read Input Registers
  //delay(500);
  if (result == node.ku8MBSuccess) {
    for (j = 0; j < 2; j++)
    {
      data[j] = node.getResponseBuffer(j);
    }
    value = data[0];
    value = value << 16;
    value = value + data[1];
    i = HexTofloat(value);
    //Serial.println("Connec modbus Ok.");
    return i;
  } else {
    //Serial.print("Connec modbus fail. REG >>> "); Serial.println(REG, HEX); // Debug
    //delay(1000); 
    return 0;
  }
}

/*void GET_METER() {     // Update read all data
  Serial.println("GET_METER");
  delay(1000);                              // เคลียบัสว่าง 
  Serial.println("GET_METER1"); 
    for (char i = 0; i < Total_of_Reg ; i++){
      DATA_METER [i] = Read_Meter_float(ID_meter, Reg_addr[i]);//แสกนหลายตัวตามค่า ID_METER_ALL=X
    }
  delay(1000);
}*/

String stringTemp1, stringTemp2;

void get_temperature1() {
    my485.println ("get_temperature1");
    sensor_inhouse.requestTemperatures();

    TEMPINBOX = sensor_inhouse.getTempCByIndex(0);
    if (TEMPINBOX == -127) {
      TEMPINBOX = 99.9;
      digitalWrite(LED_TEMPINBOX, HIGH);
    } else {
      digitalWrite(LED_TEMPINBOX, LOW);
    }
 
    stringTemp1 =  String(TEMPINBOX, 1);
    my485.print ("end get_temperature1 : ");
    my485.println (String(TEMPINBOX, 1));
}

void get_temperature2() {
    my485.println ("get_temperature2");
    sensor_outhouse.requestTemperatures();

    TEMPOUTBOX = sensor_outhouse.getTempCByIndex(0);
    if (TEMPOUTBOX == -127) {
      TEMPOUTBOX = 99.9;
      digitalWrite(LED_TEMPOUTBOX, HIGH);
    } else {
      digitalWrite(LED_TEMPOUTBOX, LOW);
    }
    stringTemp2 =  String(TEMPOUTBOX, 1);
    my485.print ("end get_temperature2 : ");
    my485.println (String(TEMPOUTBOX, 1));
}

void check_door() {
  if (digitalRead(REEDSW1) == LOW) {
    OPENDURATION += 10;
    //if (OPENDURATION >= 65000) OPENDURATION = 65000;  //Approx. 65000ms, 1 loop is not more than 1 minute,  unsigned long (max) 4,294,967,295 
    
    if (REEDSW1F) {
      REEDSW1F = false;
      OPENCOUNT++;
      my485.print ("OPENCOUNT=");
      my485.println (OPENCOUNT);
      
    }
  } else {
    if (REEDSW1F == false) REEDSW1F = true;
    delayMicroseconds(2);
  }
}

void loop() { // run over and over
    timer();
    //delay(10);
    delayMicroseconds(9570);    // 1 loop approx. 10 mSec
}

void timer() {
  check_door();
  T100US += 10;

  switch (T100US) {    // 100 uSec * 10 = 1 seconds
    case 2000 : get_temperature1(); break;    // 2 sec
    case 3000 : get_temperature2(); break;    // 3 sec

    /**************************** Power meter read ******************************/
    /* VOLT, CURRENT, WATTS, PF, FREQ, kWh */
    //case 5000 : GET_METER(); break;
    case 5000 : DATA_METER[0] = Read_Meter_float(ID_meter, Reg_addr[0]); break;                // 5 sec
    case 5200 : DATA_METER[1] = Read_Meter_float(ID_meter, Reg_addr[1]); break; 
    case 5400 : DATA_METER[2] = Read_Meter_float(ID_meter, Reg_addr[2]); break; 
    case 5600 : DATA_METER[3] = Read_Meter_float(ID_meter, Reg_addr[3]); break; 
    case 5800 : DATA_METER[4] = Read_Meter_float(ID_meter, Reg_addr[4]); break; 
    case 6000 : DATA_METER[5] = Read_Meter_float(ID_meter, Reg_addr[5]); break;

    case 8000 : datastr = "";     // DEVICE_ID, VOLT, CURRENT, WATTS, PF, FREQ, kWh, TEMPINBOX, TEMPOUTBOX, OPENCOUNT, OPENDURATIONTIME
                datastr = ADDR + "," + DATA_METER[0] + "," + DATA_METER[1] + "," + DATA_METER[2] + "," + DATA_METER[3] + "," + DATA_METER[4] + "," + DATA_METER[5] + "," + stringTemp1 + "," + stringTemp2 + "," + OPENCOUNT + "," + OPENDURATION;
                OPENDURATION = 0; OPENCOUNT = 0; my485.println (datastr); break;


    case 10000 : Serial.println("AT"); break;                           // delay 4000
    case 14000 : Serial.println("AT+SAPBR=3,1,Contype,GPRS");  break;   // delay 200
    case 14200 : Serial.println("AT+SAPBR=3,1,APN," + apn);  break;     // delay 200
    case 14400 : Serial.println("AT+SAPBR=1,1");  break;                // delay 1000
    case 15400 : Serial.println("AT+SAPBR=2,1");  break;                // delay 2000
    case 17400 : Serial.println("AT+HTTPINIT");  break;                 // delay 200
    case 17600 : Serial.println("AT+HTTPPARA=CID,1");  break;           // delay 200
    case 17800 : Serial.println("AT+HTTPPARA=URL," + url);  break;      // delay 200
    case 18000 : Serial.println("AT+HTTPPARA=CONTENT,application/x-www-form-urlencoded");  break;   // delay 200
    case 18200 : Serial.println("AT+HTTPDATA=192,10000");  break;       // delay 200
    case 18400 : Serial.println("params=" + datastr); break;            // delay 10000
    case 28400 : Serial.println("AT+HTTPACTION=1");  break;             // delay 5000
    case 33400 : Serial.println("AT+HTTPREAD");  break;                 // delay 200
    case 33600 : Serial.println("AT+HTTPTERM");  break;
    case 60000 : T100US = 0; break;
  }
  
}


void selftest() {
  my485.println("self test...");
  while(1) {
    while(ADDR == "") {
      getIMEI();
      delay(1000);
    }

    check_door();
    T100US += 10;
    delayMicroseconds(9570);    // 1 loop approx. 10 mSec
  
    switch (T100US) {    // 100 uSec * 10 = 1 seconds
      case 2000 : get_temperature1(); break;    // 2 sec
      case 3000 : get_temperature2(); break;    // 3 sec
  
      /**************************** Power meter read ******************************/
      /* VOLT, CURRENT, WATTS, PF, FREQ, kWh */
      //case 5000 : GET_METER(); break;
      case 5000 : DATA_METER[0] = Read_Meter_float(ID_meter, Reg_addr[0]); break;                // 5 sec
      case 5200 : DATA_METER[1] = Read_Meter_float(ID_meter, Reg_addr[1]); break; 
      case 5400 : DATA_METER[2] = Read_Meter_float(ID_meter, Reg_addr[2]); break; 
      case 5600 : DATA_METER[3] = Read_Meter_float(ID_meter, Reg_addr[3]); break; 
      case 5800 : DATA_METER[4] = Read_Meter_float(ID_meter, Reg_addr[4]); break; 
      case 6000 : DATA_METER[5] = Read_Meter_float(ID_meter, Reg_addr[5]); break;
  
      case 8000 : datastr = "";     // DEVICE_ID, VOLT, CURRENT, WATTS, PF, FREQ, kWh, TEMPINBOX, TEMPOUTBOX, OPENCOUNT, OPENDURATIONTIME
                  datastr = ADDR + "," + DATA_METER[0] + "," + DATA_METER[1] + "," + DATA_METER[2] + "," + DATA_METER[3] + "," + DATA_METER[4] + "," + DATA_METER[5] + "," + stringTemp1 + "," + stringTemp2 + "," + OPENCOUNT + "," + OPENDURATION;
                  OPENDURATION = 0; OPENCOUNT = 0; break;

  
      case 10000 : Serial.println("AT"); break;                           // delay 4000
      case 14000 : Serial.println("AT+SAPBR=3,1,Contype,GPRS");  break;   // delay 200
      case 14200 : Serial.println("AT+SAPBR=3,1,APN," + apn);  break;     // delay 200
      case 14400 : Serial.println("AT+SAPBR=1,1");  break;                // delay 1000
      case 15400 : Serial.println("AT+SAPBR=2,1");  break;                // delay 2000
      case 17400 : Serial.println("AT+HTTPINIT");  break;                 // delay 200
      case 17600 : Serial.println("AT+HTTPPARA=CID,1");  break;           // delay 200
      case 17800 : Serial.println("AT+HTTPPARA=URL," + url);  break;      // delay 200
      case 18000 : Serial.println("AT+HTTPPARA=CONTENT,application/x-www-form-urlencoded");  break;   // delay 200
      case 18200 : Serial.println("AT+HTTPDATA=192,10000");  break;       // delay 200
      case 18400 : Serial.println("params=" + datastr); break;            // delay 10000
      case 28400 : Serial.println("AT+HTTPACTION=1");  break;             // delay 5000
      case 33400 : Serial.println("AT+HTTPREAD");  break;                 // delay 200
      case 33600 : Serial.println("AT+HTTPTERM");  break;
      case 33610 : T100US = 0; break;
    }
    
  } // end while(1)
}


/*void gsm_sendhttp() {
  Serial.println("AT");
  //runsl();//Print GSM Status an the Serial Output;
  delay(4000);

  Serial.println("AT+SAPBR=3,1,Contype,GPRS");
  //runsl();
  delay(100);
  Serial.println("AT+SAPBR=3,1,APN," + apn);
  //runsl();
  delay(100);
  //mySerial.println("AT+SAPBR=3,1,USER," + apn_u); //Comment out, if you need username
  //runsl();
  //delay(100);
  //mySerial.println("AT+SAPBR=3,1,PWD," + apn_p); //Comment out, if you need password
  //runsl();
  //delay(1000);
  Serial.println("AT+SAPBR=1,1");
  //runsl();
  delay(1000);
  Serial.println("AT+SAPBR=2,1");
  //runsl();
  delay(2000);
  Serial.println("AT+HTTPINIT");
  //runsl();
  delay(100);
  Serial.println("AT+HTTPPARA=CID,1");
  //runsl();
  delay(100);
  Serial.println("AT+HTTPPARA=URL," + url);
  //runsl();
  delay(100);
  Serial.println("AT+HTTPPARA=CONTENT,application/x-www-form-urlencoded");
  //runsl();
  delay(100);
  Serial.println("AT+HTTPDATA=192,10000");
  //mySerial.println("AT+HTTPDATA=192,1000");
  //runsl();
  delay(100);
  //mySerial.println("params=" + data1 + "~" + data2);
  Serial.println("params=" + datastr);
  //runsl();
  delay(10000);
  //delay(1000);
  Serial.println("AT+HTTPACTION=1");
  //runsl();
  delay(5000);
  //delay(1000);
  Serial.println("AT+HTTPREAD");
  //runsl();
  delay(100);
  Serial.println("AT+HTTPTERM");
  //runsl(); 

  //Timer1.attachInterrupt(timer_callback);
}

//Print GSM Status
void runsl() {
  while (Serial.available()) {
    Serial.write(Serial.read());
  }
}
*/






