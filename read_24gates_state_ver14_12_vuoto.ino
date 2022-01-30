/*

     gate     0   ||  1  ||  2  ||  3  ||  4   ||   5  ||  6   ||  7   ||  8   ||  9   ||  10  ||  11  ||  12  ||  13  ||  14  ||  15  ||  16  ||  17  ||  18  ||  19  ||  20  ||  21  ||  22  ||  23
     inside   1   ||  3  ||  5  ||  7  ||  9   ||  11  ||  13  ||  15  ||  17  ||  19  ||  21  ||  23  ||  25  ||  27  ||  29  ||  31  ||  33  ||  35  ||  37  ||  39  ||  41  ||  43  ||  45  ||  47
     outside  2   ||  4  ||  6  ||  8  ||  10  ||  12  ||  14  ||  16  ||  18  ||  20  ||  22  ||  24  ||  26  ||  28  ||  30  ||  32  ||  34  ||  36  ||  38  ||  40  ||  42  ||  44  ||  46  ||  48


    HOW long can we drive the LEDs ON without current limiting resistors?
    LED timed pulse (tp) allowance vs period (T)
    with increasing forward current the IR LED cutsheet says max tp/T = 0.01
    so the max tp for 1 amp is 100us and 100us/0.01 = 10ms
    with an ON time of ~75us the max period or delay between ON times is 75us/0.01=7.5ms
    So the minimum OFF time between ON times is 7.5ms
    This is very conservative as the forward voltage of 3.3v/2 = 1.65v, and max current from the cutsheet is estimated ~170ma
    this is well below the absolute max of 1 amp with which the tp/T ratio was developed for


    Rough Power Draw with cheap USB current sensor
    feather ESP32 by itself with tp/T  15us/2ms => 50ma
    feather ESP32 by itself with tp/T  15us/10ms => 40-50ma
    ESP32 & LEDs
    tp/T  15us/2ms  100ma
    tp/T  15us/4ms  75ma
    tp/T  15us/6ms  60-70ma had to add 3000uf cap to smooth out drawl
    tp/T  15us/8ms  50-60ma had to add 3000uf cap to smooth out drawl
    tp/T  15us/10ms  50ma had to add 3000uf cap to smooth out drawl

    This sketch:
    It takes a honey bee around 250ms to traverse the sensor This code only looks for honeybees moving faster than 650ms.
    MODIFICA DA SERGIO VAVASSORI PER RILEVAMENTO PASSAGGI SU STATO  11.2021
                                            file:  read_24gates_state-machine_ver14_12
    >>>>>>     INTERVALLO PER INVIO DATI RIGA 64
    >>>>>>     RTC ORA RIGA 258 o 261

    INSERIRE DATA ORA DA CONSOLE gg mm aaaa hh mm ss
*/
//INIZIO mio  ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

#define ECHO_TO_SERIAL // Allows serial output if uncommented

// GSM °°°°°°°°°°°°°°°°°°°°°°
#define SIM800 // invio dati su SIM800

// DTH22 °°°°°°°°°°°°°°°°°°°°°°°°
#include <DHT.h>
#define DHTPIN 6   //Pin a cui è connesso il sensore
#define DHTTYPE DHT22   //Tipo di sensore che stiamo utilizzando (DHT22)
DHT dht(DHTPIN, DHTTYPE); //Inizializza oggetto chiamato "dht", parametri: pin a cui è connesso il sensore, tipo di dht 11/22

// SD °°°°°°°°°°°°°°°°°°°°°°°°°°°
#include <SD.h>
#define cardSelect 4  // pin connessione SD 5 su RTC wafer, 4 su adalogger
File logfile;   // Create file object
File myFile;

#include <Wire.h>

// RTC °°°°°°°°°°°°°°°°°°°°°°°°°°°°°
#include "RTClib.h"//RTCLib by Adafruit
RTC_PCF8523 rtc; //RTC su wafer
int  lastDay; // per reset 24h

// Tempi scrittura e invio dati °°°°°°°°°°°°°°°°°°°°°°°°°°°°
int intervallo = 1800000;  //intervallo di tempo scrittura su SD e invio dati, (1sec = 1000) 120000= 2 minuti; 1800000=30 minuti (legato a riga 540)  < ****************** timer

long previousMills = 0;  //tempo della funzione mills() memorizzato
long previousMills1 = - 60000;  //anticipo comando attivazione periferiche (-60000 = -1 minuto)

int n = 0;

// Valore batteria°°°°°°°°°°°°°°°°°°°
#define ANALOG_INPUT  A1
int readPin = A1;
int readVal;
float V2 = 0;

//Precipitazione °°°°°°°°°°°°°°°°°°
// lowest and highest sensor readings:
#define rainAnalog A0
const int sensorMin = 0;     // sensor minimum
const int sensorMax = 1024;  // sensor maximum
int range;
int rainAnalogVal;

// OLED Display  °°°°°°°°°°°°°°°°°°°°°°°°°°°°
// Include Adafruit Graphics & OLED libraries
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
// Reset pin not used but needed for library
#define OLED_RESET 4
Adafruit_SSD1306 display(OLED_RESET);

//Cella di carico °°°°°°°°°°°°°°°°°°°°°°°°°°°
#include "HX711.h"
#define DOUT 11
#define CLK 12
HX711 bilancia(DOUT, CLK);
int peso = 0; // zona di memorizzazione del peso corrente
int pesoprec = 0; // zona di memorizzazione dell'ultimo peso esposto

//FINE mio ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

#include <stdio.h>
#include <stdint.h>
#include <string.h>
#include <limits.h>

#include <SPI.h>

/*
   Finite State Machine structure and transition function.
*/
typedef enum {
  EMPTY,
  START_ENTERING,  // bee is entering the gate from outside, so it's entering
  MIDDLE_ENTERING,
  END_ENTERING,    // bee is almost all inside
  START_EXITING,   // bee is entering the gate from inside, so it's exiting
  MIDDLE_EXITING,
  END_EXITING,     // bee is almost all outside
  ERROR_STATE
} State;

typedef enum {
  SENSOR_LOW,             // there is no bee
  SENSOR_HIGH             // there is a bee
} Gate_Sensor;

const State transition_table[8][2][2] = {
  { { EMPTY, START_ENTERING }, { START_EXITING, ERROR_STATE } },

  // inbound pattern
  { { EMPTY, START_ENTERING }, { ERROR_STATE, MIDDLE_ENTERING } },

  { { ERROR_STATE, START_ENTERING }, { END_ENTERING, MIDDLE_ENTERING } },

  { { EMPTY, // a bee has entered
      ERROR_STATE
    }, { END_ENTERING, MIDDLE_ENTERING }
  },

  // outbound pattern
  { { EMPTY, ERROR_STATE }, { START_EXITING, MIDDLE_EXITING } },

  { { ERROR_STATE, END_EXITING }, { START_EXITING, MIDDLE_EXITING } },

  { { EMPTY, // a bee has exited
      END_EXITING
    }, { ERROR_STATE, MIDDLE_EXITING }
  },

  // Error state stays in Error state
  { { EMPTY, // gate is empty again, reset
      ERROR_STATE
    }, { ERROR_STATE, ERROR_STATE }
  }
};

typedef struct {
  State state;
  unsigned long startTime;
} Gate;

uint32_t prevEntered = 0;
uint32_t entered = 0;
uint32_t exited = 0;
uint32_t prevExited = 0;
const unsigned long minTraversalTime = 130; // in millis TEMPO MINIMO DI PASSAGGIO APE PER LA CONTA

int inConta = 0; // api in x reset dati su intervallo
int outConta = 0; //api in x reset dati su intervallo

static void next_state(Gate* gate, unsigned long time, uint8_t inner, uint8_t outer)
{
  State current = gate->state;
  State next = transition_table[current][inner][outer];

  if (next == EMPTY)
  {
    if (current == END_ENTERING)
    {
      unsigned long elapsed = time - gate->startTime;
      if (elapsed >= minTraversalTime)
      {
        entered++; //conta api in
        inConta++; //conta api in x reset time riga 61
      }
    }
    else if (current == END_EXITING)
    {
      unsigned long elapsed = time - gate->startTime;
      if (elapsed >= minTraversalTime)
      {
        exited++; //conta api out
        outConta++; //conta api out x reset time riga 61
      }
    }
    else if (current == EMPTY || current == ERROR_STATE)
    {
      gate->startTime = time;
    }
  }
  else if (next == ERROR_STATE)
  {
    // two bits are flipped at the same time (Hamming distance = 2), shouldn't happen
    gate->startTime = ULONG_MAX;
  }

  gate->state = next;
}

/**************************************************************************************/
const int testLed = 13; //onboard LED
const int LATCH = A5;

//Pins Feather ESP32
//const byte powerGates1 = 15;
//const byte powerGates2 = 33;

//Pins ItsyBitsy
//const byte powerGates1 = 10;
//const byte powerGates2 = 11;

//Pins Adalogger M0
const byte powerGates1 = 9;
const byte powerGates2 = 10;


#define NUMBER_OF_GATES 24 // 24 gates, 48 sensors
#define GATES_PER_BANK 4   // 2 bits per gate, so 4 gates in a byte
// const int startGate = 0;   // useful for testing
// const int endGate = 24;    // useful for testing
const int numberOfBanks = NUMBER_OF_GATES / GATES_PER_BANK;
unsigned long currentTime = 0;

Gate gates[NUMBER_OF_GATES];


void setup()
{
  //INIZIO mio  ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
  
  // RTC °°°°°°°°°°°°°°°°°°°°°°°°°°°°
  Wire.begin();
  if (! rtc.begin()) {
    Serial.println("Couldn't find RTC");
    while (1);
  }
  if (! rtc.isrunning()) {
    Serial.println("RTC is NOT running!");
    Serial.flush();
    abort();
  }
  if (! rtc.initialized() || rtc.lostPower()) {
    Serial.println("RTC is NOT initialized, let's set the time!");
    // When time needs to be set on a new device, or after a power loss, the
    // following line sets the RTC to the date & time this sketch was compiled
    //rtc.adjust(DateTime(F(__DATE__), F(__TIME__))); //inserire questa riga i caso cambio batteria RTC < ********************************************************
  }
  rtc.start();
  
  // SD °°°°°°°°°°°°°°°°°°°°°°°°°°°°°
  if (!SD.begin(cardSelect)) // see if the card is present and can be initialized:
  {
    Serial.println("Card init. failed! or Card not present");
  }
  pinMode(13, OUTPUT);//uscita su seriale
  pinMode(8, OUTPUT);// uscita su led verde
  Serial.println("Logging ....");

  //GSM °°°°°°°°°°°°°°°°°°°°°°°°°°°°°°
#ifdef SIM800
  Serial1.begin(115200);    // inizializza Serial1 x GSM
#endif

  // DTH °°°°°°°°°°°°°°°°°°°°°°°°°°°
  dht.begin();    //inizializza DHT

  //Batteria  °°°°°°°°°°°°°°°°°°°°°°°°°°
  pinMode (readPin, INPUT);

  // OLED display °°°°°°°°°°°°°°°°°°°°°°°°°
  display.begin(SSD1306_SWITCHCAPVCC, 0x3C); // initialize OLED with I2C addr 0x3C

  //Cella di Carico - Setup
  bilancia.set_scale(20.96); //inserire il valore di (calibrazione)
  //bilancia.tare(20); // il peso a vuoto e' considerato tara,( al reload porta il valore a zero)

  // FINE mio++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

  // initialize gates structures to default values
  for (int i = 0; i < NUMBER_OF_GATES; i++)
  {
    gates[i].state = EMPTY;
    gates[i].startTime = ULONG_MAX;
  }

  SPI.begin();
  SPI.beginTransaction(SPISettings(3000000, MSBFIRST, SPI_MODE2));

  Serial.begin (115200);
  Serial.println ("Begin switch test.");
  pinMode (LATCH, OUTPUT);
  digitalWrite (LATCH, HIGH);

  pinMode (powerGates1, OUTPUT);
  digitalWrite(powerGates1, LOW);
  pinMode (powerGates2, OUTPUT);
  digitalWrite(powerGates2, LOW);

  pinMode(testLed, OUTPUT);
}  // end of setup

void loop ()
{
  currentTime = millis();

  digitalWrite(powerGates1, HIGH);
  digitalWrite(powerGates2, HIGH);
  delayMicroseconds(75); //first 24 gates only need 15us while gates closer to the end need ~40us-75us

  digitalWrite (LATCH, LOW);    // pulse the parallel load latch
  delayMicroseconds(3);
  digitalWrite (LATCH, HIGH);

  delayMicroseconds(3);

  digitalWrite(powerGates1, LOW);
  digitalWrite(powerGates2, LOW);

  boolean anyHigh = false;
  prevEntered = entered;
  prevExited = exited;
  int gateIndex = 0;
  uint8_t bits;
  uint8_t outerSensor;
  uint8_t innerSensor;

  // read bytes and convert them to gate values
  for (int bank = 0; bank < numberOfBanks; bank++)
  {
    // reading 24 bits at 1Mhz should take about 24 microseconds,
    // reading 24 bits at 3Mhz should take about 8us
    // reading 48 bits at 3Mhz should take abotu 16us
    bits = SPI.transfer(0);

    for (int gateBank = 0; gateBank < GATES_PER_BANK; gateBank++)
    {
      anyHigh |= outerSensor = ((bits >> 0) & 1);
      anyHigh |= innerSensor = ((bits >> 1) & 1);
      bits >>= 2;

      next_state(&gates[gateIndex], currentTime, innerSensor, outerSensor);
      gateIndex++;
    }
  }


  // light the LED if any sensor is high
  digitalWrite(testLed, anyHigh ? HIGH : LOW);

  // print out time, in and out values if any have changed
  if (prevEntered != entered || prevExited != exited)  //stampa solo se  cambiamento di stato
  {
    //Serial.print(currentTime);
    //Serial.print(";    ");
    Serial.print("IN : ");
    Serial.print(entered); // STAMPA CONTA API ENTRANTI
    Serial.print("; ");
    Serial.print("OUT : ");
    Serial.print(exited); // STAMPA CONTA API USCENTI
    Serial.println(";");
    /*
        Serial.print(inConta); // STAMPA CONTA API ENTRANTI parziale
        Serial.print(" ");
        Serial.print(outConta); // STAMPA CONTA API uscenti parziali
        Serial.println(" ");
    */
  }

  delay(9);   // wait some time, adjust to have a sampling frequency of 100 Hz
  // TODO reset counters at midnight.

  // INIZIO mio++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

  // inserisce manualmente data e ora da monitor seriale
  if (Serial.available()) {
    String buffer = Serial.readStringUntil('\n');
    setDateTime(buffer);
  }

  //  Reset dati a mezzanotte °°°°°°°°°°°°°°°°°°°°°°°°°°°°°
  ResetH24();

  // Print battery voltage with Voltage Divider ( 15K + 6.8k resistor valore 985) °°°°°°°°°°°°°°°°°°°°°°°°
  readVal = analogRead(readPin);//read the A1 pin value
  V2 = (12 / 985.) * readVal; //convert the value to a true voltage (range 0 - 1023)( 985 x 12V) <**************************************** tensione batteria
  { if ( V2 < 9.8)
      V2 = 0.5; //con valore inferiore a 2,8V o batteria scollegata mette il valore a 0,5V
  }

  // Precipitazione °°°°°°°°°°°°°°°°°°°°°
  int sensorReading = analogRead(A0);
  int range = map(sensorReading, sensorMin, sensorMax, 0, 3);// map the sensor range (four options):
  int rainAnalogVal = analogRead(rainAnalog);

  //******************* VISUALIZZA, SALVA E INVIA DATI ****************************

#ifdef ECHO_TO_SERIAL
  if (millis() -  previousMills > intervallo ) {  //visualizza, salva e invia dati con intervallo di riga 64
    previousMills = millis();

    SerialOutputnow(); // scrive su seriale i totali dopo intervallo

    pesa(); // invia dati bilancia dopo intervallo

    displayOled();// scrive su OLED  dopo intervallo

    SdOutputnow(); // scrive su SD i totali dopo intervallo

    delay(4000);// ritarda per salvare dati su SD

    SIM800Output(); // invia dati su SIM800 dopo intervallo

    inConta = 0; //reset parziale count  in ogni intervallo
    outConta = 0; //reset parziale count out ogni intervallo
  }
  
  if (millis() -  previousMills1 > intervallo ) { //invia comando attivazione periferiche con intervallo di riga 64 e anticipo di riga 67 
    previousMills1 = millis();
    DateTime now = rtc.now();
    Serial.print(now.minute(), DEC);
    Serial.println("   pin 5 ON");
    pinMode (5, OUTPUT);
    digitalWrite (5, HIGH);
  }
#endif
}//end loop_____________________________

void ResetH24()//  Reset dati a mezzanotte °°°°°°°°°°°°°°°°°°°°°°°°°°°°°
{
  DateTime now = rtc.now();
  if (now.day() != lastDay) // this happens exactly once a day.
    entered = 0; // reset api in
  if (now.day() != lastDay) // this happens exactly once a day.
    exited = 0; // reset api out

  /*
    // reset watchdog a mezzanotte, (se funziona togliere righe da 442 a 446) 
    
    if (now.day() != lastDay) // this happens exactly once a day.
    digitalWrite(A2, LOW);   // pin A2 collegato a pin RST a massa x riavvio M0
  */

  lastDay = now.day();    // fetch the time
  now = rtc.now(); //log time
}

void SIM800Output()//dati su GSM #############################
{
  float h = dht.readHumidity(); //variabile per umidità
  float t = dht.readTemperature();//variabile per temperatura
  int rainAnalogVal = analogRead(rainAnalog);

  if (Serial1.available())
    Serial.write(Serial1.read());

  Serial1.println("AT");
  delay(1000);

  Serial1.println("AT+CPIN?");
  delay(1000);

  Serial1.println("AT+CSQ");
  delay(1000);

  Serial1.println("AT+CREG?");
  delay(1000);

  Serial1.println("AT+CGATT?");
  delay(1000);

  Serial1.println("AT+CIPSHUT");
  delay(1000);

  Serial1.println("AT+CIPSTATUS");
  delay(2000);

  Serial1.println("AT+CIPMUX=0");
  delay(2000);

  ShowSerialData();

  Serial1.println("AT+CSTT=\"internet.xxxxx\"");//start task and setting the APN,
  delay(1000);

  ShowSerialData();

  Serial1.println("AT+CIICR");//bring up wireless connection
  delay(3000);

  ShowSerialData();

  Serial1.println("AT+CIFSR");//get local IP adress
  delay(2000);

  ShowSerialData();

  Serial1.println("AT+CIPSPRT=0");
  delay(3000);

  ShowSerialData();

  Serial1.println("AT+CIPSTART=\"TCP\",\"api.thingspeak.com\",\"80\"");//start up the connection
  delay(6000);

  ShowSerialData();

  Serial1.println("AT+CIPSEND");//begin send data to remote server
  delay(4000);
  ShowSerialData();

  String str = "GET https://api.thingspeak.com/update?api_key=xxxxxxxxxxxxxxxxxxx&field3=" + String(entered) + "&field4=" + String(exited) + "&field1=" + String(t) + "&field2=" + String(h) + "&field5=" + String(V2) + "&field7=" + String(peso) + "&field6=" + String(rainAnalogVal) + "&field8=" + String(inConta);
  Serial.println(str);
  Serial1.println(str);//begin send data to remote server

  delay(4000);
  ShowSerialData();

  Serial1.println((char)26);//sending
  delay(10000);//waiting for reply, important! the time is base on the condition of internet(5000)
  Serial1.println();

  ShowSerialData();

  Serial1.println("AT+CIPSHUT");//close the connection
  delay(100);
  ShowSerialData();

  Serial.println();

  // comando spegnimento periferiche
  DateTime now = rtc.now();
  Serial.print(now.minute(), DEC);
  Serial.println("    pin 5 OFF");
  pinMode (5, OUTPUT);
  digitalWrite (5, LOW);
}

void ShowSerialData()// scrive su GSM  ######################################
{
  while (Serial1.available() != 0)
    Serial.write(Serial1.read());
  //delay(5000);
}

void SerialOutputnow()// scrive su serale #####################################
{
  DateTime now = rtc.now();
  Serial.print(now.day(), DEC);
  Serial.print('/');
  Serial.print(now.month(), DEC);
  Serial.print('/');
  Serial.print(now.year(), DEC);
  Serial.print(" ");
  Serial.print(now.hour(), DEC);
  Serial.print(':');
  Serial.print(now.minute(), DEC);
  Serial.print(";");

  Serial.print(" IN ape:");
  Serial.print(entered);   // Print in bee++++++++++++++
  Serial.print(",");
  Serial.print(" OUT ape:");
  Serial.println(exited);  // Print out bee++++++++++++++

  // DHT22
  float h = dht.readHumidity(); //variabile per umidità
  float t = dht.readTemperature();//variabile per temperatura
  Serial.print("Temperature = ");
  Serial.print(t);
  Serial.println(" °C");
  Serial.print("Humidity = ");
  Serial.print(h);
  Serial.println(" %");

  //Batteria
  Serial.print("Battery = ");
  Serial.print(V2);
  Serial.println(" V");

  // Precipitazione
  int rainAnalogVal = analogRead(rainAnalog);
  Serial.print("Precipitazione = ");
  rainData();
  Serial.println(rainAnalogVal);

  //Serial.println(currentTime);
  //Serial.println(previousMills);
  //  Serial.println(previousMills1);

  Serial.flush ();
}

int pesa()// dati pesa  ############################################
{
  peso = bilancia.get_units(20), 3;
  if (!(peso == pesoprec)) // se e' variato il peso
  {
    pesoprec = peso; // memorizza il peso corrente
    Serial.print("peso : ");
    Serial.print(peso);
    Serial.println("gr.");
  }
  return peso;
}

int pesaDisp()// dati pesa su OLED  ############################################
{
  peso = bilancia.get_units(20), 3;
  if (!(peso == pesoprec)) // se e' variato il peso
  {
    pesoprec = peso; // memorizza il peso corrente
    display.print("gr:");
    display.print(peso);
  }
  return peso;
}

void SdOutputnow()//scrive su SD  ############################################
{
  myFile = SD.open("conta.csv", FILE_WRITE);

  //se il file si apre correttamente, scrivo in esso
  if (myFile) {
    digitalWrite(8, HIGH);   // Turn the green LED on
    Serial.println("Scrivo su conta.csv...");

    DateTime now = rtc.now();
    myFile.print(now.day(), DEC );
    myFile.print("/");
    myFile.print(now.month(), DEC );
    myFile.print("/");
    myFile.print(now.year(), DEC );
    myFile.print(" ");
    myFile.print(",");
    myFile.print(now.hour(), DEC );
    myFile.print(":");
    myFile.print(now.minute(), DEC );
    myFile.print(";");

    myFile.print(entered);   // Print in bee++++++++++++++
    myFile.print(",");
    myFile.print(exited);   // Print out bee++++++++++++++
    myFile.print(",");
    myFile.print(";");

    //DHT22
    float h = dht.readHumidity(); //variabile per umidità
    float t = dht.readTemperature();//variabile per temperatura
    myFile.print(t);
    myFile.print(",");
    myFile.print(h);
    myFile.print(",");
    myFile.print(";");

    //Batteria
    myFile.print(V2);
    myFile.print(",");
    myFile.print(";");

    //Pesa
    myFile.print(peso);
    myFile.print(",");
    myFile.print(";");

    //rain
    int rainAnalogVal = analogRead(rainAnalog);
    myFile.print(rainAnalogVal);
    myFile.println(",");
    myFile.print(";");

    digitalWrite(8, LOW);   // Turn the green LED off

    myFile.close();// close the file
  }
}

void rainData()// Precipitazione su serial  ############################################
{
  int sensorReading = analogRead(A0);// read the sensor on analog A0:
  int range = map(sensorReading, sensorMin, sensorMax, 0, 3); // map the sensor range (four options):
  int rainAnalogVal = analogRead(rainAnalog);

  switch (range)// range value:
  {
    case 0:    // Sensor getting completely wet
      Serial.println("Pioggia");
      break;
    case 1:    // Sensor getting partially wet
      Serial.println("Debole");
      break;
    case 2:    // Sensor dry
      Serial.println("Nessuna");
      break;
  }
}

void rainDataDisp()// Precipitazione su OLED  ############################################
{
  int sensorReading = analogRead(A0);// read the sensor on analog A0:
  int range = map(sensorReading, sensorMin, sensorMax, 0, 3);// map the sensor range (four options):
  int rainAnalogVal = analogRead(rainAnalog);

  switch (range) // range value:
  {
    case 0:    // Sensor getting completely wet
      display.println("Pioggia");
      break;
    case 1:    // Sensor getting partially wet
      display.println("Debole");
      break;
    case 2:    // Sensor dry
      display.println("Nessuna");
      break;
  }
}

void displayOled()//dati su OLED ############################################
{
  display.clearDisplay();// Clear the display
  display.setTextColor(WHITE);//Set the color - always use white despite actual display color
  display.setTextSize(1);//Set the font size
  display.setCursor(0, 0);//Set the cursor coordinates

  display.print("Api IN: ");
  display.println(entered);
  display.print("Api OUT: ");
  display.println(exited);

  //DHT22
  float h = dht.readHumidity(); //variabile per umidità
  float t = dht.readTemperature();//variabile per temperatura
  display.print("T.: ");
  display.print(t);
  display.print("C");
  display.print(" ");
  display.print("H.: ");
  display.print(h);
  display.println("%");
  /*
    //Batteria
    display.print("Bat:");
    display.print(V2);
    display.print(" V");
    display.print(" ");
  */
  //Ora
  DateTime now = rtc.now();
  display.print("Ora:");
  display.print(now.hour(), DEC );
  display.print(":");
  display.print(now.minute(), DEC );
  display.print(";");

  //Rain
  //display.print("Pr.: ");
  //rainDataDisp();

  //Pesa
  pesaDisp();

  display.display();
}

void setDateTime(String value) // inserisce manualmente data e ora da monitor seriale
{
  int d, m, y, h, min, s;
  int n = sscanf(value.c_str(), "%d %d %d %d %d %d", &d, &m, &y, &h, &min, &s);

  if (n == 6) {
    rtc.adjust(DateTime(y, m, d, h, min, s));
  }
}
/*
  void EcoON()//invia comando attivazione periferiche
  {
  DateTime now = rtc.now();
  Serial.print(now.minute(), DEC);
  Serial.println("     ON pin 5");
  pinMode (5, OUTPUT);
  digitalWrite (5, HIGH);

  delay(60000);// ritarda di 2 minuti
  }
*/
