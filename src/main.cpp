/*************************************************
 * 
 *  Temperature Pression Si7021
 * 
 *  Use mysensors in Home assistant 
 *
 *  Note:
 *    Works with 2 x 1.5V batteries of your choice.
 * 
 *  Author : Eric H
 * 
 * ***********************************************/
#include <Arduino.h>

#define MON_DEBUG true

#define MY_RADIO_RF24

//MY_RF24_CHANNEL par defaut 76
//Channels: 1 to 126 - 76 = Channel 77
//MY_RF24_CHANNEL (76)
#include <perso.h>

#define MY_RX_MESSAGE_BUFFER_FEATURE //for MY_RF24_IRQ_PIN
//Define this to use the IRQ pin of the RF24 module
#define MY_RF24_IRQ_PIN (2)

//NODE_ID
#define MY_NODE_ID 10

#include <mysensors.h>
#include <Adafruit_Si7021.h>

#define CHILD_TEMP 0
#define CHILD_HUM 1
#define CHILD_ID_RESSENT 2

// Set this offset if the sensors have permanent small offsets to the real temperatures/humidity.
// In Celsius degrees or moisture percent
#define SENSOR_TEMP_OFFSET 0      // used for temperature data and heat index computation
#define SENSOR_HUM_OFFSET 0       // used for humidity data
#define SENSOR_RESSENTI_OFFSET 0   // used for heat index data

#define SKETCH_NAME "Temperature-Humidite"
#define SKETCH_VERSION "1.0"

// Wait times
#define SHORT_WAIT 50
#define LONG_WAIT 500
#define LONG_WAIT1 1000
#define LONG_WAIT2 2000

bool metric = true;

bool enableHeater = false;
uint8_t loopCnt = 0;
float newHumidity = -100;
float Humidity = -100;
float newTemperature = -100;
float Temperature = -100;
float T_Ressentie = -100;
float prevHumidity = -100;
float prevTemperature = -100;
float prevT_Ressentie = -100;
Adafruit_Si7021 sensor = Adafruit_Si7021();

MyMessage msgTEMP(CHILD_TEMP, V_TEMP);
MyMessage msgHUM(CHILD_HUM, V_HUM);
MyMessage msgRESSENTI(CHILD_ID_RESSENT, V_TEMP);

float calculRessenti(float temperature, float percentHumidity) {
  // Based on Adafruit DHT official library (https://github.com/adafruit/DHT-sensor-library/blob/master/DHT.cpp)
  // Using both Rothfusz and Steadman's equations
  // http://www.wpc.ncep.noaa.gov/html/heatindex_equation.shtml

  float hi;

  temperature = temperature + SENSOR_TEMP_OFFSET; //include TEMP_OFFSET in HeatIndex computation too
  temperature = 1.8*temperature+32; //convertion to *F

  hi = 0.5 * (temperature + 61.0 + ((temperature - 68.0) * 1.2) + (percentHumidity * 0.094));

  if (hi > 79) {
    hi = -42.379 +
      2.04901523 * temperature +
      10.14333127 * percentHumidity +
      -0.22475541 * temperature*percentHumidity +
      -0.00683783 * pow(temperature, 2) +
      -0.05481717 * pow(percentHumidity, 2) +
      0.00122874 * pow(temperature, 2) * percentHumidity +
      0.00085282 * temperature*pow(percentHumidity, 2) +
      -0.00000199 * pow(temperature, 2) * pow(percentHumidity, 2);

  if((percentHumidity < 13) && (temperature >= 80.0) && (temperature <= 112.0))
    hi -= ((13.0 - percentHumidity) * 0.25) * sqrt((17.0 - abs(temperature - 95.0)) * 0.05882);

  else if((percentHumidity > 85.0) && (temperature >= 80.0) && (temperature <= 87.0))
    hi += ((percentHumidity - 85.0) * 0.1) * ((87.0 - temperature) * 0.2);
  }

  hi = (hi-32)/1.8;
  return hi; //return Heat Index, in *C

}

void presentation()  {
  Serial.print("===> Envoyer présentation du noeud : "); Serial.println(MY_NODE_ID);
  sendSketchInfo(SKETCH_NAME, SKETCH_VERSION);
  wait(LONG_WAIT1);
  
  char sNoeud[] = STR(MY_NODE_ID);
  //  S_TEMP
  char sChild0[25];
  strcpy(sChild0, "myS ");
  strcat(sChild0, sNoeud);
  strcat(sChild0, " Temperature");
  Serial.print("Present: "); Serial.print("'"); Serial.print(sChild0); Serial.println("'");
  present(CHILD_TEMP, S_TEMP, sChild0); 
  wait(LONG_WAIT2);

  //  S_HUM
  char sChild1[25];
  strcpy(sChild1, "myS ");
  strcat(sChild1, sNoeud);
  strcat(sChild1, " Humidite");
  Serial.print("Present: "); Serial.print("'"); Serial.print(sChild1); Serial.println("'");
  present(CHILD_HUM, S_HUM, sChild1); 
  wait(LONG_WAIT2);

  //  S_TEMP RESENTI
  char sChild2[25];
  strcpy(sChild2, "myS ");
  strcat(sChild2, sNoeud);
  strcat(sChild2, " Ressenti");
  Serial.print("Present: "); Serial.print("'"); Serial.print(sChild2); Serial.println("'");
  present(CHILD_ID_RESSENT, S_TEMP, sChild2); 
  wait(LONG_WAIT2);

  metric = getControllerConfig().isMetric;

}


void setup() {
  // put your setup code here, to run once:
  Serial.println("Si7021 test!");
  
  if (!sensor.begin()) {
    Serial.println("Did not find Si7021 sensor!");
    while (true)
      ;
  }

  Serial.print("Found model ");
  switch(sensor.getModel()) {
    case SI_Engineering_Samples:
      Serial.print("SI engineering samples"); break;
    case SI_7013:
      Serial.print("Si7013"); break;
    case SI_7020:
      Serial.print("Si7020"); break;
    case SI_7021:
      Serial.print("Si7021"); break;
    case SI_UNKNOWN:
    default:
      Serial.print("Unknown");
  }
  Serial.print(" Rev(");
  Serial.print(sensor.getRevision());
  Serial.print(")");
  Serial.print(" Serial #"); Serial.print(sensor.sernum_a, HEX); Serial.println(sensor.sernum_b, HEX);

}

void loop() {

  Temperature = sensor.readTemperature();
  Humidity = sensor.readHumidity();

  // put your main code here, to run repeatedly:
  Serial.print("Humidity:    "); Serial.print(Humidity, 2); Serial.print("\tTemperature: "); Serial.println(Temperature, 2);

  if (fabs(Humidity - newHumidity) >= 0.05 || fabs(Temperature - newTemperature) >= 0.05) {
    newTemperature = Temperature;
    newHumidity = Humidity;
    T_Ressentie = calculRessenti(Temperature,Humidity); //computes Heat Index, in *C

    #ifdef MON_DEBUG
      Serial.print("T Ressentie: "); Serial.print(T_Ressentie); Serial.println(" *C");    
    #endif    
    
    if (!metric) {
      Temperature = 1.8*Temperature+32; //convertion to *F
      T_Ressentie = 1.8*T_Ressentie+32; //convertion to *F
    }
    
    if (prevTemperature != Temperature) {
      prevTemperature = Temperature;
      #ifdef MON_DEBUG
        Serial.print("Sending temperature: "); Serial.print(Temperature);
      #endif      
      send(msgTEMP.set(Temperature + SENSOR_TEMP_OFFSET, 2));
      wait(SHORT_WAIT);
    }

    if (prevHumidity != Humidity) {
      prevHumidity = Humidity;
      #ifdef MON_DEBUG
        Serial.print("Sending humidity: "); Serial.print(Humidity);
      #endif
      send(msgHUM.set(Humidity + SENSOR_HUM_OFFSET, 2));
      wait(SHORT_WAIT);
    }

    if (prevT_Ressentie != T_Ressentie) {
      prevT_Ressentie = T_Ressentie;
      #ifdef MON_DEBUG
        Serial.print("Sending T ressentie: "); Serial.print(T_Ressentie);
      #endif
      send(msgRESSENTI.set(T_Ressentie + SENSOR_RESSENTI_OFFSET, 2));
      wait(SHORT_WAIT);
    }
  }

}