#include <ArduinoJson.h>
#include <WiFi.h>
#include <PubSubClient.h>

#define TdsSensorPin 35
#define waterLevelPin 34
#define VREF 3.3      // analog reference voltage(Volt) of the ADC
#define SCOUNT  30           // sum of sample point
int analogBuffer[SCOUNT];    // store the analog value in the array, read from ADC
int analogBufferTemp[SCOUNT];
int analogBufferIndex = 0,copyIndex = 0;
float averageVoltage = 0,tdsValue = 0,temperature = 25;
int ciclo=0;


RTC_DATA_ATTR int bootCount = 0;

//conexão com o WIFI
const char* ssid = "manoel";
const char* password =  "9744294831";

//conexão com o broker MQTT
const char* mqttServer = "ddanilo.duckdns.org";
const int mqttPort = 1883;
const char* mqttUser = "dave";
const char* mqttPassword = "123456";
 
WiFiClient espClient1;
PubSubClient client(espClient1);



void setup() {
  Serial.begin(115200);
    pinMode(TdsSensorPin,INPUT);
    pinMode(32, OUTPUT);
    pinMode(33, OUTPUT);
    
   //\\Increment boot number and print it every reboot
  ++bootCount;
  Serial.println("Boot number: " + String(bootCount)); 
 
  WiFi.begin(ssid, password);
 
 while (WiFi.status() != WL_CONNECTED) {
    delay(1000);
   Serial.println("Connecting to WiFi..");
  }
 
 Serial.println("Connected to the WiFi network");
 
  client.setServer(mqttServer, mqttPort);
 
  while (!client.connected()) {
    Serial.println("Connecting to MQTT...");
 
    if (client.connect("ESP32Client1", mqttUser, mqttPassword )) {
 
     Serial.println("connected");
 
  } else {
 
      Serial.print("failed with state ");
      Serial.print(client.state());
      delay(2000);
 
 }
  }
 

}

void loop() {
   static unsigned long analogSampleTimepoint = millis();
   if(millis()-analogSampleTimepoint > 40U)     //every 40 milliseconds,read the analog value from the ADC
   {
     analogSampleTimepoint = millis();
     analogBuffer[analogBufferIndex] = analogRead(TdsSensorPin);    //read the analog value and store into the buffer
     analogBufferIndex++;
     if(analogBufferIndex == SCOUNT) 
         analogBufferIndex = 0;
   }   
   static unsigned long printTimepoint = millis();
   if(millis()-printTimepoint > 800U)
   {
      printTimepoint = millis();
      for(copyIndex=0;copyIndex<SCOUNT;copyIndex++)
        analogBufferTemp[copyIndex]= analogBuffer[copyIndex];
      averageVoltage = getMedianNum(analogBufferTemp,SCOUNT) * (float)VREF / 1024.0; // read the analog value more stable by the median filtering algorithm, and convert to voltage value
      float compensationCoefficient=1.0+0.02*(temperature-25.0);    //temperature compensation formula: fFinalResult(25^C) = fFinalResult(current)/(1.0+0.02*(fTP-25.0));
      float compensationVolatge=averageVoltage/compensationCoefficient;  //temperature compensation
      tdsValue=(133.42*compensationVolatge*compensationVolatge*compensationVolatge - 255.86*compensationVolatge*compensationVolatge + 857.39*compensationVolatge)*0.5; //convert voltage value to tds value
      //Serial.print("voltage:");
      //Serial.print(averageVoltage,2);
      //Serial.print("V   ");
      int water=map(analogRead(waterLevelPin), 0, 4095, 0, 100);
      Serial.print("Water");
      Serial.println(water,0);
      Serial.print("tdsValue");
      Serial.print(tdsValue,0);
      delay(500);


 
      
      
     if( tdsValue > 200 && ciclo == 0)
     {
      delay(1000);
      ciclo=1;
     }
     if(ciclo==1){
      digitalWrite(33, HIGH);
      delay(10000);
      digitalWrite(33, LOW);
      delay(1000);
      ciclo=2;
     }
     if(ciclo==2){
      digitalWrite(32, HIGH);
      delay(1000);
      ciclo=3;
     }
      if( ciclo==3 && water >= 40)
      {
        digitalWrite(32, LOW);
        delay(1000);
        ciclo=0;
      
     }


      //passagem dos dados para o formato json
  DynamicJsonDocument doc(4096);
  //Serial.println(pressure);   
  //Serial.println(soil_moisture);
 // doc["device"] = "tdsValue"; 
  doc["water"] = water;
  doc["tdsValue"] = tdsValue;
  //doc["humidity_1"] = humidity;
  //doc["temperatury_1"] = temperatury;
  delay(1000);

  //envio dos dados por MQTT
  char JSONmessageBuffer[500];
  serializeJson(doc, JSONmessageBuffer);
  Serial.println("Sending message to MQTT topic..");
  Serial.println(JSONmessageBuffer);
 
  if (client.publish("Sensores", JSONmessageBuffer) == true) {
    Serial.println("Success sending message");
  } else {
    Serial.println("Error sending message");
  }
 if (isnan(water) || isnan(tdsValue)) 
  { 
    Serial.println("Failed to read from DHT");
  } 
//    else
  //{
   
    
//  }


   }
}
int getMedianNum(int bArray[], int iFilterLen) 
{
      int bTab[iFilterLen];
      for (byte i = 0; i<iFilterLen; i++)
      bTab[i] = bArray[i];
      int i, j, bTemp;
      for (j = 0; j < iFilterLen - 1; j++) 
      {
      for (i = 0; i < iFilterLen - j - 1; i++) 
          {
        if (bTab[i] > bTab[i + 1]) 
            {
        bTemp = bTab[i];
            bTab[i] = bTab[i + 1];
        bTab[i + 1] = bTemp;
         }
      }
      }
      if ((iFilterLen & 1) > 0)
    bTemp = bTab[(iFilterLen - 1) / 2];
      else
    bTemp = (bTab[iFilterLen / 2] + bTab[iFilterLen / 2 - 1]) / 2;
      return bTemp;

      
     

 
  //client.loop();
  //Serial.println("-------------");
//  delay(10000); 

      

}
