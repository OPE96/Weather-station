#include <Arduino_FreeRTOS.h>
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BME280.h>
//#include <SoftwareSerial.h>
#include <queue.h>
Adafruit_BME280 BME;// using  I2C communication protocol object called bme.

QueueHandle_t Bmequeue;
QueueHandle_t Lightqueue;
struct Bmesensorvalue{
  float temperature;
  float pressure;
  float altitude;
  float humidity;
};
struct Lightsensorvalue{
  int light_sensor_value; // varriable to save  sensor value
};

void setup() {
  Bmequeue = xQueueCreate( 4, sizeof(struct Bmesensorvalue)) ; // creating queue for Bme sensor 
  Lightqueue = xQueueCreate(2, sizeof(struct Lightsensorvalue)) ; // create queue for light sensor
 Serial.begin(9600);
 //intialise Bme sensor with the right address
 if (BME.begin(0x76)) {   
		Serial.println("BME connected!");
	}else{
    Serial.println("BME sensor in the wrong address, check wiring!"); 
  }
  xTaskCreate(Lighttask, "Light_Sensor", 128, NULL, 1, NULL); // task for light sensor
  xTaskCreate(BME_connectivity, "BME_Sensor", 128, NULL, 2, NULL); // task for Bme sensor
  xTaskCreate(TaskSerial, "Serial", 128, NULL, 3, NULL); // serial task
}

void loop() {
  // put your main code here, to run repeatedly:

}

void Lighttask(void *pvParameters)/* Light sensor task */
{
  struct Lightsensorvalue sv; //initialise struct for sensor value
  int currentvalue = 0;
  int perviousvalue = 0;
  for (;;)
  {
    perviousvalue = currentvalue;   // pervious value equals to current value 
    currentvalue = analogRead(A0);  // read current light sensor value 
    // compare light sensor value in if statement
    if (perviousvalue != currentvalue){  
      sv.light_sensor_value = currentvalue;
    }
  
    xQueueSend (Lightqueue, &sv, portMAX_DELAY);  // send value into the light queue
    vTaskDelay( 5000 / portTICK_PERIOD_MS );      // delay for 5000ms == 5sec
  }
}
void TaskSerial(void *pvParameters) { // serial print function

  (void) pvParameters;
  Serial.begin(9600);
  // Wait for serial port to connect. 
  while (!Serial) {
    vTaskDelay(1);
  }
  struct Bmesensorvalue BMEvalue; //intialise Struct for the BME sensor 
  struct Lightsensorvalue sv;  //initialise struct for sensor value
  for (;;) 
  {
    //Receive an item from light queue
    if (xQueueReceive (Lightqueue, &sv, portMAX_DELAY) ==pdPASS){  

       if (sv.light_sensor_value > 100) {
        Serial.print("Light:");
      }
       if (sv.light_sensor_value < 70 ) {
        Serial.print("Dark:");
      }
        Serial.println(sv.light_sensor_value);
    }
    //Receive an item from a BME Sensor queue
    if (xQueueReceive (Bmequeue, &BMEvalue, portMAX_DELAY) ==pdPASS){
      Serial.print("Temperature = ");
  	  Serial.print(BMEvalue.temperature);
	    Serial.println("*C");

      Serial.print("Pressure = ");
	    Serial.print(BMEvalue.pressure);
	    Serial.println("hPa"); 

      Serial.print("Approx. Altitude = ");
	    Serial.print(BMEvalue.altitude);
	    Serial.println("m");

      Serial.print("Humidity = ");
	    Serial.print(BMEvalue.humidity);
	    Serial.println("%"); 
    }
    vTaskDelay( 6000 / portTICK_PERIOD_MS );   // 5 seconds delay
  }
}
void BME_connectivity(void *pvParameters){  
  
  struct Bmesensorvalue BMEvalue; // Initialise Struct for BME sensor
  // Initialise variable for Bme sensor
  float SEALEVELPRESSURE_HPA = 1013.25;
  float tempcurrentvalue = 0 ;
  float tempperviousvalue = 0;
  float humiditycurrentvalue = 0; float humidityperviousvalue = 0;
  float pressurecurrentvalue = 0 ; float pressureperviousvalue = 0;
  float altitudescurrentvalue = 0 ; float altitudesperviousvalue = 0;
  for (;;) 
  {
  // Pervious value equals to current value and compare BME  sensor value in if statement
  tempperviousvalue = tempcurrentvalue;
  tempcurrentvalue = BME.readTemperature();
  if (tempperviousvalue != tempcurrentvalue){
    BMEvalue.temperature = BME.readTemperature();
  }
  pressureperviousvalue = pressurecurrentvalue;
  pressurecurrentvalue = BME.readPressure() / 100.0F;
  if (pressureperviousvalue != pressurecurrentvalue) {
    BMEvalue.pressure = BME.readPressure() / 100.0F;
  }
  humidityperviousvalue = humiditycurrentvalue;
  humiditycurrentvalue = BME.readHumidity();
  if (humidityperviousvalue != humiditycurrentvalue){
    BMEvalue.humidity =BME.readHumidity(); 
  }
  altitudesperviousvalue = altitudescurrentvalue;
  altitudescurrentvalue = BME.readAltitude(SEALEVELPRESSURE_HPA);
  if (altitudesperviousvalue != altitudescurrentvalue){
    BMEvalue.altitude = BME.readAltitude(SEALEVELPRESSURE_HPA);
  }
    // send value into the BME queue
    xQueueSend (Bmequeue, &BMEvalue, portMAX_DELAY);
    vTaskDelay( 6000 / portTICK_PERIOD_MS );   // 6 seconds delay
  }
}

