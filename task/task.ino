#include <Arduino_FreeRTOS.h>
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BME280.h>
#define SEALEVELPRESSURE_HPA (1013.25)  //This variable is used to estimate the altitude for a given pressure by comparing it with the sea level pressure.
int light_sensor_value; // to save sensor value 
Adafruit_BME280 BME;// using  I2C communication protocol object called bme.

void setup() {
  // put your setup code here, to run once:

  xTaskCreate(Light, "Light_Sensor", 128, NULL, 1, NULL); // task for light sensor
  xTaskCreate(TaskSerial, "Serial", 128, NULL, 2, NULL); // serial task

}

void loop() {
  // put your main code here, to run repeatedly:

}
void BMESENSOR(void *pvParameters)/* led loacted on the uno board*/
{
  (void) pvParameters;
  for (;;)
  {
  Serial.print("Temperature = ");
	Serial.print(BME.readTemperature());
	Serial.println("*C");

	Serial.print("Pressure = ");
	Serial.print(BME.readPressure() / 100.0F);
	Serial.println("hPa");

  Serial.print("Approx. Altitude = ");
	Serial.print(BME.readAltitude(SEALEVELPRESSURE_HPA));
	Serial.println("m");

	Serial.print("Humidity = ");
	Serial.print(BME.readHumidity());
	Serial.println("%");

	Serial.println();
	delay(1000);
  }
}

void Light(void *pvParameters)/* led loacted on the uno board*/
{
  for (;;)
  {
    light_sensor_value = analogRead(A0); // analog read value of the light sensor 
    vTaskDelay( 5000 / portTICK_PERIOD_MS ); //5000ms == 5sec
  }
}
void TaskSerial(void * pvParameters) { // serial print function

  (void) pvParameters;
  Serial.begin(9600);
  BME_connectivity(); // Initialising BME Sensor

  // Wait for serial port to connect. Needed for native USB, on LEONARDO, MICRO, YUN, and other 32u4 based boards.
  while (!Serial) {
    vTaskDelay(1);
  }
  for (;;) 
  {
    light_description();
    Serial.print("Temperature = ");
  	Serial.print(BME.readTemperature());
	  Serial.println("*C");

  	Serial.print("Pressure = ");
	  Serial.print(BME.readPressure() / 100.0F);
	  Serial.println("hPa");

    Serial.print("Approx. Altitude = ");
	  Serial.print(BME.readAltitude(SEALEVELPRESSURE_HPA));
	  Serial.println("m");

	  Serial.print("Humidity = ");
	  Serial.print(BME.readHumidity());
	  Serial.println("%");

	  Serial.println();
    vTaskDelay( 5000 / portTICK_PERIOD_MS );   // 5 seconds delay 
  }
}
void BME_connectivity(){  // function to initialise BME sensor
  if (BME.begin(0x76)) {
		Serial.println("BME connected!");
	}else{
    Serial.println("BME sensor in the wrong address, check wiring!"); 
  }
}
void light_description(){ // function to describe light value
  if (light_sensor_value < 10) {
    Serial.println(" - Dark");
  } else if (light_sensor_value < 70) {
    Serial.println(" - Dim");
  } else if (light_sensor_value < 300) {
    Serial.println(" - Light");
  } else if (light_sensor_value < 500) {
    Serial.println(" - Bright");
  }  
}