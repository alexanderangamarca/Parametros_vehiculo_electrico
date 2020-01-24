// LIBRERIAS
#include <TinyGPS++.h>           //GPS
#include <SoftwareSerial.h>      //GPS
#include <Wire.h>                //MPU9250
#include <FaBo9Axis_MPU9250.h>   //MPU9250

//GPS---------------------------------------------------------------------------
static const int RXPin = 4, TXPin = 3;
static const uint32_t GPSBaud = 9600;

// The TinyGPS++ object
TinyGPSPlus gps;

// The serial connection to the GPS device
SoftwareSerial ss(RXPin, TXPin);

//MPU-9250----------------------------------------------------------------------
//Aceleracion
FaBo9Axis fabo_9axis;

//VARIABLES
//VOLTAJE-----------------------------------------------------------------------
float vPow = 1;
float r1 = 470000; //Valor de la resitencia 1 160kΩ
float r2 = 6200;  //Valor de la resitencia 2 13kΩ
float v2;

//GPS---------------------------------------------------------------------------
float latitud;
float longitud;
float altura;
char area;

//CONDICIONES AREAS-------------------------------------------------------------
const int estado_carro = 5;
const int rele = 6;
int valor_estado;
int on_off;


//BLUETOOTH---------------------------------------------------------------------
char bluetooth;

//MPU-9250----------------------------------------------------------------------
float ax, ay, az;
float accel_ang_x;
float accel_ang_y;
float aceleracion;

void setup() {
  Serial.begin(9600);
  analogReference(INTERNAL);
  //analogReference(INTERNAL1V1); //solo Arduino Mega

  pinMode(estado_carro, INPUT);
  pinMode(bluetooth, INPUT);
  pinMode(rele, OUTPUT);

  /* Serial.println("Voltaje,Corriente,latitud,longitud,velocidad,area,angulo_x,angulo_y");*/
  ss.begin(GPSBaud);
}
void loop()
{
  //LLAMAMAMOS A LAS FUNCIONES--------------------------------------------------
  v2 = voltaje();
  float Irms = get_corriente();
  latitud, longitud, altura, area = datos_gps();
  accel_ang_x, accel_ang_y, aceleracion = get_inclinacion();
  on_off = get_condiciones();

  //voltaje---------------------------------------------------------------------
  Serial.print(v2);
  Serial.print(String(","));

  //corriente-------------------------------------------------------------------
  Serial.print(Irms);
  Serial.print(String(","));

  //gps-------------------------------------------------------------------------
  Serial.print(latitud, 6);
  Serial.print(String(","));
  Serial.print(longitud, 6);
  Serial.print(String(","));
  Serial.print(aceleracion);
  Serial.print(String(","));
  Serial.print(area);
  Serial.print(String(","));

  //angulo----------------------------------------------------------------------
  Serial.print(accel_ang_x);
  Serial.print(String(","));
  Serial.print(accel_ang_y);
  Serial.println(String(","));



  //estado del rele------------------------------------------------------------
  digitalWrite(rele, on_off);
}

float voltaje() {
  float v = (analogRead(A3) * vPow) / 1024.0;
  float v2 = v / (r2 / (r1 + r2));
  return v2;
}

float get_corriente() {
  float voltajeSensor;
  float corriente = 0;
  float Sumatoria = 0;
  long tiempo = millis();
  int N = 0;

  while (millis() - tiempo < 500) //Duración 0.5 segundos(Aprox. 30 ciclos de 60Hz)
  {
    voltajeSensor = analogRead(A0) * (1.1 / 1023.0);////voltaje del sensor
    corriente = voltajeSensor * 100.0; //corriente=VoltajeSensor*(100A/1.02V)
    Sumatoria = Sumatoria + sq(corriente); //Sumatoria de Cuadrados
    N = N + 1;
    /*delay(1000);*/
  }
  Sumatoria = Sumatoria * 2; //Para compensar los cuadrados de los semiciclos negativos.
  corriente = sqrt((Sumatoria) / N); //ecuación del RMS
  return (corriente);
}

float datos_gps() {
  while (ss.available() > 0)
    gps.encode(ss.read());
  if (gps.location.isUpdated() || gps.speed.isUpdated()) {
    latitud = float(gps.location.lat());
    longitud = float(gps.location.lng());
  }

  if ((gps.location.lat() > -2.895510) || (gps.location.lat() < -2.899623) || (gps.location.lng() > -79.002300) || (gps.location.lng() < -79.006497)) {
    area = '3'; //Fuera deArea
  }
  else if (((gps.location.lat() < -2.895510) && (gps.location.lat() > -2.899623) && (gps.location.lng() < -79.002300) && (gps.location.lng() > -79.006497)) && ((gps.location.lat() > -2.896805) || (gps.location.lat() < -2.898211) || (gps.location.lng() >  -79.003708) || (gps.location.lng() < -79.005202))) {
    area = '2'; //Advertencia
  }
  else if ((gps.location.lat() <-2.896805) && (gps.location.lat() > -2.898211) && (gps.location.lng() < -79.003708) && (gps.location.lng() > -79.005202)) {
    area = '1'; //Area segura
  }
  return latitud, longitud, area;
}

float get_inclinacion()
{
  if (fabo_9axis.begin()) {
  }
  fabo_9axis.readAccelXYZ(&ax, &ay, &az);
  accel_ang_x = atan(ax / sqrt(pow(ay, 2) + pow(az, 2))) * (180.0 / 3.14);
  accel_ang_y = atan(ay / sqrt(pow(ax, 2) + pow(az, 2))) * (180.0 / 3.14);
  aceleracion = ay;
  return accel_ang_x, accel_ang_y, aceleracion;
}

int get_condiciones()
{
  if(valor_estado = digitalRead(estado_carro))
  {
  Serial.available();
  bluetooth = Serial.read();
  }


    if ((area == '1' || area == '2')) {
      on_off = HIGH;
      bluetooth = 'I';
    }

    if ((area == '3'))
    {
      if (millis() < 10000) {
        on_off = HIGH;
      }
      if ((millis() > 10000) && (bluetooth != 'C')) {
        on_off = LOW;
      }
      if ((millis() > 10000) && (bluetooth == 'C')) {
        on_off = HIGH;
      }
    }
  

  return on_off;
}
