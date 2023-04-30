// Uso de potenciometro y señales analógicas
#include <Servo.h>
#include <Wire.h>
#include <LiquidCrystal_I2C.h>
#include <SoftwareSerial.h>

Servo Servo1;
int const PotPin = A0;  //potenciometro
int PotVal;
int angle;



//variables de los ultrasonicos
const int Trigger1 = 6;  //Pin digital 2 para el Trigger del sensor 1
const int Echo1 = 7;     //Pin digital 3 para el Echo del sensor 1

const int Trigger2 = 3;  //Pin digital 2 para el Trigger del sensor 2
const int Echo2 = 2;     //Pin digital 3 para el Echo del sensor 2


long tiempo1;  //variables de toma de tiempo
long tiempo2;


bool bandera1 = false;  //banderas de confirmacion de paso de la pelota
bool bandera2 = false;






//clase que representa un lanzamiento de pelota
class Lanzamiento {
public:
  int intento;
  int distancia;
  double tiempo;
  double velocidad;
  double aceleracion;
  ;

public:
  Lanzamiento(int intento, int distancia, double tiempo, double velocidad, double aceleracion) {

    this->intento = intento;
    this->distancia = distancia;
    this->tiempo = tiempo;
    this->velocidad = velocidad;
    this->aceleracion = aceleracion;
  }
};

//variable contadora de intentos
int intentos = 0;
//array de intentos
Lanzamiento* lanzamientos[200];

//pantalla lcd
LiquidCrystal_I2C lcd(0x27, 20, 4);  // establece la dirección LCD en 0x27 para una pantalla de 16 caracteres y 2 líneas


void setup() {
  Servo1.attach(5);  //pin del servo
  Serial.begin(9600);

  pinMode(Trigger1, OUTPUT);    //pin como salida
  pinMode(Echo1, INPUT);        //pin como entrada
  pinMode(Trigger2, OUTPUT);    //pin como salida
  pinMode(Echo2, INPUT);        //pin como entrada
  digitalWrite(Trigger1, LOW);  //Inicializamos el pin con 0
  digitalWrite(Trigger2, LOW);  //Inicializamos el pin con 0


  lcd.init();  // initialize the lcd
  lcd.backlight();

  SoftwareSerial SerialPC(24, 26);
  Servo1.write(0);  //pone el servo al angulo obtenido
}
void loop() {
  //codigo que controla el servo y el potenciometro
  PotVal = analogRead(PotPin);  //lee el angulo del potenciometro

  angle = map(PotVal, 0, 1023, 0, 180);  // convierte el angulo del potenciometro en valores entre 0 y 180


  if (angle >= 0 && angle <= 60) {
    angle = 15;
  } else if (angle >= 61 && angle <= 120) {
    angle = 30;
  } else {
    angle = 45;
  }
  // Serial.println(angle);
  Servo1.write(angle);  //pone el servo al angulo obtenido


  long t1;  //timepo que demora en llegar el eco
  long d1;  //distancia en centimetros

  long t2;  //timepo que demora en llegar el eco
  long d2;  //distancia en centimetros

  digitalWrite(Trigger1, HIGH);
  delayMicroseconds(10);  //Enviamos un pulso de 10us
  digitalWrite(Trigger1, LOW);
  t1 = pulseIn(Echo1, HIGH);  //obtenemos el ancho del pulso
  d1 = t1 / 59;               //escalamos el tiempo a una distancia en cm

  //Segundo ultrasonico
  digitalWrite(Trigger2, HIGH);
  delayMicroseconds(10);  //Enviamos un pulso de 10us
  digitalWrite(Trigger2, LOW);
  t2 = pulseIn(Echo2, HIGH);  //obtenemos el ancho del pulso
  d2 = t2 / 59;               //escalamos el tiempo a una distancia en cm


  //evalua si la distacnia medida por el primer sensor es menor a 5 cm y si la distancia es mayor a 0cm (en caso de error)
  if (d1 < 5 && d1 > 0) {
    tiempo1 = millis();  //toma el tiempo de la primera medicion
    bandera1 = true;     //activa bandera de primer medicion
  }


  //evalua si la distacnia medida por el segundo sensor es menor a 5 cm y si la distancia es mayor a 0cm (en caso de error)
  if (bandera1 && d2 < 5 && d2 > 0) {
    tiempo2 = millis();  //toma el tiempo de la medicion
    bandera2 = true;     //activa bandera de la segunda medicion
  }

  //evaluar si las dos mediciones ya han sido tomadas, evaluar si segundo tiempo se tomo despues de haber tomado el primero
  if (bandera1 && bandera2 && tiempo2 > tiempo1) {
    bandera1 = false;  //volver a banderas a valores iniciales
    bandera2 = false;
    double tiempoTotal = (tiempo2 - tiempo1) / 1000.00;  //devuelve tiempo en segundos
    tiempo1 = 0;                                         //volver tiempos a valores iniciales
    tiempo2 = 0;

    //encontrar aceleracion con a = 100/t^2 donde t = tiempoTotal
    double aceleracion = 100.00 / (tiempoTotal * tiempoTotal);
    //encontrar velocidad final con v = a * t donde a = igual a aceleracion y t = tiempoTotal
    double velocidadFinal = aceleracion * tiempoTotal;

    //imprimir valores en pantalla lcd

    Lanzamiento* lanzamiento = new Lanzamiento(intentos + 1, 50, tiempoTotal, velocidadFinal, aceleracion);

    lanzamientos[intentos] = lanzamiento;



    if (intentos == 2) {
      intentos = 0;
      Serial.println("Distancia, Tiempo, Velocidad, Aceleracion");
      int distanciaMedia = 0;
      double tiempoMedio = 0.0;
      double velocidadMedia = 0.0;
      double aceleracionMedia = 0.0;
      for (int x = 0; x < 3; x++) {
        distanciaMedia = distanciaMedia + lanzamientos[x]->distancia;
        tiempoMedio = tiempoMedio + lanzamientos[x]->tiempo;
        velocidadMedia = velocidadMedia + lanzamientos[x]->velocidad;
        aceleracionMedia = aceleracionMedia + lanzamientos[x]->aceleracion;
        lanzamientos[x] = NULL;  //eliminamos el dato ya leido
      }

      Serial.println(String(distanciaMedia / 3) + "," + String(tiempoMedio / 3) + "," + String(velocidadMedia / 3) + "," + String(aceleracionMedia / 3));

    } else {
      intentos++;  //sumamos uno a los intentos
    }


    //imprimir los resultados en una lcd
    lcd.clear();

    String salida = "";
    lcd.setCursor(0, 0);
    salida.concat("D:");
    salida.concat(lanzamiento->distancia);
    salida.concat("T:");
    salida.concat(lanzamiento->tiempo);
    lcd.print(salida);  //imprimir primera parte de la salida
    salida = "";        //resetear la salida
    lcd.setCursor(0, 1);
    salida.concat("V:");
    salida.concat(lanzamiento->velocidad);
    salida.concat("A:");
    salida.concat(lanzamiento->aceleracion);
    lcd.print(salida);
  }

  //codigo que controla los ultra sonicos
}
