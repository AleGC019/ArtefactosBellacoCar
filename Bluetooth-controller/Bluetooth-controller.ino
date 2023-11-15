// Librerias a utilizar para la implementacion del proyecto
/************* Elementos para la conexion por medios fisicos **************/
#include "Wire.h"

/************* Elementos para la conexion WiFi y enviar datos a la plataforma **************/
#include "WiFi.h"
#include "WiFiClientSecure.h"
#include "Adafruit_MQTT.h"
#include "Adafruit_MQTT_Client.h"

/************* Elementos para gestionar el control de los motores **************/
#include "ESP32Servo.h"

/************* Elementos para gestionar todas la informacion y funciones del modulo NEO 6M **************/
#include "TinyGPSPlus.h"

/************* Elementos para la conexion  por de Bluetooth para el control remoto del carro **************/
#include "BluetoothSerial.h"

 
/************************* WiFi Access Point *********************************/

#define WLAN_SSID "NombreDeLaRed"
#define WLAN_PASS "Constraseña"

/************************* Adafruit.io Setup *********************************/

#define AIO_SERVER "io.adafruit.com"
#define AIO_SERVERPORT 1883  // use 8883 for SSL
#define AIO_USERNAME "Alegc019"
#define AIO_KEY "aio_iqRA47vdqiOBd4MqXoPOd3OBgTtX"

/************ Global State (you don't need to change this!) ******************/

// Create an ESP32 WiFiClient class to connect to the MQTT server.
// or... use WiFiClientSecure for SSL
WiFiClient client;

// Setup the MQTT client class by passing in the WiFi client and MQTT server and login details.
Adafruit_MQTT_Client mqtt(&client, AIO_SERVER, AIO_SERVERPORT, AIO_USERNAME, AIO_USERNAME, AIO_KEY);

/****************************** Feeds for Publishing***************************************/
// Setup a feed called 'coordenadas' for publishing.
Adafruit_MQTT_Publish adaFeed = Adafruit_MQTT_Publish(&mqtt, AIO_USERNAME "/feeds/location");

// Define GPS RX and TX pins para el modulo GPS
#define GPS_RX 16
#define GPS_TX 17
HardwareSerial neogps(1);

// Inicializacion de los modulos utilizados en el carro
BluetoothSerial BTSerial;
TinyGPSPlus gps;

// Define L298n module IO Pin
int ENA = 12; // Enable pin for motor A
int IN1 = 21; // Input 1 pin for motor A
int IN2 = 5;  // Input 2 pin for motor A
int ENB = 14; // Enable pin for motor B
int IN3 = 18; // Input 1 pin for motor B
int IN4 = 19; // Input 2 pin for motor B

// Define proximite sensor 
int IFA = 27;
int IFB = 26;
int IFC = 25;
int IFD = 23;

// Define motor speed
#define MotorASpeed 75 // Speed for motor A
#define MotorBSpeed 75 // Speed for motor B

int pins[6] = {ENA, IN1, IN2, ENB, IN3, IN4};

void setup() {

  Serial.begin(115200);
  
  neogps.begin(9600, SERIAL_8N1, GPS_RX, GPS_TX);

  BTSerial.begin(9600);

  for (int i = 0; i < 6; i++) {
    pinMode(pins[i], OUTPUT);
  }

  pinMode(IFA, INPUT);
  pinMode(IFB, INPUT);
  pinMode(IFC, INPUT);
  pinMode(IFD, INPUT);

  //****************************************** Connect to WiFi access point.
  Serial.println();
  Serial.println();
  Serial.print("Connecting to ");
  Serial.println(WLAN_SSID);

  WiFi.begin(WLAN_SSID, WLAN_PASS);
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.println();
  Serial.println("WiFi connected");
  Serial.println("IP address: ");
  Serial.println(WiFi.localIP());

  //*********************************************

  BTSerial.begin("BellacoCar"); // Bluetooth device name
  Serial.println("The device started, now you can pair it with bluetooth!");

 // Start Bluetooth serial at 9600 baud rate
}

void loop(){


  /* ********* Elementos Adafruit ****************/

  MQTT_connect();

  mqtt.processPackets(5000);

  if (!mqtt.ping()) {  // ping the server to keep the mqtt connection alive
    mqtt.disconnect();
  }

  /* ********* Elementos Adafruit **************** */


  /* ******* Inicio de conexión con el controlador de los motores ***************** */


  digitalWrite(ENA, HIGH);
  digitalWrite(ENB, HIGH);
   
  /* ******* Inicio de conexión con el controlador de los motores ***************** */



  /* ******* Inicio de lectura de los sensores de proximidad ***************** */


  int sensorA = digitalRead(IFA);
  int sensorB = digitalRead(IFB);
  int sensorC = digitalRead(IFC);
  int sensorD = digitalRead(IFD);

  /* ******* Inicio de lectura de los sensores de proximidad  ***************** */



  /* ******* Si un sensor detecta un objeto es impreso en consola la lectura ***************** */

  if(sensorA == LOW || sensorB == LOW || sensorC == LOW || sensorD == LOW) {

    Serial.println(sensorA);
    Serial.println(sensorB);
    Serial.println(sensorC);
    Serial.println(sensorD);
    
  }

  /* ******* Si un sensor detecta un objeto es impreso en consola la lectura ***************** */



  /* ******* Lectura del modulo Bluetooth para las indicaciones de movimiento ***************** */
  if (BTSerial.available() ) { // If Bluetooth data is available
    
    char command = BTSerial.read(); // Read the incoming data

    Serial.print("Comando: ");
    Serial.print(command); // Impresion del comando recibido
    Serial.println(" ");

    if(command == 'F'){    //If de movimiento frontal

      digitalWrite(IN1, HIGH);
      digitalWrite(IN2, LOW);
      digitalWrite(IN3, HIGH);
      digitalWrite(IN4, LOW);

      // Si los sensores A y B reciben una señal, detener el coche y luego hacer que retroceda
      if (sensorA == LOW || sensorB == LOW) {
        // Detener el coche
        digitalWrite(IN1, LOW);
        digitalWrite(IN2, LOW);
        digitalWrite(IN3, LOW);
        digitalWrite(IN4, LOW);

        delay(1000); // Esperar un segundo

        // Hacer que el coche retroceda
        digitalWrite(IN1, LOW);
        digitalWrite(IN2, HIGH);
        digitalWrite(IN3, LOW);
        digitalWrite(IN4, HIGH);

        delay(1000); // Retroceder durante un segundo

        // Detener el coche de nuevo después de retroceder
        digitalWrite(IN1, LOW);
        digitalWrite(IN2, LOW);
        digitalWrite(IN3, LOW);
        digitalWrite(IN4, LOW);
      }
    
      
    }else if(command == 'B'){//If de movimiento hacia atras

      digitalWrite(IN1, LOW);
      digitalWrite(IN2, HIGH);
      digitalWrite(IN3, LOW);
      digitalWrite(IN4, HIGH); 

      // Si los sensores C y D reciben una señal, detener el coche y luego hacer que avance
      if (sensorC == LOW || sensorD == LOW) {
          // Detener el coche
        digitalWrite(IN1, LOW);
        digitalWrite(IN2, LOW);
        digitalWrite(IN3, LOW);
        digitalWrite(IN4, LOW);

        delay(1000); // Esperar un segundo

        // Hacer que el coche avance
        digitalWrite(IN1, HIGH);
        digitalWrite(IN2, LOW);
        digitalWrite(IN3, HIGH);
        digitalWrite(IN4, LOW);

        delay(1000); // Avanzar durante un segundo

        // Detener el coche de nuevo después de avanzar
        digitalWrite(IN1, LOW);
        digitalWrite(IN2, LOW);
        digitalWrite(IN3, LOW);
        digitalWrite(IN4, LOW);
      
      }
      

    }else if(command == 'R'){// If de movimiento hacia la derecha

      digitalWrite(IN1, HIGH);
      digitalWrite(IN2, LOW);
      digitalWrite(IN3, LOW);
      digitalWrite(IN4, HIGH);

    }else if(command == 'L'){ // If de movimiento hacia la izquierda

      digitalWrite(IN1, LOW);
      digitalWrite(IN2, HIGH);
      digitalWrite(IN3, HIGH);
      digitalWrite(IN4, LOW);

    }else if(command == 'C'){ // If del comando para determinar las coordenadas del Bellaco Car

      //Variables donde se guarda la infromación de las coordenadas
      float lat = 0;
      float lng = 0; 

      //String de concatenación de las coordenadas
      String coordenadas;

      //Bandera para determianr si es posible determinar las coordenadas
      bool nuevasCoordenadas = false;

      //While para ver si el modulo esta habilitado y ver si es posible leer las coordenadas
      while (neogps.available()) {
        if (gps.encode(neogps.read())) {
          nuevasCoordenadas = true;
        }
      }

      //En caso sea posible establecer conexion
      if(nuevasCoordenadas == true){
        
        //Cambiamos la bandera puesto que una vez tenemos las coordenadas y las enviamos,
        // si las deseamos nuevamente pues debemos validar el proceso de lectura
        nuevasCoordenadas = false;

        Serial.println(gps.satellites.value());

        if(gps.location.isValid() == 1){

          //Coordenadas de latitud
          Serial.print("Latitud: ");
          Serial.print(gps.location.lat(), 8);
          Serial.print("\n");

          lat = gps.location.lat();

          //Coordenadas de longitud
          Serial.print("Longitud: ");
          Serial.print(gps.location.lng(), 8);
          Serial.print("\n");

          lng = gps.location.lng();

          //Concatenacion
          coordenadas = String(lat) + ", " + String(lng);


          Serial.print("Coordenadas juntas: ");
          Serial.print(coordenadas);

          //Envio de la informacion
          if (!adaFeed.publish('coordenadas')) {
            Serial.println(F("Failed"));
          } else {
            Serial.println(F("OK!"));
          }
        }
        else{

          Serial.println("Coordenadas no disponibles");

        }
      }

    }
    else{ //Else en caso no haya comandos el carro se detiene
      digitalWrite(IN1, LOW);
      digitalWrite(IN2, LOW);
      digitalWrite(IN3, LOW);
      digitalWrite(IN4, LOW);
    }
  }else{
    Serial.println("Conexion BT no establecida - Esperando comando de movimiento");
  }
  /* ******* Lectura del modulo Bluetooth para las indicaciones de movimiento ***************** */
}

void MQTT_connect() {
  int8_t ret;
  // Stop if already connected.
  if (mqtt.connected()) {
    return;
  }
  Serial.print("Connecting to MQTT... ");
  uint8_t retries = 3;
  while ((ret = mqtt.connect()) != 0) {  // connect will return 0 for connected
    Serial.println(mqtt.connectErrorString(ret));
    Serial.println("Retrying MQTT connection in 1 seconds...");
    mqtt.disconnect();
    delay(1000);  // wait 1 seconds
    retries--;
    if (retries == 0) {  // basically die and wait for WDT to reset me
      while (1);
    }
  }
  Serial.println("MQTT Connected!");
}