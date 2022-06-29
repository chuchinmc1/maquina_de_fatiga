/**************************************************************************
 * PROGRAMA: ESP32_Maquina_de_fatiga
 * 
 * Programado por: M.C. Jesús Medina Cervantes
 * Fecha: 17/junio/2022.
 * 
 * Pantalla OLED con driver SSD1306 
 * http://www.adafruit.com/category/63_98
 * 128x64 pixel display, comunicación I2C 
 * Librería escrita por: Limor Fried/Ladyada 
 * para Adafruit Industries, licencia BSD
 * https://github.com/bogde/HX711
 * MIT License
 *(c) 2018 Bogdan Necula
 */

// Bibliotecas

#include <SPI.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <WiFi.h>
#include "HX711.h"
#include <PubSubClient.h> //Biblioteca para conexion MQTT

//Datos de WiFi
const char* ssid = "IZZI-196B";  // Aquí debes poner el nombre de tu red
const char* password = "A811FC18196B";  // Aquí debes poner la contraseña de tu red
//const char* ssid = "jesusmc";  // Aquí debes poner el nombre de tu red
//const char* password = "chuchinmc";  // Aquí debes poner la contraseña de tu red

//Datos del broker MQTT
const char* mqtt_server = "192.168.0.43"; // Si estas en una red local, coloca la IP asignada, en caso contrario, coloca la IP publica
IPAddress server(192,168,0,43);
//const char* mqtt_server = "192.168.43.111"; // Si estas en una red local, coloca la IP asignada, en caso contrario, coloca la IP publica
//IPAddress server(192,168,43,111);

// Objetos
WiFiClient espClient; // Este objeto maneja los datos de conexion WiFi
PubSubClient client(espClient); // Este objeto maneja los datos de conexion al broker

// Definición del timer
hw_timer_t * timer = NULL;

portMUX_TYPE synch = portMUX_INITIALIZER_UNLOCKED;
portMUX_TYPE timerMux = portMUX_INITIALIZER_UNLOCKED;

// Definición para el OLED Display
// Conexión con el ESP32: GPIO21(SDA), GPIO22(SCL), 3.3V
#define I2C_SDA 21
#define I2C_SCL 22
#define SCREEN_WIDTH 128 // OLED display ancho en pixeles
#define SCREEN_HEIGHT 64 // OLED display alto en pixeles
#define OLED_RESET     -1 // Reset pin # (Usar -1 si se comparte el Reset pin del ESP32)
#define SCREEN_ADDRESS 0x3C // 0x3C para OLED display de 128x64 pixeles
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);

#define NUMFLAKES     10 // Número de copos de nieve en la animación de ejemplo

#define LOGO_HEIGHT   16
#define LOGO_WIDTH    16
static const unsigned char PROGMEM logo_bmp[] =
{ 0b00000000, 0b11000000,
  0b00000001, 0b11000000,
  0b00000001, 0b11000000,
  0b00000011, 0b11100000,
  0b11110011, 0b11100000,
  0b11111110, 0b11111000,
  0b01111110, 0b11111111,
  0b00110011, 0b10011111,
  0b00011111, 0b11111100,
  0b00001101, 0b01110000,
  0b00011011, 0b10100000,
  0b00111111, 0b11100000,
  0b00111111, 0b11110000,
  0b01111100, 0b11110000,
  0b01110000, 0b01110000,
  0b00000000, 0b00110000 };

// Led del ESP32
#define LED_BUILTIN 2
volatile int ledState = LOW;

// HX711 CONEXIONES
// Vcc --> 5V
const int LOADCELL_DOUT_PIN = 19;   // GPIO19 ESP32 (SPI MISO)
const int LOADCELL_SCK_PIN = 18;    // GPIO18 ESP32 (SPI SCK)

HX711 scale;
char valor[10];
char valvueltas[10];
float lectura;
float lectura_anterior = 0.0;
volatile int frecuencia;
uint32_t tiempo_i;
uint32_t tiempo_f;
uint32_t vueltas_total;
volatile int8_t bandera = 0;
volatile int8_t estado = 0;

// Definición de pines
#define BTN_RESET 35        // Resetea el ESP32
#define MOTOR_SWITCH 33     // Paro del motor
#define CNY70 34    // Sensor que detecta las vueltas del motor
volatile uint32_t vueltas = 0;
volatile uint32_t vueltast = 0;

// Variables
unsigned long timeNow, timeLast; // Variables de control de tiempo no bloqueante
int dato = 0; // Contador
int wait = 2000;  // Indica la espera cada 5 segundos para envío de mensajes MQTT
char dataString[8]; // Define una arreglo de caracteres para enviarlos por MQTT, especifica la longitud del mensaje en 8 caracteres
String texto;
char datos[80];

void IRAM_ATTR isr() {    // Sensor CNY70 cuentas vueltas
  portENTER_CRITICAL(&synch);
  vueltas ++;
  vueltast ++;
  portEXIT_CRITICAL(&synch);
}

void IRAM_ATTR isr_reset() {  // Poner a cero el no. de vueltas para iniciar prueba
  portENTER_CRITICAL(&synch);
  vueltas = 0;
  vueltast = 0;
  bandera = 0;
  estado = 0;
  portEXIT_CRITICAL(&synch);
}

void IRAM_ATTR onTimer() {
  portENTER_CRITICAL_ISR(&timerMux);
  frecuencia = vueltas;
  vueltas = 0;
  ledState = !ledState;
  digitalWrite(LED_BUILTIN, ledState);
  portEXIT_CRITICAL_ISR(&timerMux);
}

void setup() {
  delay(2000);
  
  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, ledState);
  
  pinMode(CNY70, INPUT);
  attachInterrupt(CNY70, isr, RISING); //Detecta blanco

  pinMode(BTN_RESET, INPUT);
  attachInterrupt(BTN_RESET, isr_reset, RISING);

  pinMode(MOTOR_SWITCH, OUTPUT);
  digitalWrite(MOTOR_SWITCH, HIGH);

  // SSD1306_SWITCHCAPVCC = generate display voltage from 3.3V internally
  if(!display.begin(SSD1306_SWITCHCAPVCC, SCREEN_ADDRESS)) {
    //Serial.println(F("SSD1306 allocation failed"));
    for(;;); // Se detiene el programa mediante ciclo infinito
  }

  // Show initial display buffer contents on the screen --
  // the library initializes this with an Adafruit splash screen.
  //display.display();
  //delay(2000);
  // Clear the buffer
  display.clearDisplay();
  display.setTextSize(2);      // Normal 1:2 pixel scale
  display.setCursor(0, 10);
  display.setTextColor(SSD1306_WHITE); // Draw white text
  display.println("Prueba de Fatiga");
  display.display();
  delay(2000);
  display.clearDisplay();
  
  scale.begin(LOADCELL_DOUT_PIN, LOADCELL_SCK_PIN);

  scale.set_scale(112230.769f); 
  scale.tare();   // reset the scale to 0
  
  display.setTextSize(1);
  display.setTextColor(SSD1306_WHITE);
  display.setCursor(40, 0);
  display.println("Fuerza:");
  display.setCursor(30, 25);
  display.println("Velocidad:");
  display.setCursor(35, 45);
  display.println("Vueltas:");
  display.display();

 
  timer = timerBegin(0, 40000, true);
  timerAttachInterrupt(timer, &onTimer, true);
  timerAlarmWrite(timer, 2000, true);
  timerAlarmEnable(timer);

  // Iniciar comunicación serial
  Serial.begin (115200);

  Serial.println();
  Serial.println();
  Serial.print("Conectar a ");
  Serial.println(ssid);

  WiFi.begin(ssid, password); // Esta es la función que realiz la conexión a WiFi
 
  while (WiFi.status() != WL_CONNECTED) { // Este bucle espera a que se realice la conexión
    delay(500); //dado que es de suma importancia esperar a la conexión, debe usarse espera bloqueante
    Serial.print(".");  // Indicador de progreso
  }
  // Cuando se haya logrado la conexión, el programa avanzará, por lo tanto, puede informarse lo siguiente
  Serial.println();
  Serial.println("WiFi conectado");
  Serial.println("Direccion IP: ");
  Serial.println(WiFi.localIP());

  delay (1000); // Esta espera es solo una formalidad antes de iniciar la comunicación con el broker

  // Conexión con el broker MQTT
  client.setServer(mqtt_server, 1883); // Conectarse a la IP del broker en el puerto indicado
  delay(1500);  // Esta espera es preventiva, espera a la conexión para no perder información

  timeLast = millis (); // Inicia el control de tiempo
   
}

void loop() {

  //Verificar siempre que haya conexión al broker
  if (!client.connected()) {
    reconnect();  // En caso de que no haya conexión, ejecutar la función de reconexión, definida despues del void setup ()
  }// fin del if (!client.connected())
  client.loop(); // Esta función es muy importante, ejecuta de manera no bloqueante las funciones necesarias para la comunicación con el broker
  
  lectura = scale.get_units(5);
  
  if ((lectura - lectura_anterior) < -5.0){
    vueltas_total = vueltast;
    digitalWrite(MOTOR_SWITCH, LOW);
    estado = 1;
    display.clearDisplay();
    display.setTextSize(1); 
    display.setCursor(10, 10);
    display.println("Prueba Terminada!");
    display.setCursor(10, 30);
    display.println("Total de vueltas:");
    sprintf(valor,"%u", vueltas_total);
    display.setCursor(20, 50);
    display.println(valor);
    display.display();
    texto = "{\"carga\":"+ String(lectura,2)+ ",\"velocidad\":"+ String(frecuencia*60)+ ",\"no_vueltas\":"+ String(vueltas_total)+ ",\"estado\":"+ String(estado)+ "}";
    texto.toCharArray(datos,80);
    client.publish("maquina_fatiga/datos",datos); // Esta es la función que envía los datos por MQTT, especifica el tema y el valor
    bandera = 1;
    while(bandera == 1){
    }
  }
  lectura_anterior = lectura;
    
  display.fillRect(40,10,60,8,SSD1306_BLACK);  
  display.setCursor(40, 10);
  sprintf(valor,"%.2f kgf", lectura);
  display.println(valor);
  display.fillRect(40,35,60,8,SSD1306_BLACK);  
  sprintf(valvueltas,"%d RPM", frecuencia*60);
  display.setCursor(40, 35);
  display.println(valvueltas);
  display.fillRect(20,55,90,8,SSD1306_BLACK);  
  sprintf(valvueltas,"%d", vueltast);
  display.setCursor(20, 55);
  display.println(valvueltas);
  display.display();

  timeNow = millis(); // Control de tiempo para esperas no bloqueantes
  if (timeNow - timeLast > wait) { // Manda un mensaje por MQTT cada cinco segundos
    timeLast = timeNow; // Actualización de seguimiento de tiempo
    texto = "{\"carga\":"+ String(lectura,2)+ ",\"velocidad\":"+ String(frecuencia*60)+ ",\"no_vueltas\":"+ String(vueltast)+ ",\"estado\":"+ String(estado)+"}";
    texto.toCharArray(datos,80);
    client.publish("maquina_fatiga/datos",datos); // Esta es la función que envía los datos por MQTT, especifica el tema y el valor
  }// fin del if (timeNow - timeLast > wait)
  
}

// Funciones de usuario

// Función para reconectarse
void reconnect() {
  // Bucle hasta lograr conexión
  while (!client.connected()) { // Pregunta si hay conexión
    Serial.print("Tratando de contectarse...");
    // Intentar reconexión
    if (client.connect("ESP32CAMClient")) { //Pregunta por el resultado del intento de conexión
      Serial.println("Conectado");
    }// fin del  if (client.connect("ESP32CAMClient"))
    else {  //en caso de que la conexión no se logre
      Serial.print("Conexion fallida, Error rc=");
      Serial.print(client.state()); // Muestra el codigo de error
      Serial.println(" Volviendo a intentar en 5 segundos");
      // Espera de 5 segundos bloqueante
      delay(5000);
      Serial.println (client.connected ()); // Muestra estatus de conexión
    }// fin del else
  }// fin del bucle while (!client.connected())
}// fin de void reconnect()
