// Librerías
#include <esp_timer.h>
// -----------------------Librerías OLED----------------------------
#include <Wire.h>			          // libreria para bus I2C
#include <Adafruit_GFX.h>		    // libreria para pantallas graficas
#include <Adafruit_SSD1306.h>		// libreria para controlador SSD1306

#define ANCHO 128			// reemplaza ocurrencia de ANCHO por 128
#define ALTO 64				// reemplaza ocurrencia de ALTO por 64

#define OLED_RESET 4			// necesario por la libreria pero no usado
Adafruit_SSD1306 oled(ANCHO, ALTO, &Wire, OLED_RESET);	// crea objeto
// -----------------------Librerías OLED----------------------------FIN
////*Definición de pines de McU para control de registros*///
//int pinData  = 15;
//int pinLatch = 33;
//int pinClock = 27;
//int pinOE = 12;

int pinData  = 12;
int pinLatch = 33;
int pinClock = 15;
int pinOE = 27;
////*Definición de pines de McU de entrada para pruebas*///
#define CantidadBotonEntrada 4
int botonEntrada[CantidadBotonEntrada] = {26, 25, 34, 39};
int estadoBoton[CantidadBotonEntrada] = {LOW, LOW, LOW, LOW};

#define LED_PIN 13 // The pin number of the LED

// Variables Globales
int modo = 0;
//Variable de control del Timer (millis)
unsigned long previousTime = 0;
unsigned long te = 375;
unsigned long t = 10000;
// Arreglos de programación
unsigned long prog0[31] = {
                            0b00000000000000000000000000000000, // 

                            0b10010010010000100100000000000000, // ***Escenario 1***

                            0b00000000010000100100000000000000, // Transición de Verde a Ambar
                            0b10010010010000100100000000000000, // Transición de Verde a Ambar
                            0b00000000010000100100000000000000, // Transición de Verde a Ambar
                            0b10010010010000100100000000000000, // Transición de Verde a Ambar
                            0b00000000010000100100000000000000, // Transición de Verde a Ambar
                            0b10010010010000100100000000000000, // Transición de Verde a Ambar
                            0b00000000010000100100000000000000, // Transición de Verde a Ambar
                            0b10010010010000100100000000000000, // Transición de Verde a Ambar

                            0b01001001010000100100000000000000, // Tiempo de ambar  

                            0b00100100110000110000000000000000, // ***Escenario 2***

                            0b00100100100000100000000000000000, // Transición de Verde a Ambar
                            0b00100100110000110000000000000000, // Transición de Verde a Ambar
                            0b00100100100000100000000000000000, // Transición de Verde a Ambar
                            0b00100100110000110000000000000000, // Transición de Verde a Ambar
                            0b00100100100000100000000000000000, // Transición de Verde a Ambar
                            0b00100100110000110000000000000000, // Transición de Verde a Ambar
                            0b00100100100000100000000000000000, // Transición de Verde a Ambar
                            0b00100100110000110000000000000000, // Transición de Verde a Ambar

                            0b00100100101000101000000000000000, // Tiempo de ambar  

                            0b00100110000110000100000000000000, // ***Escenario 3***

                            0b00100110000100000100000000000000, // Transición de Verde a Ambar
                            0b00100110000110000100000000000000, // Transición de Verde a Ambar
                            0b00100110000100000100000000000000, // Transición de Verde a Ambar
                            0b00100110000110000100000000000000, // Transición de Verde a Ambar
                            0b00100110000100000100000000000000, // Transición de Verde a Ambar
                            0b00100110000110000100000000000000, // Transición de Verde a Ambar
                            0b00100110000100000100000000000000, // Transición de Verde a Ambar
                            0b00100110000110000100000000000000, // Transición de Verde a Ambar

                            0b00100110000101000100000000000000  // Tiempo de ambar  
};    

int longitud0 = sizeof(prog0) / sizeof(prog0[0]);


// Arreglos de programación
unsigned long prog00[31] = {
                            0b00000000000000000000000000000000, // 
                            
                            0b00100100100110010000000000000000, // ***Escenario 1***

                            0b00000000000010010000000000000000, // Transición de Verde a Ambar
                            0b00100100100110010000000000000000, // Transición de Verde a Ambar
                            0b00000000000010010000000000000000, // Transición de Verde a Ambar
                            0b00100100100110010000000000000000, // Transición de Verde a Ambar
                            0b00000000000010010000000000000000, // Transición de Verde a Ambar
                            0b00100100100110010000000000000000, // Transición de Verde a Ambar
                            0b00000000000010010000000000000000, // Transición de Verde a Ambar
                            0b00100100100110010000000000000000, // Transición de Verde a Ambar

                            0b01001001000110010000000000000000, // Transición de Verde a Ambar

                            0b10010010000110000100000000000000, // ***Escenario 2***

                            0b10010010000010000000000000000000, // Transición de Verde a Ambar
                            0b10010010000110000100000000000000, // Transición de Verde a Ambar
                            0b10010010000010000000000000000000, // Transición de Verde a Ambar
                            0b10010010000110000100000000000000, // Transición de Verde a Ambar
                            0b10010010000010000000000000000000, // Transición de Verde a Ambar
                            0b10010010000110000100000000000000, // Transición de Verde a Ambar
                            0b10010010000010000000000000000000, // Transición de Verde a Ambar
                            0b10010010000110000100000000000000, // Transición de Verde a Ambar

                            0b10010010001010001000000000000000, // Transición de Verde a Ambar

                            0b10010000110000110000000000000000, // ***Escenario 3***

                            0b10010000110000010000000000000000, // Transición de Verde a Ambar
                            0b10010000110000110000000000000000, // Transición de Verde a Ambar
                            0b10010000110000010000000000000000, // Transición de Verde a Ambar
                            0b10010000110000110000000000000000, // Transición de Verde a Ambar
                            0b10010000110000010000000000000000, // Transición de Verde a Ambar
                            0b10010000110000110000000000000000, // Transición de Verde a Ambar
                            0b10010000110000010000000000000000, // Transición de Verde a Ambar
                            0b10010000110000110000000000000000, // Transición de Verde a Ambar

                            0b10010000110001010000000000000000  // Transición de Verde a Ambar

};

int longitud = sizeof(prog00) / sizeof(prog00[0]);

unsigned long time0[31] = {  
                            0,

                            10000,
                            375,
                            375,
                            375,
                            375,
                            375,
                            375,
                            375,
                            375,
                            3000,

                            10000,
                            375,
                            375,
                            375,
                            375,
                            375,
                            375,
                            375,
                            375,
                            3000,

                            12000,
                            375,
                            375,
                            375,
                            375,
                            375,
                            375,
                            375,
                            375,
                            3000
};

unsigned long prog1[3] = {
                            0b00000000000000000000000000000000,
                            0b00000000000000000000000000000000,
                            0b10010010010010010010000000000000 
};    

unsigned long time1[3] = {
                            0,
                            375,
                            375 
};    

int longitud1 = sizeof(prog1) / sizeof(prog1[0]);

// Variables de Programación
unsigned long EscOn   = 0b11111111111111111111111111111111; // Todo Apagado
unsigned long EscOff  = 0b00000000000000000000000000000000; // Todo Encendido
///////////////////////////////////*FIN Programación*//////////////////////////////////////////

// Declarar una variable para almacenar el índice del arreglo
int indice = 0;

//////////////*Void Setup*/////////////
void setup() {
  //Inicialización del puerto serial del mCU
  Serial.begin(115200);

  //////////////////////INICIALIZA PANTALLA OLED/////////////////////////
  initOLED();

  //Designación de  pines del mCU como entrada y salida
  ////////////*Definición de pines como salida*////////////
  pinMode(pinData, OUTPUT);
  pinMode(pinLatch, OUTPUT);
  pinMode(pinClock, OUTPUT);
  pinMode(pinOE, OUTPUT);

  pinMode (LED_PIN, OUTPUT); // Set the LED pin as output

  ////////////*Desactivar Registros*////////////////////////
  digitalWrite(pinOE, HIGH);
  ////////////*Activar Registros*////////////////////////
  digitalWrite(pinOE, LOW);
  // Apagado de todas las fases
  ledWrite(0xff,0xff,0xff,0xff);
  //interfaceProg(EscOff);
  delay(2000);
 
  ////////////*Definición de pines como entrada*////////////
  for (int i=0; i<CantidadBotonEntrada; i++){
    pinMode(botonEntrada[i], INPUT);
  }
}
/////////////*Void Loop*/////////////
void loop() {
  // Declaración de variables locales
  ////////////*Activar Registros*////////////////////////
  digitalWrite(pinOE, LOW);

  // Lectura de Modo
  modofunc();
  //aislado();

  
  //delay(1000);

}
//////////////////////*Funciones*/////////////////////////
// Función de interface 32 a 8 bits - en base a variables
void interfaceProg(unsigned long var32Bits) {
    unsigned char var1 = (var32Bits & 0xFF) ^ 0xFF;
    unsigned char var2 = ((var32Bits >> 8) & 0xFF) ^ 0xFF;
    unsigned char var3 = ((var32Bits >> 16) & 0xFF) ^ 0xFF;
    unsigned char var4 = ((var32Bits >> 24) & 0xFF) ^ 0xFF;

    ledWrite(var1,var2,var3,var4);
}
// Función de Modo
void modofunc(){
  int lecturaBoton[CantidadBotonEntrada];
  for (int i=0; i<CantidadBotonEntrada; i++){
    lecturaBoton[i] = digitalRead(botonEntrada[i]);

    if (lecturaBoton[i]==LOW && i==0 && estadoBoton[i] == LOW){
      modo = 0; // Aislado
      indice = 0;
      Serial.println("Modo: Aislado");
      displayInfo("Aislado");
      estadoBoton[i] = HIGH;
      previousTime = millisESP32 ();
    }
    else if (lecturaBoton[i]==HIGH && i==0){
      estadoBoton[i] = LOW;
    }

    if (lecturaBoton[i]==LOW && i==1 && estadoBoton[i] == LOW){
      modo = 1; // Manual
      indice++;
      Serial.println("Modo: Manual");
      Serial.print("Indice: ");
      displayInfo("Manual");
      Serial.println(indice);
      estadoBoton[i] = HIGH;
      previousTime = millisESP32 ();
      interfaceProg(*(prog00 + indice));
    }
    else if (lecturaBoton[i]==HIGH && i==1){
      estadoBoton[i] = LOW;
    }

    if (lecturaBoton[i]==LOW && i==2 && estadoBoton[i] == LOW){
      modo = 2; // Destello
      indice = 0;
      Serial.println("Modo: Destello");
      displayInfo("Destello");
      estadoBoton[i] = HIGH;
      previousTime = millisESP32 ();
    }
    else if (lecturaBoton[i]==HIGH && i==2){
      estadoBoton[i] = LOW;
    }

    if (lecturaBoton[i]==LOW && i==3 && estadoBoton[i] == LOW){
      modo = 3; // Sicronizado
      indice = 0;
      Serial.println("Modo: Sicronizado");
      displayInfo("Sicronizado");
      estadoBoton[i] = HIGH;
      previousTime = millisESP32 ();
    }
    else if (lecturaBoton[i]==HIGH && i==3){
      estadoBoton[i] = LOW;
    }
   
   
  }
  // Modos de funcionamiento
  switch (modo){
    case 0: //Aislado
        //Actualiza la máquina de estados
        aislado();
        break;
    case 1: //Manual
        aislado();
        break;
    case 2: //Destello
        destello();
        break;
    case 3: //Sincronizado
        sincronizado();
        break;
  }
}

// Función de modo aislado
void aislado(){
  tiempoReal(time0, prog00, longitud);
}
// Función de modo manual
void manual(){
  tiempoReal(time0, prog00, longitud);
}
// Función de destello
void destello(){
  tiempoReal(time1, prog1, longitud1);
}
// Función de sincronización
void sincronizado(){
// Pendiente  
}
// Función de tiempo real
void tiempoReal(unsigned long* time, unsigned long* prog, int longitud){
  //Variable de tiempo actual del timer
  //Revisión de tiempo cumplido
  if ( (millisESP32 () - previousTime >= *(time + indice)) ){
    previousTime = millisESP32 ();
    
    digitalWrite (LED_PIN, !digitalRead (LED_PIN));

    // Incrementar el índice en uno
    indice++;
    
    // Si el índice llega al final del arreglo, reiniciarlo a cero
    if (indice >= longitud) {
        indice = 0;
    }
    else {
        // Ejecución de la Programación
        interfaceProg(*(prog + indice));
    }

    

    Serial.print("Indice: ");
    Serial.println(indice);
  }
}
// Returns the number of milliseconds passed since the ESP32 chip was powered on or reset
unsigned long long millisESP32 () {
  return (unsigned long long) (esp_timer_get_time () / 1000ULL);
}
////*Función de interface de Registros de Desplazamiento*////
void ledWrite(char Reg4, char Reg3, char Reg2, char Reg1){
   shiftOut(pinData, pinClock, LSBFIRST, Reg4);
   shiftOut(pinData, pinClock, LSBFIRST, Reg3);
   shiftOut(pinData, pinClock, LSBFIRST, Reg2);
   shiftOut(pinData, pinClock, LSBFIRST, Reg1);
   digitalWrite(pinLatch, HIGH);
   digitalWrite(pinLatch, LOW);
}

// Inicializa la pantalla OLED
void initOLED() {
  Wire.begin();					// inicializa bus I2C
  oled.begin(SSD1306_SWITCHCAPVCC, 0x3C);	// inicializa pantalla con direccion 0x3C
}

void displayInfo(String modo) {
   /////////////////////OLED////////////////////////////
    oled.clearDisplay();			// limpia pantalla
    oled.setTextColor(WHITE);		// establece color al unico disponible (pantalla monocromo)
    oled.setCursor(0, 0);			// ubica cursor en inicio de coordenadas 0,0
    oled.setTextSize(2);			// establece tamano de texto en 1
    oled.print("Modo: "); 	// escribe en pantalla el texto
    oled.setCursor(0, 16);
    oled.print(modo); 	// escribe en pantalla el texto
    oled.display();			// muestra en pantalla todo lo establecido anteriormente
}