#include <Arduino.h>
#include "Wire.h"
#include <ESP32SPISlave.h>

#define I2C_DEV_ADDR 0x55   // Dirección del dispositivo I2C
#define potPin 27           // Pin del potenciómetro (ADC)
#define ledRojo 32          // Pin del LED rojo
#define ledAma 17           // Pin del LED amarillo
#define ledVerde 16          // Pin del LED verde
#define Size 1

ESP32SPISlave slave;

static constexpr uint32_t BUFFER_SIZE{10};

uint8_t spi_slave_tx_buf[BUFFER_SIZE];
uint8_t spi_slave_rx_buf[BUFFER_SIZE];
char cmd;

uint8_t i = 0;
int potValue = 0;
int tiempo = 0;


void onRequest(); //CC:En mi código yo no tenía esto xd
void onReceive(int len);

void setup() {
  Serial.begin(115200);
  Serial.setDebugOutput(true); 

  // Configura pines de los LEDs
  pinMode(ledRojo, OUTPUT);
  pinMode(ledAma, OUTPUT);
  pinMode(ledVerde, OUTPUT);
  digitalWrite(ledRojo, LOW);
  digitalWrite(ledAma, LOW);
  digitalWrite(ledVerde, LOW);
  // Inicia comunicación I2C
   Serial.println("Empezamos");
  Wire.begin((uint8_t)I2C_DEV_ADDR);    
  Wire.onReceive(onReceive);
  Wire.onRequest(onRequest);
  
  // HSPI = cs: 15, clk: 14, mosi: 13, miso: 12
  // VSPI = CS: 5, CLK:18, MOSI:23, MISO:19
  slave.setDataMode(SPI_MODE0);
  slave.setQueueSize(Size);
  slave.begin(VSPI);
  // Inicializa el buffer de recepción en cero
  memset(spi_slave_tx_buf, 0, BUFFER_SIZE);
  memset(spi_slave_rx_buf, 0, BUFFER_SIZE);

}

void loop() {
  //potValue = analogRead(potPin);  // Leer el valor del potenciómetro
    //const int BUFFER_SIZE = 3;
    //uint8_t spi_slave_rx_buf[BUFFER_SIZE];  // Buffer para datos recibidos por SPI

    // Esperar hasta que haya datos disponibles
  slave.wait(spi_slave_rx_buf, spi_slave_tx_buf, BUFFER_SIZE);

    if (slave.available()) {

      for (int i = 0; i < BUFFER_SIZE; i++) {  
      Serial.println("Digito recibido");
      Serial.println(spi_slave_rx_buf[i]);
      Serial.print(" ");
    }
  tiempo = atoi((const char*)&spi_slave_rx_buf[1]); // ⁠ posicion_coma + 1 ⁠ apunta al carácter después de la coma
  Serial.print("Tiempo: ");
  Serial.println(tiempo);
  slave.pop();

  }
  if(spi_slave_rx_buf[0] == 49){


    Serial.println("LED ROJA");
    digitalWrite(ledRojo,  1);
    delay(tiempo);
    digitalWrite(ledRojo, 0);
    }if(spi_slave_rx_buf[0] == 50){
      Serial.println("LED AMARILLA");
      digitalWrite(ledAma, 1);
      delay(tiempo);
      digitalWrite(ledAma, 0);
    }if(spi_slave_rx_buf[0] == 51){
      Serial.println("LED VERDE");
      digitalWrite(ledVerde, 1);
      delay(tiempo);
      digitalWrite(ledVerde, 0);
    }



}

void onRequest() {
  potValue = analogRead(potPin);  // Leer el valor del potenciómetro

  Wire.write((uint8_t)(potValue >> 8));   // Envía el byte alto del valor del potenciómetro
  Wire.write((uint8_t)(potValue & 0xFF)); // Envía el byte bajo del valor
  Serial.printf("Enviando valor del potenciómetro en I2C: %d\n", potValue);  // Imprime el valor completo en el monitor serial
}

void onReceive(int len) {
  Serial.printf("Datos recibidos [%d]: ", len);
  
  while (Wire.available()) {
    uint8_t incomingByte = Wire.read();  // Lee el byte entrante
    Serial.printf("Comando recibido: %d\n", incomingByte);  // Mensaje de depuración

    // Control de los LEDs dependiendo del valor recibido
    /*
    switch (incomingByte) {
      case 1:
        digitalWrite(ledRojo, HIGH);
        digitalWrite(ledAma, LOW);
        digitalWrite(ledVerde, LOW);
        Serial.println(" -> LED rojo encendido");
        break;
      case 2:
        digitalWrite(ledRojo, LOW);
        digitalWrite(ledAma, HIGH);
        digitalWrite(ledVerde, LOW);
        Serial.println(" -> LED amarillo encendido");
        break;
      case 3:
        digitalWrite(ledRojo, LOW);
        digitalWrite(ledAma, LOW);
        digitalWrite(ledVerde, HIGH);
        Serial.println(" -> LED verde encendido");
        break;
      default:
        Serial.println(" -> Comando no reconocido");
        break;
        */
    }
    Serial.println();
  }
  
