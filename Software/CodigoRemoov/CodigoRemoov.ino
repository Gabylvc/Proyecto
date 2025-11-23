 // Librerias
#include "I2Cdev.h"
#include "MPU6050.h"
#include "Wire.h"
#include "BluetoothSerial.h"

// Crear objeto Bluetooth 
BluetoothSerial SerialBT;
MPU6050 sensor;

// Valores RAW del acelerometro
int16_t ax, ay, az;
const int fsrPin = 32, batteryPin= 34;    // Pin de FSR y lectura Nivel de batería
const float R1 = 100000.0, R2 = 50000.0;     // Resistencia superior e inferior (ohmios)
const float ADCmax = 4095.0;   // Resolución ADC ESP32 (12 bits)
const float Vref = 3.3;        // Voltaje de referencia ADC
const int ledCargado = 5, ledMedio = 17, ledBajo = 15;  // LEDs de bateria
float angMin = 9999, angMax = -9999, radio = 0.15, arco = 0;
unsigned long lastMeasureTime = 0;  // Tiempo de última medición
const unsigned long interval = 20000;  // 20 segundos

// --- FUNCIÓN PARA LEER Y CLASIFICAR ESTADO DE BATERÍA ---
float readBatteryVoltage() {
  int raw = analogRead(batteryPin);
  float v_adc = (raw / ADCmax) * Vref;            // Voltaje en el pin ADC
  float v_bat = v_adc * (R1 + R2) / R2;           // Corrige por divisor
  return v_bat*1.112;
}

int batteryStatus(float v_bat) {
  if (v_bat >= 3.7) return 3;
  else if (v_bat >= 3.5) return 2;
  else return 0;
}
void updateBatteryLEDs(int estado) {
  digitalWrite(ledCargado, LOW);
  digitalWrite(ledMedio, LOW);
  digitalWrite(ledBajo, LOW);

  if (estado == 3) digitalWrite(ledCargado, HIGH);
  else if (estado == 2) digitalWrite(ledMedio, HIGH);
  else digitalWrite(ledBajo, HIGH);
}

void setup() {
  Serial.begin(9600);   
  Wire.begin();           //Iniciando I2C  
  SerialBT.begin("ESP32");
  sensor.initialize();    //Iniciando el sensor

  if (sensor.testConnection()) Serial.println("Sensor iniciado correctamente");
  else Serial.println("Error al iniciar el sensor");

  // pines
  pinMode(fsrPin, INPUT);
  pinMode(ledCargado, OUTPUT);
  pinMode(ledMedio, OUTPUT);
  pinMode(ledBajo, OUTPUT);
}

void loop() {
  // Leer voltaje de batería
  float vbat = readBatteryVoltage();
  int estadoBat = batteryStatus(vbat);

  // Mostrar por monitor serial
  Serial.print("Batería: ");
  Serial.print(vbat, 2);
  Serial.print(" V  Estado: ");
  Serial.println(estadoBat);
  // ---- Actualizar LEDs según estado ----
  updateBatteryLEDs(estadoBat); 
  
  // Leer las aceleraciones 
  sensor.getAcceleration(&ax,&ay,&az);
  //Calcular los angulos de inclinacion:
  float accel_ang_x=atan(ax/sqrt(pow(ay,2) + pow(az,2)))*(180.0/3.14)+90;
  float accel_ang_y=atan(ay/sqrt(pow(ax,2) + pow(az,2)))*(180.0/3.14)+90;
  // --- Actualizar ángulos extremos ---
  if (accel_ang_x < angMin) angMin = accel_ang_y;
  if (accel_ang_x > angMax) angMax = accel_ang_y;

  int fsrValue = analogRead(fsrPin)/100;

  //Mostrar angulos y fuerza
  Serial.print("Inclinacion en X: ");
  Serial.print(accel_ang_x); 
  Serial.print(" Inclinacion en Y:");
  Serial.println(accel_ang_y);
  Serial.print("FSR: ");
  Serial.println(fsrValue);
  

  // Enviar por Bluetooth
  if (millis() - lastMeasureTime >= interval) {
    float deltaAng = abs(angMax - angMin);         // Diferencia angular (°)
    arco = radio * (deltaAng * PI / 180.0);  // Longitud del arco (m)
    Serial.print("Longitud del arco: ");
    Serial.print(arco, 3);
    Serial.println(" m");

    // Reiniciar medición
    angMin = 9999;
    angMax = -9999;
    lastMeasureTime = millis();
  }
  SerialBT.print(fsrValue); 
  SerialBT.print("?");
  SerialBT.print(accel_ang_y);
  SerialBT.print("?");
  SerialBT.print(estadoBat);
  SerialBT.print("?");
  SerialBT.println(arco, 3);
  
  if (SerialBT.available()) {
    char dato = SerialBT.read();  // Leer hasta salto de línea
    if (dato == '1') { 
      radio = 0.54; 
    } else if (dato == '2') {
      radio = 0.56;
    } else if (dato == '3') {
      radio = 0.58;
    } else if (dato == '4') {
      radio = 0.6;
    }
  }
  delay(500);
}