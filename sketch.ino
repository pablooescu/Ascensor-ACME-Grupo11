#include <Servo.h>
#include <IRremote.h>
#include <DHT.h>
#include <Wire.h>
#include <LiquidCrystal_I2C.h>
#include <RTClib.h>

// --- Configuración Motor Paso a Paso (A4988) ---
const int PIN_STEP = 4;
const int PIN_DIR  = 12;
const int PIN_HOMING = A2; 

// --- Pines de Actuadores ---
const int PIN_VALVULA_FRIA = 5;
const int PIN_VALVULA_CALIENTE = 6;
const int PIN_SERVO_PUERTA = 9;
const int PIN_LED_ALERTA = 13;
const int PIN_LED_LUX = 8;

// --- Pines de Sensores ---
const int PIN_IR = 11;
const int PIN_PIR = 7;
const int PIN_DHT_MOTOR = 2;
const int PIN_DHT_BAT = 3;
const int pinLDR = A1;

// --- Constantes de Seguridad y Control ---
const float TEMP_LIMITE = 45.0;
const float HUM_LIMITE = 80.0;
const int UMBRAL_LUX = 50;

// --- Parámetros del PID MANUAL (Variables Globales) ---
float setpointBat = 25.0; 
float salidaPID = 0.0;    

// Constantes de sintonización manual estables para PWM por software
float Kp = 800.0; 
float Ki = 0.1;
float Kd = 150.0; 

// Variables internas del cálculo PID
float errorAnterior = 0.0;
float terminoIntegral = 0.0;

// Ventana de tiempo en milisegundos para el PWM por software (5 segundos)
const unsigned long VENTANA_TIEMPO = 5000; 
unsigned long inicioVentana; 

// --- Variables de Control ---
int plantaActual = 0;
long posicionesPlantas[] = {0, 800, 1600, 2400, 3200}; 
const int PUERTA_CERRADA = 0;
const int PUERTA_ABIERTA = 90;

// Objetos
Servo servoPuerta;
LiquidCrystal_I2C lcd(0x27, 20, 4);
RTC_DS1307 rtc;
DHT dhtMotor(PIN_DHT_MOTOR, DHT22);
DHT dhtBat(PIN_DHT_BAT, DHT22);

unsigned long anteriorMillisLED = 0;
bool estadoLEDAlerta = LOW;
const int intervaloParpadeo = 100;

void setup() {
  pinMode(PIN_STEP, OUTPUT);
  pinMode(PIN_DIR, OUTPUT);
  pinMode(PIN_HOMING, INPUT_PULLUP);
  pinMode(PIN_LED_ALERTA, OUTPUT);
  pinMode(PIN_VALVULA_FRIA, OUTPUT);
  pinMode(PIN_VALVULA_CALIENTE, OUTPUT);
  pinMode(PIN_LED_LUX, OUTPUT);

  servoPuerta.attach(PIN_SERVO_PUERTA);
  servoPuerta.write(PUERTA_CERRADA);

  lcd.init();
  lcd.backlight();
  lcd.clear(); 
  
  realizarHoming();
  
  Serial.begin(9600);
  rtc.begin();
  IrReceiver.begin(PIN_IR, ENABLE_LED_FEEDBACK);
  dhtMotor.begin();
  dhtBat.begin();

  inicioVentana = millis();
  
  mostrarEnLCD("SISTEMA TERMICO", "MODO: MOTOR + SERVO", "Control: PID Manual", "Iniciando...");
  delay(2000);
  lcd.clear(); 
}

void loop() {
  int luxActual = obtenerLux();
  digitalWrite(PIN_LED_LUX, (luxActual < UMBRAL_LUX) ? HIGH : LOW);

  float tB = dhtBat.readTemperature();
  float hB = dhtBat.readHumidity();
  float tM = dhtMotor.readTemperature();

  // 1. CONTROL TÉRMICO PID MANUAL
  controlTermicoPIDManual(tB);

  // 2. SEGURIDAD CRÍTICA
  if (tM >= TEMP_LIMITE || tB >= TEMP_LIMITE || hB >= HUM_LIMITE) {
     bloqueoEmergencia(tM, tB, hB);
     return; 
  } else {
     digitalWrite(PIN_LED_ALERTA, LOW);
  }
  
  // 3. CONTROL REMOTO
  if (IrReceiver.decode()) {
    int nuevaPlanta = mapearBoton(IrReceiver.decodedIRData.command);
    if (nuevaPlanta != plantaActual) viajarAPlanta(nuevaPlanta);
    IrReceiver.resume();
  }

  // 4. ACTUALIZACIÓN PANTALLA
  static unsigned long lastLCD = 0;
  if (millis() - lastLCD > 1000) {
    actualizarLCD_Fisico(luxActual, tM, tB);
    lastLCD = millis();
  }
}

// --- FUNCIÓN: ECUACIÓN PID ALGEBRAICA DIRECTA PROTEGIDA ---
void controlTermicoPIDManual(float tB) {
  if (isnan(tB)) return;

  unsigned long tiempoActual = millis();
  
  // Fijamos un dt constante de 0.1 segundos. 
  // Elimina los desbordamientos causados por el lag de CPU del simulador.
  float dt = 0.1; 

  // 1. CÁLCULO DEL ERROR
  float error = tB - setpointBat;

  // 2. TÉRMINO PROPORCIONAL
  float P = Kp * error;

  // 3. TÉRMINO INTEGRAL CON ACUMULACIÓN LIMITADA (Anti-Windup)
  terminoIntegral += error * dt;
  if (terminoIntegral > 25.0)  terminoIntegral = 25.0;
  if (terminoIntegral < -25.0) terminoIntegral = -25.0;
  float I = Ki * terminoIntegral;

  // 4. TÉRMINO DERIVATIVO SEGURO
  float D = Kd * ((error - errorAnterior) / dt);
  errorAnterior = error;

  // 5. SALIDA TOTAL Y CONTROL DE SATURACIÓN ESTRICTO
  float calculoTotal = P + I + D;
  
  if (isnan(calculoTotal) || isinf(calculoTotal) || calculoTotal > 1000000.0) {
    terminoIntegral = 0.0;
    errorAnterior = 0.0;
    salidaPID = 0.0;
  } else {
    // Forzamos el límite usando flotantes puros independientes del unsigned long
    salidaPID = constrain(calculoTotal, -5000.0, 5000.0); 
  }

  // 6. LÓGICA DE CONTROL PWM POR TIEMPO
  if (tiempoActual - inicioVentana >= VENTANA_TIEMPO) {
    inicioVentana = tiempoActual; 
  }
  unsigned long tiempoEnVentana = tiempoActual - inicioVentana;

  if (salidaPID > 0.0) { 
    // MODO ENFRIAMIENTO
    digitalWrite(PIN_VALVULA_CALIENTE, LOW);
    if (salidaPID > (float)tiempoEnVentana) {
      digitalWrite(PIN_VALVULA_FRIA, HIGH);
    } else {
      digitalWrite(PIN_VALVULA_FRIA, LOW);
    }
  } 
  else if (salidaPID < 0.0) { 
    // MODO CALENTAMIENTO
    digitalWrite(PIN_VALVULA_FRIA, LOW);
    float tiempoCalor = salidaPID * -1.0; // Evita el uso de abs() propenso a fallos de casting
    if (tiempoCalor > (float)tiempoEnVentana) {
      digitalWrite(PIN_VALVULA_CALIENTE, HIGH);
    } else {
      digitalWrite(PIN_VALVULA_CALIENTE, LOW);
    }
  } 
  else {
    digitalWrite(PIN_VALVULA_FRIA, LOW);
    digitalWrite(PIN_VALVULA_CALIENTE, LOW);
  }

  // --- Salida a Monitor Serie ---
  static unsigned long lastSerial = 0;
  if (millis() - lastSerial > 1000) {
    Serial.print("Temp. Bat: "); Serial.print(tB, 1); Serial.print(" C | ");
    Serial.print("Salida PID: "); Serial.print(salidaPID, 2);
    Serial.print(" | V.Fria: "); Serial.print(digitalRead(PIN_VALVULA_FRIA) ? "[ON]" : "[OFF]");
    Serial.print(" | V.Caliente: "); Serial.println(digitalRead(PIN_VALVULA_CALIENTE) ? "[ON]" : "[OFF]");
    lastSerial = millis();
  }
}

void realizarHoming() {
  lcd.clear();
  lcd.print("   CALIBRANDO...   ");
  digitalWrite(PIN_DIR, LOW); 
  while (digitalRead(PIN_HOMING) == HIGH) { 
    digitalWrite(PIN_STEP, HIGH);
    delayMicroseconds(2000); 
    digitalWrite(PIN_STEP, LOW);
    delayMicroseconds(2000);
  }
  plantaActual = 0; 
  lcd.clear();
  lcd.print("  SISTEMA LISTO  ");
  delay(1000);
}

void viajarAPlanta(int destino) {
  servoPuerta.write(PUERTA_CERRADA);
  delay(500);

  long pasosTotales = posicionesPlantas[destino] - posicionesPlantas[plantaActual];
  digitalWrite(PIN_DIR, (pasosTotales > 0) ? HIGH : LOW);
  
  long pasosRestantes = abs(pasosTotales);
  unsigned long lastCheck = 0;

  while (pasosRestantes > 0) {
    if (digitalRead(PIN_PIR)) {
      lcd.setCursor(0, 1); lcd.print(" ESPERANDO DESPEJE  ");
      continue; 
    }

    digitalWrite(PIN_STEP, HIGH);
    delayMicroseconds(800); 
    digitalWrite(PIN_STEP, LOW);
    delayMicroseconds(800);
    pasosRestantes--;

    if (millis() - lastCheck > 1000) {
      if (dhtBat.readTemperature() > TEMP_LIMITE) return;
      lastCheck = millis();
    }
  }

  plantaActual = destino;
  servoPuerta.write(PUERTA_ABIERTA);
  delay(3000); 
  servoPuerta.write(PUERTA_CERRADA);
}

void actualizarLCD_Fisico(int lux, float tM, float tB) {
  DateTime now = rtc.now();
  
  lcd.setCursor(0, 0);
  lcd.print("P:"); lcd.print(plantaActual + 1);
  lcd.print("              "); 
  lcd.setCursor(15, 0);
  if (now.hour() < 10) lcd.print('0');
  lcd.print(now.hour()); lcd.print(":"); 
  if (now.minute() < 10) lcd.print('0');
  lcd.print(now.minute());

  lcd.setCursor(0, 1);
  lcd.print("BAT REAL: "); lcd.print(tB, 1); lcd.print(" C  ");

  lcd.setCursor(0, 2);
  lcd.print("V.F:"); lcd.print(digitalRead(PIN_VALVULA_FRIA) ? "ON " : "OFF");
  lcd.print(" V.C:"); lcd.print(digitalRead(PIN_VALVULA_CALIENTE) ? "ON " : "OFF");
  lcd.print("    ");

  lcd.setCursor(0, 3);
  lcd.print("M:"); lcd.print(tM, 1);
  lcd.print("C P:"); lcd.print(digitalRead(PIN_PIR) ? "S" : "N");
  lcd.print(" L:"); lcd.print(lux);
  lcd.print("   ");
}

// --- RECUPERACIÓN DE FUNCIONES COMPLETAS ---
void bloqueoEmergencia(float tM, float tB, float hB) {
  unsigned long actualMillis = millis();
  if (actualMillis - anteriorMillisLED >= intervaloParpadeo) {
    anteriorMillisLED = actualMillis;
    estadoLEDAlerta = !estadoLEDAlerta;
    digitalWrite(PIN_LED_ALERTA, estadoLEDAlerta);
  }

  lcd.clear();
  lcd.setCursor(0, 0); lcd.print("!!! ALERTA CRITICA !!!");
  lcd.setCursor(0, 1); lcd.print("M:"); lcd.print(tM, 1); lcd.print(" B:"); lcd.print(tB, 1);
  lcd.setCursor(0, 2); lcd.print("HUM BAT: "); lcd.print(hB, 1); lcd.print("%");
  delay(500);
}

int obtenerLux() {
  int analogValue = analogRead(pinLDR);
  if (analogValue == 0) return 0;
  float voltage = analogValue * 5.0 / 1024.0;
  float resistance = 2000 * voltage / (1 - voltage / 5.0);
  return pow(50 * 1e3 * pow(10, 0.7) / resistance, (1 / 0.7));
}

void mostrarEnLCD(String l1, String l2, String l3, String l4) {
  lcd.clear();
  lcd.setCursor(0, 0); lcd.print(l1);
  lcd.setCursor(0, 1); lcd.print(l2);
  lcd.setCursor(0, 2); lcd.print(l3);
  lcd.setCursor(0, 3); lcd.print(l4);
}

int mapearBoton(int cmd) {
  switch (cmd) {
    case 48: return 0; case 24: return 1; case 122: return 2;
    case 16: return 3; case 56: return 4; default: return plantaActual;
  }
}