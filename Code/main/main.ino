/*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
  ~Proyecto Electrónica Analógica (TOYBOX)~
  ~Hecho por:   Víctor Caro Pastor        ~
  ~Versión:     18.0                       ~
  ~Fecha:       26/11/2021                ~
  ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ */
  
/* Librerias */  
  #include <Adafruit_GFX.h>    
  #include <Adafruit_ST7735.h> 
  #include <SPI.h>
  #include <math.h> 
  #include <DHT.h>
  #include <NewPing.h> 
  #include <Wire.h>
  #include "Kalman.h"
  
  
// Constantes
  const int DELAY=1500;  // Tiempo de muestreo.
  
// Pantalla LCD 1.8" TFT (128x160)  
  const int TFT_CS = 10;
  const int TFT_RST = -1; // Conectado al RST de Arduino.
  const int TFT_DC = 9;
  Adafruit_ST7735 tft = Adafruit_ST7735(TFT_CS, TFT_DC, TFT_RST);
  
// Sensor PIR
  const int Pin_PIR = 3;   
  int Valor_PIR;                             
        
// Sensor DHT
  const int Pin_DHT=2;                            
  DHT dht(Pin_DHT, DHT22); // Objeto tipo dht modelo DHT22.                       
  float Humedad;
  float Temperatura_DHT;

// Sensor Termistor 10k
  const int Pin_TERM=0;
  const int Resistencia_termistor=9740; // Resistencia del divisor de tension (Medida con el polimetro).
  float Temperatura; 
  uint32_t Resistencia;                        
  
// Sensor Ultrasonido HC-SR04
  int Distancia;
  const int TRIGGER_PIN=5;                          
  const int ECHO_PIN=4;                          
  const int MAX_DISTANCE=450;
  NewPing sonar(TRIGGER_PIN, ECHO_PIN, MAX_DISTANCE); // Objeto tipo sonar.   
  
// Sensor LDR
  const int Pin_LDR=1;                            
  const int VIN=5;   
  const int R=10000; 
  int Val_ANALOGICO;                              
  int Val_LUMEN;                                    
 
//Sensor Giroscopio GY-521 
#define RESTRICT_PITCH // Comentar para restringir el roll a ±90º.

Kalman kalmanX; // Creamos las variables Kalman.
Kalman kalmanY;

/* Datos IMU */
double accX, accY, accZ;
double gyroX, gyroY, gyroZ;
int16_t tempRaw;

double gyroXangle, gyroYangle; // Ángulo calculado usando el giroscopio.
double compAngleX, compAngleY; // Ángulo calculado usando el filtro complementario.
double kalAngleX, kalAngleY; // Ángulo calculado usando el filtro Kalman.

uint32_t timer;
uint8_t i2cData[14]; // Buffer para los datos I2C.

/*======================================================================================*/

void setup() 
{
  Serial.begin(115200);
  pinMode(Pin_PIR, INPUT);                        
  dht.begin(); // Iniciamos el objeto dht.
  //Serial.println("OBTENIENDO LAS MEDICIONES, POR FAVOR ESPERE:");
  
  // Use this initializer if using a 1.8" TFT screen:
  tft.initR(INITR_BLACKTAB);      // Init ST7735S chip, black tab

  
  Wire.begin();
  TWBR = ((F_CPU / 400000L) - 16) / 2; // Frecuencia del I2C a 400 kHz.

  i2cData[0] = 7; // Set the sample rate to 1000Hz - 8kHz/(7+1) = 1000Hz
  i2cData[1] = 0x00; // Disable FSYNC and set 260 Hz Acc filtering, 256 Hz Gyro filtering, 8 KHz sampling
  i2cData[2] = 0x00; // Set Gyro Full Scale Range to ±250deg/s
  i2cData[3] = 0x00; // Set Accelerometer Full Scale Range to ±2g
  while (i2cWrite(0x19, i2cData, 4, false)); // Write to all four registers at once
  while (i2cWrite(0x6B, 0x01, true)); // PLL with X axis gyroscope reference and disable sleep mode

  while (i2cRead(0x75, i2cData, 1));
  // Lee el registro "Quien soy"
  if (i2cData[0] != 0x68) {     
    Serial.print(F("Error reading sensor"));
    while (1);
  }

  delay(100); // Esperamos a que se estabilice

  /* Indicamos el ángulo inicial para Kalman y el giroscopio */ 
  while (i2cRead(0x3B, i2cData, 6));
  accX = (i2cData[0] << 8) | i2cData[1];
  accY = (i2cData[2] << 8) | i2cData[3];
  accZ = (i2cData[4] << 8) | i2cData[5];

// La salida de atan2 esta entre -π y π radianes. Luego se pasa a grados.
#ifdef RESTRICT_PITCH 
  double roll  = atan2(accY, accZ) * RAD_TO_DEG;
  double pitch = atan(-accX / sqrt(accY * accY + accZ * accZ)) * RAD_TO_DEG;
#else 
  double roll  = atan(accY / sqrt(accX * accX + accZ * accZ)) * RAD_TO_DEG;
  double pitch = atan2(-accX, accZ) * RAD_TO_DEG;
#endif

  kalmanX.setAngle(roll);  // Ángulo inicial.
  kalmanY.setAngle(pitch);
  gyroXangle = roll;
  gyroYangle = pitch;
  compAngleX = roll;
  compAngleY = pitch;

  timer = micros();
}

/*======================================================================================*/

void loop() 
{ 
  /* Sensor Giroscopio */
  double temperature = (double)tempRaw / 340.0 + 36.53;

  // Actualizamos todos los valores
  while (i2cRead(0x3B, i2cData, 14));
  accX = ((i2cData[0] << 8) | i2cData[1]);
  accY = ((i2cData[2] << 8) | i2cData[3]);
  accZ = ((i2cData[4] << 8) | i2cData[5]);
  tempRaw = (i2cData[6] << 8) | i2cData[7];
  gyroX = (i2cData[8] << 8) | i2cData[9];
  gyroY = (i2cData[10] << 8) | i2cData[11];
  gyroZ = (i2cData[12] << 8) | i2cData[13];

  double dt = (double)(micros() - timer) / 1000000; // Calculamos el delta de tiempo.
  timer = micros();

// La salida de atan2 esta entre -π y π radianes. Luego se pasa a grados.  
#ifdef RESTRICT_PITCH 
  double roll  = atan2(accY, accZ) * RAD_TO_DEG;
  double pitch = atan(-accX / sqrt(accY * accY + accZ * accZ)) * RAD_TO_DEG;
#else 
  double roll  = atan(accY / sqrt(accX * accX + accZ * accZ)) * RAD_TO_DEG;
  double pitch = atan2(-accX, accZ) * RAD_TO_DEG;
#endif
  double gyroXrate = gyroX / 131.0; // Convertimos a º/s.
  double gyroYrate = gyroY / 131.0; // Convertimos a º/s.

#ifdef RESTRICT_PITCH
  // Esto fija el problema de transición cuando el acelerómetro salta entre -180 y 180 grados.
  if ((roll < -90 && kalAngleX > 90) || (roll > 90 && kalAngleX < -90)) {
    kalmanX.setAngle(roll);
    compAngleX = roll;
    kalAngleX = roll;
    gyroXangle = roll;
  } else
    kalAngleX = kalmanX.getAngle(roll, gyroXrate, dt); // Calculo el angulo usando el filtro Kalman.

  if (abs(kalAngleX) > 90)
    gyroYrate = -gyroYrate; // Ratio invertido para que encaje en las medidas restringidas del acelerómetro.
  kalAngleY = kalmanY.getAngle(pitch, gyroYrate, dt);
#else
  // Esto fija el problema de transición cuando el acelerómetro salta entre -180 y 180 grados.
  if ((pitch < -90 && kalAngleY > 90) || (pitch > 90 && kalAngleY < -90)) {
    kalmanY.setAngle(pitch);
    compAngleY = pitch;
    kalAngleY = pitch;
    gyroYangle = pitch;
  } else
    kalAngleY = kalmanY.getAngle(pitch, gyroYrate, dt); // Calculo el angulo usando el filtro Kalman.

  if (abs(kalAngleY) > 90)
    gyroXrate = -gyroXrate; // Ratio invertido para que encaje en las medidas restringidas del acelerómetro.
  kalAngleX = kalmanX.getAngle(roll, gyroXrate, dt); // Calculo el ángulo usando el filtro Kalman.
#endif

  gyroXangle += gyroXrate * dt; // Cálculo del angulo de giroscopio sin filtro.
  gyroYangle += gyroYrate * dt;
 
  compAngleX = 0.93 * (compAngleX + gyroXrate * dt) + 0.07 * roll; // Cálculo del ángulo usando el filtro complementario.
  compAngleY = 0.93 * (compAngleY + gyroYrate * dt) + 0.07 * pitch;

  // Resetea el angulo del giroscopio si le damos mucho meneo
  if (gyroXangle < -180 || gyroXangle > 180)
    gyroXangle = kalAngleX;
  if (gyroYangle < -180 || gyroYangle > 180)
    gyroYangle = kalAngleY;

  /* Sensor PIR */
  Valor_PIR = digitalRead(Pin_PIR);
  /* Sensor DHT */
  Humedad = dht.readHumidity();        
  Temperatura_DHT = dht.readTemperature();        
  /* Sensor termistor 10k */
  Resistencia = Obtener_Resistencia_Termistor(analogRead(Pin_TERM)); 
  Temperatura = Obtener_Temperatura_Termistor(Resistencia);
  /* Sensor Ultrasonido HC-SR04 */
  Distancia=sonar.ping_cm();
  /* Sensor LDR */
  Val_ANALOGICO = analogRead(Pin_LDR);
  Val_LUMEN=Conversion_Analogica_Lumen(Val_ANALOGICO);

  /* Presento los datos por el puerto serie. */
  #if 1 //Poner a 1 para visualizar por el monitor serie
  #if 1 
  Serial.print("Sensor PIR"); Serial.print(" \t");
  Serial.print("Humedad"); Serial.print(" \t");
  Serial.print("Temperatura DHT22"); Serial.print(" \t");
  Serial.print("Temperatura Termistor"); Serial.print(" \t");
  Serial.print("Temperatura Giroscopio"); ; Serial.print(" \t");
  Serial.print("Distancia"); Serial.print(" \t");
  Serial.println("Luz ambiente");
  Serial.print(Valor_PIR); Serial.print(" \t\t");
  Serial.print(Humedad); Serial.print(" %\t\t");
  Serial.print(Temperatura_DHT); Serial.print(" *C\t\t");
  Serial.print(Temperatura); Serial.print(" *C\t\t");
  Serial.print(temperature); Serial.print(" *C\t\t");
  Serial.print(Distancia); Serial.print(" cm\t\t");
  Serial.print(Val_LUMEN); Serial.println(" lm");
  #endif
  
  #if 0
  Serial.print("A_X_RAW"); Serial.print("\t"); Serial.print(accX);
  Serial.print("  G_X_RAW"); Serial.print("\t"); Serial.println(gyroX); 
  Serial.print("A_Y_RAW"); Serial.print("\t"); Serial.print(accY);
  Serial.print("  G_Y_RAW"); Serial.print("\t"); Serial.println(gyroY);
  Serial.print("A_Z_RAW"); Serial.print("\t"); Serial.print(accZ);
  Serial.print("  G_Z_RAW"); Serial.print("\t"); Serial.println(gyroZ);
  #endif
  
  #if 1
  Serial.print("Roll"); Serial.print(" \t\t");
  Serial.print("Angulo X (Gyr)"); Serial.print(" \t");
  Serial.print("Angulo X (Comp)"); Serial.print(" \t");
  Serial.print("Angulo X (Kaplan)"); Serial.println(" \t");
  Serial.print(roll); Serial.print("\t\t");
  Serial.print(gyroXangle); Serial.print("\t\t");
  Serial.print(compAngleX); Serial.print("\t\t\t");
  Serial.print(kalAngleX); Serial.println("\t\t\t");
  Serial.print("Pitch"); Serial.print(" \t\t");
  Serial.print("Angulo Y (Gyr)"); Serial.print(" \t");
  Serial.print("Angulo Y (Comp)"); Serial.print(" \t");
  Serial.print("Angulo Y (Kaplan)"); Serial.println(" \t");
  Serial.print(pitch); Serial.print("\t\t");
  Serial.print(gyroYangle); Serial.print("\t\t");
  Serial.print(compAngleY); Serial.print("\t\t\t");
  Serial.print(kalAngleY); Serial.print("\t\t\t");
  Serial.print("\r\n");
  Serial.print("\r\n");
  Serial.print("\r\n");
  #endif
  #endif
  /* Imprimir por pantalla */  
  ImprimirSIMPLES();
  ImprimirGYRO();
  delay(DELAY);
}


/* FUNCIONES */
// Esta función obtiene la resistencia del termistor resolviendo el divisor resistivo.
int32_t Obtener_Resistencia_Termistor(uint16_t adcval){
    return (Resistencia_termistor * ((1023.0 / adcval) - 1));
  }
// Esta función obtiene la temperatura en grados centígrados a partir de la resistencia actual del termistor.
float Obtener_Temperatura_Termistor(int32_t resistance){
  float temp; 
    temp = log(resistance);
    temp = 1 / (0.001129148 + (0.000234125 * temp) + (0.0000000876741 * temp * temp * temp));
    return temp - 273.15;   
  }
// Esta función convierte el valor analógico a un valor en lumens.
int Conversion_Analogica_Lumen(int raw){ 
    float Vout = float(raw) * (VIN / float(1023));   
    float RLDR = (R * (VIN - Vout))/Vout;      
    int phys=500/(RLDR/1000);                     
    return phys;
  } 
// Esta función presenta los datos simples por la pantalla TFT.
void ImprimirSIMPLES() {
  tft.setTextWrap(false);   // El texto que no entre pasa a la siguiente línea.
  tft.setRotation(0);       // Establece la pantalla en vertical.
  tft.fillScreen(ST77XX_BLACK);
  tft.setCursor(0, 0);tft.setTextSize(1); tft.setTextColor(ST77XX_CYAN);
  tft.println("  MEDICIONES TOYBOX"); tft.setTextColor(ST77XX_WHITE); tft.println();
  tft.print("Humedad:     "); tft.setTextColor(ST77XX_BLUE); tft.print(Humedad); tft.setTextColor(ST77XX_WHITE); tft.println(" %");
  tft.print("Temp DHT:    "); tft.setTextColor(ST77XX_RED); tft.print(Temperatura_DHT); tft.setTextColor(ST77XX_WHITE); tft.println(" *C");
  tft.print("Temp TERM:   "); tft.setTextColor(ST77XX_RED); tft.print(Temperatura); tft.setTextColor(ST77XX_WHITE); tft.println(" *C");
  tft.print("Distancia:   "); tft.setTextColor(ST77XX_MAGENTA); tft.print(Distancia); tft.setTextColor(ST77XX_WHITE); tft.setCursor(113,50); tft.println("cm");
  tft.print("Iluminacion: "); tft.setTextColor(ST77XX_YELLOW); tft.print(Val_LUMEN); tft.setTextColor(ST77XX_WHITE); tft.setCursor(113,58); tft.println("lm");
  tft.print("Sensor PIR:  "); tft.println(Valor_PIR);
}
// Esta función presenta los datos del giroscopio por la pantalla TFT.
void ImprimirGYRO() {
  tft.println();
  tft.println("             Angulo X");
  tft.print("Giroscopio:  "); tft.setTextColor(ST77XX_ORANGE); tft.println(gyroXangle); tft.setTextColor(ST77XX_WHITE);
  tft.print("Compl:       "); tft.setTextColor(ST77XX_ORANGE); tft.println(compAngleX); tft.setTextColor(ST77XX_WHITE);
  tft.print("Kaplan:      "); tft.setTextColor(ST77XX_ORANGE); tft.println(kalAngleX); tft.setTextColor(ST77XX_WHITE);
  tft.println("             Angulo Y");
  tft.print("Giroscopio:  "); tft.setTextColor(ST77XX_ORANGE); tft.println(gyroYangle); tft.setTextColor(ST77XX_WHITE);
  tft.print("Compl:       "); tft.setTextColor(ST77XX_ORANGE); tft.println(compAngleY); tft.setTextColor(ST77XX_WHITE);
  tft.print("Kaplan:      "); tft.setTextColor(ST77XX_ORANGE); tft.println(kalAngleY); tft.setTextColor(ST77XX_WHITE);
}
