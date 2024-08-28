# TOYBOX
El proyecto consistirá en un puñado de sensores mostrando los datos por una pantalla TFT de 1.8" pulgadas y por el puerto serie. Para llevar a cabo la implementación se necesitarán los siguientes componentes:

- Una protoboard.
- Jumpers.
- Arduino NANO.
- Sensor piroeléctrico (PIR): HC-SR501
- Sensor ultrasónico: HC-SR04.
- Fotorresistencia LDR: GL5528.
- Termistor NTC 10k Ω.
- Sensor de temperatura y humedad: HDT22.
- Sensor de aceleración: GY-521.
- Pantalla de cristal líquido LCD TFT: ST7735.
- 2 Resistencias de 10k Ω.

El comportamiento del proyecto será el siguiente:  

1) Iniciar los modulos
2) Imprimir una portada por pantalla
3) Tomar los datos de los sensores
4) Mostrar los datos por el puerto Serie y la pantalla TFT.

# Pasar de lecturas analógicas y digitales a medidas físicas
Todos los sensores una vez realizada la medición no envían directamente el valor en su magnitud física correspondiente al microcontrolador, 
sino que lo que devuelven son tensiones o corrientes la cuales hay que interpretar.

1) Sensor piroeléctrico (PIR): HC-SR501

  Este sensor posee una salida digital con dos valores: 1 si detecta un cambio de movimiento y 0 si no hay cambios en el entorno.

2) Sensor ultrasónico: HC-SR04

  En este proyecto se ha utilizado la librería NewPing.h la cual incluye una instrucción que proporciona directamente el valor de la distancia en centímetros.
 
  Este sensor devuelve un valor digital correspondiente al tiempo del pulso ECHO. 𝐷𝑖𝑠𝑡𝑎𝑛𝑐𝑖𝑎(𝑚) = (𝐷8 ∗ 340) ⁄ 2

3) Fotorresistencia LDR: GL5528
   
Este sensor está configurado como un divisor de tensión y tiene una salida analógica.

Conversión analógica a voltaje: 𝑉𝑜𝑢𝑡 = 𝐴1 ∗ (𝑉𝑖𝑛/1023)

Conversión voltaje a resistencia: 𝑅𝐿𝐷𝑅 = 𝑅 ∗ (𝑉𝑖𝑛−𝑉𝑜𝑢𝑡) / 𝑉𝑜𝑢𝑡

Conversión resistencia a lumen: 𝑃ℎ𝑦𝑠 = 500 / (𝑅𝐿𝐷𝑅/1000) 

𝑉𝑖𝑛 = 5𝑉; 𝑅 = 10𝑘 Ω ; Lux a kΩ = 500 / R

4) Termistor NTC 10k Ω
   
Este sensor está configurado como un divisor de tensión y tiene una salida analógica.
Los termistores NTC o de coeficiente de temperatura negativo disminuyen su resistencia a medida que su temperatura aumenta.
Para obtener la temperatura a partir de los datos analógicos necesitamos implementar la ecuación Steinhart-Hart la cual es un modelo de la resistencia
de un semiconductor a diferentes temperaturas, siendo esta la siguiente:

1/𝑇 = 𝐴 + 𝐵𝑙𝑛𝑅 + 𝐶(𝑅)^3

T = Temperatura en kelvin; 
R = Resistencia en ohmios; 
A, B, C = Coeficientes; 

La resistencia del termistor se calcula resolviendo el divisor resistivo: 𝑅 = 𝑅_10𝑘 ∗ ((1023 / 𝑉𝑎𝑙𝑜𝑟𝐴𝑛𝑎𝑙𝑜𝑔𝑖𝑐𝑜) − 1)

5) Sensor de temperatura y humedad: HDT22
   
Este sensor dispone de una salida digital que proporciona los datos de manera directa a través de una librería.

6) Sensor de aceleración: GY-521

Ese sensor se comunica mediante el protocolo I2c y proporciona 6 datos brutos, 3 para el giro y 3 para la aceleración, además de la temperatura.
Los datos necesitan ser filtrados mediante un filtro complementario el cual es una simplificación del también implementado filtro Kalman.
