/* PROTOCOLO I2C */

const uint8_t IMUAddress = 0x68;      // AD0 es el valor logico bajo en la PCB
const uint16_t I2C_TIMEOUT = 1000;    // Usado para comprobar errores en la comunicación I2C. 

uint8_t i2cWrite(uint8_t registerAddress, uint8_t data, bool sendStop) {
  return i2cWrite(registerAddress, &data, 1, sendStop); // Devuelve un 0 si es correcto.
}

uint8_t i2cWrite(uint8_t registerAddress, uint8_t *data, uint8_t length, bool sendStop) {
  Wire.beginTransmission(IMUAddress);
  Wire.write(registerAddress);
  Wire.write(data, length);
  uint8_t rcode = Wire.endTransmission(sendStop); // Devuelve un 0 si es correcto.
  if (rcode) {
    Serial.print(F("i2cWrite failed: "));
    Serial.println(rcode);
  }
  return rcode;
}

uint8_t i2cRead(uint8_t registerAddress, uint8_t *data, uint8_t nbytes) {
  uint32_t timeOutTimer;
  Wire.beginTransmission(IMUAddress);
  Wire.write(registerAddress);
  uint8_t rcode = Wire.endTransmission(false); // No deja el bus.
  if (rcode) {
    Serial.print(F("i2cRead failed: "));
    Serial.println(rcode);
    return rcode; 
  }
  Wire.requestFrom(IMUAddress, nbytes, (uint8_t)true); // Envia un inicio repetido y luego suelta el bus despues de leer.
  for (uint8_t i = 0; i < nbytes; i++) {
    if (Wire.available())
      data[i] = Wire.read();
    else {
      timeOutTimer = micros();
      while (((micros() - timeOutTimer) < I2C_TIMEOUT) && !Wire.available());
      if (Wire.available())
        data[i] = Wire.read();
      else {
        Serial.println(F("i2cRead timeout"));
        return 5; // Este valor de error no esta tomado todavia por endTransmission.
      }
    }
  }
  return 0; // Exito.
}
