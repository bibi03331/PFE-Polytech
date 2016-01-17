#define BUFFER_LENGTH 32

#define ACCEL_XOUT_H     0x3B
#define MPU9250_REF_ADDRESS  0x69

uint8_t rxBuffer[BUFFER_LENGTH];
uint8_t rxBufferIndex = 0;
uint8_t rxBufferLength = 0;

uint8_t txBuffer[BUFFER_LENGTH+1];
uint8_t txBufferIndex = 0;
uint8_t txBufferLength = 0;
uint8_t transmitting = 0;

uint8_t slave_mode = 0;
uint8_t irqcount = 0;

int16_t accelCount[3] = {0,0,0};

int x,y,z; //triple axis data

void setup() {
  
  beginI2c();

  beginI2cTransmission(0x1E);
  writeI2c(0x02);
  writeI2c(0x00);
  endI2cTransmission(false);

  x = 0;
  y = 0;
  z = 0;
}

void loop() {

  /*
  beginI2cTransmission(0x1E);
  writeI2c(0x03);
  endI2cTransmission(false);

  i2cRequestFrom(0x1E, 6, false);
  if ( i2cDataAvailable() ) {
    x = i2cRead() << 8; // X msb
    x |= i2cRead(); // X lsb
    z = i2cRead() << 8; // Z msb
    z |= i2cRead(); // Z lsb
    y = i2cRead() << 8; // Y msb
    y |= i2cRead(); // Y lsb
  }
  */

  readAccelData(accelCount, MPU9250_REF_ADDRESS);

  //Print out values of each axis
  Serial.print("x: ");
  Serial.print(accelCount[0]);
  Serial.print("  y: ");
  Serial.print(accelCount[1]);
  Serial.print("  z: ");
  Serial.println(accelCount[2]);
  
  
  delay(250);
  
}

void readAccelData(int16_t * destination, byte address)
{
  uint8_t rawData[6];  // x/y/z accel register data stored here
  
  readBytes(address, ACCEL_XOUT_H, 6, &rawData[0]);  // Read the six raw data registers into data array

  destination[0] = rawData[0] << 8; // X msb
  destination[0] |= rawData[1]; // X lsb
  destination[1] = rawData[2] << 8; // Z msb
  destination[1] |= rawData[3]; // Z lsb
  destination[2] = rawData[4] << 8; // Y msb
  destination[2] |= rawData[5]; // Y lsb
}

void readBytes(uint8_t address, uint8_t subAddress, uint8_t count, uint8_t * dest)
{
  beginI2cTransmission(address);   // Initialize the Tx buffer
  writeI2c(subAddress);            // Put slave register address in Tx buffer
  endI2cTransmission(false);       // Send the Tx buffer, but send a restart to keep connection alive
  uint8_t i = 0;
  i2cRequestFrom(address, count, false);  // Read bytes from slave register address
  while ( i2cDataAvailable() ) {
    dest[i++] = i2cRead(); // Put read results in the Rx buffer
  }         
}

void beginI2c(void) {

  SIM_SCGC4 |= SIM_SCGC4_I2C0;
  I2C0_C1 = 0;
  PORTB_PCR3 = PORT_PCR_MUX(2)|PORT_PCR_ODE|PORT_PCR_SRE|PORT_PCR_DSE;
  PORTB_PCR2 = PORT_PCR_MUX(2)|PORT_PCR_ODE|PORT_PCR_SRE|PORT_PCR_DSE;
  I2C0_F = 0x28; // F_CPU = 72 000 000 | F_BUS = 36 000 000 | F_PLL = 72 000 000
  I2C0_C2 = I2C_C2_HDRS;
  I2C0_C1 = I2C_C1_IICEN;
}

uint8_t i2c_status(void)
{
  static uint32_t p = 0xFFFF;
  uint32_t s = I2C0_S;
  
  if (s != p) {
    p = s;
  }
  
  return s;
}

void i2c_wait(void)
{
  while (1) {
    if ((i2c_status() & I2C_S_IICIF)) break;
  }
  
  I2C0_S = I2C_S_IICIF;
}

void beginI2cTransmission(uint8_t address)
{
  txBuffer[0] = (address << 1);
  transmitting = 1;
  txBufferLength = 1;
}

uint8_t writeI2c(uint8_t data)
{
  if (transmitting || slave_mode) {
    if (txBufferLength >= BUFFER_LENGTH+1) return 0;
    txBuffer[txBufferLength++] = data;
    return 1;
  }
  return 0;
}

uint8_t endI2cTransmission(uint8_t sendStop)
{
  uint8_t i, status, ret=0;

  // clear the status flags
  I2C0_S = I2C_S_IICIF | I2C_S_ARBL;
  // now take control of the bus...
  if (I2C0_C1 & I2C_C1_MST) {
    // we are already the bus master, so send a repeated start
    I2C0_C1 = I2C_C1_IICEN | I2C_C1_MST | I2C_C1_RSTA | I2C_C1_TX;
  } else {
    // we are not currently the bus master, so wait for bus ready
    uint32_t wait_begin = millis();
    while (i2c_status() & I2C_S_BUSY) {
      
      if (millis() - wait_begin > 15) {
        // bus stuck busy too long
        I2C0_C1 = 0;
        I2C0_C1 = I2C_C1_IICEN;
        return 4;
      }
    }
    // become the bus master in transmit mode (send start)
    slave_mode = 0;
    I2C0_C1 = I2C_C1_IICEN | I2C_C1_MST | I2C_C1_TX;
  }
  // wait until start condition establishes control of the bus
  while (1) {
    status = i2c_status();
    if ((status & I2C_S_BUSY)) break;
  }
  // transmit the address and data
  for (i=0; i < txBufferLength; i++) {
    I2C0_D = txBuffer[i];
    while (1) {
      status = i2c_status();
      if ((status & I2C_S_IICIF)) break;
      if (!(status & I2C_S_BUSY)) break;
    }
    I2C0_S = I2C_S_IICIF;
    status = i2c_status();
    if ((status & I2C_S_ARBL)) {
      // we lost bus arbitration to another master
      I2C0_C1 = I2C_C1_IICEN;
      ret = 4; // 4:other error
      break;
    }
    if (!(status & I2C_S_BUSY)) {
      // suddenly lost control of the bus!
      I2C0_C1 = I2C_C1_IICEN;
      ret = 4; // 4:other error
      break;
    }
    if (status & I2C_S_RXAK) {
      // the slave device did not acknowledge
      if (i == 0) {
        ret = 2; // 2:received NACK on transmit of address
      } else {
        ret = 3; // 3:received NACK on transmit of data 
      }
      sendStop = 1;
      break;
    }
  }
  if (sendStop) {
    // send the stop condition
    I2C0_C1 = I2C_C1_IICEN;
  }
  transmitting = 0;
  
  return ret;
}

uint8_t i2cRequestFrom(uint8_t address, uint8_t lengthData, uint8_t sendStop)
{
  uint8_t tmp __attribute__((unused));
  uint8_t status, count=0;

  rxBufferIndex = 0;
  rxBufferLength = 0;
  // clear the status flags
  I2C0_S = I2C_S_IICIF | I2C_S_ARBL;
  // now take control of the bus...
  if (I2C0_C1 & I2C_C1_MST) {
    // we are already the bus master, so send a repeated start
    I2C0_C1 = I2C_C1_IICEN | I2C_C1_MST | I2C_C1_RSTA | I2C_C1_TX;
  } else {
    // we are not currently the bus master, so wait for bus ready
    while (i2c_status() & I2C_S_BUSY) ;
    // become the bus master in transmit mode (send start)
    slave_mode = 0;
    I2C0_C1 = I2C_C1_IICEN | I2C_C1_MST | I2C_C1_TX;
  }
  // send the address
  I2C0_D = (address << 1) | 1;
  i2c_wait();
  status = i2c_status();
  if ((status & I2C_S_RXAK) || (status & I2C_S_ARBL)) {
    // the slave device did not acknowledge
    // or we lost bus arbitration to another master
    I2C0_C1 = I2C_C1_IICEN;
    return 0;
  }
  if (lengthData == 0) {
    I2C0_C1 = I2C_C1_IICEN | (sendStop ? 0 : I2C_C1_MST);
    return 0;
  } else if (lengthData == 1) {
    I2C0_C1 = I2C_C1_IICEN | I2C_C1_MST | I2C_C1_TXAK;
  } else {
    I2C0_C1 = I2C_C1_IICEN | I2C_C1_MST;
  }
  tmp = I2C0_D; // initiate the first receive
  while (lengthData > 1) {
    i2c_wait();
    lengthData--;
    if (lengthData == 1) I2C0_C1 = I2C_C1_IICEN | I2C_C1_MST | I2C_C1_TXAK;
    rxBuffer[count++] = I2C0_D;
  }
  i2c_wait();
  I2C0_C1 = I2C_C1_IICEN | I2C_C1_MST | I2C_C1_TX;
  rxBuffer[count++] = I2C0_D;
  if (sendStop) I2C0_C1 = I2C_C1_IICEN;
  rxBufferLength = count;
  return count;
}

uint8_t i2cDataAvailable(void)
{
  return rxBufferLength - rxBufferIndex;
}

uint16_t i2cRead(void)
{
  if (rxBufferIndex >= rxBufferLength) return -1;
  return rxBuffer[rxBufferIndex++];
}

void i2c0_isr(void)
{
  uint8_t status, c1, data;
  static uint8_t receiving=0;

  status = I2C0_S;
  if (status & I2C_S_ARBL) {
    // Arbitration Lost
    I2C0_S = I2C_S_ARBL;
    if (receiving && rxBufferLength > 0) {

    }
    if (!(status & I2C_S_IAAS)) return;
  }
  if (status & I2C_S_IAAS) {
    // Addressed As A Slave
    if (status & I2C_S_SRW) {
      // Begin Slave Transmit
      receiving = 0;
      txBufferLength = 0;

      if (txBufferLength == 0) {
        txBufferLength = 1;
        txBuffer[0] = 0;
      }
      I2C0_C1 = I2C_C1_IICEN | I2C_C1_IICIE | I2C_C1_TX;
      I2C0_D = txBuffer[0];
      txBufferIndex = 1;
    } else {
      // Begin Slave Receive
      receiving = 1;
      rxBufferLength = 0;
      I2C0_C1 = I2C_C1_IICEN | I2C_C1_IICIE;
      data = I2C0_D;
    }
    I2C0_S = I2C_S_IICIF;
    return;
  }
  c1 = I2C0_C1;
  if (c1 & I2C_C1_TX) {
    // Continue Slave Transmit
    if ((status & I2C_S_RXAK) == 0) {
      // Master ACK'd previous byte
      if (txBufferIndex < txBufferLength) {
        I2C0_D = txBuffer[txBufferIndex++];
      } else {
        I2C0_D = 0;
      }
      I2C0_C1 = I2C_C1_IICEN | I2C_C1_IICIE | I2C_C1_TX;
    } else {
      // Master did not ACK previous byte
      I2C0_C1 = I2C_C1_IICEN | I2C_C1_IICIE;
      data = I2C0_D;
    }
  } else {
    // Continue Slave Receive
    irqcount = 0;
    
    data = I2C0_D;
    
    if (rxBufferLength < BUFFER_LENGTH && receiving) {
      rxBuffer[rxBufferLength++] = data;
    }
    
  }
  I2C0_S = I2C_S_IICIF;
}
