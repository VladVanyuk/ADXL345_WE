/********************************************************************
 * This is a library for the ADXL345 accelerometer.
 *
 * You'll find an example which should enable you to use the library.
 *
 * You are free to use it, change it or build on it. In case you like
 * it, it would be cool if you give it a star.
 *
 * If you find bugs, please inform me!
 *
 * Written by Wolfgang (Wolle) Ewald
 * https://wolles-elektronikkiste.de/adxl345-teil-1 (German)
 * https://wolles-elektronikkiste.de/en/adxl345-the-universal-accelerometer-part-1 (English)
 *
 *********************************************************************/

 #include "Arduino.h"

 #include "ADXL345_WE.h"

 // #define ADXL345_DEBUG 1
 
 #ifdef ADXL345_DEBUG
 #define ADXL_PRINT(x) Serial.print(x)
 #define ADXL_PRINTLN(x) Serial.println(x)
 #define ADXL_PRINT_BIN(x) Serial.print(x, BIN)
 #define ADXL_PRINT_HEX(x) Serial.print(x, HEX)
 #else
 #define ADXL_PRINT(x)
 #define ADXL_PRINTLN(x)
 #define ADXL_PRINT_BIN(x)
 #define ADXL_PRINT_HEX(x)
 #endif
 
 #define ADXL345_TO_READ (6) // num of bytes we are going to read each time (two bytes for each axis)
 
 void print_byte(byte val)
 {
     int i;
     ADXL_PRINT("B");
     for (i = 7; i >= 0; i--)
     {
         ADXL_PRINT_BIN(val >> i & 1);
     }
 }
 
 /************ Basic settings ************/
 
 bool ADXL345_WE::init()
 {
     // status = ADXL345_OK;
     // error_code = ADXL345_NO_ERROR;
 
 #ifdef USE_SPI
     if (useSPI)
     {
         if (mosiPin == 999)
         {
             _spi->begin();
         }
 #ifdef ESP32
         else
         {
             _spi->begin(sckPin, misoPin, mosiPin, csPin);
         }
 #endif
         mySPISettings = SPISettings(5000000, MSBFIRST, SPI_MODE3);
         pinMode(csPin, OUTPUT);
         digitalWrite(csPin, HIGH);
     }
 #endif
 #if (defined USE_SPI) && (defined USE_I2C)
     else
     {
         if (_wire == nullptr)
         {
             _wire = &Wire;
         }
         #ifndef I2C_NO_SPEED_CONFIG
         _wire->setClock(400000L);
         #endif
         _wire->begin();
     }
 #elif defined USE_I2C
     if (!useSPI)
     {
         if (_wire == nullptr)
         {
             _wire = &Wire;
         }
         #ifndef I2C_NO_SPEED_CONFIG
         _wire->setClock(400000L);
         #endif
         _wire->begin();
     }
 #endif
 
     writeToRegister(ADXL345_POWER_CTL, 0);
     writeToRegister(ADXL345_POWER_CTL, 16);
     setMeasureMode(true);
     rangeFactor = 1.0;
    //  corrFact.x = 1.0;
    //  corrFact.y = 1.0;
    //  corrFact.z = 1.0;
    corrFact = {1.0, 1.0, 1.0};
    //  offsetVal.x = 0.0;
    //  offsetVal.y = 0.0;
    //  offsetVal.z = 0.0;
    offsetVal = {0.0, 0.0, 0.0};
    //  angleOffsetVal.x = 0.0;
    //  angleOffsetVal.y = 0.0;
    //  angleOffsetVal.z = 0.0;
    angleOffsetVal = {0.0, 0.0, 0.0};
     writeToRegister(ADXL345_DATA_FORMAT, 0);
     setFullRes(true);
     uint8_t ctrlVal = readRegisterSingle(ADXL345_DATA_FORMAT);
     if (ctrlVal != 0b1000)
     {
         return false;
     }
     // if(!((readRegisterSingle(ADXL345_DATA_FORMAT)) & (1<<ADXL345_FULL_RES))){
     //     return false;
     // }
     writeToRegister(ADXL345_INT_ENABLE, 0);
     writeToRegister(ADXL345_INT_MAP, 0);
     writeToRegister(ADXL345_TIME_INACT, 0);
     writeToRegister(ADXL345_THRESH_INACT, 0);
     writeToRegister(ADXL345_ACT_INACT_CTL, 0);
     writeToRegister(ADXL345_DUR, 0);
     writeToRegister(ADXL345_LATENT, 0);
     writeToRegister(ADXL345_THRESH_TAP, 0);
     writeToRegister(ADXL345_TAP_AXES, 0);
     writeToRegister(ADXL345_WINDOW, 0);
     readAndClearInterrupts();
     writeToRegister(ADXL345_FIFO_CTL, 0);
     writeToRegister(ADXL345_FIFO_STATUS, 0);
 
     return true;
 }
 
 
/**
 * @brief Power ON
 */
// void FaBo3Axis::powerOn()
// {
//   uint8_t power = ADXL345_AUTO_SLEEP_OFF;
//   power |= ADXL345_MEASURE_ON;
//   power |= ADXL345_SLEEP_OFF;
//   power |= ADXL345_WAKEUP_8HZ;
//   writeI2c(ADXL345_POWER_CTL_REG, power);
// }

 /**************************************************************************/
 /*!
     @brief  Reads the device ID (can be used to check connection)
     @return The Device ID of the connected sensor
 */
 /**************************************************************************/
 uint8_t ADXL345_WE::getDeviceID(void) {
     // Check device ID register
     return readRegisterSingle(ADXL345_DEVID);
   }
 
bool ADXL345_WE::checkConnection()
   {
    uint8_t deviceid = getDeviceID();
    if (deviceid != ADXL345_DEVICE) {
    /* No ADXL345 detected ... return false */
        return false;
    }
    return true;
   }
   
 #ifdef USE_I2C
 void ADXL345_WE::setWire(TwoWire *w)
 {
     if (w == nullptr)
     {
         return;
     }
     this->_wire = w;
 }
 
 void ADXL345_WE::setAddr(uint8_t addr)
 {
     this->i2cAddress = addr;
 }
 #endif
 
 #ifdef USE_SPI
 void ADXL345_WE::setSPIClockSpeed(unsigned long clock)
 {
     mySPISettings = SPISettings(clock, MSBFIRST, SPI_MODE3);
 }
 #endif
 
 // gets the state of the SELF_TEST bit
 bool ADXL345_WE::getSelfTestBit()
 {
     return getRegisterBit(ADXL345_DATA_FORMAT, 7);
 }
 
 // Sets the SELF-TEST bit
 // if set to 1 it applies a self-test force to the sensor causing a shift in the output data
 // if set to 0 it disables the self-test force
 void ADXL345_WE::setSelfTestBit(bool selfTestBit)
 {
     setRegisterBit(ADXL345_DATA_FORMAT, 7, selfTestBit);
 }
 
 void ADXL345_WE::setCorrFactors(float xMin, float xMax, float yMin, float yMax, float zMin, float zMax)
 {
     corrFact.x = UNITS_PER_G / (0.5 * (xMax - xMin));
     corrFact.y = UNITS_PER_G / (0.5 * (yMax - yMin));
     corrFact.z = UNITS_PER_G / (0.5 * (zMax - zMin));
     offsetVal.x = (xMax + xMin) * 0.5;
     offsetVal.y = (yMax + yMin) * 0.5;
     offsetVal.z = (zMax + zMin) * 0.5;
 }
 
 void ADXL345_WE::setDataRate(adxl345_dataRate rate)
 {
     this->regVal |= readRegisterSingle(ADXL345_BW_RATE);
     this->regVal &= 0xF0;
     this->regVal |= rate;
     writeToRegister(ADXL345_BW_RATE, this->regVal);
 }
 
 adxl345_dataRate ADXL345_WE::getDataRate()
 {
     return (adxl345_dataRate)(readRegisterSingle(ADXL345_BW_RATE) & 0x0F);
 }
 
 String ADXL345_WE::getDataRateAsString()
 {
     adxl345_dataRate dataRate = (adxl345_dataRate)(readRegisterSingle(ADXL345_BW_RATE) & 0x0F);
     String returnString = "";
 
     switch (dataRate)
     {
     case ADXL345_DATA_RATE_3200:
         returnString = "3200 Hz";
         break;
     case ADXL345_DATA_RATE_1600:
         returnString = "1600 Hz";
         break;
     case ADXL345_DATA_RATE_800:
         returnString = "800 Hz";
         break;
     case ADXL345_DATA_RATE_400:
         returnString = "400 Hz";
         break;
     case ADXL345_DATA_RATE_200:
         returnString = "200 Hz";
         break;
     case ADXL345_DATA_RATE_100:
         returnString = "100 Hz";
         break;
     case ADXL345_DATA_RATE_50:
         returnString = "50 Hz";
         break;
     case ADXL345_DATA_RATE_25:
         returnString = "25 Hz";
         break;
     case ADXL345_DATA_RATE_12_5:
         returnString = "12.5 Hz";
         break;
     case ADXL345_DATA_RATE_6_25:
         returnString = "6.25 Hz";
         break;
     case ADXL345_DATA_RATE_3_13:
         returnString = "3.13 Hz";
         break;
     case ADXL345_DATA_RATE_1_56:
         returnString = "1.56 Hz";
         break;
     case ADXL345_DATA_RATE_0_78:
         returnString = "0.78 Hz";
         break;
     case ADXL345_DATA_RATE_0_39:
         returnString = "0.39 Hz";
         break;
     case ADXL345_DATA_RATE_0_20:
         returnString = "0.20 Hz";
         break;
     case ADXL345_DATA_RATE_0_10:
         returnString = "0.10 Hz";
         break;
     }
 
     return returnString;
 }
 
 uint8_t ADXL345_WE::getPowerCtlReg()
 {
     return readRegisterSingle(ADXL345_POWER_CTL);
 }
 
 void ADXL345_WE::setRange(adxl345_range range)
 {
     uint8_t regValue = readRegisterSingle(ADXL345_DATA_FORMAT);
     if (adxl345_lowRes)
     {
         switch (range)
         {
         case ADXL345_RANGE_2G:
             rangeFactor = 1.0;
             break;
         case ADXL345_RANGE_4G:
             rangeFactor = 2.0;
             break;
         case ADXL345_RANGE_8G:
             rangeFactor = 4.0;
             break;
         case ADXL345_RANGE_16G:
             rangeFactor = 8.0;
             break;
         }
     }
     else
     {
         rangeFactor = 1.0;
     }
     regValue &= 0b11111100;
     regValue |= range;
     writeToRegister(ADXL345_DATA_FORMAT, regValue);
 }
 
 adxl345_range ADXL345_WE::getRange()
 {
     this->regVal = readRegisterSingle(ADXL345_DATA_FORMAT);
     this->regVal &= 0x03;
     return adxl345_range(this->regVal);
 }
 
 void ADXL345_WE::setFullRes(boolean full)
 {
     this->regVal = readRegisterSingle(ADXL345_DATA_FORMAT);
     if (full)
     {
         adxl345_lowRes = false;
         rangeFactor = 1.0;
         this->regVal |= (1 << ADXL345_FULL_RES);
     }
     else
     {
         adxl345_lowRes = true;
         this->regVal &= ~(1 << ADXL345_FULL_RES);
         setRange(getRange());
     }
     writeToRegister(ADXL345_DATA_FORMAT, this->regVal);
 }
 
 String ADXL345_WE::getRangeAsString()
 {
     String rangeAsString = "";
     adxl345_range range = getRange();
     switch (range)
     {
     case ADXL345_RANGE_2G:
         rangeAsString = "2g";
         break;
     case ADXL345_RANGE_4G:
         rangeAsString = "4g";
         break;
     case ADXL345_RANGE_8G:
         rangeAsString = "8g";
         break;
     case ADXL345_RANGE_16G:
         rangeAsString = "16g";
         break;
     }
     return rangeAsString;
 }
 
 /************ x,y,z results ************/
 
 xyzFloat ADXL345_WE::getRawValues()
 {
     uint8_t rawData[ADXL345_TO_READ];
     xyzFloat rawVal = {0.0, 0.0, 0.0};
     readFromRegisterMulti(ADXL345_DATAX0, ADXL345_TO_READ, rawData);
     rawVal.x = (static_cast<int16_t>((rawData[1] << 8) | rawData[0])) * 1.0;
     rawVal.y = (static_cast<int16_t>((rawData[3] << 8) | rawData[2])) * 1.0;
     rawVal.z = (static_cast<int16_t>((rawData[5] << 8) | rawData[4])) * 1.0;
 
     return rawVal;
 }
 void ADXL345_WE::getRawValues(xyzFloat *rawVal){
    // uint8_t rawData[ADXL345_TO_READ]; 
    // readMultipleRegisters(ADXL345_DATAX0, ADXL345_TO_READ, rawData);
    // rawVal->x = (static_cast<int16_t>((rawData[1] << 8) | rawData[0])) * 1.0;
    // rawVal->y = (static_cast<int16_t>((rawData[3] << 8) | rawData[2])) * 1.0;
    // rawVal->z = (static_cast<int16_t>((rawData[5] << 8) | rawData[4])) * 1.0;
    *rawVal = getRawValues();
}

 xyzFloat ADXL345_WE::getCorrectedRawValues()
 {
     uint8_t rawData[ADXL345_TO_READ];
     xyzFloat rawVal = {0.0, 0.0, 0.0};
     readFromRegisterMulti(ADXL345_DATAX0, ADXL345_TO_READ, rawData);
     int16_t xRaw = static_cast<int16_t>(rawData[1] << 8) | rawData[0];
     int16_t yRaw = static_cast<int16_t>(rawData[3] << 8) | rawData[2];
     int16_t zRaw = static_cast<int16_t>(rawData[5] << 8) | rawData[4];
 
     rawVal.x = xRaw * 1.0 - (offsetVal.x / rangeFactor);
     rawVal.y = yRaw * 1.0 - (offsetVal.y / rangeFactor);
     rawVal.z = zRaw * 1.0 - (offsetVal.z / rangeFactor);
 
     return rawVal;
 }
 
 void ADXL345_WE::getCorrectedRawValues(xyzFloat *rawVal){
    // uint8_t rawData[ADXL345_TO_READ]; 
    // readFromRegisterMulti(ADXL345_DATAX0, ADXL345_TO_READ, rawData);
    // int16_t xRaw = static_cast<int16_t>(rawData[1] << 8) | rawData[0];
    // int16_t yRaw = static_cast<int16_t>(rawData[3] << 8) | rawData[2];
    // int16_t zRaw = static_cast<int16_t>(rawData[5] << 8) | rawData[4];
        
    // rawVal->x = xRaw * 1.0 - (offsetVal.x / rangeFactor);
    // rawVal->y = yRaw * 1.0 - (offsetVal.y / rangeFactor);
    // rawVal->z = zRaw * 1.0 - (offsetVal.z / rangeFactor);
    *rawVal = getCorrectedRawValues();
}
 xyzFloat ADXL345_WE::getGValues()
 {
     xyzFloat rawVal = getCorrectedRawValues();
     xyzFloat gVal = {0.0, 0.0, 0.0};
     gVal.x = rawVal.x * MILLI_G_PER_LSB * rangeFactor * corrFact.x / 1000.0;
     gVal.y = rawVal.y * MILLI_G_PER_LSB * rangeFactor * corrFact.y / 1000.0;
     gVal.z = rawVal.z * MILLI_G_PER_LSB * rangeFactor * corrFact.z / 1000.0;
     return gVal;
 }

 void ADXL345_WE::getGValues(xyzFloat *gVal){
    // getCorrectedRawValues(&rawVal);
    // xyzFloat rawVal = getCorrectedRawValues();
    // gVal->x = rawVal.x * corrFact.x * MILLI_G_PER_LSB * rangeFactor / 1000.0; 
    // gVal->y = rawVal.y * corrFact.y * MILLI_G_PER_LSB * rangeFactor / 1000.0; 
    // gVal->z = rawVal.z * corrFact.z * MILLI_G_PER_LSB * rangeFactor / 1000.0; 
    *gVal = getGValues();
}
 
 float ADXL345_WE::getVectorG()
 {
     xyzFloat gVal = getGValues();
     return VECTOR_G(gVal);
 }
 
 float ADXL345_WE::getVectorG(xyzVectorG *allValues)
 {
     if (allValues == nullptr)
     {
         return 0.0;
     }
     xyzFloat gVal = getGValues();
     *allValues->values->x = gVal.x;
     *allValues->values->y = gVal.y;
     *allValues->values->z = gVal.z;
     *allValues->g = VECTOR_G(gVal);
     return (*allValues->g);
 }
 
 // float ADXL345_WE::getVectorG(void *gValStruct){
 // if (gValStruct == nullptr)
 // {
 // return 0.0;
 // }
 // xyzFloat *gVal = static_cast<xyzFloat*>(gValStruct);
 // *gVal =
 //
 // *(static_cast<xyzFloat*>(gValStruct)) = getGValues();;
 // return sqrt(gVal->x * gVal->x + gVal->y * gVal->y + gVal->z * gVal->z);
 // }
 //
 
 // float ADXL345_WE::getVectorG(float *x, float *y, float *z){
 //     xyzFloat gVal  = {0.0, 0.0, 0.0};
 //     float vector = getVectorG(&gVal);
 //     *x = gVal.x;
 //     *y = gVal.y;
 //     *z = gVal.z;
 //     return vector;
 // }
 
 xyzFloat ADXL345_WE::getAngles()
 {
     xyzFloat gVal = getGValues();
     xyzFloat angleVal = {0.0, 0.0, 0.0};
     if (gVal.x > 1)
     {
         gVal.x = 1;
     }
     else if (gVal.x < -1)
     {
         gVal.x = -1;
     }
     angleVal.x = (asin(gVal.x)) * 57.296;
 
     if (gVal.y > 1)
     {
         gVal.y = 1;
     }
     else if (gVal.y < -1)
     {
         gVal.y = -1;
     }
     angleVal.y = (asin(gVal.y)) * 57.296;
 
     if (gVal.z > 1)
     {
         gVal.z = 1;
     }
     else if (gVal.z < -1)
     {
         gVal.z = -1;
     }
     angleVal.z = (asin(gVal.z)) * 57.296;
 
     return angleVal;
 }
 
 xyzFloat ADXL345_WE::getCorrAngles()
 {
     xyzFloat corrAnglesVal = getAngles();
     corrAnglesVal.x -= angleOffsetVal.x;
     corrAnglesVal.y -= angleOffsetVal.y;
     corrAnglesVal.z -= angleOffsetVal.z;
 
     return corrAnglesVal;
 }
 
 /************ Angles and Orientation ************/
 
 xyzFloat ADXL345_WE::getAngleOffsets()
 {
     return angleOffsetVal;
 }
 
 void ADXL345_WE::setAngleOffsets(xyzFloat aos)
 {
     this->angleOffsetVal = aos;
 }
 
 void ADXL345_WE::measureAngleOffsets(xyzFloat * aos)
 {
     if (aos)
     {
         *aos = getAngles();
         setAngleOffsets(*aos);
     }
     else
     {
         // angleOffsetVal = getAngles();
         setAngleOffsets(getAngles());
     }
     
 }
 
 adxl345_orientation ADXL345_WE::getOrientation()
 {
     adxl345_orientation orientation = FLAT;
     xyzFloat angleVal = getAngles();
     if (abs(angleVal.x) < 45)
     { // |x| < 45
         if (abs(angleVal.y) < 45)
         { // |y| < 45
             if (angleVal.z > 0)
             { //  z  > 0
                 orientation = FLAT;
             }
             else
             { //  z  < 0
                 orientation = FLAT_1;
             }
         }
         else
         { // |y| > 45
             if (angleVal.y > 0)
             { //  y  > 0
                 orientation = XY;
             }
             else
             { //  y  < 0
                 orientation = XY_1;
             }
         }
     }
     else
     { // |x| >= 45
         if (angleVal.x > 0)
         { //  x  >  0
             orientation = YX;
         }
         else
         { //  x  <  0
             orientation = YX_1;
         }
     }
     return orientation;
 }
 
 String ADXL345_WE::getOrientationAsString()
 {
     adxl345_orientation orientation = getOrientation();
     String orientationAsString = "";
     switch (orientation)
     {
     case FLAT:
         orientationAsString = "z up";
         break;
     case FLAT_1:
         orientationAsString = "z down";
         break;
     case XY:
         orientationAsString = "y up";
         break;
     case XY_1:
         orientationAsString = "y down";
         break;
     case YX:
         orientationAsString = "x up";
         break;
     case YX_1:
         orientationAsString = "x down";
         break;
     }
     return orientationAsString;
 }
 
 float ADXL345_WE::getPitch()
 {
     xyzFloat gVal = getGValues();
     float pitch = (atan2(-gVal.x, sqrt(abs((gVal.y * gVal.y + gVal.z * gVal.z)))) * 180.0) / M_PI;
     return pitch;
 }
 
 float ADXL345_WE::getRoll()
 {
     xyzFloat gVal = getGValues();
     float roll = (atan2(gVal.y, gVal.z) * 180.0) / M_PI;
     return roll;
 }
 
 /************ Power, Sleep, Standby ************/
 
 void ADXL345_WE::setMeasureMode(bool measure)
 {
     this->regVal = readRegisterSingle(ADXL345_POWER_CTL);
     if (measure)
     {
         this->regVal |= (1 << ADXL345_MEASURE);
     }
     else
     {
         this->regVal &= ~(1 << ADXL345_MEASURE);
     }
     writeToRegister(ADXL345_POWER_CTL, this->regVal);
 }
 
 void ADXL345_WE::setSleep(bool sleep, adxl345_wUpFreq freq)
 {
     this->regVal = readRegisterSingle(ADXL345_POWER_CTL);
     this->regVal &= 0b11111100;
     this->regVal |= freq;
     if (sleep)
     {
         this->regVal |= (1 << ADXL345_SLEEP);
     }
     else
     {
         setMeasureMode(false); // it is recommended to enter Stand Mode when clearing the Sleep Bit!
         this->regVal &= ~(1 << ADXL345_SLEEP);
         this->regVal &= ~(1 << ADXL345_MEASURE);
     }
     writeToRegister(ADXL345_POWER_CTL, this->regVal);
     if (!sleep)
     {
         setMeasureMode(true);
     }
 }
 
 void ADXL345_WE::setSleep(bool sleep)
 {
     this->regVal = readRegisterSingle(ADXL345_POWER_CTL);
     if (sleep)
     {
         this->regVal |= (1 << ADXL345_SLEEP);
     }
     else
     {
         setMeasureMode(false); // it is recommended to enter Stand Mode when clearing the Sleep Bit!
         this->regVal &= ~(1 << ADXL345_SLEEP);
         this->regVal &= ~(1 << ADXL345_MEASURE);
     }
     writeToRegister(ADXL345_POWER_CTL, this->regVal);
     if (!sleep)
     {
         setMeasureMode(true);
     }
 }
 
 void ADXL345_WE::setAutoSleep(bool autoSleep, adxl345_wUpFreq freq)
 {
     if (autoSleep)
     {
         setLinkBit(true);
     }
     this->regVal = readRegisterSingle(ADXL345_POWER_CTL);
     this->regVal &= 0b11111100;
     this->regVal |= freq;
     if (autoSleep)
     {
         this->regVal |= (1 << ADXL345_AUTO_SLEEP);
     }
     else
     {
         this->regVal &= ~(1 << ADXL345_AUTO_SLEEP);
     }
     writeToRegister(ADXL345_POWER_CTL, this->regVal);
 }
 
 void ADXL345_WE::setAutoSleep(bool autoSleep)
 {
     if (autoSleep)
     {
         setLinkBit(true);
         this->regVal = readRegisterSingle(ADXL345_POWER_CTL);
         this->regVal |= (1 << ADXL345_AUTO_SLEEP);
         writeToRegister(ADXL345_POWER_CTL, this->regVal);
     }
     else
     {
         this->regVal = readRegisterSingle(ADXL345_POWER_CTL);
         this->regVal &= ~(1 << ADXL345_AUTO_SLEEP);
         writeToRegister(ADXL345_POWER_CTL, this->regVal);
     }
 }
 
 bool ADXL345_WE::isAsleep()
 {
     return readRegisterSingle(ADXL345_ACT_TAP_STATUS) & (1 << ADXL345_ASLEEP);
 }
 
 void ADXL345_WE::setLowPower(bool lowpwr)
 {
     this->regVal = readRegisterSingle(ADXL345_BW_RATE);
     if (lowpwr)
     {
         this->regVal |= (1 << ADXL345_LOW_POWER);
     }
     else
     {
         this->regVal &= ~(1 << ADXL345_LOW_POWER);
     }
     writeToRegister(ADXL345_BW_RATE, this->regVal);
 }
 
 bool ADXL345_WE::isLowPower()
 {
     return readRegisterSingle(ADXL345_BW_RATE) & (1 << ADXL345_LOW_POWER);
 }
 
 /************ Interrupts ************/
 
 void ADXL345_WE::setInterrupt(adxl345_int type, uint8_t pin)
 {
     this->regVal = readRegisterSingle(ADXL345_INT_ENABLE);
     this->regVal |= (1 << type);
     writeToRegister(ADXL345_INT_ENABLE, this->regVal);
     this->regVal = readRegisterSingle(ADXL345_INT_MAP);
     if (pin == INT_PIN_1)
     {
         this->regVal &= ~(1 << type);
     }
     else
     {
         this->regVal |= (1 << type);
     }
     writeToRegister(ADXL345_INT_MAP, this->regVal);
 }
 
 void ADXL345_WE::setInterruptPolarity(uint8_t pol)
 {
     this->regVal = readRegisterSingle(ADXL345_DATA_FORMAT);
     if (pol == ADXL345_ACT_HIGH)
     {
         this->regVal &= ~(0b00100000);
     }
     else if (pol == ADXL345_ACT_LOW)
     {
         this->regVal |= 0b00100000;
     }
     writeToRegister(ADXL345_DATA_FORMAT, this->regVal);
 }
 
 void ADXL345_WE::deleteInterrupt(adxl345_int type)
 {
     this->regVal = readRegisterSingle(ADXL345_INT_ENABLE);
     this->regVal &= ~(1 << type);
     writeToRegister(ADXL345_INT_ENABLE, this->regVal);
 }
 
 uint8_t ADXL345_WE::readAndClearInterrupts()
 {
     this->regVal = readRegisterSingle(ADXL345_INT_SOURCE);
     return this->regVal;
 }
 
 // TODO RENAMETO if (adxl.triggered(intEvent, ADXL345_WATERMARK)) { // if watermark interrupt occured

 bool ADXL345_WE::checkInterrupt(uint8_t source, adxl345_int type) 
 {
     source &= (1 << type);
     return source;
 }
 
 void ADXL345_WE::setLinkBit(bool link)
 {
     this->regVal = readRegisterSingle(ADXL345_POWER_CTL);
     if (link)
     {
         this->regVal |= (1 << ADXL345_LINK);
     }
     else
     {
         this->regVal &= ~(1 << ADXL345_LINK);
     }
     writeToRegister(ADXL345_POWER_CTL, this->regVal);
 }
 
 void ADXL345_WE::setFreeFallThresholds(float threshold, float fftime)
 {
     double regValRaw = round(threshold / 0.0625);
     this->regVal = static_cast<uint8_t>(regValRaw > UINT8_MAX ? UINT8_MAX : regValRaw);
     if (this->regVal < 1)
     {
         this->regVal = 1;
     }
     writeToRegister(ADXL345_THRESH_FF, this->regVal);
     this->regVal = static_cast<uint8_t>(round(fftime / 5));
     if (this->regVal < 1)
     {
         this->regVal = 1;
     }
     writeToRegister(ADXL345_TIME_FF, this->regVal);
 }
 
 void ADXL345_WE::setActivityParameters(adxl345_dcAcMode mode, adxl345_actTapSet axes, float threshold)
 {
     double regValRaw = round(threshold / 0.0625);
     this->regVal = static_cast<uint8_t>(regValRaw > UINT8_MAX ? UINT8_MAX : regValRaw);
     if (this->regVal < 1)
     {
         this->regVal = 1;
     }
     writeToRegister(ADXL345_THRESH_ACT, this->regVal);
 
     this->regVal = readRegisterSingle(ADXL345_ACT_INACT_CTL);
     this->regVal &= 0x0F;
     this->regVal |= (static_cast<uint8_t>(mode) + static_cast<uint8_t>(axes)) << 4;
     writeToRegister(ADXL345_ACT_INACT_CTL, this->regVal);
 }
 
 void ADXL345_WE::setInactivityParameters(adxl345_dcAcMode mode, adxl345_actTapSet axes, float threshold, uint8_t inactTime)
 {
     double regValRaw = round(threshold / 0.0625);
     this->regVal = static_cast<uint8_t>(regValRaw > UINT8_MAX ? UINT8_MAX : regValRaw);
     if (this->regVal < 1)
     {
         this->regVal = 1;
     }
     writeToRegister(ADXL345_THRESH_INACT, this->regVal);
 
     this->regVal = readRegisterSingle(ADXL345_ACT_INACT_CTL);
     this->regVal &= 0xF0;
     this->regVal |= static_cast<uint8_t>(mode) + static_cast<uint16_t>(axes);
     writeToRegister(ADXL345_ACT_INACT_CTL, this->regVal);
 
     writeToRegister(ADXL345_TIME_INACT, inactTime);
 }
 
 /* The following four parameters have to be set for tap application (single and double):
     1. Axes, that are considered:
         ADXL345_000  -  no axis (which makes no sense)
         ADXL345_00Z  -  z
         ADXL345_0Y0  -  y
         ADXL345_0YZ  -  y,z
         ADXL345_X00  -  x
         ADXL345_X0Z  -  x,z
         ADXL345_XY0  -  x,y
         ADXL345_XYZ  -  all axes
     2. Threshold in g
         It is recommended to not choose the value to low. 3g is a good starting point.
     3. Duration in milliseconds (max 159 ms):
         maximum time that the acceleration must be over g threshold to be regarded as a single tap. If
         the acceleration drops below the g threshold before the duration is exceeded an interrupt will be
         triggered. If also double tap is active an interrupt will only be triggered after the double tap
         conditions have been checked. Duration should be greater than 10.
     4. Latency time in milliseconds (maximum: 318 ms): minimum time before the next tap can be detected.
         Starts at the end of duration or when the interrupt was triggered. Should be greater than 20 ms.
 */
 void ADXL345_WE::setGeneralTapParameters(adxl345_actTapSet axes, float threshold, float duration, float latent)
 {
     this->regVal = readRegisterSingle(ADXL345_TAP_AXES);
     this->regVal &= 0b11111000;
     this->regVal |= static_cast<uint8_t>(axes);
     writeToRegister(ADXL345_TAP_AXES, this->regVal);
 
     double regValRaw = round(threshold / 0.0625); // todo name coef 0.0625
     this->regVal = static_cast<uint8_t>(regValRaw > UINT8_MAX ? UINT8_MAX : regValRaw);
     if (this->regVal < 1)
     {
         this->regVal = 1;
     }
     writeToRegister(ADXL345_THRESH_TAP, this->regVal);
 
     if (duration < 10)
     {
         duration = 10;
     }
     if (duration > 159) // todo make a define or variable
     {
         duration = 159;
     }
 
     regValRaw = round(duration / 0.625);
     this->regVal = static_cast<uint8_t>(regValRaw > UINT8_MAX ? UINT8_MAX : regValRaw);
     if (this->regVal < 1)
     {
         this->regVal = 1;
     }
     writeToRegister(ADXL345_DUR, this->regVal);
 
     if (latent < 20)
     {
         latent = 20;
     }
 
     if (latent > 318)
     {
         latent = 318;
     }
 
     regValRaw = round(latent / 1.25);
     this->regVal = static_cast<uint8_t>(regValRaw > UINT8_MAX ? UINT8_MAX : regValRaw);
     if (this->regVal < 1)
     {
         this->regVal = 1;
     }
     writeToRegister(ADXL345_LATENT, this->regVal);
 }
 
 void ADXL345_WE::setAdditionalDoubleTapParameters(bool suppress, float window)
 {
     this->regVal = readRegisterSingle(ADXL345_TAP_AXES);
     if (suppress)
     {
         this->regVal |= (1 << ADXL345_SUPPRESS);
     }
     else
     {
         this->regVal &= ~(1 << ADXL345_SUPPRESS);
     }
     writeToRegister(ADXL345_TAP_AXES, this->regVal);
 
     this->regVal = static_cast<uint8_t>(round(window / 1.25));
     writeToRegister(ADXL345_WINDOW, this->regVal);
 }
 
 uint8_t ADXL345_WE::getActTapStatus()
 {
     return readRegisterSingle(ADXL345_ACT_TAP_STATUS);
 }
 
 uint8_t ADXL345_WE::getActTapStatusAsValue()
 {
     uint8_t mask = (readRegisterSingle(ADXL345_ACT_INACT_CTL)) & 0b01110000;
     mask |= ((readRegisterSingle(ADXL345_TAP_AXES)) & 0b00000111);
 
     this->regVal = getActTapStatus();
     this->regVal &= mask;
     return this->regVal;
 }
 
 String ADXL345_WE::getActTapStatusAsString()
 {
     uint8_t regValue = getActTapStatusAsValue();
     String returnStr = "STS: ";
 
     if (regValue & (1 << ADXL345_TAP_Z))
     {
         returnStr += "TAP-Z ";
     }
     if (regValue & (1 << ADXL345_TAP_Y))
     {
         returnStr += "TAP-Y ";
     }
     if (regValue & (1 << ADXL345_TAP_X))
     {
         returnStr += "TAP-X ";
     }
     if (regValue & (1 << ADXL345_ACT_Z))
     {
         returnStr += "ACT-Z ";
     }
     if (regValue & (1 << ADXL345_ACT_Y))
     {
         returnStr += "ACT-Y ";
     }
     if (regValue & (1 << ADXL345_ACT_X))
     {
         returnStr += "ACT-X ";
     }
 
     return returnStr;
 }
 
 /************ FIFO ************/
 
 void ADXL345_WE::setFifoParameters(adxl345_triggerInt intNumber, uint8_t samples)
 {
     if (samples > MAX_ADXL_BUFF_SIZE)
     {
         samples = MAX_ADXL_BUFF_SIZE;
     }
 
     this->regVal = readRegisterSingle(ADXL345_FIFO_CTL);
     this->regVal &= 0b11000000;
     this->regVal |= (samples - 1);
     if (intNumber == ADXL345_TRIGGER_INT_2)
     {
         this->regVal |= 0x20;
     }
     writeToRegister(ADXL345_FIFO_CTL, this->regVal);
 }
 
 void ADXL345_WE::setFifoMode(adxl345_fifoMode mode)
 {
     this->regVal = readRegisterSingle(ADXL345_FIFO_CTL);
     this->regVal &= 0b00111111;
     this->regVal |= (mode << 6);
     writeToRegister(ADXL345_FIFO_CTL, this->regVal);
 }
 
 uint8_t ADXL345_WE::getFifoStatus()
 {
     return readRegisterSingle(ADXL345_FIFO_STATUS);
 }

 void ADXL345_WE::resetTrigger()
 {
     setFifoMode(ADXL345_BYPASS);
     setFifoMode(ADXL345_TRIGGER);
 }
 
// getFifoEntries OR getFifoSize
byte ADXL345_WE::getFifoSize(void) {
    byte _b;
    readFromRegisterMulti(ADXL345_FIFO_STATUS, 1, &_b);
    _b &=  0b00111111; //MASK FOR CURRENT SIZE OF FIFO BUFFER
    return _b;
}


void ADXL345_WE::burstReadXYZ(float* x, float* y, float* z, byte samples) {
    for (int i = 0; i < samples; i++) {
        xyzFloat rawData = getRawValues();
        x[i] = rawData.x;
        y[i] = rawData.y;
        z[i] = rawData.z;
        ADXL_PRINT("Sample "); ADXL_PRINT(i); ADXL_PRINTLN(": X="); ADXL_PRINT(x[i]); ADXL_PRINT(" Y="); ADXL_PRINT(y[i]); ADXL_PRINT(" Z="); ADXL_PRINT(z[i]);
    }
}

 /************************************************
     private functions
 *************************************************/
 
 // writeTo
 uint8_t ADXL345_WE::writeToRegister(uint8_t reg_addr, uint8_t val)
 {
     if (!useSPI)
     {
 #ifdef USE_I2C
         _wire->beginTransmission(i2cAddress);
         _wire->write(reg_addr);
         _wire->write(val);
         return _wire->endTransmission();
 #endif
     }
     else
     {
 #ifdef USE_SPI
         _spi->beginTransaction(mySPISettings);
         digitalWrite(csPin, LOW);
         _spi->transfer(reg_addr);
         _spi->transfer(val);
         digitalWrite(csPin, HIGH);
         _spi->endTransaction();
 #endif
         return false; // to be amended
     }
 }
 
 uint8_t ADXL345_WE::readRegisterSingle(uint8_t reg_addr)
 {
     uint8_t regValue = 0;
     if (!useSPI)
     {
 #ifdef USE_I2C
         _wire->beginTransmission(i2cAddress);
         _wire->write(reg_addr);
         _wire->endTransmission(false);
         _wire->requestFrom(i2cAddress, static_cast<uint8_t>(1));
         if (_wire->available())
         {
             regValue = _wire->read();
         }
        // Wire.endTransmission();
 #endif
     }
     else
     {
 #ifdef USE_SPI
         reg_addr |= 0x80;
         _spi->beginTransaction(mySPISettings);
         digitalWrite(csPin, LOW);
         _spi->transfer(reg_addr);
         regValue = _spi->transfer(0x00);
         digitalWrite(csPin, HIGH);
         _spi->endTransaction();
 #endif
     }
     return regValue;
 }
 
 // readFrom
 void ADXL345_WE::readFromRegisterMulti(uint8_t reg_addr, uint8_t count, uint8_t *buf)
 {
     if (!useSPI)
     {
 #ifdef USE_I2C
         _wire->beginTransmission(i2cAddress);
         _wire->write(reg_addr);
         _wire->endTransmission(false);
         _wire->requestFrom(i2cAddress, count);
         for (int i = 0; i < count; i++)
         {
             buf[i] = _wire->read();
         }
        // Wire.endTransmission();
         /* todo OR
         Wire.beginTransmission(ADXL345_DEVICE); // start transmission to device
         Wire.requestFrom(ADXL345_DEVICE, num);    // request 6 bytes from device
         int i = 0;
         while (Wire.available()) {      // device may send less than requested (abnormal)
             _buff[i] = Wire.read();    // receive a byte
             i++;
             if (i > num) {
                 break;
             }
         }
         if (i != num) {
             status = ADXL345_ERROR;         // #define ADXL345_OK    0 // no error
             error_code = ADXL345_READ_ERROR; // #define ADXL345_ERROR 1 // indicates error is predent
         }
         Wire.endTransmission();
 
         ? #define ADXL345_NO_ERROR   0 // initial state
         ? #define ADXL345_READ_ERROR 1 // problem reading accel
         ? #define ADXL345_BAD_ARG    2 // bad method argument
 
         */
 #endif
     }
     else
     {
 #ifdef USE_SPI
         reg_addr = reg_addr | 0x80;
         reg_addr = reg_addr | 0x40;
         _spi->beginTransaction(mySPISettings);
         digitalWrite(csPin, LOW);
         _spi->transfer(reg_addr);
         for (int i = 0; i < count; i++)
         {
             buf[i] = _spi->transfer(0x00);
         }
         digitalWrite(csPin, HIGH);
         _spi->endTransaction();
 #endif
     }
 }
 
 // print all register value to the serial ouptut, which requires it to be setup
 // this can be used to manually to check the current configuration of the device
 

void ADXL345_WE::printAllRegister() {
	byte _b;
	Serial.print("0x00: ");
	readFromRegisterMulti(ADXL345_DEVID, 1, &_b);
//	print_byte(_b);
//	Serial.println("");
	int i;
	for (i=29;i<=57;i++){
        //todo add adxl debug macro
		// Serial.print("0x");
		// Serial.print(i, HEX);
		// Serial.print(": ");
		readFromRegisterMulti(i, 1, &_b);
	//	print_byte(_b);
	//	Serial.println("");
	}
}

 bool ADXL345_WE::getRegisterBit(byte regAdress, int bitPos)
 {
     byte _b;
     readFromRegisterMulti(regAdress, 1, &_b);
     return ((_b >> bitPos) & 1);
 }
 
 void ADXL345_WE::setRegisterBit(byte regAdress, int bitPos, bool state)
 {
     byte _b;
     readFromRegisterMulti(regAdress, 1, &_b);
     if (state)
     {
         _b |= (1 << bitPos); // forces nth bit of _b to be 1.  all other bits left alone.
     }
     else
     {
         _b &= ~(1 << bitPos); // forces nth bit of _b to be 0.  all other bits left alone.
     }
     writeToRegister(regAdress, _b);
 }
 
 /*
 
 // read how many samples in Fifi
 https://github.com/Seeed-Studio/Accelerometer_ADXL345/blob/master/ADXL345.cpp#L59
 
 byte ADXL345::getFifoEntries(void) {
     byte _b;
     readFromRegisterMulti(ADXL345_FIFO_STATUS, 1, &_b);
     _b &=  0b00111111;
 
     return _b;
 }
 */
 
 
 void ADXL345_WE::readAccel(int* xyz) {
     readXYZ(xyz, xyz + 1, xyz + 2);
 }
 
 void ADXL345_WE::readXYZ(int* x, int* y, int* z) {
     uint8_t rawData[ADXL345_TO_READ];
     readFromRegisterMulti(ADXL345_DATAX0, ADXL345_TO_READ, rawData); //read the acceleration data from the ADXL345
     *x = (short)((((unsigned short)rawData[1]) << 8) | rawData[0]);
     *y = (short)((((unsigned short)rawData[3]) << 8) | rawData[2]);
     *z = (short)((((unsigned short)rawData[5]) << 8) | rawData[4]);
 }
 
 void ADXL345_WE::getAcceleration(double* xyz) {
     int i;
     int xyz_int[3];
     readAccel(xyz_int);
       double gains[3];        // counts to Gs
     gains[0] = 0.00376390;
     gains[1] = 0.00376009;
     gains[2] = 0.00349265;
     for (i = 0; i < 3; i++) {
         xyz[i] = xyz_int[i] * gains[i];
     }
 }
 
 
   