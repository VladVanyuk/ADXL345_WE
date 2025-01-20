/******************************************************************************
 *
 * This is a library for the ADXL345 accelerometer.
 *
 * You'll find several example sketches which should enable you to use the library. 
 *
 * You are free to use it, change it or build on it. In case you like it, it would
 * be cool if you give it a star.
 *
 * If you find bugs, please inform me!
 * 
 * Written by Wolfgang (Wolle) Ewald
 * https://wolles-elektronikkiste.de/adxl345-teil-1 (German)
 * https://wolles-elektronikkiste.de/en/adxl345-the-universal-accelerometer-part-1 (English)
 *
 * 
 ******************************************************************************/

#ifndef ADXL345_WE_H_
#define ADXL345_WE_H_

#if (ARDUINO >= 100)
 #include "Arduino.h"
#else
 #include "WProgram.h"
#endif

#define USE_I2C
//#define USE_SPI

#ifdef USE_I2C
#include <Wire.h>
#define ADXL345_I2CADDR 0x53
#define ADXL345_I2CADDR_HIGH 0x1D // if SDO = HIGH
#endif

#ifdef USE_SPI
#include <SPI.h>
#endif


// #define VECTOR_MAGNITUDE(x, y, z) (sqrt((x) * (x) + (y) * (y) + (z) * (z)))
#define VECTOR(x, y, z) (sqrt((x) * (x) + (y) * (y) + (z) * (z)))
#define VECTOR_G(xyz) (VECTOR(xyz.x, xyz.y, xyz.z))

/* Definitions */

constexpr uint8_t INT_PIN_1        {0x01};   
constexpr uint8_t INT_PIN_2        {0x02};
constexpr uint8_t ADXL345_ACT_LOW  {0x01}; 
constexpr uint8_t ADXL345_ACT_HIGH {0x00};

typedef enum ADXL345_PWR_CTL {
    ADXL345_WAKE_UP_0, ADXL_WAKE_UP_1, ADXL345_SLEEP, 
    ADXL345_MEASURE, ADXL345_AUTO_SLEEP, ADXL345_LINK
} adxl345_pwrCtl;

typedef enum ADXL345_WAKE_UP { // belongs to POWER CTL
    ADXL345_WAKE_UP_FREQ_8,
    ADXL345_WAKE_UP_FREQ_4,
    ADXL345_WAKE_UP_FREQ_2,
    ADXL345_WAKE_UP_FREQ_1,
} adxl345_wakeUpFreq;

typedef enum ADXL345_DATA_RATE {
    ADXL345_DATA_RATE_3200    = 0x0F,
    ADXL345_DATA_RATE_1600    = 0x0E,
    ADXL345_DATA_RATE_800     = 0x0D,
    ADXL345_DATA_RATE_400     = 0x0C,
    ADXL345_DATA_RATE_200     = 0x0B,
    ADXL345_DATA_RATE_100     = 0x0A,
    ADXL345_DATA_RATE_50      = 0x09,
    ADXL345_DATA_RATE_25      = 0x08,
    ADXL345_DATA_RATE_12_5    = 0x07,
    ADXL345_DATA_RATE_6_25    = 0x06,
    ADXL345_DATA_RATE_3_13    = 0x05,
    ADXL345_DATA_RATE_1_56    = 0x04,
    ADXL345_DATA_RATE_0_78    = 0x03,
    ADXL345_DATA_RATE_0_39    = 0x02,
    ADXL345_DATA_RATE_0_20    = 0x01,
    ADXL345_DATA_RATE_0_10    = 0x00
} adxl345_dataRate;

typedef enum ADXL345_RANGE {
    ADXL345_RANGE_16G          = 0b11,
    ADXL345_RANGE_8G           = 0b10,
    ADXL345_RANGE_4G           = 0b01,
    ADXL345_RANGE_2G           = 0b00
} adxl345_range;

typedef enum ADXL345_ORIENTATION {
  FLAT, FLAT_1, XY, XY_1, YX, YX_1
} adxl345_orientation;

typedef enum ADXL345_INT {
    ADXL345_OVERRUN, ADXL345_WATERMARK, ADXL345_FREEFALL, ADXL345_INACTIVITY, 
    ADXL345_ACTIVITY, ADXL345_DOUBLE_TAP, ADXL345_SINGLE_TAP, ADXL345_DATA_READY
} adxl345_int;

typedef enum ADXL345_ACT_TAP_SET {
    ADXL345_000, ADXL345_00Z, ADXL345_0Y0, ADXL345_0YZ,
    ADXL345_X00, ADXL345_X0Z, ADXL345_XY0, ADXL345_XYZ
} adxl345_actTapSet;

typedef enum ADXL345_DC_AC {
    ADXL345_DC_MODE = 0,
    ADXL345_AC_MODE = 0x08
} adxl345_dcAcMode;

typedef enum ADXL345_WAKE_UP_FREQ{
    ADXL345_WUP_FQ_8, ADXL345_WUP_FQ_4, ADXL345_WUP_FQ_2, ADXL345_WUP_FQ_1
} adxl345_wUpFreq;

typedef enum ADXL345_ACT_TAP {
    ADXL345_TAP_Z, ADXL345_TAP_Y, ADXL345_TAP_X, ADXL345_ASLEEP, ADXL345_ACT_Z, ADXL345_ACT_Y, ADXL345_ACT_X
} adxl345_actTap;

typedef enum ADXL345_FIFO_MODE {
    ADXL345_BYPASS, ADXL345_FIFO, ADXL345_STREAM, ADXL345_TRIGGER
} adxl345_fifoMode;

typedef enum ADXL345_TRIGGER_INT {
    ADXL345_TRIGGER_INT_1, ADXL345_TRIGGER_INT_2
} adxl345_triggerInt;

struct xyzFloat {
    float x;
    float y;
    float z;
};
//typedef struct xyzFloat xyzFloat_t;

class ADXL345_WE
{
    public: 
        
        /* Constructors */
        ADXL345_WE() : useSPI{false} {}
#ifdef USE_I2C
        ADXL345_WE(uint8_t addr) : _wire{&Wire}, i2cAddress{addr}, useSPI{false} {} //uint8_t addr = 0x53
        
        ADXL345_WE(TwoWire *w, uint8_t addr = 0x53) : _wire{w}, i2cAddress{addr}, useSPI{false} {}
#endif
#ifdef USE_SPI
        ADXL345_WE(int cs, bool spi, int mosi = 999, int miso = 999, int sck = 999) 
            : _spi{&SPI}, csPin{cs}, useSPI{spi}, mosiPin{mosi}, misoPin{miso}, sckPin{sck} {}
            
        ADXL345_WE(SPIClass *s, int cs, bool spi, int mosi = 999, int miso = 999, int sck = 999)
            :  _spi{s}, csPin{cs}, useSPI{spi}, mosiPin{mosi}, misoPin{miso}, sckPin{sck} {}
#endif
        /* registers */
        
        static constexpr uint8_t ADXL345_DEVID            {0x00};
        static constexpr uint8_t ADXL345_THRESH_TAP       {0x1D}; 
        static constexpr uint8_t ADXL345_OFSX             {0x1E};
        static constexpr uint8_t ADXL345_OFSY             {0x1F};
        static constexpr uint8_t ADXL345_OFSZ             {0x20};
        static constexpr uint8_t ADXL345_DUR              {0x21};
        static constexpr uint8_t ADXL345_LATENT           {0x22};
        static constexpr uint8_t ADXL345_WINDOW           {0x23};
        static constexpr uint8_t ADXL345_THRESH_ACT       {0x24};
        static constexpr uint8_t ADXL345_THRESH_INACT     {0x25}; 
        static constexpr uint8_t ADXL345_TIME_INACT       {0x26};
        static constexpr uint8_t ADXL345_ACT_INACT_CTL    {0x27};
        static constexpr uint8_t ADXL345_THRESH_FF        {0x28};
        static constexpr uint8_t ADXL345_TIME_FF          {0x29};
        static constexpr uint8_t ADXL345_TAP_AXES         {0x2A};
        static constexpr uint8_t ADXL345_ACT_TAP_STATUS   {0x2B};
        static constexpr uint8_t ADXL345_BW_RATE          {0x2C};
        static constexpr uint8_t ADXL345_POWER_CTL        {0x2D};
        static constexpr uint8_t ADXL345_INT_ENABLE       {0x2E};
        static constexpr uint8_t ADXL345_INT_MAP          {0x2F};
        static constexpr uint8_t ADXL345_INT_SOURCE       {0x30};
        //TODO ANALYZE
        static constexpr uint8_t ADXL345_DATA_FORMAT      {0x31};
        static constexpr uint8_t ADXL345_DATAX0           {0x32};
        static constexpr uint8_t ADXL345_DATAX1           {0x33};
        static constexpr uint8_t ADXL345_DATAY0           {0x34};
        static constexpr uint8_t ADXL345_DATAY1           {0x35};
        static constexpr uint8_t ADXL345_DATAZ0           {0x36};
        static constexpr uint8_t ADXL345_DATAZ1           {0x37};
        static constexpr uint8_t ADXL345_FIFO_CTL         {0x38};
        static constexpr uint8_t ADXL345_FIFO_STATUS      {0x39};

        /* Register bits */
        
        static constexpr uint8_t ADXL345_FULL_RES         {0x03};
        static constexpr uint8_t ADXL345_SUPPRESS         {0x03};
        static constexpr uint8_t ADXL345_LOW_POWER        {0x04};
        
        /* Other */
        
        static constexpr float MILLI_G_PER_LSB             {3.9};
        static constexpr float UNITS_PER_G              {256.41};// = 1/0.0039
    
        /* Basic settings */
        
        bool init();
#ifdef USE_I2C
        void setWire(TwoWire *w);
        void setAddr(uint8_t addr);
#endif
#ifdef USE_SPI
        void setSPIClockSpeed(unsigned long clock);
#endif
        void setCorrFactors(float xMin, float xMax, float yMin, float yMax, float zMin, float zMax);
        void setDataRate(adxl345_dataRate rate);
        adxl345_dataRate getDataRate();
        String getDataRateAsString();
        uint8_t getPowerCtlReg();
        void setRange(adxl345_range range);
        adxl345_range getRange();
        void setFullRes(boolean full);
        String getRangeAsString();
        
        /* x,y,z results */
            
        xyzFloat getRawValues();
        xyzFloat getCorrectedRawValues();
        xyzFloat getGValues();
        float getVectorG();
        float getVectorG(xyzFloat *gVal);
        float getVectorG(float *x, float *y, float *z);
       // float getImpact(float *x, float *y, float *z);
        xyzFloat getAngles();
        xyzFloat getCorrAngles();
            
        /* Angles and Orientation */ 
        
        void measureAngleOffsets();
        xyzFloat getAngleOffsets();
        void setAngleOffsets(xyzFloat aos);
        adxl345_orientation getOrientation();
        String getOrientationAsString();
        float getPitch();
        float getRoll();
        
        /* Power, Sleep, Standby */ 
        
        void setMeasureMode(bool measure);
        void setSleep(bool sleep, adxl345_wUpFreq freq);
        void setSleep(bool sleep);
        void setAutoSleep(bool autoSleep, adxl345_wUpFreq freq);
        void setAutoSleep(bool autoSleep);
        bool isAsleep();
        void setLowPower(bool lowpwr);
        bool isLowPower();
        
        /* Interrupts */
        
        void setInterrupt(adxl345_int type, uint8_t pin);
        void setInterruptPolarity(uint8_t pol);
        void deleteInterrupt(adxl345_int type);
        uint8_t readAndClearInterrupts();
        bool checkInterrupt(uint8_t source, adxl345_int type);
        void setLinkBit(bool link);

        /** The parameters of the setFreeFallThresholds function are:
             - g threshold - do not choose a parameter which is too low. 0.3 - 0.6 g is fine.
             - time threshold in ms, maximum is 1275. Recommended is 100 - 350;
            If time threshold is too low, vibrations can be detected as free fall.
        */
        void setFreeFallThresholds(float threshold, float fftime);
        void setActivityParameters(adxl345_dcAcMode mode, adxl345_actTapSet axes, float threshold);
        void setInactivityParameters(adxl345_dcAcMode mode, adxl345_actTapSet axes, float threshold, uint8_t inactTime);
        void setGeneralTapParameters(adxl345_actTapSet axes, float threshold, float duration, float latent);

        /* For double tap detection additional parameters have to be set:
            1. Suppress bit: if the bit is set, a spike over the threshold during the latency time will invalidate
                the double tap. You find graphs that explain this better than words in the data sheet. If the bit is 
                not set, the tap during latency time will be ignored. Another tap in the time window (see 2) will lead 
                to double tap detection. 
            2. Time window in milliseconds: starts after latency time. The second tap must occur during the time window
                period, otherwise it is a single tap (if single tap is active). Maximum window is 318 ms.
        */
        void setAdditionalDoubleTapParameters(bool suppress, float window);
        uint8_t getActTapStatus();
        String getActTapStatusAsString();
        uint8_t getActTapStatusAsValue();

        /** FIFO */
       
        /** The following two FIFO parameters need to be set:
         *    1. Trigger Bit: the trigger is an interrupt at INT1 or INT2
         *       ADXL345_TRIGGER_INT_1 - Trigger is an interrupt at INT1
         *       ADXL345_TRIGGER_INT_2 - Trigger is an interrupt at INT2 
         *      @note not relevant for: 
         *             ADXL345_STREAM 
         *             ADXL345_FIFO
         *             ADXL345_TRIGGER
         *    2. FIFO samples (max 32). Defines the size of the FIFO. @note One sample is an x,y,z triple.
         */
        void setFifoParameters(adxl345_triggerInt intNumber, uint8_t samples);
    
        /** Choose the following FIFO modes:
        *    ADXL345_FIFO     -  you choose the start, ends when FIFO is full (at defined limit)
        *    ADXL345_STREAM   -  FIFO always filled with new data, old data replaced if FIFO is full; you choose the stop
        *    ADXL345_TRIGGER  -  FIFO always filled up to 32 samples; when the trigger event occurs only defined number of samples
        *                        is kept in the FIFO and further samples are taken after the event until FIFO is full again. 
        *    ADXL345_BYPASS   -  no FIFO
        */  
        void setFifoMode(adxl345_fifoMode mode);
        uint8_t getFifoStatus();
        void resetTrigger();
       
    protected:
#ifdef USE_I2C
        TwoWire *_wire = nullptr;
        uint8_t i2cAddress=ADXL345_I2CADDR;
#endif
#ifdef USE_SPI
        SPIClass *_spi = nullptr;
        SPISettings mySPISettings;
#endif
        uint8_t regVal;   // intermediate storage of register values
        xyzFloat offsetVal;
        xyzFloat angleOffsetVal;
        xyzFloat corrFact;
        
        bool useSPI; 
 #ifdef USE_SPI   
        int csPin;
        int mosiPin;
        int misoPin;
        int sckPin;  
 #endif   
        float rangeFactor;
        uint8_t writeRegister(uint8_t reg, uint8_t val);
        uint8_t readRegister8(uint8_t reg);
        void readMultipleRegisters(uint8_t reg, uint8_t count, uint8_t *buf);
        bool adxl345_lowRes;
};

#endif


