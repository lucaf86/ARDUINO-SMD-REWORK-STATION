/*
 * Hot air gun controller based on atmega328 IC
 * Version 2.8b
 * Released Mar 18, 2021
 */

//to test: thermocouple error check should work also when not ON. OK done!
//todo: Keep temp must avoid resetting back to another state after an abort
//todo: draw current temp with different color if near, below, higher that set temp
//todo: evaluate if possile to join fanTune and PID cfg menu in one
//todo: if possible try to avoid AC synch error on power off 
//todo: better handling of eeprom save to avoid frequent save or re-save the same config. Fan cfg and preset temp/fanSpeed alway rewritten in eeprom now
//done: add watchdog
//todo: make editable the min fan cooling speed
//todo: two set of pid coeff, one aggressive for fast heating and another to keep temp
//dobe: more sensitive cradle detection

//done: tune Encoder Time in ms to change encoder quickly (see fast_timeoutn library) 
//              todo: still not ok, try to ignore fast move and set 1 increment in any case or change library!


#include <avr/io.h>
#include <avr/interrupt.h>
#include <Wire.h>
#include <CommonControls.h>
#include <EEPROM.h>
#include <SPI.h>
#include <mdPushButton.h>
#include <avr/wdt.h>

//free pin 10, A1, (A6, A7 if usin g arduino nano)

// LCD pins define in User_setup.h
//#define TFT_CS  A5 //SS  // Chip select control pin
//#define TFT_DC  A4 //10  // Data Command control pin
//#define TFT_RST -1  // Reset pin (could connect to Arduino RESET pin)

/* for sw spi on Max6675 pin could be changed as follow:
TFT_CS  A5 --> 10
MAXCS   A3
MAX_CLK  13 --> A5 (or A1)
MAX_SO   12 --> A1 (or A5)
*/

#include <TFT_ST7735.h>
#include <MAX6675.h>

/* DEBUG Defines*/
//#define DISABLE_AC_CHECK
//#define SIMULATE_SYNC_AC
//#define DISABLE_K_TH_CHECK 
//#define DISABLE_FAN_CHECK
//#define EMULATE_TEMP
//#define DEBUG_PRINT_ENABLE

#ifdef DEBUG_PRINT_ENABLE
#define printDebug(a) Serial.print(a)
#define printDebugLn(a) Serial.println(a)
#else
#define printDebug(a)
#define printDebugLn(a)
#endif
/****************/

#define PID_SCREEN

#define digital_write    digitalWrite
#define pin_mode         pinMode

#define SLOW_D9_PWM_FREQUENCY
#define SCREEN_WIDTH             ST7735_TFTHEIGHT //160;                                          //  display width, in pixels
#define SCREEN_HEIGHT            ST7735_TFTWIDTH //128;                                          //  display height, in pixels
#define LCD_BACKGROUND_COLOR     TFT_BLACK // 0x5AEB // grey color code
#define LCD_TEXT_COLOR           TFT_WHITE
#define TFT_TEXT_HIGHLIGHT_COLOR TFT_GREEN
#define LCD_LINE_COLOR           TFT_LIGHTGREY
#define LCD_DEFAULT_FONT         4
#define LCD_SMALL_FONT           LCD_DEFAULT_FONT
#define LCD_MENU_FONT            LCD_DEFAULT_FONT
#define LCD_BIG_FONT             6
#define LCD_STATUS_OFFSET        4
#define X_PIXEL_OFFSET          10
#define Y_PIXEL_OFFSET           8
#define X_PIXEL(x)              (X_PIXEL_OFFSET + x)
#define Y_PIXEL(y)              (Y_PIXEL_OFFSET + y)

//#define _FONT_WIDTH            8 
//#define _FONT_HEIGHT           8  
//#define setCharCursor(x, y) setCursor((x)*_FONT_WIDTH, (y+1)*_FONT_HEIGHT)

#define TEMP_MIN_C                100            // min configurable temperature setpoint
#define TEMP_MAX_C                450            // Max configurable temperature setpoint
#define TEMP_AMB_C                 25            // Ambient temperature
#define TEMP_GUN_COLD             40            // The temperature to consider the cold iron
#define TEMP_GUN_COLD_HYSTERESIS  20            // delta T applied to temp_gun_cold before switcing fan on again
//const uint16_t temp_tip[3]  = {200, 300, 400};                     // Temperature reference points for calibration
#define min_working_fan  45                    // Minimal possible fan speed in 0-100% step
#define max_working_fan  100
#define MAX_COOL_FAN     96

#define PID_KP           638
#define PID_KI           196
#define PID_KD             1

#define H_LENGTH_POWER 16  // History queue for pid power calculation /* must be greater that 3! */
#define H_LENGTH_TEMP 8    // History queue for temperature read accumulation /* must be greater that 3! */
#define H_LENGTH_FAN  4    // History queue for fan current read accumulation /* must be greater that 3! */

#define REED_ON_TIME_DELAY    500
#define REED_OFF_TIME_DELAY   700//1000

#define BUZZER_TYPE_ACTIVE 0                                        // 1 - Active buzzer beeps when +5v supplied to it
                                                                    // 0 - passive buzzer beeps when a tone signal is supplied

#define AC_SYNC_PIN        2                                        // Outlet 220 v synchronization pin. Do not change!
#define HOT_GUN_PIN        7                                        // Hot gun heater management pin
#define FAN_GUN_PIN        9                                        // Hot gun fan management pin. DO NOT CHANGE due to PWM control!
#define FAN_GUN_SENS_PIN  A0                                        // Hot gun fan checking pin
#define AC_RELAY_PIN      A2

#define R_MAIN_PIN         3                                        // Rotary encoder main pin. Do not change!
#define R_SECD_PIN         5                                        // Rotary encoder secondary pin
#define R_BUTN_PIN         4                                        // Rotary encoder button pin

#define REED_SW_PIN        8                                        // Reed switch pin
#define BUZZER_PIN         6                                        // Buzzer pin

#define MAXCS             A3

const char failText[] = {"= FAILED ="};

#ifdef DISABLE_AC_CHECK
  #define TC_ERROR_TOLLERANCE 999999
  #define FAN_ERROR_TOLLERANCE 999999
#else
  #define TC_ERROR_TOLLERANCE 5     // maximum number of consecutive temperature error reading allowed
  #define FAN_ERROR_TOLLERANCE 5
#endif

MAX6675 thermocouple(MAXCS);

/* Arduino Software reset function */
void(* resetFunc) (void) = 0; //declare arduino software reset function @ address 0


//debug //#define TFT_print(stringBuff, xPos, yPos, font)   TFT_ST7735::drawString(stringBuff, (xPos)*_FONT_WIDTH, (yPos+1)*_FONT_HEIGHT, font)
#define TFT_print(stringBuff, xPos, yPos, font)   TFT_ST7735::drawString(stringBuff, xPos, yPos, font)

//#define TFT_print(stringBuff, xPos, yPos, font)     TFT_ST7735::setTextFont(font); \
                                                    TFT_ST7735::setCursor(xPos, yPos); \
                                                    print(stringBuff)

#define TFT_centrePrint(stringBuff, xPos, yPos, font)     TFT_ST7735::drawCentreString(stringBuff, xPos, yPos, font)
//#define TFT_centrePrint(stringBuff, xPos, yPos, font)     TFT_print(stringBuff, xPos - (TFT_ST7735::textWidth(stringBuff, font) / 2), yPos, font) // Will not work if TFT_print uses TFT_ST7735::drawString

#define TFT_rightPrint(stringBuff, xPos, yPos, font)     TFT_ST7735::drawRightString(stringBuff, xPos, yPos, font)
//#define TFT_rightPrint(stringBuff, xPos, yPos, font)      TFT_print(stringBuff, xPos - TFT_ST7735::textWidth(stringBuff, font), yPos, font) // Will not work if TFT_print uses TFT_ST7735::drawString

//#define TFT_printNumber(num, xPos, yPos, font)     TFT_ST7735::drawNumber(num, xPos, yPos, font)
#define TFT_printNumber(num, xPos, yPos, font)     TFT_ST7735::setTextFont(font); \
                                                   TFT_ST7735::setCursor(xPos, yPos); \
                                                   print(num)

#define TFT_textPadding(p)    TFT_ST7735::setTextPadding(p)
#define TFT_resetTextPadding()  TFT_ST7735::setTextPadding(0)

//------------------------------------------ Configuration data ------------------------------------------------
/* Config record in the EEPROM has the following format:
 * uint32_t ID                           each time increment by 1
 * struct cfg                            config data, 8 bytes
 * byte CRC                              the checksum
*/
struct cfg {
//    uint32_t    calibration;                                                // Packed calibration data by three temperature points
    uint16_t    temp;                                                       // The preset temperature of the IRON in internal units
    uint8_t     fan;                                                        // The preset fan speed 0 - 255
    uint8_t     off_timeout;                                                // Automatic switch-off timeout
    uint8_t     minFanSpeedSens;                                            // min adc value to consider fan working
    uint8_t     maxFanSpeedSens;                                            // max adc value to consider fan working
    uint16_t    kp;                                                         // The PID algorithm coefficients
    uint16_t    ki;                                                         // The PID algorithm coefficients
    uint16_t    kd;                                                         // The PID algorithm coefficients
};

class CONFIG {
    public:
        CONFIG() {
            can_write     = false;
            buffRecords   = 0;
            rAddr = wAddr = 0;
            eLength       = 0;
            nextRecID     = 0;
            uint8_t rs = sizeof(struct cfg) + 5;                             // The total config record size
            // Select appropriate record size; The record size should be power of 2, i.e. 8, 16, 32, 64, ... bytes
            for (record_size = 8; record_size < rs; record_size <<= 1);
        }
        void clear();
        void init();
        bool load(void);
        void getConfig(struct cfg &Cfg);                                    // Copy config structure from this class
        void updateConfig(struct cfg &Cfg);                                 // Copy updated config into this class
        bool save(void);                                                    // Save current config copy to the EEPROM
        bool saveConfig(struct cfg &Cfg);                                   // write updated config into the EEPROM

    protected:
        struct   cfg Config;

    private:
        bool     readRecord(uint16_t addr, uint32_t &recID);
        bool     can_write;                                                 // The flag indicates that data can be saved
        uint8_t  buffRecords;                                               // Number of the records in the outpt buffer
        uint16_t rAddr;                                                     // Address of thecorrect record in EEPROM to be read
        uint16_t wAddr;                                                     // Address in the EEPROM to start write new record
        uint16_t eLength;                                                   // Length of the EEPROM, depends on arduino model
        uint32_t nextRecID;                                                 // next record ID
        uint8_t  record_size;                                               // The size of one record in bytes
};

// clear all eeprom
void CONFIG::clear(void) {
    for (uint16_t i = 0 ; i < EEPROM.length() ; i++) {
    EEPROM.write(i, 0);
  }
  return;
}

 // Read the records until the last one, point wAddr (write address) after the last record
void CONFIG::init(void) {
    eLength = EEPROM.length();
    uint32_t recID;
    uint32_t minRecID = 0xffffffff;
    uint16_t minRecAddr = 0;
    uint32_t maxRecID = 0;
    uint16_t maxRecAddr = 0;
    uint8_t  records = 0;

    nextRecID = 0;

    // read all the records in the EEPROM find min and max record ID
    for (uint16_t addr = 0; addr < eLength; addr += record_size) {
        if (readRecord(addr, recID)) {
            ++records;
            if (minRecID > recID) {
                minRecID = recID;
                minRecAddr = addr;
            }
            if (maxRecID < recID) {
                maxRecID = recID;
                maxRecAddr = addr;
            }
        } else {
            break;
        }
    }

    if (records == 0) {
        wAddr = rAddr = 0;
        can_write = true;
        return;
    }

    rAddr = maxRecAddr;
    if (records < (eLength / record_size)) {                                // The EEPROM is not full
        wAddr = rAddr + record_size;
        if (wAddr > eLength) wAddr = 0;
    } else {
        wAddr = minRecAddr;
    }
    can_write = true;
}

void CONFIG::getConfig(struct cfg &Cfg) {
    memcpy(&Cfg, &Config, sizeof(struct cfg));
}

void CONFIG::updateConfig(struct cfg &Cfg) {
    memcpy(&Config, &Cfg, sizeof(struct cfg));
}

bool CONFIG::saveConfig(struct cfg &Cfg) {
    updateConfig(Cfg);
    return save();                                                          // Save new data into the EEPROM
}

bool CONFIG::save(void) {
    if (!can_write) return can_write;
    if (nextRecID == 0) nextRecID = 1;

    uint16_t startWrite = wAddr;
    uint32_t nxt = nextRecID;
    uint8_t summ = 0;
    for (uint8_t i = 0; i < 4; ++i) {
        EEPROM.write(startWrite++, nxt & 0xff);
        summ <<=2; summ += nxt;
        nxt >>= 8;
    }
    uint8_t* p = (byte *)&Config;
    for (uint8_t i = 0; i < sizeof(struct cfg); ++i) {
        summ <<= 2; summ += p[i];
        EEPROM.write(startWrite++, p[i]);
    }
    summ ++;                                                                // To avoid empty records
    EEPROM.write(wAddr+record_size-1, summ);

    rAddr = wAddr;
    wAddr += record_size;
    if (wAddr > EEPROM.length()) wAddr = 0;
    nextRecID ++;                                                           // Get ready to write next record
    return true;
}

bool CONFIG::load(void) {
    bool is_valid = readRecord(rAddr, nextRecID);
    nextRecID ++;
    return is_valid;
}

bool CONFIG::readRecord(uint16_t addr, uint32_t &recID) {
    uint8_t Buff[record_size];

    for (uint8_t i = 0; i < record_size; ++i)
        Buff[i] = EEPROM.read(addr+i);

    uint8_t summ = 0;
    for (byte i = 0; i < sizeof(struct cfg) + 4; ++i) {
        summ <<= 2; summ += Buff[i];
    }
    summ ++;                                                                // To avoid empty fields
    if (summ == Buff[record_size-1]) {                                      // Checksumm is correct
        uint32_t ts = 0;
        for (char i = 3; i >= 0; --i) {
            ts <<= 8;
            ts |= Buff[byte(i)];
        }
        recID = ts;
        memcpy(&Config, &Buff[4], sizeof(struct cfg));
        return true;
    }
    return false;
}

//------------------------------------------ class HOT GUN CONFIG ----------------------------------------------
class HOTGUN_CFG : public CONFIG {
    public:
        HOTGUN_CFG()                                                        { }
        void     clear(void);
        bool     init(void);
        uint16_t tempPreset(void);                                          // The preset temperature 
        uint8_t  fanPreset(void);                                           // The preset fan speed 0 - 255
        uint16_t tempInternal(uint16_t temp);                               // Translate the human readable temperature into internal value
        //uint16_t tempHuman(uint16_t temp);                                  // Translate temperature from internal units to the Celsius
        void     save(uint16_t temp, uint8_t fanSpeed);                     // Save preset temperature in the internal units and fan speed
        //void     applyCalibrationData(uint16_t tip[3]);
        //void     getCalibrationData(uint16_t tip[3]);
        //void     saveCalibrationData(uint16_t tip[3]);
        //void     applyFanData(uint16_t fan_min, uint16_t fan_max);
        void     saveFanData(uint16_t fan_min, uint16_t fan_max, bool Write);
        void     savePidData(uint16_t kp, uint16_t ki, uint16_t kd, bool Write);
        void     setDefaults(bool Write);                                   // Set default parameter values if failed to load data from EEPROM
        uint8_t  getMinFanSpeedSens(void);
        uint8_t  getMaxFanSpeedSens(void);
        uint16_t getPidKp(void);
        uint16_t getPidKi(void);
        uint16_t getPidKd(void);
    private:
        //uint16_t t_tip[3];
        //uint16_t minFanSpeedSens;
        //uint16_t maxFanSpeedSens;
        //const   uint16_t def_tip[3] = {200, 300, 400};//{55, 445, 900};//{587, 751, 850};                      // Default values of internal sensor readings at reference temperatures
        const   uint16_t min_temp  = TEMP_MIN_C;
        const   uint16_t max_temp  = TEMP_MAX_C;
        const   uint16_t def_temp  = 190;                                   // Default preset temperature
        const   uint8_t  def_fan   = 90;                                    // Default preset fan speed 0 - 100%
        //const   uint16_t ambient_temp = TEMP_AMB_C;
        //const   uint16_t ambient_tempC=TEMP_AMB_C;
        const   uint16_t def_minFanSpeedSens = 105;                         // fan min current threshold in 0-255 pwm step
        const   uint16_t def_maxFanSpeedSens = 230;                         // fan MAX current threshold in 0-255 pwm step
        const   uint16_t def_pid_kp = PID_KP;
        const   uint16_t def_pid_ki = PID_KI;
        const   uint16_t def_pid_kd = PID_KD;
};

void HOTGUN_CFG::clear(void) {
    CONFIG::clear();
    return;
}

bool HOTGUN_CFG::init(void) {
    bool cfgLoad; 
    CONFIG::init();
    cfgLoad = CONFIG::load();
    if (!cfgLoad) setDefaults(false);                                // If failed to load the data from EEPROM, initialize the config data with the default values
    //uint32_t   cd = Config.calibration;
    //t_tip[0] = cd & 0x3FF; cd >>= 10;                                       // 10 bits per calibration parameter, because the ADC readings are 10 bits
    //t_tip[1] = cd & 0x3FF; cd >>= 10;
    //t_tip[2] = cd & 0x3FF;
    //// Check the tip calibration is correct
    //if ((t_tip[0] >= t_tip[1]) || (t_tip[1] >= t_tip[2])) {
    //    setDefaults(false);
    //    for (uint8_t i = 0; i < 3; ++i)
    //        t_tip[i] = def_tip[i];
    //}
//    minFanSpeedSens = Config.minFanSpeedSens;
//    maxFanSpeedSens = Config.maxFanSpeedSens;
    return cfgLoad;
}

uint16_t HOTGUN_CFG::tempPreset(void) {
    return Config.temp;
}

uint8_t HOTGUN_CFG::fanPreset(void) {
    return Config.fan;
}

uint16_t HOTGUN_CFG::tempInternal(uint16_t t) {                             // temperature in °C
    t = constrain(t, min_temp, max_temp);
    return t;
}

// Thanslate temperature from internal units to the human readable value (Celsius or Fahrenheit)
/*
uint16_t HOTGUN_CFG::tempHuman(uint16_t temp) {
    return temp;

    uint16_t tempH = 0;
    if (temp <= ambient_temp) {
        tempH = ambient_tempC;
    } else if (temp < t_tip[0]) {
        tempH = map(temp, ambient_temp, t_tip[0], ambient_tempC, temp_tip[0]);
    } else if (temp <= t_tip[1]) {
        tempH = map(temp, t_tip[0], t_tip[1], temp_tip[0], temp_tip[1]);
    } else if (temp <= t_tip[2]) {
        tempH = map(temp, t_tip[1], t_tip[2], temp_tip[1], temp_tip[2]);
    } else {
        tempH = map(temp, t_tip[0], t_tip[2], temp_tip[0], temp_tip[2]);
    }
    return tempH;
}
*/

void HOTGUN_CFG::save(uint16_t temp, uint8_t fanSpeed) {
    Config.temp        = constrain(temp, min_temp, max_temp);
    Config.fan         = fanSpeed;
    CONFIG::save();                                                         // Save new data into the EEPROM
}
/*
void HOTGUN_CFG::applyCalibrationData(uint16_t tip[3]) {
    if (tip[0] < ambient_temp) {
        uint16_t t = ambient_temp + tip[1];
        tip[0] = t >> 1;
    }
    t_tip[0] = tip[0];
    t_tip[1] = tip[1];
    if (tip[2] > max_temp) tip[2] = max_temp;
    t_tip[2] = tip[2];
}

void HOTGUN_CFG::getCalibrationData(uint16_t tip[3]) {
    tip[0] = t_tip[0];
    tip[1] = t_tip[1];
    tip[2] = t_tip[2];
}

void HOTGUN_CFG::saveCalibrationData(uint16_t tip[3]) {
    if (tip[2] > max_temp) tip[2] = max_temp;
    uint32_t cd = tip[2] & 0x3FF; cd <<= 10;                                // Pack tip calibration data in one 32-bit word: 10-bits per value
    cd |= tip[1] & 0x3FF; cd <<= 10;
    cd |= tip[0];
    Config.calibration = cd;
    t_tip[0] = tip[0];
    t_tip[1] = tip[1];
    t_tip[2] = tip[2];
}
*/
/*
void HOTGUN_CFG::applyFanData(uint16_t fan_min, uint16_t fan_max) {
    Config.minFanSpeedSens = fan_min;
    Config.maxFanSpeedSens = fan_max;
}
*/
void HOTGUN_CFG::saveFanData(uint16_t fan_min, uint16_t fan_max, bool Write) {
    Config.minFanSpeedSens = fan_min;
    Config.maxFanSpeedSens = fan_max;
    if (Write) {
        CONFIG::save();
    }
}

void HOTGUN_CFG::savePidData(uint16_t kp, uint16_t ki, uint16_t kd, bool Write) {
    Config.kp = kp;
    Config.ki = ki;
    Config.kd = kd;
    if (Write) {
        CONFIG::save();
    }
}

void HOTGUN_CFG::setDefaults(bool Write) {
 /*   uint32_t c = def_tip[2] & 0x3FF; c <<= 10;
    c |= def_tip[1] & 0x3FF;         c <<= 10;
    c |= def_tip[0] & 0x3FF;
    Config.calibration = c;*/
    Config.temp        = def_temp;
    Config.fan         = def_fan;
    Config.minFanSpeedSens = def_minFanSpeedSens;
    Config.maxFanSpeedSens = def_maxFanSpeedSens;
    Config.kp = def_pid_kp;
    Config.ki = def_pid_ki;
    Config.kd = def_pid_kd;
    if (Write) {
        CONFIG::save();
    }
}

uint8_t HOTGUN_CFG::getMinFanSpeedSens(void) {
    //return minFanSpeedSens;
    return Config.minFanSpeedSens;
}

uint8_t HOTGUN_CFG::getMaxFanSpeedSens(void) {
    return Config.maxFanSpeedSens;
}

uint16_t HOTGUN_CFG::getPidKp(void) {
    return Config.kp;
}

uint16_t HOTGUN_CFG::getPidKi(void) {
    return Config.ki;
}

uint16_t HOTGUN_CFG::getPidKd(void) {
    return Config.kd;
}

//------------------------------------------ class BUZZER ------------------------------------------------------
class BUZZER {
  public:
        BUZZER(byte buzzerP)    { buzzer_pin = buzzerP; }
    void init(void);
    void shortBeep(void);
    void lowBeep(void);
    void doubleBeep(void);
    void failedBeep(void);
  private:
    byte buzzer_pin;
};

void BUZZER::init(void) {
    pin_mode(buzzer_pin, OUTPUT);
    #if (BUZZER_TYPE_ACTIVE) 
    {
        digital_write(buzzer_pin, LOW);
    } 
    #else 
    {
        noTone(buzzer_pin);
    }
    #endif
}

void BUZZER::shortBeep(void) {
    #if (BUZZER_TYPE_ACTIVE) 
    {
       digital_write(buzzer_pin, HIGH);
       delay(80);
       digital_write(buzzer_pin, LOW);
    } 
    #else
    {
        tone(buzzer_pin, 3520, 160);
    }
    #endif
}

void BUZZER::lowBeep(void) {
    #if (BUZZER_TYPE_ACTIVE) 
    {
        digital_write(buzzer_pin, HIGH);
        delay(160);
        digital_write(buzzer_pin, LOW);
    } 
    #else 
    {
        tone(buzzer_pin,  880, 160);
    }
    #endif
}

void BUZZER::doubleBeep(void) {
    #if (BUZZER_TYPE_ACTIVE) 
    {
        digital_write(buzzer_pin, HIGH);
        delay(160);
        digital_write(buzzer_pin, LOW);
        delay(150);
        digital_write(buzzer_pin, HIGH);
        delay(160);
        digital_write(buzzer_pin, LOW);
    } 
    #else 
    {
        tone(buzzer_pin, 3520, 160);
        delay(300);
        tone(buzzer_pin, 3520, 160);
    }
    #endif
}

void BUZZER::failedBeep(void) {
    #if (BUZZER_TYPE_ACTIVE) 
    {
        digital_write(buzzer_pin, HIGH);
        delay(170);
        digital_write(buzzer_pin, LOW);
        delay(10);
        digital_write(buzzer_pin, HIGH);
        delay(80);
        digital_write(buzzer_pin, LOW);
        delay(100);
        digital_write(buzzer_pin, HIGH);
        delay(80);
        digital_write(buzzer_pin, LOW);
    } 
    #else
    {
        tone(buzzer_pin, 3520, 160);
        delay(170);
        tone(buzzer_pin,  880, 250);
        delay(260);
        tone(buzzer_pin, 3520, 160);
    }
    #endif
}

//------------------------------------------ class lcd DSPLay for soldering IRON -----------------------------
const uint8_t bmDegree[5] PROGMEM = {0b00111000,
                                     0b01000100,
                                     0b01000100,
                                     0b01000100,
                                     0b00111000};
 const uint8_t bmTemperature[20] PROGMEM ={0b00010000,
                                           0b00101000,
                                           0b01101000,
                                           0b00101000,
                                           0b01101000,
                                           0b00101000,
                                           0b00101000,
                                           0b00101000,
                                           0b00101000,
                                           0b00111000,
                                           0b00111000,
                                           0b00111000,
                                           0b00111000,
                                           0b00111000,
                                           0b01111100,
                                           0b11111110,
                                           0b11111110,
                                           0b11111110,
                                           0b01111100,
                                           0b00111000};
/*static const uint8_t bmFan[4][32] PROGMEM = {
   {0x07, 0x00, 0x07, 0x00, 0x03, 0x80, 0x03, 0x80, 0x01, 0x80, 0x01, 0x80,
    0x01, 0x80, 0x03, 0xc0, 0x03, 0xc0, 0x0f, 0x70, 0x1c, 0x3b, 0x38, 0x1f,
    0x38, 0x0f, 0x70, 0x00, 0x38, 0x00, 0x00, 0x00 },
   {0x00, 0xc0, 0x00, 0xe0, 0x00, 0x70, 0x00, 0x70, 0x00, 0x60, 0x00, 0xc0,
    0x01, 0xc0, 0x2b, 0x80, 0x7f, 0xc0, 0xf1, 0x80, 0xe0, 0xc0, 0x00, 0xe0,
    0x00, 0x70, 0x00, 0x78, 0x00, 0x38, 0x00, 0x18 },
   {0x00, 0x00, 0x00, 0x00, 0x00, 0x06, 0x00, 0x0e, 0x20, 0x1e, 0x78, 0x38,
    0xfd, 0x70, 0xd7, 0xe0, 0x03, 0xc0, 0x01, 0x80, 0x01, 0x00, 0x01, 0x80,
    0x03, 0x80, 0x03, 0x80, 0x01, 0xc0, 0x00, 0xe0 },
   {0x00, 0x00, 0x20, 0x00, 0xf8, 0x00, 0xfc, 0x00, 0xde, 0x00, 0x06, 0x03,
    0x03, 0x8f, 0x01, 0xff, 0x03, 0xfe, 0x03, 0x80, 0x06, 0x00, 0x0e, 0x00,
    0x1c, 0x00, 0x3c, 0x00, 0x3e, 0x00, 0x0e, 0x00 }};*/
#define FAN_BM_WIDTH 16
#define FAN_BM_HEIGHT 16
 static const uint8_t bmFan[FAN_BM_WIDTH/8 * FAN_BM_HEIGHT] PROGMEM = { 0b00000111, 0b11000000,
                                                                        0b00000111, 0b11100000,
                                                                        0b00000011, 0b11110000,
                                                                        0b00000011, 0b11110000,
                                                                        0b00000001, 0b11100000,
                                                                        0b00000001, 0b11000000,
                                                                        0b00000001, 0b11000000,
                                                                        0b00100011, 0b11000000,
                                                                        0b01111111, 0b11000000,
                                                                        0b11111111, 0b11110000,
                                                                        0b11111100, 0b11111010,
                                                                        0b11111000, 0b11111110,
                                                                        0b01111000, 0b01111110,
                                                                        0b01110000, 0b01111100,
                                                                        0b00110000, 0b00111100,
                                                                        0b00000000, 0b00011000 };   

static const uint8_t bmPower[15] PROGMEM = { 0b00011111,
                                             0b00011111,
                                             0b00111111, 
                                             0b00111110, 
                                             0b01111100, 
                                             0b01110000, 
                                             0b11111110, 
                                             0b11111111, 
                                             0b11111111, 
                                             0b00001111, 
                                             0b00011110, 
                                             0b00011100,
                                             0b00111000, 
                                             0b00110000, 
                                             0b01100000};

#define DSPL_MAX_BUFF_SIZE 20

class DSPL : protected TFT_ST7735 {
    public:
        DSPL(void) : TFT_ST7735(SCREEN_WIDTH, SCREEN_HEIGHT) { }
        void    init(void);
        void    clear(uint16_t bg_color = LCD_BACKGROUND_COLOR)               { TFT_ST7735::fillScreen(bg_color); TFT_ST7735::setCursor(0, 0);}
        void    backgroundInit(void);
        void    tSet(uint16_t t, bool isActive = false);                      // Show the preset temperature
        void    tCurr(uint16_t t);                                          // Show the current temperature
 //       void    tInternal(uint16_t t);                                      // Show the current temperature in internal units
  //      void    tReal(uint16_t t);                                          // Show the real temperature in Celsius in calibrate mode
        void    fanSpeed(uint8_t s, bool isActive = false);                                        // Show the fan speed
        void    appliedPower(uint8_t p, bool show_zero = true);             // Show applied power (%)
        void    setupMode(uint8_t mode);
        void    msgON(uint16_t color = TFT_RED);                                                // Show message: "ON"
        void    msgOFF(uint16_t color = LCD_TEXT_COLOR);
        void    msgReady(void);
        void    msgCold(void);
        void    msgFail(void);                                              // Show 'Fail' message
 //       void    msgTune(void);                                              // Show 'Tune' message
 //       void    msgTip(uint16_t tip0, uint16_t tip1, uint16_t tip2);
        void    msgFanTune(uint8_t modeSel, uint8_t sel, uint16_t min, uint16_t max, uint16_t fan_current);
        void    msgPidTune(uint8_t mode, uint16_t kp, uint16_t ki, uint16_t kd, uint8_t kSel);
        void    message(const char *msg, const char *msg1);
    private:
        //const char    tempUnits[2] = {'C','\0'};
        char    buff[DSPL_MAX_BUFF_SIZE+1];
        char    buff1[DSPL_MAX_BUFF_SIZE+1];
        uint8_t textHeight;
};

void DSPL::init(void) {
    clear();
    TFT_ST7735::init();
    TFT_ST7735::setRotation(1);
    TFT_ST7735::fillScreen(LCD_BACKGROUND_COLOR);
    TFT_ST7735::setCursor(0, 0, LCD_DEFAULT_FONT); // Set "cursor" at top left corner of display (0,0) and select font 
    TFT_ST7735::setTextColor(LCD_TEXT_COLOR,LCD_BACKGROUND_COLOR);
    textHeight = TFT_ST7735::fontHeight(LCD_DEFAULT_FONT);
    TFT_centrePrint("Starting ...", SCREEN_WIDTH/2, SCREEN_HEIGHT/2 - 1 - textHeight/2 , LCD_SMALL_FONT);
    //TFT_ST7735::setTextFont(LCD_DEFAULT_FONT);
    //TFT_ST7735::setTextSize(1);
}
#define FAN_BITMAP_X_OFFSET 16
#define FAN_BITMAP_Y_OFFSET 26

void DSPL::backgroundInit(void){
    TFT_ST7735::fillScreen(LCD_BACKGROUND_COLOR);
    // draw preset temp icon
    TFT_ST7735::drawBitmap(X_PIXEL(0), Y_PIXEL(1), bmTemperature, 8, 20, TFT_RED);
    // draw fan icon
    TFT_ST7735::drawBitmap(X_PIXEL(FAN_BITMAP_X_OFFSET), SCREEN_HEIGHT - Y_PIXEL_OFFSET - FAN_BM_HEIGHT/*SCREEN_HEIGHT-1-Y_PIXEL_OFFSET-16 - FAN_BITMAP_Y_OFFSET*/, bmFan, FAN_BM_WIDTH, FAN_BM_HEIGHT, TFT_CYAN);
    // draw power icon 
    //TFT_ST7735::drawBitmap(SCREEN_WIDTH - X_PIXEL_OFFSET - 40, SCREEN_HEIGHT-1-Y_PIXEL_OFFSET-16, bmPower, 8, 15, TFT_YELLOW);
    TFT_ST7735::drawBitmap(X_PIXEL(15), SCREEN_HEIGHT/2 - 1 - 20, bmPower, 8, 15, TFT_YELLOW);
    // draw separation lines 
    TFT_ST7735::drawLine((0), Y_PIXEL(26), SCREEN_WIDTH-1, Y_PIXEL(26), LCD_LINE_COLOR);
    TFT_ST7735::drawLine((0), SCREEN_HEIGHT-1-Y_PIXEL_OFFSET-FAN_BITMAP_Y_OFFSET, SCREEN_WIDTH-1, SCREEN_HEIGHT-1-Y_PIXEL_OFFSET-26, LCD_LINE_COLOR);
    TFT_ST7735::drawLine(X_PIXEL(90), (0), X_PIXEL(90), Y_PIXEL(26), LCD_LINE_COLOR);
    TFT_ST7735::drawLine(X_PIXEL(40), Y_PIXEL(26), X_PIXEL(40), SCREEN_HEIGHT-1-Y_PIXEL_OFFSET-26, LCD_LINE_COLOR);
}

/*
void DSPL::msgTip(uint16_t tip0, uint16_t tip1, uint16_t tip2){
    char buff[17];
    setCharCursor(0, 4);
    sprintf(buff, "Cal: %3d %3d %3d", tip0, tip1, tip2);
    print(buff);
} 
 */
 
void DSPL::tSet(uint16_t t, bool isActive) {
    snprintf(buff, DSPL_MAX_BUFF_SIZE, "%3d", t);
    //print(buff);
    uint16_t color = LCD_TEXT_COLOR;
    if(isActive) {
        color = TFT_TEXT_HIGHLIGHT_COLOR;
        TFT_ST7735::setTextColor(color, LCD_BACKGROUND_COLOR);
    }
    uint8_t strWidth = TFT_ST7735::textWidth(buff, LCD_SMALL_FONT);
    TFT_print(buff, X_PIXEL(10), Y_PIXEL(0), LCD_SMALL_FONT);
    TFT_ST7735::drawBitmap(X_PIXEL(10+strWidth), Y_PIXEL(1), bmDegree, 8, 5, color);
    TFT_print("C", X_PIXEL(10+8+strWidth), Y_PIXEL(0), LCD_SMALL_FONT);
    //TFT_ST7735::drawCircle(X_PIXEL(16+strWidth), Y_PIXEL(16), 2, color);
    TFT_ST7735::setTextColor(LCD_TEXT_COLOR, LCD_BACKGROUND_COLOR);
    printDebug("Tset: ");
    printDebug(t);
    printDebug("  active: ");
    printDebug(isActive);
}

void DSPL::tCurr(uint16_t t) {
    uint8_t yPos = SCREEN_HEIGHT/2 + 4 - TFT_ST7735::fontHeight(LCD_BIG_FONT)/2; 
    if (t < 1000) {
        snprintf(buff, DSPL_MAX_BUFF_SIZE, "%3d", t);
    } else {
        snprintf(buff, DSPL_MAX_BUFF_SIZE, "999"); // font 6 has only numbers! print all 9 to indicate error
    }
    TFT_textPadding(TFT_ST7735::textWidth("000", LCD_BIG_FONT));
    TFT_centrePrint(buff, SCREEN_WIDTH/2 + 24, yPos, LCD_BIG_FONT);
    TFT_resetTextPadding();
   // printDebugLn(t);
    printDebug(" tCurr: ");
    printDebug(t);
    printDebug("  ");
}

/*
void DSPL::tInternal(uint16_t t) {
   // char buff[6];
    //setCharCursor(0, 1);
    if (t < 1023) {
        sprintf(buff, "%4d", t);
    } else {
        //print(F("xxxx"));
        sprintf(buff, "xxxx");
        //return;
    }
    TFT_print(buff,X_PIXEL(0), Y_PIXEL(0), LCD_SMALL_FONT);
}
*/
/*
void DSPL::tReal(uint16_t t) {
    if (t < 1000) {
        snprintf(buff, DSPL_MAX_BUFF_SIZE, "%3d", t);
    } else {
        snprintf(buff, DSPL_MAX_BUFF_SIZE, "xxx");
    }
    TFT_ST7735::setTextColor(TFT_GREEN, LCD_BACKGROUND_COLOR);
    TFT_centrePrint(buff, SCREEN_WIDTH/2 + 24, SCREEN_HEIGHT/2 + 3, LCD_SMALL_FONT);
    TFT_ST7735::setTextColor(LCD_TEXT_COLOR, LCD_BACKGROUND_COLOR);
}
*/
void DSPL::fanSpeed(uint8_t s, bool isActive) {
    snprintf(buff, DSPL_MAX_BUFF_SIZE, "%3d%c", s, '%');
    uint16_t color = LCD_TEXT_COLOR;
    if (isActive) {
        color = TFT_TEXT_HIGHLIGHT_COLOR;
        TFT_ST7735::setTextColor(color, LCD_BACKGROUND_COLOR);
    }

    ///*TFT_textPadding*/TFT_ST7735::setTextPadding(TFT_ST7735::textWidth("100%", LCD_SMALL_FONT/*14*3+17+1*/));
    //TFT_ST7735::fillRect( 54,101,9,19,TFT_YELLOW);
    
    uint16_t maxStringWidth = TFT_ST7735::textWidth("100%", LCD_SMALL_FONT);
    // clear extra digit
    if (s < 100) {
        uint8_t padding = maxStringWidth - TFT_ST7735::textWidth("00%", LCD_SMALL_FONT); // fan speed will never go under 10, 2 digits + % char
        TFT_ST7735::fillRect( X_PIXEL(FAN_BM_WIDTH/*icon pixel*/ + FAN_BITMAP_X_OFFSET + 16/*offset*/), SCREEN_HEIGHT - 1 - textHeight, padding, textHeight-7, LCD_BACKGROUND_COLOR);
    }
  //  TFT_print(buff,X_PIXEL(FAN_BM_WIDTH/*icon pixel*/+FAN_BITMAP_X_OFFSET+16), SCREEN_HEIGHT - Y_PIXEL_OFFSET - 16, LCD_SMALL_FONT);
    TFT_rightPrint(buff,X_PIXEL(FAN_BM_WIDTH/*icon pixel*/ + FAN_BITMAP_X_OFFSET + 16/*offset*/ + maxStringWidth), SCREEN_HEIGHT - 1 - textHeight/*16*/, LCD_SMALL_FONT);
  //  TFT_resetTextPadding();
    TFT_ST7735::setTextColor(LCD_TEXT_COLOR, LCD_BACKGROUND_COLOR);
    printDebug("Fan: ");
    printDebug(s);
    printDebug("  active: ");
    printDebugLn(isActive);
    
}

void DSPL::appliedPower(uint8_t p, bool show_zero) {
    if (p > 99) p = 99;
    if (p == 0 && !show_zero) {
        snprintf(buff, DSPL_MAX_BUFF_SIZE, " -");
    } else {
        snprintf(buff, DSPL_MAX_BUFF_SIZE, "%2d", p);
    }
    TFT_textPadding(TFT_ST7735::textWidth("00", LCD_SMALL_FONT));
    TFT_print(buff,X_PIXEL(7), SCREEN_HEIGHT/2 - 1, LCD_SMALL_FONT);
    TFT_resetTextPadding();
}

void DSPL::setupMode(byte mode) {
    char menu[6][10] = {"PID", "fan", "save", "exit", "reset cfg"};
    clear();
    TFT_centrePrint(menu[mode], SCREEN_WIDTH/2 -1, (SCREEN_HEIGHT/2) - 1 - textHeight/2, LCD_MENU_FONT);
}

//const char statusMsg[2][4] = {"on","off"};
void DSPL::msgON(uint16_t color) {
    TFT_ST7735::setTextColor(color, LCD_BACKGROUND_COLOR);
    TFT_rightPrint("on", SCREEN_WIDTH - X_PIXEL_OFFSET - LCD_STATUS_OFFSET, Y_PIXEL(0), LCD_SMALL_FONT);
    TFT_ST7735::setTextColor(LCD_TEXT_COLOR, LCD_BACKGROUND_COLOR);
    printDebug(" ON ");
}

void DSPL::msgOFF(uint16_t color) {
    TFT_ST7735::setTextColor(color, LCD_BACKGROUND_COLOR);
    TFT_rightPrint("off",SCREEN_WIDTH - X_PIXEL_OFFSET - LCD_STATUS_OFFSET, Y_PIXEL(0), LCD_SMALL_FONT);
    TFT_ST7735::setTextColor(LCD_TEXT_COLOR, LCD_BACKGROUND_COLOR);
    printDebug(" OFF ");
}

void DSPL::msgReady(void) {
    msgON(TFT_PINK);
}

void DSPL::msgCold(void) {
    msgOFF(TFT_BLUE);
}

void DSPL::msgFail(void) {
    TFT_ST7735::setTextColor(TFT_WHITE, TFT_RED);
    clear(TFT_RED);
    TFT_centrePrint(buff, SCREEN_WIDTH/2 -1, SCREEN_HEIGHT/2 - textHeight - 4, LCD_DEFAULT_FONT);
    TFT_centrePrint(buff1, SCREEN_WIDTH/2 -1, SCREEN_HEIGHT/2 + 4, LCD_DEFAULT_FONT);
    TFT_ST7735::setTextColor(LCD_TEXT_COLOR, LCD_BACKGROUND_COLOR);
    printDebug(buff);
    printDebugLn(buff1);
}

void DSPL::msgFanTune(uint8_t modeSel, uint8_t sel, uint16_t f_min, uint16_t f_max, uint16_t fan_current)  {
    uint16_t max_color = LCD_TEXT_COLOR;
    TFT_print("Fan Tune",X_PIXEL(0), Y_PIXEL(0), LCD_DEFAULT_FONT);
    
    // fisrt time call will have modeSel = TRUE so the selection arrow will be initialized!
    // selection mode, all text with same color, just set the strings and the arrow to the correct line
    if (modeSel)
    {
        // prepare text strings, will be kept on screen when in edit mode
        snprintf(buff, DSPL_MAX_BUFF_SIZE, "min: ");
        TFT_print(buff, X_PIXEL(18), Y_PIXEL(textHeight + 4), LCD_DEFAULT_FONT);
        snprintf(buff, DSPL_MAX_BUFF_SIZE, "max: ");
        TFT_print(buff, X_PIXEL(18), Y_PIXEL(textHeight * 2 + 4), LCD_DEFAULT_FONT);
        if (0 == sel){
            TFT_print(">", X_PIXEL(0), Y_PIXEL(textHeight + 4), LCD_DEFAULT_FONT);
            TFT_print("   ", X_PIXEL(0), Y_PIXEL(textHeight * 2 + 4 + 0), LCD_DEFAULT_FONT);
        }
        else {
            TFT_print("   ", X_PIXEL(0), Y_PIXEL(textHeight + 4), LCD_DEFAULT_FONT);
            TFT_print(">", X_PIXEL(0), Y_PIXEL(textHeight * 2 + 4), LCD_DEFAULT_FONT);
        }
    }
    // value edit mode, higlight the editable text 
    else {
      if (0 == sel) {
        TFT_ST7735::setTextColor(TFT_TEXT_HIGHLIGHT_COLOR, LCD_BACKGROUND_COLOR);
      }
      else {
        max_color = TFT_TEXT_HIGHLIGHT_COLOR;
      }
    }
    
    TFT_textPadding(TFT_ST7735::textWidth("1000", LCD_DEFAULT_FONT));
    snprintf(buff, DSPL_MAX_BUFF_SIZE, "%4d", f_min);
    TFT_rightPrint(buff, SCREEN_WIDTH - X_PIXEL_OFFSET - 5, Y_PIXEL(textHeight + 4), LCD_DEFAULT_FONT);
    TFT_ST7735::setTextColor(max_color, LCD_BACKGROUND_COLOR);
    
    snprintf(buff, DSPL_MAX_BUFF_SIZE, "%4d", f_max);
    TFT_rightPrint(buff, SCREEN_WIDTH - X_PIXEL_OFFSET - 5, Y_PIXEL(textHeight * 2 + 4), LCD_DEFAULT_FONT);
    TFT_ST7735::setTextColor(LCD_TEXT_COLOR, LCD_BACKGROUND_COLOR);
    TFT_resetTextPadding();
    // draw fan icon
    TFT_ST7735::drawBitmap(X_PIXEL(FAN_BITMAP_X_OFFSET + 16), SCREEN_HEIGHT - Y_PIXEL_OFFSET - 26 + 4, bmFan, FAN_BM_WIDTH, FAN_BM_HEIGHT, TFT_CYAN);
    uint16_t maxStringWidth = TFT_ST7735::textWidth("255", LCD_SMALL_FONT); // Max adc read value with 8bit precision is 255
    snprintf(buff, DSPL_MAX_BUFF_SIZE, "%3d", fan_current);
    uint8_t padding = maxStringWidth - TFT_ST7735::textWidth(buff, LCD_SMALL_FONT); // with 8bit adc precision fan current read by adc could go under 100
    // clear extra digit
    TFT_ST7735::fillRect(X_PIXEL(FAN_BITMAP_X_OFFSET + 16 + FAN_BM_WIDTH/*icon pixel*/ + 16/*offset*/), SCREEN_HEIGHT - 1 - Y_PIXEL_OFFSET - textHeight, padding, textHeight-7, LCD_BACKGROUND_COLOR);
    TFT_rightPrint(buff, X_PIXEL(FAN_BITMAP_X_OFFSET + 16 + FAN_BM_WIDTH/*icon pixel*/ + 16/*offset*/ + maxStringWidth), SCREEN_HEIGHT - 1 - Y_PIXEL_OFFSET - textHeight, LCD_DEFAULT_FONT);
    printDebug("Fan_adc: ");
    printDebugLn(fan_current);
}

void DSPL::msgPidTune(uint8_t mode, uint16_t kp, uint16_t ki, uint16_t kd, uint8_t kSel)
{
   /** NOTE: kSel value is in range 1-3 only when (mode == 0), otherwise it's equal to the selected pid coefficient value **/
    
    uint16_t color[3] = {LCD_TEXT_COLOR, LCD_TEXT_COLOR, LCD_TEXT_COLOR};
    TFT_print("PID Tune",X_PIXEL(0), Y_PIXEL(0), LCD_DEFAULT_FONT);

    // fisrt time call will have mode = 0 so the selection arrow will be initialized!
    // selection mode, all text with same color, just set the strings and the arrow to the correct line
    if (0 == mode)
    {
        // prepare text strings, will be kept on screen when in edit mode
        snprintf(buff, DSPL_MAX_BUFF_SIZE, "Kp: ");
        TFT_print(buff, X_PIXEL(18), Y_PIXEL(textHeight + 4), LCD_DEFAULT_FONT);
        snprintf(buff, DSPL_MAX_BUFF_SIZE, "Ki  : ");
        TFT_print(buff, X_PIXEL(18), Y_PIXEL(textHeight * 2 + 4), LCD_DEFAULT_FONT);
        snprintf(buff, DSPL_MAX_BUFF_SIZE, "Kd: ");
        TFT_print(buff, X_PIXEL(18), Y_PIXEL(textHeight * 3 + 4), LCD_DEFAULT_FONT);

        // Clear the selection arrow
        TFT_print("   ", X_PIXEL(0), Y_PIXEL(textHeight + 4), LCD_DEFAULT_FONT);
        TFT_print("   ", X_PIXEL(0), Y_PIXEL(textHeight * 2 + 4), LCD_DEFAULT_FONT);
        TFT_print("   ", X_PIXEL(0), Y_PIXEL(textHeight * 3 + 4), LCD_DEFAULT_FONT);
        
        // draw the selection arrow
        TFT_print(">", X_PIXEL(0), Y_PIXEL(textHeight * kSel + 4), LCD_DEFAULT_FONT);
    }
    else {  // value edit mode, higlight the editable text 
        color[mode - 1] = TFT_TEXT_HIGHLIGHT_COLOR;
    }

  //  TFT_textPadding(TFT_ST7735::textWidth("10000", LCD_DEFAULT_FONT));
    
    TFT_ST7735::setTextColor(color[0], LCD_BACKGROUND_COLOR);
    snprintf(buff, DSPL_MAX_BUFF_SIZE, "%5d", kp);
    TFT_rightPrint(buff, SCREEN_WIDTH - X_PIXEL_OFFSET - 5, Y_PIXEL(textHeight + 4), LCD_DEFAULT_FONT);
    
    TFT_ST7735::setTextColor(color[1], LCD_BACKGROUND_COLOR);
    snprintf(buff, DSPL_MAX_BUFF_SIZE, "%5d", ki);
    TFT_rightPrint(buff, SCREEN_WIDTH - X_PIXEL_OFFSET - 5, Y_PIXEL(textHeight * 2 + 4), LCD_DEFAULT_FONT);

    TFT_ST7735::setTextColor(color[2], LCD_BACKGROUND_COLOR);
    snprintf(buff, DSPL_MAX_BUFF_SIZE, "%5d", kd);
    TFT_rightPrint(buff, SCREEN_WIDTH - X_PIXEL_OFFSET - 5, Y_PIXEL(textHeight * 3 + 4), LCD_DEFAULT_FONT);
    
    TFT_ST7735::setTextColor(LCD_TEXT_COLOR, LCD_BACKGROUND_COLOR);
  //  TFT_resetTextPadding();
}

void DSPL::message(const char *msg, const char *msg1) {
  if (msg[0]) {
    strncpy(buff, msg, DSPL_MAX_BUFF_SIZE);
    if (msg1[0]) {
        strncpy(buff1, msg1, DSPL_MAX_BUFF_SIZE);
    }
    else {
       buff1[0] = '\0'; 
    }    
  }
  else {
    buff[0] = '\0';
    buff1[0] = '\0';
  }
}

//------------------------------------------ class HISTORY ----------------------------------------------------
class HISTORY {
    public:
        HISTORY(uint8_t h_len):queue(new uint16_t[h_len])                   { hLen = h_len; len = 0; index = 0; }
        void     init(void)                                                 { len = 0; index = 0; }
        uint16_t last(void);
        uint16_t top(void)                                                  { return queue[0]; }
        void     put(uint16_t item);                                        // Put new entry to the history
        uint16_t average(void);                                             // calculate the average value
        float    dispersion(void);                                          // calculate the math dispersion
        ~HISTORY()                                                          { delete [] queue; }  // destructor, free memory
    private:
        uint8_t hLen;                                                       // The queue buffer len
        volatile uint16_t *queue;
        volatile byte len;                                                  // The number of elements in the queue
        volatile byte index;                                                // The current element position, use ring buffer
};

void HISTORY::put(uint16_t item) {
    if (len < hLen) {
        queue[len++] = item;
    } else {
        queue[index ] = item;
        if (++index >= hLen) index = 0;                                 // Use ring buffer
    }
}

uint16_t HISTORY::last(void) {
    if (len == 0) return 0;
    uint8_t i = len - 1;
    if (index)
        i = index - 1;
    return queue[i];
}

uint16_t HISTORY::average(void) {
    uint32_t sum = 0;
    if (len == 0) return 0;
    if (len == 1) return queue[0];
    for (uint8_t i = 0; i < len; ++i) sum += queue[i];
    sum += len >> 1;                                                        // round the average
    sum /= len;
    return uint16_t(sum);
}

float HISTORY::dispersion(void) {
    if (len < 3) return 1000;
    uint32_t sum = 0;
    uint32_t avg = average();
    for (uint8_t i = 0; i < len; ++i) {
        long q = queue[i];
        q -= avg;
        q *= q;
        sum += q;
    }
    sum += len << 1;
    float d = (float)sum / (float)len;
    return d;
}

#ifdef EMP_AVERAGE
//-------------------------------------------class Exponential average ----------------------------------------
class EMP_AVERAGE {
    public:
        EMP_AVERAGE(uint8_t h_length = 8)               { emp_k = h_length; emp_data = 0; }
        void            length(uint8_t h_length)        { emp_k = h_length; emp_data = 0; }
        void            reset(void)                     { emp_data = 0; }
        int32_t         average(int32_t value);
        void            update(int32_t value);
        int32_t         read(void);
    private:
        volatile    uint8_t     emp_k       = 8;
        volatile    uint32_t    emp_data    = 0;
};

int32_t EMP_AVERAGE::average(int32_t value) {
    uint8_t round_v = emp_k >> 1;
    update(value);
    return (emp_data + round_v) / emp_k;
}

void EMP_AVERAGE::update(int32_t value) {
    uint8_t round_v = emp_k >> 1;
    emp_data += value - (emp_data + round_v) / emp_k;
}

int32_t EMP_AVERAGE::read(void) {
    uint8_t round_v = emp_k >> 1;
    return (emp_data + round_v) / emp_k;
}
#endif

//------------------------------------------ class PID algoritm to keep the temperature -----------------------
/*  The PID algorithm
 *  Un = Kp*(Xs - Xn) + Ki*summ{j=0; j<=n}(Xs - Xj) + Kd(Xn - Xn-1),
 *  Where Xs - is the setup temperature, Xn - the temperature on n-iteration step
 *  In this program the interactive formula is used:
 *    Un = Un-1 + Kp*(Xn-1 - Xn) + Ki*(Xs - Xn) + Kd*(Xn-2 + Xn - 2*Xn-1)
 *  With the first step:
 *  U0 = Kp*(Xs - X0) + Ki*(Xs - X0); Xn-1 = Xn;
 *
 *  PID coefficients history:
 *  10/14/2017  [768, 32, 328]
 */
class PID {
    public:
        PID(long  _Kp, long _Ki, long _Kd) {
            Kp = _Kp;
            Ki = _Ki;
            Kd = _Kd;
        }
        void resetPID(int temp = -1);                                       // reset PID algorithm history parameters
        // Calculate the power to be applied
        long reqPower(int temp_set, int temp_curr);
        int  changePID(uint8_t p, int k);                                   // set or get (if parameter < 0) PID parameter
    private:
        //void  debugPID(int t_set, int t_curr, long kp, long ki, long kd, long delta_p);
        int   temp_h0, temp_h1;                                             // previously measured temperature
        bool  pid_iterate;                                                  // Whether the iterative process is used
        long  i_summ;                                                       // Ki summary multiplied by denominator
        long  power;                                                        // The power iterative multiplied by denominator
        long  Kp, Ki, Kd;                                                   // The PID algorithm coefficients multiplied by denominator
        const byte denominator_p = 11;                                      // The common coefficient denominator power of 2 (11 means divide by 2048)
};

void PID::resetPID(int temp) {
    temp_h0 = 0;
    power  = 0;
    i_summ = 0;
    pid_iterate = false;
    if ((temp > 0) && (temp < 1000))
        temp_h1 = temp;
    else
        temp_h1 = 0;
}

int PID::changePID(uint8_t p, int k) {
    switch(p) {
        case 1:
            if (k >= 0) Kp = k;
            return Kp;
        case 2:
            if (k >= 0) Ki = k;
            return Ki;
        case 3:
            if (k >= 0) Kd = k;
            return Kd;
        default:
        break;
    }
    return 0;
}

long PID::reqPower(int temp_set, int temp_curr) {
    if (temp_h0 == 0) {
        // When the temperature is near the preset one, reset the PID and prepare iterative formula
        if ((temp_set - temp_curr) < 30) {
            if (!pid_iterate) {
                pid_iterate = true;
                power = 0;
                i_summ = 0;
            }
        }
        i_summ += temp_set - temp_curr;                                     // first, use the direct formula, not the iterate process
        power = Kp*(temp_set - temp_curr) + Ki*i_summ;
    // If the temperature is near, prepare the PID iteration process
    } else {
        long kp = Kp * (temp_h1 - temp_curr);
        long ki = Ki * (temp_set - temp_curr);
        long kd = Kd * (temp_h0 + temp_curr - 2*temp_h1);
        long delta_p = kp + ki + kd;
        power += delta_p;                                                   // power kept multiplied by denominator!
    }
    if (pid_iterate) temp_h0 = temp_h1;
    temp_h1 = temp_curr;
    long pwr = power + (1 << (denominator_p-1));                            // prepare the power to delete by denominator, round the result
    pwr >>= denominator_p;                                                  // delete by the denominator
    return pwr;
}

//--------------------- High frequency PWM signal calss on D9 pin ------------------------- ---------------
class FastPWM_D9 {
    public:
        FastPWM_D9()                                { }
        void init(void);
        void setDutyPwm(uint8_t d)                  { if(OCR1A != d) { OCR1A = d; } }
        uint8_t   getFanSpeedPwm(void)              { return OCR1A; }
        uint8_t   readFanCurrent(void)              { //bitClear(ADCSRA, ADEN); // disable ADC while reading result conversion
                                                      while (bit_is_set(ADCSRA, ADSC)) { } // assert the end of measurement
                                                      return ADCH; // read 8bit adc High register. Low register doesn't matter with ADLAR set
                                                      
                                                      //ADCSRA |= (1 << ADEN);  // re-enable ADC
                                                      //ADCSRA |= (1 << ADSC);  // re-start ADC measurements
                                                      //return a; 
                                                    }
        void        fanAdcStart(void)               { bitSet(ADCSRA, ADSC); }
};

void FastPWM_D9::init(void) {
    pin_mode(FAN_GUN_PIN, OUTPUT);
    pin_mode(FAN_GUN_SENS_PIN, INPUT);
    digital_write(FAN_GUN_PIN, LOW);
    
    /* ADC setup */
    //ADCSRA = 0;             // clear ADCSRA register
    ADCSRB = 0;             // clear ADCSRB register to enable free running mode
    //ADMUX  = 0;
    //DIDR0 |= bit(FAN_GUN_SENS_PIN - 14);         // digital input disable for analog pins A0-A5
    ADMUX  = ((FAN_GUN_SENS_PIN - 14) & 0x07) | (1 << REFS0) | (1 << ADLAR);    // set A0 analog input pin
                                                                                  // set reference voltage to AVCC
                                                                                  // left align ADC value to 8 bits precision instead of 10bits from ADCH register. Just read ADCH register, range 0-255 instead of 0-1023
    //ADMUX |= (1 << REFS0);  // set reference voltage to AVCC
    //ADMUX |= (1 << ADLAR);  // left align ADC value to 8 bits precision instead of 10bits from ADCH register. Just read ADCH register, range 0-255 instead of 0-1023

    // sampling rate is [ADC clock] / [prescaler] / [conversion clock cycles]
    // for Arduino Uno ADC clock is 16 MHz and a conversion takes 13 clock cycles
    ADCSRA = (1 << ADPS2) | (1 << ADPS0);    // 32 prescaler for 38.5 KHz
    //ADCSRA |= (1 << ADPS2);                     // 16 prescaler for 76.9 KHz

    
    //ADCSRA |= (1 << ADATE) | (1 << ADEN); // enable auto trigger, enable ADC
    //ADCSRA |= (1 << ADIE);  // enable interrupts when measurement complete
    ADCSRA |= (1 << ADEN);  // enable ADC
    //ADCSRA |= (1 << ADSC);  // start ADC measurements
    //bitSet(ADCSRA, ADSC); // start ADC measurements
    //fanAdcStart();
    
    /* Timer setup */
    noInterrupts();
    TCNT1   = 0;
    TCCR1B  = _BV(WGM13);                           // set mode as phase and frequency correct pwm, stop the timer
    TCCR1A  = 0;
    ICR1    = 256;
#ifdef SLOW_D9_PWM_FREQUENCY
    TCCR1B   = _BV(WGM13) | _BV(CS12) | _BV(CS10);  // Top value = ICR1, prescale = 1024 (30.5 Hz)
#else
    TCCR1B  = _BV(WGM13) | _BV(CS10);               // Top value = ICR1, prescale = 1
#endif
    TCCR1A |= _BV(COM1A1);                          // XOR D9 on OCR1A, detached from D10
    OCR1A   = 0;                                    // Switch-off the signal on pin 9;
    interrupts();
}

//--------------------- Hot air gun manager using total sine shape to power on the hardware ---------------
class HOTGUN : public PID {
    public:
        typedef enum { POWER_OFF, POWER_ON, POWER_COOLING } PowerMode;
        HOTGUN(/*uint8_t HG_sen_pin,*/ uint8_t HG_pwr_pin, uint8_t HG_ACrelay_pin, uint8_t _h_power_len, uint8_t _h_temp_len, uint8_t _h_fan_len) ;
        void        init(void);
        bool        isOn(void)                                              { return (mode == POWER_ON /*|| mode == POWER_FIXED*/); }
        void        setTemp(uint16_t t)                                     { temp_set = constrain(t, min_temp, max_temp); }
        uint16_t    getTemp(void)                                           { return temp_set; }
        uint16_t    getCurrTemp(void)                                       { return h_temp.last(); }
        uint16_t    tempAverage(void)                                       { return h_temp.average(); }
        void        fillTempQueue(uint16_t t)                               { h_temp.put(t); }
        uint8_t     powerAverage(void)                                      { return h_power.average(); }
        uint8_t     appliedPower(void)                                      { return actual_power; }
        void        setFanSpeed(uint8_t f)                                  { fan_speed = constrain(f, min_working_fan, max_working_fan); } //set fan speed value but not update PWM register
        uint8_t     getFanSpeed(void)                                       { return fan_speed; }
        void        setFanDuty(uint8_t fanPcnt)                             { hg_fan.setDutyPwm(map(fanPcnt,0,100,0,255)); }
        void        setFanCurrentThreshold(uint8_t minTh, uint8_t maxTh)    { minFanTh = minTh; maxFanTh = maxTh; }
        uint8_t     getFanCurrentMinThreshold(void)                         { return minFanTh; }
        uint8_t     getFanCurrentMaxThreshold(void)                         { return maxFanTh; }
        uint8_t     getFanCurrent(void)                                     { return h_fanCurrent.average();/*hg_fan.fanCurrent();*/ } 
        uint8_t     readFanCurrent(void)                                    { return hg_fan.readFanCurrent(); }                             
        uint8_t     presetFanPcnt(void);
        uint8_t     fanPwmToPcnt(uint8_t fanPwm);
        void        fanAdcStart(void)                                       { return hg_fan.fanAdcStart(); }
        uint16_t    tempDispersion(void)                                    { return h_temp.dispersion(); }
        bool        isCold(void)                                            { return h_temp.average() < temp_gun_cold; }
        bool        areExternalInterrupts(void)                             { return millis() - last_period < period * 15; }
        uint8_t     avgPowerPcnt(void);
        void        switchPower(bool On);
       // void        fixPower(uint8_t Power);                                // Set the specified power to the the hot gun
        int8_t      keepTemp(void);
        uint8_t     readTemp(void);
        //uint8_t     getMaxFixedPower(void)                                  { return max_fix_power; }
        bool        syncCB(void);                                           // Return true at the end of the power period
        void        activateRelay(bool activate);
        void        shutdown(void);
    private:
        bool        isGunConnected(void)                                    { return true; }
        uint16_t    emulateTemp(void);                                      // To debug the project, simulate the Hot Air Gun heating process
        FastPWM_D9  hg_fan;
        uint16_t    temp_set;                                               // The preset temperature of the hot air gun (internal units)
        uint8_t     fan_speed;
        uint8_t     minFanTh;
        uint8_t     maxFanTh;
        uint8_t     gun_pin;
        uint8_t     relay_pin;
        HISTORY     h_power;                                                // The history queue of power applied values
        HISTORY     h_temp;                                                 // The history queue of the temperature
        HISTORY     h_fanCurrent;                                           // The exponential average of fan current
        volatile    uint8_t     cnt;
        volatile    uint8_t     actual_power;
        volatile    bool        active;
        uint8_t     actual_fan  = 0;                                        // Power applied to the fan (can be turned off)
        uint8_t     fix_power   = 0;                                        // Fixed power value of the Hot Air Gun (or zero if off)
        PowerMode   mode        = POWER_OFF;
        bool        chill;                                                  // To chill the hot gun
        volatile    uint32_t    last_period;                                // The time in ms when the counter reset
        volatile    uint8_t     relay_ready_cnt = 0;
        const       uint8_t     period          = 100;
       // const       uint16_t    int_temp_max    = 900;                      // The raw ADC temp value 900 corresponds to about 400°C
        const       uint8_t     max_fix_power   = 70;
        const       uint8_t     max_power       = 99;
        const       uint16_t    min_temp  = TEMP_MIN_C;
        const       uint16_t    max_temp  = TEMP_MAX_C;
        //const       uint16_t    min_fan_speed   = 30;
        //const       uint16_t    max_fan_speed   = max_working_fan;
        const       uint8_t    max_cool_fan    = MAX_COOL_FAN;
        const       uint8_t    temp_gun_cold   = TEMP_GUN_COLD;                       // The temperature of the cold iron
        const       uint8_t     temp_gun_cold_hysteresis = TEMP_GUN_COLD_HYSTERESIS;
        //const       uint8_t     e_sensor_length = 10;                       // Exponential average length of sensor data
        const       uint32_t    relay_activate  = 1;        // The relay activation delay
};

HOTGUN::HOTGUN(/*uint8_t HG_sen_pin,*/ uint8_t HG_pwr_pin, uint8_t HG_ACrelay_pin, uint8_t _h_power_len, uint8_t _h_temp_len, uint8_t _h_fan_len) : PID(PID_KP, PID_KI, PID_KD), h_power(_h_power_len), h_temp(_h_temp_len), h_fanCurrent(_h_fan_len) {
    //sen_pin   = HG_sen_pin;
    gun_pin   = HG_pwr_pin;
    relay_pin = HG_ACrelay_pin;
}

void HOTGUN::init(void) {
    cnt             = 0;
    fan_speed       = 0;
    actual_power    = 0;
    fix_power       = 0;
    active      = false;
    chill       = false;
    last_period = 0;
    //pin_mode(sen_pin, INPUT);
    pin_mode(gun_pin, OUTPUT);
    pin_mode(relay_pin, OUTPUT);
    digital_write(gun_pin, LOW);
    activateRelay(false);
    hg_fan.init();
    h_temp.init();
    //e_sensor.length(e_sensor_length);
    resetPID();
}

bool HOTGUN::syncCB(void) {
    if (++cnt >= period) {
        cnt = 0;
        last_period = millis();                                             // Save the current time to check the external interrupts
        if (!active && (actual_power > 0)) {
            digital_write(gun_pin, HIGH);
            active = true;
        }
    } else if (cnt >= actual_power) {
        if (active) {
            digital_write(gun_pin, LOW);
            active = false;
        }
    }
 /*   if (!active) {
        //e_sensor.update(analogRead(sen_pin));
        temp_update = true;
    }*/
    return (cnt == 0);                                                      // End of the Power period (period AC voltage shapes)
}

void HOTGUN::switchPower(bool On) {
    switch (mode) {
        case POWER_OFF:
            if (hg_fan.getFanSpeedPwm() == 0) {                                   // Not power supplied to the Fan
                if (On) {                                                    // !FAN && On
                    activateRelay(true);
                    mode = POWER_ON;
                }
            } else {
                if (On) {
                    if (isGunConnected()) {                                // FAN && On && connected
                        activateRelay(true);
                        mode = POWER_ON;
                    } else {
                        shutdown();
                    }
                } else {
                    if (isGunConnected()) {                                 // FAN && !On && connected
                        if (isCold()) {                                     // FAN && !On && connected && cold
                            shutdown();
                        } else {                                            // FAN && !On && connected && !cold
                            mode = POWER_COOLING;
                        }
                    }
                }
            }
            break;
        case POWER_ON:
            if (!On) {
                mode = POWER_COOLING;
                //activateRelay(false); // will be done next in shutdown
            }
            break;
/*        case POWER_FIXED:
            if (hg_fan.fanSpeed()) {
                if (On) {                                                   // FAN && On
                    activateRelay(true);
                    mode = POWER_ON;
                } else {                                                    // FAN && !On
                    if (isGunConnected()) {                                 // FAN && !On && connected
                        if (isCold()) {                                     // FAN && !On && connected && cold
                            shutdown();
                        } else {                                            // FAN && !On && connected && !cold
                            mode = POWER_COOLING;
                            activateRelay(false);
                        }
                    }
                }
            } else {                                                        // !FAN
                if (!On) {                                                  // !FAN && !On
                    shutdown();
                }
            }
            break;*/
        case POWER_COOLING:
            if (hg_fan.getFanSpeedPwm()) {
                if (On) {                                                   // FAN && On
                    if (isGunConnected()) {                                 // FAN && On && connected
                        activateRelay(true);
                        mode = POWER_ON;
                    } else {                                                // FAN && On && !connected
                        shutdown();
                    }
                } else {                                                    // FAN && !On
                    if (isGunConnected()) {
                        if (isCold()) {                                     // FAN && !On && connected && cold
                            shutdown();
                        }
                    } else {                                                // FAN && !On && !connected
                        shutdown();
                    }
                }
            } else {
                if (On) {                                                   // !FAN && On
                    activateRelay(true);
                    mode = POWER_ON;
                }
            }
    }
    h_power.init();
}

uint8_t HOTGUN::readTemp(void) {
    fanAdcStart(); // start adc measurments here. Conversion will take place durin temperature reading from MAX6675
    int16_t temp = (int16_t)(thermocouple.readTempC());
    static uint8_t thermocoupleErrorCnt = 0;
    static uint8_t fanCurrentErrorCnt = 0;
    
    //Serial.println(temp);
    if ((/*MAX6675_THERMOCOUPLE_OPEN == temp ||*/ temp < 0) || (isOn() && temp == 0)) { // MAX6675_THERMOCOUPLE_OPEN is -1 so checking for <= includes open connection error!
        temp = h_temp.average();
        thermocoupleErrorCnt++;
        //Serial.println("th error");
    }
    else
    {
        thermocoupleErrorCnt = 0;
        //Serial.println("th ok");
    }
    
    h_fanCurrent.put(hg_fan.readFanCurrent());
    uint8_t fanCurrent = h_fanCurrent.average();
    printDebug("fanCurrent: ");
    printDebugLn(fanCurrent);

    if (hg_fan.getFanSpeedPwm() && ((fanCurrent < minFanTh) || (fanCurrent > maxFanTh))) { /* Fan is on and sensing current is lower or higher than the threshold */
        fanCurrentErrorCnt++;
    }
    else {
        fanCurrentErrorCnt = 0; 
    }
    
    //e_sensor.update(temp);
#ifdef EMULATE_TEMP
    temp = emulateTemp();
#endif
    h_temp.put((uint16_t)temp);
    
    return ((thermocoupleErrorCnt & 0xF) | fanCurrentErrorCnt << 4);
}

// This routine is used to keep the hot air gun temperature near required value
int8_t HOTGUN::keepTemp(void) {
    long p = 0;
    uint8_t errorTc;
    uint8_t errorFan;
    errorTc = readTemp();
    errorFan = errorTc >> 4;
    errorTc = errorTc & 0xF;

    if(/* isOn() && */                                                                    /* Heater is ON */ 
       ((errorTc > TC_ERROR_TOLLERANCE) // ||                                          /* MAX6675 reading error */
       //(hg_fan.fanSpeed() && ((fanCurrent < minFanTh) || (fanCurrent > maxFanTh))) /* Fan sensing current is lower or higher tha the threshold */
       ))
    {
        // In case of errors in temperature reading shutdown immediately
        // but leave the fan spinning at max speed for safety and cool down the heater 
        shutdown();
        setFanDuty(max_working_fan);
        return -1;
    }
    else if (errorFan > FAN_ERROR_TOLLERANCE) { // tollerance to take into account spin up time of the fan
        // In case of errors on Fan feedback shutdown immediately
        // but leave the fan spinning at max speed for safety and cool down the heater
        shutdown();
        setFanDuty(max_working_fan);
        return -2;
    }

   // hg_fan.fanAdcStart(); // start adc conversion for next run
    
    uint16_t temp = getCurrTemp();
    uint8_t fan;
     
    if (!chill && (mode == POWER_ON) && (temp > (temp_set + 10))) { // Prevent global over heating
        //digital_write(gun_pin, LOW);
        p = 0;
        chill = true;                                               // Turn off the power in main working mode only;
    }

    switch (mode) {
        case POWER_OFF:
             if (/*!isCold() &&*/ (temp > (temp_gun_cold + temp_gun_cold_hysteresis))) {                                               // FAN && connected && !cold
                ///*uint16_t*/ fan = map(temp, temp_gun_cold, temp_maxC, max_cool_fan, min_working_fan);
                //fan = max_cool_fan;
                //fan = constrain(fan, min_working_fan, max_working_fan);
                setFanDuty(max_cool_fan); // just start the fun here. Speed will be adjusted in POWER_COOLING mode during next iteration 
                mode = POWER_COOLING;
            }
            break;
        case POWER_ON:
            setFanDuty(fan_speed);                                         // Turn on the fan immediately
            if (chill) {
                if (temp < (temp_set - 5)) {
                    chill = false;
                    resetPID();
                } else {
                    p = 0;
                    break;
                }
            }
            if (relay_ready_cnt > 0) {
                --relay_ready_cnt;
            }
            else {
                p = PID::reqPower(temp_set, temp);
                p = constrain(p, 0, max_power);
            }
            break;
/*        case POWER_FIXED:
            if (relay_ready_cnt > 0) {
                --relay_ready_cnt;
            }
            else {
                p  = fix_power;
            }
            setFanDuty(fan_speed);
            break;*/
        case POWER_COOLING:
            if (fanPwmToPcnt((hg_fan.getFanSpeedPwm())) < min_working_fan) {
                shutdown();
            } else {
                if (isGunConnected()) {
                    if (isCold()) {                                         // FAN && connected && cold
                        shutdown();
                    } else {                                                // FAN && connected && !cold
                        /*uint16_t*/ fan = map(temp, temp_gun_cold, max_temp, max_cool_fan, min_working_fan);
                        //fan = max_cool_fan;
                        fan = constrain(fan, min_working_fan, max_working_fan);
                        setFanDuty(fan);
                    }
                } else {                                                    // FAN && !connected
                    shutdown();
                }
            }
            break;
        default:
            break;
    }
    // Only supply the power to the heater if the gun is connected and the fan is on
    if((fanPwmToPcnt((hg_fan.getFanSpeedPwm())) < min_working_fan) || !isGunConnected()) {
        p = 0;
    }
    h_power.put(p);
    actual_power = p;
    if (p == 0) {
        digital_write(gun_pin, LOW);
    }
    return 0;
}
/*
void HOTGUN::fixPower(uint8_t Power) {
    if (Power == 0) {                                                       // To switch off the hot gun, set the Power to 0
        switchPower(false);
        return;
    }

    if (Power > max_power) Power = max_power;
    mode = POWER_FIXED;
    activateRelay(true);
    fix_power   = Power;
}
*/
uint8_t HOTGUN::avgPowerPcnt(void) {
    uint8_t pcnt = 0;
//    if (mode == POWER_FIXED) {
//        pcnt = map(fix_power, 0, max_fix_power, 0, 100);
//    } else {
        pcnt = map(h_power.average(), 0, max_power, 0, 100);
//    }
    if (pcnt > 100) pcnt = 100;
    return pcnt;
}

void HOTGUN::shutdown(void) {
    digital_write(gun_pin, LOW);
    hg_fan.setDutyPwm(0);
    mode            = POWER_OFF;
    actual_power    = 0;
    active          = false;
    activateRelay(false);
}

uint16_t HOTGUN::emulateTemp(void) {
    static int16_t t = 0;
    uint8_t ap = actual_power;
//    if (mode == POWER_FIXED)
//        ap = fix_power;
    ap = constrain(ap, 0, 100);
    t += map(ap, 0, 100, 0, 30);
    uint8_t fn = hg_fan.getFanSpeedPwm();
    t -= fn/40 + t/50 + 1;
    if (t < 0) t = 0;
    return t;
}

/*
 * We need some time to activate the relay, so we initialize the relay_ready_cnt variable.
 *
 */
void HOTGUN::activateRelay(bool activate) {
    if (activate) {
        digital_write(relay_pin, HIGH);
        relay_ready_cnt = relay_activate;
    } else {
        digital_write(relay_pin, LOW);
        relay_ready_cnt = 0;
    }
}

uint8_t HOTGUN::presetFanPcnt(void) {
    return fan_speed;
/*  uint16_t pcnt = map(fan_speed, 0, max_working_fan, 0, 100);
  if (pcnt > 100) pcnt = 100;
  return pcnt;*/
}

uint8_t HOTGUN::fanPwmToPcnt(uint8_t fanPwm) {
  //return ((fanPwm * 100) / 255);
  return (((fanPwm*100)+128)>>8);
}

//------------------------------------------ class SCREEN ------------------------------------------------------
class SCREEN {
    public:
        SCREEN* next;                                                       // Pointer to the next screen
        SCREEN() {
            next            = 0;
            update_screen   = 0;
            //scr_timeout     = 0;
           // time_to_return  = 0;
        }
        virtual void    init(void)                                          { }
        virtual SCREEN* show(void)                                          { return this; }
        virtual SCREEN* menu(void)                                          { return this; }
        virtual SCREEN* menu_long(void)                                     { if (this->next != 0)  return this->next;  else return this; }
        virtual SCREEN* reedSwitch(bool on)                                 { return this; }
        virtual void    rotaryValue(int16_t value)                          { }
        void            forceRedraw(void)                                   { update_screen = 0; }
    protected:
        uint32_t update_screen;                                             // Time in ms when the screen should be updated
       // uint32_t scr_timeout;                                               // Timeout is sec. to return to the main screen, canceling all changes
       // uint32_t time_to_return;                                            // Time in ms to return to main screen
};

//---------------------------------------- class mainSCREEN [the hot air gun is OFF] ---------------------------
class mainSCREEN : public SCREEN {
    public:
        mainSCREEN(HOTGUN* HG, DSPL* DSP, ENCODER* ENC, BUZZER* Buzz, HOTGUN_CFG* Cfg) {
            pHG     = HG;
            pD      = DSP;
            pEnc    = ENC;
            pBz     = Buzz;
            pCfg    = Cfg;
        }
        virtual void    init(void);
        virtual SCREEN* show(void);
        virtual SCREEN* menu(void);
        virtual SCREEN* reedSwitch(bool on);
        virtual void    rotaryValue(int16_t value);                         // Setup the preset temperature
        SCREEN*     on;                                                     // Screen mode when the power is
    private:
        HOTGUN*     pHG;                                                    // Pointer to the hot air gun instance
        DSPL*       pD;                                                     // Pointer to the DSPLay instance
        ENCODER*    pEnc;                                                   // Pointer to the rotary encoder instance
        BUZZER*     pBz;                                                    // Pointer to the simple buzzer instance
        HOTGUN_CFG* pCfg;                                                   // Pointer to the configuration instance
        uint32_t    clear_used_ms;                                          // Time in ms when used flag should be cleared (if > 0)
        bool        mode_temp;                                              // Preset mode: change temperature or change fan speed
        bool        used;                                                   // Whether the IRON was used (was hot)
        bool        cool_notified;                                          // Whether there was cold notification played
        const uint16_t period               = 1000;                         // The period to update the screen
        const uint32_t cool_notify_period   = 120000;                       // The period to display 'cool' message (ms)
        const uint16_t show_temp            = 20000;                        // The period to show the preset temperature (ms)
};

void mainSCREEN::init(void) {
    pHG->switchPower(false);
    uint16_t temp_set   = pHG->getTemp();
    uint16_t tempH      = /*pCfg->tempHuman(*/temp_set/*)*/;                        // The preset temperature in the human readable units
    pEnc->reset(tempH, TEMP_MIN_C, TEMP_MAX_C, 1, 5);
    used = !pHG->isCold();
    cool_notified = !used;
    if (used) {                                                             // the hot gun was used, we should save new data in EEPROM
        pCfg->save(temp_set, pHG->getFanSpeed());
    }
    mode_temp = true;
    clear_used_ms = 0;
    //pD->clear();
    pD->backgroundInit();
    forceRedraw();
}

void mainSCREEN::rotaryValue(int16_t value) {
    if (mode_temp) {                                                        // set hot gun temperature
        uint16_t temp = pCfg->tempInternal(value);
        pHG->setTemp(temp);
        //pD->tSet(value, true);
    } else {                                                                // set fan speed
        pHG->setFanSpeed(value);
        //pD->fanSpeed(pHG->presetFanPcnt(), true);
    }
    //update_screen  = millis() + period;
    forceRedraw();
}

SCREEN* mainSCREEN::show(void) {
    if (millis() < update_screen) return this;
    update_screen = millis() + period;

    if (clear_used_ms && (millis() > clear_used_ms)) {
        clear_used_ms = 0;
        used = false;
    }

    uint16_t temp_set = pHG->getTemp();
    pD->tSet(/*pCfg->tempHuman(*/temp_set/*)*/, mode_temp);
    uint16_t temp  = pHG->tempAverage();
    uint16_t tempH = /*pCfg->tempHuman(*/temp/*)*/;
    if (pHG->isCold()) {
        if (used) {
            pD->msgCold();
        } else {
            pD->msgOFF();
        }
        if (used && !cool_notified) {
            pBz->lowBeep();
            cool_notified = true;
            clear_used_ms = millis() + cool_notify_period;
        }
    } else {
        pD->msgOFF();
    }
    pD->tCurr(tempH/*pHG->getCurrTemp()*//*(uint16_t)thermocouple.readTempC()*/);
    pD->appliedPower(0, false);
    pD->fanSpeed(pHG->presetFanPcnt(), !mode_temp);
    return this;
}

SCREEN* mainSCREEN::menu(void) {
    if (mode_temp) {                                                        // Prepare to adjust the fan speed
        uint8_t fs = pHG->getFanSpeed();
        pEnc->reset(fs, min_working_fan, max_working_fan, 1, 5);
        mode_temp = false;
    } else {                                                                // Prepare to adjust the preset temperature
        uint16_t temp_set   = pHG->getTemp();
        uint16_t tempH      = /*pCfg->tempHuman(*/temp_set/*)*/;
        pEnc->reset(tempH, TEMP_MIN_C, TEMP_MAX_C, 1, 5);
        mode_temp = true;
    }
    forceRedraw();
    return this;
}

SCREEN* mainSCREEN::reedSwitch(bool on) {
    if (on && this->on)
        return this->on;
    return this;
}

//---------------------------------------- class workSCREEN [the hot air gun is ON] ----------------------------
class workSCREEN : public SCREEN {
    public:
        workSCREEN(HOTGUN* HG, DSPL* DSP, ENCODER* Enc, BUZZER* Buzz, HOTGUN_CFG* Cfg) {
            update_screen = 0;
            pHG     = HG;
            pD      = DSP;
            pBz     = Buzz;
            pEnc    = Enc;
            pCfg    = Cfg;
        }
        virtual void    init(void);
        virtual SCREEN* show(void);
        virtual SCREEN* menu(void);
        virtual SCREEN* reedSwitch(bool on);
        virtual void    rotaryValue(int16_t value);                         // Change the preset temperature
    private:
        HOTGUN*     pHG;                                                    // Pointer to the IRON instance
        DSPL*       pD;                                                     // Pointer to the DSPLay instance
        BUZZER*     pBz;                                                    // Pointer to the simple Buzzer instance
        ENCODER*    pEnc;                                                   // Pointer to the rotary encoder instance
        HOTGUN_CFG* pCfg;                                                   // Pointer to the configuration instance
        bool        ready;                                                  // Whether the IRON have reached the preset temperature
        bool        mode_temp;                                              // Preset mode: temperature or fan speed
        const uint16_t period = 1000;                                       // The period to update the screen (ms)
};

void workSCREEN::init(void) {
    uint8_t fs = pHG->getFanSpeed();
    pEnc->reset(fs, min_working_fan, max_working_fan, 1, 5);
    mode_temp   = false;                                                    // By default adjust the fan speed
    pHG->switchPower(true);
    ready = false;
    //pD->clear();
    pD->backgroundInit();
    forceRedraw();
}

void workSCREEN::rotaryValue(int16_t value) {                               // Setup new preset temperature by rotating the encoder
    if (mode_temp) {
        ready = false;
        uint16_t temp = pCfg->tempInternal(value);                          // Translate human readable temperature into internal value
        pHG->setTemp(temp);
        //pD->tSet(value, true);
    } else {
        pHG->setFanSpeed(value);
        //pD->fanSpeed(pHG->presetFanPcnt(), true);
    }
    //update_screen = millis() + period;
    forceRedraw();
}

SCREEN* workSCREEN::show(void) {
    if (millis() < update_screen) return this;
    update_screen = millis() + period;

    int temp_set  = pHG->getTemp();
    int tempH_set = /*pCfg->tempHuman(*/temp_set/*)*/;
    pD->tSet(tempH_set, mode_temp);
    int temp      = pHG->tempAverage();
    int tempH     = /*pCfg->tempHuman(*/temp/*)*/;
    pD->tCurr(tempH);
    pD->msgON();
    uint8_t p   = pHG->appliedPower();
    pD->appliedPower(p);
    pD->fanSpeed(pHG->presetFanPcnt(), !mode_temp);


//Serial.print("Diff = "); Serial.print(temp_set - temp);
//uint32_t disp = pHG->tempDispersion();
//Serial.print(", Dispersion = "); Serial.println(disp);

    if ((abs(temp_set - temp) < 2) && (pHG->tempDispersion() <= 10))  {
        if (!ready) {
            pBz->shortBeep();
            ready = true;
            pD->msgReady();
//            Serial.println("Ready");
            update_screen = millis() + (period << 2);
            return this;
        }
    }
    return this;
}

SCREEN* workSCREEN::menu(void) {
    if (mode_temp) {
        uint8_t fs = pHG->getFanSpeed();
        pEnc->reset(fs, min_working_fan, max_working_fan, 1, 5);
        mode_temp = false;
    } else {
        uint16_t temp_set   = pHG->getTemp();
        uint16_t tempH      = /*pCfg->tempHuman(*/temp_set/*)*/;
        pEnc->reset(tempH, TEMP_MIN_C, TEMP_MAX_C, 1, 5);
        mode_temp = true;
    }
    forceRedraw();
    return this;
}

SCREEN* workSCREEN::reedSwitch(bool on) {
    if (!on && next) {
        pBz->doubleBeep();
        return next;
    }
    return this;
}

//---------------------------------------- class errorSCREEN [the error detected] ------------------------------
class errorSCREEN : public SCREEN {
    public:
        errorSCREEN(HOTGUN* HG, DSPL* DSP, BUZZER* Buzz) {
            pHG     = HG;
            pD      = DSP;
            pBz     = Buzz;
        }
        virtual void init(void);
        virtual SCREEN* menu(void)                                          { if (this->next != 0)  return this->next;  else return this; }
    private:
        HOTGUN*     pHG;                                                    // Pointer to the got air gun instance
        DSPL*       pD;                                                     // Pointer to the display instance
        BUZZER*     pBz;                                                    // Pointer to the simple Buzzer instance
};

void errorSCREEN::init(void) {
    pHG->switchPower(false);
    pD->clear();
    pD->msgFail();
    pBz->failedBeep();
}

//---------------------------------------- class configSCREEN [configuration menu] -----------------------------
class configSCREEN : public SCREEN {
    public:
        configSCREEN(HOTGUN* HG, DSPL* DSP, ENCODER* Enc, HOTGUN_CFG* Cfg) {
            pHG     = HG;
            pD      = DSP;
            pEnc    = Enc;
            pCfg    = Cfg;
        }
        virtual void    init(void);
        virtual SCREEN* show(void);
        virtual SCREEN* menu(void);
        //virtual SCREEN* menu_long(void);
        virtual void    rotaryValue(int16_t value);
//        SCREEN*         calib;                                              // Pointer to the calibration SCREEN
        SCREEN*         pid;                                                // Pointer to the pid tune SCREEN
        SCREEN*         fan;                                                // Pointer to fan SCREEN
    private:
        HOTGUN*     pHG;                                                    // Pointer to the HOTGUN instance
        DSPL*       pD;                                                     // Pointer to the DSPLay instance
        ENCODER*    pEnc;                                                   // Pointer to the rotary encoder instance
        HOTGUN_CFG* pCfg;                                                   // Pointer to the config instance
        uint8_t     mode;                                                   // 0 - PID tune, 1 - Fan tune, 2 - save, 3 - cancel, 4 - defaults
        const uint16_t period = 10000;                                      // The period in ms to update the screen
};

void configSCREEN::init(void) {
    pHG->switchPower(false);
    mode = 0;
    pEnc->reset(mode, 0, 4, 1, 0, true);
    pD->clear();
    pD->setupMode(0);
   // this->scr_timeout = 30;                                                 // This variable is defined in the superclass
}

SCREEN* configSCREEN::show(void) {
    if (millis() < update_screen) return this;
    update_screen = millis() + period;
    pD->setupMode(mode);
    return this;
}

SCREEN* configSCREEN::menu(void) {
    switch (mode) {
        case 0:                                                           // PID Tune
            if (pid) return pid;
            break;
        case 1:
            if(fan) return fan;
            break;
        case 2:                                                             // Save configuration data
//            pCfg->saveFanData(fan_min, fan_max, false);
//            pCfg->savePidData(fan_min, fan_max, false);
            pCfg->CONFIG::save();
            menu_long();
        case 3:                                                             // Exit, Return to the main menu
            if (next) return next;
            break;
        case 4:                                                             // Save defaults
            pCfg->setDefaults(true);
            resetFunc();  //call reset to auto-reboot Arduino and re-load evrything from EEPROM
            if (next) return next;
            break;
    }
    forceRedraw();
    return this;
}

void configSCREEN::rotaryValue(int16_t value) {
    mode = value;
    forceRedraw();
}
/*
SCREEN* configSCREEN::menu_long(void) {   
    //uint8_t fan = pHG->getFanSpeed();
    //pCfg->save(preset_temp, fan);
    if (next) return next;
    return this;
}
*/

#ifdef PID_SCREEN
//---------------------------------------- class pidSCREEN [tune the PID coefficients] -------------------------
class pidSCREEN : public SCREEN {
    public:
        pidSCREEN(HOTGUN* HG, DSPL* DSP, ENCODER* Enc, HOTGUN_CFG* Cfg) {
            pHG     = HG;
            pD      = DSP;
            pEnc    = Enc;
            pCfg    = Cfg;
        }
        virtual void    init(void);
        virtual SCREEN* menu(void);
        virtual SCREEN* menu_long(void);
        virtual SCREEN* show(void);
        virtual void    rotaryValue(int16_t value);
    private:
        void        showCfgInfo(void);                                      // show the main config information: Temp set, fan speed and PID coefficients
        HOTGUN*     pHG;                                                    // Pointer to the HOTGUN instance
        DSPL*       pD;                                                     // Pointer to the DSPLay instance
        ENCODER*    pEnc;                                                   // Pointer to the rotary encoder instance
        HOTGUN_CFG* pCfg;                                                   // Pointer to the config instance
        uint8_t     mode;                                                   // Which parameter to tune [0-5]: select element, Kp, Ki, Kd, temp, speed
        uint32_t    update_screen;                                          // Time in ms when to print thee info
        int         temp_set;
        const uint16_t period = 1100;
};

void pidSCREEN::init(void) {
    //temp_set = pHG->getTemp();
    mode = 0;                                                               // select the element from the list
    pEnc->reset(1, 1, 3, 1, 1, true);                                       // 1 - Kp, 2 - Ki, 3 - Kd
    //showCfgInfo();
    pD->clear();
    //pD->msgPidTune(mode, pHG->changePID(1, -1), pHG->changePID(2, -1), pHG->changePID(3, -1), 1 );  // When called with negative values changePID() return current value
    forceRedraw();
}

void pidSCREEN::rotaryValue(int16_t value) {
    if (mode == 0) {                                                        // select element from the menu
        //showCfgInfo();
        //kSel = value; is the same of pEnc->read();
    } else {
        pHG->changePID(mode, value);
    }
    pD->msgPidTune(mode, pHG->changePID(1, -1), pHG->changePID(2, -1), pHG->changePID(3, -1), pEnc->read() );  // When called with negative values changePID() return current value

    forceRedraw();
}

SCREEN* pidSCREEN::show(void) {
    if (millis() < update_screen) return this;
    update_screen = millis() + period;

    pD->msgPidTune(mode, pHG->changePID(1, -1), pHG->changePID(2, -1), pHG->changePID(3, -1), pEnc->read() );  // When called with negative values changePID() return current value
    return this;
}
SCREEN* pidSCREEN::menu(void) {                                             // The encoder button pressed
    if (mode == 0) {                                                        // select upper or lower temperature limit
        mode = pEnc->read();
//        if (mode > 0 && mode < 4) {
            int k = pHG->changePID(mode, -1);
            pEnc->reset(k, 0, 10000, 1, 5);
//        }
    } else {
        pEnc->reset(mode, 1, 3, 1, 1, true);                                   // 1 - Kp, 2 - Ki, 3 - Kd
        mode = 0;
    }
    forceRedraw();
    return this;
}

SCREEN* pidSCREEN::menu_long(void) {
  //pHG->switchPower(false);
  pCfg->savePidData(pHG->changePID(1, -1), pHG->changePID(2, -1), pHG->changePID(3, -1), false);
  if (next) return next;
  return this;
}

void pidSCREEN::showCfgInfo(void) {
/*    Serial.print(F("Temp set: "));
    Serial.print(temp_set, DEC);
    Serial.print(F(", fan speed = "));
    Serial.print(pHG->getFanSpeed());
    Serial.print(F(", PID: ["));
    for (byte i = 1; i < 4; ++i) {
        int k = pHG->changePID(i, -1);
        Serial.print(k, DEC);
        if (i < 3) Serial.print(", ");
    }
    Serial.print("]; ");*/
}
#endif
//---------------------------------------- class fanSCREEN [ fan threshold calibration ] -------------------------------
class fanSCREEN : public SCREEN {
    public:
        fanSCREEN(HOTGUN* HG, DSPL* DSP, ENCODER* Enc, HOTGUN_CFG* Cfg) {
            pHG     = HG;
            pD      = DSP;
            pEnc    = Enc;
            pCfg    = Cfg;
        }
        virtual void    init(void);
        virtual SCREEN* show(void);
        virtual void    rotaryValue(int16_t value);
        virtual SCREEN* menu(void);
        virtual SCREEN* menu_long(void);
    private:
        HOTGUN*         pHG;                                                // Pointer to the HOTGUN instance
        DSPL*           pD;                                                 // Pointer to the DSPLay instance
        ENCODER*        pEnc;                                               // Pointer to the rotary encoder instance
        HOTGUN_CFG*     pCfg;                                               // Pointer to the config instance
        bool            modeSel;                                            // Which parameter to change: fan_min, fan_max
        uint16_t        preset_temp;                                        // The preset temp in human readable units
        bool            ready;                                              // Whether the temperature has been established
        uint8_t         fanSet;                                             // Which parameter is modifiying
        const uint32_t  period   = 1000;                                    // Update screen period
        const uint16_t  temp_max = 900;
        uint16_t        fan_min;
        uint16_t        fan_max;
};

void fanSCREEN::init(void) {
    fan_min = pHG->getFanCurrentMinThreshold();
    fan_max = pHG->getFanCurrentMaxThreshold();
    modeSel = true;                                                         // TRUE - select the element to change [fan_min, fan_max],  FALSE - change the selected value
    fanSet = 0;                                                             // 0 - fan_min, 1 - fan_max
    pEnc->reset(0, 0, 1, 1, 1, true);                                       // 0 - fan_min, 1 - fan_max
    pHG->switchPower(false);
    pD->clear();
    //pD->msgFanTune(modeSel, fanSet, fan_min, fan_max, 0);
    forceRedraw();
}

SCREEN* fanSCREEN::show(void){
    if (millis() < update_screen) return this;
    update_screen = millis() + period;
    pD->msgFanTune(modeSel, fanSet, fan_min, fan_max, pHG->getFanCurrent());
    return this;
}

SCREEN* fanSCREEN::menu(void){
    if (modeSel) {                                                         // Prepare to select which value to adjust
        uint16_t fanThr;
        if (fanSet == 0) {
            fanThr = pHG->getFanCurrentMinThreshold();
            pHG->setFanDuty(min_working_fan);
        }
        else {
            fanThr = pHG->getFanCurrentMaxThreshold();
            pHG->setFanDuty(max_working_fan);
        }
        pEnc->reset(fanThr, 0, 255, 1, 5);
        modeSel = false;
    } else {                                                                // Prepare to adjust the preset value
        pEnc->reset(0, 0, 1, 1, 1, true);                                   // 0 - fan_min, 1 - fan_max
        pHG->setFanDuty(0);                                                 // power off fan
        modeSel = true;
    }
    forceRedraw();
    return this;
}

SCREEN* fanSCREEN::menu_long(void){
    //pHG->switchPower(false);
    pHG->setFanDuty(0); // power off fan
    //pCfg->applyFanData(fan_min, fan_max);
    pCfg->saveFanData(fan_min, fan_max, true);    // update threshold in config struct and write to eeprom
    pHG->setFanCurrentThreshold(fan_min, fan_max); // update threshold in hotgan class, needed in keepTemp for runtime fan check
    if (next) return next;
    return this;
}

void fanSCREEN::rotaryValue(int16_t value){
    update_screen = millis() + period;
    if (modeSel) {                                                            // select the value to be changed, fan_min, fan_max
        fanSet = value;
        if (fanSet > 1) fanSet = 1;
    }
    else {
        if (fanSet == 0) {
            fan_min = value;
        }
        else {
            fan_max = value;
        }
    }
    forceRedraw();
}

//=========================================================================================================
HOTGUN      hg(/*FAN_GUN_SENS_PIN,*/ HOT_GUN_PIN, AC_RELAY_PIN, H_LENGTH_POWER, H_LENGTH_TEMP, H_LENGTH_FAN);
DSPL        disp;
//ENCODER     rotEncoder(R_MAIN_PIN, R_SECD_PIN, 0 /*init position*/, 600 /*fast change time*/, 1000 /*over pressed*/);
ENCODER     rotEncoder(R_MAIN_PIN, R_SECD_PIN, 0 /*init position*/, 12 /*fast change speed in steps per second*/, 0 /* not used */);
//BUTTON      rotButton(R_BUTN_PIN, 3000, 200, 900, 50);
#ifdef DISABLE_AC_CHECK
mdPushButton rotButton = mdPushButton(R_BUTN_PIN, LOW, true);
#else
mdPushButton rotButton = mdPushButton(R_BUTN_PIN, LOW, false);
#endif
SWITCH      reedSwitch(REED_SW_PIN);
HOTGUN_CFG  hgCfg;
BUZZER      simpleBuzzer(BUZZER_PIN);

mainSCREEN   offScr(&hg,  &disp, &rotEncoder, &simpleBuzzer, &hgCfg);
workSCREEN   wrkScr(&hg,  &disp, &rotEncoder, &simpleBuzzer, &hgCfg);
configSCREEN cfgScr(&hg,  &disp, &rotEncoder, &hgCfg);
errorSCREEN  errScr(&hg,  &disp, &simpleBuzzer);
#ifdef PID_SCREEN
pidSCREEN    pidScr(&hg,  &disp, &rotEncoder, &hgCfg);
#endif
fanSCREEN    fanScr(&hg,  &disp, &rotEncoder, &hgCfg);

SCREEN  *pCurrentScreen = &offScr;

volatile bool   end_of_power_period = false;

void syncAC(void) {
    end_of_power_period = hg.syncCB();
}

void rotEncChange(void) {
    rotEncoder.changeINTR();
}

// Initialize watchdog with 500ms timeout
void watchdog_init() {
    // Enable the watchdog timer, but only for the interrupt.
    // Take care, as this requires the correct order of operation, with interrupts disabled.
    // See the datasheet of any AVR chip for details.
    wdt_reset();
    cli();
    _WD_CONTROL_REG = _BV(_WD_CHANGE_BIT) | _BV(WDE);
    //_WD_CONTROL_REG = _BV(WDIE) | (1 << WDP2) | (1 << WDP0); // enable interrupt, no system reset mode, 0.5s time-out
    _WD_CONTROL_REG = _BV(WDIE) | (1 << WDP2) | (1 << WDP1); // enable interrupt, no system reset mode, 1s time-out
    sei();
    wdt_reset();
    //delay(800); // test it!
}

void watchdog_disable() {
    /* disable in terrupt */
    cli();
    /* clear MCU status register */
    MCUSR = 0;
    /* Disable an clear all watchdog settings */
    _WD_CONTROL_REG = (1 << WDCE) | (1 << WDE);
    _WD_CONTROL_REG = 0;
    /* enable interrupt */
    sei();
}

void setup() {
  watchdog_disable();
  
#if defined (DEBUG_PRINT_ENABLE)
    Serial.begin(115200);
    printDebugLn("setup...");
#endif
    uint8_t fan_current_min, fan_current_max;
    boolean cfgLoad;
    disp.init();
    
#ifdef DISABLE_AC_CHECK
    // Initialize rotary encoder
    rotEncoder.init();
//    rotButton.init();
#else
    // Initialize rotary encoder
    rotEncoder.init(false);
//    rotButton.init(false);
#endif
    rotButton.setMultiClickTime(150);
    rotButton.setHoldTime(1000);
    
    pCurrentScreen = &errScr; // set error screen as  default, if everything OK will be changed at the end of the setup
    
    delay(100);
    int buttonState = digitalRead(R_BUTN_PIN); // pressed = LOW, open = HIGH
    delay(500);
    if (buttonState == LOW) { // button pressed
        if (digitalRead(R_BUTN_PIN) == LOW) { // still pressed
            // eeprom clear
            disp.message("Resetting", "Memory");
            pCurrentScreen->init();
            hgCfg.clear();
            disp.clear();
        }
    }

    // Load configuration parameters
    cfgLoad = hgCfg.init();
    hg.init();
    uint16_t temp   = hgCfg.tempPreset();
    uint8_t  fan    = hgCfg.fanPreset();
    hg.setTemp(temp);
    hg.setFanSpeed(fan);
    hg.changePID(1, hgCfg.getPidKp()); 
    hg.changePID(2, hgCfg.getPidKi()); 
    hg.changePID(3, hgCfg.getPidKd());
    hg.setFanCurrentThreshold(hgCfg.getMinFanSpeedSens(),hgCfg.getMaxFanSpeedSens());
    
    reedSwitch.init(REED_ON_TIME_DELAY, REED_OFF_TIME_DELAY);

//    attachInterrupt(digitalPinToInterrupt(R_MAIN_PIN), rotEncChange,   CHANGE);
    attachInterrupt(digitalPinToInterrupt(AC_SYNC_PIN), syncAC, RISING);

    /* enable pin change interrupt on pin 3 and 5 for rotary encoder */
    PCICR  |= (1 << PCIE2);    // This enables Pin Change Interrupt 2 that covers port D with pin 0-7, PCINT16-PCINT23
    PCMSK2 |= (1 << PCINT19) | (1 << PCINT21);  // This enables the interrupt for pin 3 and 5 of Port D

    // Initialize SCREEN hierarchy
    offScr.next     = &cfgScr;
    offScr.on       = &wrkScr;
    wrkScr.next     = &offScr;
    cfgScr.next     = &offScr;
#ifdef PID_SCREEN
    cfgScr.pid     = &pidScr;//tuneScr;
#else
    cfgScr.pid     = &cfgScr;
#endif
    cfgScr.fan      = &fanScr;
#ifdef PID_SCREEN
    pidScr.next    = &cfgScr;
#endif
    errScr.next     = &offScr;
    fanScr.next     = &offScr;//&cfgScr;
    
    //thermocouple.begin();
    
    // safety check: detect gun by reading reed wsitch status. When connected and in the cradle it must be LOW.
    // If status is HIGH the gun can be either not connected or connected but not in the cradle.
    // Do not contiune until the status is LOW!
    if (reedSwitch.status() != LOW) {
    //while(reedSwitch.status() != LOW){
       //pCurrentScreen = &errScr;
       disp.message(failText, "GUN");
       pCurrentScreen->init();
       //delay(1000);
       //uint32_t cnt=0;
       while(reedSwitch.status() != LOW /*&& cnt < 10*/){
           delay(50);
           //cnt = cnt + ((reedSwitch.status() == LOW) ? 1 : 0);
       }
    }

#ifndef DISABLE_K_TH_CHECK    
    // K-type Thermocouple safety check
    //uint8_t queueId;
    //for(queueId=0; queueId<(H_LENGTH_TEMP-1); queueId++) { // leave last idx free for next read
        int16_t readTemp = (int16_t)(thermocouple.readTempC());
        if(/*MAX6675_THERMOCOUPLE_OPEN == readTemp ||*/ readTemp < 0) {
           disp.message(failText, "K - Therm");
           pCurrentScreen->init();
           while(1)
               delay(500);
        }
        //else {
           hg.fillTempQueue((uint16_t)readTemp);
        //}
    //}
#endif

#ifndef DISABLE_FAN_CHECK    
    //fan safety check
    hg.setFanDuty(max_working_fan);
    delay(3000);
    hg.fanAdcStart();
    fan_current_max = hg.readFanCurrent();
    hg.setFanDuty(min_working_fan);
    delay(3000);
    hg.fanAdcStart();
    fan_current_min = hg.readFanCurrent();
    hg.setFanDuty(0);

    //cfgLoad = false; //for test only
    if(cfgLoad) {
        if((fan_current_min >= fan_current_max) || (fan_current_min < (hg.getFanCurrentMinThreshold())) || (fan_current_max > (hg.getFanCurrentMaxThreshold()))) {
           //pCurrentScreen = &errScr;
           disp.message(failText, "FAN");
           pCurrentScreen->init();
           while(1)
               delay(500);
        }
        pCurrentScreen = &offScr;
        pCurrentScreen->init();   
    }
    // if not set, go to fan menu to set min/max values 
    else {
        pCurrentScreen = &fanScr;
        pCurrentScreen->init();
    }
#else
    pCurrentScreen = &offScr;
    pCurrentScreen->init();
#endif

    watchdog_init();
}

void loop(){
  //Serial.println("loop");
    static bool     reset_encoder   = true;
    static int16_t  old_pos         = 0;
    static bool     old_reed_status = false;
#ifdef DISABLE_AC_CHECK
 static uint32_t ac_check        = 1000;
#else
    static uint32_t ac_check        = 5000;
#endif
    int8_t hgError;
    SCREEN* nxt = pCurrentScreen;
    
    int16_t pos = rotEncoder.read();
    if (reset_encoder) {
        old_pos = pos;
        reset_encoder = false;
    } else {
        if (old_pos != pos) {
            pCurrentScreen->rotaryValue(pos);
            old_pos = pos;
        }
    }

    bool reed_status = reedSwitch.status();
    if(old_reed_status != reed_status) {
      nxt = pCurrentScreen->reedSwitch(reedSwitch.status());
      old_reed_status = reed_status;
      if (nxt != pCurrentScreen) {
          //Serial.println("reedSwitch change");
          pCurrentScreen = nxt;
          pCurrentScreen->init();
          reset_encoder = true;
          return;
      }
    }
//    uint8_t bStatus = rotButton.buttonCheck();
    int bStatus = rotButton.status();
    switch (bStatus) {
        case -1:  // long press 
            //Serial.println("Long button press");
            nxt = pCurrentScreen->menu_long();
            if (nxt != pCurrentScreen) {
                pCurrentScreen = nxt;
                pCurrentScreen->init();
                //reset_encoder = true;
            }
            reset_encoder = true;
            break;
        case  1:  // short press
            //Serial.println("Button pressed once");
            nxt = pCurrentScreen->menu();
            if (nxt != pCurrentScreen) {
                pCurrentScreen = nxt;
                pCurrentScreen->init();
                //reset_encoder = true;
            }
            reset_encoder = true;
            break;
        case  2:  // double short press
            //Serial.print("Button pressed "); Serial.print(bStatus); Serial.println(" times");
            nxt = pCurrentScreen->reedSwitch(true);
            if (nxt != pCurrentScreen) {
                pCurrentScreen = nxt;
                pCurrentScreen->init();
                reset_encoder = true;
                return;
                //Serial.println("this print should be skipped because of return");
            }
            break;
        case  0: // not pressed or more than 2 short press
        default:
            break;
    }
/*
    switch (bStatus) {
        case 2:                                                             // long press;
            nxt = pCurrentScreen->menu_long();
            if (nxt != pCurrentScreen) {
                pCurrentScreen = nxt;
                pCurrentScreen->init();
                reset_encoder = true;
            }
            break;
        case 1:                                                             // short press
            nxt = pCurrentScreen->menu();
            if (nxt != pCurrentScreen) {
                pCurrentScreen = nxt;
                pCurrentScreen->init();
                reset_encoder = true;
            }
            break;
        case 0:                                                             // Not pressed
        default:
            break;
    }
*/
    nxt = pCurrentScreen->show();
    if (nxt && pCurrentScreen != nxt) {                                     // Be paranoiac, the returned value must not be null
        pCurrentScreen = nxt;
        pCurrentScreen->init();
        reset_encoder = true;
    }
/*
    if (temp_update) {
        hg.readTemp();
        temp_update = false;
    }
    */
    if (end_of_power_period) {                                              // Calculate the required power
        hgError = hg.keepTemp();
        //Serial.print("hgERR ");
        //Serial.println(hgError);
        if(0 != hgError) {
            nxt = &errScr;
            if (nxt != pCurrentScreen) {
                pCurrentScreen = nxt;
                if (hgError == -1) {
                    disp.message(failText, "TEMP");
                }
                else
                {
                    disp.message(failText, "FAN");
                }
                pCurrentScreen->init();
                reset_encoder = true;
            }
        }
        end_of_power_period = false;
    }

#ifdef SIMULATE_SYNC_AC
    if (millis() > ac_check) {
        syncAC();
        ac_check = millis() + 10;
#else
    if (millis() > ac_check) {
        ac_check = millis() + 1000;
#endif
#ifndef DISABLE_AC_CHECK
        if (!hg.areExternalInterrupts()) {
            nxt = &errScr;
            disp.message(failText, "AC");
            if (nxt != pCurrentScreen) {
                pCurrentScreen = nxt;
                pCurrentScreen->init();
                reset_encoder = true;
            }
        }
#endif
    }
    
    wdt_reset();

}

ISR(PCINT2_vect) {
static bool busy = false;
  if (!busy){
    busy = true;
    rotEncChange();
    busy = false;
  }
}

#define FORCE_INLINE  __attribute__((always_inline)) inline

//#define NOP __asm__ __volatile__ ("nop\n\t")
#define nop() __asm__ __volatile__("nop;\n\t":::)

/*
  Delay by the provided number of milliseconds.
  Thus, a 16 bit value will allow a delay of 0..65 seconds
  Makes use of the _delay_loop_2
  
  _delay_loop_2 will do a delay of n * 4 processor cycles.
  with f = F_CPU cycles per second,
  n = f / (1000 * 4 )
  with f = 16000000 the result is 4000
  with f = 1000000 the result is 250
  
  the millisec loop, gcc requires the following overhead:
  - movev 1
  - subwi 2x2
  - bne i 2
  ==> 7 cycles
  ==> must be devided by 4, rounded up 7/4 = 2
*/
void _MillisecDelay(uint16_t val)
{
  while( val != 0 )
  {
    _delay_loop_2( (F_CPU / 4000 ) -2);
    val--;
  }
}

/* delay by one micro second */
void _MicroDelay(void)
{
#if (F_CPU / 4000000 ) > 0 
  _delay_loop_2( (F_CPU / 4000000 ) );
#endif
}

/* delay by 10 micro seconds */
void _10MicroDelay(void)
{
#if (F_CPU / 400000 ) > 0 
  _delay_loop_2( (F_CPU / 400000 ) );
#endif
}

ISR(WDT_vect) {
    //sei();  // With the interrupt driven serial we need to allow interrupts.
    // poweroff
    //wdt_disable();     
    hg.shutdown();
    // fan at full power to prevent overheating
    digital_write(FAN_GUN_PIN, HIGH);//hg.setFanDuty(max_working_fan);
    //Serial.println("WATCHDOG_FIRED");
    disp.message(failText, "WDT");
    pCurrentScreen = &errScr;
    pCurrentScreen->init();
    cli(); // Stop interrupts
    // wait some time to cool it down if necessary, if we are here we don't know what happened, just wait some time
    for (int i = 500; i>0 ; i--) { // about 120 seconds delay
        wdt_reset(); // avoid firing another watchdog during wait time
        //NOP;
        _MillisecDelay(250);
    }
    resetFunc();  //call reset to auto-reboot Arduino
}
