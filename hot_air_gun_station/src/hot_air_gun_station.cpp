/*
 * Hot air gun controller based on atmega328 IC
 * Version 1.4a
 * Released Dec 19, 2020
 */
#include <avr/io.h>
#include <avr/interrupt.h>
#include <Wire.h>
//#include <LiquidCrystal_I2C.h>
#include <CommonControls.h>
#include <EEPROM.h>
#include <SPI.h>
#include <TFT_ST7735.h>
//#include <Adafruit_MAX31855.h>
#include <MAX6675.h>


#define SLOW_D9_PWM_FREQUENCY
const uint8_t SCREEN_WIDTH  = ST7735_TFTWIDTH; //160;                                          //  display width, in pixels
const uint8_t SCREEN_HEIGHT = ST7735_TFTHEIGHT; //128;                                          //  display height, in pixels
const uint8_t FONT_WIDTH    = 8;
const uint8_t FONT_HEIGHT   = 8;
const char DEGREE_CHAR      = 248; //176
const char FAN_CHAR         = 70;//15; //70
const char POWER_CHAR       = 80;//232; //80
const char ARROW            = 62;
#define setCharCursor(x, y) setCursor((x)*FONT_WIDTH, (y)*FONT_HEIGHT)
const uint16_t temp_minC    = 100;
const uint16_t temp_maxC    = 500;
const uint16_t temp_ambC    = 25;
const uint16_t temp_tip[3]  = {200, 300, 400};                               // Temperature reference points for calibration
const uint16_t min_working_fan = 114;                                       // Minimal possible fan speed

const uint8_t AC_SYNC_PIN   = 2;                                            // Outlet 220 v synchronization pin. Do not change!
const uint8_t HOT_GUN_PIN   = 7;                                            // Hot gun heater management pin
const uint8_t FAN_GUN_PIN   = 9;                                            // Hot gun fan management pin. Do not change!
const uint8_t FAN_GUN_SENS_PIN  = A0;                                       // Hot gun fan checking pin
const uint8_t AC_RELAY_PIN  = A2;

const uint8_t R_MAIN_PIN    = 3;                                            // Rotary encoder main pin. Do not change!
const uint8_t R_SECD_PIN    = 4;                                            // Rotary encoder secondary pin
const uint8_t R_BUTN_PIN    = 5;                                            // Rotary encoder button pin

const uint8_t REED_SW_PIN   = 8;                                            // Reed switch pin
const uint8_t BUZZER_PIN    = 6;                                            // Buzzer pin
const bool    BUZZER_ACTIVE = false;                                         // Active buzzer beeps when +5v supplied to it
#define MAXCS A3
// Initialize the Thermocouple
//Adafruit_MAX31855 thermocouple(MAXCS);
MAX6675 thermocouple(MAXCS);

uint16_t    fan_speed_raw;
//volatile bool   temp_update = false;

//------------------------------------------ Configuration data ------------------------------------------------
/* Config record in the EEPROM has the following format:
 * uint32_t ID                           each time increment by 1
 * struct cfg                            config data, 8 bytes
 * byte CRC                              the checksum
*/
struct cfg {
    uint32_t    calibration;                                                // Packed calibration data by three temperature points
    uint16_t    temp;                                                       // The preset temperature of the IRON in internal units
    uint8_t     fan;                                                        // The preset fan speed 0 - 255
    uint8_t     off_timeout;                                                // Automatic switch-off timeout
    uint16_t    minFanSpeedSens;                                            // min adc value to consider fan working
    uint16_t    maxFanSpeedSens;                                            // max adc value to consider fan working
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
        void     init(void);
        uint16_t tempPreset(void);                                          // The preset temperature in internal units
        uint8_t  fanPreset(void);                                           // The preset fan speed 0 - 255
        uint16_t tempInternal(uint16_t temp);                               // Translate the human readable temperature into internal value
        uint16_t tempHuman(uint16_t temp);                                  // Translate temperature from internal units to the Celsius
        void     save(uint16_t temp, uint8_t fanSpeed);                     // Save preset temperature in the internal units and fan speed
        void     applyCalibrationData(uint16_t tip[3]);
        void     getCalibrationData(uint16_t tip[3]);
        void     saveCalibrationData(uint16_t tip[3]);
        void     applyFanData(uint16_t fan_min, uint16_t fan_max);
        void     saveFanData(uint16_t fan_min, uint16_t fan_max);
        void     setDefaults(bool Write);                                   // Set default parameter values if failed to load data from EEPROM
        uint16_t getMinFanSpeedSens(void);
        uint16_t getMaxFanSpeedSens(void);
    private:
        uint16_t t_tip[3];
        uint16_t minFanSpeedSens;
        uint16_t maxFanSpeedSens;
        const   uint16_t def_tip[3] = {200, 300, 400};//{55, 445, 900};//{587, 751, 850};                      // Default values of internal sensor readings at reference temperatures
        const   uint16_t min_temp  = temp_minC;
        const   uint16_t max_temp  = temp_maxC;
        const   uint16_t def_temp  = 190;                                   // Default preset temperature
        const   uint8_t  def_fan   = 225;                                   // Default preset fan speed 0 - 255
        const   uint16_t ambient_temp = temp_ambC;
        const   uint16_t ambient_tempC= temp_ambC;
        const   uint16_t def_minFanSpeedSens = 250;
        const   uint16_t def_maxFanSpeedSens = 1000;
};

void HOTGUN_CFG::clear(void) {
    CONFIG::clear();
    return;
}

void HOTGUN_CFG::init(void) {
    CONFIG::init();
    if (!CONFIG::load()) setDefaults(false);                                // If failed to load the data from EEPROM, initialize the config data with the default values
    uint32_t   cd = Config.calibration;
    t_tip[0] = cd & 0x3FF; cd >>= 10;                                       // 10 bits per calibration parameter, because the ADC readings are 10 bits
    t_tip[1] = cd & 0x3FF; cd >>= 10;
    t_tip[2] = cd & 0x3FF;
    // Check the tip calibration is correct
    if ((t_tip[0] >= t_tip[1]) || (t_tip[1] >= t_tip[2])) {
        setDefaults(false);
        for (uint8_t i = 0; i < 3; ++i)
            t_tip[i] = def_tip[i];
    }
    minFanSpeedSens = Config.minFanSpeedSens;
    maxFanSpeedSens = Config.maxFanSpeedSens;
    return;
}
    uint32_t    calibration;                                                // Packed calibration data by three temperature points
    uint16_t    temp;                                                       // The preset temperature of the IRON in internal units
    uint8_t     fan;                                                        // The preset fan speed 0 - 255
    uint8_t     off_timeout;                                                // Automatic switch-off timeout
    uint16_t    minFanSpeedSens;                                            // min adc value to consider fan working
    uint16_t    maxFanSpeedSens;                                            // max adc value to consider fan working

uint16_t HOTGUN_CFG::tempPreset(void) {
    return Config.temp;
}

uint8_t HOTGUN_CFG::fanPreset(void) {
    return Config.fan;
}

uint16_t HOTGUN_CFG::tempInternal(uint16_t t) {                             // Translate the human readable temperature into internal value

    uint16_t temp = t;
    t = constrain(t, temp_minC, temp_maxC);

    return t;

    if (t >= temp_tip[1])
        temp = map(t+1, temp_tip[1], temp_tip[2], t_tip[1], t_tip[2]);
    else
        temp = map(t+1, temp_tip[0], temp_tip[1], t_tip[0], t_tip[1]);

    for (uint8_t i = 0; i < 10; ++i) {
        uint16_t tH = tempHuman(temp);
        if (tH <= t) break;
        --temp;
    }
    return temp;
}

// Thanslate temperature from internal units to the human readable value (Celsius or Fahrenheit)
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

void HOTGUN_CFG::save(uint16_t temp, uint8_t fanSpeed) {
    Config.temp        = constrain(temp, min_temp, max_temp);
    Config.fan         = fanSpeed;
    CONFIG::save();                                                         // Save new data into the EEPROM
}

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

void HOTGUN_CFG::applyFanData(uint16_t fan_min, uint16_t fan_max) {
    minFanSpeedSens = fan_min;
    maxFanSpeedSens = fan_max;
}

void HOTGUN_CFG::saveFanData(uint16_t fan_min, uint16_t fan_max) {
    Config.minFanSpeedSens = fan_min;
    Config.maxFanSpeedSens = fan_max;
    CONFIG::save();
}

void HOTGUN_CFG::setDefaults(bool Write) {
    uint32_t c = def_tip[2] & 0x3FF; c <<= 10;
    c |= def_tip[1] & 0x3FF;         c <<= 10;
    c |= def_tip[0] & 0x3FF;
    Config.calibration = c;
    Config.temp        = def_temp;
    Config.fan         = def_fan;
    Config.minFanSpeedSens = def_minFanSpeedSens;
    Config.maxFanSpeedSens = def_maxFanSpeedSens;
    if (Write) {
        CONFIG::save();
    }
}

uint16_t HOTGUN_CFG::getMinFanSpeedSens(void) {
    return minFanSpeedSens;
}

uint16_t HOTGUN_CFG::getMaxFanSpeedSens(void) {
    return maxFanSpeedSens;
}

//------------------------------------------ class BUZZER ------------------------------------------------------
class BUZZER {
  public:
        BUZZER(byte buzzerP, bool active = true)
                            { buzzer_pin = buzzerP; this->active = active; }
    void init(void);
    void shortBeep(void);
    void lowBeep(void);
    void doubleBeep(void);
    void failedBeep(void);
  private:
    byte buzzer_pin;
        bool active;
};

void BUZZER::init(void) {
    pinMode(buzzer_pin, OUTPUT);
    if (active) {
        digitalWrite(buzzer_pin, LOW);
    } else {
    noTone(buzzer_pin);
    }
}

void BUZZER::shortBeep(void) {
    if (active) {
       digitalWrite(buzzer_pin, HIGH);
       delay(80);
       digitalWrite(buzzer_pin, LOW);
    } else {
        tone(buzzer_pin, 3520, 160);
    }
}

void BUZZER::lowBeep(void) {
    if (active) {
        digitalWrite(buzzer_pin, HIGH);
        delay(160);
        digitalWrite(buzzer_pin, LOW);
    } else {
        tone(buzzer_pin,  880, 160);
    }
}

void BUZZER::doubleBeep(void) {
    if (active) {
        digitalWrite(buzzer_pin, HIGH);
        delay(160);
        digitalWrite(buzzer_pin, LOW);
        delay(150);
        digitalWrite(buzzer_pin, HIGH);
        delay(160);
        digitalWrite(buzzer_pin, LOW);
    } else {
        tone(buzzer_pin, 3520, 160);
        delay(300);
        tone(buzzer_pin, 3520, 160);
    }
}

void BUZZER::failedBeep(void) {
    if (active) {
        digitalWrite(buzzer_pin, HIGH);
        delay(170);
        digitalWrite(buzzer_pin, LOW);
        delay(10);
        digitalWrite(buzzer_pin, HIGH);
        delay(80);
        digitalWrite(buzzer_pin, LOW);
        delay(100);
        digitalWrite(buzzer_pin, HIGH);
        delay(80);
        digitalWrite(buzzer_pin, LOW);
    } else {
        tone(buzzer_pin, 3520, 160);
        delay(170);
        tone(buzzer_pin,  880, 250);
        delay(260);
        tone(buzzer_pin, 3520, 160);
    }
}

//------------------------------------------ class lcd DSPLay for soldering IRON -----------------------------
class DSPL : protected TFT_ST7735 {
    public:
        DSPL(void) : TFT_ST7735() { }
        void    init(void);
        void    clear(void)                                                 { TFT_ST7735::fillScreen(0x5AEB); setCharCursor(0, 0);}
        void    tSet(uint16_t t, uint16_t tInt = 1234, bool Celsius = true);                      // Show the preset temperature
        void    tCurr(uint16_t t, uint16_t tInt = 1234);                                          // Show the current temperature
        void    tInternal(uint16_t t);                                      // Show the current temperature in internal units
        void    tReal(uint16_t t);                                          // Show the real temperature in Celsius in calibrate mode
        void    fanSpeed(uint8_t s);                                        // Show the fan speed
        void    appliedPower(uint8_t p, bool show_zero = true);             // Show applied power (%)
        void    setupMode(uint8_t mode);
        void    msgON(void);                                                // Show message: "ON"
        void    msgOFF(void);
        void    msgReady(void);
        void    msgCold(void);
        void    msgFail(void);                                              // Show 'Fail' message
        void    msgTune(void);                                              // Show 'Tune' message
        void    msgTip(uint16_t tip0, uint16_t tip1, uint16_t tip2);
        void    msgFanTune(uint8_t sel, uint16_t min, uint16_t max);
    private:
        bool    full_second_line;                                           // Whether the second line is full with the message
        char    temp_units;
  /*      const   uint8_t custom_symbols[3][8] = {
                          { 0b00110,                                        // Degree
                            0b01001,
                            0b01001,
                            0b00110,
                            0b00000,
                            0b00000,
                            0b00000,
                            0b00000
                          },
                          { 0b00100,                                        // Fan sign
                            0b01100,
                            0b01100,
                            0b00110,
                            0b01011,
                            0b11001,
                            0b10000,
                            0b00000
                          },
                          { 0b00011,                                        // Power sign
                            0b00110,
                            0b01100,
                            0b11111,
                            0b00110,
                            0b01100,
                            0b01000,
                            0b10000
                          }
                        };*/
};

void DSPL::init(void) {
    //LiquidCrystal_I2C::begin();
    clear();
    //for (uint8_t i = 0; i < 3; ++i)
    //    LiquidCrystal_I2C::createChar(i+1, (uint8_t *)custom_symbols[i]);
    TFT_ST7735::init();
    TFT_ST7735::setRotation(1);
    TFT_ST7735::fillScreen(0x5AEB); // grey color code
    TFT_ST7735::setCursor(0, 0, 1); // Set "cursor" at top left corner of display (0,0) and select font 1
    TFT_ST7735::setTextColor(TFT_WHITE);
    TFT_ST7735::setTextColor(TFT_WHITE,TFT_BLACK);
    TFT_ST7735::setTextFont(1);
    TFT_ST7735::setTextSize(1);
    full_second_line = false;
    temp_units = 'C';
}

void DSPL::msgTip(uint16_t tip0, uint16_t tip1, uint16_t tip2){
    char buff[17];
    setCharCursor(0, 4);
    sprintf(buff, "Cal: %3d %3d %3d", tip0, tip1, tip2);
    print(buff);
}

void DSPL::tSet(uint16_t t, uint16_t tInt, bool Celsius) {
    char buff[10];
    char buff1[12];
    if (Celsius) {
        temp_units = 'C';
    } else {
        temp_units = 'F';
    }
    setCharCursor(0, 0);
    sprintf(buff, "Set:%3d%c%c", t, DEGREE_CHAR, temp_units);
    print(buff);
    //setCharCursor(0, 2);
    //sprintf(buff1, "SetInt:%4d", tInt);
    //print(buff1);
}

void DSPL::tCurr(uint16_t t, uint16_t tInt) {
    char buff[6];
    char buff1[12];
    setCharCursor(0, 1);
    if (t < 1000) {
        sprintf(buff, "%3d%c ", t, DEGREE_CHAR);
    } else {
        print(F("xxx"));
        return;
    }
    print(buff);
    //setCharCursor(0, 3);
    //sprintf(buff1, "CurInt:%4d", tInt);
    //print(buff1);
    if (full_second_line) {
        print(F("           "));
        full_second_line = false;
    }
}

void DSPL::tInternal(uint16_t t) {
    char buff[6];
    setCharCursor(0, 1);
    if (t < 1023) {
        sprintf(buff, "%4d ", t);
    } else {
        print(F("xxxx"));
        return;
    }
    print(buff);
    if (full_second_line) {
        print(F("           "));
        full_second_line = false;
    }
}

void DSPL::tReal(uint16_t t) {
    char buff[6];
    setCharCursor(11, 1);
    if (t < 1000) {
        sprintf(buff, ">%3d%c", t, DEGREE_CHAR);
    } else {
        print(F("xxx"));
        return;
    }
    print(buff);
}

void DSPL::fanSpeed(uint8_t s) {
    char buff[7];
    sprintf(buff, " %c%3d%c", FAN_CHAR, s, '%');
    setCharCursor(11, 1);
    print(buff);
    sprintf(buff, "%4d ", fan_speed_raw);
    setCharCursor(11, 2);
    print(buff);
}

void DSPL::appliedPower(uint8_t p, bool show_zero) {
    char buff[6];
    if (p > 99) p = 99;
    setCharCursor(5, 1);
    if (p == 0 && !show_zero) {
        print(F("     "));
    } else {
        sprintf(buff, " %c%2d%c", POWER_CHAR, p, '%');
        print(buff);
    }
}

void DSPL::setupMode(byte mode) {
    clear();
    print(F("Setup"));
    setCharCursor(1,1);
    switch (mode) {
        case 0:                                                             // tip calibrate
            print(F("calibrate"));
            break;
        case 1:                                                             // tune
            print(F("tune"));
            break;
        case 2:
            print(F(("fan")));
            break;
        case 3:                                                             // save
            print(F("save"));
            break;
        case 4:                                                             // cancel
            print(F("cancel"));
            break;
        case 5:                                                             // set defaults
            print(F("reset config"));
            break;
        default:
            break;
    }
}

void DSPL::msgON(void) {
    setCharCursor(10, 0);
    print(F("    ON"));
}

void DSPL::msgOFF(void) {
    setCharCursor(10, 0);
    print(F("   OFF"));
}

void DSPL::msgReady(void) {
    setCharCursor(10, 0);
    print(F(" Ready"));
}

void DSPL::msgCold(void) {
    setCharCursor(10, 0);
    print(F("  Cold"));
}

void DSPL::msgFail(void) {
    setCharCursor(0, 1);
    print(F(" -== Failed ==- "));
}

void DSPL::msgTune(void) {
    setCharCursor(0, 0);
    print(F("Tune"));
}

void DSPL::msgFanTune(uint8_t sel, uint16_t min, uint16_t max)  {
    char buff[20];
    setCharCursor(0, 0);
    print(F("Fan Tune"));
    if (0 == sel) {
        setCharCursor(0, 1);
        sprintf(buff, "%c min: %4d", ARROW, min);
        print(buff);
        setCharCursor(0, 2);
        sprintf(buff, "  max: %4d", max);
        print(buff);
    }
    else {
        setCharCursor(0, 1);
        sprintf(buff, "  min: %4d", min);
        print(buff);
        setCharCursor(0, 2);
        sprintf(buff, "%c max: %4d", ARROW, max);
        print(buff);
    }
}

//------------------------------------------ class HISTORY ----------------------------------------------------
#define H_LENGTH 16
class HISTORY {
    public:
        HISTORY(void)                                                       { len = 0; }
        void     init(void)                                                 { len = 0; }
        uint16_t last(void);
        uint16_t top(void)                                                  { return queue[0]; }
        void     put(uint16_t item);                                        // Put new entry to the history
        uint16_t average(void);                                             // calculate the average value
        float    dispersion(void);                                          // calculate the math dispersion
    private:
        volatile uint16_t queue[H_LENGTH];
        volatile byte len;                                                  // The number of elements in the queue
        volatile byte index;                                                // The current element position, use ring buffer
};

void HISTORY::put(uint16_t item) {
    if (len < H_LENGTH) {
        queue[len++] = item;
    } else {
        queue[index ] = item;
        if (++index >= H_LENGTH) index = 0;                                 // Use ring buffer
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
        PID(void) {
            Kp = 638;
            Ki = 196;
            Kd =   1;
        }
        void resetPID(int temp = -1);                                       // reset PID algorithm history parameters
        // Calculate the power to be applied
        long reqPower(int temp_set, int temp_curr);
        int  changePID(uint8_t p, int k);                                   // set or get (if parameter < 0) PID parameter
    private:
        void  debugPID(int t_set, int t_curr, long kp, long ki, long kd, long delta_p);
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
        void duty(uint8_t d)                        { OCR1A = d; }
        uint8_t     fanSpeed(void)                  { /*TODO: get fan speed  from sensing resistor here*/ return 255; /*analogRead(sen_pin);*/ }
};

void FastPWM_D9::init(void) {
    pinMode(9, OUTPUT);
    digitalWrite(9, LOW);
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
        typedef enum { POWER_OFF, POWER_ON, POWER_FIXED, POWER_COOLING } PowerMode;
        HOTGUN(uint8_t HG_sen_pin, uint8_t HG_pwr_pin, uint8_t HG_ACrelay_pin);
        void        init(void);
        bool        isOn(void)                                              { return (mode == POWER_ON || mode == POWER_FIXED);             }
        void        setTemp(uint16_t t)                                     { temp_set = constrain(t, temp_minC, temp_maxC); }
        uint16_t    getTemp(void)                                           { return temp_set; }
        uint16_t    getCurrTemp(void)                                       { return h_temp.last(); }
        uint16_t    tempAverage(void)                                       { return h_temp.average(); }
        uint8_t     powerAverage(void)                                      { return h_power.average(); }
        uint8_t     appliedPower(void)                                      { return actual_power; }
        void        setFanSpeed(uint8_t f)                                  { fan_speed = constrain(f, min_working_fan, max_fan_speed);   }
        uint8_t     getFanSpeed(void)                                       { return fan_speed; }
        uint8_t	    presetFanPcnt(void);
        uint16_t    tempDispersion(void)                                    { return h_temp.dispersion(); }
        bool        isCold(void)                                            { return h_temp.average() < temp_gun_cold;                      }
        bool        areExternalInterrupts(void)                             { return millis() - last_period < period * 15;                  }
        uint8_t     avgPowerPcnt(void);
        void        switchPower(bool On);
        void        fixPower(uint8_t Power);                                // Set the specified power to the the hot gun
        void        keepTemp(void);
        void        readTemp(void);
        uint8_t     getMaxFixedPower(void)                                  { return max_fix_power; }
        bool        syncCB(void);                                           // Return true at the end of the power period
        void        activateRelay(bool activate);
    private:
        bool        isGunConnected(void)                                    { return true; }
        void        shutdown(void);
        uint16_t    emulateTemp(void);                                      // To debug the project, simulate the Hot Air Gun heating process
        FastPWM_D9  hg_fan;
        uint16_t    temp_set;                                               // The preset temperature of the hot air gun (internal units)
        uint8_t     fan_speed;
        uint16_t    fan_speed_sens;
        uint8_t     sen_pin;
        uint8_t     gun_pin;
        uint8_t     relay_pin;
        HISTORY     h_power;                                                // The history queue of power applied values
        HISTORY     h_temp;                                                 // The history queue of the temperature
        //EMP_AVERAGE e_sensor;                                               // The exponential average of sensor data
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
        const       uint16_t    min_fan_speed   = 30;
        const       uint16_t    max_fan_speed   = 255;
        const       uint16_t    max_cool_fan    = 220;
        const       uint16_t    temp_gun_cold   = 50;                       // The temperature of the cold iron
        const       uint8_t     e_sensor_length = 10;                       // Exponential average length of sensor data
        const       uint32_t    relay_activate  = 1;        // The relay activation delay
};

HOTGUN::HOTGUN(uint8_t HG_sen_pin, uint8_t HG_pwr_pin, uint8_t HG_ACrelay_pin) {
    sen_pin   = HG_sen_pin;
    gun_pin   = HG_pwr_pin;
    relay_pin = HG_ACrelay_pin;
}

void HOTGUN::init(void) {
    cnt         = 0;
    fan_speed       = 0;
    actual_power    = 0;
    fix_power       = 0;
    active      = false;
    chill       = false;
    last_period = 0;
    pinMode(sen_pin, INPUT);
    pinMode(gun_pin, OUTPUT);
    pinMode(relay_pin, OUTPUT);
    digitalWrite(gun_pin, LOW);
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
            digitalWrite(gun_pin, HIGH);
            active = true;
        }
    } else if (cnt >= actual_power) {
        if (active) {
            digitalWrite(gun_pin, LOW);
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
            if (hg_fan.fanSpeed() == 0) {                                   // Not power supplied to the Fan
                if (On)                                                     // !FAN && On
                    activateRelay(true);
                    mode = POWER_ON;
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
                activateRelay(false);
            }
            break;
        case POWER_FIXED:
            if (hg_fan.fanSpeed()) {
                if (On) {                                                   // FAN && On
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
            break;
        case POWER_COOLING:
            if (hg_fan.fanSpeed()) {
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

void HOTGUN::readTemp(void) {
    double c = thermocouple.readTempC();
    uint16_t temp;

    if (MAX6675_THERMOCOUPLE_OPEN == c || c <= 0) { //TODO: handle error condition!
        temp = h_temp.average();
    }
    else
    {
        temp = (uint16_t)(c);
    }
    //e_sensor.update(temp);
    h_temp.put(temp);
}

// This routine is used to keep the hot air gun temperature near required value
void HOTGUN::keepTemp(void) {
    long p = 0;
    readTemp();
    fan_speed_sens = analogRead(sen_pin);                                   // Check the hot air fan speed
    fan_speed_raw = fan_speed_sens;                                         // Check the hot air fan speed

    //uint16_t temp = analogRead(sen_pin);                                  // Check the hot air gun temperature
    //double c = thermocouple.readCelsius();
    //uint16_t temp = e_sensor.read();                                      // Average value of the hot air gun temperature
    //h_temp.put(temp);
    uint16_t temp = getCurrTemp();
    if (!chill && (mode == POWER_ON) && (temp > (temp_set + 8))) { // Prevent global over heating
        //digitalWrite(gun_pin, LOW);
        p = 0;
        chill = true;                                               // Turn off the power in main working mode only;
    }

    switch (mode) {
        case POWER_OFF:
             if (!isCold()) {                                               // FAN && connected && !cold
                        //uint16_t fan = map(temp, temp_gun_cold, temp_set, max_cool_fan, min_fan_speed);
                        fan = constrain(/*fan*/fan_speed, min_fan_speed, max_fan_speed);
                        hg_fan.duty(fan);
                        mode = POWER_COOLING;
                    }
            break;
        case POWER_ON:
            hg_fan.duty(fan_speed);                                         // Turn on the fan immediately
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
        case POWER_FIXED:
            if (relay_ready_cnt > 0) {
                --relay_ready_cnt;
            }
            else {
                p  = fix_power;
            }
            hg_fan.duty(fan_speed);
            break;
        case POWER_COOLING:
            if (hg_fan.fanSpeed() < min_fan_speed) {
                shutdown();
            } else {
                if (isGunConnected()) {
                    if (isCold()) {                                         // FAN && connected && cold
                        shutdown();
                    } else {                                                // FAN && connected && !cold
                        //uint16_t fan = map(temp, temp_gun_cold, temp_set, max_cool_fan, min_fan_speed);
                        fan = constrain(/*fan*/fan_speed, min_fan_speed, max_fan_speed);
                        hg_fan.duty(fan);
                    }
                } else {                                                    // FAN && !connected
                    shutdown();
                }
            }
            break;
        default:
            break;
    }
    h_power.put(p);
    actual_power = p;
    if (p == 0) {
        digitalWrite(gun_pin, LOW);
    }
}

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

uint8_t HOTGUN::avgPowerPcnt(void) {
    uint8_t pcnt = 0;
    if (mode == POWER_FIXED) {
        pcnt = map(fix_power, 0, max_fix_power, 0, 100);
    } else {
        pcnt = map(h_power.average(), 0, max_power, 0, 100);
    }
    if (pcnt > 100) pcnt = 100;
    return pcnt;
}

void HOTGUN::shutdown(void) {
    digitalWrite(gun_pin, LOW);
    hg_fan.duty(0);
    mode            = POWER_OFF;
    actual_power    = 0;
    active          = false;
    activateRelay(false);
}

uint16_t HOTGUN::emulateTemp(void) {
    static int16_t t = 0;
    uint8_t ap = actual_power;
    if (mode == POWER_FIXED)
        ap = fix_power;
    ap = constrain(ap, 0, 100);
    t += map(ap, 0, 100, 0, 30);
    uint8_t fn = hg_fan.fanSpeed();
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
        digitalWrite(relay_pin, HIGH);
        relay_ready_cnt = relay_activate;
    } else {
        digitalWrite(relay_pin, LOW);
        relay_ready_cnt = 0;
    }
}

uint8_t	HOTGUN::presetFanPcnt(void) {
	uint16_t pcnt = map(fan_speed, 0, max_fan_speed, 0, 100);
	if (pcnt > 100) pcnt = 100;
	return pcnt;
}

//------------------------------------------ class SCREEN ------------------------------------------------------
class SCREEN {
    public:
        SCREEN* next;                                                       // Pointer to the next screen
        SCREEN() {
            next            = 0;
            update_screen   = 0;
            scr_timeout     = 0;
            time_to_return  = 0;
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
        uint32_t scr_timeout;                                               // Timeout is sec. to return to the main screen, canceling all changes
        uint32_t time_to_return;                                            // Time in ms to return to main screen
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
    uint16_t tempH      = pCfg->tempHuman(temp_set);                        // The preset temperature in the human readable units
    pEnc->reset(tempH, temp_minC, temp_maxC, 1, 5);
    used = !pHG->isCold();
    cool_notified = !used;
    if (used) {                                                             // the hot gun was used, we should save new data in EEPROM
        pCfg->save(temp_set, pHG->getFanSpeed());
    }
    mode_temp = true;
    clear_used_ms = 0;
    pD->clear();
    forceRedraw();
}

void mainSCREEN::rotaryValue(int16_t value) {
    if (mode_temp) {                                                        // set hot gun temperature
        uint16_t temp = pCfg->tempInternal(value);
        pHG->setTemp(temp);
        pD->tSet(value, temp);
    } else {                                                                // set fan speed
        pHG->setFanSpeed(value);
        pD->fanSpeed(pHG->presetFanPcnt());
    }
    update_screen  = millis() + period;
}

SCREEN* mainSCREEN::show(void) {
    if (millis() < update_screen) return this;
    update_screen = millis() + period;

    if (clear_used_ms && (millis() > clear_used_ms)) {
        clear_used_ms = 0;
        used = false;
    }

    uint16_t temp_set = pHG->getTemp();
    pD->tSet(pCfg->tempHuman(temp_set),temp_set);
    uint16_t temp  = pHG->tempAverage();
    uint16_t tempH = pCfg->tempHuman(temp);
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
    pD->tCurr(tempH,temp);
    pD->appliedPower(0, false);
    pD->fanSpeed(pHG->presetFanPcnt());
    return this;
}

SCREEN* mainSCREEN::menu(void) {
    if (mode_temp) {                                                        // Prepare to adjust the fan speed
        uint8_t fs = pHG->getFanSpeed();
        pEnc->reset(fs, min_working_fan, 255, 1, 10);
        mode_temp = false;
    } else {                                                                // Prepare to adjust the preset temperature
        uint16_t temp_set   = pHG->getTemp();
        uint16_t tempH      = pCfg->tempHuman(temp_set);
        pEnc->reset(tempH, temp_minC, temp_maxC, 1, 5);
        mode_temp = true;
    }
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
    pEnc->reset(fs, min_working_fan, 255, 1, 10);
    mode_temp   = false;                                                    // By default adjust the fan speed
    pHG->switchPower(true);
    ready = false;
    pD->clear();
    forceRedraw();
}

void workSCREEN::rotaryValue(int16_t value) {                               // Setup new preset temperature by rotating the encoder
    if (mode_temp) {
        ready = false;
        uint16_t temp = pCfg->tempInternal(value);                          // Translate human readable temperature into internal value
        pHG->setTemp(temp);
        pD->tSet(value,temp);
    } else {
        pHG->setFanSpeed(value);
        pD->fanSpeed(pHG->presetFanPcnt());
    }
    update_screen = millis() + period;
}

SCREEN* workSCREEN::show(void) {
    if (millis() < update_screen) return this;
    update_screen = millis() + period;

    int temp_set  = pHG->getTemp();
    int tempH_set = pCfg->tempHuman(temp_set);
    pD->tSet(tempH_set,temp_set);
    int temp      = pHG->tempAverage();
    int tempH     = pCfg->tempHuman(temp);
    pD->tCurr(tempH,temp);
    pD->msgON();
    uint8_t p   = pHG->appliedPower();
    pD->appliedPower(p);
    pD->fanSpeed(pHG->presetFanPcnt());


//Serial.print("Diff = "); Serial.print(temp_set - temp);
//uint32_t disp = pHG->tempDispersion();
//Serial.print(", Dispersion = "); Serial.println(disp);

    if ((abs(temp_set - temp) < 5) && (pHG->tempDispersion() <= 20))  {
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
        pEnc->reset(fs, min_working_fan, 255, 1, 10);
        mode_temp = false;
    } else {
        uint16_t temp_set   = pHG->getTemp();
        uint16_t tempH      = pCfg->tempHuman(temp_set);
        pEnc->reset(tempH, temp_minC, temp_maxC, 1, 5);
        mode_temp = true;
    }
    return this;
}

SCREEN* workSCREEN::reedSwitch(bool on) {
    if (!on && next)
        return next;
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
        virtual void init(void)                                             { pHG->switchPower(false); pD->clear(); pD->msgFail(); pBz->failedBeep(); }
        virtual SCREEN* menu(void)                                          { if (this->next != 0)  return this->next;  else return this; }
    private:
        HOTGUN*     pHG;                                                    // Pointer to the got air gun instance
        DSPL*       pD;                                                     // Pointer to the display instance
        BUZZER*     pBz;                                                    // Pointer to the simple Buzzer instance
};

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
        virtual void    rotaryValue(int16_t value);
        SCREEN*         calib;                                              // Pointer to the calibration SCREEN
        SCREEN*         tune;                                               // Pointer to the tune SCREEN
        SCREEN*         fan;                                                // Pointer to fan SCREEN
    private:
        HOTGUN*     pHG;                                                    // Pointer to the HOTGUN instance
        DSPL*       pD;                                                     // Pointer to the DSPLay instance
        ENCODER*    pEnc;                                                   // Pointer to the rotary encoder instance
        HOTGUN_CFG* pCfg;                                                   // Pointer to the config instance
        uint8_t     mode;                                                   // 0 - hotgun calibrate, 1 - tune, 2 - save, 3 - cancel, 4 - defaults
        const uint16_t period = 10000;                                      // The period in ms to update the screen
};

void configSCREEN::init(void) {
    pHG->switchPower(false);
    mode = 0;
    pEnc->reset(mode, 0, 5, 1, 0, true);
    pD->clear();
    pD->setupMode(0);
    this->scr_timeout = 30;                                                 // This variable is defined in the superclass
}

SCREEN* configSCREEN::show(void) {
    if (millis() < update_screen) return this;
    update_screen = millis() + period;
    pD->setupMode(mode);
    return this;
}

SCREEN* configSCREEN::menu(void) {
    switch (mode) {
        case 0:                                                             // calibrate hotgun
            if (calib) return calib;
            break;
        case 1:                                                             // Tune potentiometer
            if (tune) return tune;
            break;
        case 2:
            if(fan) return fan;
            break;
        case 3:                                                             // Save configuration data
            menu_long();
        case 4:                                                             // Cancel, Return to the main menu
            if (next) return next;
            break;
        case 5:                                                             // Save defaults
            pCfg->setDefaults(true);
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

//---------------------------------------- class calibSCREEN [ tip calibration ] -------------------------------
class calibSCREEN : public SCREEN {
    public:
        calibSCREEN(HOTGUN* HG, DSPL* DSP, ENCODER* Enc, BUZZER* Buzz, HOTGUN_CFG* Cfg) {
            pHG     = HG;
            pD      = DSP;
            pEnc    = Enc;
            pCfg    = Cfg;
            pBz     = Buzz;
        }
        virtual void    init(void);
        virtual SCREEN* show(void);
        virtual void    rotaryValue(int16_t value);
        virtual SCREEN* menu(void);
        virtual SCREEN* menu_long(void);
    private:
        uint16_t        selectTemp(byte index);                             // Calculate the value of the temperature limit depending on mode
        void            buildCalibration(uint16_t tip[3]);
        HOTGUN*         pHG;                                                // Pointer to the HOTGUN instance
        DSPL*           pD;                                                 // Pointer to the DSPLay instance
        ENCODER*        pEnc;                                               // Pointer to the rotary encoder instance
        HOTGUN_CFG*     pCfg;                                               // Pointer to the config instance
        BUZZER*         pBz;                                                // Pointer to the buzzer instance
        uint8_t         mode;                                               // Which parameter to change: t_min, t_mid, t_max
        uint16_t        calib_temp[2][3];                                   // Calibration temperature data measured at each of calibration points (Celsius, internal temp)
        uint16_t        preset_temp;                                        // The preset temp in human readable units
        bool            ready;                                              // Whether the temperature has been established
        bool            tune;                                               // Whether the parameter is modifiying
        const uint32_t  period   = 1000;                                    // Update screen period
        const uint16_t  temp_max = 900;
};

void calibSCREEN::init(void) {
    mode = 0;
    pEnc->reset(mode, 0, 2, 1, 0, true);                                    // Select the reference temperature: 0 - temp_tip[0], 1 - temp_tip[1], 2 - temp_tip[2]
    pHG->switchPower(false);
    tune  = false;
    ready = false;
    for (uint8_t i = 0; i < 3; ++i)
        calib_temp[0][i] = temp_tip[i];
    pCfg->getCalibrationData(&calib_temp[1][0]);
    pD->clear();
    pD->msgOFF();
    uint16_t temp = temp_tip[mode];
    preset_temp = pHG->getTemp();                                           // Preset Temp in internal units
    preset_temp = pCfg->tempHuman(preset_temp);                             // Save the preset temperature in Celsius
    uint16_t temp_set = pCfg->tempInternal(temp);
    pHG->setTemp(temp_set);
    forceRedraw();
}

SCREEN* calibSCREEN::show(void) {
    if (millis() < update_screen) return this;
    update_screen       = millis() + period;
    int temp            = pHG->tempAverage();                               // The Hot gun average value of the current temp. (internal)
    int temp_set        = pHG->getTemp();                                   // The preset
    uint16_t tempH      = pCfg->tempHuman(temp);
    uint16_t temp_setH  = pCfg->tempHuman(temp_set);
    pD->tSet(temp_setH,temp_set);
    pD->tCurr(tempH,temp);

    uint8_t p = pHG->appliedPower();
    if (!pHG->isOn()) p = 0;
    pD->appliedPower(p);
    if (tune && (abs(temp_set - temp) < 5) && (pHG->tempDispersion() <= 20) && (p > 1))  {
        if (!ready) {
            pBz->shortBeep();
            pD->msgReady();
            ready = true;
            }
    }
    if (ready) {
        pD->tReal(pEnc->read());
    } else {
        if (pHG->isOn())
            pD->fanSpeed(pHG->presetFanPcnt());
    }
    if (tune && !pHG->isOn()) {                                             // The hot gun was switched off by error
        pD->msgOFF();
        tune  = false;
        ready = false;
    }
    return this;
}

void calibSCREEN::rotaryValue(int16_t value) {
    update_screen = millis() + period;
    if (!tune) {                                                            // select the temperature to be calibrated, t_min, t_mid or t_max
        mode = value;
        if (mode > 2) mode = 2;
        uint16_t temp = temp_tip[mode];
        temp = pCfg->tempInternal(temp);
        pHG->setTemp(temp);
    }
    forceRedraw();
}

SCREEN* calibSCREEN::menu(void) {
    if (tune) {                                                             // Calibrated value for the temperature limit jus has been setup
        tune = false;
        calib_temp[0][mode] = pEnc->read();                                 // Real temperature (Celsius)
        calib_temp[1][mode] = pHG->tempAverage();                           // The temperature on the hot gun
        pHG->switchPower(false);
        pD->msgOFF();
        pEnc->reset(mode, 0, 2, 1, 0, true);                                // The temperature limit has been adjusted, switch to select mode
        uint16_t tip[3];
        buildCalibration(tip);
        pCfg->applyCalibrationData(tip);
        pD->msgTip(tip[0],tip[1],tip[2]);
    } else {                                                                // Calibration point selected
        tune = true;
        uint16_t temp = temp_tip[mode];
        pEnc->reset(temp, 40, 600, 1, 5);
        temp = pCfg->tempInternal(temp);
        pHG->setTemp(temp);
        pHG->switchPower(true);
        pD->msgON();
        uint16_t storedCalTip[3];
        pCfg->getCalibrationData(storedCalTip);
        pD->msgTip(storedCalTip[0],storedCalTip[1],storedCalTip[2]);
    }
    ready = false;
    forceRedraw();
    return this;
}

SCREEN* calibSCREEN::menu_long(void) {
    pHG->switchPower(false);
    // temp_tip - array of calibration temperatures in Celsius
    uint16_t tip[3];
    buildCalibration(tip);
    pCfg->saveCalibrationData(tip);
    uint8_t fan = pHG->getFanSpeed();
    pCfg->save(preset_temp, fan);
    uint16_t temp = pCfg->tempInternal(preset_temp);
    pHG->setTemp(temp);
    if (next) return next;
    return this;
}

/*
 * Calculate hot gun calibration parameters using linear approximation by Ordinary Least Squares method
 * Y = a * X + b, where
 * Y - internal temperature, X - real temperature. a and b are double coefficients
 * a = (N * sum(Xi*Yi) - sum(Xi) * sum(Yi)) / ( N * sum(Xi^2) - (sum(Xi))^2)
 * b = 1/N * (sum(Yi) - a * sum(Xi))
 * N = 3 (3 reference temperature points are used)
 */
void calibSCREEN::buildCalibration(uint16_t tip[3]) {
    long sum_XY = 0;                                                        // sum(Xi * Yi)
    long sum_X  = 0;                                                        // sum(Xi)
    long sum_Y  = 0;                                                        // sum(Yi)
    long sum_X2 = 0;                                                        // sum(Xi^2)

    for (uint8_t i = 0; i < 3; ++i) {
        uint16_t X  = calib_temp[0][i];
        uint16_t Y  = calib_temp[1][i];
        sum_XY  += X * Y;
        sum_X   += X;
        sum_Y   += Y;
        sum_X2  += X * X;
    }

    double a = (double)(3 * sum_XY - sum_X * sum_Y) / (double)(3 * sum_X2 - sum_X * sum_X);
    double b = ((double)sum_Y - a * (double)sum_X) / 3.0;

    for (uint8_t i = 0; i < 3; ++i) {
        double temp = a * (double)temp_tip[i] + b;
        tip[i] = round(temp);
    }
    if (tip[2] > temp_max) tip[2] = temp_max;                               // Maximal possible temperature
}

//---------------------------------------- class tuneSCREEN [tune the potentiometer ] --------------------------
class tuneSCREEN : public SCREEN {
    public:
        tuneSCREEN(HOTGUN* HG, DSPL* DSP, ENCODER* Enc, BUZZER* Buzz) {
            pHG     = HG;
            pD      = DSP;
            pEnc    = Enc;
            pBz     = Buzz;
        }
        virtual void    init(void);
        virtual SCREEN* menu(void);
        virtual SCREEN* menu_long(void);
        virtual SCREEN* show(void);
        virtual void    rotaryValue(int16_t value);
    private:
        HOTGUN*     pHG;                                                    // Pointer to the IRON instance
        DSPL*       pD;                                                     // Pointer to the display instance
        ENCODER*    pEnc;                                                   // Pointer to the rotary encoder instance
        BUZZER*     pBz;                                                    // Pointer to the simple Buzzer instance
        bool        on;                                                     // Wether the power is on
        uint32_t    heat_ms;                                                // Time in ms when power was on
        uint8_t     max_power;                                              // Maximum possible power to be applied
        const uint16_t period = 500;                                        // The period in ms to update the screen
};

void tuneSCREEN::init(void) {
    pHG->switchPower(false);
    max_power = pHG->getMaxFixedPower();
    pEnc->reset(max_power >> 2, 0, max_power, 1, 2);                        // Rotate the encoder to change the power supplied
    on = false;
    heat_ms = 0;
    pD->clear();
    pD->msgTune();
    pD->msgOFF();
    forceRedraw();
}

void tuneSCREEN::rotaryValue(int16_t value) {
    if (on) {
        heat_ms = millis();
        pHG->fixPower(value);
    }
    forceRedraw();
}

SCREEN* tuneSCREEN::show(void) {
    if (millis() < update_screen) return this;
    update_screen = millis() + period;
    uint16_t temp   = pHG->getCurrTemp();
    uint8_t  power  = pHG->appliedPower();
    if (!on) {
        power = pEnc->read();
    }
    pD->tInternal(temp);
    pD->appliedPower(power);
    if (heat_ms && ((millis() - heat_ms) > 3000) && (pHG->tempDispersion() < 10) && (power > 1)) {
        pBz->shortBeep();
        heat_ms = 0;
    }
    return this;
}

SCREEN* tuneSCREEN::menu(void) {                                            // The rotary button pressed
    if (on) {
        pHG->fixPower(0);
        on = false;
        pD->msgOFF();
    } else {
        on = true;
        heat_ms = millis();
        uint8_t power = pEnc->read();                                       // applied power
        pHG->fixPower(power);
        pD->msgON();
    }
    return this;
}

SCREEN* tuneSCREEN::menu_long(void) {
    pHG->fixPower(0);                                                       // switch off the power
    pHG->switchPower(false);
    if (next) return next;
    return this;
}

//---------------------------------------- class pidSCREEN [tune the PID coefficients] -------------------------
class pidSCREEN : public SCREEN {
    public:
        pidSCREEN(HOTGUN* HG, ENCODER* ENC) {
            pHG     = HG;
            pEnc    = ENC;
        }
        virtual void    init(void);
        virtual SCREEN* menu(void);
        virtual SCREEN* menu_long(void);
        virtual SCREEN* show(void);
        virtual void    rotaryValue(int16_t value);
    private:
        void        showCfgInfo(void);                                      // show the main config information: Temp set, fan speed and PID coefficients
        HOTGUN*     pHG;                                                    // Pointer to the IRON instance
        ENCODER*    pEnc;                                                   // Pointer to the rotary encoder instance
        uint8_t     mode;                                                   // Which parameter to tune [0-5]: select element, Kp, Ki, Kd, temp, speed
        uint32_t    update_screen;                                          // Time in ms when to print thee info
        int         temp_set;
        const uint16_t period = 1100;
};

void pidSCREEN::init(void) {
    temp_set = pHG->getTemp();
    mode = 0;                                                               // select the element from the list
    pEnc->reset(1, 1, 5, 1, 1, true);                                       // 1 - Kp, 2 - Ki, 3 - Kd, 4 - temp, 5 - fan
    showCfgInfo();
    Serial.println("");
}

void pidSCREEN::rotaryValue(int16_t value) {
    if (mode == 0) {                                                        // select element from the menu
        showCfgInfo();
        switch (value) {
            case 1:
                Serial.println("Kp");
                break;
            case 2:
                Serial.println("Ki");
                break;
            case 4:
                Serial.println(F("Temp"));
                break;
            case 5:
                Serial.println(F("Fan"));
                break;
            case 3:
            default:
                Serial.println("Kd");
            break;
        }
    } else {
        switch (mode) {
            case 1:
                Serial.print(F("Kp = "));
                pHG->changePID(mode, value);
                break;
            case 2:
                Serial.print(F("Ki = "));
                pHG->changePID(mode, value);
                break;
            case 4:
                Serial.print(F("Temp = "));
                temp_set = value;
                pHG->setTemp(value);
                break;
            case 5:
                Serial.print(F("Fan Speed = "));
                pHG->setFanSpeed(value);
                break;
            case 3:
            default:
                Serial.print(F("Kd = "));
                pHG->changePID(mode, value);
                break;
        }
        Serial.println(value);
    }
}

SCREEN* pidSCREEN::show(void) {
    if (millis() < update_screen) return this;
    update_screen = millis() + period;
    if (pHG->isOn()) {
        char buff[80];
        int      temp   = pHG->getCurrTemp();
        uint8_t  pwr    = pHG->powerAverage();
        uint8_t  fs     = pHG->getFanSpeed();
        fs = map(fs, 0, 255, 0, 100);
        sprintf(buff, "%3d: power = %3d%c, fan = %3d;", temp_set - temp, pwr, '%', fs);
        Serial.println(buff);
    }
    return this;
}
SCREEN* pidSCREEN::menu(void) {                                             // The encoder button pressed
    if (mode == 0) {                                                        // select upper or lower temperature limit
        mode = pEnc->read();
        if (mode > 0 && mode < 4) {
            int k = pHG->changePID(mode, -1);
            pEnc->reset(k, 0, 10000, 1, 10);
        } else if (mode == 4) {
            pEnc->reset(temp_set, 0, 970, 1, 5);
        } else {
            pEnc->reset(pHG->getFanSpeed(), 0, 250, 5, 20);
        }
    } else {
        mode = 0;
        pEnc->reset(1, 1, 5, 1, 1, true);                                   // 1 - Kp, 2 - Ki, 3 - Kd, 4 - temp, 5 - fan speed
    }
    return this;
}

SCREEN* pidSCREEN::menu_long(void) {
    bool on = pHG->isOn();
    pHG->switchPower(!on);
    if (on)
        Serial.println(F("The air gun is OFF"));
    else
        Serial.println(F("The air gun is ON"));
  return this;
}

void pidSCREEN::showCfgInfo(void) {
    Serial.print(F("Temp set: "));
    Serial.print(temp_set, DEC);
    Serial.print(F(", fan speed = "));
    Serial.print(pHG->getFanSpeed());
    Serial.print(F(", PID: ["));
    for (byte i = 1; i < 4; ++i) {
        int k = pHG->changePID(i, -1);
        Serial.print(k, DEC);
        if (i < 3) Serial.print(", ");
    }
    Serial.print("]; ");
}

//---------------------------------------- class fanSCREEN [ fan threshold calibration ] -------------------------------
class fanSCREEN : public SCREEN {
    public:
        fanSCREEN(HOTGUN* HG, DSPL* DSP, ENCODER* Enc, BUZZER* Buzz, HOTGUN_CFG* Cfg) {
            pHG     = HG;
            pD      = DSP;
            pEnc    = Enc;
            pCfg    = Cfg;
            pBz     = Buzz;
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
        BUZZER*         pBz;                                                // Pointer to the buzzer instance
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
    fan_min = pCfg->getMinFanSpeedSens();
    fan_max = pCfg->getMaxFanSpeedSens();
    modeSel = true;                                                         // 0 - select the element to change [fan_min, fan_max], 1 - change the selected value
    fanSet = 0;                                                             // 0 - fan_min, 1 - fan_max
    pEnc->reset(0, 0, 1, 1, 1, true);                                       // 0 - fan_min, 1 - fan_max
    pHG->switchPower(false);
    pD->clear();
    pD->msgFanTune(fanSet,fan_min, fan_max);
    //pD->msgOFF();
    forceRedraw();
}

SCREEN* fanSCREEN::show(void){
    if (millis() < update_screen) return this;
    update_screen       = millis() + period;

 //   if (modeSel) {
       pD->msgFanTune(fanSet,fan_min, fan_max);
 //   }
    return this;

}

SCREEN* fanSCREEN::menu(void){
    if (modeSel) {                                                         // Prepare to select which value to adjust
        uint16_t fanThr;
        if (fanSet == 0) {
            fanThr = pCfg->getMinFanSpeedSens();
        }
        else {
            fanThr = pCfg->getMaxFanSpeedSens();
        }
        pEnc->reset(fanThr, 0, 1024, 1, 5);
        modeSel = false;
    } else {                                                                // Prepare to adjust the preset value
        pEnc->reset(0, 0, 1, 1, 1, true);                                       // 0 - fan_min, 1 - fan_max
        modeSel = true;
    }
    return this;
}

SCREEN* fanSCREEN::menu_long(void){
    pHG->switchPower(false);
    pCfg->applyFanData(fan_min, fan_max);
    pCfg->saveFanData(fan_min, fan_max);
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
HOTGUN      hg(FAN_GUN_SENS_PIN, HOT_GUN_PIN, AC_RELAY_PIN);
DSPL        disp;
ENCODER     rotEncoder(R_MAIN_PIN, R_SECD_PIN);
BUTTON      rotButton(R_BUTN_PIN);
SWITCH      reedSwitch(REED_SW_PIN);
HOTGUN_CFG  hgCfg;
BUZZER      simpleBuzzer(BUZZER_PIN, BUZZER_ACTIVE);

mainSCREEN   offScr(&hg,  &disp, &rotEncoder, &simpleBuzzer, &hgCfg);
workSCREEN   wrkScr(&hg,  &disp, &rotEncoder, &simpleBuzzer, &hgCfg);
configSCREEN cfgScr(&hg,  &disp, &rotEncoder, &hgCfg);
calibSCREEN  clbScr(&hg,  &disp, &rotEncoder, &simpleBuzzer, &hgCfg);
tuneSCREEN   tuneScr(&hg, &disp, &rotEncoder, &simpleBuzzer);
errorSCREEN  errScr(&hg,  &disp, &simpleBuzzer);
pidSCREEN    pidScr(&hg,  &rotEncoder);
fanSCREEN    fanScr(&hg,  &disp, &rotEncoder, &simpleBuzzer, &hgCfg);

SCREEN  *pCurrentScreen = &offScr;

volatile bool   end_of_power_period = false;

void syncAC(void) {
    end_of_power_period = hg.syncCB();
}

void rotEncChange(void) {
    rotEncoder.changeINTR();
}

void setup() {
    Serial.begin(115200);
    disp.init();

    // Initialize rotary encoder
    rotEncoder.init();
    rotButton.init();
    delay(100);
    int buttonState = digitalRead(R_BUTN_PIN); // pressed = LOW, open = HIGH
    delay(500);
    if (buttonState == LOW) { // button pressed
        if (digitalRead(R_BUTN_PIN) == LOW) { // still pressed
            // eeprom clear
            hgCfg.clear();
        }
    }

    // Load configuration parameters
    hgCfg.init();
    hg.init();
    uint16_t temp   = hgCfg.tempPreset();
    uint8_t  fan    = hgCfg.fanPreset();
    hg.setTemp(temp);
    hg.setFanSpeed(fan);

    reedSwitch.init(500, 3000);

    attachInterrupt(digitalPinToInterrupt(R_MAIN_PIN), rotEncChange,   CHANGE);
    attachInterrupt(digitalPinToInterrupt(AC_SYNC_PIN), syncAC, RISING);

    // Initialize SCREEN hierarchy
    offScr.next     = &cfgScr;
    offScr.on       = &wrkScr;
    wrkScr.next     = &offScr;
    cfgScr.next     = &offScr;
    cfgScr.calib    = &clbScr;
    cfgScr.tune     = &tuneScr;
    cfgScr.fan      = &fanScr;
    clbScr.next     = &offScr;
    tuneScr.next    = &offScr;
    errScr.next     = &offScr;
    fanScr.next     = &offScr;

    //thermocouple.begin();

    pCurrentScreen->init();
}

void loop() {
    static bool     reset_encoder   = true;
    static int16_t  old_pos         = 0;
    static uint32_t ac_check        = 5000;


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

    SCREEN* nxt = pCurrentScreen->reedSwitch(reedSwitch.status());
    if (nxt != pCurrentScreen) {
        pCurrentScreen = nxt;
        pCurrentScreen->init();
        reset_encoder = true;
        return;
    }

    uint8_t bStatus = rotButton.buttonCheck();
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
        hg.keepTemp();
        end_of_power_period = false;
    }

    if (millis() > ac_check) {
        ac_check = millis() + 1000;
#ifndef DISABLE_AC_CHECK
        if (!hg.areExternalInterrupts()) {
            nxt = &errScr;
            if (nxt != pCurrentScreen) {
                pCurrentScreen = nxt;
                pCurrentScreen->init();
                reset_encoder = true;
            }
        }
#endif
    }
}