/**
 * @file Nano-CurrentSensors.ino
 * @brief 
 * @version 0.1
 * @date 2022-07-01
 * 
 * @copyright Copyright (c) 2022
 * 
 * @board Duemilanove / Nano
 */
#include <Wire.h>
#include "Arduino.h"
#include "EEPROM.h"

#define AVERAGER_MAX_SIZE 32

enum wireCommands:int {
    readOneSensorAvg,//
    readAllSensorsAvg,//
    setSensorVperA,//
    getStatus,//
    readOneSensorRaw,//
    readOneSensorInstant,//
    readAllSensorsInstant,//
    readOneSensorMax,//
    readAllSensorsMax,//
    resetOneSensorMax,// 
    resetAllSensorsMax,//
    getNumSerialBytesAvailable,//
    getAvailableSerialBytes,//
    setSerialBaud,//
    writeSerialData,//
    setAveragerN_vals//
    };

class ACS712{
private:
    /* data */
    int pin;
    double voltsPerAmp = 0;
    long averageMilliAmps;
    long averager[AVERAGER_MAX_SIZE] = { 0 };
    long maxMilliamps = 0;
    long instantMilliamps = 0;
    uint8_t averagerN_vals = 20;
    int id;
public:
    ACS712(int pinNum, int id){
        pin = pinNum;
        uint32_t VperA = 0;
        this->id = id;
        EEPROM.get(id * 4,VperA);
        if (VperA != 0xffffffff) this->set_mvPerA(VperA);
    }
    ~ACS712(){}

    long getMilliAmps(){
        double milliamps = 0.0;
        double reading = analogRead(pin);
        milliamps = ((2.5 - (reading * 5.0 / 1023.0)) / voltsPerAmp) * 1000.0;
        if (milliamps > this->maxMilliamps) this->maxMilliamps = milliamps;
        return long(milliamps);
    }

    int getRawRead(){
        return analogRead(pin);
    }

    long getAverageMilliAmps(){
        return averageMilliAmps;
    }
    void update(){
        if(voltsPerAmp == 0)return;
        for(int i = 0; i < averagerN_vals - 1; i++){
            averager[i] = averager[i+1];
        }
        this->instantMilliamps = averager[averagerN_vals - 1] = this->getMilliAmps();
        long long total = 0;
        for(int i = 0; i < 20; i++){
            total += averager[i];
        }
        averageMilliAmps = total/averagerN_vals;
    }
    void set_mvPerA(unsigned int val){
        if(val == 0) return;
        this->voltsPerAmp = val / 1000.0;

        uint32_t VperA = 0;
        this->id = id;
        EEPROM.get(id * 4,VperA);
        if (VperA != this->voltsPerAmp) EEPROM.put(this->id*4,VperA);
    }
    bool isReady(){
        if(voltsPerAmp == 0.0)return false;
        return true;
    }
    void resetMaxMillis() {
        this->maxMilliamps = 0;
    }
    long getMaxMilliamps() {
        return this->maxMilliamps;
    }

    long getInstantMilliAmps(){
        return this->instantMilliamps;
    }

    void setAveragerN_vals(int v){
        if(v > AVERAGER_MAX_SIZE)return;
        this->averagerN_vals = v;
    }
};

ACS712 ammeter0(A0,0);
ACS712 ammeter1(A1,1);
ACS712 ammeter2(A2,2);
ACS712 ammeter3(A3,3);
ACS712 ammeter4(A6,4);
ACS712 ammeter5(A7,5);
ACS712* ammeters[6] = {&ammeter0,&ammeter1,&ammeter2,&ammeter3,&ammeter4,&ammeter5};

unsigned char wireInCommand[32] = {0};
bool ready[2] = {false,false};
unsigned int delayTime = 1;

uint8_t serialBuf[256];
uint8_t serialBuf_readPntr = 0;
uint8_t serialBuf_writePntr = 0;
uint8_t serialBuf_bytesToRead = 0;

void setup()
{
    pinMode(12,INPUT_PULLUP); // | I2C addr pins
    pinMode(11,INPUT_PULLUP); // v
    pinMode(10,INPUT_PULLUP); //
    pinMode(9,INPUT_PULLUP);  //
    pinMode(8,INPUT_PULLUP);  //
    pinMode(7,INPUT_PULLUP);  // ^
    pinMode(6,INPUT_PULLUP);  // | I2C addr pins
    pinMode(5,INPUT_PULLUP);  // Device mode pin. High for ammeter, low for voltmeter @TODO: Need to actually implement this
    Serial.begin(115200);
    Wire.begin(digitalRead(12) + (digitalRead(11) << 1) + (digitalRead(10) << 2) + (digitalRead(9) << 3) + (digitalRead(8) << 4) + (digitalRead(7) << 5) + (digitalRead(6) << 6));
    //Wire.setClock(400000);
    Wire.onReceive(wireReceiveEvent);
    Wire.onRequest(wireRequestEvent);
    pinMode(A0, INPUT);
    pinMode(A1, INPUT);
    pinMode(A2, INPUT);
    pinMode(A3, INPUT);
    pinMode(A6, INPUT);
    pinMode(A7, INPUT);
    ready[0] = true;
}

void loop()
{
    for(int i = 0; i < 6; i++){
        ammeters[i]->update();
    }
    delayMicroseconds(delayTime);
    if(!ready[1]){
        if( ammeters[0]->isReady() &&
            ammeters[1]->isReady() &&
            ammeters[2]->isReady() &&
            ammeters[3]->isReady() &&
            ammeters[4]->isReady() &&
            ammeters[5]->isReady()) ready[1] = true;
    }
    while(Serial.available()){
        serialBuf[serialBuf_writePntr++]=Serial.read();
        if(serialBuf_bytesToRead < 255)serialBuf_bytesToRead++;
    }
}

void wireReceiveEvent(int numBytes){
    int byteCount = 0;
    for(int i = 0; i < 32; i++)wireInCommand[i] = 0;
    while(Wire.available() && byteCount < numBytes){
        wireInCommand[byteCount] = Wire.read();
        byteCount++;
    }
    
    switch(wireInCommand[0]){
        case wireCommands::setSensorVperA:
        {
            if(wireInCommand[1] < 6){
                unsigned int val1 = wireInCommand[2];
                unsigned int val2 = wireInCommand[3];
                unsigned int val3 = (val1 << 8) | val2;
                ammeters[wireInCommand[1]]->set_mvPerA(val3);
                // Serial.print("set_mvPerA: ");
                // Serial.println(val3);
                if( ammeters[0]->isReady() &&
                    ammeters[1]->isReady() &&
                    ammeters[2]->isReady() &&
                    ammeters[3]->isReady() &&
                    ammeters[4]->isReady() &&
                    ammeters[5]->isReady()) ready[1] = true;
            }
            break;
        }
        case wireCommands::resetOneSensorMax:
        {
            if (wireInCommand[1] < 6) {
                ammeters[wireInCommand[1]]->resetMaxMillis();
            }
            break;
        }
        case wireCommands::resetAllSensorsMax:
        {
            for(int i = 0; i < 6; i++){
                ammeters[i]->resetMaxMillis();
            }
            break;
        }
        case wireCommands::setAveragerN_vals:
        {
            if(wireInCommand[1] < 6){
                ammeters[wireInCommand[1]]->setAveragerN_vals(wireInCommand[2]);
            }
            break;
        }
        case wireCommands::setSerialBaud:
        {
            uint16_t baud = 0;
            baud |= wireInCommand[1] << 8;
            baud |= wireInCommand[2];

            Serial.end();
            Serial.begin(baud);
            break;
        }
        case wireCommands::writeSerialData:
        {
            uint32_t count = 0;
            count |= wireInCommand[1] << 8;
            count |= wireInCommand[2];
            for(uint32_t i = 0; i < count; i++){
                Serial.write(wireInCommand[i+2]);
            }
            break;
        }
    }
    
    
}

void wireRequestEvent(){
    switch(wireInCommand[0]){
        case wireCommands::readAllSensorsAvg:
        {
            Wire.write(6*4);
            for(int i = 0;i < 6; i++){
                long val = ammeters[i]->getAverageMilliAmps();
                // Serial.print("val: ");
                // Serial.println(val);
                Wire.write((val >> 24) & 0xff);
                Wire.write((val >> 16) & 0xff);
                Wire.write((val >> 8) & 0xff);
                Wire.write(val & 0xff);
            }
            // Serial.println("");
            break;
        }
        case wireCommands::readOneSensorAvg:
        {
            Wire.write(4);
            unsigned char sensor = wireInCommand[1];
            if(sensor < 6){
                long val = ammeters[sensor]->getAverageMilliAmps();
                Wire.write(val >> 24);
                Wire.write(val >> 16);
                Wire.write(val >> 8);
                Wire.write(val & 0xff);
            }
            break;
        }
        case wireCommands::getStatus:
        {
            Wire.write(0xa0); // indicates that this is an ammeter // @TODO: use digitalRead to set this 
            Wire.write(((ready[0])?0xf0:0x00) | ((ready[1])?0x0f:0x00));
            break;
        }
        case wireCommands::readOneSensorMax:
        {
            //@TODO: This needs to be rewritten
            Wire.write(4);
            unsigned char command = wireInCommand[1];
            for(int i = 0; i < 6; i++){
                if((command & 1) == 1){
                    long val = ammeters[i]->getMaxMilliamps();
                    Wire.write(val >> 24);
                    Wire.write(val >> 16);
                    Wire.write(val >> 8);
                    Wire.write(val & 0xff);
                    i = 6;
                }
                command = command >> 1;
            }
            break;
        }
        case wireCommands::getNumSerialBytesAvailable:
        {
            Wire.write(serialBuf_bytesToRead);
            break;
        }
        case wireCommands::getAvailableSerialBytes:
        {
            if(serialBuf_bytesToRead > 32){
                for(uint8_t i = 0; i < 32; i++){
                    Wire.write(serialBuf[serialBuf_readPntr++]);
                    if(serialBuf_bytesToRead>0)serialBuf_bytesToRead--;
                }
            }else{
                uint8_t t = serialBuf_bytesToRead;
                for(uint8_t i = 0; i < t; i++){
                    Wire.write(serialBuf[serialBuf_readPntr++]);
                    if(serialBuf_bytesToRead>0)serialBuf_bytesToRead--;
                }
            }   
            break;
        }
        case wireCommands::readAllSensorsInstant:
        {
            Wire.write(6*4);
            for(int i = 0;i < 6; i++){
                long val = ammeters[i]->getInstantMilliAmps();
                // Serial.print("val: ");
                // Serial.println(val);
                Wire.write((val >> 24) & 0xff);
                Wire.write((val >> 16) & 0xff);
                Wire.write((val >> 8) & 0xff);
                Wire.write(val & 0xff);
            }
            break;
        }
        case wireCommands::readAllSensorsMax:
        {
            Wire.write(6*4);
            for(int i = 0;i < 6; i++){
                long val = ammeters[i]->getMaxMilliamps();
                // Serial.print("val: ");
                // Serial.println(val);
                Wire.write((val >> 24) & 0xff);
                Wire.write((val >> 16) & 0xff);
                Wire.write((val >> 8) & 0xff);
                Wire.write(val & 0xff);
            }
            break;
        }
        case wireCommands::readOneSensorInstant:
        {
            Wire.write(4);
            unsigned char sensor = wireInCommand[1];
            if(sensor < 6){
                long val = ammeters[sensor]->getInstantMilliAmps();
                Wire.write(val >> 24);
                Wire.write(val >> 16);
                Wire.write(val >> 8);
                Wire.write(val & 0xff);
            }
            break;
        }
    }
}