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

enum wireCommands:int {
    readOneSensorAvg,
    readAllSensorsAvg,
    readSelectedSensorsAvg,
    setSensorVperA,
    getStatus,
    readOneSensorInstant,
    readOneSensorRaw,
    readOneSensorMax,
    resetOneSensorMax,
    getNumSerialBytesAvailable,
    getAvailableSerialBytes,
    setSerialBaud,
    writeSerialData
    };

class ACS712{
private:
    /* data */
    int pin;
    double voltsPerAmp = 0;
    long averageMilliAmps;
    long averager[20] = { 0 };
    long maxMilliamps = 0;
public:
    ACS712(int pinNum){
        pin = pinNum;
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
        for(int i = 0; i < 19; i++){
            averager[i] = averager[i+1];
        }
        averager[19] = this->getMilliAmps();
        long long total = 0;
        for(int i = 0; i < 20; i++){
            total += averager[i];
        }
        averageMilliAmps = total/20;
    }
    void set_mvPerA(unsigned int val){
        if(val == 0) return;
        voltsPerAmp = val / 1000.0;
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
};

ACS712 ammeter0(A0);
ACS712 ammeter1(A1);
ACS712 ammeter2(A2);
ACS712 ammeter3(A3);
ACS712 ammeter4(A6);
ACS712 ammeter5(A7);
ACS712* ammeters[6] = {&ammeter0,&ammeter1,&ammeter2,&ammeter3,&ammeter4,&ammeter5};

unsigned char wireInCommand[4] = {0};
bool ready[2] = {false,false};
unsigned int numSerialBytesRead = 0, numSerialByteToBeWritten = 0;
unsigned char serialDataIn[256], serialDataOut[256];
unsigned int serialDataInIndex = 0, serialDataOutIndex = 0;

void setup()
{
    pinMode(12,INPUT_PULLUP);
    Serial.begin(115200);
    Wire.begin(8+digitalRead(12));
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

    while(Serial.available()){
        serialDataIn[serialDataInIndex]Serial.read();
        serialDataInIndex++;
        if(serialDataInIndex > 255) serialDataInIndex = 0;
        numSerialBytesRead++;
    }

    while (numSerialByteToBeWritten > 0){
        Serial.write(serialDataOut[serialDataOutIndex]);
        serialDataOutIndex++;
        if(serialDataOutIndex > 255) serialDataOutIndex = 0;
        numSerialByteToBeWritten--;
    }
}

void wireReceiveEvent(int numBytes){
    int byteCount = 0;
    unsigned char incoming[numBytes] = {0};
    for(int i = 0; i < 4; i++)wireInCommand[i] = 0;
    while(Wire.available() && byteCount < numBytes){
        incoming[byteCount] = Wire.read();
        byteCount++;
    }
    
    if(wireInCommand[0] == wireCommands::setSensorVperA && wireInCommand[1] < 6){
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
    if (wireInCommand[0] == wireCommands::resetOneSensorMax && wireInCommand[1] < 6) {
        ammeters[wireInCommand[1]]->resetMaxMillis();
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
            unsigned char command = wireInCommand[1];
            for(int i = 0; i < 6; i++){
                if(command & 1 == 1){
                    long val = ammeters[i]->getAverageMilliAmps();
                    Wire.write(val >> 24);
                    Wire.write(val >> 16);
                    Wire.write(val >> 8);
                    Wire.write(val & 0xff);
                    i = 6;
                }
                command >> 1;
            }
            break;
        }
        case wireCommands::readSelectedSensorsAvg:
        {
            unsigned char command = wireInCommand[1];
            int count = countBits(command);
            Wire.write(4 * count);
            for(int i = 0; i < 6; i++){
                if(command & 1 == 1){
                    long val = ammeters[i]->getAverageMilliAmps();
                    Wire.write(val >> 24);
                    Wire.write(val >> 16);
                    Wire.write(val >> 8);
                    Wire.write(val & 0xff);
                }
                command >> 1;
            }
            break;
        }
        case wireCommands::getStatus:
        {
            Wire.write(1);
            // if(ready[0] && ready[1])Wire.write(0xff);
            // else Wire.write(0);
            Wire.write(((ready[0])?0xf0:0x00) | ((ready[1])?0x0f:0x00));
            break;
        }
        case wireCommands::readOneSensorMax:
        {
            Wire.write(4);
            unsigned char command = wireInCommand[1];
            for(int i = 0; i < 6; i++){
                if(command & 1 == 1){
                    long val = ammeters[i]->getMaxMilliamps();
                    Wire.write(val >> 24);
                    Wire.write(val >> 16);
                    Wire.write(val >> 8);
                    Wire.write(val & 0xff);
                    i = 6;
                }
                command >> 1;
            }
            break;
        }
        case wireCommands::getNumSerialBytesAvailable:
        {
            break;
        }
        case wireCommands::getAvailableSerialBytes:
        {
            break;
        }
        case wireCommands::setSerialBaud:
        {
            break;
        }
    }
}

int countBits(unsigned char val){
    int count = 0;
    while(val > 0){
        if(val & 1) count++;
        val >> 1;
    }
    return count;
}