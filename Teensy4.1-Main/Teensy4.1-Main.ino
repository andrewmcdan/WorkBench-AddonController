/**
 *  Board:          Teensy 4.1
 *  Overclock:      default
 *  
 */

#include <Arduino.h>
#include <Wire.h>
#include <EEPROM.h>
#include <MIDI.h>
#include "Teensy4.1-Main.h"
#include <functional>
#include <vector>
#include <string>
#include <SoftwareSerial.h>

#define BIG_BUF_SIZE 65536
#define METER_READING_TIME_MS 100
#define SERIAL_READ_TIME_MS 10
#define MAX_NUMBER_OF_METERS 30
#define SERIAL_START_BYTE1 0b10101010
#define SERIAL_START_BYTE2 0b11001100

// these must be set according to which ammeters are soldered onto the PCB
//const int ammeter1_mVperA[6] = { 40,40,185,185,185,185 }; // 50A, 50A, 5A, 5A, 5A, 5A
//const int ammeter2_mVperA[6] = { 66,66,66,66,185,185 }; //  30A, 30A, 30A, 30A, 5A, 5A

// these lines use MACRO calls to setup the serial midi 
// MIDI_CREATE_INSTANCE(HardwareSerial, Serial1, serialMIDI_1);
// MIDI_CREATE_INSTANCE(HardwareSerial, Serial2, serialMIDI_2);
// MIDI_CREATE_INSTANCE(HardwareSerial, Serial3, serialMIDI_3);
// MIDI_CREATE_INSTANCE(HardwareSerial, Serial4, serialMIDI_4);
// MIDI_CREATE_INSTANCE(HardwareSerial, Serial5, serialMIDI_5);
// MIDI_CREATE_INSTANCE(HardwareSerial, Serial6, serialMIDI_6);
// MIDI_CREATE_INSTANCE(HardwareSerial, Serial7, serialMIDI_7);
// MIDI_CREATE_INSTANCE(HardwareSerial, Serial8, serialMIDI_8);

// this does the same as the macro calls above, but stores the midi devices
// in an array for ease of use.
midi::SerialMIDI<HardwareSerial> serialMIDI_1(Serial1);
midi::SerialMIDI<HardwareSerial> serialMIDI_2(Serial2);
midi::SerialMIDI<HardwareSerial> serialMIDI_3(Serial3);
midi::SerialMIDI<HardwareSerial> serialMIDI_4(Serial4);
midi::SerialMIDI<HardwareSerial> serialMIDI_5(Serial5);
midi::SerialMIDI<HardwareSerial> serialMIDI_6(Serial6);
midi::SerialMIDI<HardwareSerial> serialMIDI_7(Serial7);
midi::SerialMIDI<HardwareSerial> serialMIDI_8(Serial8);

midi::MidiInterface<midi::SerialMIDI<HardwareSerial>> HW_midi[] = {
    (midi::SerialMIDI<HardwareSerial> &)serialMIDI_1,
    (midi::SerialMIDI<HardwareSerial> &)serialMIDI_2,
    (midi::SerialMIDI<HardwareSerial> &)serialMIDI_3,
    (midi::SerialMIDI<HardwareSerial> &)serialMIDI_4,
    (midi::SerialMIDI<HardwareSerial> &)serialMIDI_5,
    (midi::SerialMIDI<HardwareSerial> &)serialMIDI_6,
    (midi::SerialMIDI<HardwareSerial> &)serialMIDI_7,
    (midi::SerialMIDI<HardwareSerial> &)serialMIDI_8 };

SoftwareSerial softSer_1 = SoftwareSerial(2,3);
midi::SerialMIDI<SoftwareSerial> softSerMidi_1(softSer_1);
SoftwareSerial softSer_2 = SoftwareSerial(4,5);
midi::SerialMIDI<SoftwareSerial> softSerMidi_2(softSer_2);
SoftwareSerial softSer_3 = SoftwareSerial(9,10);
midi::SerialMIDI<SoftwareSerial> softSerMidi_3(softSer_3);
SoftwareSerial softSer_4 = SoftwareSerial(11,12);
midi::SerialMIDI<SoftwareSerial> softSerMidi_4(softSer_4);
SoftwareSerial softSer_5 = SoftwareSerial(26,27);
midi::SerialMIDI<SoftwareSerial> softSerMidi_5(softSer_5);
SoftwareSerial softSer_6 = SoftwareSerial(30,31);
midi::SerialMIDI<SoftwareSerial> softSerMidi_6(softSer_6);
SoftwareSerial softSer_7 = SoftwareSerial(32,33);
midi::SerialMIDI<SoftwareSerial> softSerMidi_7(softSer_7);
SoftwareSerial softSer_8 = SoftwareSerial(36,37);
midi::SerialMIDI<SoftwareSerial> softSerMidi_8(softSer_8);

midi::MidiInterface<midi::SerialMIDI<SoftwareSerial>> SW_midi[] = {
    (midi::SerialMIDI<SoftwareSerial> &)softSerMidi_1,
    (midi::SerialMIDI<SoftwareSerial> &)softSerMidi_2,
    (midi::SerialMIDI<SoftwareSerial> &)softSerMidi_3,
    (midi::SerialMIDI<SoftwareSerial> &)softSerMidi_4,
    (midi::SerialMIDI<SoftwareSerial> &)softSerMidi_5,
    (midi::SerialMIDI<SoftwareSerial> &)softSerMidi_6,
    (midi::SerialMIDI<SoftwareSerial> &)softSerMidi_7,
    (midi::SerialMIDI<SoftwareSerial> &)softSerMidi_8 };

class SerialMidiManager{
public:
    SerialMidiManager(midi::MidiInterface<midi::SerialMIDI<HardwareSerial>>* serMidiDev, int cable){
        this->midiDev = serMidiDev;
        this->usbCable = cable;
        this->midiDev->begin(MIDI_CHANNEL_OMNI);
        this->setThru(false);
        this->softSer = false;
    }

    SerialMidiManager(midi::MidiInterface<midi::SerialMIDI<SoftwareSerial>>* serMidiDev, int cable){
        this->SoftSer_midiDev = serMidiDev;
        this->usbCable = cable;
        this->SoftSer_midiDev->begin(MIDI_CHANNEL_OMNI);
        this->setThru(false);
        this->softSer = true;
    }

    void update(){
        if(this->softSer?this->SoftSer_midiDev->read():this->midiDev->read()){
            byte type = this->softSer?this->SoftSer_midiDev->getType():this->midiDev->getType();
            byte chan = this->softSer?this->SoftSer_midiDev->getChannel():this->midiDev->getChannel();
            byte data1 = this->softSer?this->SoftSer_midiDev->getData1():this->midiDev->getData1();
            byte data2 = this->softSer?this->SoftSer_midiDev->getData2():this->midiDev->getData2();
            if(type!=midi::SystemExclusive){
                usbMIDI.send(type,data1,data2,chan,this->usbCable);
                //if(this->thruEn) this->midiDev->send((midi::MidiType)type, data1, data2, chan);
            }else{
                unsigned int SysExLength = data1 + (data2 * 256);
                usbMIDI.sendSysEx(SysExLength, this->softSer?this->SoftSer_midiDev->getSysExArray():this->midiDev->getSysExArray(), true, this->usbCable);
                //if(this->thruEn) this->midiDev->sendSysEx(SysExLength, this->midiDev->getSysExArray(),true);
            }
        }
    }

    void setThru(bool en){
        this->thruEn = en;
        if(this->softSer){
            //return;
            if(this->thruEn) this->SoftSer_midiDev->turnThruOn();
            else this->SoftSer_midiDev->turnThruOff();
        }else{
            if(this->thruEn) this->midiDev->turnThruOn();
            else this->midiDev->turnThruOff();
        }
        
    }

    midi::MidiInterface<midi::SerialMIDI<HardwareSerial>>* midiDev;
    midi::MidiInterface<midi::SerialMIDI<SoftwareSerial>>* SoftSer_midiDev;
private:
    int usbCable = 0;
    bool thruEn = false;
    bool softSer = false;
};


class dependableWire{
public:
    TwoWire* wireDev;
    dependableWire(TwoWire* dev){
        this->wireDev = dev;
    }

    bool write(byte addr, byte *data, byte len, int retryCount){
        byte r = 0;
        int v = 0;
        int count = 0;
        bool retry = retryCount > 0;
        do{
            this->wireDev->beginTransmission(addr);
            r = this->wireDev->write(data,len);
            v = this->wireDev->endTransmission();
            count++;
            if(count >= retryCount && retry)return false;
        }while((r != len) && retry && (v != 0));
        return r==len && v==0;
    }

    bool request(byte addr, byte *data, byte len){
        if(len<1)return false;
        int r = this->wireDev->requestFrom(addr, len);
        if(r != len)return false;
        for(byte i = 0; i < len; i++){
            data[i] = this->wireDev->read();
            delayMicroseconds(100);
        }
        return true;
    }
};

dependableWire dWire = dependableWire(&Wire);

enum meterWireCommands:int {
    readOneSensorAvg,
    readAllSensorsAvg,
    setSensorVperA,
    getStatus,
    readOneSensorRaw,
    readOneSensorInstant,
    readAllSensorsInstant,
    readOneSensorMax,
    readAllSensorsMax,
    resetOneSensorMax,
    resetAllSensorsMax,
    getNumSerialBytesAvailable,
    getAvailableSerialBytes,
    setSerialBaud,
    writeSerialData,
    setAveragerN_vals,
    setDelayTime
};
enum meterWireNumBytesToRead:int {
    readOneSensor = 5, 
    readAllSensors = 25, 
    readSelectedSensors = 5, 
    getStatus_B = 8,
    getNumSerialBytesAvail = 2
};


class Meter_Manager {
private:
    byte deviceClass = 0;
    byte id_;
    uint8_t I2C_Addr;
    long avgReadings[6] = {0};
    long maxReadings[6] = {0};
    long instantReadings[6] = {0};
    char* serialOutBuf;
    uint16_t serialOutBufWriteIndex = 0;
    uint16_t serialOutBufReadIndex = 0;
    
    elapsedMillis meterReadTimer = 0;
    uint32_t meterReadTimeMillis = METER_READING_TIME_MS;

    elapsedMillis serialReadTimer = 0;
    uint32_t serialReadTimerMillis = SERIAL_READ_TIME_MS;

public:
    Meter_Manager(int addr, byte id){
        this->I2C_Addr = addr;
        this->meterReady_ = this->meterReady();
        this->serialOutBuf = (char*)extmem_malloc(BIG_BUF_SIZE);
        this->id_ = id;
    }
    ~Meter_Manager() {}

    bool meterReady_ = false;

    // Gets updated data from the meter, including max, avg, raw, and serial data. Then sends any new
    // serial data to the meter. 
    void update() {
        // @TODO:
        // Run tests on this code to ensure that it is working correctly
        while(this->serialOutBufReadIndex != this->serialOutBufWriteIndex){
            uint32_t byteCount = 0;
            if(this->serialOutBufReadIndex < this->serialOutBufWriteIndex) byteCount = this->serialOutBufWriteIndex - this->serialOutBufReadIndex;
            else byteCount = BIG_BUF_SIZE - this->serialOutBufReadIndex + this->serialOutBufWriteIndex;
            byte arr[256];

            arr[0] = meterWireCommands::writeSerialData;
            arr[1] = (byteCount>28)?28:byteCount >> 8 & 0xff;
            arr[2] = (byteCount>28)?28:byteCount & 0xff;
            unsigned int length = 3;
            for(int i = 0; i < (byteCount>28)?28:byteCount ; i++){
                arr[i+3] = this->serialOutBuf[this->serialOutBufReadIndex++];
                length++;
                if(this->serialOutBufReadIndex >= BIG_BUF_SIZE) this->serialOutBufReadIndex = 0;
            }
            dWire.write(this->I2C_Addr,arr,length,0);
            if(byteCount >=28 ) delay(5); // give the nano time to deal with the data before looping again
        }

        
        if(serialReadTimer > serialReadTimerMillis){
            serialReadTimer = 0;
            byte arr2[1] = {meterWireCommands::getNumSerialBytesAvailable};
            if(dWire.write(this->I2C_Addr,arr2,1,0)){
                delayMicroseconds(500);
                if(dWire.request(this->I2C_Addr,arr2,1)){
                    uint8_t number = arr2[0];
                    while(number>0){
                        arr2[0] = meterWireCommands::getAvailableSerialBytes;
                        if(dWire.write(this->I2C_Addr,arr2,1,0)){
                            delayMicroseconds(500);
                            byte arr32[32];
                            if(dWire.request(this->I2C_Addr,arr32,(number>32)?32:number)){
                                Serial.write(SERIAL_START_BYTE1);
                                Serial.write(SERIAL_START_BYTE2);
                                Serial.write(0);
                                Serial.write((number>32)?32:number);
                                Serial.write(0xa2); // serial data class
                                Serial.write(this->id_ + 1);
                                for(int i = 0; i < ((number>32)?32:number); i++){
                                    Serial.write(arr32[i]);
                                }
                                if(number>32)number-=32;
                                else number = 0;
                            }
                        }else{
                            delayMicroseconds(500);
                        }
                    }
                }else{
                    delayMicroseconds(500);
                }
            }else{
                delayMicroseconds(500);
            }
        }
        if (meterReadTimer < meterReadTimeMillis)return;
        meterReadTimer = 0;
        if (this->meterReady_) {
            byte arr1[1] = {meterWireCommands::readAllSensorsAvg};
            if(dWire.write(this->I2C_Addr,arr1,1,0)){
                delayMicroseconds(500);
                byte arr25[25];
                if(dWire.request(this->I2C_Addr,arr25,meterWireNumBytesToRead::readAllSensors)){
                    long val = 0;
                    for (int b = 0; b < 6; b++) {
                        val |= (long(arr25[(4 * b) + 1]) << 24);
                        val |= (long(arr25[(4 * b) + 2]) << 16);
                        val |= (long(arr25[(4 * b) + 3]) << 8);
                        val |= long(arr25[(4 * b) + 4]);
                        this->avgReadings[b] = val;
                        val = 0;
                    }
                }else{
                    delayMicroseconds(500);
                }
            }else{
                delayMicroseconds(500);
            }

            arr1[0] = meterWireCommands::readAllSensorsMax;
            if(dWire.write(this->I2C_Addr,arr1,1,0)){
                delayMicroseconds(500);
                byte arr25[25];
                if(dWire.request(this->I2C_Addr,arr25,meterWireNumBytesToRead::readAllSensors)){
                    long val = 0;
                    for (int b = 0; b < 6; b++) {
                        val |= (long(arr25[(4 * b) + 1]) << 24);
                        val |= (long(arr25[(4 * b) + 2]) << 16);
                        val |= (long(arr25[(4 * b) + 3]) << 8);
                        val |= long(arr25[(4 * b) + 4]);
                        this->maxReadings[b] = val;
                        val = 0;
                    }
                }else{
                    delayMicroseconds(500);
                }
            }else{
                delayMicroseconds(500);
            }
            arr1[0] = meterWireCommands::readAllSensorsInstant;
            if(dWire.write(this->I2C_Addr,arr1,1,0)){
                delayMicroseconds(500);
                byte arr25[25];
                if(dWire.request(this->I2C_Addr,arr25,meterWireNumBytesToRead::readAllSensors)){
                    long val = 0;
                    for (int b = 0; b < 6; b++) {
                        val |= (long(arr25[(4 * b) + 1]) << 24);
                        val |= (long(arr25[(4 * b) + 2]) << 16);
                        val |= (long(arr25[(4 * b) + 3]) << 8);
                        val |= long(arr25[(4 * b) + 4]);
                        this->instantReadings[b] = val;
                        val = 0;
                    }
                }else{
                    delayMicroseconds(500);
                }
            }else{
                delayMicroseconds(500);
            }

        } else {
            this->meterReady_ = this->meterReady();
        }

        /// send all of the readings over "Serial" to the host
        for(int i = 0; i < 6; i++){
            Serial.write(SERIAL_START_BYTE1);
            Serial.write(SERIAL_START_BYTE2);
            int byteCount = 14;
            Serial.write(byteCount >> 8 & 0xff);
            Serial.write(byteCount & 0xff);
            Serial.write(this->deviceClass);
            Serial.write(this->id_ * 6 + i + 1);

            Serial.write(instantReadings[i] >> 24 & 0xff);
            Serial.write(instantReadings[i] >> 16 & 0xff);
            Serial.write(instantReadings[i] >> 8 & 0xff);
            Serial.write(instantReadings[i] & 0xff);

            Serial.write(avgReadings[i] >> 24 & 0xff);
            Serial.write(avgReadings[i] >> 16 & 0xff);
            Serial.write(avgReadings[i] >> 8 & 0xff);
            Serial.write(avgReadings[i] & 0xff);

            Serial.write(maxReadings[i] >> 24 & 0xff);
            Serial.write(maxReadings[i] >> 16 & 0xff);
            Serial.write(maxReadings[i] >> 8 & 0xff);
            Serial.write(maxReadings[i] & 0xff);
        }
        
    }

    // Check to see if both ammeters are ready for reading. Returns false if either one is not ready.
    bool meterReady() {        
        byte arr[2] = {meterWireCommands::getStatus};
        bool t = true;
        while(t){
            dWire.write(this->I2C_Addr,arr,1,10);
            delayMicroseconds(500);
            t = !dWire.request(this->I2C_Addr,arr,2);
        }
        this->deviceClass = arr[0];
        return arr[1] == 0xff;
    }

    void setAmmeter_VperA(uint16_t cal, uint8_t meter){
        byte arr[4] = {meterWireCommands::setSensorVperA,meter,cal>>8,cal&0xff};
        dWire.write(this->I2C_Addr,arr,4,10);
    }

    void resetOneSensorMax(uint8_t sensNum){
        Wire.beginTransmission(this->I2C_Addr);
        Wire.write(meterWireCommands::resetOneSensorMax);
        Wire.write(sensNum);
        Wire.endTransmission();
    }

    void resetAllSensorsMax(){
        Wire.beginTransmission(this->I2C_Addr);
        Wire.write(meterWireCommands::resetAllSensorsMax);
        Wire.endTransmission();
    }

    void readOneSensorMax(uint8_t meter){
        if(meter >= 6) return;
        Wire.beginTransmission(this->I2C_Addr);
        Wire.write(meterWireCommands::readOneSensorMax);
        Wire.write(meter);
        Wire.endTransmission();
        Wire.requestFrom(this->I2C_Addr, 5);
        long val = 0;
        val |= (long(Wire.read()) << 24);
        val |= (long(Wire.read()) << 16);
        val |= (long(Wire.read()) << 8);
        val |= long(Wire.read());
        this->maxReadings[meter] = val;
    }
    
    void readOneSensorInstant(uint8_t meter){
        if(meter >= 6) return;
        Wire.beginTransmission(this->I2C_Addr);
        Wire.write(meterWireCommands::readOneSensorInstant);
        Wire.write(meter);
        Wire.endTransmission();
        Wire.requestFrom(this->I2C_Addr, 5);
        long val = 0;
        val |= (long(Wire.read()) << 24);
        val |= (long(Wire.read()) << 16);
        val |= (long(Wire.read()) << 8);
        val |= long(Wire.read());
        this->instantReadings[meter] = val;
    }
    
    void readOneSensorAvg(uint8_t meter){
        if(meter >= 6) return;
        Wire.beginTransmission(this->I2C_Addr);
        Wire.write(meterWireCommands::readOneSensorAvg);
        Wire.write(meter);
        Wire.endTransmission();
        Wire.requestFrom(this->I2C_Addr, 5);
        long val = 0;
        val |= (long(Wire.read()) << 24);
        val |= (long(Wire.read()) << 16);
        val |= (long(Wire.read()) << 8);
        val |= long(Wire.read());
        this->avgReadings[meter] = val;
    }

    void setAveragerN_vals(uint8_t v, uint8_t meter){
        Wire.beginTransmission(this->I2C_Addr);
        Wire.write(meterWireCommands::setAveragerN_vals);
        Wire.write(meter);
        Wire.write(v);
        Wire.endTransmission();
    }

    bool addSerialDataToOutBuffer(byte* arr, unsigned int len){
        uint32_t byteCount = 0;
        if(this->serialOutBufReadIndex < this->serialOutBufWriteIndex) byteCount = this->serialOutBufWriteIndex - this->serialOutBufReadIndex;
        else byteCount = BIG_BUF_SIZE - this->serialOutBufReadIndex + this->serialOutBufWriteIndex;
        if(byteCount + len > BIG_BUF_SIZE)return false; // not enough room

        for(unsigned int i = 0; i < len; i++){
            this->serialOutBuf[this->serialOutBufWriteIndex++] = arr[i];
            if(this->serialOutBufWriteIndex >= BIG_BUF_SIZE) this->serialOutBufWriteIndex = 0;
        }

        return true; // there was room
    }

    void setSerialBaud(uint32_t baud){
        Wire.beginTransmission(this->I2C_Addr);
        Wire.write(meterWireCommands::setSerialBaud);
        Wire.write((baud >> 24) & 0xff);
        Wire.write((baud >> 16) & 0xff);
        Wire.write((baud >> 8) & 0xff);
        Wire.write(baud & 0xff);
        Wire.endTransmission();
    }
    
    void setMeterReadTime_ms(int time) {
        this->meterReadTimeMillis = time;
    }

    void setMeterReadTime_Hz(int hertz) {
        this->meterReadTimeMillis = 1000 / hertz;
    }
};

Meter_Manager* meters[MAX_NUMBER_OF_METERS];
SerialMidiManager* serialMidiDevs[16];
int nDevices = 0;
EXTMEM byte unprocessedSerialInData[BIG_BUF_SIZE];
unsigned int unprocessedSerialIn_readPntr = 0;
unsigned int unprocessedSerialIn_writePntr = 0;
unsigned int unprocessedSerialIn_numBytes = 0;

void setup() {
    delay(1000); // Give the sensors time to start
    Serial.begin(115200);
    while (!Serial) {}
    Wire.begin();
    Wire.setClock(400000);
    // Scan the Wire bus for all devices to find all the volt meters and ammeters
    byte error, address;
    for(address = 0; address <= 127; address++){
        Wire.beginTransmission(address);
        error = Wire.endTransmission();
        if(error == 0){
            meters[nDevices] = new Meter_Manager(address,nDevices);
            nDevices++;
            if(nDevices >= MAX_NUMBER_OF_METERS) break;
        }
        delay(2);
    }

    for(int i = 0; i < 8; i++){
        serialMidiDevs[i] = new SerialMidiManager(&HW_midi[i], i);
    }
    for(int i = 8; i < 16; i++){
        serialMidiDevs[i] = new SerialMidiManager(&SW_midi[i - 8], i);
    }
}
elapsedMillis heartBeat = 0;
void loop() {
    // call update() for all the meters
    for(int i = 0; i < nDevices; i++){
        meters[i]->update();
    }
    // call update() for all the midi ports
    for(int i = 0; i < 16; i++){
        serialMidiDevs[i]->update();
    }
    // get midi data from the usb
    if(usbMIDI.read()){
        byte type = usbMIDI.getType();
        byte channel = usbMIDI.getChannel();
        byte data1 = usbMIDI.getData1();
        byte data2 = usbMIDI.getData2();
        byte cable = usbMIDI.getCable();
        if (type != usbMIDI.SystemExclusive) {
            midi::MidiType mtype = (midi::MidiType)type;
            serialMidiDevs[cable]->midiDev->send(mtype,data1,data2,channel);
        }else{
            unsigned int SysExLength = data1 + data2 * 256;
            serialMidiDevs[cable]->midiDev->sendSysEx(SysExLength,usbMIDI.getSysExArray(),true);
        }
    }

    // Send a heartbeat to the host on a regular
    if(heartBeat > 1000){
        heartBeat = 0;
        Serial.write(SERIAL_START_BYTE1);
        Serial.write(SERIAL_START_BYTE2);
        Serial.write(((uint16_t)5 >> 8 ) & 0xff);
        Serial.write(((uint16_t)5) & 0xff);
        Serial.write(0xff);
        Serial.write(0x00);
        Serial.write(0xff);
        Serial.write(0x00);
        Serial.write(0xff);
    }

    // Read serial data into ring buffer
    while(Serial.available()){
        unprocessedSerialInData[unprocessedSerialIn_writePntr++] = Serial.read();
        if(unprocessedSerialIn_writePntr >= BIG_BUF_SIZE) unprocessedSerialIn_writePntr = 0;
        unprocessedSerialIn_numBytes++;
        if(unprocessedSerialIn_numBytes > BIG_BUF_SIZE) unprocessedSerialIn_numBytes = BIG_BUF_SIZE;
        if(unprocessedSerialIn_numBytes == BIG_BUF_SIZE) unprocessedSerialIn_readPntr++;
        if(unprocessedSerialIn_readPntr >= BIG_BUF_SIZE) unprocessedSerialIn_readPntr = 0;
    }


    bool serialProcessing = true;
    while(serialProcessing){
        // read through the buffer until we get to a start byte
        while(unprocessedSerialInData[unprocessedSerialIn_readPntr] != SERIAL_START_BYTE1 && unprocessedSerialIn_numBytes > 0){
            unprocessedSerialIn_readPntr++;
            unprocessedSerialIn_numBytes--;
        }
        // only continue if there are more than a headers worth of bytes in the buffer
        if(unprocessedSerialIn_numBytes > 4){
            if(unprocessedSerialInData[unprocessedSerialIn_readPntr + 1] == SERIAL_START_BYTE2){
                uint16_t byteCount = (unprocessedSerialInData[unprocessedSerialIn_readPntr + 2] >> 8) | unprocessedSerialInData[unprocessedSerialIn_readPntr + 3];
                if(unprocessedSerialIn_numBytes > byteCount){
                    uint8_t command = unprocessedSerialInData[unprocessedSerialIn_readPntr + 4];
                    switch(command){ //@TODO: finish all of he cases in this switch
                        case 0x01: //set VperA for ammeter
                            byte ammeterId = unprocessedSerialInData[unprocessedSerialIn_readPntr +5];
                            uint16_t calVal = (unprocessedSerialInData[unprocessedSerialIn_readPntr + 6] << 8) | unprocessedSerialInData[unprocessedSerialIn_readPntr + 7];
                            byte meterID = ammeterId  / 6; // meterId is the Arduino Nano
                            byte meterSubId = ammeterId % 6; // meteSubId is the ammeter connected to the Nano
                            if(meterID < nDevices){
                                meters[meterID]->setAmmeter_VperA(calVal, meterSubId);
                            }
                        break;
                        case 0x02: // set baud for serial port
                            byte meterId = unprocessedSerialInData[unprocessedSerialIn_readPntr + 5];
                            uint32_t baudR = 0;
                            baudR |= unprocessedSerialInData[unprocessedSerialIn_readPntr + 6] << 24;
                            baudR |= unprocessedSerialInData[unprocessedSerialIn_readPntr + 7] << 16;
                            baudR |= unprocessedSerialInData[unprocessedSerialIn_readPntr + 8] << 8;
                            baudR |= unprocessedSerialInData[unprocessedSerialIn_readPntr + 9];
                            if(meterId < nDevices){
                                meters[meterId]->setSerialBaud(baudR);
                            }
                        break;
                        case 0x03: // set ammeter read time in ms

                        break;
                        case 0x04: // set ammeter read time in Hz

                        break;
                        case 0x05: // set midi thru for port
                        break;
                        case 0x06: //  reboot the teensy
                        break;
                    }
                    
                    for(unsigned int i = 0; i < byteCount + 4; i++){
                        unprocessedSerialIn_readPntr++;
                        if(unprocessedSerialIn_readPntr >= BIG_BUF_SIZE) unprocessedSerialIn_readPntr = 0;
                        if(unprocessedSerialIn_numBytes > 0) unprocessedSerialIn_numBytes--;
                    }
                }else{
                    serialProcessing = false;
                }
            }else{
                unprocessedSerialIn_readPntr++;
                unprocessedSerialIn_numBytes--;
            }
        }else{
            serialProcessing = false;
        }
    }
}
