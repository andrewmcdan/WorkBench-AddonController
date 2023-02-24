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
#include <SoftwareSerial.h>

#define BIG_BUF_SIZE 65536
#define METER_READING_TIME_MS 50
#define MAX_NUMBER_OF_METERS 30
#define SERIAL_START_BYTE1 0b10101010
#define SERIAL_START_BYTE2 0b11001100

// these must be set according to which ammeters are soldered onto the PCB
//const int ammeter1_mVperA[6] = { 40,40,185,185,185,185 }; // 50A, 50A, 5A, 5A, 5A, 5A
//const int ammeter2_mVperA[6] = { 66,66,66,66,185,185 }; //  30A, 30A, 30A, 30A, 5A, 5A

char* bigBufs[MAX_NUMBER_OF_METERS];

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
        this->softSer = false;
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
    int I2C_Addr;
    long avgReadings[6] = {0};
    long maxReadings[6] = {0};
    long instantReadings[6] = {0};
    char* serialInBuf;
    char* serialOutBuf;
    uint16_t serialInBufWriteIndex = 0;
    uint16_t serialInBufReadIndex = 0;
    uint16_t serialOutBufWriteIndex = 0;
    uint16_t serialOutBufReadIndex = 0;
    
    elapsedMillis meterReadTimer = 0;
    uint32_t meterReadTimeMillis = METER_READING_TIME_MS;

public:
    Meter_Manager(int addr, byte id){
        this->I2C_Addr = addr;
        this->meterReady_ = this->meterReady();
        this->serialInBuf = (char*)extmem_malloc(BIG_BUF_SIZE);
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
        if(this->serialInBufReadIndex != this->serialInBufWriteIndex){
            Serial.write(SERIAL_START_BYTE1);
            Serial.write(SERIAL_START_BYTE2);
            uint32_t byteCount = 0;
            if(this->serialInBufReadIndex < this->serialInBufWriteIndex) byteCount = this->serialInBufWriteIndex - this->serialInBufReadIndex;
            else byteCount = BIG_BUF_SIZE - this->serialInBufReadIndex + this->serialInBufWriteIndex;
            Serial.write(byteCount >> 8 & 0xff);
            Serial.write(byteCount & 0xff);
            Serial.write(0xa2); // serial data class
            Serial.write(this->id_ + 1);
            while(this->serialInBufReadIndex != this->serialInBufWriteIndex){
                Serial.write(this->serialInBuf[this->serialInBufReadIndex++]);
                if(this->serialInBufReadIndex >= BIG_BUF_SIZE) this->serialInBufReadIndex = 0;
            }
        }

        // @TODO:
        // Run tests on this code to ensure that it is working correctly
        while(this->serialOutBufReadIndex != this->serialOutBufWriteIndex){
            uint32_t byteCount = 0;
            if(this->serialOutBufReadIndex < this->serialOutBufWriteIndex) byteCount = this->serialOutBufWriteIndex - this->serialOutBufReadIndex;
            else byteCount = BIG_BUF_SIZE - this->serialOutBufReadIndex + this->serialOutBufWriteIndex;
            Wire.beginTransmission(this->I2C_Addr);
            Wire.write(meterWireCommands::writeSerialData);
            // the receive buffer on the Arduino Nano is only 32 bytes, so we have to break this up to ensure 
            // we don't overrun the buffer and lose data.
            Wire.write((byteCount>28)?28:byteCount >> 8 & 0xff);
            Wire.write((byteCount>28)?28:byteCount & 0xff);
            for(int i = 0; i < (byteCount>28)?28:byteCount ; i++){
                Wire.write(this->serialOutBuf[this->serialOutBufReadIndex++]);
                if(this->serialOutBufReadIndex >= BIG_BUF_SIZE) this->serialOutBufReadIndex = 0;
            }
            Wire.endTransmission();
            if(byteCount >=28 ) delay(15); // give the nano time to deal with the data before looping again
        }
        if (meterReadTimer < meterReadTimeMillis)return;
        meterReadTimer = 0;
        if (this->meterReady_) {
            Wire.beginTransmission(this->I2C_Addr);
            Wire.write(meterWireCommands::readAllSensorsAvg);
            Wire.endTransmission();
            Wire.requestFrom(this->I2C_Addr, meterWireNumBytesToRead::readAllSensors);
            /*unsigned char numBytes = */Wire.read(); // first byte indicates number of bytes that will be read. Unused. 
            long val = 0;
            for (int b = 0; b < 6; b++) {
                val |= (long(Wire.read()) << 24);
                val |= (long(Wire.read()) << 16);
                val |= (long(Wire.read()) << 8);
                val |= long(Wire.read());
                this->avgReadings[b] = val;
                val = 0;
            }

            Wire.beginTransmission(this->I2C_Addr);
            Wire.write(meterWireCommands::readAllSensorsMax);
            Wire.endTransmission();
            Wire.requestFrom(this->I2C_Addr, meterWireNumBytesToRead::readAllSensors);
            /*numBytes = */Wire.read(); // first byte indicates number of bytes that will be read. Unused. 
            val = 0;
            for (int b = 0; b < 6; b++) {
                val |= (long(Wire.read()) << 24);
                val |= (long(Wire.read()) << 16);
                val |= (long(Wire.read()) << 8);
                val |= long(Wire.read());
                this->maxReadings[b] = val;
                val = 0;
            }

            Wire.beginTransmission(this->I2C_Addr);
            Wire.write(meterWireCommands::readAllSensorsInstant);
            Wire.endTransmission();
            Wire.requestFrom(this->I2C_Addr, meterWireNumBytesToRead::readAllSensors);
            /*numBytes = */Wire.read(); // first byte indicates number of bytes that will be read. Unused. 
            val = 0;
            for (int b = 0; b < 6; b++) {
                val |= (long(Wire.read()) << 24);
                val |= (long(Wire.read()) << 16);
                val |= (long(Wire.read()) << 8);
                val |= long(Wire.read());
                this->instantReadings[b] = val;
                val = 0;
            }

            Wire.beginTransmission(this->I2C_Addr);
            Wire.write(meterWireCommands::getNumSerialBytesAvailable);
            Wire.endTransmission();
            Wire.requestFrom(this->I2C_Addr, meterWireNumBytesToRead::getNumSerialBytesAvail);
            uint16_t number = 0;
            number |= (uint16_t(Wire.read()) << 8);
            number |= uint16_t(Wire.read());
            if(number > 0){
                Wire.beginTransmission(this->I2C_Addr);
                Wire.write(meterWireCommands::getAvailableSerialBytes);
                Wire.endTransmission();
                Wire.requestFrom(this->I2C_Addr, number);
                for(int i = 0; i < number; i++){
                    this->serialInBuf[this->serialInBufWriteIndex++] = Wire.read();
                    if(this->serialInBufWriteIndex >= BIG_BUF_SIZE) this->serialInBufWriteIndex = 0;
                }
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
        delay(100);
        Wire.beginTransmission(this->I2C_Addr);
        Wire.write(meterWireCommands::getStatus);
        Wire.endTransmission();
        Wire.requestFrom(this->I2C_Addr,meterWireNumBytesToRead::getStatus_B);
        unsigned char val1 = Wire.read();
        switch(val1){
            case 0xaa:
            {
                this->deviceClass = 0xa0; // ammeter
                break;
            }
            case 0xbb:
            {
                this->deviceClass = 0xa1; // voltmeter
                break;
            }
        }
        unsigned char val2 = Wire.read();
        return val2 == 0xff && val1 == 1;
    }

    void setAmmeter_VperA(int cal, uint8_t meter){
        Wire.beginTransmission(this->I2C_Addr);
        Wire.write(meterWireCommands::setSensorVperA);
        Wire.write(meter);
        unsigned int val = cal;
        Wire.write(val >> 8);
        Wire.write(val & 0xff);
        Wire.endTransmission();
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

    void setSerialBaud(uint16_t baud){
        Wire.beginTransmission(this->I2C_Addr);
        Wire.write(meterWireCommands::resetOneSensorMax);
        Wire.write(baud >> 8 & 0xff);
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

void setup() {
    delay(1000); // Give the sensors time to start
    Wire.begin();
    Wire.setClock(400000);
    // Scan the Wire bus for all devices to find all the volt meters and ammeters
    byte error, address;
    for(address = 1; address < 127; address++){
        Wire.beginTransmission(address);
        error = Wire.endTransmission();
        if(error == 0){
            meters[nDevices] = new Meter_Manager(address,nDevices);
            nDevices++;
            if(nDevices >= MAX_NUMBER_OF_METERS) break;
        }
    }

    for(int i = 0; i < 8; i++){
        serialMidiDevs[i] = new SerialMidiManager(&HW_midi[i], i);
    }
    for(int i = 8; i < 16; i++){
        serialMidiDevs[i] = new SerialMidiManager(&SW_midi[i - 8], i);
    }
    Serial.begin(115200);
    while (!Serial) {}
}
elapsedMillis watchdog = 1100;
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

    while(Serial.available()){
        //@TODO:
        // this entire section....
    }
}

