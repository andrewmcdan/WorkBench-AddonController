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

#define AMMETER1_ADDR 0x08
#define AMMETER2_ADDR 0x09
#define BIG_BUF_SIZE 65536 // 2^20 - This is quite large for a serial buffer, but why not?
#define AMMETER_READING_TIME_MS 100

// these must be set according to which ammeters are soldered onto the PCB
const int ammeter1_mVperA[6] = { 40,40,185,185,185,185 }; // 50A, 50A, 5A, 5A, 5A, 5A
const int ammeter2_mVperA[6] = { 66,66,66,66,185,185 }; //  30A, 30A, 30A, 30A, 5A, 5A

EXTMEM char bigBuffer1[BIG_BUF_SIZE];
EXTMEM char bigBuffer2[BIG_BUF_SIZE];
EXTMEM char bigBuffer3[BIG_BUF_SIZE];
EXTMEM char bigBuffer4[BIG_BUF_SIZE];
EXTMEM char bigBuffer5[BIG_BUF_SIZE];
EXTMEM char bigBuffer6[BIG_BUF_SIZE];
EXTMEM char bigBuffer7[BIG_BUF_SIZE];
EXTMEM char bigBuffer8[BIG_BUF_SIZE];

//char* bigBufs[8] = {&bigBuffer1,&bigBuffer2,&bigBuffer3,&bigBuffer4,&bigBuffer5,&bigBuffer6,&bigBuffer7,&bigBuffer8};

MIDI_CREATE_INSTANCE(HardwareSerial, Serial1, serialMIDI_1);
MIDI_CREATE_INSTANCE(HardwareSerial, Serial2, serialMIDI_2);
MIDI_CREATE_INSTANCE(HardwareSerial, Serial3, serialMIDI_3);
MIDI_CREATE_INSTANCE(HardwareSerial, Serial4, serialMIDI_4);
MIDI_CREATE_INSTANCE(HardwareSerial, Serial5, serialMIDI_5);
MIDI_CREATE_INSTANCE(HardwareSerial, Serial6, serialMIDI_6);
MIDI_CREATE_INSTANCE(HardwareSerial, Serial7, serialMIDI_7);
MIDI_CREATE_INSTANCE(HardwareSerial, Serial8, serialMIDI_8);

enum ammeterWireCommands:int {
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
enum ammeterWireNumBytesToRead:int {
    readOneSensor = 5, 
    readAllSensors = 25, 
    readSelectedSensors = 5, 
    getStatus_B = 2
};


///\brief i2c/Wire buffer. 64 message FIFO buffer that can contain 64 byte messages. This object allows for very fast writing to the Wire bus.
///\param dev I2C device to use i.e. &Wire, &Wire1, etc.
struct WireBuffer {
    // data buffer
    struct BUF {
        byte data[256];
        byte address;
        uint8_t len;
        uint16_t waitTime = 0;
    };
    uint8_t last_in = 0;   // index of the last in data
    uint8_t first_out = 0; // index of the first data to write out
    elapsedMicros microTime = 0;
    BUF buffer[64];
    TwoWire* wireDev;
    WireBuffer(TwoWire* dev) { this->wireDev = dev; }
    ///\brief Add an entry to wire buffer to be sent at some point in the future.
    ///\param newData_addr Address to send this entry to.
    ///\param newData_arr Data to send
    ///\param newData_len Length of the data array.
    ///\param newData_wait Optional. Minimum time to wait in microseconds from when the previous message was sent
    /// to send this message. Useful for operations that stall the i2c device. 
    ///\return True if the data was added to the buffer. False if the buffer was full.
    bool addEntryToBuffer(byte newData_addr, byte newData_arr[], uint8_t newData_len, uint16_t newData_wait = 0) {
        // Serial.println("Adding entry to wire buffer.");
        if (this->last_in == this->first_out - 1) {
            return false; // buffer is full
        }
        this->buffer[this->last_in].address = newData_addr;
        this->buffer[this->last_in].len = newData_len;
        this->buffer[this->last_in].waitTime = newData_wait;
        for (uint8_t i = 0; i < newData_len; i++) {
            this->buffer[this->last_in].data[i] = newData_arr[i];
        }
        this->last_in++;
        if (this->last_in == 64) {
            this->last_in = 0;
        }
        return true;
    }
    ///\brief Sends the next available message in th is buffer is wait time has been satisfied. Needs to be called
    /// on a regular basis.
    uint8_t sendNextOutMessage() {
        if (this->last_in == this->first_out) return 0;
        if (this->microTime > this->buffer[this->first_out > 0 ? this->first_out - 1 : 63].waitTime) {
            this->wireDev->beginTransmission(this->buffer[this->first_out].address);
            for (uint8_t i = 0; i < this->buffer[this->first_out].len; i++)
                // On Teensy4.1 (and probably others) Wire.write returns as soon as the data is added to the output buffer.
                // We don't wait for it to get transmitted, as that is handled in hardware, so this is very fast code.
                this->wireDev->write(this->buffer[this->first_out].data[i]);
            this->wireDev->endTransmission();
            this->first_out++;
            if (this->first_out == 64) {
                this->first_out = 0;
            }
            this->microTime = 0;
            return this->last_in - this->first_out;
        }
        return 0xff;
    }
};


class Ammeter_manager {
private:
    int meter1I2C_Addr;
    int_fast16_t meter2I2C_Addr;
    long readings[12] = {0};
    bool bothMetersReady = false;
    elapsedMillis meterReadTimer = 0;
    uint32_t meterReadTimeMillis = AMMETER_READING_TIME_MS;

public:
    Ammeter_manager(int meter1addr, int meter2addr) {
        this->meter1I2C_Addr = meter1addr;
        this->meter2I2C_Addr = meter2addr;
        this->bothMetersReady = this->ammetersReady();
    }
    ~Ammeter_manager() {}

    void update() {
        if (meterReadTimer < meterReadTimeMillis)return;
        meterReadTimer = 0;
        if (this->bothMetersReady) {
            Wire.beginTransmission(this->meter1I2C_Addr);
            Wire.write(ammeterWireCommands::readAllSensorsAvg);
            Wire.endTransmission();
            Wire.requestFrom(this->meter1I2C_Addr, ammeterWireNumBytesToRead::readAllSensors);
            unsigned char numBytes = Wire.read();
            long val = 0;
            for (int b = 0; b < 6; b++) {
                val |= (long(Wire.read()) << 24);
                val |= (long(Wire.read()) << 16);
                val |= (long(Wire.read()) << 8);
                val |= long(Wire.read());
                this->readings[b] = val;
                val = 0;
            }

            Wire.beginTransmission(this->meter2I2C_Addr);
            Wire.write(ammeterWireCommands::readAllSensorsAvg);
            Wire.endTransmission();
            Wire.requestFrom(this->meter2I2C_Addr,ammeterWireNumBytesToRead::readAllSensors);
            numBytes = Wire.read();
            val = 0;
            for (int b = 6; b < 12; b++) {
                val |= (long(Wire.read()) << 24);
                val |= (long(Wire.read()) << 16);
                val |= (long(Wire.read()) << 8);
                val |= long(Wire.read());
                this->readings[b] = val;
                val = 0;
            }
        } else {
            this->bothMetersReady = this->ammetersReady();
        }
    }

    // Check to see if both ammeters are ready for reading. Returns false if either one is not ready.
    bool ammetersReady() {
        delay(500);
        for (int i = 0; i < 6; i++) {
            Wire.beginTransmission(this->meter1I2C_Addr);
            Wire.write(ammeterWireCommands::setSensorVperA);
            Wire.write(0);
            unsigned int val = ammeter1_mVperA[i];
            Wire.write(val >> 8);
            Wire.write(val & 0xff);
            Wire.endTransmission();
        }

        for (int i = 0; i < 6; i++) {
            Wire.beginTransmission(this->meter2I2C_Addr);
            Wire.write(ammeterWireCommands::setSensorVperA);
            Wire.write(0);
            unsigned int val = ammeter2_mVperA[i];
            Wire.write(val >> 8);
            Wire.write(val & 0xff);
            Wire.endTransmission();
        }
        delay(100);
        Wire.beginTransmission(this->meter1I2C_Addr);
        Wire.write(ammeterWireCommands::getStatus);
        Wire.endTransmission();
        Wire.requestFrom(this->meter1I2C_Addr,ammeterWireNumBytesToRead::getStatus_B);
        unsigned char val1 = Wire.read();
        unsigned char val2 = Wire.read();
        if (val2 != 0xff) return false;

        Wire.beginTransmission(this->meter2I2C_Addr);
        Wire.write(ammeterWireCommands::getStatus);
        Wire.endTransmission();
        Wire.requestFrom(this->meter2I2C_Addr,ammeterWireNumBytesToRead::getStatus_B);
        val1 = Wire.read();
        val2 = Wire.read();
        return val2 == 0xff;
    }

    void setMeterReadTime_ms(int time) {
        this->meterReadTimeMillis = time;
    }

    void setMeterReadTime_Hz(int hertz) {
        this->meterReadTimeMillis = 1000 / hertz;
    }
};

Ammeter_manager meters(AMMETER1_ADDR,AMMETER2_ADDR);

void setup() {
    Wire.begin();
    Wire1.begin();
    Wire.setClock(400000);
    Wire1.setClock(400000);
    Serial.begin(115200);
    while (!Serial) {}
    //Serial.print('#');
    //serialMIDI.begin(); 
    //serialMIDI.setThruFilterMode(midi::Thru::Full);
    //serialMIDI.turnThruOn();
    //serialMIDI.setHandleNoteOn(handleSerMidiNoteOnOff);
    //serialMIDI.setHandleNoteOff(handleSerMidiNoteOnOff);
    //USBmidi.setHandleNoteOn(handleUsbMidiNoteOnOff);
    //USBmidi.setHandleNoteOff(handleUsbMidiNoteOnOff);
}
elapsedMillis watchdog = 1100;
void loop() {
    if (watchdog > 1000) {
        Serial.print('.');
        Serial8.println("test");
        watchdog = 0;
    }
    Serial.write(0b10101010);
    Serial.write(0b11001100);
    Serial.write(0);
    Serial.write(14);
    Serial.write(0xa0);
    Serial.write(1);

    Serial.write(0);
    Serial.write(0);
    Serial.write(0);
    Serial.write(0);
    
    Serial.write(0);
    Serial.write(0);
    Serial.write(0);
    Serial.write(0);
    
    Serial.write(0);
    Serial.write(0);
    Serial.write(0);
    Serial.write(0);
}

void handleSerMidiNoteOnOff(byte channel, byte note, byte velocity) {
    //terminal.handleSerMIDInoteOnOff(channel, note, velocity);
}

void handleUsbMidiNoteOnOff(byte channel, byte note, byte velocity) {
    //terminal.handleUsbMIDInoteOnOff(channel, note, velocity);
}