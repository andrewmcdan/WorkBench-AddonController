/**
 *  Board:          Teensy 4.1
 *  Overclock:      default
 *  
 */

#include <Arduino.h>
#include <Wire.h>
#include <PS2Keyboard.h>
#include "USBHost_t36.h"
#include <EEPROM.h>
#include <MIDI.h>
#include "Teensy-Main-lib.h"

#define DEBUG_NO_LCDS_PRESENT 1
#define DEBUG_USE_DEFAULT_SETTINGS 1
#define DEBUG_STACKED_USB_HUBS 1

#define ADDR_I2C_TO_LCD 0x6c
#define LCD_COMMAND_SET_COLS_ROWS 0x05
#define LCD_COMMAND_START_LCD 0x0b
#define LCD_COMMAND_MK_CUST_CHAR_BASE 0x20
#define LCD_COMMAND_SEND_CUST_CHAR 0x30
#define LCD_COMMAND_DISP_TEXT_BASE 0xa0
#define LCD_COMMAND_SET_BL_BRIGHTNESS 0xff
#define AMMETER1_ADDR 0x08
#define AMMETER2_ADDR 0x09
#define SCREEN_BUFFER_SIZE 20*4
#define SCROLLBACK_BUFFER_SIZE 20*100000
#define COMMAND_HX_SIZE 1000
#define COMMAND_HX_LEN 1000
#define PS2DATAPIN 31
#define PS2IRQPIN 32
#define SER_IN_BUF_SIZE 1048576 // 2^20 - This is quite large for a serial buffer, but why not?
#define AMMETER_READING_TIME_MS 100

// these must be set according to which ammeters are soldered onto the PCB
const int ammeter1_mVperA[6] = { 40,40,185,185,185,185 }; // 50A, 50A, 5A, 5A, 5A, 5A
const int ammeter2_mVperA[6] = { 66,66,66,66,185,185 }; //  30A, 30A, 30A, 30A, 5A, 5A

USBHost usb_Host;
USBHub usbHostHub1(usb_Host);
#if DEBUG_STACKED_USB_HUBS
USBHub usbHostHub2(usb_Host);
#endif
// USBHub usbHostHub3(usb_Host);
// USBHub usbHostHub4(usb_Host);
KeyboardController usbHostKeyboard(usb_Host);
// USBHIDParser usbHostHid1(usb_Host);
// USBHIDParser usbHostHid2(usb_Host);
MIDIDevice USBmidi(usb_Host);

EXTMEM char screenBuffer1[SCREEN_BUFFER_SIZE] = { ' ' };
EXTMEM char screenBuffer2[SCREEN_BUFFER_SIZE] = { ' ' };
EXTMEM char screenBuffer3[SCREEN_BUFFER_SIZE] = { ' ' };
EXTMEM char scrollbackBuffer[SCROLLBACK_BUFFER_SIZE] = { ' ' };
EXTMEM char commandHxArr[COMMAND_HX_SIZE * COMMAND_HX_LEN] = { 0 };
EXTMEM char bigBufferForIncomingSerial[SER_IN_BUF_SIZE];

MIDI_CREATE_INSTANCE(HardwareSerial, Serial3, serialMIDI);

enum ammeterWireCommands:int {
    readOneSensorAvg,
    readAllSensorsAvg,
    readSelectedSensorsAvg,
    setSensorVperA,
    getStatus,
    readOneSensorInstant,
    readOneSensorRaw
};
enum ammeterWireNumBytesToRead:int {
    readOneSensor = 5, 
    readAllSensors = 25, 
    readSelectedSensors = 5, 
    getStatus_B = 2
};


EXTMEM class Settings_s {
private:
    String ammeterNames[12] = {
        "12V Supply",
        "50A Extern",
        "3.3V #1 5A",
        "3.3V #2 5A",
        "5V #1 5A",
        "5V #2 5A",
        "12V #1 30A",
        "12V #2 30A",
        "5A Extern",
        "5A Extern",
        "30A Extern",
        "30A Extern"
    };
    int selectedAmmetersToDisplay[4] = { 0, 2, 4, 6 };
    int ammeterDisplayDecimalPoints[12] = { 2 };
    uint32_t serialInBaud = 115200;
    uint16_t ammeterReadTime_ms = 100;
    bool serialMidiThru = true;
    enum terminalInViewModes_e :int { ScrollEachLineToEndOfLine, TruncateLineAtEndOfScreen, WrapToNextLine };
    
public:
    Settings_s() {
#if DEBUG_USE_DEFAULT_SETTINGS == 0
        this->loadSettingsFromEEPROM();
#endif
    }

    void saveSettingsToEEPROM() {
        
    }

    void loadSettingsFromEEPROM() {
        
    }

    String getAmmeterName(int i) {
        return ammeterNames[i];
    }
}settings;

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

class LiquidCrystal_I2C_ {
private:
    int lcdID = 0;
    WireBuffer* wireBufForLCDs;
    int cursorRow = 0;
    int cursorCol = 0;
    ///\brief Read status from LCD controller and wait if not ready. Blocks code execution until LCD controller is ready for more data.
    ///This function is necessary to prevent interrupting the LCD controller while it is writing data to theLCD's.
    ///Must be called after every endTransmission when not using "WireBuffer::wireBufForLCDs".
    void waitForLCDready() {
        bool isReady = false;
        while (!isReady) {
            Wire.requestFrom(ADDR_I2C_TO_LCD, 4);
            byte readBytes[4] = { 0, 0, 0, 0xaa };
            int byteNum = 0;
            while (Wire.available()) {
                readBytes[byteNum] = Wire.read();
                byteNum++;
            }
            isReady = readBytes[3] == 0xff;
#if DEBUG_NO_LCDS_PRESENT
            break;
#endif
        }
    }

    ///\brief Determine if LCD controller is ready for more data.
    ///\return True is ready, false if not...
    bool isLCDready() {
        Wire.requestFrom(ADDR_I2C_TO_LCD, 4);
        byte readBytes[4] = { 0, 0, 0, 0xaa };
        int byteNum = 0;
        while (Wire.available()) {
            readBytes[byteNum] = Wire.read();
            byteNum++;
        }
        return readBytes[3] == 0xff;
    }
public:
    LiquidCrystal_I2C_(int id, WireBuffer* wireBuf) {
        this->lcdID = id;
        this->wireBufForLCDs = wireBuf;
    }
    void init() {
        Wire.beginTransmission(ADDR_I2C_TO_LCD); // transmit to device #8
        Wire.write(LCD_COMMAND_SET_COLS_ROWS);   // set cols and rows
        Wire.write(this->lcdID);
        Wire.write(20);
        Wire.write(4);
        Wire.endTransmission(); // stop transmitting
        this->waitForLCDready();
        Wire.beginTransmission(ADDR_I2C_TO_LCD); // transmit to device #8
        Wire.write(LCD_COMMAND_START_LCD);       // start lcd
        Wire.write(this->lcdID);                           // lcd id
        Wire.endTransmission();                  // stop transmitting
        this->waitForLCDready();
        
    }
    void backlight(int br = 255) {
        Wire.beginTransmission(ADDR_I2C_TO_LCD);
        Wire.write(LCD_COMMAND_SET_BL_BRIGHTNESS);
        Wire.write(br);
        Wire.endTransmission();
        this->waitForLCDready();
    }
    void print(String str) {
        int len = str.length();
        byte bytesToSend[len + 2];
        bytesToSend[0] = LCD_COMMAND_DISP_TEXT_BASE + this->lcdID;
        bytesToSend[1] = byte((this->cursorRow << 6) + this->cursorCol);
        for (uint16_t i = 2, p = 0; i < len + 2; i++, p++)
            bytesToSend[i] = str[p];
        // wait time calculated to be about 0.5ms for each byte of data to transfer.
        this->wireBufForLCDs->addEntryToBuffer(ADDR_I2C_TO_LCD, bytesToSend, len + 2, 282 * (len * 2));
    }
    void setCursor(int c, int r) {
        this->cursorCol = c;
        this->cursorRow = r;
    }
};

class LCD_manager{
private:
    char* screenBuffer;
    LiquidCrystal_I2C_* lcd;
    elapsedMillis millisecondTimer = 0;

public:
    LCD_manager(LiquidCrystal_I2C_* lcd, char buf[SCREEN_BUFFER_SIZE]) {
        lcd->init();
        lcd->backlight();
        this->screenBuffer = buf;
        this->lcd = lcd;
    }

    ~LCD_manager() {}

    void update() {
        if(millisecondTimer < 100) return;
        millisecondTimer = 0;
        // print screenBuffer to LCD
        for (int r = 0; r < 4; r++) {
            lcd->setCursor(0, r);
            for (int c = 0; c < 20; c++) {
                lcd->print(screenBuffer[(r * 20) + c]);
            } 
        }
    }

    // Add data to screenBuffer. Returns true if there is enough space given 
    // position and length.
    bool printToBuf(char* buf, uint8_t len, uint8_t col, uint8_t row) {
        if((row*20) + col + len > 80) return false;
        for (int i = 0; i < len; i++) {
            screenBuffer[(row * 20) + col] = buf[i];
        }
        return true;
    }

    void clearBuf() {
        for (int i = 0; i < 80; i++) {
            this->screenBuffer[i] = ' ';
        }
    }
};

class Terminal_manager{
private:
    LCD_manager* topLCD;
    LCD_manager* botLCD;
    char* scrollBack;
    char* commandHistory;
    int commandHx_commandIndex = 0;
    int commandHx_charIndex = 0;
    char* bigSerialBuffer;
    char tempCommand[COMMAND_HX_LEN] = { 0 };
    int tempCommandIndex = 0;
    uint32_t scrollBackPos = 0;
    uint32_t scrollBackBufPos = 0;
    PS2Keyboard ps2_keyboard;
    enum LCDmode :int { SerialIn, SerialOut, UsbMidiIn, UsbMidiOut, SerMidiIn, SerMidiOut, KbMidiOut } currentModeTop, currentModeBot;
    enum ToB :uint8_t { TOP, BOT };

    void printMidiNoteOnOffToLCD(uint8_t topOrBot, byte channel, byte note, byte velocity) {
        if (topOrBot == ToB::TOP) {
            this->topLCD->clearBuf();
            String temp = "Received MIDI on/off";
            char tempArr[20];
            temp.toCharArray(tempArr, 20);
            topLCD->printToBuf(tempArr, temp.length(), 0, 0);
            temp = "Chan: " + channel;
            temp.toCharArray(tempArr, 20);
            topLCD->printToBuf(tempArr, temp.length(), 0, 1);
            temp = "Note: " + note;
            temp.toCharArray(tempArr, 20);
            topLCD->printToBuf(tempArr, temp.length(), 0, 2);
            temp = "Velocity: " + velocity;
            temp.toCharArray(tempArr, 20);
            topLCD->printToBuf(tempArr, temp.length(), 0, 3);
        } else if (topOrBot == ToB::BOT) {
            this->botLCD->clearBuf();
            String temp = "Received MIDI on/off";
            char tempArr[20];
            temp.toCharArray(tempArr, 20);
            botLCD->printToBuf(tempArr, temp.length(), 0, 0);
            temp = "Chan: " + channel;
            temp.toCharArray(tempArr, 20);
            botLCD->printToBuf(tempArr, temp.length(), 0, 1);
            temp = "Note: " + note;
            temp.toCharArray(tempArr, 20);
            botLCD->printToBuf(tempArr, temp.length(), 0, 2);
            temp = "Velocity: " + velocity;
            temp.toCharArray(tempArr, 20);
            botLCD->printToBuf(tempArr, temp.length(), 0, 3);
        }
    }


public:
    Terminal_manager(LCD_manager* top, LCD_manager* bottom, char bufSB[SCROLLBACK_BUFFER_SIZE], char bufCmdHx[COMMAND_HX_SIZE * COMMAND_HX_LEN], char serInBuf[SER_IN_BUF_SIZE]) {
        this->bigSerialBuffer = serInBuf;
        this->commandHistory = bufCmdHx;
        this->scrollBack = bufSB;
        this->topLCD = top;
        this->botLCD = bottom;
        this->ps2_keyboard.begin(PS2DATAPIN, PS2IRQPIN);
        this->currentModeTop = LCDmode::SerialIn;
        this->currentModeBot = LCDmode::SerialOut;
        Serial8.begin(115200);
        Serial8.addMemoryForRead(bigSerialBuffer, SER_IN_BUF_SIZE);
    }

    ~Terminal_manager() {}

    void update(){
        if (this->ps2_keyboard.available()) this->handleKbEvent(this->ps2_keyboard.read());
        if (Serial8.available()) this->handleSerIn();

        switch (this->currentModeTop) {
        case LCDmode::SerialOut: {
            // find index of Null terminator
            int term = 0;
            for (int i = 0; i < COMMAND_HX_LEN; i++) if (this->tempCommand[i] == '\0') term = i;

            char tempArr[80] = { ' ' };
            for (int i = 79, p = 0; i >= 0; i--, p++) {
                tempArr[i] = this->tempCommand[term - p];
                if (term - p == 0)break;
            }
            topLCD->printToBuf(tempArr,term,0,0);
            break;
        }
        case LCDmode::SerialIn: {
            
            break;
        }
        }

        botLCD->update();
        topLCD->update();
    }

    void setScrollBackPos(uint32_t pos) {
        
    }

    void incrScrollBack() {
        
    }

    void decrScrollBack() {
        
    }

    void changeSerialBaud(int baud) {
        Serial8.end();
        Serial8.begin(baud);
        // Serial8.addMemoryForRead(bigSerialBuffer, SER_IN_BUF_SIZE);
    }

    void handleKbEvent(int key) {
        char keyChar = char(key);
        
        if (key < 127 && key > 31) {
            // if the key pressed was a printable character, add it the command hx array
            this->tempCommand[tempCommandIndex] = keyChar;
            if(tempCommandIndex < COMMAND_HX_LEN - 2) this->tempCommandIndex++;
        } else {
            // do specific things for each of the special keys that can be pressed e.g. enter, tab, backspace, del, arrows, etc.
            switch (key) {
            case 10: // "enter"
            {
                // terminate the string
                this->tempCommand[this->tempCommandIndex] = '\0';

                // Send the string to the serial port
                Serial8.println(this->tempCommand);

                for (int i = 0; i < COMMAND_HX_LEN; i++) {
                    this->commandHistory[(this->commandHx_commandIndex * COMMAND_HX_LEN) + i] = this->tempCommand[i];
                }
                for (int i = 0; i < COMMAND_HX_LEN; i++) {
                    this->tempCommand[i] = ' ';
                }

                // increment the index and wrap to 0
                if (this->commandHx_commandIndex < COMMAND_HX_SIZE - 1) this->commandHx_commandIndex++;
                else this->commandHx_commandIndex = 0;

                // reset index for temp command
                this->tempCommandIndex = 0;
                break;
            }
            case 127: // delete
            {
                
                break;
            }
            }
        }
        Serial.print("key (HEX): ");
        Serial.print(key, HEX);
        Serial.print("  - ");
        Serial.println(keyChar);
    }

    void handleSerIn() {
        while (Serial8.available()) {
            this->scrollBack[this->scrollBackBufPos] = Serial8.read();
            if (this->scrollBackBufPos < SCROLLBACK_BUFFER_SIZE - 1)this->scrollBackBufPos++;
            else this->scrollBackBufPos = 0;
        }
    }

    void handleUsbMIDInoteOnOff(byte channel, byte note, byte velocity) {
        if (this->currentModeTop == LCDmode::UsbMidiIn) this->printMidiNoteOnOffToLCD(ToB::TOP, channel, note, velocity);
        else if(this->currentModeBot == LCDmode::UsbMidiIn) this->printMidiNoteOnOffToLCD(ToB::BOT, channel, note, velocity);
        Serial.print("USB MIDI in -- chan: ");
        Serial.print(channel);
        Serial.print("  note: ");
        Serial.print(note);
        Serial.print("  vel: ");
        Serial.println(velocity);
    }

    void handleSerMIDInoteOnOff(byte channel, byte note, byte velocity) {
        if (this->currentModeTop == LCDmode::SerMidiIn) this->printMidiNoteOnOffToLCD(ToB::TOP, channel, note, velocity);
        else if(this->currentModeBot == LCDmode::SerMidiIn) this->printMidiNoteOnOffToLCD(ToB::BOT, channel, note, velocity);
        Serial.print("Serial MIDI in -- chan: ");
        Serial.print(channel);
        Serial.print("  note: ");
        Serial.print(note);
        Serial.print("  vel: ");
        Serial.println(velocity);
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

class Main_Screen_Manager{
private:
    LCD_manager* lcd;
    Ammeter_manager* meters;
    enum mode :int { ammeters_m, settings_m } curMode;

public:
    Main_Screen_Manager(LCD_manager* lcd, Ammeter_manager* meters) {
        this->lcd = lcd;
        this->meters = meters;
        this->curMode = mode::ammeters_m;
    }
    ~Main_Screen_Manager() {}

    void update() {
        if (this->curMode == mode::ammeters_m) {
            this->meters->update();
        }
        char testarr[] = "testarr";
        this->lcd->printToBuf(testarr, 4, 0, 0);
        this->lcd->update();
    }
};


// TODO: an external LCD display to show additional data.
class externalDisplay {
private:
    WireBuffer* wireBuffer;
    Ammeter_manager* ammeters;
public:
    externalDisplay(WireBuffer* wireB, Ammeter_manager* met) {
        this->wireBuffer = wireB;
        this->ammeters = met;
    }
};

WireBuffer wireBufForLCDs = WireBuffer(&Wire);
WireBuffer wireExternDisp = WireBuffer(&Wire1);
LiquidCrystal_I2C_ lcd1 = LiquidCrystal_I2C_(0, &wireBufForLCDs);
LiquidCrystal_I2C_ lcd2 = LiquidCrystal_I2C_(1, &wireBufForLCDs);
LiquidCrystal_I2C_ lcd3 = LiquidCrystal_I2C_(2, &wireBufForLCDs);
LCD_manager LCD_main(&lcd1, screenBuffer1);
LCD_manager LCD_TermTop(&lcd2, screenBuffer2);
LCD_manager LCD_TermBot(&lcd3, screenBuffer3);
Ammeter_manager meters(AMMETER1_ADDR,AMMETER2_ADDR);
Terminal_manager terminal(&LCD_TermTop, &LCD_TermBot, scrollbackBuffer, commandHxArr, bigBufferForIncomingSerial);
Main_Screen_Manager mainLCD(&LCD_main, &meters);
externalDisplay externDisp = externalDisplay(&wireExternDisp, &meters);

void setup() {
    settings.loadSettingsFromEEPROM();
    Wire.begin();
    Wire1.begin();
    Wire.setClock(400000);
    Wire1.setClock(400000);
    Serial.begin(115200);
    while (!Serial) {}
    Serial.print('#');
    usb_Host.begin();
    usbHostKeyboard.attachPress(onUsbKeyboardPress);
    serialMIDI.begin();
    serialMIDI.setThruFilterMode(midi::Thru::Full);
    serialMIDI.turnThruOn();
    serialMIDI.setHandleNoteOn(handleSerMidiNoteOnOff);
    serialMIDI.setHandleNoteOff(handleSerMidiNoteOnOff);
    USBmidi.setHandleNoteOn(handleUsbMidiNoteOnOff);
    USBmidi.setHandleNoteOff(handleUsbMidiNoteOnOff);
}
elapsedMillis watchdog = 1100;
void loop() {
    if (watchdog > 1000) {
        Serial.print('.');
        Serial8.println("test");
        watchdog = 0;
    }
    terminal.update();
    mainLCD.update();
    usb_Host.Task();
    USBmidi.read();
    wireBufForLCDs.sendNextOutMessage();
}

void onUsbKeyboardPress(int key) {
    terminal.handleKbEvent(key);
}

void handleSerMidiNoteOnOff(byte channel, byte note, byte velocity) {
    terminal.handleSerMIDInoteOnOff(channel, note, velocity);
}

void handleUsbMidiNoteOnOff(byte channel, byte note, byte velocity) {
    terminal.handleUsbMIDInoteOnOff(channel, note, velocity);
}