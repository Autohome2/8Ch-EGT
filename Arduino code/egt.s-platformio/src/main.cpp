#include <Arduino.h>
#include "MAX31855.h"

#define DEBUG 1

#ifndef DEBUG
#define DEBUG 0
#endif

#if DEBUG > 0
#define LOG_D Serial.printf
#else
#define LOG_D
#endif

#define EGT_SPEED            100


HardwareSerial Serial3(USART3); //for some reason this isn't defined in arduino_core_stm32

#define Sensor1_4_CAN_ADDRESS     0x20A //the data from sensers 1-4 will be sent using this address
#define Sensor5_8_CAN_ADDRESS     0x20B //the data from sensers 5-8 will be sent using this address

enum BitRate { CAN_50KBPS, CAN_100KBPS, CAN_125KBPS, CAN_250KBPS, CAN_500KBPS, CAN_1000KBPS };

BitRate bitRateArray[4] = {
    CAN_500KBPS,
    CAN_1000KBPS,
    CAN_125KBPS,
    CAN_250KBPS,
};


typedef struct {
    uint16_t id;
    uint8_t  data[8];
    uint8_t  len;
} CAN_msg_t;

typedef const struct {
    uint8_t TS2;
    uint8_t TS1;
    uint8_t BRP;
} CAN_bit_timing_config_t;

CAN_bit_timing_config_t can_configs[6] = {{2, 13, 45}, {2, 15, 20}, {2, 13, 18}, {2, 13, 9}, {2, 15, 4}, {2, 15, 2}};

CAN_msg_t CAN_msg_14;        // data from egt 1-4 that will be sent to CAN bus
CAN_msg_t CAN_msg_58;        // data from egt 5-8 that will be sent to CAN bus
uint16_t  canAddress;
uint8_t   canin_channel;

extern CAN_bit_timing_config_t can_configs[6];

uint8_t cs[8]           = { PA0, PA1, PA2, PA3, PB0, PB1, PB13, PB12 }; //chip select pins for the MAX31855 chips
int32_t rawData[8]      = { 0 }; //raw data from all 8 MAX31855 chips

int16_t egt[8]          = { 0 }; //calculated egt values from all 8 MAX31855 chips
int16_t coldJunction[8] = { 0 }; //calculated cold junction temperatures from all 8 MAX31855 chips, not needed for any other than debugging

uint8_t max31855_OK_bits; // this is used to check if all MAX31855 chips are working or even installed and code skips the ones that don't work.

MAX31855 MAX31855_chips[8] = {
    MAX31855(cs[0]),
    MAX31855(cs[1]),
    MAX31855(cs[2]),
    MAX31855(cs[3]),
    MAX31855(cs[4]),
    MAX31855(cs[5]),
    MAX31855(cs[6]),
    MAX31855(cs[7])
};


enum IdCan {
    ID_CAN0,
    ID_CAN1,
    ID_CAN2,
    ID_CAN3,
    ID_CAN4,
    ID_CAN_SIZE
};

enum IdPins {
    ID_PIN0        = PB3,
    ID_PIN1        = PB4,
    ID_PIN2        = PB5,
    ID_PIN3        = PB6,
    ID_PIN4        = PB7,
    CAN_SPEED_PIN1 = PB8,
    CAN_SPEED_PIN2 = PB9
};

struct IdAddrCan {
    uint8_t pin;
    uint8_t shift;
};

IdAddrCan pins[ID_CAN_SIZE] = {
    { ID_PIN0,  4 },
    { ID_PIN1,  5 },
    { ID_PIN2,  6 },
    { ID_PIN3,  7 },
    { ID_PIN4, 10 },
};

void canInit(enum BitRate bitrate) {
    RCC->APB1ENR   |= 0x2000000UL;            // Enable CAN clock 
    RCC->APB2ENR   |= 0x1UL;                  // Enable AFIO clock
    AFIO->MAPR     &= 0xFFFF9FFF;             // reset CAN remap
    AFIO->MAPR     |= 0x00000000;             //   set CAN remap, use PA11, PA12

    RCC->APB2ENR   |= 0x4UL;                  // Enable GPIOA clock (bit2 to 1)
    GPIOA->CRH     &= 0xFFF00FFF;
    GPIOA->CRH     |= 0xB8000UL;              // Configure PA11 and PA12
    GPIOA->ODR     |= 0x1000UL;

    CAN1->MCR = 0x51UL;                       // Set CAN to initialization mode
     
    // Set bit rates 
    CAN1->BTR &= ~(((0x03) << 24) | ((0x07) << 20) | ((0x0F) << 16) | (0x1FF)); 
    CAN1->BTR |=  (((can_configs[bitrate].TS2-1) & 0x07) << 20) | (((can_configs[bitrate].TS1-1) & 0x0F) << 16) | ((can_configs[bitrate].BRP-1) & 0x1FF);

    // Configure Filters to default values
    CAN1->FMR    |=     0x1UL;                // Set to filter initialization mode
    CAN1->FMR    &= 0xFFFFC0FF;               // Clear CAN2 start bank
    CAN1->FMR    |= 0x1C << 8;                // Assign all filters to CAN1
    CAN1->FA1R   &= ~(0x1UL);                 // Deactivate filter 0
    CAN1->FS1R   |=     0x1UL;                // Set first filter to single 32 bit configuration

    CAN1->sFilterRegister[0].FR1 = 0x0UL;     // Set filter registers to 0
    CAN1->sFilterRegister[0].FR2 = 0x0UL;     // Set filter registers to 0
    CAN1->FM1R   &= ~(0x1UL);                 // Set filter to mask mode

    CAN1->FFA1R  &= ~(0x1UL);                 // Apply filter to FIFO 0    
    CAN1->FA1R   |=     0x1UL;                // Activate filter 0
    
    CAN1->FMR    &= ~(0x1UL);                 // Deactivate initialization mode
    CAN1->MCR    &= ~(0x1UL);                 // Set CAN to normal mode 
    while(CAN1->MSR & 0x1UL); 
 }

void setup() {
    pinMode(CAN_SPEED_PIN1, INPUT_PULLUP);
    pinMode(CAN_SPEED_PIN2, INPUT_PULLUP);
    uint16_t canMask = 0;
    for(uint8_t i = 0; i < ID_CAN_SIZE; ++i) {
        pinMode(pins[i].pin, INPUT_PULLUP);
        delay(1);
        canMask |= (!digitalRead(pins[i].pin)) << pins[i].shift;
    }
    max31855_OK_bits = 0;
    Serial.begin(115200); //debug
    Serial3.begin(115200); //data to speeduino
    BitRate canSpeed = bitRateArray[digitalRead(CAN_SPEED_PIN2) <<1 | digitalRead(CAN_SPEED_PIN1)];
    canInit(canSpeed); //init can at 500KBPS speed
    CAN_msg_14.len = 8; //8 bytes in can message
    CAN_msg_58.len = 8;
    CAN_msg_14.id = Sensor1_4_CAN_ADDRESS | canMask;
    CAN_msg_58.id = Sensor5_8_CAN_ADDRESS | canMask;

    // begin communication to MAX31855 chips
    for(uint32_t i = 0; i < 8; ++i) {
        MAX31855_chips[i].begin();
    }

    // check that all MAX31855 chips work or are even installed
    for(uint32_t i = 0; i < 8; ++i) {
        if (MAX31855_chips[i].readRawData() != 0) {             //we will just get 0 if there is no chip at all.
            if (MAX31855_chips[i].getChipID() != MAX31855_ID)   //if there is something there, check that it responds like MAX31855 (maybe useless step)
            {
                LOG_D("MAX31855 %d Error\r\n", i+1);
            }
            else{
                bitSet(max31855_OK_bits, i);                    //set corresponding bit to 1
            }
        }
        else{
            LOG_D("Can't find MAX31855 %d\r\n", i+1);           //nothing in this CS line
        }
    }
}

void sendDataToSpeeduino() {
    // Consider Serial.printf("G1%d", canin_channel);
    Serial3.write("G");                 // reply "G" cmd
    Serial3.write(1);                     //send 1 to confirm cmd received and valid
    Serial3.write(canin_channel);           //confirms the destination channel
    if((canAddress & Sensor1_4_CAN_ADDRESS) == Sensor1_4_CAN_ADDRESS) {
        for(uint32_t i = 0; i < 8; ++i) {
                Serial3.write(CAN_msg_14.data[i]);
        }
    }
    if((canAddress & Sensor5_8_CAN_ADDRESS) == Sensor1_4_CAN_ADDRESS) {
        for(uint32_t i = 0; i < 8; ++i) {
                Serial3.write(CAN_msg_58.data[i]);
        }
    }
}

void checkDataRequest() {
    if(Serial3.read() == 'R') {
         uint8_t tmp0;
         uint8_t tmp1;
         if(Serial3.available() >= 3) {
            canin_channel = Serial3.read();
            tmp0 = Serial3.read();          //read in lsb of source can address 
            tmp1 = Serial3.read();          //read in msb of source can address
            canAddress = tmp1<<8 | tmp0 ;
            if((canAddress & Sensor1_4_CAN_ADDRESS) == Sensor1_4_CAN_ADDRESS 
            || (canAddress & Sensor5_8_CAN_ADDRESS) == Sensor1_4_CAN_ADDRESS) {    //check if the speeduino request includes adresses for EGTs
                sendDataToSpeeduino();      //request ok, send the data to speeduino
            }
         }
    }
}


void canSend(CAN_msg_t* CAN_tx_msg) {
    volatile uint32_t count = 0;
     
    CAN1->sTxMailBox[0].TIR   = (CAN_tx_msg->id) << 21;
    
    CAN1->sTxMailBox[0].TDTR &= ~(0xF);
    CAN1->sTxMailBox[0].TDTR |= CAN_tx_msg->len & 0xFUL;
    
    CAN1->sTxMailBox[0].TDLR  = (((uint32_t) CAN_tx_msg->data[3] << 24) |
                                 ((uint32_t) CAN_tx_msg->data[2] << 16) |
                                 ((uint32_t) CAN_tx_msg->data[1] <<  8) |
                                 ((uint32_t) CAN_tx_msg->data[0]      ));
    CAN1->sTxMailBox[0].TDHR  = (((uint32_t) CAN_tx_msg->data[7] << 24) |
                                 ((uint32_t) CAN_tx_msg->data[6] << 16) |
                                 ((uint32_t) CAN_tx_msg->data[5] <<  8) |
                                 ((uint32_t) CAN_tx_msg->data[4]      ));
    CAN1->sTxMailBox[0].TIR  |= 0x1UL;
    while(CAN1->sTxMailBox[0].TIR & 0x1UL && count++ < 1000000);
            
    // if (!(CAN1->sTxMailBox[0].TIR & 0x1UL)) return;
     
    //Sends error log to screen
    if(CAN1->sTxMailBox[0].TIR & 0x1UL) {
       LOG_D("ESR %5lu MSR %5lu TSR %5lu\r\n", CAN1->ESR, CAN1->MSR, CAN1->TSR);
    }
 }

void loop() {
    static uint32_t egtQueryStamp = 0;
    uint32_t currentTime = millis();
    if((currentTime - egtQueryStamp) > EGT_SPEED) {
        egtQueryStamp = currentTime;
        for(uint32_t i = 0; i < 8; ++i) {
            if(bitRead(max31855_OK_bits, i) == 1) {
                rawData[i] = MAX31855_chips[i].readRawData();
                LOG_D("EGT%d:   ", i+1);
                switch(MAX31855_chips[i].detectThermocouple(rawData[i])) {
                    case MAX31855_THERMOCOUPLE_OK:
                        //egt[i]    = (MAX31855_chips[i].getTemperature(rawData[i]));
                        egt[i]    = (rawData[i] >> 18)/4;
                        LOG_D("Temp: %6d\r\n", egt[i]);
                        break;
                    case MAX31855_THERMOCOUPLE_SHORT_TO_VCC:
                        LOG_D("SHORT TO VCC\r\n");
                        egt[i]    = 2000;
                        break;
                    case MAX31855_THERMOCOUPLE_SHORT_TO_GND:
                        LOG_D("SHORT TO GND\r\n");
                        egt[i]    = 3000;
                        break;
                    case MAX31855_THERMOCOUPLE_NOT_CONNECTED:
                        LOG_D("NOT CONNECTED\r\n");
                        egt[i]    = 4000;
                        break;
                    default:
                        LOG_D("MAX31855 ERROR\r\n");
                        egt[i]    = 5000;
                        break;
                }
                //coldJunction[i]   = (MAX31855_chips[i].getColdJunctionTemperature(rawData[i]));
                coldJunction[i]   = ((rawData[i] & 0xFFFF) >> 4)/16;
                if (i < 4) {
                    CAN_msg_14.data[2*i]   = lowByte ((uint16_t)(egt[i]));
                    CAN_msg_14.data[2*i+1] = highByte((uint16_t)(egt[i]));
                } else{
                    CAN_msg_58.data[2*i-8] = lowByte ((uint16_t)(egt[i]));
                    CAN_msg_58.data[2*i-7] = highByte((uint16_t)(egt[i]));
                }
                LOG_D("Cold Junction %5d \r\n Temp: %6d", i+1, coldJunction[i]);
            }
        }
        canSend(&CAN_msg_14);
        canSend(&CAN_msg_58);
    }

    while(Serial3.available () > 0) {       //is there data on serial3, presumably from speeduino
        checkDataRequest();                 //there is data, but is request from speeduino and is it for EGTs
    }
}