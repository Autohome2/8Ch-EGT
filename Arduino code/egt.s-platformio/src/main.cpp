#include <Arduino.h>
#include "MAX31855.h"

//#define DEBUG 1

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

//typedef const struct {
//    uint8_t TS2;
//    uint8_t TS1;
//    uint8_t BRP;
//} CAN_bit_timing_config_t;
//
//CAN_bit_timing_config_t can_configs[6] = {{2, 13, 45}, {2, 15, 20}, {2, 13, 18}, {2, 13, 9}, {2, 15, 4}, {2, 15, 2}};



CAN_msg_t CAN_msg_14;        // data from egt 1-4 that will be sent to CAN bus
CAN_msg_t CAN_msg_58;        // data from egt 5-8 that will be sent to CAN bus
uint16_t  canAddress;
uint8_t   canin_channel;

//extern CAN_bit_timing_config_t can_configs[6];

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

/* Real speed for bit rate of CAN message                                    */
uint32_t SPEED[6] = {50*1000, 100*1000, 125*1000, 250*1000, 500*1000, 1000*1000};
typedef struct
{
    uint16_t baud_rate_prescaler;                /// [1 to 1024]
    uint8_t time_segment_1;                      /// [1 to 16]
    uint8_t time_segment_2;                      /// [1 to 8]
    uint8_t resynchronization_jump_width;        /// [1 to 4] (recommended value is 1)
} CAN_bit_timing_config_t;

#define CAN_STM32_ERROR_UNSUPPORTED_BIT_RATE     1000
#define CAN_STM32_ERROR_MSR_INAK_NOT_SET         1001
#define CAN_STM32_ERROR_MSR_INAK_NOT_CLEARED     1002
#define CAN_STM32_ERROR_UNSUPPORTED_FRAME_FORMAT 1003

/*
 * Calculation of bit timing dependent on peripheral clock rate
 */
int16_t ComputeCANTimings(const uint32_t peripheral_clock_rate,
                          const uint32_t target_bitrate,
                          CAN_bit_timing_config_t* out_timings)
{
    if (target_bitrate < 1000)
    {
        return -CAN_STM32_ERROR_UNSUPPORTED_BIT_RATE;
    }

    //assert(out_timings != NULL);  // NOLINT
    memset(out_timings, 0, sizeof(*out_timings));

    /*
     * Hardware configuration
     */
    static const uint8_t MaxBS1 = 16;
    static const uint8_t MaxBS2 = 8;

    /*
     * Ref. "Automatic Baudrate Detection in CANopen Networks", U. Koppe, MicroControl GmbH & Co. KG
     *      CAN in Automation, 2003
     *
     * According to the source, optimal quanta per bit are:
     *   Bitrate        Optimal Maximum
     *   1000 kbps      8       10
     *   500  kbps      16      17
     *   250  kbps      16      17
     *   125  kbps      16      17
     */
    const uint8_t max_quanta_per_bit = (uint8_t)((target_bitrate >= 1000000) ? 10 : 17);    // NOLINT
    //assert(max_quanta_per_bit <= (MaxBS1 + MaxBS2));

    static const uint16_t MaxSamplePointLocationPermill = 900;

    /*
     * Computing (prescaler * BS):
     *   BITRATE = 1 / (PRESCALER * (1 / PCLK) * (1 + BS1 + BS2))       -- See the Reference Manual
     *   BITRATE = PCLK / (PRESCALER * (1 + BS1 + BS2))                 -- Simplified
     * let:
     *   BS = 1 + BS1 + BS2                                             -- Number of time quanta per bit
     *   PRESCALER_BS = PRESCALER * BS
     * ==>
     *   PRESCALER_BS = PCLK / BITRATE
     */
    const uint32_t prescaler_bs = peripheral_clock_rate / target_bitrate;

    /*
     * Searching for such prescaler value so that the number of quanta per bit is highest.
     */
    uint8_t bs1_bs2_sum = (uint8_t)(max_quanta_per_bit - 1);    // NOLINT

    while ((prescaler_bs % (1U + bs1_bs2_sum)) != 0)
    {
        if (bs1_bs2_sum <= 2)
        {
            return -CAN_STM32_ERROR_UNSUPPORTED_BIT_RATE;          // No solution
        }
        bs1_bs2_sum--;
    }

    const uint32_t prescaler = prescaler_bs / (1U + bs1_bs2_sum);
    if ((prescaler < 1U) || (prescaler > 1024U))
    {
        return -CAN_STM32_ERROR_UNSUPPORTED_BIT_RATE;              // No solution
    }

    /*
     * Now we have a constraint: (BS1 + BS2) == bs1_bs2_sum.
     * We need to find such values so that the sample point is as close as possible to the optimal value,
     * which is 87.5%, which is 7/8.
     *
     *   Solve[(1 + bs1)/(1 + bs1 + bs2) == 7/8, bs2]  (* Where 7/8 is 0.875, the recommended sample point location *)
     *   {{bs2 -> (1 + bs1)/7}}
     *
     * Hence:
     *   bs2 = (1 + bs1) / 7
     *   bs1 = (7 * bs1_bs2_sum - 1) / 8
     *
     * Sample point location can be computed as follows:
     *   Sample point location = (1 + bs1) / (1 + bs1 + bs2)
     *
     * Since the optimal solution is so close to the maximum, we prepare two solutions, and then pick the best one:
     *   - With rounding to nearest
     *   - With rounding to zero
     */
    uint8_t bs1 = (uint8_t)(((7 * bs1_bs2_sum - 1) + 4) / 8);       // Trying rounding to nearest first  // NOLINT
    uint8_t bs2 = (uint8_t)(bs1_bs2_sum - bs1);  // NOLINT
    //assert(bs1_bs2_sum > bs1);

    {
        const uint16_t sample_point_permill = (uint16_t)(1000U * (1U + bs1) / (1U + bs1 + bs2));  // NOLINT

        if (sample_point_permill > MaxSamplePointLocationPermill)   // Strictly more!
        {
            bs1 = (uint8_t)((7 * bs1_bs2_sum - 1) / 8);             // Nope, too far; now rounding to zero
            bs2 = (uint8_t)(bs1_bs2_sum - bs1);
        }
    }

    const bool valid = (bs1 >= 1) && (bs1 <= MaxBS1) && (bs2 >= 1) && (bs2 <= MaxBS2);

    /*
     * Final validation
     * Helpful Python:
     * def sample_point_from_btr(x):
     *     assert 0b0011110010000000111111000000000 & x == 0
     *     ts2,ts1,brp = (x>>20)&7, (x>>16)&15, x&511
     *     return (1+ts1+1)/(1+ts1+1+ts2+1)
     */
    if ((target_bitrate != (peripheral_clock_rate / (prescaler * (1U + bs1 + bs2)))) ||
        !valid)
    {
        // This actually means that the algorithm has a logic error, hence assert(0).
        //assert(0);  // NOLINT
        return -CAN_STM32_ERROR_UNSUPPORTED_BIT_RATE;
    }

    out_timings->baud_rate_prescaler = (uint16_t) prescaler;
    out_timings->resynchronization_jump_width = 1;      // One is recommended by UAVCAN, CANOpen, and DeviceNet
    out_timings->time_segment_1 = bs1;
    out_timings->time_segment_2 = bs2;

    if (DEBUG) {
      Serial.print("target_bitrate=");
      Serial.println(target_bitrate);
      Serial.print("peripheral_clock_rate=");
      Serial.println(peripheral_clock_rate);
  
      Serial.print("timings.baud_rate_prescaler=");
      Serial.println(out_timings->baud_rate_prescaler);
      Serial.print("timings.time_segment_1=");
      Serial.println(out_timings->time_segment_1);
      Serial.print("timings.time_segment_2=");
      Serial.println(out_timings->time_segment_2);
      Serial.print("timings.resynchronization_jump_width=");
      Serial.println(out_timings->resynchronization_jump_width);
    }
    return 0;
}

void canInit(enum BitRate bitrate) {
    RCC->APB1ENR   |= 0x2000000UL;            // Enable CAN clock 
    RCC->APB2ENR   |= 0x1UL;                  // Enable AFIO clock
    AFIO->MAPR     &= 0xFFFF9FFF;             // reset CAN remap
    AFIO->MAPR     |= 0x00000000;             //   set CAN remap, use PA11, PA12

    RCC->APB2ENR   |= 0x4UL;                  // Enable GPIOA clock (bit2 to 1)
    GPIOA->CRH     &= 0xFFF00FFF;
    GPIOA->CRH     |= 0xB8000UL;              // Configure PA11 and PA12
    GPIOA->ODR     |= 0x1800UL;

    CAN1->MCR = 0x51UL;                       // Set CAN to initialization mode

    while ((CAN1->MSR & CAN_MSR_INAK) == 0U);
     

    CAN_bit_timing_config_t timings;
    Serial.print("bitrate=");
    Serial.println(bitrate);
    uint32_t target_bitrate = SPEED[bitrate];
    Serial.print("target_bitrate=");
    Serial.println(target_bitrate);
    int result = ComputeCANTimings(HAL_RCC_GetPCLK1Freq(), target_bitrate, &timings);
    if (result) while(true);  
    CAN1->BTR = (((timings.resynchronization_jump_width - 1U) &    3U) << 24U) |
              (((timings.time_segment_1 - 1U)               &   15U) << 16U) |
              (((timings.time_segment_2 - 1U)               &    7U) << 20U) |
              ((timings.baud_rate_prescaler - 1U)           & 1023U);
  
    //// Set bit rates 
    //CAN1->BTR &= ~(((0x03) << 24) | ((0x07) << 20) | ((0x0F) << 16) | (0x1FF)); 
    //CAN1->BTR |=  (((can_configs[bitrate].TS2-1) & 0x07) << 20) | (((can_configs[bitrate].TS1-1) & 0x0F) << 16) | ((can_configs[bitrate].BRP-1) & 0x1FF);

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
    BitRate canSpeed = bitRateArray[(!digitalRead(CAN_SPEED_PIN2)) << 1 | (!digitalRead(CAN_SPEED_PIN1))];
    canInit(canSpeed); //init can at 500KBPS speed
    
    CAN_msg_14.len = 8; //8 bytes in can message
    CAN_msg_58.len = 8;
    CAN_msg_14.id = Sensor1_4_CAN_ADDRESS | canMask;
    CAN_msg_58.id = Sensor5_8_CAN_ADDRESS | canMask;

    uint32_t realCanSpeed = 0;
    switch(canSpeed) {
        default:
        case CAN_500KBPS:  realCanSpeed = 500;  break;
        case CAN_1000KBPS: realCanSpeed = 1000; break;
        case CAN_125KBPS:  realCanSpeed = 125;  break;
        case CAN_250KBPS:  realCanSpeed = 250;  break;
    }
    Serial.printf("CAN ID 1: 0x%03X CAN ID 2: 0x%03X CanSpeed: %4d\n\r", CAN_msg_14.id, CAN_msg_58.id, realCanSpeed);

    // begin communication to MAX31855 chips
    for(uint32_t i = 0; i < 8; ++i) {
        MAX31855_chips[i].begin();
    }

    // check that all MAX31855 chips work or are even installed
    for(uint32_t i = 0; i < 8; ++i) {
        if (MAX31855_chips[i].readRawData() != 0) {             //we will just get 0 if there is no chip at all.
            if (MAX31855_chips[i].getChipID() != MAX31855_ID)   //if there is something there, check that it responds like MAX31855 (maybe useless step)
            {
                LOG_D("MAX31855 %d Error\n\r", i+1);
            }
            else{
                bitSet(max31855_OK_bits, i);                    //set corresponding bit to 1
            }
        }
        else{
            LOG_D("Can't find MAX31855 %d\n\r", i+1);           //nothing in this CS line
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


void  canSend(CAN_msg_t* CAN_tx_msg, uint8_t mbx = 0) {
    volatile uint32_t count = 0;
     
    CAN1->sTxMailBox[mbx].TIR   = 0; 
    CAN1->sTxMailBox[mbx].TIR   = (CAN_tx_msg->id) << 21;
    
    CAN1->sTxMailBox[mbx].TDTR &= ~(0xF);
    CAN1->sTxMailBox[mbx].TDTR |= CAN_tx_msg->len & 0xFUL;
    
    CAN1->sTxMailBox[mbx].TDLR  = (((uint32_t) CAN_tx_msg->data[3] << 24) |
                                 ((uint32_t) CAN_tx_msg->data[2] << 16) |
                                 ((uint32_t) CAN_tx_msg->data[1] <<  8) |
                                 ((uint32_t) CAN_tx_msg->data[mbx]      ));
    CAN1->sTxMailBox[mbx].TDHR  = (((uint32_t) CAN_tx_msg->data[7] << 24) |
                                 ((uint32_t) CAN_tx_msg->data[6] << 16) |
                                 ((uint32_t) CAN_tx_msg->data[5] <<  8) |
                                 ((uint32_t) CAN_tx_msg->data[4]      ));
    CAN1->sTxMailBox[mbx].TIR  |= 0x1UL;
    while(CAN1->sTxMailBox[mbx].TIR & 0x1UL && count++ < 1000000);
            
    // if (!(CAN1->sTxMailBox[mbx].TIR & 0x1UL)) return;
     
    //Sends error log to screen
    if(CAN1->sTxMailBox[mbx].TIR & 0x1UL) {
       LOG_D("ESR %5lu MSR %5lu TSR %5lu\n\r", CAN1->ESR, CAN1->MSR, CAN1->TSR);
    }
 }

void loop() {
    static uint32_t egtQueryStamp = 0;
    static uint32_t secondStamp = 0;
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
                        egt[i]    = (rawData[i] >> 20);
                        LOG_D("Temp: %6d \n\r", egt[i]);
                        break;
                    case MAX31855_THERMOCOUPLE_SHORT_TO_VCC:
                        LOG_D("SHORT TO VCC\n\r");
                        egt[i]    = 2000;
                        break;
                    case MAX31855_THERMOCOUPLE_SHORT_TO_GND:
                        LOG_D("SHORT TO GND\n\r");
                        egt[i]    = 3000;
                        break;
                    case MAX31855_THERMOCOUPLE_NOT_CONNECTED:
                        //LOG_D("NOT CONNECTED\n\r");
                        egt[i]    = 4000;
                        break;
                    default:
                        LOG_D("MAX31855 ERROR\n\r");
                        egt[i]    = 5000;
                        break;
                }
                //coldJunction[i]   = (MAX31855_chips[i].getColdJunctionTemperature(rawData[i]));
                coldJunction[i]   = ((rawData[i] & 0xFFFF) >> 4)/16;
                if (i < 4) {
                    CAN_msg_14.data[2*i]   = lowByte ((uint16_t)(egt[i]));
                    CAN_msg_14.data[2*i+1] = highByte((uint16_t)(egt[i]));
                } else {
                    CAN_msg_58.data[2*i-8] = lowByte ((uint16_t)(egt[i]));
                    CAN_msg_58.data[2*i-7] = highByte((uint16_t)(egt[i]));
                }
                LOG_D("Cold Junction %5d Temp: %6d \n\r", i+1, coldJunction[i]);
            }
        }
        canSend(&CAN_msg_14, 0);
        canSend(&CAN_msg_58, 1);
    }

    while(Serial3.available () > 0) {       //is there data on serial3, presumably from speeduino
        checkDataRequest();                 //there is data, but is request from speeduino and is it for EGTs
    }
}