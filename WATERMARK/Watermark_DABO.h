/**
 * @file Deakin_DB.h
 * @author Gabriel Ballesteros (gabriel@coratronics.com)
 * @brief This file contains the functions to interact with the custom DB made for Deaking that can
 *        support up to 4 WaterMark sensors and 2 temperature probes (PT1000 or DS18B20)
 * @version 0.1
 * @date 2021-01-12
 * @see  https://www.irrometer.com/200ss.html
 * 
 * @copyright Copyright (c) 2021
 * 
 */


#ifndef __WATERMARK_DB_H
#define __WATERMARK_DB_H

#include "Arduino.h"
#include "../BOARD_SUPPORT/board_support.h"
#include "../CAYENNE_LPP/CayenneLPP.h"
#include "Watermark_CayenneLPP.h"
#include "MCP3221.h"
#include "ds18b20.h"


/**
 * @brief Configuration parameters 
 * 
 */
#define EXCITATION_TIME     20 //in ms
#define CHARGE_CANCEL_TIME  5 //in ms
#define WATERMARK_SAMPLES   10
#define DEFAULT_TEMPERATURE 24
/**
 * @brief Constants
 * 
 */
#define BOTTOM_RES        7870.0f
#define MUX_RES              5
#define PT1000_RES        1000.0f
#define PT1000_BOTTOM_RES 1000.0f
#define PT1000_ALPHA      0.00385f      
#define PT1000_A          0.0039083f
#define PT1000_B         -0.00000057750f

/**
 * \defgroup I2CDeakinAddress I2C Slave Addresses
 * @{
 */

#define ADS_WATERMARK_ADDRESS      0x49    
#define MCP_WATERMARK_ADDRESS      0x4D     
/**@}*/

/**
 * @brief Convenient defintions
 * 
 */
#define POWER_MUX   0
#define CHANNEL_MUX 1
#define SOIL_PROBE   0
#define CANOPY_PROBE 1
#define MAX_TEMPERATURE_PROBES 2
#define MAX_WM 4
#define MAX_TEMP_PROBES 2

/**
 * @brief Temperature Probe Types
 * 
 */
#define PT1000          0x00
#define DS18B20         0x01
#define NO_TEMP_PROBE   0xFF

enum mux1{
    NEG_POL,
    POS_POL,
    READ,
    TEMP
};

enum mux2{
    WM1,
    WM2,
    WM3,
    WM4
};

/**
 * @brief pinout
 * 
 */
#define TEMP2         AN1
#define TEMP1         AN2
#define _5V_REF       AN3

//MUX2 is the mux that applies polarities / selects temperature probes
#define MUX2_EN       GPIO2
#define MUX2_A0       SCK
#define MUX2_A1       MISO

//MUX1 is the mux that selects the Watermark channels
#define MUX1_EN       GPIO3
#define MUX1_A0       MOSI
#define MUX1_A1       CS2

/**
 * @brief Gains
 * 
 */
#define TEMP_GAIN     GAIN_ONE
#define _5V_REF_GAIN  GAIN_ONE

#define _5V_DIV_GAIN    2
#define PT1000_DIV_GAIN 2



class watermarkDB
{
    //public
    public:
        /**
         * @brief WaterMarks attributes. 
         * - Voltage measures.
         * - Impedances
         * - Tensions.
         * .
         * 
         */
        uint16_t waterMarkValues[4] = {0,0,0,0};
        uint16_t waterMarkImp[4];
        int16_t  waterkPa[4];
        int8_t temperatureValues[2];
        /**
         * @brief Select the type of temperature probe.
         * There are two temperatures probes for SOIL and CANOPY.
         * Specifies with which sensor is measured each temperature.
         * 
         */
        uint8_t temperatureProbeType[2];
        int8_t temperatureProbeOffset[2];
        /**
         * @brief Amount of WaterMark sensors used.
         * 
         */
        uint8_t waterMarkSensorsUsed;

        uint16_t referenceVoltage;
        /**
         * @brief Data values of temperature measured
         * for SOIL and CANOPY.
         * 
         */
        float soilTemperature;
        float canopyTemperature;
        /**
         * @brief Construct a new Watermark DaugtherBoard object
         * 
         * @param numWM Specifies the number of waterMarks used.
         */
        watermarkDB(uint8_t numWM)
        {
            waterMarkSensorsUsed = numWM;
        }
        /**
         * @brief **Initialize the Watermark DaughterBoard**
         * - Set pinMode for the Mux pins.
         * - Disables the Mux power.
         * - Sets the Reference Voltage for MCP3221 ADC chip
         * .
         */
        void initialise(void);
        /**
         * @brief this method is used to measure the soil and canopy temperature.
         * It can be used with PT1000 or DS18B20 digital sensors.
         * This is configured previously with the "temperatureProbeType". 
         * 
         * 
         * @return uint8_t NO_ERR if the measurement was done properly. or 1 if there was an error.
         */
        uint8_t measureTemperatures(void);
        /**
         * @brief This method performs the measurement for the all the "waterMarkSensorsUsed". 
         * On each channel the watermarks gives the voltage measurement that is used to 
         * calculate the impedance and then with the soil temperature is used to calculate the 
         * water tension.
         * 
         * @return uint8_t Returns NO_ERR if the measurement finishes properly. And a value
         * different of NO_ERR is returned if an error occurs.
         */
        uint8_t measureWaterTension(void);
        

        //private
        ds18b20 digitalSensor;
        enum_oneWireState measureState;

        /**
         * @brief Enables or disables the desired mux.
         * 
         * @param mux Specifies the desired MUX.
         * @param en Enables or Disable the MUX.
         */
        void muxCtl(uint8_t mux, uint8_t en);
        /**
         * @brief This method is used to manage watermarks channels and digital temperature 
         * sensors.
         * @param mode
         * POS_POL puts positive polarity to the desired watermark sensor.
         * NEG_POL puts negative polarity to the desired watermark sensor.
         * READ reads the data from the desired watermark sensor.
         * TEMP changes the select pin to read the desired digital sensor. 
         * @note For more information use the schematic watermark_db.pdf.
         * The desired watermark sensor is controlled with the method 
         * watermarkDB::chSel(uint8_t ch).
         */
        void powerSel(uint8_t mode);
        /**
         * @brief Selects the multiplexer channels to select differents WaterMarks sensors to
         * measure the soil moisture.
         * 
         * @param ch Specifies the desired channel.
         */
        void chSel(uint8_t ch);
        /**
         * @brief This method is use to calculate the impedance based on a referenceVoltage
         * and a voltage measured with the ADC.
         * 
         * @param voltage Voltage previosly measured to calculate impedance.
         * @return uint16_t Impedance Estimated.
         */
        uint16_t calcImp(uint16_t voltage);
        /**
         * @brief This method is to calculate the waterPressure based on the impedance and the 
         * temperature measured.
         * 
         * @param impedance TBD
         * @param temperature Temperature previously measured with the selected sensor.
         * @return float Water pressure estimated.
         */
        float calcWaterTension(uint16_t voltage, float temperature);
        /**
         * @brief This method converts the voltage data to temperature. 
         * 
         * @param voltage Voltage measured with the ADC in the PT1000 circuit.
         * @return float Current temperature measured.
         */
        float calcTemp(uint16_t voltage);
        /**
         * @brief This method is used to read the voltage applied in the PT1000 sensors (CH1 and CH2)
         * and also is used to verify the correct voltage level in the 5V_PERIPH pin.
         * For more information check the schematic in watermark_db.pdf.
         * 
         * @param channel Channel selection of the ADC1015
         * @param gain Select the desired gain for the internal PGA (Programable Gain Amplifier).
         * @return uint16_t Voltage measured stored in an integer.
         */
        uint16_t readVoltage(sensorInputs channel, adsGain_t gain);
        /**
         * @brief This method is used to select the desired DS18B20 digital temperature sensor.
         * Uses the multiplexer to put the SELECT pin in HIGH or LOW.
         * 
         * @param ch if the channel is 1, SELECT pin is HIGH. SELECT pin is low otherwise.
         */
        void digTempSel(uint8_t ch);
        /**
         * @brief Reads the configuration stored in the EEPROM about the daughter board
         * this is executed after a reset or a DeepSleep resume.
         * 
         */
        void readEEPROM(void);
        /**
         * @brief This method is used to update the Daughter Board current configuration
         * to the EEPROM. This prevents information lost caused of low battery voltage,
         * resets or DeepSleep mode.
         * 
         */
        void writeEEPROM(void);
        /**
         * @brief TBD
         * 
         * @param packet TBD
         * @param packetType TBD 
         * @param daBoVersion TBD 
         */
        void extendPacket(CayenneLPP * packet, uint8_t packetType, uint8_t daBoVersion);
};

#endif //__WATERMARK_DB_H