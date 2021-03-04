#include "Watermark_DABO.h"




// watermarkDB DB1;
MCP3221 mcp3221(MCP_WATERMARK_ADDRESS);
Adafruit_ADS1015  ads_deakin(ADS_WATERMARK_ADDRESS); 

    // void readTemperature(void);
    // void measureWaterTension(void);
    // void initialise(void);



/**
 * @brief **Initialize the Watermark DaughterBoard**
 * - Set pinMode for the Mux pins.
 * - Disables the Mux power.
 * - Sets the Reference Voltage for MCP3221 ADC chip
 * .
 */
void watermarkDB::initialise(void)
{
    //make sure the pins are configured properly
    mcp.pinMode(MUX2_EN, OUTPUT);
    pinMode(MUX2_A0, OUTPUT);
    pinMode(MUX2_A1, OUTPUT);

    mcp.pinMode(MUX1_EN, OUTPUT);
    pinMode(MUX1_A0, OUTPUT);
    pinMode(MUX1_A1, OUTPUT);

    //Serial.printlnc(ANSI_RED, "DB INITIALISED");

    //initialise pins : disable mux
    muxCtl(POWER_MUX, LOW);
    muxCtl(CHANNEL_MUX, LOW);

    pmc.busPower(_3V3, ON);

    //initialise ADS first so we can read 5V to calibrate the MCP
    referenceVoltage = readVoltage(_5V_REF, _5V_REF_GAIN);
    referenceVoltage *=2;

    //set reference voltage for MCP3221 ADC chip
    mcp3221.setVref(referenceVoltage);                      // sets voltage reference for the ADC in mV (change as needed)
    mcp3221.setVinput(VOLTAGE_INPUT_5V);              // sets voltage input type to be measured (change as needed)
    // mcp3221.setRes1(10000);                           // sets exact value of the voltage divider's Resistor 1 for 12V readings (change as needed)
    // mcp3221.setRes2(4700);                            // sets exact value of the voltage divider's Resistor 2 for 12V readings (change as needed)
    // mcp3221.setAlpha(178);                            // sets the Alpha value used by the EMAVG smoothing method (change as needed)
}
/**
 * @brief This method performs the measurement for the all the "waterMarkSensorsUsed". 
 * On each channel the watermarks gives the voltage measurement that is used to 
 * calculate the impedance and then with the soil temperature is used to calculate the 
 * water tension.
 * 
 * @return uint8_t Returns NO_ERR if the measurement finishes properly. And a value
 * different of NO_ERR is returned if an error occurs.
 */
uint8_t watermarkDB::measureWaterTension(void)
{
  uint8_t error = NO_ERR; 

  //read reference voltage to have proper following measurements of temperature and WM
  referenceVoltage = readVoltage(_5V_REF, _5V_REF_GAIN);
  referenceVoltage *=2;

  //measure temperature, and check if it's invalid
  error = measureTemperatures();


  for(int i=0; i<waterMarkSensorsUsed; i++)
  {
    //enable power mux and disable WM mux before starting
    muxCtl(POWER_MUX, HIGH);
    muxCtl(CHANNEL_MUX, LOW);

    //select the right WM and enable the mux
    chSel(i);
    muxCtl(CHANNEL_MUX, HIGH);

    //apply negative polarity
    powerSel(NEG_POL);

    //delay a bit
    vTaskDelay(EXCITATION_TIME);

    //apply positive polarity
    powerSel(POS_POL);
    
    //delay a bit
    vTaskDelay(EXCITATION_TIME);

    //stop applying voltage, switch to read
    powerSel(READ);

    //read the voltage within 100uS
    waterMarkValues[i] = mcp3221.getVoltage();

    //flip polarity one last time to cancel charge
    powerSel(NEG_POL);

    //delay a bit
    vTaskDelay(CHARGE_CANCEL_TIME);

    //disconnect 
    powerSel(TEMP);

    //calculate kPa from voltage
    waterMarkImp[i] = calcImp(waterMarkValues[i]);
    waterkPa[i] =  calcWaterTension(waterMarkImp[i], soilTemperature);

    //stop applying voltage, switch to read
    muxCtl(POWER_MUX, LOW);

    Serial.printf("WM %d TENSION        : %d kPa\r\n",i, waterkPa[i]);
  }

  //read temperature once and use for all sensors
  pmc.busPower(_5V, OFF);


  //disable muxes
  muxCtl(POWER_MUX, LOW);
  muxCtl(CHANNEL_MUX, LOW);

  return error;
}
/**
 * @brief this method is used to measure the soil and canopy temperature.
 * It can be used with PT1000 or DS18B20 digital sensors.
 * This is configured previously with the "temperatureProbeType". 
 * 
 * 
 * @return uint8_t NO_ERR if the measurement was done properly. or 1 if there was an error.
 */
uint8_t watermarkDB::measureTemperatures(void)
{
  uint8_t error = NO_ERR;

  for(int i=0; i<MAX_TEMPERATURE_PROBES; i++)
  {
      switch(temperatureProbeType[i])
      {
          case PT1000:
          {
              //MUX not needed for PT1000
              muxCtl(CHANNEL_MUX, LOW);
              muxCtl(POWER_MUX, LOW);

              //TODO: Unify under single array to compact code?
              //for the soil probe
              if(i == SOIL_PROBE) 
              {
                uint16_t voltage = readVoltage(TEMP1, TEMP_GAIN);
                //Serial.printfc(ANSI_GREEN,"Soil Voltage[%d]: %f\r\n",i,voltage);

                //catch a voltage of 0 -> it means the probe is disconnected, register error, do not allow next operation
                if(voltage == 0)
                {
                  error = 1;
                  soilTemperature = DEFAULT_TEMPERATURE;
                }
                else
                {
                  soilTemperature = calcTemp(voltage);
                }

                //apply offset
                soilTemperature += temperatureProbeOffset[0];

                //load to int variable
                temperatureValues[0] = soilTemperature;

                //Serial.printfc(ANSI_GREEN,"Soil Temperature[%d]: %f\r\n",i,soilTemperature);
              }
              //for the canopy
              else 
              {
                float voltage = readVoltage(TEMP2, TEMP_GAIN);
                //Serial.printfc(ANSI_GREEN,"Canopy Voltage[%d]: %f\r\n",i,voltage);
              
                //catch a voltage of 0 -> it means the probe is disconnected, register error, do not allow next operation
                if(voltage == 0)
                {
                  error = 1;
                  canopyTemperature = DEFAULT_TEMPERATURE;
                }
                else
                {
                  canopyTemperature = calcTemp(voltage);
                }

                //apply offset
                soilTemperature += temperatureProbeOffset[1];

                //load to int variable
                temperatureValues[1] = canopyTemperature;

                //Serial.printfc(ANSI_GREEN,"Canopy Temperature[%d]: %f\r\n",i,canopyTemperature);
              }
          
          }
              break;
          case DS18B20:
              // select correct channel
              // "i" is used to SOIL_PROBE or CANOPY_PROBE
              digTempSel(i);
              // Wait to switching time to select the desired channel.
              vTaskDelay(EXCITATION_TIME);

              // Read temperature
              /**
               * @brief *******THIS MUST BE REPLACED WITH THE DATA SENSOR PIN*******
               * 
               */
              digitalSensor.begin( 26 ); 
              do
              {
                measureState = digitalSensor.takeMeasure((i == SOIL_PROBE) ? (&soilTemperature), (&canopyTemperature) );
                vTaskDelay(5);
                
                if(measureState == functionError )
                {
                  error = 1;
                  if(i == SOIL_PROBE)
                    soilTemperature = DEFAULT_TEMPERATURE;
                  else
                    canopyTemperature = DEFAULT_TEMPERATURE;
                }

              }while ( (measureState != functionFinishes) || (measureState != functionError) )
              
              break;
          default:
              //Serial.printlnc(ANSI_RED, "UKNOWN TEMPERATURE PROBE TYPE");
              break;
          
      }
  }

  return error;
}
/**
 * @brief Enables or disables the desired mux.
 * 
 * @param mux Specifies the desired MUX.
 * @param en Enables or Disable the MUX.
 */
void watermarkDB::muxCtl(uint8_t mux, uint8_t en)
{
    mcp.digitalWrite(mux == POWER_MUX ? MUX1_EN : MUX2_EN, en);
}
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
void watermarkDB::powerSel(uint8_t mode)
{
    switch(mode)
    {
        case POS_POL:
            digitalWrite(MUX1_A0, LOW);
            digitalWrite(MUX1_A1, LOW);
            break;
        case NEG_POL:
            digitalWrite(MUX1_A0, HIGH);
            digitalWrite(MUX1_A1, LOW);
            break;
        case READ:
            digitalWrite(MUX1_A0, LOW);
            digitalWrite(MUX1_A1, HIGH);
            break;
        case TEMP:
            digitalWrite(MUX1_A0, HIGH);
            digitalWrite(MUX1_A1, HIGH);
            break;
    }
}
/**
 * @brief Selects the multiplexer channels to select differents WaterMarks sensors to
 * measure the soil moisture.
 * 
 * @param ch Specifies the desired channel.
 */
void watermarkDB::chSel(uint8_t ch)
{
    switch(ch)
    {
        case WM1:
            digitalWrite(MUX2_A0, LOW);
            digitalWrite(MUX2_A1, HIGH);
            break;
        case WM2:
            digitalWrite(MUX2_A0, HIGH);
            digitalWrite(MUX2_A1, HIGH);
            break;
        case WM3:
            digitalWrite(MUX2_A0, LOW);
            digitalWrite(MUX2_A1, LOW);
            break;
        case WM4:
            digitalWrite(MUX2_A0, HIGH);
            digitalWrite(MUX2_A1, LOW);
            break;
    }
}
/**
 * @brief This method is use to calculate the impedance based on a referenceVoltage
 * and a voltage measured with the ADC.
 * 
 * @param voltage Voltage previosly measured to calculate impedance.
 * @return uint16_t Impedance Estimated.
 */
uint16_t watermarkDB::calcImp(uint16_t voltage)
{
  float imp = 0;

  Serial.printfc(ANSI_MAGENTA,"Reference voltage: %d\r\n", referenceVoltage);
  Serial.printfc(ANSI_CYAN,"Probe voltage: %d\r\n", voltage);

  if(voltage != 0)
  {
    imp = (float)referenceVoltage/(float)voltage;
    //Serial.printfc(ANSI_RED,"Voltage ration: %f\r\n",imp);
    imp -= 1;
    //Serial.printfc(ANSI_RED,"Voltage ratio - 1 %f\r\n",imp);
    imp *= BOTTOM_RES;
    //Serial.printfc(ANSI_RED,"voltage ratio - 1 , times RES %f\r\n",imp);
    imp -= MUX_RES;    
    //Serial.printfc(ANSI_RED,"voltage ratio - 1 , times RES, minus mux imp %f\r\n",imp);
  }

  //trim impedance
  if(imp < 0) imp = 0;
  else if(imp > 0xFFFF) imp = 0xFFFF;
  

  Serial.printfc(ANSI_RED,"%d\r\n",(uint16_t)imp);
  return (uint16_t)imp;
      
      // return (BOTTOM_RES * (((float)referenceVoltage/(float)voltage) - 1)) - MUX_RES;
}
/**
 * @brief This method is to calculate the waterPressure based on the impedance and the 
 * temperature measured.
 * 
 * @param impedance TBD
 * @param temperature Temperature previously measured with the selected sensor.
 * @return float Water pressure estimated.
 */
float watermarkDB::calcWaterTension(uint16_t impedance, float temperature)
{
    float waterPressure = 0;

    //Serial.printfc(ANSI_YELLOW, "%f\r\n", impedance);

    //divide in ranges as specified by irrometer and return the calculated water centibars
    if(impedance < 550) 
    {
      //Serial.printlnc(ANSI_GREEN, "Water Pressure Range 0");
      waterPressure = 0;
    }
    else if(impedance < 1000) 
    {
      //Serial.printlnc(ANSI_GREEN, "Water Pressure Range 1");
      waterPressure = 0.018 *(temperature - 24);
      //Serial.printfc(ANSI_MAGENTA,"%3.1f\r\n", waterPressure);
      waterPressure += 1;
      //Serial.printfc(ANSI_MAGENTA,"%3.1f\r\n", waterPressure);
      waterPressure *= (impedance/1000.0);
      //Serial.printfc(ANSI_MAGENTA,"%3.1f\r\n", waterPressure);
      waterPressure -= 0.55;
      //Serial.printfc(ANSI_MAGENTA,"%3.1f\r\n", waterPressure);
      waterPressure *= -20.0;
      //Serial.printfc(ANSI_MAGENTA,"%3.1f\r\n", waterPressure);

      // waterPressure = -20.00*((impedance/1000.00)*(1.00+0.018*(temperature-24.00))-0.55);
    }
    else if(impedance < 8000)
    {
      //Serial.printlnc(ANSI_GREEN, "Water Pressure Range 2");
      waterPressure = (-3.213*(impedance/1000.00)-4.093)/(1-0.009733*(impedance/1000.00)-0.01205*(temperature));
    }
    else 
    {
      //Serial.printlnc(ANSI_GREEN, "Water Pressure Range 3");
      waterPressure = -2.246-5.239*(impedance/1000.00)*(1+.018*(temperature-24.00))-.06756*(impedance/1000.00)*(impedance/1000.00)*((1.00+0.018*(temperature-24.00))*(1.00+0.018*(temperature-24.00)));
    }

    //Other formula
    // waterPressure = (-3.213*impedance - 4.093) / (1 - 0.009733*impedance - 0.01205*temperature);

    return waterPressure;
}
/**
 * @brief This method converts the voltage data to temperature. 
 * 
 * @param voltage Voltage measured with the ADC in the PT1000 circuit.
 * @return float Current temperature measured.
 */
float watermarkDB::calcTemp(uint16_t voltage)
{
    float PTresistance = PT1000_BOTTOM_RES * ((referenceVoltage / (float)voltage) - 1);
    float temperature = 0;

    temperature = 1 - (PTresistance / PT1000_RES);
    //Serial.printf("%f\r\n", temperature);
    temperature = 4*PT1000_B*temperature;
    // Serial.printf("%f\r\n", temperature);
    temperature = (PT1000_A*PT1000_A) - temperature;
    // Serial.printf("%f\r\n", temperature);
    temperature = sqrt(temperature);
    // Serial.printf("%f\r\n", temperature);
    temperature = -PT1000_A + temperature;
    // Serial.printf("%f\r\n", temperature);
    temperature = temperature / (2*PT1000_B);

    return temperature;
}
/**
 * @brief This method is used to read the voltage applied in the PT1000 sensors (CH1 and CH2)
 * and also is used to verify the correct voltage level in the 5V_PERIPH pin.
 * For more information check the schematic in watermark_db.pdf.
 * 
 * @param channel Channel selection of the ADC1015
 * @param gain Select the desired gain for the internal PGA (Programable Gain Amplifier).
 * @return uint16_t Voltage measured stored in an integer.
 */
uint16_t watermarkDB::readVoltage(sensorInputs channel, adsGain_t gain)
{
  float voltage = 0;

  //up to 4.095V, no divider
  ads_deakin.setGain(gain);

  for(int i=0; i < WATERMARK_SAMPLES; i++)
  {
    switch(channel)
    {
        case AN1:
          voltage += ads_deakin.readADC_SingleEnded(0);
          break;
        case AN2:
          voltage += ads_deakin.readADC_SingleEnded(1);
          break;
        case AN3:
          voltage += ads_deakin.readADC_SingleEnded(2);
          break;
        case AN4:
          voltage += ads_deakin.readADC_SingleEnded(3);
          break;
        default:
          Serial.printlnc(ANSI_RED, "Invalid Voltage reading, channel not defined.");
          break;
    }
  }

  //apply ADC bit to mV conversion
  switch(gain)
  {
    case GAIN_TWOTHIRDS:
      voltage *= 3;
      break;
    case GAIN_ONE:
      voltage *= 2;
      break;
    case GAIN_TWO:
      break;
    case GAIN_FOUR:
      voltage *= 0.5;
      break;
    case GAIN_EIGHT:
      voltage *= 0.25;
      break;
    case GAIN_SIXTEEN:
      voltage *= 0.125;
    default:
      Serial.printlnc(ANSI_RED, "UKNOWN ADC GAIN!");
      break;
  }

  //average
  voltage /= WATERMARK_SAMPLES;

  //apply resistor divider gain
  // voltage /= ANALOG_INPUT_VOLTAGE_GAIN;

  return((uint16_t)voltage);
}
/**
 * @brief This method is used to select the desired DS18B20 digital temperature sensor.
 * Uses the multiplexer to put the SELECT pin in HIGH or LOW.
 * 
 * @param ch if the channel is 1, SELECT pin is HIGH. SELECT pin is low otherwise.
 */
void watermarkDB::digTempSel(uint8_t ch)
{
  if(!ch)
  {
    powerSel(TEMP);
    muxCtl(POWER_MUX, HIGH);
  }

  //just disable the mux to select the other channel
  else
  {
    muxCtl(POWER_MUX, LOW);
  }
}
/**
 * @brief Reads the configuration stored in the EEPROM about the daughter board
 * this is executed after a reset or a DeepSleep resume.
 * 
 */
void watermarkDB::readEEPROM(void)
{
    //Serial.printlnc(ANSI_YELLOW, "Reading Watermark DB Configuration from EEPROM");

    //load Core parameters from EEPROM
    getCheckByte(WATERMARK_DABO_NUM_WM, waterMarkSensorsUsed);
    getCheckByte(WATERMARK_DABO_TEMP1_TYPE, temperatureProbeType[0]);
    getCheckByte(WATERMARK_DABO_TEMP2_TYPE, temperatureProbeType[1]);
    getCheckByte(WATERMARK_DABO_TEMP1_OFFSET, (uint8_t &)temperatureProbeOffset[0]);
    getCheckByte(WATERMARK_DABO_TEMP2_OFFSET, (uint8_t &)temperatureProbeOffset[1]);
 
    //Convert temperature offsets from byte to signed
    uint8_t offset = 0;

    for(int i=0; i<MAX_TEMPERATURE_PROBES; i++)
    {
      //if it's 0xFF, just set as 0
      if(temperatureProbeOffset[i] == 0xFF) offset = 0;
      else offset = temperatureProbeOffset[i];

      //convert and load to settings variable
      uint8_t sign = (offset >> 7) & 0x01;
      offset &= ~(1<<7);

      //reload into variables after coversion
      temperatureProbeOffset[i] = offset * (sign?-1:1);
    }    
}
/**
 * @brief This method is used to update the Daughter Board current configuration
 * to the EEPROM. This prevents information lost caused of low battery voltage,
 * resets or DeepSleep mode.
 * 
 */
void watermarkDB::writeEEPROM(void)
{
  writeTripleByte(WATERMARK_DABO_NUM_WM, waterMarkSensorsUsed);
  writeTripleByte(WATERMARK_DABO_TEMP1_TYPE, temperatureProbeType[0]);
  writeTripleByte(WATERMARK_DABO_TEMP2_TYPE, temperatureProbeType[1]);

  // for(int i=0; i<MAX_TEMPERATURE_PROBES; i++)
  // {
  //   //identify if offset is positive or negative
  //   uint8_t sign = (temperatureProbeOffset[i] > 0)?0:1;

  //   //get absolute value of offset, we only care about integers, 1 degree accuracy is enough
  //   uint8_t offset = (uint8_t)(temperatureProbeOffset[i] * (sign?-1:1));

  //   //store offset in lower 7 bits and sign in top bit
  //   offset &= ~(1<<7);
  //   offset |= (sign << 7); 

  //   Serial.printf("The temperature offset is %d%d\r\n", sign, offset);
  // }


  writeTripleByte(WATERMARK_DABO_TEMP1_OFFSET, temperatureProbeOffset[0]);
  writeTripleByte(WATERMARK_DABO_TEMP2_OFFSET, temperatureProbeOffset[1]);
}
/**
 * @brief TBD
 * 
 * @param packet TBD
 * @param packetType TBD 
 * @param daBoVersion TBD 
 */
void watermarkDB::extendPacket(CayenneLPP * packet, uint8_t packetType, uint8_t daBoVersion)
{
  //add daughterboard info - only during sync!
  //Sync
  if(packetType == SYNC_CORE_UPDATE)
  {
    //watermark sensors used?
    packet->addGenericByte(LPP_CODE_NUM_WM, waterMarkSensorsUsed);

    //types of temperature sensors
    packet->addGenericByte(LPP_CODE_TEMP1_TYPE, temperatureProbeType[0]);
    packet->addGenericByte(LPP_CODE_TEMP2_TYPE, temperatureProbeType[1]);
  }

  //normal update
  else if((packetType == DAILY_UPDATE) || (packetType == NORMAL_UPDATE))
  {
    //add the dabo version for proper parsing on cloud 
    packet->addGenericByte(LPP_CODE_DB2, daBoVersion);

    //watermark raw readings
    for(int i=0; i<waterMarkSensorsUsed; i++) packet->addGenericWord(LPP_CODE_WM1_RAW+i, waterMarkImp[i]);

    //temp values
    for(int i=0; i<MAX_TEMP_PROBES; i++)
    {
      if(temperatureProbeType[i] != NO_TEMP_PROBE) packet->addGenericByte(LPP_CODE_TEMP1_RAW+i, temperatureValues[i]);
    }
  }

  //some debugging
  // for(int i=0; i<51; i++)
  // {
  //   Serial.printfc(ANSI_RED,"%d\r\n", packet->getBuffer()[i]);

  // }

  // Serial.printfc(ANSI_RED,"Size: %d\r\n", packet->getSize());

}



