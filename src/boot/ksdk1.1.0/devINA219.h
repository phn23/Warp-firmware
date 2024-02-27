void		initINA219(const uint8_t i2cAddress, uint16_t operatingVoltageMillivolts);
WarpStatus	readSensorRegisterINA219(uint8_t deviceRegister, int numberOfBytes);
WarpStatus	writeSensorRegisterINA219(uint8_t deviceRegister, uint8_t payloadBtye[2]); // TODO: CONFIRM WHETHER THIS IS UNIT16_t or not
// WarpStatus 	configureSensorINA219(uint8_t payloadF_SETUP, uint8_t payloadCTRL_REG1);
void		printSensorDataINA219(bool hexModeFlag);
uint8_t		appendSensorDataINA219(uint8_t* buf);


// NOT SURE ABOUT THESE THREE
const uint8_t bytesPerMeasurementINA219            = 6;
const uint8_t bytesPerReadingINA219                = 2;
const uint8_t numberOfReadingsPerMeasurementINA219 = 3;


/***********************************************************************
// from ADAFRUIT library
***********************************************************************/

/** calculated I2C address: 0 = GND, 1 = V+ **/
/* The address is controlled by the A0 and A1 inputs on the INA219:
 *
 * Calculated address: b100ABCD
 * A0 controls C and D: GND = 00, V+ = 01, SDA = 10, SCL = 11
 * A1 controls A and B: GND = 00, V+ = 01, SDA = 10, SCL = 11
 *
 * E.g. if A0 is tied to ground and A1 is tied to V+,
 * the resulting address is b1000100 = 0x44
 *
 * SDA and SCL options aren't implemented.
 */


// proly no need 
// #define INA219_CALC_ADDRESS(INA_ADDR0, INA_ADDR1)                              \
//   (0x40 | (INA_ADDR0 != 0 ? 0x01 : 0x00) | (INA_ADDR1 != 0 ? 0x04 : 0x00))

// /** default I2C address **/
// #define INA219_ADDRESS (0x40) // 1000000 (A0+A1=GND)

// /** read **/
// #define INA219_READ (0x01)

/*=========================================================================
    CONFIG REGISTER (R/W)
**************************************************************************/


/** config register address **/
#define INA219_REG_CONFIG (0x00)

/** reset bit **/
#define INA219_CONFIG_RESET (0x8000) // Reset Bit

/** mask for bus voltage range **/
#define INA219_CONFIG_BVOLTAGERANGE_MASK (0x2000) // Bus Voltage Range Mask

/** bus voltage range values **/
enum {
  INA219_CONFIG_BVOLTAGERANGE_16V = (0x0000), // 0-16V Range
  INA219_CONFIG_BVOLTAGERANGE_32V = (0x2000), // 0-32V Range
};

/** mask for gain bits **/
#define INA219_CONFIG_GAIN_MASK (0x1800) // Gain Mask

/** values for gain bits **/
enum {
  INA219_CONFIG_GAIN_1_40MV = (0x0000),  // Gain 1, 40mV Range
  INA219_CONFIG_GAIN_2_80MV = (0x0800),  // Gain 2, 80mV Range
  INA219_CONFIG_GAIN_4_160MV = (0x1000), // Gain 4, 160mV Range
  INA219_CONFIG_GAIN_8_320MV = (0x1800), // Gain 8, 320mV Range
};

/** mask for bus ADC resolution bits **/
#define INA219_CONFIG_BADCRES_MASK (0x0780)

/** values for bus ADC resolution **/
enum {
  INA219_CONFIG_BADCRES_9BIT =                          (0x0000),  // 9-bit bus res = 0..511
  INA219_CONFIG_BADCRES_10BIT =                         (0x0080), // 10-bit bus res = 0..1023
  INA219_CONFIG_BADCRES_11BIT =                         (0x0100), // 11-bit bus res = 0..2047
  INA219_CONFIG_BADCRES_12BIT =                         (0x0180), // 12-bit bus res = 0..4097
  INA219_CONFIG_BADCRES_12BIT_2S_1060US =               (0x0480), // 2 x 12-bit bus samples averaged together
  INA219_CONFIG_BADCRES_12BIT_4S_2130US =               (0x0500), // 4 x 12-bit bus samples averaged together
  INA219_CONFIG_BADCRES_12BIT_8S_4260US =               (0x0580), // 8 x 12-bit bus samples averaged together
  INA219_CONFIG_BADCRES_12BIT_16S_8510US =              (0x0600), // 16 x 12-bit bus samples averaged together
  INA219_CONFIG_BADCRES_12BIT_32S_17MS =                (0x0680), // 32 x 12-bit bus samples averaged together
  INA219_CONFIG_BADCRES_12BIT_64S_34MS =                (0x0700), // 64 x 12-bit bus samples averaged together
  INA219_CONFIG_BADCRES_12BIT_128S_69MS =               (0x0780), // 128 x 12-bit bus samples averaged together
};

/** mask for shunt ADC resolution bits **/
#define INA219_CONFIG_SADCRES_MASK  (0x0078) // Shunt ADC Resolution and Averaging Mask

/** values for shunt ADC resolution **/
enum {
  INA219_CONFIG_SADCRES_9BIT_1S_84US =              (0x0000),   // 1 x 9-bit shunt sample
  INA219_CONFIG_SADCRES_10BIT_1S_148US =            (0x0008), // 1 x 10-bit shunt sample
  INA219_CONFIG_SADCRES_11BIT_1S_276US =            (0x0010), // 1 x 11-bit shunt sample
  INA219_CONFIG_SADCRES_12BIT_1S_532US =            (0x0018), // 1 x 12-bit shunt sample
  INA219_CONFIG_SADCRES_12BIT_2S_1060US =           (0x0048), // 2 x 12-bit shunt samples averaged together
  INA219_CONFIG_SADCRES_12BIT_4S_2130US =           (0x0050), // 4 x 12-bit shunt samples averaged together
  INA219_CONFIG_SADCRES_12BIT_8S_4260US =           (0x0058), // 8 x 12-bit shunt samples averaged together
  INA219_CONFIG_SADCRES_12BIT_16S_8510US =          (0x0060), // 16 x 12-bit shunt samples averaged together
  INA219_CONFIG_SADCRES_12BIT_32S_17MS =            (0x0068), // 32 x 12-bit shunt samples averaged together
  INA219_CONFIG_SADCRES_12BIT_64S_34MS =            (0x0070), // 64 x 12-bit shunt samples averaged together
  INA219_CONFIG_SADCRES_12BIT_128S_69MS =           (0x0078), // 128 x 12-bit shunt samples averaged together
};

/** mask for operating mode bits **/
#define INA219_CONFIG_MODE_MASK (0x0007) // Operating Mode Mask

/** values for operating mode **/
enum {
  INA219_CONFIG_MODE_POWERDOWN =                   0x00,       /**< power down */
  INA219_CONFIG_MODE_SVOLT_TRIGGERED =             0x01, /**< shunt voltage triggered */
  INA219_CONFIG_MODE_BVOLT_TRIGGERED =             0x02, /**< bus voltage triggered */
  INA219_CONFIG_MODE_SANDBVOLT_TRIGGERED =         0x03,                         /**< shunt and bus voltage triggered */
  INA219_CONFIG_MODE_ADCOFF =                      0x04, /**< ADC off */
  INA219_CONFIG_MODE_SVOLT_CONTINUOUS =            0x05, /**< shunt voltage continuous */
  INA219_CONFIG_MODE_BVOLT_CONTINUOUS =            0x06, /**< bus voltage continuous */
  INA219_CONFIG_MODE_SANDBVOLT_CONTINUOUS =        0x07, /**< shunt and bus voltage continuous */
};

/** shunt voltage register **/
#define INA219_REG_SHUNTVOLTAGE (0x01)

/** bus voltage register **/
#define INA219_REG_BUSVOLTAGE (0x02)

/** power register **/
#define INA219_REG_POWER (0x03)

/** current register **/
#define INA219_REG_CURRENT (0x04)

/** calibration register **/
#define INA219_REG_CALIBRATION (0x05)



// TODO: TO ADD
//   Adafruit_INA219(uint8_t addr = INA219_ADDRESS);
//   ~Adafruit_INA219();
//   bool begin(TwoWire *theWire = &Wire);
//   void setCalibration_32V_2A();
//   void setCalibration_32V_1A();
//   void setCalibration_16V_400mA();
//   float getBusVoltage_V();
//   float getShuntVoltage_mV();
//   float getCurrent_mA();
//   float getPower_mW();
//   void powerSave(bool on);
//   bool success();


//   Adafruit_I2CDevice *i2c_dev = NULL;

//   bool _success;

//   uint8_t ina219_i2caddr = -1;
//   uint32_t ina219_calValue;
//   // The following multipliers are used to convert raw current and power
//   // values to mA and mW, taking into account the current config settings
//   uint32_t ina219_currentDivider_mA;
//   float ina219_powerMultiplier_mW;


//   int16_t getBusVoltage_raw();
//   int16_t getShuntVoltage_raw();
//   int16_t getCurrent_raw();
//   int16_t getPower_raw();
