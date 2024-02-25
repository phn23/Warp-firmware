void		initINA219(const uint8_t i2cAddress, uint16_t operatingVoltageMillivolts);
WarpStatus	readSensorRegisterINA219(uint8_t deviceRegister, int numberOfBytes);
WarpStatus	writeSensorRegisterINA219(uint8_t deviceRegister, uint8_t payloadBtye);
WarpStatus 	configureSensorINA219(uint8_t payloadF_SETUP, uint8_t payloadCTRL_REG1);
void		printSensorDataINA219(bool hexModeFlag);
uint8_t		appendSensorDataINA219(uint8_t* buf);


// NOT SURE ABOUT THESE THREE
const uint8_t bytesPerMeasurementINA219            = 6;
const uint8_t bytesPerReadingINA219                = 2;
const uint8_t numberOfReadingsPerMeasurementINA219 = 3;
