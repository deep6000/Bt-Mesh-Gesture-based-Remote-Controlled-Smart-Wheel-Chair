//***********************************************************************************
// Include files
//***********************************************************************************
#include "em_i2c.h"
#include "em_gpio.h"
#include "em_emu.h"
#include "native_gecko.h"
#include "gatt_db.h"

#include "src/cmu.h"
#include "src/I2C.h"
#include "bg_types.h"

//***********************************************************************************
// defined files
//***********************************************************************************
#define read_slave  (0x01)
#define write_slave (0x00)
//***********************************************************************************
// global variables
//***********************************************************************************


//***********************************************************************************
// function prototypes
//**********************************************************************************

void I2C_setup(void)
 {

	// Select clocks

	 CMU_ClockEnable(cmuClock_HFPER, true);
	 CMU_ClockEnable(cmuClock_I2C0, true);

	 I2C0->ROUTEPEN  = I2C_ROUTEPEN_SDAPEN | I2C_ROUTEPEN_SCLPEN;
	 I2C0->ROUTELOC0 = ((SCL_locReg << SCL_Route_LOC_shift ) | (SDA_locReg << SDA_Route_LOC_shift)); // ) SCL is at location 16(5:0 in RouteLOC0)

	 I2C_IntClear(I2C0,0x0007FFF);
	 I2C_IntEnable(I2C0,0x0007FFF);



	 const I2C_Init_TypeDef i2cInit =
	 {
	     false,                  /* Enable when initialization done. */
	     true,                  /* Set to master mode. */
	     0,                     /* Use currently configured reference clock. */
	     I2C_FREQ_STANDARD_MAX, /* Set to standard rate assuring being */
	     /*                        within I2C specification. */
	     i2cClockHLRStandard    /* Set to use 4:4 low/high duty cycle. */
	 };


	 I2C_Init(I2C0, &i2cInit);
	 I2C_Enable(I2C0,true);
	 if(I2C0->STATE & I2C_STATE_BUSY)
	 {
	   I2C0->CMD=I2C_CMD_ABORT;
	 }
	 int j=0;
	 GPIO_PinModeSet(gpioPortC, 10, gpioModeWiredAnd, 1);   // SCL
	 GPIO_PinModeSet(gpioPortC, 11, gpioModeWiredAnd, 1);   // SDA


	 	 // toggle SCL pin 9 times to reset any I2C slave that may require it
	 for ( j = 0; j < 9; j++)
	 {
		 GPIO_PinModeSet(gpioPortC, 10, gpioModeWiredAnd, 0);
	 	 GPIO_PinModeSet(gpioPortC, 10, gpioModeWiredAnd, 1);
	 }
 }


void I2C_Start()
{
	I2C0->CMD = I2C_CMD_START;
}

void I2C_Stop()
{
	I2C0->CMD = I2C_CMD_STOP;
}
void I2C_Send_NACK()
{
	 I2C0->CMD=I2C_CMD_NACK;
}

void I2C_Send_ACK(void)
{
	I2C0->CMD = I2C_CMD_ACK;
}

void I2C_Write_Byte(unsigned char data)
{
	I2C0->TXDATA = data;

	while((I2C0->IF & I2C_IF_ACK)== 0);
	I2C_IntClear(I2C0,I2C_IFC_ACK);  // clear the ack flag
}

unsigned char I2C_Read_Byte(void)
{
		uint8_t data;
		while(( I2C0->STATUS & I2C_STATUS_RXDATAV) == 0);
		data = I2C0->RXDATA;
		return data;
}

unsigned char I2C_Read2Bytes(void)
{
		uint16_t data;
		while(( I2C0->STATUS & I2C_STATUS_RXDATAV) == 0);
		data = I2C0->RXDATA;
		data <<= 8;
		I2C0->CMD = I2C_CMD_ACK;
		while(( I2C0->STATUS & I2C_STATUS_RXDATAV) == 0);
		data |= I2C0->RXDATA;

		return data;
}
