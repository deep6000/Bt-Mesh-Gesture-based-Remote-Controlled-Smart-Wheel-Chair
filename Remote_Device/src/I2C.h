//***********************************************************************************
// Include files
//***********************************************************************************


//***********************************************************************************
// defined files
//***********************************************************************************
#define SCL_locReg	 			14   //  SCL location register
#define SDA_locReg 				16   // SDA location Register
#define SCL_Route_LOC_shift 	8 	 // (13:8 in RouteLOC0)
#define SDA_Route_LOC_shift 	0    //



//***********************************************************************************
// global variables
//***********************************************************************************

//***********************************************************************************
// function prototypes
//***********************************************************************************
void I2C_setup(void);
void I2C_Start();
void I2C_Stop();
void I2C_Send_NACK();
void I2C_Send_ACK(void);
void I2C_Write_Byte(unsigned char data);
unsigned char I2C_Read_Byte(void);
unsigned char I2C_Read2Bytes(void);

