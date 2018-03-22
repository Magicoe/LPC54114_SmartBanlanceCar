#include "board.h"
#include <stdio.h>
#include <string.h>

#include "fsl_debug_console.h"
#include "fsl_spi.h"

#include "fsl_common.h"
#include "fsl_iocon.h"
#include "pin_mux.h"

#include "fsl_spi_dma.h"
#include "fsl_dma.h"

#include "mpu9250_reg.h"
#include "driver_spi_mpu9250.h"

#define MPU9250_CS_PORT         0U
#define MPU9250_CS_PIN          14U
#define MPU9250_CS_SET()        GPIO_WritePinOutput(GPIO, MPU9250_CS_PORT, MPU9250_CS_PIN, 1)
#define MPU9250_CS_CLR()        GPIO_WritePinOutput(GPIO, MPU9250_CS_PORT, MPU9250_CS_PIN, 0)

#define MPU9250_INT_PORT        0U
#define MPU9250_INT_PIN         15U
#define MPU9250_INT_GET()       GPIO_ReadPinInput(GPIO, MPU9250_CS_PORT, MPU9250_CS_PIN)

#define MPU9250_SPI             SPI3
#define MPU9250_SPI_IRQ         FLEXCOMM3_IRQn
#define MPU9250_SPI_CLKFREQ     CLOCK_GetFreq(kCLOCK_Flexcomm3)

#define MPU9250_DMATX_CHANNEL   7
#define MPU9250_DMARX_CHANNEL   6

#define IOCON_PIO_DIGITAL_EN          0x80u   /*!< Enables digital function */
#define IOCON_PIO_FUNC0               0x00u   /*!< Selects pin function 1 */
#define IOCON_PIO_FUNC1               0x01u   /*!< Selects pin function 1 */
#define IOCON_PIO_FUNC2               0x02u   /*!< Selects pin function 2 */
#define IOCON_PIO_FUNC4               0x04u   /*!< Selects pin function 4 */
#define IOCON_PIO_INPFILT_OFF       0x0100u   /*!< Input filter disabled */
#define IOCON_PIO_INV_DI              0x00u   /*!< Input function is not inverted */
#define IOCON_PIO_MODE_INACT          0x00u   /*!< No addition pin function */
#define IOCON_PIO_MODE_PULLUP         0x10u   /*!< Selects pull-up function */
#define IOCON_PIO_OPENDRAIN_DI        0x00u   /*!< Open drain is disabled */
#define IOCON_PIO_SLEW_STANDARD       0x00u   /*!< Standard mode, output slew rate control is enabled */

const uint32_t port0_pin11_config = (
    IOCON_PIO_FUNC1 |                                        /* Pin is configured as FC3_SCK */
    IOCON_PIO_MODE_PULLUP |                                  /* Selects pull-up function */
    IOCON_PIO_INV_DI |                                       /* Input function is not inverted */
    IOCON_PIO_DIGITAL_EN |                                   /* Enables digital function */
    IOCON_PIO_INPFILT_OFF |                                  /* Input filter disabled */
    IOCON_PIO_SLEW_STANDARD |                                /* Standard mode, output slew rate control is enabled */
    IOCON_PIO_OPENDRAIN_DI                                   /* Open drain is disabled */
);
  
const uint32_t port0_pin12_config = (
    IOCON_PIO_FUNC1 |                                        /* Pin is configured as FC3_RXD_SDA_MOSI */
    IOCON_PIO_MODE_PULLUP |                                  /* Selects pull-up function */
    IOCON_PIO_INV_DI |                                       /* Input function is not inverted */
    IOCON_PIO_DIGITAL_EN |                                   /* Enables digital function */
    IOCON_PIO_INPFILT_OFF |                                  /* Input filter disabled */
    IOCON_PIO_SLEW_STANDARD |                                /* Standard mode, output slew rate control is enabled */
    IOCON_PIO_OPENDRAIN_DI                                   /* Open drain is disabled */
);

const uint32_t port0_pin13_config = (
    IOCON_PIO_FUNC1 |                                        /* Pin is configured as FC3_TXD_SCL_MISO */
    IOCON_PIO_MODE_PULLUP |                                  /* Selects pull-up function */
    IOCON_PIO_INV_DI |                                       /* Input function is not inverted */
    IOCON_PIO_DIGITAL_EN |                                   /* Enables digital function */
    IOCON_PIO_INPFILT_OFF |                                  /* Input filter disabled */
    IOCON_PIO_SLEW_STANDARD |                                /* Standard mode, output slew rate control is enabled */
    IOCON_PIO_OPENDRAIN_DI                                   /* Open drain is disabled */
);

const uint32_t port0_pin14_config = (
    IOCON_PIO_FUNC1 |                                        /* Pin is configured as FC3_TXD_SCL_MISO */
    IOCON_PIO_MODE_PULLUP |                                  /* Selects pull-up function */
    IOCON_PIO_INV_DI |                                       /* Input function is not inverted */
    IOCON_PIO_DIGITAL_EN |                                   /* Enables digital function */
    IOCON_PIO_INPFILT_OFF |                                  /* Input filter disabled */
    IOCON_PIO_SLEW_STANDARD |                                /* Standard mode, output slew rate control is enabled */
    IOCON_PIO_OPENDRAIN_DI                                   /* Open drain is disabled */
);

static   spi_master_handle_t    g_MPU9250SpiHandle;
volatile uint8_t                g_MPU9250SpiFinished;
volatile uint8_t                g_MPU9250SpiDMATxFinished;
volatile uint8_t                g_MPU9250SpiDMARxFinished;
volatile uint8_t                g_MPU9250SpiDMAFinished;

volatile spi_dma_handle_t       g_MPU9250SpiDMAHandle;

volatile dma_transfer_config_t  g_MPU9250SpiDMATxConfig;
volatile dma_transfer_config_t  g_MPU9250SpiDMARxConfig;

volatile spi_transfer_t         g_MPU9250SpiXfer = {0};
volatile uint8_t                g_MPU9250TxBuf[2];
volatile uint8_t                g_MPU9250RxBuf[2];

volatile uint32_t               g_MPU9250TxDwBuf[32];
volatile uint32_t               g_MPU9250RxDwBuf[32];


volatile MPU9250_SensorInfo_T   g_MPU9250SensorData = {0, 0, 0, };

volatile dma_handle_t           g_MPU9250SpiTxHandle;
volatile dma_handle_t           g_MPU9250SpiRxHandle;

static void MPU9250SpiCallback(SPI_Type *base, spi_master_handle_t *handle, status_t status, void *userData)
{
    g_MPU9250SpiFinished = true;
}

static void MPU9250SpiDMACallback(SPI_Type *base, spi_master_handle_t *handle, status_t status, void *userData)
{
    g_MPU9250SpiDMAFinished = true;
}

static void MPU9250SpiDMATxCallback(dma_handle_t *handle, void *param, bool transferDone, uint32_t tcds)
{
    g_MPU9250SpiDMATxFinished = true;
    if(g_MPU9250SpiDMARxFinished == true)
    {
        g_MPU9250SpiDMAFinished = true;
    }
}

static void MPU9250SpiDMARxCallback(dma_handle_t *handle, void *param, bool transferDone, uint32_t tcds)
{
    g_MPU9250SpiDMARxFinished = true;
    if(g_MPU9250SpiDMATxFinished == true)
    {
        g_MPU9250SpiDMAFinished = true;
    }
}


void mpu9250_pins_init(void)
{
    spi_master_config_t SpiMPU9250Config = {0};
    uint32_t Spi3SourceClock = 0;
    
    /* DMA init */
    DMA_Init(DMA0);
    
    IOCON_PinMuxSet(IOCON, 0, 11, port0_pin11_config);
    IOCON_PinMuxSet(IOCON, 0, 12, port0_pin12_config);
    IOCON_PinMuxSet(IOCON, 0, 13, port0_pin13_config);
    IOCON_PinMuxSet(IOCON, 0, 14, port0_pin14_config);
    
    /* Define the init structure for the output LED pin*/
    gpio_pin_config_t spi_cs_config = {
        kGPIO_DigitalOutput, 0,
    };
    /* Init output LED GPIO. */
    GPIO_PinInit(GPIO, MPU9250_CS_PORT, MPU9250_CS_PIN, &spi_cs_config);
    GPIO_WritePinOutput(GPIO, MPU9250_CS_PORT, MPU9250_CS_PIN, 1);
    
    /* attach 12 MHz clock to SPI2 */
    CLOCK_AttachClk(kFRO12M_to_FLEXCOMM3);
    /* reset FLEXCOMM for SPI */
    RESET_PeripheralReset(kFC3_RST_SHIFT_RSTn);

    /* Init SPI master */
    /*
     * masterConfig.enableLoopback = false;
     * masterConfig.polarity = flexSPI_ClockPolarityActiveHigh;
     * masterConfig.phase = flexSPI_ClockPhaseFirstEdge;
     * masterConfig.direction = flexSPI_MsbFirst;
     * masterConfig.baudRate_Bps = 500000U;
     * masterConfig.dataWidth = flexSPI_Data8Bits;
     * config->sselNum = flexSPI_Ssel0;
     */
    SPI_MasterGetDefaultConfig(&SpiMPU9250Config);
    Spi3SourceClock = MPU9250_SPI_CLKFREQ;
    SpiMPU9250Config.sselNum = (spi_ssel_t)0;
    SpiMPU9250Config.baudRate_Bps = 600000;
    SPI_MasterInit(MPU9250_SPI, &SpiMPU9250Config, Spi3SourceClock);
    
    DMA_EnableChannel(DMA0, MPU9250_DMATX_CHANNEL);
    DMA_EnableChannel(DMA0, MPU9250_DMARX_CHANNEL);
    DMA_SetChannelPriority(DMA0, MPU9250_DMATX_CHANNEL, kDMA_ChannelPriority3);
    DMA_SetChannelPriority(DMA0, MPU9250_DMARX_CHANNEL, kDMA_ChannelPriority2);
    DMA_CreateHandle((dma_handle_t *)&g_MPU9250SpiTxHandle, DMA0, MPU9250_DMATX_CHANNEL);
    DMA_CreateHandle((dma_handle_t *)&g_MPU9250SpiRxHandle, DMA0, MPU9250_DMARX_CHANNEL);
//    DMA_SetCallback((dma_handle_t *)&g_MPU9250SpiTxHandle, MPU9250SpiDMATxCallback, NULL);
//    DMA_SetCallback((dma_handle_t *)&g_MPU9250SpiRxHandle, MPU9250SpiDMARxCallback, NULL);

    g_MPU9250SpiFinished = false;

    /* Create handle for SPI master */
    SPI_MasterTransferCreateHandle(MPU9250_SPI, &g_MPU9250SpiHandle, MPU9250SpiCallback, NULL); 
//    /* Set up spi master */
//    SPI_MasterTransferCreateHandleDMA(MPU9250_SPI, &g_MPU9250SpiDMAHandle, MPU9250SpiCallback, NULL, (dma_handle_t *)&g_MPU9250SpiTxHandle, (dma_handle_t *)&g_MPU9250SpiRxHandle);
    
    /* Set priority, slave have higher priority */
    NVIC_SetPriority(MPU9250_SPI_IRQ, 1U);
}

uint8_t mpu9250_reg_write(uint8_t reg, uint8_t value)
{
	MPU9250_CS_CLR();

    memset((void *)g_MPU9250TxBuf, 0x00, sizeof(g_MPU9250TxBuf));
    memset((void *)g_MPU9250RxBuf, 0x00, sizeof(g_MPU9250RxBuf));
    g_MPU9250TxBuf[0] = reg;
    g_MPU9250TxBuf[1] = value;
    /* SPI master start transfer */
    g_MPU9250SpiXfer.txData = (uint8_t *)g_MPU9250TxBuf;
    g_MPU9250SpiXfer.dataSize = 2;
    g_MPU9250SpiXfer.rxData = (uint8_t *)g_MPU9250RxBuf;
    g_MPU9250SpiXfer.configFlags |= kSPI_FrameAssert;
    
    g_MPU9250SpiFinished = false;
    SPI_MasterTransferNonBlocking(MPU9250_SPI, &g_MPU9250SpiHandle, (spi_transfer_t *)&g_MPU9250SpiXfer);
    
    while (g_MPU9250SpiFinished != true)
    {
    }
    
	MPU9250_CS_SET();
	return 0x01U;
}

uint8_t mpu9250_reg_read(uint8_t reg)
{
	MPU9250_CS_CLR();

    memset((void *)g_MPU9250TxBuf, 0x00, sizeof(g_MPU9250TxBuf));
    memset((void *)g_MPU9250RxBuf, 0x00, sizeof(g_MPU9250RxBuf));
    g_MPU9250TxBuf[0] = reg|0x80;
    g_MPU9250TxBuf[1] = 0xFF;
    
    /* SPI master start transfer */
    g_MPU9250SpiXfer.txData       = (uint8_t *)g_MPU9250TxBuf;
    g_MPU9250SpiXfer.dataSize     = 2;
    g_MPU9250SpiXfer.rxData       = (uint8_t *)g_MPU9250RxBuf;
    g_MPU9250SpiXfer.configFlags |= kSPI_FrameAssert;
    
    g_MPU9250SpiFinished = false;
    SPI_MasterTransferNonBlocking(MPU9250_SPI, &g_MPU9250SpiHandle, (spi_transfer_t *)&g_MPU9250SpiXfer);

    /* Wait until transfer completed */
    while (g_MPU9250SpiFinished != true)
    {
        
    }
    
	MPU9250_CS_SET();
	return g_MPU9250RxBuf[1];
}

void mpu9250_delay(uint32_t cnt)
{
    uint32_t i, j;
    for(i=0; i<cnt; i++)
    {
        for(j=0; j<10000; j++);
    }
}


void mpu8250_dmaRead_init(void)
{
    uint8_t i;
    memset((void *)g_MPU9250TxDwBuf, 0x00, sizeof(g_MPU9250TxDwBuf));
    g_MPU9250TxDwBuf[0] = (MPU9250_ACCEL_XOUT_H|0x80) | (0x07 << 24);
    for(i=0; i<23; i++)
    {
        g_MPU9250TxDwBuf[i+1] = (0xFF) | (0x07 << 24);
    }
    
    SPI_MasterTransferCreateHandleDMA(MPU9250_SPI, &g_MPU9250SpiDMAHandle, MPU9250SpiDMACallback, NULL, (dma_handle_t *)&g_MPU9250SpiTxHandle, (dma_handle_t *)&g_MPU9250SpiRxHandle);

    /* Start master transfer */
    g_MPU9250SpiXfer.txData = (uint8_t *)&g_MPU9250TxDwBuf;
    g_MPU9250SpiXfer.rxData = (uint8_t *)&g_MPU9250RxDwBuf;
    g_MPU9250SpiXfer.dataSize = 23 * sizeof(g_MPU9250TxDwBuf[0]);;
    g_MPU9250SpiXfer.configFlags |= kSPI_FrameAssert;
}


void mpu8250_value_read(MPU9250_SensorInfo_T *sensorDat)
{
	MPU9250_CS_CLR();

    g_MPU9250SpiDMAFinished = g_MPU9250SpiDMATxFinished = g_MPU9250SpiDMARxFinished = false;

    if (kStatus_Success != SPI_MasterTransferDMA(MPU9250_SPI, &g_MPU9250SpiDMAHandle, &g_MPU9250SpiXfer))
    {
        PRINTF("There is error when start SPI_MasterTransferDMA \r\n ");
    }
    while (g_MPU9250SpiDMAFinished != true)
    {
    }
    
	sensorDat->AccX        = (int16_t)(g_MPU9250RxDwBuf[ 1]<<8) | (g_MPU9250RxDwBuf[ 2]&0x000000FF);
	sensorDat->AccY        = (int16_t)(g_MPU9250RxDwBuf[ 3]<<8) | (g_MPU9250RxDwBuf[ 4]&0x000000FF);
	sensorDat->AccZ        = (int16_t)(g_MPU9250RxDwBuf[ 5]<<8) | (g_MPU9250RxDwBuf[ 6]&0x000000FF);
    
    sensorDat->Temperature = (int16_t)(g_MPU9250RxDwBuf[ 7]<<8) | (g_MPU9250RxDwBuf[ 8]&0x000000FF); 
    
	sensorDat->GyrX        = (int16_t)(g_MPU9250RxDwBuf[ 9]<<8) | (g_MPU9250RxDwBuf[10]&0x000000FF);
	sensorDat->GyrY        = (int16_t)(g_MPU9250RxDwBuf[11]<<8) | (g_MPU9250RxDwBuf[12]&0x000000FF);
	sensorDat->GyrZ        = (int16_t)(g_MPU9250RxDwBuf[13]<<8) | (g_MPU9250RxDwBuf[14]&0x000000FF);
	
    sensorDat->MagX        = (int16_t)(g_MPU9250RxDwBuf[17]<<8) | (g_MPU9250RxDwBuf[16]&0x000000FF);
	sensorDat->MagY        = (int16_t)(g_MPU9250RxDwBuf[19]<<8) | (g_MPU9250RxDwBuf[18]&0x000000FF);
	sensorDat->MayZ        = (int16_t)(g_MPU9250RxDwBuf[21]<<8) | (g_MPU9250RxDwBuf[20]&0x000000FF);
	
	MPU9250_CS_SET();
}

void ak8963_init()
{
	uint8_t temp=0;
	
	mpu9250_delay(500);
	
	mpu9250_reg_write(MPU9250_I2C_SLV0_ADDR, 0x0C);
	mpu9250_reg_write(MPU9250_I2C_SLV0_DO,   0x01);
	mpu9250_reg_write(MPU9250_I2C_SLV0_REG,  MPU9250_AK8963_CNTL2);
	mpu9250_reg_write(MPU9250_I2C_SLV0_CTRL,0X81);
	
	mpu9250_delay(100);
	
	//Set Slave to Read AK8963
	mpu9250_reg_write(MPU9250_I2C_SLV0_ADDR,0x8c);
	mpu9250_reg_write(MPU9250_I2C_SLV0_REG,0x00);
	mpu9250_reg_write(MPU9250_I2C_SLV0_CTRL,0x81);
	mpu9250_delay(1000);
	temp = mpu9250_reg_read(MPU9250_EXT_SENS_DATA_00);
		
	if(temp == 0x48)
	{
        mpu9250_delay(500);
	}
	else 
	{
        while(1);
	}
	
	mpu9250_reg_write(MPU9250_I2C_SLV0_ADDR,0x0C);
	mpu9250_reg_write(MPU9250_I2C_SLV0_DO,0x16);
	mpu9250_reg_write(MPU9250_I2C_SLV0_REG, MPU9250_AK8963_CNTL1);
	mpu9250_reg_write(MPU9250_I2C_SLV0_CTRL,0X81);
	mpu9250_delay(1000);
	mpu9250_reg_write(MPU9250_I2C_SLV0_ADDR,0x8C);
	mpu9250_reg_write(MPU9250_I2C_SLV0_REG, MPU9250_AK8963_CNTL1);
	mpu9250_reg_write(MPU9250_I2C_SLV0_CTRL,0X81);
	mpu9250_delay(1000);
	temp = mpu9250_reg_read(MPU9250_EXT_SENS_DATA_00);
	if(temp == 0x16)
	{
        mpu9250_delay(500);
	}
	else 
	{
        while(1);
	}
	//OLED_Clear();
	
	mpu9250_reg_write(MPU9250_I2C_SLV0_ADDR, 0x8C);
	mpu9250_delay(1000);
	mpu9250_reg_write(MPU9250_I2C_SLV0_REG,  MPU9250_AK8963_ST1);
	mpu9250_reg_write(MPU9250_I2C_SLV0_CTRL, 0x88);
	mpu9250_delay(1000);
}

uint8_t mpu9250_func_init(void)
{
    mpu9250_pins_init();
    if(mpu9250_reg_read(MPU9250_WHO_AM_I) == 0x71)
    {
        mpu9250_reg_write(MPU9250_PWR_MGMT_1,0X80);
		mpu9250_delay(1);
		mpu9250_reg_write(MPU9250_PWR_MGMT_1,0X01);
		mpu9250_reg_write(MPU9250_PWR_MGMT_2,0X00);
		mpu9250_reg_write(MPU9250_CONFIG,0X02);
		mpu9250_reg_write(MPU9250_SMPLRT_DIV,0x00);
		mpu9250_reg_write(MPU9250_GYRO_CONFIG,0X10);
		mpu9250_reg_write(MPU9250_ACCEL_CONFIG,0x08);
		mpu9250_reg_write(MPU9250_ACCEL_CONFIG2,0x00);
		mpu9250_reg_write(MPU9250_INT_PIN_CFG,0x30);
        mpu9250_reg_write(MPU9250_I2C_MST_CTRL,0x4D); 
		mpu9250_reg_write(MPU9250_USER_CTRL,0X30);
		mpu9250_delay(1);
		
		ak8963_init();
		return 1;
    }

    return 0;
}



// end file
