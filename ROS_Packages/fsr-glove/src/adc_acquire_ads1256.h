/**
 * Copyright 2015 by California Institute of Technology.  ALL RIGHTS RESERVED.
 * United  States  Government  sponsorship  acknowledged.   Any commercial use
 * must   be  negotiated  with  the  Office  of  Technology  Transfer  at  the
 * California Institute of Technology.
 *
 * This software may be subject to  U.S. export control laws  and regulations.
 * By accepting this document,  the user agrees to comply  with all applicable
 * U.S. export laws and regulations.  User  has the responsibility  to  obtain
 * export  licenses,  or  other  export  authority  as may be required  before
 * exporting  such  information  to  foreign  countries or providing access to
 * foreign persons.
 *
 * 
 * @file adc_acquire_ads1255.h
 * @author Brandon Rothrock
 * 
 * @brief ADC data acquisition for TI ADS1255 on RPI.
 * @details
 *
 */

#pragma once

#include <stdint.h>

#define DRDY    RPI_GPIO_P1_11  //P0
#define RST     RPI_GPIO_P1_12  //P1
#define SPICS   RPI_GPIO_P1_15  //P3   ads1256 CS
#define SPICS1  RPI_GPIO_P1_16  //P4   DAC8552 CS

#define CS_1()  bcm2835_gpio_write(SPICS,HIGH)
#define CS_0()  bcm2835_gpio_write(SPICS,LOW)

#define CS1_1() bcm2835_gpio_write(SPICS1,HIGH)
#define CS1_0() bcm2835_gpio_write(SPICS1,LOW)


#define DRDY_IS_LOW() ((bcm2835_gpio_lev(DRDY)==0))

#define RST_1()   bcm2835_gpio_write(RST,HIGH);
#define RST_0()   bcm2835_gpio_write(RST,LOW);

// #define channel_A   0x30
// #define channel_B   0x34


/* gain channelî */
typedef enum {
  ADS1256_GAIN_1      = (0),  /* GAIN   1 */
  ADS1256_GAIN_2      = (1),  /*GAIN   2 */
  ADS1256_GAIN_4      = (2),  /*GAIN   4 */
  ADS1256_GAIN_8      = (3),  /*GAIN   8 */
  ADS1256_GAIN_16     = (4),  /* GAIN  16 */
  ADS1256_GAIN_32     = (5),  /*GAIN    32 */
  ADS1256_GAIN_64     = (6),  /*GAIN    64 */
} ADS1256_GAIN_E;

/* Sampling speed choice*/
/* 
  11110000 = 30,000SPS (default)
  11100000 = 15,000SPS
  11010000 = 7,500SPS
  11000000 = 3,750SPS
  10110000 = 2,000SPS
  10100001 = 1,000SPS
  10010010 = 500SPS
  10000010 = 100SPS
  01110010 = 60SPS
  01100011 = 50SPS
  01010011 = 30SPS
  01000011 = 25SPS
  00110011 = 15SPS
  00100011 = 10SPS
  00010011 = 5SPS
  00000011 = 2.5SPS
*/
typedef enum {
  ADS1256_30000SPS = 0,
  ADS1256_15000SPS,
  ADS1256_7500SPS,
  ADS1256_3750SPS,
  ADS1256_2000SPS,
  ADS1256_1000SPS,
  ADS1256_500SPS,
  ADS1256_100SPS,
  ADS1256_60SPS,
  ADS1256_50SPS,
  ADS1256_30SPS,
  ADS1256_25SPS,
  ADS1256_15SPS,
  ADS1256_10SPS,
  ADS1256_5SPS,
  ADS1256_2d5SPS,

  ADS1256_DRATE_MAX
} ADS1256_DRATE_E;

#define ADS1256_DRAE_COUNT = 15;

typedef struct {
  ADS1256_GAIN_E Gain;    /* GAIN  */
  ADS1256_DRATE_E DataRate; /* DATA output  speed*/
  int32_t AdcNow[8];      /* ADC  Conversion value */
  int32_t AdcOne;	/* ADC one time*/
  uint8_t Channel;      /* The current channel*/
  uint8_t ScanMode; /*Scanning mode,   0  Single-ended input  8 channel£¬ 1 Differential input  4 channel*/
} ADS1256_VAR_T;



/*Register definition: Table 23. Register Map --- ADS1256 datasheet Page 30*/
enum {
  /*Register address, followed by reset the default values */
  REG_STATUS = 0, // x1H
  REG_MUX    = 1, // 01H
  REG_ADCON  = 2, // 20H
  REG_DRATE  = 3, // F0H
  REG_IO     = 4, // E0H
  REG_OFC0   = 5, // xxH
  REG_OFC1   = 6, // xxH
  REG_OFC2   = 7, // xxH
  REG_FSC0   = 8, // xxH
  REG_FSC1   = 9, // xxH
  REG_FSC2   = 10, // xxH
};

/* Command definition: TTable 24. Command Definitions --- ADS1256 datasheet Page 34 */
enum {
  CMD_WAKEUP  = 0x00, // Completes SYNC and Exits Standby Mode 0000  0000 (00h)
  CMD_RDATA   = 0x01, // Read Data 0000  0001 (01h)
  CMD_RDATAC  = 0x03, // Read Data Continuously 0000   0011 (03h)
  CMD_SDATAC  = 0x0F, // Stop Read Data Continuously 0000   1111 (0Fh)
  CMD_RREG    = 0x10, // Read from REG rrr 0001 rrrr (1xh)
  CMD_WREG    = 0x50, // Write to REG rrr 0101 rrrr (5xh)
  CMD_SELFCAL = 0xF0, // Offset and Gain Self-Calibration 1111    0000 (F0h)
  CMD_SELFOCAL= 0xF1, // Offset Self-Calibration 1111    0001 (F1h)
  CMD_SELFGCAL= 0xF2, // Gain Self-Calibration 1111    0010 (F2h)
  CMD_SYSOCAL = 0xF3, // System Offset Calibration 1111   0011 (F3h)
  CMD_SYSGCAL = 0xF4, // System Gain Calibration 1111    0100 (F4h)
  CMD_SYNC    = 0xFC, // Synchronize the A/D Conversion 1111   1100 (FCh)
  CMD_STANDBY = 0xFD, // Begin Standby Mode 1111   1101 (FDh)
  CMD_RESET   = 0xFE, // Reset to Power-Up Values 1111   1110 (FEh)
};


static const uint8_t s_tabDataRate[ADS1256_DRATE_MAX] =
{
  0xF0,   /*reset the default values  */
  0xE0,
  0xD0,
  0xC0,
  0xB0,
  0xA1,
  0x92,
  0x82,
  0x72,
  0x63,
  0x53,
  0x43,
  0x33,
  0x20,
  0x13,
  0x03
};






class ADCAcquire {
public:
  ADCAcquire();
  ~ADCAcquire();

  /**
   * @brief Reads all 8 channels from both ADC's
   * @return float array with voltages for each channel
   */
  float* read_all_channels();
  float  read_channel(uint8_t _ch);

private:

  void      bsp_DelayUS(uint64_t micros);
  void      bsp_InitADS1256();
  void      ADS1256_Send8Bit(uint8_t _data);
  void      ADS1256_DelayDATA();
  uint8_t   ADS1256_Recive8Bit();
  void      ADS1256_WriteReg(uint8_t _RegID, uint8_t _RegValue);
  uint8_t   ADS1256_ReadReg(uint8_t _RegID);
  void      ADS1256_WriteCmd(uint8_t _cmd);
  void      ADS1256_SetChannel(uint8_t _ch);
  void      ADS1256_SetDiffChannel(uint8_t _ch);
  void      ADS1256_WaitDRDY();
  int32_t   ADS1256_ReadData();
  void      ADS1256_StartScan(uint8_t _ucScanMode);
  void      ADS1256_CfgADC(ADS1256_GAIN_E _gain, ADS1256_DRATE_E _drate);
  uint8_t   ADS1256_ReadChipID();
  int32_t   ADS1256_GetAdc(uint8_t _ch);
  void	    ADS1256_ch_ISR();
  void      ADS1256_ISR();
  uint8_t   ADS1256_Scan(uint8_t _ch);
  uint8_t   ADS1256_ScanAll();

  /***************************************************/
  void      Write_DAC8552(uint8_t channel, uint16_t Data);
  uint16_t  Voltage_Convert(float Vref, float voltage);

  ADS1256_VAR_T g_tADS1256;

  bool initialized_;

  static const uint8_t ch_num = 8;
  int32_t adc_[ch_num];
  int32_t volt_[ch_num];
  float voltf_[ch_num];
};
