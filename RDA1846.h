#ifndef RDA1846_H
#define RDA1846_H


#define DEBUGPRINT
//#define DEBUGPRINTI2C

/* Retro-compatibility with arduino 0023 and previous version */
#if ARDUINO >= 100
#include "Arduino.h"
#define I2CWRITE(x) Wire.write(x)
#define I2CREAD() Wire.read()
#else
#include "WProgram.h"
#define I2CWRITE(x) Wire.send(x)
#define I2CREAD() Wire.receive()
#endif


#include <inttypes.h>

#ifndef I2CTIMEOUT
  #define I2CTIMEOUT 100
#endif

#ifndef RDA_SEN
  #define RDA_SEN 0
#endif

#if RDA_SEN 
  #define RDA_ADDRESS  0b0101110 //0x2E
#else
  #define RDA_ADDRESS  0x71 //0x71
#endif

//some default values may be in question
//Registers attempted to be identified from various sources. Documentation leaves a lot to be desired...

#define REG_CHIP_ID 0x00  	//Chip ID default:0x1846
#define REG_MID 0x01 		//Metal Revision ID default:0x0007
#define REG_ADC 0x03 		//default:0x2B51 
							//BIT[12]tx_adc_reset_dr 1=enable direct reg 0=disable direct reg  
							//BIT[11]tx_adc_reset_reg 1=enable reset adc 0=disable reset adc
#define REG_CLK_MODE 0x04
#define REG_LDO_BYPASS 0x08 	//default:0x02A0
								//BIT[14] 1=bypass all LDOs inside RDA1846S. 0=normal. VHF Band must be 0.
#define RGD_LDO_CONTROL 0x09 	//default:0x03C2
								//BIT[9:7] ldo_dig_vbit If 08H[14]=0,control LDO Vout for digital. 100=2.20V 101=2.40V 110=2.80V 111=3.30V
#define REG_PABIAS 0x0A  		//BIT[4:11]padrv_ibit 10:6pga_gain 5:0pabias_voltage
#define REG_PLL_LOCK 0x0D 		//BIT[15:0]=Pll Unlock 1=pll lock (read only)
#define REG_RF_BAND 0x0F
#define REG_TUNING 0x15  //BIT[12:9]tuning_bit Tuning IF filter center freq & bw
#define REG_GPIO 0x1F
#define REG_PLL 0x24 //default:0x0001 
					//BIT[15]pll_lock_det_sel BIT[14:13]reset_pll_lock_delay BIT[7]dsp_resetn_dr 6dsp_resetn_reg
					//BIT[15] 1=bypass pll lock det function
					//BIT[14:13] pll_lock_delay
					// 00=10uS
					// 01=20uS
					// 10=30uS
					// 11=40uS
#define REG_FREQ_H 0x29
#define REG_FREQ_L 0x2A  //default:3A84
#define REG_XTAL_FREQ 0x2B
#define REG_ADC_FREQ 0x2C
#define REG_INT 0x2D
#define REG_MODE 0x30  	//default:0x0000
						//BIT[15] - other
						//BIT[14] - xtal_mode 1=26MHz/13MHz 0=25.6MHz/12.8MHz
						//BIT[13] - filter_band_sel Analog Filter Band 1=25khz 0=12.5khz
						//BIT[12] - band_mode_sel DSP Band Mode Select 1=25khz 0=12.5khz
						//BIT[11] - tail_elim_en 1=enable eliminate tail noise, 0= disable eliminate 
						//BIT[10] - direct_reg 1=enable direct reg 0=disable
						//BIT[9:8] - other
						//BIT[7] - mute 1=mute 0=no mute
						//BIT[6] - tx_on
						//BIT[5] - rx_on
						//BIT[4] - vox_on 1=enable vox 0=disable vox
						//BIT[3] - sq_on 1=auto squelch 0=off
						//BIT[2] - pdn_reg 1=follow PDN pin, 0=disbale
						//BIT[1] - chip_cal_en 1=cal enable, 0=cal disable
						//BIT[0] - soft_reset 1=reset-all registers reset to default, 0=normal
#define REG_AGC 0x32 	//default:0x7497
						//bits 11:6=agc_target_pwr unit=2dB
#define REG_TONE1_FREQ 0x35
#define REG_TONE2_FREQ 0x36
#define REG_OUTSEL 0x3A //default:0x40C3
						//BIT[15] code_out_sel 1=output code sample signal via GPIO3, 0=output dtmf_sample/dtmf_idle signal VIA GPIO3
						//BIT[14:12] voice_sel voice path select
						//BIT[11] sq_out_sel 1=gpio6=sq_cmp & sub_aud_cmp; 0=gpio6=sq_smp
						//BIT[10:6] sq_dten sq condition enable
						//BIT[5] ctcss/cdcss_out_sel - Select CTCSS/CDCSS mode for rx
						//BIT[4:0] ctcss/cdcss_deten = Select CTCSS/CDCSS detect mode for Rx
#define REG_TXVOICE_CHAN 0x3C
#define REG_VOX_H 0x41 //default:0x4006  BIT[6:0]voice_gain_tx
#define REG_VOX_L 0x42
#define REG_TX_DEVIATION 0x43
#define REG_RX_VOL 0x44 //default:00FF 
						//BIT[11:8] Voice digital gain after tx ADC down sample
						//BIT[7:4] Analog DAC gain
						//BIT[3:0] Digital Voice Gain
#define REG_SUBAUDIO 0x45
#define REG_SQ_H 0x48
#define REG_SQ_L 0x49
#define REG_CTCSS_FREQ 0x4A
#define REG_CTCSS_CODE_H 0x4B
#define REG_CTCSS_CODE_L 0x4C 
#define REG_CTCSS_PHASE 0x4E //default:20C2
#define REG_SQOUT_SEL 0x54
#define REG_VOICE_FILT 0x58
#define REG_FLAG 0x5C
#define REG_RSSI 0x5F
#define REG_VSSI 0x60
#define REG_DTMF 0x63
#define REG_DTMF_C0_C1 0x66
#define REG_DTMF_C2_C3 0x67
#define REG_DTMF_C4_C5 0x68
#define REG_DTMF_C6_C7 0x69
#define REG_DTMF_DET 0x6C

#define SUBAUDIODISABLE 0x0000
#define INNERCTCSS 0x0001
#define INNERCDCSS 0x0002
#define OUTERCTCSS 0x0005
#define OUTERCDCSS 0x0006
#define SUBAUDIO_CMP_GPIO 0x0008
#define SUBAUDIO_SDO_GPIO 0x0000
#define CDCSS_24BIT 0x0010
#define CDCSS_23BIT 0x0000
#define CDCSS_INVERSE 0x0080
#define CDCSS_POSITIVE 0x0800

#define INT_CSS_CMP 0x0200
#define INT_RXON_RF 0x0100
#define INT_TXON_RF 0x0080
#define INT_DTMF_IDLE 0x0040
#define INT_CTCSS_PHASE 0x0020
#define INT_IDLE_TIMEOUT 0x0010
#define INT_RXON_RF_TIMEOUT 0x0008
#define INT_SQ 0x0004
#define INT_TXON_RF_TIMEOUT 0x0002
#define INT_VOX 0x0001

struct RDACommand {
  uint8_t reg;
  uint16_t data;
};

  enum CHAN_MODE {
    MODE25KHz = 0x00,
    MODE125KHz = 0x01
  };
  
  enum TX_VOICE_CHAN {
    MIC = 0x00,
	INNERTONE2 = 0x01,
	GPIO1CODE = 0x02
  };
  
  enum BIAS_VOLTAGE {
    BIAS101V = 0x00,
	BIAS105V = 0x01,
	BIAS109V = 0x02,
	BIAS118V = 0x04,
	BIAS134V = 0x08,
	BIAS168V = 0x10,
	BIAS245V = 0x20,
	BIAS313V = 0x7F
  };
  
  enum CTCSSPHASE {
    SHIFT120DEGREES = 0x00,
	SHIFT180DEGREES = 0x01,
	SHIFT240DEGREES = 0x02
  };

  enum DTMF_MODE {
    SINGLETONE = 0x03,
	DUALTONE = 0x01,
	DIABLE = 0x00
  };

  enum DTMF_CLOCK {
    CLK12POINT8MHZ,
	CLK25POINT6MHZ,
	CLK13MHZ,
	CLK26MHZ
  };
  
  enum GPIO7 {
    GP7_HIZ,
	VOX,
	GP7_LOW,
	GP7_HIGH
  };
  
  enum GPIO6 {
    GP6_HIZ,
    SQ,
	GP6_LOW,
	GP6_HIGH
  };
  
  enum GPIO5 {
    GP5_HIZ,
	TXON_RF,
	GP5_LOW,
	GP5_HIGH
  };
  
  enum GPIO4 {
    GP4_HIZ,
	RXON_RF,
	GP4_LOW,
	GP4_HIGH
  };
  
  enum GPIO3 {
    GP3_HIZ,
	SDO,
	GP3_LOW,
	GP3_HIGH
  };
  
  enum GPIO2 {
    GP2_HIZ,
	GP2_INT,
	GP2_LOW,
	GP2_HIGH
  };
  
  enum GPIO1 {
    GP1_HIZ,
	CODE_IO,
	GP1_LOW,
	GP1_HIGH
  };
  
  enum GPIO0 {
    GP0_HIZ,
	CSS_IOC,
	GP0_LOW,
	GP0_HIGH
  };
  
  enum STMODE {
    TXON_RXONAUTO = 0x10,
	RXONAUTO_TXONMANUAL = 0x01,
	TXONMANUAL_RXONMANUAL = 0x00
  };

class RDA1846 {
public:

struct RDACommand RDAinit[30] = {  //init values from http://elazary.com
 { 0x30, 0x0001 }, 
 { 0x30, 0x0004 },
 { 0x04, 0x0FD0 },
 { 0x0B, 0x1A10 },
 { 0x2B, 0x32C8 },
 { 0x2C, 0x1964 },
 { 0x32, 0x627C },
 { 0x33, 0x0AF2 },
 { 0x47, 0x2C2F },
 { 0x4E, 0x293A }, 
 { 0x54, 0x1D4C },
 { 0x56, 0x0652 }, 
 { 0x6E, 0x062D },
 { 0x70, 0x1029 },
 { 0x7F, 0x0001 },
 { 0x05, 0x001F },
 { 0x7F, 0x0000 },
 { 0x30, 0x3006 },
 { 0x0A, 0x0400 },        //PA Bias 0000 0100 '00 00' 0000
 { 0x1F, 0x0000 },        //GPIO selection 0001 1110 1011 1001
 { 0x30, 0x3006 }, 
 { 0x0F, 0x6be4 },
 { 0x29, 0x0011 }, //145.525
 { 0x2A, 0xC3A8 }, //145.525
 { 0x48, 0x03FF }, //0000 0011 1111 0000
 { 0x49, 0x01b3 }, //0003
 { 0x3C, 0x0958 }, //0000 1001 0101 1000 
 { 0x43, 0x1F1F },
 { 0x30, 0x3006 },
 { 0x36, 0x1000 },

};

uint16_t dtmfTone[10][2] = { //init values from http://elazary.com
 { 5472, 3854 }, //0 
 { 4952, 2855 }, //1 
 { 5472, 2855 }, //2 
 { 6050, 2855 }, //3 
 { 4952, 3154 }, //4 
 { 5472, 3154 }, //5 
 { 6050, 3154 }, //6 
 { 4952, 3490 }, //7 
 { 5472, 3490 }, //8 
 { 6050, 3490 }, //9 
};


  
  void begin(void);
  void writeWord(uint16_t data, uint8_t reg);
  bool readWord(uint16_t *data, uint8_t reg);
  void rdaRegDump(uint8_t start, uint8_t end);
  void rdaInit(void);
  void setRefClock(double freq);
  void setFrequency(double freq);
  void setMode(uint16_t mode);
  uint16_t getMode(void);
  void setChannelMode(CHAN_MODE cmode);
  void txOn(void);
  void txOff(void);
  void rxOn(void);
  void rxOff(void);
  void setTxMode(bool cmode);
  void setRxMode(bool cmode);
  void setDeepSleep(bool cmode);
  void setTxChannel(TX_VOICE_CHAN cmode);
  void setPABias(BIAS_VOLTAGE volts);
  void setSubaudio(uint16_t mode);
  void setSubaudioFreq(float freq);
  void setCDCSScode(uint32_t code);
  void setSquelch(bool sq);
  void setCTCSSsel(bool sel);
  void setCTCSSdet(bool det);
  void setSquelchThreshold(uint16_t open, uint16_t shut);
  void setSquelchThresholddBm(int16_t open, int16_t shut);
  void setSquelchout(bool sq);
  void setVOX(bool vox);
  void setVOXThresholdmV(uint16_t open, uint16_t shut);
  void setTailNoiseElim(bool tn);
  void setTailNoisePhase(CTCSSPHASE phase);
  void setDTMFMode(DTMF_MODE mode);
  void setDTMFToneTime(uint8_t t);
  void setDTMFIdleTime(uint8_t t);
  void setDTMFTone1Freq(uint16_t freq);
  void setDTMFTone2Freq(uint16_t freq);
  void setDTMFIdle(bool i);
  void setDTMFClk(DTMF_CLOCK clk);
  void setTxFMDeviation(uint8_t dev);
  void setTxCTCSSDeviation(uint8_t dev);
  void setRxAnalogDACGain(uint8_t range);
  void setRxDigitalVoiceGain(uint8_t range);
  void setGPIO7(GPIO7 mode);
  void setGPIO6(GPIO6 mode);
  void setGPIO5(GPIO5 mode);
  void setGPIO4(GPIO4 mode);
  void setGPIO3(GPIO3 mode);
  void setGPIO2(GPIO2 mode);
  void setGPIO1(GPIO1 mode);
  void setGPIO0(GPIO0 mode);
  void setIntMode(uint16_t mode);
  void setSTMode(STMODE mode);
  void setEmphasis(bool emp);
  uint16_t getRSSI(void);
  uint16_t getVSSI(void);
  uint8_t getDTMFIndex(void);
  uint8_t getDTMFCode(void);
  uint16_t getFlags(void);
  uint16_t getChipID(void);
  void reset(void);
  
protected:

private:
  uint8_t _address;
  
};


#endif
