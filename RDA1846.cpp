/*
* Arduino Library for the RDA1846 VHF/UHF Transceiver Chip
*
* Copyright (C) 2015 RF Designs. All rights reserved.
*
* Author: Bob Frady <rfdesigns@live.com>
*
* This program is free software; you can redistribute it and/or
* modify it under the terms of the GNU General Public
* License v2 as published by the Free Software Foundation.
*
* This program is distributed in the hope that it will be useful,
* but WITHOUT ANY WARRANTY; without even the implied warranty of
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU
* General Public License for more details.
*
* You should have received a copy of the GNU General Public
* License along with this program; if not, write to the
* Free Software Foundation, Inc., 59 Temple Place - Suite 330,
* Boston, MA 021110-1307, USA.
*
* If you can't comply with GPLv2, alternative licensing terms may be
* arranged. Please contact RF Designs <rfdesigns@live.com> for proprietary
* alternative licensing inquiries.
*/


#include <Wire.h>

#include <RDA1846.h>


void RDA1846::begin(void)
{
  _address = RDA_ADDRESS;
  Wire.begin();
}

void RDA1846::writeWord(uint16_t data, uint8_t reg) {
  uint8_t TempReg;
  union {
    uint8_t b[2];
    uint16_t w;
  } datau;
  
  TempReg = reg;
  datau.w = data;

  if(TempReg > 0x7F) {  		//RDA1846 uses only 7-bit register addresses. For addresses > 0x7F
    uint16_t TempData=0x0001;  	//we must recursively call this function to enable register
								//writes above 0x7F		
	writeWord(TempData, 0x7F);
	
	reg &= 0x7F;
  }
  
#ifdef DEBUGPRINTI2C
    Serial.print("writeWord -- ");
    Serial.print(_address);
	Serial.print(":");
	Serial.print(reg, HEX);
	Serial.print(":");
	if(data < 0x1000)
	  Serial.print("0");
	if(data < 0x0100)
	  Serial.print("0");
	if(data < 0x0010)
	  Serial.print("0");
	Serial.println(data, HEX);
#endif

  Wire.beginTransmission(_address);
  I2CWRITE((uint8_t) reg);
  
  I2CWRITE((uint8_t) datau.b[1]);
  I2CWRITE((uint8_t) datau.b[0]);
  
  Wire.endTransmission();
  
  if(TempReg > 0x7F) {  			//Reset back to register < 0x7F
    uint16_t TempData=0x0000;  			   		
	
	writeWord(TempData, 0x7F);
  }
  
  return;
}

bool RDA1846::readWord(uint16_t *data, uint8_t reg) {
  uint8_t TempReg;
  union {
    uint8_t b[2];
    uint16_t w;
  } datau;
  
  TempReg = reg;
  datau.w = *data;

  if(TempReg > 0x7F) {  		//RDA1846 uses only 7-bit register addresses. For addresses > 0x7F
    uint16_t TempData=0x0001;  	//we must call the writeWord function to enable register
								//writes above 0x7F		
	writeWord(TempData, 0x7F);
	
	reg &= 0x7F;
  }

  Wire.beginTransmission(_address);
  I2CWRITE((uint8_t) reg);
  Wire.endTransmission(0);
  uint8_t timeout=0;
  
  Wire.requestFrom(_address, (uint8_t) 0x02);
  while(Wire.available() < 2) {
    timeout++;
	if(timeout > I2CTIMEOUT) {
	  return(true);
	}
	delay(1);
  } 		//Experimental
  
  datau.b[1] = I2CREAD();
  datau.b[0] = I2CREAD();
  Wire.endTransmission(1);
  
  *data = datau.w;
  
  if(TempReg > 0x7F) {  			//Reset back to register < 0x7F
    uint16_t TempData=0x0000;  			   		
	
	writeWord(TempData, 0x7F);
  }
  
#ifdef DEBUGPRINTI2C
    Serial.print("readWord -- ");
    Serial.print(_address);
	Serial.print(":");
	Serial.print(reg, HEX);
	Serial.print(":");
	if(*data < 0x1000)
	  Serial.print("0");
	if(*data < 0x0100)
	  Serial.print("0");
	if(*data < 0x0010)
	  Serial.print("0");
	Serial.println(*data, HEX);
#endif

return(false);
}


void RDA1846::rdaRegDump(uint8_t start, uint8_t end) {
  uint16_t val;
  
  Serial.println("Register Dump");
  Serial.println("--- BEGIN DUMP ---");
  for(uint8_t n=start;n <= end;n++) {
    readWord(&val, n);
	if(!(n%8)) {
      Serial.print("\n");
    }
	if(n<16)
	  Serial.print("0");
    Serial.print(n, HEX);
    Serial.print(":");
	if(val < 0x1000)
	  Serial.print("0");
	if(val < 0x0100)
	  Serial.print("0");
	if(val < 0x0010)
	  Serial.print("0");
    Serial.print(val, HEX);
    Serial.print(" ");
  }
  Serial.print("\n");
  Serial.println("--- END DUMP ---");  
}


void RDA1846::rdaInit(void) {

#ifdef DEBUGPRINT
  Serial.println("--DEBUG: rdaInit Print---");
#endif
  for(int x=0; x<30; x++) {
    writeWord(RDAinit[x].reg, RDAinit[x].data);
#ifdef DEBUGPRINT
    Serial.print(x);
	Serial.print(":");
	if(RDAinit[x].reg < 0x10)
	  Serial.print("0");
	Serial.print(RDAinit[x].reg, HEX);
	Serial.print(":");
	if(RDAinit[x].data < 0x1000)
	  Serial.print("0");
	if(RDAinit[x].data < 0x0100)
	  Serial.print("0");
	if(RDAinit[x].data < 0x0010)
	  Serial.print("0");
	Serial.println(RDAinit[x].data, HEX);
#endif
  }
}


void RDA1846::setRefClock(double freq) {		//Crystal Frequency in MHz
  uint16_t xtal_freq, adclk_freq;
  uint8_t mode;
  
  if((freq >= 12.0) && (freq <= 14.0)) {
    xtal_freq = freq * 1000.0;
	adclk_freq = freq * 500.0;  //(xtal_freq / 2) * 1000
	mode = 1;
  }
  
  if((freq >= 24.0) && (freq <= 28.0)) {
    xtal_freq = freq * 500.0;  //(xtal_freq / 2) * 1000
	adclk_freq = freq * 250.0;  //(xtal_freq / 4) * 1000
	mode = 0;
  }
  
  uint16_t tmp;
  
  writeWord(xtal_freq, REG_XTAL_FREQ);
  writeWord(adclk_freq, REG_ADC_FREQ);
  readWord(&tmp, REG_CLK_MODE);
  if(mode)
    bitSet(tmp, 0);
  else
    bitClear(tmp, 0);
	
  writeWord(tmp, REG_CLK_MODE);
}


void RDA1846::setFrequency(double freq) {  //Frequency in MHz Decimal okay. Automatically sets band.
  union {
    uint16_t w[2];
    uint32_t l;
  } datau;
  
  datau.l = freq * 8000;  // Freq(MHz) * 1000 * 8
  
  writeWord(datau.w[1], REG_FREQ_H);
  writeWord(datau.w[0], REG_FREQ_L);
  
  uint16_t tmp = 0x0024;
  //readWord(&tmp, REG_RF_BAND);
  if((freq >= 134.0) && (freq <= 174.0)) {
    bitSet(tmp, 7);
	bitSet(tmp, 6);
  }
  if((freq >= 200.0) && (freq <= 260.0)) {
    bitSet(tmp, 7);
	bitClear(tmp, 6);
  }
  if((freq >= 400.0) && (freq <= 520.0)) {
    bitClear(tmp, 7);
	bitClear(tmp, 6);
  }
  writeWord(tmp, REG_RF_BAND);
}


void RDA1846::setMode(uint16_t mode) {
  writeWord(mode, REG_MODE);
}

uint16_t RDA1846::getMode(void) {
  uint16_t mode;
  
  readWord(&mode, REG_MODE);
  return(mode);
}

void RDA1846::setChannelMode(CHAN_MODE cmode) {
  uint16_t mode;
  
  mode = getMode();
  switch(cmode) {
    case MODE25KHz:
	  bitSet(mode, 12);
	  bitSet(mode, 13);
	  break;
	case MODE125KHz:
	  bitClear(mode, 12);
	  bitClear(mode, 13);
	  break;
	default:
	  break;
  }
  setMode(mode);
}

void RDA1846::setTxMode(bool cmode) {
  uint16_t mode;
  
  mode = getMode();
  if(cmode) {
    bitSet(mode, 6);
  } else {
    bitClear(mode, 6);
  }
  setMode(mode);
}

void RDA1846::setRxMode(bool cmode) {
  uint16_t mode;
  
  mode = getMode();
  if(cmode) {
    bitSet(mode, 5);
  } else {
    bitClear(mode, 5);
  }
  setMode(mode);
}

void RDA1846::setDeepSleep(bool cmode) {
  uint16_t mode;
  
  mode = getMode();
  if(cmode) {
    bitSet(mode, 2);
  } else {
    bitClear(mode, 2);
  }
  setMode(mode);
}

void RDA1846::txOn(void) {
  setMode(0x3046);
}

void RDA1846::txOff(void) {
  setMode(0x3006);
}

void RDA1846::rxOn(void) {
  setMode(0x3026);
}

void RDA1846::rxOff(void) {
 setMode(0x3006);
}

void RDA1846::setTxChannel(TX_VOICE_CHAN cmode) {
  uint16_t mode;
  
  readWord(&mode, REG_TXVOICE_CHAN);
  switch(cmode) {
    case MIC:
	  bitClear(mode, 15);
	  bitClear(mode, 14);
	  break;
	case INNERTONE2:
	  bitClear(mode, 15);
	  bitSet(mode, 14);
	  break;
	case GPIO1CODE:
	  bitSet(mode, 15);
	  bitClear(mode, 14);
	  break;
	default:
	  break;
  }
  writeWord(mode, REG_TXVOICE_CHAN);
}


void RDA1846::setPABias(BIAS_VOLTAGE volts) {
  uint16_t mode;
  
  readWord(&mode, REG_PABIAS);
  
  mode &= 0xFFFFFC00; 
  mode |= volts;
  
  writeWord(mode, REG_PABIAS);
}


void RDA1846::setSubaudio(uint16_t mode) {

  writeWord(mode, REG_SUBAUDIO);
}


void RDA1846::setSubaudioFreq(float freq) {
  uint16_t ctcss_freq;
  
  ctcss_freq = freq * 2.0 * 16000;
  
  writeWord(ctcss_freq, REG_CTCSS_FREQ);
}

void RDA1846::setCDCSScode(uint32_t code) {
  uint16_t upper, lower;
  
  upper = 0x00FF & (code >> 16);
  lower = 0x0000FFFF & code;
  
  writeWord(upper, REG_CTCSS_CODE_H);
  writeWord(lower, REG_CTCSS_CODE_L);
}

void RDA1846::setSquelch(bool sq) {
  uint16_t mode;
  
  readWord(&mode, REG_MODE);
  
  if(sq) {
    bitSet(mode, 3);
  } else {
    bitClear(mode, 3);
  }
  
  writeWord(mode, REG_MODE);
}

void RDA1846::setCTCSSsel(bool sel) {
  uint16_t val;
  
  readWord(&val, REG_SUBAUDIO);
  if(sel) {
    bitSet(val, 3);
  } else {
    bitClear(val, 3);
  }
   
  writeWord(val, REG_SUBAUDIO);
}

void RDA1846::setCTCSSdet(bool det) {
  uint16_t val;
  
  readWord(&val, REG_SUBAUDIO);
  if(det) {
    bitSet(val, 10);
  } else {
    bitClear(val, 10);
  }
  writeWord(val, REG_SUBAUDIO);
}

void RDA1846::setSquelchThreshold(uint16_t open, uint16_t shut) {

  writeWord((open<<3), REG_SQ_H);
  writeWord((shut<<3), REG_SQ_L);
}

void RDA1846::setSquelchThresholddBm(int16_t open, int16_t shut) {
  uint16_t opendBm, shutdBm;
  
  opendBm = ((135 + open) & 0x0003FFFF);
  shutdBm = ((135 + shut) & 0x0003FFFF);
  
  setSquelchThreshold(opendBm, shutdBm);
}

void RDA1846::setSquelchout(bool sq) {
  uint16_t mode;
  
  readWord(&mode, REG_SQOUT_SEL);
  
  if(sq) {
    bitSet(mode, 7);
  } else {
    bitClear(mode, 7);
  }
  
  writeWord(mode, REG_SQOUT_SEL);
}


void RDA1846::setVOX(bool vox) {
  uint16_t mode;
  
  readWord(&mode, REG_MODE);
  
  if(vox) {
    bitSet(mode, 4);
  } else {
    bitClear(mode, 4);
  }
  
  writeWord(mode, REG_MODE);
}


void RDA1846::setVOXThresholdmV(uint16_t open, uint16_t shut) {
  uint16_t openVox, shutVox;
  
  openVox = ((225 * open) & 0xFFFF);
  shutVox = ((225 * shut) & 0xFFFF);
  
  writeWord(openVox, REG_VOX_H);
  writeWord(shutVox, REG_VOX_L);
}

void RDA1846::setTailNoiseElim(bool tn) {
uint16_t mode;
  
  readWord(&mode, REG_MODE);
  
  if(tn) {
    bitSet(mode, 11);
  } else {
    bitClear(mode, 11);
  }
  
  writeWord(mode, REG_MODE);
}

void RDA1846::setTailNoisePhase(CTCSSPHASE phase) {
  uint16_t val;
  
  readWord(&val, REG_SUBAUDIO);
  switch(phase) {
    case SHIFT120DEGREES:		// *** TO DO ***
	case SHIFT180DEGREES:
	case SHIFT240DEGREES:
	default:
	;
  }
  
  writeWord(val, REG_SUBAUDIO);
}

void RDA1846::setDTMFMode(DTMF_MODE mode) {
  uint16_t val;
  
  readWord(&val, REG_DTMF);
  val &= 0xFCFF;
  val |= mode << 8;
  writeWord(val, REG_DTMF);
}

void RDA1846::setDTMFToneTime(uint8_t t) {
  uint16_t val;
  
  t &= 0x0F;
  readWord(&val, REG_DTMF);
  val &= 0xFF0F;
  val |= t << 4;
  writeWord(val, REG_DTMF);
}

void RDA1846::setDTMFIdleTime(uint8_t t) {
  uint16_t val;
  
  t &= 0x0F;
  readWord(&val, REG_DTMF);
  val &= 0xFFF0;
  val |= t;
  writeWord(val, REG_DTMF);
}  

void RDA1846::setDTMFTone1Freq(uint16_t freq) {

  freq = freq * 4096;
  
  writeWord(freq, REG_TONE1_FREQ);
}

void RDA1846::setDTMFTone2Freq(uint16_t freq) {

  freq = freq * 4096;
  
  writeWord(freq, REG_TONE2_FREQ);
}

void RDA1846::setDTMFIdle(bool i) {
  uint16_t val;
  
  readWord(&val, REG_FLAG);
  if(i) {
    bitSet(val, 12);
  } else {
    bitClear(val, 12);
  }
  
  writeWord(val, REG_FLAG);
} 

void RDA1846::setDTMFClk(DTMF_CLOCK clk) {

  switch(clk) {
    case CLK12POINT8MHZ:
	case CLK25POINT6MHZ:
	  writeWord((uint16_t)0x615D, REG_DTMF_C0_C1);
	  writeWord(0x534D, REG_DTMF_C2_C3);
	  writeWord(0x2C1E, REG_DTMF_C4_C5);
	  writeWord(0x0AF6, REG_DTMF_C6_C7);
	  break;
	case CLK13MHZ:
	case CLK26MHZ:
	default:
	  writeWord(0x615E, REG_DTMF_C0_C1);
	  writeWord(0x574D, REG_DTMF_C2_C3);
	  writeWord(0x311E, REG_DTMF_C4_C5);
	  writeWord(0x0FFD, REG_DTMF_C6_C7);
	  break;
  }
}

void RDA1846::setTxFMDeviation(uint8_t dev) {
  uint16_t currdev;
  
  readWord(&currdev, REG_TX_DEVIATION);
  dev &= 0x7F;
  currdev &= 0xE03F;
  currdev |= dev << 6;
  writeWord(currdev, REG_TX_DEVIATION);
}


void RDA1846::setTxCTCSSDeviation(uint8_t dev) {
  uint16_t currdev;
  
  readWord(&currdev, REG_TX_DEVIATION);
  dev &= 0x3F;
  currdev &= 0xFFC0;
  currdev |= dev;
  writeWord(currdev, REG_TX_DEVIATION);
}


void RDA1846::setRxAnalogDACGain(uint8_t range) {
  uint16_t currrange;
  
  readWord(&currrange, REG_RX_VOL);
  range &= 0x0F;
  currrange &= 0xFF0F;
  currrange |= range << 4;
  writeWord(currrange, REG_RX_VOL);
}

void RDA1846::setRxDigitalVoiceGain(uint8_t range) {
  uint16_t currrange;
  
  readWord(&currrange, REG_RX_VOL);
  range &= 0x0F;
  currrange &= 0xFFF0;
  currrange |= range;
  writeWord(currrange, REG_RX_VOL);
}

void RDA1846::setGPIO7(GPIO7 mode) {
  uint16_t val;
  
  readWord(&val, REG_GPIO);
  switch(mode) {
    case GP7_HIZ:
	  bitClear(val, 1);
	  bitClear(val, 0);
	  break;
	case VOX:
	  bitClear(val, 1);
	  bitSet(val, 0);
	  break;
	case GP7_LOW:
	  bitSet(val, 1);
	  bitClear(val, 0);
	  break;
	case GP7_HIGH:
	  bitSet(val, 1);
	  bitSet(val, 0);
	  break;
	default:
	  break;
  }
  writeWord(val, REG_GPIO);
}

void RDA1846::setGPIO6(GPIO6 mode) {
  uint16_t val;
  
  readWord(&val, REG_GPIO);
  switch(mode) {
    case GP6_HIZ:
	  bitClear(val, 1);
	  bitClear(val, 0);
	  break;
	case SQ:
	  bitClear(val, 1);
	  bitSet(val, 0);
	  break;
	case GP6_LOW:
	  bitSet(val, 1);
	  bitClear(val, 0);
	  break;
	case GP6_HIGH:
	  bitSet(val, 1);
	  bitSet(val, 0);
	  break;
	default:
	  break;
  }
  writeWord(val, REG_GPIO);
}

void RDA1846::setGPIO5(GPIO5 mode) {
  uint16_t val;
  
  readWord(&val, REG_GPIO);
  switch(mode) {
    case GP5_HIZ:
	  bitClear(val, 1);
	  bitClear(val, 0);
	  break;
	case TXON_RF:
	  bitClear(val, 1);
	  bitSet(val, 0);
	  break;
	case GP5_LOW:
	  bitSet(val, 1);
	  bitClear(val, 0);
	  break;
	case GP5_HIGH:
	  bitSet(val, 1);
	  bitSet(val, 0);
	  break;
	default:
	  break;
  }
  writeWord(val, REG_GPIO);
}

void RDA1846::setGPIO4(GPIO4 mode) {
  uint16_t val;
  
  readWord(&val, REG_GPIO);
  switch(mode) {
    case GP4_HIZ:
	  bitClear(val, 1);
	  bitClear(val, 0);
	  break;
	case RXON_RF:
	  bitClear(val, 1);
	  bitSet(val, 0);
	  break;
	case GP4_LOW:
	  bitSet(val, 1);
	  bitClear(val, 0);
	  break;
	case GP4_HIGH:
	  bitSet(val, 1);
	  bitSet(val, 0);
	  break;
	default:
	  break;
  }
  writeWord(val, REG_GPIO);
}

void RDA1846::setGPIO3(GPIO3 mode) {
  uint16_t val;
  
  readWord(&val, REG_GPIO);
  switch(mode) {
    case GP3_HIZ:
	  bitClear(val, 1);
	  bitClear(val, 0);
	  break;
	case SDO:
	  bitClear(val, 1);
	  bitSet(val, 0);
	  break;
	case GP3_LOW:
	  bitSet(val, 1);
	  bitClear(val, 0);
	  break;
	case GP3_HIGH:
	  bitSet(val, 1);
	  bitSet(val, 0);
	  break;
	default:
	  break;
  }
  writeWord(val, REG_GPIO);
}

void RDA1846::setGPIO2(GPIO2 mode) {
  uint16_t val;
  
  readWord(&val, REG_GPIO);
  switch(mode) {
    case GP2_HIZ:
	  bitClear(val, 1);
	  bitClear(val, 0);
	  break;
	case GP2_INT:
	  bitClear(val, 1);
	  bitSet(val, 0);
	  break;
	case GP2_LOW:
	  bitSet(val, 1);
	  bitClear(val, 0);
	  break;
	case GP2_HIGH:
	  bitSet(val, 1);
	  bitSet(val, 0);
	  break;
	default:
	  break;
  }
  writeWord(val, REG_GPIO);
}

void RDA1846::setGPIO1(GPIO1 mode) {
  uint16_t val;
  
  readWord(&val, REG_GPIO);
  switch(mode) {
    case GP1_HIZ:
	  bitClear(val, 1);
	  bitClear(val, 0);
	  break;
	case CODE_IO:
	  bitClear(val, 1);
	  bitSet(val, 0);
	  break;
	case GP1_LOW:
	  bitSet(val, 1);
	  bitClear(val, 0);
	  break;
	case GP1_HIGH:
	  bitSet(val, 1);
	  bitSet(val, 0);
	  break;
	default:
	  break;
  }
  writeWord(val, REG_GPIO);
}

void RDA1846::setGPIO0(GPIO0 mode) {
  uint16_t val;
  
  readWord(&val, REG_GPIO);
  switch(mode) {
    case GP0_HIZ:
	  bitClear(val, 1);
	  bitClear(val, 0);
	  break;
	case CSS_IOC:
	  bitClear(val, 1);
	  bitSet(val, 0);
	  break;
	case GP0_LOW:
	  bitSet(val, 1);
	  bitClear(val, 0);
	  break;
	case GP0_HIGH:
	  bitSet(val, 1);
	  bitSet(val, 0);
	  break;
	default:
	  break;
  }
  writeWord(val, REG_GPIO);
}

void RDA1846::setIntMode(uint16_t mode) {
  writeWord(mode, REG_INT);
}

void RDA1846::setSTMode(STMODE mode) {
  uint16_t val;
  
  readWord(&val, REG_MODE);
  val &= 0xFCFF;
  val |= mode << 8;
  writeWord(val, REG_MODE);
}

void RDA1846::setEmphasis(bool emp) {
  uint16_t val;
  
  readWord(&val, REG_VOICE_FILT);
  if(emp) {
    bitSet(val, 3);
  } else {
    bitClear(val, 3);
  }
  writeWord(val, REG_VOICE_FILT);
}

uint16_t RDA1846::getRSSI(void) {
  uint16_t val;
  
  readWord(&val, REG_RSSI);
  return (val & 0x03FF);
}

uint16_t RDA1846::getVSSI(void) {
  uint16_t val;
  
  readWord(&val, REG_VSSI);
  return (val & 0x07FF);
}

uint8_t RDA1846::getDTMFIndex(void) {
  uint16_t val;
  
  readWord(&val, REG_DTMF_DET);
  return ((val & 0x07E0)>>5);
}

uint8_t RDA1846::getDTMFCode(void) {
  uint16_t val;
  
  readWord(&val, REG_DTMF_DET);
  return (val & 0x000F);
}

uint16_t RDA1846::getFlags(void) {
  uint16_t val;
  
  readWord(&val, REG_FLAG);
  return (val);
}

uint16_t RDA1846::getChipID(void) {
  uint16_t val;
  
  readWord(&val, REG_CHIP_ID);
  return (val);
}

void RDA1846::reset(void) {
  writeWord(0x0001, REG_MODE);
  writeWord(0x0000, REG_MODE);
}
