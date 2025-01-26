//==============================================================================
//AUTORE        :    Oscilloscopio.it
//NOME PROJ     :    CV Keychain
//HW            :    STC8G1K08-SSOP20
//COMPILATORE   :    SDCC
//REVISIONI     :    BETA - 15/12/2024 -  prima release
//==============================================================================

#include "fw_reg_stc8g.h"

//==============================================================================
//ASSEGNAZIONE DEI PIN DEL micro e configurazioni
//==============================================================================


#define DCC1 P14
#define DCC2 P15
#define LED P13                                                                 
#define ENA P34

#define lncv_number         10                                                  // number of lncv implemented in the decoder
#define ARTNR             1111                                                  //article number during programming (11111)

unsigned char speed, j, i, dat, timeco, tmp, time_count, adc_count;
volatile unsigned int millis, adc0, adcmax;
volatile unsigned char railgo, railwin, railbit, railpoint, dcc_idle, ack;

unsigned char dcc_state, dcc_gen, ledco;
unsigned char dcc_off, railcom;
unsigned char preamble, preamble_max;
unsigned char dcc_byte, dcc_bit;
unsigned char dcc_tx, dcc_rx, dcc_rx1;
unsigned char dcc_len, dcc_mask;
unsigned char id2, cv1, progbit, lack1, lack2, pomstb, lnlen;
unsigned char tx_point, tx_point2, rx_point, rx_point2, rx_data, chksum;
__XDATA unsigned char maindcc, retry;
__XDATA unsigned int pom_addr, pom_cvadd, tmpi; 
__XDATA unsigned char pom_cvdata, pom_mode, pom_go, pom_stat;
__XDATA unsigned char lndecode, index;
__XDATA unsigned int article, lncv_address, lncv_data, val1, val2;
__XDATA unsigned flag, progmode, val3;
__XDATA unsigned char dcc[256];
__XDATA unsigned char rail[16];
__XDATA unsigned char uart_rx[256];
__XDATA unsigned char uart_tx[256];
__XDATA unsigned char lnrx[16];
__XDATA unsigned int lncv[lncv_number];										    //appoggio per la programmazione delle CV
__XDATA unsigned char acklen, respre, respost, prognum, pomnum, retrylim;
__XDATA unsigned int currthr, modaddr;

__CODE const unsigned int lncvdef [lncv_number]=
{
 1,                                                                             //LNCV0 = modaddr (1)
 60,                                                                            //LNCV1 = currthr (60 = 60mA)
 40,                                                                            //LNCV2 = acklen (6ms = 66)
 3,                                                                             //LNCV3 = respre # (3-20)
 5,                                                                             //LNCV4 = prognum # (5-10)
 6,                                                                             //LNCV5 = respost # (6-20)
 5,                                                                             //LNCV6 = pomnum # (5-10)
 3,                                                                             //LNCV7 = retrylim # (3-10)
 1,                                                                             //LNCV8 = pomstrobe
 0                                                                              //LNCV9 = spare
};
            
__CODE const unsigned char codez[256] = {  
0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0x40,//0x40 = NACK
0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0x33,0xFF,0xFF,0xFF,0x34,0xFF,0x35,0x36,0xFF,
0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0x3A,0xFF,0xFF,0xFF,0x3B,0xFF,0x3C,0x37,0xFF,
0xFF,0xFF,0xFF,0x3F,0xFF,0x3D,0x38,0xFF,0xFF,0x3E,0x39,0xFF,0xFF,0xFF,0xFF,0xFF,
0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0x24,0xFF,0xFF,0xFF,0x23,0xFF,0x22,0x21,0xFF,
0xFF,0xFF,0xFF,0x1F,0xFF,0x1E,0x20,0xFF,0xFF,0x1D,0x1C,0xFF,0x1B,0xFF,0xFF,0xFF,
0xFF,0xFF,0xFF,0x19,0xFF,0x18,0x1A,0xFF,0xFF,0x17,0x16,0xFF,0x15,0xFF,0xFF,0xFF,
0xFF,0x25,0x14,0xFF,0x13,0xFF,0xFF,0xFF,0x32,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,
0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0x0E,0xFF,0x0D,0x0C,0xFF,
0xFF,0xFF,0xFF,0x0A,0xFF,0x09,0x0B,0xFF,0xFF,0x08,0x07,0xFF,0x06,0xFF,0xFF,0xFF,
0xFF,0xFF,0xFF,0x04,0xFF,0x03,0x05,0xFF,0xFF,0x02,0x01,0xFF,0x00,0xFF,0xFF,0xFF,
0xFF,0x0F,0x10,0xFF,0x11,0xFF,0xFF,0xFF,0x12,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,
0xFF,0xFF,0xFF,0xFF,0xFF,0x2B,0x30,0xFF,0xFF,0x2A,0x2F,0xFF,0x31,0xFF,0xFF,0xFF,
0xFF,0x29,0x2E,0xFF,0x2D,0xFF,0xFF,0xFF,0x2C,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,
0x42,0xFF,0x28,0xFF,0x27,0xFF,0xFF,0xFF,0x26,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,//0x42 = BUSY
0x41,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF};//0x41 = ACK

//==============================================================================
// PROCEDURA DI INTERRUPT
// genera il segnale dcc ogni 30 us
//==============================================================================

INTERRUPT (Timer0_Isr, 1)
{
//  LED=1;
  TF0=0;                                                            	        //Resetta il FLAG di overflow
  
//==============================================================================
// Controllo del DCC
//==============================================================================
  switch (dcc_state) {
      case 0:
        DCC1 = dcc_off | railbit;
        DCC2 = 1;
        if ((railcom==1) && (preamble==0)) {
          railbit=1;
        }  
        dcc_state=1;
      break;
      case 1:
        DCC1 = dcc_off | railbit;
        DCC2 = 1;
        switch (dcc_gen) {
          case 0:
            preamble++;
            if (preamble!=preamble_max) {
              dcc_bit=1; 
              dcc_gen=0;
            } else {
              dcc_bit=0;
              dcc_mask=0x80;
              if (dcc_tx==dcc_rx) {
                dcc_len=3;
                dcc_byte=255;
                dcc_gen=2;
                dcc_idle=1;
              } else {
                dcc_len=dcc[dcc_tx];
                dcc_tx++;
                dcc_byte=dcc[dcc_tx];
                dcc_tx++;
                dcc_gen=1;              
              }
            } 
          break; 
          case 1:                                                               //sending the packet
            if (dcc_mask==0) {
              dcc_len--;
              if (dcc_len==0) {
                dcc_bit=1;
                preamble=0;
                dcc_gen=0;
              } else {
                dcc_bit=0;
                dcc_mask=0x80;
                dcc_byte=dcc[dcc_tx];
                dcc_tx++;
                dcc_gen=1;               
              }  
            } else {
              dcc_bit=dcc_byte & dcc_mask;
              dcc_mask >>= 1;            
              dcc_gen=1;
            }              
          break;            
          case 2:                                                               //sending the IDLE
            if (dcc_mask==0) {
              dcc_len--;
              if (dcc_len==0) {
                dcc_bit=1;
                preamble=0;
                dcc_gen=0;
              } else {
                dcc_bit=0;
                dcc_mask=0x80;
                if (dcc_len==2) dcc_byte=0; else dcc_byte=0xFF; 
                dcc_gen=2;               
              }  
            } else {
              dcc_bit=dcc_byte & dcc_mask;
              dcc_mask >>= 1;
              dcc_gen=2;
            }              
          break;            
        }
        if (dcc_bit!=0) dcc_state=2; else dcc_state=4;
      break;
      case 2:
        DCC1 = 1;
        DCC2 = dcc_off | railbit;
        dcc_state=3;
      break;
      case 3:
        DCC1 = 1;
        DCC2 = dcc_off | railbit;
        if ((railcom==1) && (preamble==4)) railbit=0;
        //if (preamble==4) LED=0;
        dcc_state=0;
      break;
      case 4:
        DCC1 = dcc_off;
        DCC2 = 1;
        dcc_state=5;
      break;
      case 5:
        DCC1 = dcc_off;
        DCC2 = 1;
        dcc_state=6;
      break;
      case 6:
        DCC1 = 1;
        DCC2 = dcc_off;
        dcc_state=7;
      break;
      case 7:
        DCC1 = 1;
        DCC2 = dcc_off;
        dcc_state=8;
      break;
      case 8:
        DCC1 = 1;
        DCC2 = dcc_off;
        dcc_state=9;
      break;
      case 9:
        DCC1 = 1;
        DCC2 = dcc_off;
        dcc_state=0;
      break;
  }

//==============================================================================
// Controllo RAILCOM
//==============================================================================
  
  if ((railwin==1) && (SCON & 0x01) && (railgo==0)) {
    //LED=1;
    rail[railpoint]=SBUF;
    if (railpoint!=15) railpoint++;
    SCON &= 0xFE;
    //LED=0;
  }
  
  if ((railbit==0) && (railwin==1) && (railgo==0)) {                            //negative edge of railwin
    if (railpoint!=0) {
      railgo=1;
      //LED=1;
    }  
  }
  
  if ((railbit==1) && (railwin==0) && (railgo==0)) {                            //positive edge of railwin
    railpoint=0;
    SCON &= 0xFE;  
  }
   
  railwin = railbit; 
  
//==============================================================================
// Controllo della UART
// ogni 30us si controllano i caratteri ricevuti dalla UART
//==============================================================================

  if (S2CON & 0x01)                                                             //se e' arrivato qualcosa da UART RX
  {
    uart_rx[rx_point]=S2BUF;
    S2CON &= 0xFE;
    rx_point++;
  }
  if ((tx_point!=tx_point2) && (S2CON & 0x02)) {                                //se abbiamo qualcosa da trasmettere e UART TX e' libera
    S2CON &= 0xFD;
    S2BUF=uart_tx[tx_point];
    tx_point++;
  }  
  
//==============================================================================
// Controllo del tempo
// la variabile millis contiene i millisecondi passati
//==============================================================================

  time_count++;
  if (time_count==33) {
    time_count=0;
    millis++;
  }
  
//==============================================================================
// Controllo A/D converter
//==============================================================================

  adc_count++;
  if (adc_count==3) {                                                           //1 conversion each 90us
    adc_count=0;
    adc0 = ADC_RESL+(ADC_RES << 8);
    ADC_CONTR |= 0x40;
    if ((adc0>currthr) && (ack!=255)) {
      ack++;                                     //in multiple of 90us
      //LED=1;
    } //else LED=0;
//    if (adc0>adcmax) adcmax=adc0;
  }
  
//  LED=0;
    
}

//==============================================================================
// EEPROM management routines
//==============================================================================

void EEPROM_Idlex(void)
{
    IAP_CONTR = 0;                                                              //bank normal
    IAP_CMD = 0;                                                                //no command
    IAP_TRIG = 0;                                                               //no start
    IAP_ADDRH = 0;                                                              //address=0
    IAP_ADDRL = 0;
}

char EEPROM_readx(char addr)
{

    IAP_CONTR = 0x80;                                                           //bank eeprom
    IAP_TPS = 24;                                                               //processor at 24MHz
    IAP_CMD = 1;                                                                //READ
    IAP_ADDRL = addr;                                                           //only use addr<256
    IAP_ADDRH = 0;                      				                        //
    IAP_TRIG = 0x5a;                                                            //trigger sequence
    IAP_TRIG = 0xa5;                                                            //trigger sequence
    NOP();                                                                      //wait here
    dat = IAP_DATA;                                                             //read data
    EEPROM_Idlex();                                                             //eeprom idle
    return dat;                                                                 //data from eeprom in output
}

void EEPROM_writex(char addr, char dat)
{
    IAP_CONTR = 0x80;                                                           //bank eeprom
    IAP_TPS = 24;                                                               //24MHz clock
    IAP_CMD = 2;                                                                //write command
    IAP_ADDRL = addr;                                                           //we want to write at this address
    IAP_ADDRH = 0;                      				
    IAP_DATA = dat;                                                             //this data
    IAP_TRIG = 0x5a;                            
    IAP_TRIG = 0xa5;                            
    NOP();
    EEPROM_Idlex();                                   
}

void EEPROM_erasex(char addr)
{
    IAP_CONTR = 0x80;                           
    IAP_TPS = 24;                                                               //24MHz clock
    IAP_CMD = 3;                                                                //Erase command
    IAP_ADDRL = addr;                                                           //any address in page, we use only first page (512 bytes)
    IAP_ADDRH = 0;                      				
    IAP_TRIG = 0x5a;                            
    IAP_TRIG = 0xa5;                            
    NOP();
    EEPROM_Idlex();                                  
}


//==============================================================================
//Store changed LNCV
//==============================================================================

void lncv_update(void)
{

  EEPROM_erasex(0);
  for (i=0;i<lncv_number;i++)
  {
    j = i << 1;
    tmp = lncv[i] & 0x00FF;
    EEPROM_writex(j,tmp);
    tmp = (lncv[i] & 0xFF00) >> 8;
    EEPROM_writex(j+1,tmp);
  }
}


//==============================================================================
// LNCV reset at default value
//==============================================================================

void lncv_default (void)
{
  
  EEPROM_erasex(0);                                                             // erase first 512 bytes
  for (i=0;i<lncv_number;i++)
  {
    j = i << 1;
    tmp = lncvdef[i] & 0x00FF;
    EEPROM_writex(j,tmp);
    tmp = (lncvdef[i] & 0xFF00) >> 8;
    EEPROM_writex(j+1,tmp);
  }
}

//==============================================================================
// LNCV read at startup
//==============================================================================

void lncv_init (void)
{

  i = EEPROM_readx(2);
  if (i==255) lncv_default();
  
  for (i=0;i<lncv_number;i++)
  {
    j = i << 1;
	tmpi = EEPROM_readx(j);
	tmpi = tmpi + (EEPROM_readx(j+1) << 8);
    lncv[i] = tmpi;
  }
  
  modaddr=lncv[0];
  //modaddr=1;
  
  currthr=lncv[1];
  if (currthr<40) currthr=40;
  if (currthr>200) currthr=200;
  //currthr=60;
    
  acklen=lncv[2];
  if (acklen<40) acklen=40;
  if (acklen>100) acklen=100;
  //acklen=40;
  
  respre=lncv[3];
  if (respre<3) respre=3;
  if (respre>12) respre=12;
  //respre=3;
  
  prognum=lncv[4];
  if (prognum<5) prognum=5;
  if (prognum>20) prognum=20;
  //prognum=5;
  
  respost=lncv[5];
  if (respost<6) respost=6;
  if (respost>24) respost=24;
  //respost=6;
  
  pomnum=lncv[6];
  if (pomnum<5) pomnum=5;
  if (pomnum>20) pomnum=20;
  //pomnum=6;
  
  retrylim=lncv[7];
  if (retrylim<3) retrylim=3;
  if (retrylim>10) retrylim=10;
  //retrylim=3;  

  pomstb=lncv[8];
  if (pomstb<1) pomstb=1;
  if (pomstb>20) pomstb=20;
  //pomstb=1;  

}

//==============================================================================
// Delay in milliseconds
//==============================================================================

void delay_ms(unsigned int ms)
{
  time_count=0;
  millis=0;
  while (millis != ms) {
  };
}


void servicemsg(void)
{
  chksum=0xFF;
  uart_tx[tx_point2]=0xE7;
  chksum ^= uart_tx[tx_point2];                                                  
  tx_point2++;
  uart_tx[tx_point2]=0x0E;
  chksum ^= uart_tx[tx_point2];                                                  
  tx_point2++;
  uart_tx[tx_point2]=0x55;     
  chksum ^= uart_tx[tx_point2];                                             
  tx_point2++;
  uart_tx[tx_point2]=rail[0];     
  chksum ^= uart_tx[tx_point2];                                             
  tx_point2++;
  uart_tx[tx_point2]=rail[1];     
  chksum ^= uart_tx[tx_point2];                                             
  tx_point2++;
  uart_tx[tx_point2]=rail[2];     
  chksum ^= uart_tx[tx_point2];                                             
  tx_point2++;
  uart_tx[tx_point2]=rail[3];     
  chksum ^= uart_tx[tx_point2];                                             
  tx_point2++;
  uart_tx[tx_point2]=rail[4];     
  chksum ^= uart_tx[tx_point2];                                             
  tx_point2++;
  uart_tx[tx_point2]=rail[5];     
  chksum ^= uart_tx[tx_point2];                                             
  tx_point2++;
  uart_tx[tx_point2]=rail[6];     
  chksum ^= uart_tx[tx_point2];                                             
  tx_point2++;
  uart_tx[tx_point2]=rail[7];     
  chksum ^= uart_tx[tx_point2];                                             
  tx_point2++;
  uart_tx[tx_point2]=timeco & 0x3F;     
  chksum ^= uart_tx[tx_point2];                                             
  tx_point2++;
  uart_tx[tx_point2]=railpoint;     
  chksum ^= uart_tx[tx_point2];                                             
  tx_point2++;
  uart_tx[tx_point2]=chksum;                                                  
  tx_point2++;
}


void answer_prog(void)
{
  chksum=0xFF;
  uart_tx[tx_point2]=0xE7;
  chksum ^= uart_tx[tx_point2];                                                  
  tx_point2++;
  uart_tx[tx_point2]=0x0E;
  chksum ^= uart_tx[tx_point2];                                                  
  tx_point2++;
  uart_tx[tx_point2]=0x7C;     
  chksum ^= uart_tx[tx_point2];                                             
  tx_point2++;
  if (pom_mode==4) uart_tx[tx_point2]=0x2B; else uart_tx[tx_point2]=0x6B;     
  chksum ^= uart_tx[tx_point2];                                             
  tx_point2++;
  uart_tx[tx_point2]=pom_stat;     
  chksum ^= uart_tx[tx_point2];                                             
  tx_point2++;
  uart_tx[tx_point2]=0x00;     
  chksum ^= uart_tx[tx_point2];                                             
  tx_point2++;
  uart_tx[tx_point2]=0x00;     
  chksum ^= uart_tx[tx_point2];                                             
  tx_point2++;
  uart_tx[tx_point2]=0x07;     
  chksum ^= uart_tx[tx_point2];                                             
  tx_point2++;

  uart_tx[tx_point2]=0x00;                                                      //CVH
  if (cv1 & 0x80) uart_tx[tx_point2]|=0x02;                                     //CVH
  if (pom_cvadd & 0x0080) uart_tx[tx_point2]|=0x01;                             //CVH
  if (pom_cvadd & 0x0100) uart_tx[tx_point2]|=0x10;                             //CVH
  if (pom_cvadd & 0x0200) uart_tx[tx_point2]|=0x20;                             //CVH
  chksum ^= uart_tx[tx_point2];                                                  
  tx_point2++;
  uart_tx[tx_point2]=pom_cvadd & 0x7F;                                          //CVL
  chksum ^= uart_tx[tx_point2];                                                  
  tx_point2++;
  uart_tx[tx_point2]=cv1 & 0x7F;                                                //DATA7
  chksum ^= uart_tx[tx_point2];                                                  
  tx_point2++;

  uart_tx[tx_point2]=0x7F;     
  chksum ^= uart_tx[tx_point2];                                             
  tx_point2++;
  uart_tx[tx_point2]=0x7F;     
  chksum ^= uart_tx[tx_point2];                                             
  tx_point2++;
  uart_tx[tx_point2]=chksum;                                                  
  tx_point2++;
}

/*
void answer_slot(void)
{   
  chksum=0xFF;
  uart_tx[tx_point2]=0xE7;
  chksum ^= uart_tx[tx_point2];                                                 //type of packet 
  tx_point2++;
  tx_point2 &= 0x3F;
  uart_tx[tx_point2]=0x0E;                                                      //packet lenght = 14                                        
  chksum ^= uart_tx[tx_point2];                                                  
  tx_point2++;
  tx_point2 &= 0x3F;
  
  uart_tx[tx_point2]=slot[0];                                                   //slot #
  chksum ^= uart_tx[tx_point2];                                                  
  tx_point2++;
  tx_point2 &= 0x3F;
  uart_tx[tx_point2]=slot[1];                                                   // STAT1
  chksum ^= uart_tx[tx_point2];                                                  
  tx_point2++;
  tx_point2 &= 0x3F;
  uart_tx[tx_point2]=slot[2];                                                   // ADDR LOW
  chksum ^= uart_tx[tx_point2];                                                  
  tx_point2++;
  tx_point2 &= 0x3F;
  uart_tx[tx_point2]=slot[3];                                                   // SPEED
  chksum ^= uart_tx[tx_point2];                                                  
  tx_point2++;
  tx_point2 &= 0x3F;
  uart_tx[tx_point2]=slot[4];                                                   // DIRF
  chksum ^= uart_tx[tx_point2];                                                  
  tx_point2++;
  tx_point2 &= 0x3F;
  uart_tx[tx_point2]=slot[5];                                                   // TRK
  chksum ^= uart_tx[tx_point2];                                                  
  tx_point2++;
  tx_point2 &= 0x3F;
  uart_tx[tx_point2]=slot[6];                                                   // STAT2
  chksum ^= uart_tx[tx_point2];                                                  
  tx_point2++;
  tx_point2 &= 0x3F;
  uart_tx[tx_point2]=slot[7];                                                   // ADDR HIGH
  chksum ^= uart_tx[tx_point2];                                                  
  tx_point2++;
  tx_point2 &= 0x3F;
  uart_tx[tx_point2]=slot[8];                                                   // SOUND
  chksum ^= uart_tx[tx_point2];                                                  
  tx_point2++;
  tx_point2 &= 0x3F;

  uart_tx[tx_point2]=0x7F;                                                      //THROTTLE IDH
  chksum ^= uart_tx[tx_point2];                                                  
  tx_point2++;
  tx_point2 &= 0x3F;
  uart_tx[tx_point2]=0x7F;                                                      //THROTTLE IDL
  chksum ^= uart_tx[tx_point2];                                                  
  tx_point2++;
  tx_point2 &= 0x3F;
  uart_tx[tx_point2]=(chksum & 0x7F);                                           //CHKSUM       
  tx_point2++;
  tx_point2 &= 0x3F;
}

void answer_slot2(void)
{   
  chksum=0xFF;
  uart_tx[tx_point2]=0xE7;
  chksum ^= uart_tx[tx_point2];                                                 //type of packet 
  tx_point2++;
  tx_point2 &= 0x3F;
  uart_tx[tx_point2]=0x0E;                                                      //packet lenght = 14                                        
  chksum ^= uart_tx[tx_point2];                                                  
  tx_point2++;
  tx_point2 &= 0x3F;
  
  uart_tx[tx_point2]=lnrx[0];                                                   //slot #
  chksum ^= uart_tx[tx_point2];                                                  
  tx_point2++;
  tx_point2 &= 0x3F;
  uart_tx[tx_point2]=0;                                                   // STAT1
  //chksum ^= uart_tx[tx_point2];                                                  
  tx_point2++;
  tx_point2 &= 0x3F;
  uart_tx[tx_point2]=0;                                                   // ADDR LOW
  //chksum ^= uart_tx[tx_point2];                                                  
  tx_point2++;
  tx_point2 &= 0x3F;
  uart_tx[tx_point2]=0;                                                   // SPEED
  //chksum ^= uart_tx[tx_point2];                                                  
  tx_point2++;
  tx_point2 &= 0x3F;
  uart_tx[tx_point2]=0;                                                   // DIRF
  //chksum ^= uart_tx[tx_point2];                                                  
  tx_point2++;
  tx_point2 &= 0x3F;
  uart_tx[tx_point2]=0x07;                                                   // TRK
  chksum ^= uart_tx[tx_point2];                                                  
  tx_point2++;
  tx_point2 &= 0x3F;
  uart_tx[tx_point2]=0;                                                   // STAT2
  //chksum ^= uart_tx[tx_point2];                                                  
  tx_point2++;
  tx_point2 &= 0x3F;
  uart_tx[tx_point2]=0;                                                   // ADDR HIGH
  //chksum ^= uart_tx[tx_point2];                                                  
  tx_point2++;
  tx_point2 &= 0x3F;
  uart_tx[tx_point2]=0;                                                   // SOUND
  //chksum ^= uart_tx[tx_point2];                                                  
  tx_point2++;
  tx_point2 &= 0x3F;

  uart_tx[tx_point2]=0x7F;                                                      //THROTTLE IDH
  chksum ^= uart_tx[tx_point2];                                                  
  tx_point2++;
  tx_point2 &= 0x3F;
  uart_tx[tx_point2]=0x7F;                                                      //THROTTLE IDL
  chksum ^= uart_tx[tx_point2];                                                  
  tx_point2++;
  tx_point2 &= 0x3F;
  uart_tx[tx_point2]=(chksum & 0x7F);                                           //CHKSUM       
  tx_point2++;
  tx_point2 &= 0x3F;
}
*/

void answer_pom(void)
{
  chksum=0xFF;
  uart_tx[tx_point2]=0xE7;
  chksum ^= uart_tx[tx_point2];                                                 //type of packet 
  tx_point2++;
  uart_tx[tx_point2]=0x0E;                                                      //packet lenght                                         
  chksum ^= uart_tx[tx_point2];                                                  
  tx_point2++;
  uart_tx[tx_point2]=0x7C;                                                      //slot #124
  chksum ^= uart_tx[tx_point2];                                                  
  tx_point2++;
  uart_tx[tx_point2]=val3;                                                      //PCMD 0x2F or 0x67
  chksum ^= uart_tx[tx_point2];                                                  
  tx_point2++;
  uart_tx[tx_point2]=0x00;                                                      //PSTAT
  chksum ^= uart_tx[tx_point2];                                                  
  tx_point2++;
  uart_tx[tx_point2]=(pom_addr >> 7) & 0x7F;                                    //HOPSA
  chksum ^= uart_tx[tx_point2];                                                  
  tx_point2++;
  uart_tx[tx_point2]=pom_addr & 0x7F;                                           //LOPSA
  chksum ^= uart_tx[tx_point2];                                                  
  tx_point2++;
  uart_tx[tx_point2]=0x07;                                                      //TRK
  chksum ^= uart_tx[tx_point2];                                                  
  tx_point2++;
  uart_tx[tx_point2]=0x00;                                                      //CVH
  if (cv1 & 0x80) uart_tx[tx_point2]|=0x02;                                     //CVH
  if (pom_cvadd & 0x0080) uart_tx[tx_point2]|=0x01;                             //CVH
  if (pom_cvadd & 0x0100) uart_tx[tx_point2]|=0x10;                             //CVH
  if (pom_cvadd & 0x0200) uart_tx[tx_point2]|=0x20;                             //CVH
  chksum ^= uart_tx[tx_point2];                                                  
  tx_point2++;
  uart_tx[tx_point2]=pom_cvadd & 0x7F;                                          //CVL
  chksum ^= uart_tx[tx_point2];                                                  
  tx_point2++;
  uart_tx[tx_point2]=cv1 & 0x7F;                                                //DATA7
  chksum ^= uart_tx[tx_point2];                                                  
  tx_point2++;
  uart_tx[tx_point2]=0x7F;                                                      //THROTTLE IDH
  chksum ^= uart_tx[tx_point2];                                                  
  tx_point2++;
  uart_tx[tx_point2]=0x7F;                                                      //THROTTLE IDL
  chksum ^= uart_tx[tx_point2];                                                  
  tx_point2++;
  uart_tx[tx_point2]=(chksum & 0x7F);                                           //CHKSUM       
  tx_point2++;
}

/*
void answer_pom2(void)
{
  chksum=0xFF;
  uart_tx[tx_point2]=0xE7;
  chksum ^= uart_tx[tx_point2];                                                 //type of packet 
  tx_point2++;
  uart_tx[tx_point2]=0x0E;                                                      //packet lenght                                         
  chksum ^= uart_tx[tx_point2];                                                  
  tx_point2++;
  uart_tx[tx_point2]=0x7C;                                                      //slot #124
  chksum ^= uart_tx[tx_point2];                                                  
  tx_point2++;
  uart_tx[tx_point2]=0x67;                                                      //PCMD
  chksum ^= uart_tx[tx_point2];                                                  
  tx_point2++;
  uart_tx[tx_point2]=0x00;                                                      //PSTAT
  chksum ^= uart_tx[tx_point2];                                                  
  tx_point2++;
  uart_tx[tx_point2]=(pom_addr >> 7) & 0x7F;                                    //HOPSA
  chksum ^= uart_tx[tx_point2];                                                  
  tx_point2++;
  uart_tx[tx_point2]=pom_addr & 0x7F;                                           //LOPSA
  chksum ^= uart_tx[tx_point2];                                                  
  tx_point2++;
  uart_tx[tx_point2]=0x07;                                                      //TRK
  chksum ^= uart_tx[tx_point2];                                                  
  tx_point2++;
  uart_tx[tx_point2]=0x00;                                                      //CVH
  if (cv1 & 0x80) uart_tx[tx_point2]|=0x02;                                     //CVH
  if (pom_cvadd & 0x0080) uart_tx[tx_point2]|=0x01;                             //CVH
  if (pom_cvadd & 0x0100) uart_tx[tx_point2]|=0x10;                             //CVH
  if (pom_cvadd & 0x0200) uart_tx[tx_point2]|=0x20;                             //CVH
  chksum ^= uart_tx[tx_point2];                                                  
  tx_point2++;
  uart_tx[tx_point2]=pom_cvadd & 0x7F;                                          //CVL
  chksum ^= uart_tx[tx_point2];                                                  
  tx_point2++;
  uart_tx[tx_point2]=cv1 & 0x7F;                                                //DATA7
  chksum ^= uart_tx[tx_point2];                                                  
  tx_point2++;
  uart_tx[tx_point2]=0x7F;                                                      //THROTTLE IDH
  chksum ^= uart_tx[tx_point2];                                                  
  tx_point2++;
  uart_tx[tx_point2]=0x7F;                                                      //THROTTLE IDL
  chksum ^= uart_tx[tx_point2];                                                  
  tx_point2++;
  uart_tx[tx_point2]=(chksum & 0x7F);                                           //CHKSUM       
  tx_point2++;
}
*/

void lack (void) 
{
  chksum=0xFF;
  uart_tx[tx_point2]=0xB4;                                                      //LACK    
  chksum ^= 0xB4;                                                  
  tx_point2++;
  uart_tx[tx_point2]=lack1;                                                  
  chksum ^= lack1;                                                                                                   
  tx_point2++;
  uart_tx[tx_point2]=lack2;                                                  
  chksum ^= lack2;
  tx_point2++;
  uart_tx[tx_point2]=chksum;                                                  
  tx_point2++;
}

/*
void nlack (void) 
{
  chksum=0xFF;
  chksum ^= 0xB4;
  uart_tx[tx_point2]=0xB4;                                                 
  tx_point2++;
  tx_point2 &= 0x3F;      
  chksum ^= 0x6F;
  uart_tx[tx_point2]=0x6F;                                                  
  tx_point2++;
  tx_point2 &= 0x3F;      
  chksum ^= 0x7F;
  uart_tx[tx_point2]=0x7F;                                                  
  tx_point2++;
  tx_point2 &= 0x3F;      
  uart_tx[tx_point2]=chksum;                                                  
  tx_point2++;
  tx_point2 &= 0x3F;
}
*/

void sendLNCV(void) 
{
  chksum = 0xFF;
  uart_tx[tx_point2]=0xE5;
  chksum ^= 0xE5;                                                  
  tx_point2++;
  uart_tx[tx_point2]=0x0F;                                                  
  chksum ^= 0x0F;                                                  
  tx_point2++;
  uart_tx[tx_point2]=0x05;                                                  
  chksum ^= 0x05;                                                  
  tx_point2++;
  uart_tx[tx_point2]=0x49;                                                  
  chksum ^= 0x49;                                                  
  tx_point2++;
  uart_tx[tx_point2]=0x4B;                                                  
  chksum ^= 0x4B;                                                  
  tx_point2++;
  uart_tx[tx_point2]=0x1F;                                                  
  chksum ^= 0x1F;                                                  
  tx_point2++;
  tmp=0x00;                                                                     //DHI bit 1 e 2 per ARTNR                                     
  if ((val1 & 0x0080)!=0) tmp |= 0x04;   
  if ((val1 & 0x8000)!=0) tmp |= 0x08;   
  if ((val2 & 0x0080)!=0) tmp |= 0x10;   
  if ((val2 & 0x8000)!=0) tmp |= 0x20;   
  if ((val3 & 0x80)!=0) tmp |= 0x40;   

  uart_tx[tx_point2]=tmp;                                                  
  chksum ^= tmp;                                                  
  tx_point2++;
  
  uart_tx[tx_point2]=0x57;                                                      //ARTNR = 1111
  chksum ^= 0x57;                                                  
  tx_point2++;
  uart_tx[tx_point2]=0x04;                                                      
  chksum ^= 0x04;                                                  
  tx_point2++;

  uart_tx[tx_point2]=(val1 & 0x007F);                                           //LNCV ADDR       
  chksum ^= uart_tx[tx_point2];                                                  
  tx_point2++;
  uart_tx[tx_point2]=(val1 & 0x7F00) >> 8;                                                  
  chksum ^= uart_tx[tx_point2];                                                  
  tx_point2++;

  uart_tx[tx_point2]=(val2 & 0x007F);                                           //LNCV DATA        
  chksum ^= uart_tx[tx_point2];                                                  
  tx_point2++;
  uart_tx[tx_point2]=(val2 & 0x7F00) >> 8;                                                  
  chksum ^= uart_tx[tx_point2];                                                  
  tx_point2++;

  uart_tx[tx_point2]=(val3 & 0x7F);                                             //LNCV FLAG      
  chksum ^= uart_tx[tx_point2];                                                  
  tx_point2++;

  uart_tx[tx_point2]=chksum;                                                                                                    
  tx_point2++;
}

//=============================================================================
//UART parsing
//=============================================================================

void parser(void)
{

  if (rx_data & 0x80) {
    lndecode = 0;
    chksum = 0;
    index = 0;
    lack1 = rx_data & 0x7F;
  }

  switch (lndecode) {
    case 0:
      switch (rx_data) {
        case 0xEF:                                                              //write to slot
          chksum ^= rx_data;
          lndecode = 1;
        break;
        case 0x82:                                                              //power off
          chksum ^= rx_data;
          lndecode = 4;
          lnlen=1;
        break;
        case 0x83:                                                              //power on
          chksum ^= rx_data;
          lndecode = 5;
          lnlen=1;
        break;
        case 0xE5:                                                              //manage LNCV
          chksum ^= rx_data;
          lndecode = 6;
        break;        
        case 0xED:                                                              //manage LNCV
          chksum ^= rx_data;
          lndecode = 6;
        break;        
      }
    break;

//==============================================================================
// WRITE to SLOT
//==============================================================================

    case 1:
      chksum ^= rx_data;
      lnlen=rx_data-2;
      lndecode=2;
    break;
    case 2:
      chksum ^= rx_data;
      lnrx[index]=rx_data;
      index++;
      index &= 0x0F;
      if (lnlen==index)  {                                                      //has the packet completely arrived?
        if (chksum==0xFF) {                                                     //is the checksum correct?
          if (lnrx[0]==0x7C) {                                                  //is an access to slot #124?
            switch (lnrx[1]) {                                                  //operation select
              case 0x6B:                                                        //PROG byte access (WRITE)
                lndecode=0;
                //pom_addr = lnrx[3];
                //pom_addr <<= 7;
                //pom_addr |= lnrx[4];
                pom_cvadd = lnrx[6] & 0x30;
                pom_cvadd <<= 4;
                if (lnrx[6] & 0x01) pom_cvadd |= 0x0080;
                pom_cvadd |= lnrx[7];
                pom_cvdata = lnrx[8];
                if (lnrx[6] & 0x02) pom_cvdata |= 0x80;
                pom_mode = 5;
                pom_go = 1;
                lack2 = 0x40;
                lack();
              break;
              case 0x2B:                                                        //PROG byte access (READ)
                lndecode=0;
                pom_cvadd = lnrx[6] & 0x30;
                pom_cvadd <<= 4;
                if (lnrx[6] & 0x01) pom_cvadd |= 0x0080;
                pom_cvadd |= lnrx[7];
                pom_mode = 4;
                pom_go = 1;
                lack2 = 0x01;
                lack();
              break;
              case 0x2F:                                                        //POM byte access (READ)
                lndecode=0;
                pom_addr = lnrx[3];
                pom_addr <<= 7;
                pom_addr |= lnrx[4];
                pom_cvadd = lnrx[6] & 0x30;
                pom_cvadd <<= 4;
                if (lnrx[6] & 0x01) pom_cvadd |= 0x0080;
                pom_cvadd |= lnrx[7];
                pom_cvdata = lnrx[8];
                if (lnrx[6] & 0x02) pom_cvdata |= 0x80;
                pom_mode = 1;
                pom_go = 1;
                lack2 = 0x01;
                lack();
              break;
              case 0x6F:                                                        //POM byte access (WRITE) answer
                lndecode=0;
                pom_addr = lnrx[3];
                pom_addr <<= 7;
                pom_addr |= lnrx[4];
                pom_cvadd = lnrx[6] & 0x30;
                pom_cvadd <<= 4;
                if (lnrx[6] & 0x01) pom_cvadd |= 0x0080;
                pom_cvadd |= lnrx[7];
                pom_cvdata = lnrx[8];
                if (lnrx[6] & 0x02) pom_cvdata |= 0x80;
                pom_mode = 2;
                pom_go = 1;
                lack2 = 0x01;
                lack();
              break;
              case 0x67:                                                        //POM byte access (WRITE) no answer
                lndecode=0;
                pom_addr = lnrx[3];
                pom_addr <<= 7;
                pom_addr |= lnrx[4];
                pom_cvadd = lnrx[6] & 0x30;
                pom_cvadd <<= 4;
                if (lnrx[6] & 0x01) pom_cvadd |= 0x0080;
                pom_cvadd |= lnrx[7];
                pom_cvdata = lnrx[8];
                if (lnrx[6] & 0x02) pom_cvdata |= 0x80;
                pom_mode = 3;
                pom_go = 1;
                lack2 = 0x40;
                lack();
              break;
              default:
                lack2 = 0x7F;
                lack();
                lndecode=0;
              break;                   
            }
          } else {
             lack2 = 0x7F;
             lack();
          } 
        } else lndecode=0;
      }   
    break;

//==============================================================================
// Power OFF
//==============================================================================

    case 4:                                                                     //power off
      lndecode=0;
      chksum ^= rx_data;
      index++;
      index &= 0x0F;
      if (lnlen==index)  {                                                      //has the packet completely arrived?
        if (chksum==0xFF) {                                                     //is the checksum correct?
          dcc_off=1;                                                        
        }
      }  
    break;

//==============================================================================
// Power ON
//==============================================================================

    case 5:                                                                     //power on
      lndecode=0;
      chksum ^= rx_data;
      index++;
      index &= 0x0F;
      if (lnlen==index)  {                                                      //has the packet completely arrived?
        if (chksum==0xFF) {                                                     //is the checksum correct?
          dcc_off=0;                                                        
        }
      }  
    break;
    
//==============================================================================
// LNCV management
//==============================================================================

    case 6:
      chksum ^= rx_data;
      lnlen=rx_data-2;
      lndecode=7;
    break;
    case 7:
      chksum ^= rx_data;
      lnrx[index]=rx_data;
      index++;
      index &= 0x0F;
      if (lnlen==index)  {                                                      //has the packet completely arrived?
        if (chksum==0xFF) {                                                     //is the checksum correct?
          if ((lnlen==13) && (lnrx[0]==0x01) && (lnrx[1]==0x05) && (lnrx[2]==0)) { //it's a LNCV message
            if (lnrx[4] & 0x01) lnrx[5] |= 0x80;                                // N. art. low
            if (lnrx[4] & 0x02) lnrx[6] |= 0x80;                                // N. art. high
            if (lnrx[4] & 0x04) lnrx[7] |= 0x80;                                // 0x00 for PRON/PROFF - LNCV low address for READ/WRITE
            if (lnrx[4] & 0x08) lnrx[8] |= 0x80;                               // 0x00 for PRON/PROFF - LNCV high address for READ/WRITE
            if (lnrx[4] & 0x10) lnrx[9] |= 0x80;                               // decoder low address for PRON/PROFF - LNCV low data for READ/WRITE
            if (lnrx[4] & 0x20) lnrx[10] |= 0x80;                               // decoder high address for PRON/PROFF - LNCV high data for READ/WRITE
            if (lnrx[4] & 0x40) lnrx[11] |= 0x80;                               // flag: 0x80 = PRON, 0x40 = PROFF, 0x00 for READ/WRITE
            article = lnrx[5] + (lnrx[6] << 8);
            lncv_address = lnrx[7] + (lnrx[8] << 8);
            lncv_data = lnrx[9] + (lnrx[10] << 8);
            flag = lnrx[11];
            
            if ((lnrx[3]==0x21) && (flag==0x80)) {                              // PRON
    		  if (((article == ARTNR) || (article == 0xFFFF) || (article == 0x0000)) && ((lncv_data == modaddr) || (lncv_data == 0xFFFF))) {
                progmode = 1;
                val1=0x00;
                val2=modaddr;
                val3=0x80;
                sendLNCV();
              }   
            }
            if ((lnrx[3]==0x21) && (flag==0x40)) {                              // PROFF
    		  if (((article == ARTNR) || (article == 0xFFFF) || (article == 0x0000)) && ((lncv_data == modaddr) || (lncv_data == 0xFFFF))) {
                progmode = 0;
              }   
              
            }
            if ((lnrx[3]==0x21) && (flag==0x00)) {                              // READ LNCV
              if (progmode==1) {
                if (lncv_address < lncv_number) {
                  lncv_data=lncv[lncv_address];
                  val1=lncv_address;
                  val2=lncv_data;
                  val3=0x00;
                  sendLNCV();
			    } else {
                  lack2 = 0x01;
                  lack();
                }
              }
            }
            
            if (lnrx[3]==0x20)                                                  // LNCV WRITE
              if (progmode==1) {
                if (lncv_address < lncv_number) {
                  lncv[lncv_address] = lncv_data;
                  lack2 = 0x7F;
                  lack();
                  lncv_update();
                  lncv_init();
			    } else {
                  lack2 = 0x01;
                  lack();
                }
              }  
            break;
          }  
        }      
      }
    break;          
  }
}


//=============================================================================
//UART management
//=============================================================================

void uartcontrol(void)
{ 
  while (rx_point!=rx_point2) {                                                 //process all received chars
    rx_data=uart_rx[rx_point2];
    rx_point2++;
    
    uart_tx[tx_point2]=rx_data;                                                 //TEST
    tx_point2++;                                                                //TEST
      
    parser();                                                                   //process actual char
  }  
}

//=============================================================================
//Railcom management
//=============================================================================

void railcom_decode(void)
{
  for(j=0;j<railpoint;j++) rail[j]=codez[rail[j]];
  j=0;
  id2=0xFF;
  while (j<railpoint) {
    if (rail[j] < 0x40) {
      tmp=(rail[j] >> 2) & 0x0F;
      if (tmp==0) {
        cv1=((rail[j] & 0x03) << 6) | (rail[j+1] & 0x3F);
        j=j+2;
        id2=0;
      }  
      if ((tmp==1) || (tmp==2)) {
        j=j+2;
      }  
    } else {
      j++;
    }
  }
}

//=============================================================================
//prepare DCC packet in POM mode
//=============================================================================

void prepare_pomdcc(void)
{
  if (pom_addr>127) {
    dcc[dcc_rx1]=6;
    dcc_rx1++;
    dcc[dcc_rx1] = 0xC0;
    dcc[dcc_rx1] |= (pom_addr & 0x3F00) >> 8;
    chksum = dcc[dcc_rx1];
    dcc_rx1++;
    dcc[dcc_rx1] = pom_addr & 0x00FF;
    chksum ^= dcc[dcc_rx1];
  } else {
    dcc[dcc_rx1]=5;
    dcc_rx1++;
    dcc[dcc_rx1]=pom_addr & 0x7F;
    chksum = dcc[dcc_rx1];
  } 
  dcc_rx1++;
  if (pom_mode==1) dcc[dcc_rx1]=0xE4; else dcc[dcc_rx1]=0xEC;
  dcc[dcc_rx1] |= (pom_cvadd >> 8) & 0x03;
  chksum ^= dcc[dcc_rx1];
  dcc_rx1++;
  dcc[dcc_rx1]=pom_cvadd & 0x00FF;
  chksum ^= dcc[dcc_rx1];
  dcc_rx1++;
  if (pom_mode==1) dcc[dcc_rx1]=0x00; else dcc[dcc_rx1]=pom_cvdata;
  chksum ^= dcc[dcc_rx1];
  dcc_rx1++;
  dcc[dcc_rx1]=chksum;
  dcc_rx1++;
  dcc_rx=dcc_rx1; 
}

//=============================================================================
//prepare DCC packet in PROG mode
//=============================================================================

void prepare_progdcc(void)
{ 
  dcc[dcc_rx1]=4;
  dcc_rx1++;
  if (maindcc==16) dcc[dcc_rx1]=0x78;
  if (maindcc==18) dcc[dcc_rx1]=0x74;
  if (maindcc==21) dcc[dcc_rx1]=0x7C;
  dcc[dcc_rx1] |= (pom_cvadd >> 8) & 0x03;
  chksum = dcc[dcc_rx1];
  dcc_rx1++;
  dcc[dcc_rx1]=pom_cvadd & 0x00FF;
  chksum ^= dcc[dcc_rx1];
  dcc_rx1++;
  if (maindcc==16) dcc[dcc_rx1]=(0xE0 | progbit); else dcc[dcc_rx1]=pom_cvdata;
  chksum ^= dcc[dcc_rx1];
  dcc_rx1++;
  dcc[dcc_rx1]=chksum;
  dcc_rx1++;
  dcc_rx=dcc_rx1;
}

//=============================================================================
//prepare DCC packet in PROG mode
//=============================================================================
/*
void prepare_progdcc2(void)
{ 
  dcc[dcc_rx1]=4;
  dcc_rx1++;
  dcc[dcc_rx1]=0x74;
  dcc[dcc_rx1] |= (pom_cvadd >> 8) & 0x03;
  chksum = dcc[dcc_rx1];
  dcc_rx1++;
  dcc[dcc_rx1]=pom_cvadd & 0x00FF;
  chksum ^= dcc[dcc_rx1];
  dcc_rx1++;
  dcc[dcc_rx1]=pom_cvdata;
  chksum ^= dcc[dcc_rx1];
  dcc_rx1++;
  dcc[dcc_rx1]=chksum;
  dcc_rx1++;
  dcc_rx=dcc_rx1; 
}
*/

//=============================================================================
//prepare RESET
//=============================================================================

void prepare_reset(void)
{ 
  dcc[dcc_rx1]=3;
  dcc_rx1++;
  dcc[dcc_rx1] = 0;
  dcc_rx1++;
  dcc[dcc_rx1] = 0;
  dcc_rx1++;
  dcc[dcc_rx1] = 0;
  dcc_rx1++;
  dcc_rx=dcc_rx1;
}

//=============================================================================
//LED control
//=============================================================================

void ledcontrol(void)
{ 
  ledco++;
  if (dcc_off==1) {
    if (ledco & 0x80) LED=1; else LED=0;
  } else {
    if (progmode==1) {
      if (ledco & 0x40) LED=1; else LED=0;
    }
  }
}

//=============================================================================
//MAIN
//=============================================================================

void main(void)
{

 //=============================================================================
 //INIZIALIZZO e SET PORTE
 //=============================================================================
                                                                 
  P1=0x02;                                                                         
  P3=0x02;
  P5=0x00;

  P_SW2 = 0x80;																    // processor frequency set
  CKSEL = 0x00;                                                                 // internal clock 24MHz
  CLKDIV = 0x01;                                                                // divided by 1 -> 24MHz
  P_SW2 = 0x00;																     

  P1M1 = 0x05;                                                                  //PORT1 direction IN 
  P1M0 = 0xFA;  	                                                            //PORT1 direction OUT
  P3M1 = 0x01;                                                                  //PORT3 direction IN 
  P3M0 = 0xFE;  	                                                            //PORT3 direction OUT
  P5M1 = 0x00;                                                                  //PORT5 direction IN 
  P5M0 = 0xFF;  	                                                            //PORT5 direction OUT
  
  dcc_state=0;
  dcc_gen=0;
  dcc_off=0;
  railcom=1;
  preamble_max=17;                                                              //generates preamble_max "1" - 1 (default 17)
  preamble=0;
  dcc_bit=0; 
  dcc_byte=0;
  railbit=0; 
  dcc_tx=0;
  dcc_rx=0;
  dcc_rx1=0;
  dcc_len=0;
  dcc_mask=0;
  railwin=0;
  railgo=0;
  tx_point=0;
  tx_point2=0;
  rx_point=0;
  rx_point2=0;
  lndecode=0;
  maindcc=0;
  pom_go=0;
  progmode=0;
  dcc_idle=0;                                                                    
   
  AUXR = 0x44;								                                    // SYSCLK/12 sul T0, SYSCLK sul T1 e T2
  TMOD = 0x00;                                                                  // timer 0 e timer 1 = mode 0 (reload)
  TL0 = 0xC4;                                                                   // overflow dopo 30us per controllo DCC
  TH0 = 0xFF;														
  TCON = 0x10;									                                // timer 0 start
  ET0 = 1;                                                                      // Enable timer0 interrupt

  SCON = 0x50;		                                                            // 8 bits and variable baudrate, receiver enabled 
  TL1 = 0xE8;                                                                   // Initial timer value for 250000 baud 
  TH1 = 0xFF;
  TCON |= 0x40;														            // timer 1 start
 
  S2CON = 0x12;		                                                            // 8 bits and variable baudrate, receiver enabled 
  T2L = 0x98;		                                                            // Initial timer value for 115200 (FFCC) or 57600 (FF98) baud
  T2H = 0xFF;		                                                            // Initial timer value
  AUXR |= 0x10;		                                                            // Timer2 start run
  
  P_SW2 |= 0x80;									
  ADCTIM = 0x3F;							                                    // CSSETUP=0 (1), CSHOLD=1 (2), SMPDUTY = 10 (11), CONVERSION = 10, TOTAL = 24 * 2.66us = 64us conversion time
  P_SW2 &= 0x7F;
  ADCCFG = 0x2F;							                                    // right just, speed=15 (24MHz / 32 = 0.375MHz or 2.66us/bit)
  ADC_CONTR = 0x82;							                                    // ADC ON, channel 2 selected 
  
 //=============================================================================
 // INIZIALIZZA LA EE e LE LNCV
 //=============================================================================

  lncv_init ();                                                                 //init LNCV at startup  

  EA = 1;									                                    // Enable global interrupts
  ENA=1;                                                                        //Enable HV  
  delay_ms(100);                                                                //to stabilize power supply
 
  while(1) {                                                                     //main loop
    
    uartcontrol();
    ledcontrol();

    delay_ms(1);
    
    switch (maindcc) {
      case 0:
        LED=1;
        railgo=0;
        railcom=0;
        preamble_max=17;
        if (pom_go==1) {        
          if (pom_mode==1) maindcc=1;                                           //RD POM byte with answer
          if (pom_mode==2) maindcc=5;                                           //WR POM byte with answer
          if (pom_mode==3) maindcc=10;                                          //WR POM byte without answer          
          if (pom_mode==4) {
            maindcc=15;                                                         //RD PROG byte (8 bit+byte)
            retry=retrylim;
          }  
          if (pom_mode==5) {                                                    //WR PROG byte
            maindcc=20;
            retry=retrylim;
          }                                                           
        }
      break;     
//==============================================================================
// RD POM (0x2F)
//==============================================================================
      case 1:
        pom_go=0;
        railcom=1;
        preamble_max=17;
        for (i=0;i<pomnum;i++) prepare_pomdcc();
        dcc_idle=0;
        railgo=0;
        maindcc=2;
        timeco=0;
      break;
      case 2:       
        if (dcc_idle==1) {
          maindcc=0;
        }   
        if (railgo==1) {
          railgo=0;
          timeco++;
          if (timeco>pomstb) {
            railcom_decode();
            //servicemsg();
            if (id2==0) {
              LED=0;
              val3=0x2F;
              answer_pom();
              maindcc=3;
            }  
          }
        } 
      break; 
      case 3:
        if (dcc_idle==1) {
          maindcc=0;
        }         
      break;
//==============================================================================
// WR POM with answer (0x6B)
//==============================================================================      
      case 5:
        pom_go=0;
        railcom=1;
        preamble_max=17;
        for (i=0;i<pomnum;i++) prepare_pomdcc();
        dcc_idle=0;
        maindcc=6;
        timeco=0;
      break;
      case 6:       
        if (dcc_idle==1) {
          maindcc=0;
        }   
        if (railgo==1) {
          railgo=0;
          timeco++;
          if (timeco>pomstb) {
            railcom_decode();
            //servicemsg();
            if (id2==0) {
              LED=0;
              val3=0x6B;
              answer_pom();
              maindcc=7;
            }  
          }
        } 
      break; 
      case 7:
        if (dcc_idle==1) {
          maindcc=0;
        }         
      break;     
//==============================================================================
// WR POM without answer (0x67)
//==============================================================================             
      case 10:
        pom_go=0;
        railcom=1;
        preamble_max=17;
        for (i=0;i<pomnum;i++) prepare_pomdcc();
        dcc_idle=0;
        maindcc=11;
        timeco=0;
      break;      
      case 11:       
        if (dcc_idle==1) {
          maindcc=0;
        }         
      break;
//==============================================================================
// RD PROG 8 BIT + BYTE
//==============================================================================            
      case 15:
        pom_go=0;
        progbit=7;
        railcom=0;
        cv1=0;
        preamble_max=21;
        maindcc=16;
      break;      
      case 16:
        for (i=0;i<respre;i++) prepare_reset();
        for (i=0;i<prognum;i++) prepare_progdcc();
        for (i=0;i<respost;i++) prepare_reset();
        dcc_idle=0;
        maindcc=17;
        timeco=0;
      break;     
      case 17:
        timeco++;
        if (timeco==20) {
          ack=0;
          LED=0;
        }  
          
        if (dcc_idle==1) {

          timeco=0;
          progbit--;
          LED=1;
          if (ack>acklen) {
            cv1<<=1;
          } else {
            cv1<<=1;
            cv1|=0x01;
          }
          //ack=0;
          if (progbit==255) maindcc=18; else maindcc=16;
        }
      break;
      case 18:
        pom_cvdata=cv1;
        for (i=0;i<respre;i++) prepare_reset();
        for (i=0;i<prognum;i++) prepare_progdcc();
        for (i=0;i<respost;i++) prepare_reset();
        dcc_idle=0;
        maindcc=19;
        timeco=0;
      break;           
      case 19:
        timeco++;
        if (timeco==20) {
          ack=0;
          LED=0;
        }  
        if (dcc_idle==1) {
          timeco=0;  
          LED=1;                 
          if (ack>acklen) {
            pom_stat=0;
            answer_prog();
            maindcc=0;
          } else {
            if (retry==0) {
              pom_stat=4;
              answer_prog();
              maindcc=0;
            } else {
              retry--;
              maindcc=15;
            }
          }  
        }
      break;
//==============================================================================
// WR PROG without answer
//==============================================================================               
      case 20:
        pom_go=0;
        railcom=0;
        preamble_max=21;
        maindcc=21;
      break;      
      case 21:
        for (i=0;i<respre;i++) prepare_reset();
        for (i=0;i<prognum;i++) prepare_progdcc();
        for (i=0;i<respost;i++) prepare_reset();
        dcc_idle=0;
        maindcc=22;
        timeco=0;
      break;     
      case 22:
        timeco++;
        if (timeco==20) {
          ack=0;
          LED=0;
        }  
        if (dcc_idle==1) {
          timeco=0;
          LED=1;
          if (ack>acklen) {
            maindcc=0;
          } else {
            if (retry==0) {
              maindcc=0;
              pom_stat=2;
              answer_prog();
            } else {
              retry--;
              maindcc=20;
            }            
          }
        }
      break;
    }                                                                           // end switch (maindcc)
  }                                                                             // end while(1)
}                                                                               // end main
