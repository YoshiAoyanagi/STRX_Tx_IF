#include <TimerOne.h>
#include <avr/io.h>
#include <avr/interrupt.h>

#define FOSC 10000000 // 20MHz
#define BAUD 57600
#define MYUBRR FOSC/16/BAUD-1

#define OBC_CLK         A0 //output PC0 
#define OBC_DATA        A1 //GPIO IO3 PC1 

#define COMNM_VREF      A2  //COMM_voltage status input PC2
#define VBAT            A3  //LM61 Analog temperature sensor(Option) PC3
#define TX_NRZL         5   //STRX_data_input flag  PD5
#define TX_ON           4   //STRX_data ONOFF output low  PD4

#define BR4KBPS        0
#define BR32KBPS       1
#define BR64KBPS       2
#define CMD_TX_ON      0x0A
#define CMD_TX_OFF     0x0B

#define IF_STX1  0xEB
#define IF_STX2  0x90
#define IF_ETX1  0xC5
#define IF_ETX2  0x79

#define   TX_DATA_BYTE  256
#define   PN_DUMMY_BYTE  28
#define   TX_EMPTY_CODE 0xAA

volatile static int           tx_send_ena;
volatile static unsigned int  tx_data_pos;
volatile static unsigned char tx_bit_pos;
volatile static unsigned int  dummy_data_pos;
volatile static unsigned char dummy_bit_pos;
volatile static unsigned char rec_sts;

static unsigned char          bit_rate;  //0 0: 4kbps, 0 1: 32kbps, 1 0 : 64kbps
static unsigned char          tx_on_flg;

volatile unsigned char tx_data[TX_DATA_BYTE];
volatile static unsigned char dummy_data[TX_DATA_BYTE];
volatile static unsigned char pn_dummy[PN_DUMMY_BYTE] = {0xFF,0x48,0x0E,0xC0,0x9A,0x0D,0x70,0xBC,0x8E,0x2C,0x93,0xAD,0xA7,0xB7, 0x46,0xCE,0x5A,0x97,0x7D,0xCC,0x32,0xA2,0xBF,0x3E,0x0A,0x10,0xF1,0x88};

volatile int rec_bit_pos = 7;
volatile int rec_data_pos;

ISR(PCINT1_vect)
{
  unsigned char tmp_bit=0;
  if(tx_send_ena)
    return;
    
  if(bit_is_set(PINC, PC0))
  {
    tx_send_ena = 0;

    if(bit_is_set(PINC,1) == 0)
      tmp_bit = 0;
    else
      tmp_bit = 1;
        
    tx_data[rec_data_pos] += ((tmp_bit&0x01) << rec_bit_pos)&0xFF;
    rec_bit_pos--;
    if(rec_bit_pos == -1)
    {
      rec_bit_pos = 7;
      rec_data_pos++;
    }

    if(rec_data_pos > (255))
    {
      tx_send_ena = 1;
      rec_data_pos = 0;
    }
  }
}

ISR(TIMER1_COMPA_vect)
{
  if(tx_send_ena)
  {
    bool bit_val = (tx_data[tx_data_pos] >> (7 - tx_bit_pos)) & 0x01;
    tx_portwrite(bit_val);
    tx_bit_pos++;
    if(tx_bit_pos == 8)
    {
      tx_bit_pos = 0;
      tx_data_pos++;
    }
    
    if(tx_data_pos == TX_DATA_BYTE)
    {
      TxDataClear();
      tx_send_ena = 0;
      tx_bit_pos = 0;
      tx_data_pos = 0;
    }
  }
  else
  {
    bool bit_val = (pn_dummy[dummy_data_pos] >> (7 - dummy_bit_pos)) & 0x01;
    tx_portwrite(bit_val);
    dummy_bit_pos++;
    if(dummy_bit_pos == 8)
    {
      dummy_bit_pos = 0;
      dummy_data_pos++;
    }
    if(dummy_data_pos == PN_DUMMY_BYTE)
    {
      dummy_bit_pos = 0;
      dummy_data_pos = 0;
    }
  }
}

ISR(USART_RX_vect)
{
    IF_Cmd_Dispatcher(UDR0);
}
void TxDataSetDummy(unsigned char val)
{
  int i;
  for(i = 0; i < TX_DATA_BYTE; i++)
  {
    dummy_data[i] = val;
  }
}

void TxDataClear(void)
{
  int i;
  for(i = 0; i < TX_DATA_BYTE; i++)
  {
    tx_data[i] = 0x00;
  }
}

void TxDataSetContinue(void)
{
  int i;
  for(i = 0; i < TX_DATA_BYTE; i++)
  {
    dummy_data[i] = tx_data[i];
  }
}

void TxSetBitrate(unsigned char _bit_rate)
{
  bit_rate = _bit_rate;
  if(bit_rate == BR4KBPS)
  {
    noInterrupts();      // 割り込み禁止
    bit_rate = BR4KBPS;
    TIMSK1 = 0x00;
    TCCR1B = 0b00000000;
    TCNT1  = 0x00;
    TCCR1A = 0b00000000;
    OCR1AH = 0x09;           //4k:0x09 //32k: 0x01 //64k: 0x00
    OCR1AL = 0xC3;           // 4k: 0xC3 //32k: 0x37 //64k: 155
    TCCR1B = 0b00001001;      //CTC, プリスケーラ無
    TIMSK1 = 0b00000010;            // enable ocr2A interroupt
    TIMSK0 = 0x00000000;
    interrupts();   // 割り込み許可
  }
  else if(_bit_rate == BR32KBPS)
  {
    noInterrupts();      // 割り込み禁止
    bit_rate = BR32KBPS;
    TIMSK1 = 0x00;
    TCCR1B = 0b00000000;
    TCNT1  = 0x00;
    TCCR1A = 0b00000000;
    OCR1AH = 0x01;           //4k:0x09 //32k: 0x01 //64k: 0x00
    OCR1AL = 0x37;           // 4k: 0xC3 //32k: 0x37 //64k: 155
    TCCR1B = 0b00001001;      //CTC, プリスケーラ無
    TIMSK1 = 0b00000010;            // enable ocr2A interroupt
    TIMSK0 = 0x00000000;
    interrupts();    // 割り込み許可
  }
  else if(_bit_rate == BR64KBPS)
  {
    noInterrupts();      // 割り込み禁止
    bit_rate = BR64KBPS;
    TIMSK1 = 0x00;
    TCCR1B = 0b00000000;
    TCNT1  = 0x00;
    TCCR1A = 0b00000000;
    OCR1AH = 0x00;           //4k:0x09 //32k: 0x01 //64k: 0x00
    OCR1AL = 0x9B;           // 4k: 0xC3 //32k: 0x37 //64k: 155
    TCCR1B = 0b00001001;      //CTC, プリスケーラ無
    TIMSK1 = 0b00000010;            // enable ocr2A interroupt
    TIMSK0 = 0x00000000;
    interrupts();    // 割り込み許可
  }
}

void setup(){

  //PCINT割り込み設定
  PCICR  |= (1 << PCIE1);
  PCMSK2 = 0;
  PCMSK1 |= 1 << PCINT8;
  PCMSK0 = 0;
  
  pinMode(COMNM_VREF ,INPUT);
  //pinMode(OBC_CLK, INPUT);
  pinMode(OBC_DATA ,INPUT);
  
  pinMode(TX_NRZL  ,OUTPUT);
  pinMode(TX_ON  ,OUTPUT);
  
  DDRC = 0b00000000;

  digitalWrite(TX_NRZL,HIGH);  //

  unsigned int ubrr = MYUBRR;
  UBRR0H = (unsigned char)(ubrr>>8);    // ボーレート上位8bit
  UBRR0L = (unsigned char)ubrr;        // ボーレート下位8bit
  UCSR0B = (1<<RXEN0) | (1<<TXEN0) | (1<<RXCIE0);    // 送受信許可,割り込み許可
  UCSR0C = (3<<UCSZ00) ;        // stopbit 1bit , 8bit送信
   
  tx_send_ena = 0;
  tx_on_flg = 0;
  rec_data_pos = 0;
  tx_data_pos = 0;
  
  TxOff();
  TxDataClear();
  sei(); // 全割り込み許可
  TxSetBitrate(BR4KBPS);
}

void loop(){
  
}

void IF_Cmd_Dispatcher(unsigned char recv)
{
    switch(recv)
    {
      case BR4KBPS: TxSetBitrate(BR4KBPS);break;
      case BR32KBPS: TxSetBitrate(BR32KBPS);break;
      case BR64KBPS: TxSetBitrate(BR64KBPS);break;
      case CMD_TX_ON: TxOn();break;
      case CMD_TX_OFF: TxOff();break;
      default: break;
    }
}


void TxOn(void)
{
  tx_on_flg = 1;
  digitalWrite(TX_ON, HIGH); //TX_ON
}

void TxOff(void)
{
  tx_on_flg = 0;
  digitalWrite(TX_ON, LOW); //TX_OFF
}
void tx_portwrite(int8_t ioflag) {
  if(!tx_on_flg)
  {
    return;
  }
  if (ioflag == 1) {
    if(tx_on_flg){ PORTD |= _BV(PD5);}
  } else {
    if(tx_on_flg){ PORTD &= ~_BV(PD5);}
  }
}
