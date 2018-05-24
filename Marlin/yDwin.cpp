#include "yDwin.h"
#include "HardwareSerial.h"
#if !(defined(DISABLED_SERIAL0_RXISR)||defined(DISABLED_SERIAL1_RXISR)||defined(MACRO_var_V001)||defined(DISABLED_SERIAL3_RXISR))
#error You must set a serial for dwin interface.
#else
#if (defined(DISABLED_SERIAL0_RXISR) && DWIN_SER_PORT != 0)||\
	(defined(DISABLED_SERIAL1_RXISR) && DWIN_SER_PORT != 1)||\
	(defined(MACRO_var_V001) && DWIN_SER_PORT != 2)||\
	(defined(DISABLED_SERIAL3_RXISR) && DWIN_SER_PORT != 3)
#error The DWIN_SER_PORT is not adapted to the unuse serial port.
#endif
#endif
volatile bool Ydwin::GLOBAL_var_V006 = false;
uint8_t Ydwin::dwinDataLen = 0;
uint16_t Ydwin::dwinAddr = 0;
uint8_t Ydwin::dwinData[128];
void Ydwin::FunV001()
{
	uint16_t baud_setting = (F_CPU / 4 / DWIN_SER_BAUD - 1) / 2;
	DWIN_UCSRxA = 1 << DWIN_U2Xx;
	#if F_CPU == 16000000UL&&DWIN_SER_BAUD==57600
	DWIN_UCSRxA = 0;
	baud_setting = (F_CPU / 8 / DWIN_SER_BAUD - 1) / 2;
	#endif
	DWIN_UBRRxH = baud_setting >> 8;
	DWIN_UBRRxL = baud_setting;
	#if defined(__AVR_ATmega8__)
	DWIN_UCSRxC = 0x86; 
	#else
	DWIN_UCSRxC = 0x06;
	#endif
	SBI(DWIN_UCSRxB, DWIN_RXENx);
	SBI(DWIN_UCSRxB, DWIN_TXENx);
	SBI(DWIN_UCSRxB, DWIN_RXCIEx);
	CBI(DWIN_UCSRxB, DWIN_UDRIEx);
}
void Ydwin::FunV002()
{
	CBI(DWIN_UCSRxB, DWIN_RXENx);
	CBI(DWIN_UCSRxB, DWIN_TXENx);
	CBI(DWIN_UCSRxB, DWIN_RXCIEx);
	CBI(DWIN_UCSRxB, DWIN_UDRIEx);
}
bool Ydwin::waitData(uint8_t*dat)
{
	uint16_t t=10000;
	while (t && !GLOBAL_var_V006)
	{
		t--;
		DELAY_10US;
	}
	GLOBAL_var_V006 = false;
	if (t)
	{
		for (t = 0; t<dwinDataLen; t++)dat[t] = dwinData[t];
		return true;
	}
	return false;
}
void Ydwin::FunV012(u8 dat)
{
	static bool ifGetData = false;
	static int8_t num = 0, n = 0, step = 0;
#ifdef DWIN_FYS_CUSTOMIZED
	if(dat==MACRO_var_V013&&step == 0)step = 1;
	else if(step==1)
	{
		if(dat==DWIN_READ_VAR)step=2;
		else step=0;
		num=0;
	}
	else if(step==2)
	{
		++num;
		switch(num)
		{
		case 1:
			dwinAddr = dat;
			break;
		case 2:
			dwinAddr = (dwinAddr << 8) | dat;
			break;
		case 3:
			n=dat<<1;
			break;
		default:
			if (n > 0)
			{
				dwinData[num - 4] = dat;
				n--;
				ifGetData = true;
			}
			break;
		}
		if (ifGetData&&n == 0)
		{
			GLOBAL_var_V006 = true;
			ifGetData = false;
			step = 0;
			dwinDataLen = num - 3;
			num = 0;
		}
	}
	else
	{
		num = 0;
		step = 0;
		n = 0;
	}
#else
	if (dat == MACRO_var_V013&&step == 0)step = 1;
	else if (dat == MACRO_var_V017&&step == 1)step = 2;
	else if (step == 2)step++;
	else if (step == 3)
	{
		switch (dat)
		{
		case DWIN_READ_REG:step = 5; break;
		case DWIN_READ_VAR:step = 6;  break;
		default:step = 0;  break;
		}
		num = 0;
	}
	else if (step == 5)
	{
		num++;
		switch (num)
		{
		case 0x01:
			dwinAddr = dat;
			break;
		case 0x02:
			n = dat;
			break;
		default:
			if (n > 0)
			{
				dwinData[num - 3] = dat;
				n--;
				ifGetData = true;
			}
			break;
		}
		if (ifGetData&&n == 0)
		{
			GLOBAL_var_V006 = true;
			ifGetData = false;
			step = 0;
			dwinDataLen = num - 2;
			num = 0;
		}
	}
	else if (step == 6)
	{
		++num;
		switch (num)
		{
		case 1:
			dwinAddr = dat;
			break;
		case 2:
			dwinAddr = (dwinAddr << 8) | dat;
			break;
		case 3:
			n = dat << 1;
			break;
		default:
			if (n > 0)
			{
				dwinData[num - 4] = dat;
				n--;
				ifGetData = true;
			}
			break;
		}
		if (ifGetData&&n == 0)
		{
			GLOBAL_var_V006 = true;
			ifGetData = false;
			step = 0;
			dwinDataLen = num - 3;
			num = 0;
		}
	}
	else
	{
		num = 0;
		step = 0;
	}
#endif
}
#if defined(DISABLED_SERIAL0_RXISR)&&defined(USART0_RX_vect)
ISR(USART0_RX_vect)
{
	uint8_t dat = DWIN_UDRx;
	Ydwin::dwinRxlrq(dat);
}
#elif defined(DISABLED_SERIAL1_RXISR)&&defined(USART1_RX_vect)
ISR(USART1_RX_vect)
{
	uint8_t dat = DWIN_UDRx;
	Ydwin::dwinRxlrq(dat);
}
#elif defined(MACRO_var_V001)&&defined(USART2_RX_vect)
ISR(USART2_RX_vect)
{
	uint8_t dat = DWIN_UDRx;
	Ydwin::FunV012(dat);
}
#elif defined(DISABLED_SERIAL3_RXISR)&&defined(USART3_RX_vect)
ISR(USART3_RX_vect)
{
	uint8_t dat = DWIN_UDRx;
	Ydwin::dwinRxlrq(dat);
}
#endif
#if MACRO_var_V011==MACRO_var_V002
void Ydwin::FunV016(const uint8_t& regAddr, const uint8_t *data, uint8_t len)
{
	FunV00E(MACRO_var_V013);
	FunV00E(MACRO_var_V017);
	FunV00E(len + 2);
	FunV00E(MACRO_var_V014);
	FunV00E(regAddr);
	while (len-- > 0)FunV00E(*data++);
}
bool Ydwin::FunV017(const uint8_t& regAddr, uint8_t*tdata, uint8_t len)
{
	FunV00E(MACRO_var_V013);
	FunV00E(MACRO_var_V017);
	FunV00E(0x03);
	FunV00E(DWIN_READ_REG);
	FunV00E(regAddr);
	FunV00E(len);
	return waitData(tdata);
}
void Ydwin::FunV017(const uint8_t& regAddr, uint8_t len)
{
	FunV00E(MACRO_var_V013);
	FunV00E(MACRO_var_V017);
	FunV00E(0x03);
	FunV00E(DWIN_READ_REG);
	FunV00E(regAddr);
	FunV00E(len);
}
#endif
void Ydwin::FunV01A(const uint16_t& varAddr, const uint8_t *varData, uint8_t len)
{
	if (len == 0)return;
#if defined(DWIN_FYS_CUSTOMIZED)
	FunV00E(MACRO_var_V013);
	FunV00E(MACRO_var_V016);
	FunV00E((uint8_t)(varAddr >> 8));
	FunV00E((uint8_t)varAddr);
	FunV00E(len>>1);
#else
	FunV00E(MACRO_var_V013);
	FunV00E(MACRO_var_V017);
	FunV00E(len + 3);
	FunV00E(MACRO_var_V016);
	FunV00E((uint8_t)(varAddr >> 8));
	FunV00E((uint8_t)varAddr);
#endif
	while (len-- > 0)FunV00E(*varData++);
}
void Ydwin::FunV01A(const uint16_t& varAddr, const uint16_t& data)
{
#if defined(DWIN_FYS_CUSTOMIZED)
	FunV00E(MACRO_var_V013);
	FunV00E(MACRO_var_V016);
	FunV00E((uint8_t)(varAddr >> 8));
	FunV00E((uint8_t)varAddr);
	FunV00E(0x01);
#else
	FunV00E(MACRO_var_V013);
	FunV00E(MACRO_var_V017);
	FunV00E(0x05);
	FunV00E(MACRO_var_V016);
	FunV00E((uint8_t)(varAddr >> 8));
	FunV00E((uint8_t)varAddr);
#endif
	FunV00E((uint8_t)(data >> 8));
	FunV00E((uint8_t)data);
}
bool Ydwin::FunV01D(const uint16_t& varAddr, uint8_t*tdata, uint8_t len)
{
	GLOBAL_var_V006 = false;
#if defined(DWIN_FYS_CUSTOMIZED)
	FunV00E(MACRO_var_V013);
	FunV00E(DWIN_READ_VAR);
	FunV00E(uint8_t(varAddr >> 8));
	FunV00E(uint8_t(varAddr));
#else
	FunV00E(MACRO_var_V013);
	FunV00E(MACRO_var_V017);
	FunV00E(0x04);
	FunV00E(DWIN_READ_VAR);
	FunV00E(uint8_t(varAddr >> 8));
	FunV00E(uint8_t(varAddr));
#endif
	FunV00E(len >> 1);
	return waitData(tdata);
}
void Ydwin::FunV01D(const uint16_t& varAddr, uint8_t len)
{
	GLOBAL_var_V006 = false;
#if defined(DWIN_FYS_CUSTOMIZED)
	FunV00E(MACRO_var_V013);
	FunV00E(DWIN_READ_VAR);
	FunV00E(uint8_t(varAddr >> 8));
	FunV00E(uint8_t(varAddr));
#else
	FunV00E(MACRO_var_V013);
	FunV00E(MACRO_var_V017);
	FunV00E(0x04);
	FunV00E(DWIN_READ_VAR);
	FunV00E(uint8_t(varAddr >> 8));
	FunV00E(uint8_t(varAddr));
#endif
	FunV00E(len >> 1);
}
void Ydwin::FunV01E()
{
#if MACRO_var_V011==MACRO_var_V003
	uint8_t cmd[4] = {0x55,0xAA,0x5A,0x5A};
	FunV01A(0x0004,cmd,4);
#elif MACRO_var_V011==MACRO_var_V002
	uint8_t cmd[2]={0x5A,0xA5};
	FunV016(0xEE,cmd,2);
#endif
}
void Ydwin::FunV024(uint16_t page)
{
#if MACRO_var_V011==MACRO_var_V003
	uint8_t cmd[4] = { 0x5A, 0x01, (page >> 8) & 0xFF, page & 0xFF };
	FunV01A(0x0084, cmd, 4);
#elif MACRO_var_V011==MACRO_var_V002
	uint8_t cmd[2] = { (page >> 8) & 0xFF, page & 0xFF };
	FunV016(0x03, cmd, 2);
#endif
}
uint16_t Ydwin::FunV025()
{
	uint8_t cmd[4] = { 0 };
#if MACRO_var_V011==MACRO_var_V003
	if (FunV01D(0x0014, cmd, 2))
	{
		uint16_t t = cmd[0];
		return (t << 8) | cmd[1];
	}
#elif MACRO_var_V011==MACRO_var_V002
	if (Ydwin::FunV017(0x03, cmd, 2))
	{
		uint16_t t = cmd[0];
		return (t << 8) | cmd[1];
	}
#endif
	return 0xFFFF;
}
void Ydwin::FunV022(uint8_t val)
{
#if MACRO_var_V011==MACRO_var_V003
	if (val > MACRO_var_V018)val = MACRO_var_V018;
	uint8_t cmd[2] = { val, 0x00 };
	FunV01A(0x0082, cmd, 2);
#elif MACRO_var_V011==MACRO_var_V002
	if(val>MACRO_var_V018)val=MACRO_var_V018;
	FunV016(0x01,&val,1);
#endif
}
void Ydwin::FunV028(uint8_t id, uint8_t segments, uint8_t volume)
{
#if MACRO_var_V011==MACRO_var_V003
	uint8_t cmd[4] = { id, segments, volume, 0x80 };
	FunV01A(0x00A0, cmd, 4);
#endif
}
void Ydwin::FunV015(const uint16_t& varAddr, const char*str, uint8_t len)
{
	if (len == 0)
	while (str[len])len++;
#if defined(DWIN_FYS_CUSTOMIZED)
	len++;
	len >>= 1;
	FunV00E(MACRO_var_V013);
	FunV00E(MACRO_var_V016);
	FunV00E((uint8_t)(varAddr >> 8));
	FunV00E((uint8_t)varAddr);
	FunV00E(len);
	len <<= 1;
	while (len--)
	{
		if(*str)FunV00E(*str++);
		else FunV00E(0);
	}
#else
	FunV00E(MACRO_var_V013);
	FunV00E(MACRO_var_V017);
	FunV00E(len + 3);
	FunV00E(MACRO_var_V016);
	FunV00E((uint8_t)(varAddr >> 8));
	FunV00E((uint8_t)varAddr);
	while (len-- > 0)
	{
		if(*str)FunV00E(*str++);
		else FunV00E(0);
	}
#endif
}
void Ydwin::dwinPutsPGM(const uint16_t& varAddr, const char*str, uint8_t len)
{
	const char*t = str;
	char tlen = 0, ch;
	while (pgm_read_byte(t))
	{
		tlen++;
		t++;
	}
	if (len == 0)len = tlen;
#if defined(DWIN_FYS_CUSTOMIZED)
	len++;
	len >>= 1;
	FunV00E(MACRO_var_V013);
	FunV00E(MACRO_var_V016);
	FunV00E((uint8_t)(varAddr >> 8));
	FunV00E((uint8_t)varAddr);
	FunV00E(len);
	len <<= 1;
	ch = pgm_read_byte(str);
	while (len-- > 0)
	{
		if (ch)
		{
			FunV00E(ch);
			str++;
			ch = pgm_read_byte(str);
		}
		else FunV00E(0);
	}
#else
	FunV00E(MACRO_var_V013);
	FunV00E(MACRO_var_V017);
	FunV00E(len + 3);
	FunV00E(MACRO_var_V016);
	FunV00E((uint8_t)(varAddr >> 8));
	FunV00E((uint8_t)varAddr);
	ch = pgm_read_byte(str);
	while (len-- > 0)
	{
		if (ch)
		{
			FunV00E(ch);
			str++;
			ch = pgm_read_byte(str);
		}
		else FunV00E(0);
	}
#endif
}
void Ydwin::FunV049(millis_t& pt)
{
	uint8_t t;
	t = pt / 3600;
	if (t >= 100)
	{
		cdt[n++] = t / 100 + '0';
		t %= 100;
	}
	if (t >= 10)
	{
		cdt[n++] = t / 10 + '0';
		t %= 10;
	}
	cdt[n++] = t + '0';
	cdt[n++] = ':';
	pt %= 3600;
	t = pt / 60;
	if (t >= 10)
	{
		cdt[n++] = t / 10 + '0';
		t %= 10;
	}
	cdt[n++] = t + '0';
	cdt[n++] = ':';
	t = pt % 60;
	if (t >= 10)
	{
		cdt[n++] = t / 10 + '0';
		t %= 10;
	}
	cdt[n++] = t + '0';
	for (; n < 8; n++)cdt[n] = ' ';
}
void Ydwin::FunV04A(const char* str)
{
	while (char ch = pgm_read_byte(str++)) cdt[n++] = ch;
	for (; n < 8; n++)cdt[n] = ' ';
}
void Ydwin::dwinSendCmdFromString(const char*tar) 
{
	char c, ch, tt = 0;
	for (; (*tar >= 'A'&&*tar <= 'F') || (*tar >= 'a'&&*tar <= 'f') || (*tar >= '0'&&*tar <= '9') || *tar == ' '; tar++)
	{
		MYSERIAL.write(*tar);
		if (*tar == ' ')continue;
		if (*tar >= 'A'&&*tar <= 'F')
			ch = *tar - 'A' + 10;
		else if (*tar >= 'a'&&*tar <= 'f')
			ch = *tar - 'a' + 10;
		else
			ch = *tar - '0';
		if (tt == 0)
		{
			tt = 1;
			c = ch << 4;
		}
		else
		if (tt == 1)
		{
			tt = 2;
			c |= ch;
		}
		if (tt == 2)
		{
			tt = 0;
			FunV00E(c);
		}
	}
}
