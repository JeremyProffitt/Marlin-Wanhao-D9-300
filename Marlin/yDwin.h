#ifndef Y_DWIN_H
#define Y_DWIN_H
#include "Marlin.h"
#define		MACRO_var_V002							0
#define		MACRO_var_V003							1
#define		DWIN_SER_BAUD							115200
#define		DWIN_SER_PORT							2
#define		DWIN_FILENAME_LEN						16 
#define		DWIN_DOT_TEN_MUL						10.0 
#define		MACRO_var_V011	MACRO_var_V003 
#define	MACRO_var_V014					0x80 
#define	DWIN_READ_REG					0x81
#ifdef DWIN_FYS_CUSTOMIZED
#define	MACRO_var_V013					0xCA
#define	MACRO_var_V016					0xA1 
#define	DWIN_READ_VAR					0xA2
#else
#define	MACRO_var_V013					0x5A
#define	MACRO_var_V017					0xA5
#define	MACRO_var_V016					0x82 
#define	DWIN_READ_VAR					0x83
#endif
#if MACRO_var_V011==MACRO_var_V002
#define		MACRO_var_V018						0x40 
#elif MACRO_var_V011==MACRO_var_V003
#define		MACRO_var_V018						0x64 
#endif
#define	_TNAME(X,Y,Z)					X##Y##Z
#define	TNAME(X,Y,Z)					_TNAME(X,Y,Z)
#define DWIN_SERIAL_RX_VECT				TNAME(USART,DWIN_SER_PORT,_RX_vect)
#define DWIN_UCSRxA						TNAME(UCSR,DWIN_SER_PORT,A)
#define DWIN_UCSRxB						TNAME(UCSR,DWIN_SER_PORT,B)
#define DWIN_UCSRxC						TNAME(UCSR,DWIN_SER_PORT,C)
#define DWIN_UBRRxH						TNAME(UBRR,DWIN_SER_PORT,H)
#define DWIN_UBRRxL						TNAME(UBRR,DWIN_SER_PORT,L)
#define DWIN_UDRx						TNAME(UDR,DWIN_SER_PORT,)
#define DWIN_U2Xx						TNAME(U2X,DWIN_SER_PORT,)
#define DWIN_RXENx						TNAME(RXEN,DWIN_SER_PORT,)
#define DWIN_TXENx						TNAME(TXEN,DWIN_SER_PORT,)
#define DWIN_TXCx						TNAME(TXC,DWIN_SER_PORT,)
#define DWIN_RXCIEx						TNAME(RXCIE,DWIN_SER_PORT,)
#define DWIN_UDRIEx						TNAME(UDRIE,DWIN_SER_PORT,)
#define DWIN_UDREx						TNAME(UDRE,DWIN_SER_PORT,)
class Ydwin
{
private:
	static inline void FunV00E(uint8_t dat)
	{
		while (!(DWIN_UCSRxA&(1 << DWIN_UDREx)));
		DWIN_UDRx = dat;
	}
	static bool waitData(uint8_t*dat);
#if MACRO_var_V011==MACRO_var_V002
	static void FunV016(const uint8_t& regAddr, const uint8_t *data, uint8_t len);
	static bool FunV017(const uint8_t& regAddr, uint8_t*data, uint8_t len);
	static void FunV017(const uint8_t& varAddr, uint8_t len);
#endif
	static void FunV01A(const uint16_t& varAddr, const uint16_t& data);
	static void FunV01D(const uint16_t& varAddr, uint8_t len = 2);
public:
	volatile static bool GLOBAL_var_V006;
	static uint8_t dwinDataLen;
	static uint16_t dwinAddr;
	static uint8_t dwinData[128];
	static void FunV001();
	static void FunV002();
	static void FunV012(u8 dat);
	static void FunV01A(const uint16_t& varAddr, const uint8_t *data, uint8_t len = 2);
	static bool FunV01D(const uint16_t& varAddr, uint8_t*data, uint8_t len = 2);
	static void FunV01E();
	static void FunV022(uint8_t val);
	static void FunV024(uint16_t page);
	static void FunV028(uint8_t id, uint8_t segments, uint8_t volume);
	static void FunV015(const uint16_t& dwinAddr, const char*str, uint8_t len = 0);
	static void dwinPutsPGM(const uint16_t& dwinAddr, const char*str, uint8_t len = 0);
	static uint16_t FunV025();
	static void dwinSendCmdFromString(const char*tar); 
private:
	uint8_t n;
	uint8_t cdt[39];
	uint16_t cAddr;
	inline void FunV02A(){ CBI(DWIN_UCSRxB, DWIN_RXCIEx); }
	inline void FunV02B(){ SBI(DWIN_UCSRxB, DWIN_RXCIEx); }
public:
	inline void FunV02C(const uint16_t& dwinAddr)
	{
		cAddr = dwinAddr;
		memset(cdt, 0, 39);
		n = 0;
	}
	inline void FunV02E(uint8_t jump=0)
	{
		FunV01A(cAddr, cdt, n);
		memset(cdt, 0, n);
		if (jump)cAddr += jump;
		else cAddr += (n >> 1);
		n = 0;
	}
	inline bool FunV027(uint8_t num)
	{
		bool ifGet=FunV01D(cAddr, cdt, num);
		cAddr += (num >> 1);
		n = 0;
		return ifGet;
	}
	inline void dwinCmdClear(){ memset(cdt, 0, 39);}
	inline void FunV026(uint8_t byteNum){ n += byteNum; }
	inline void dwinPut16(const int16_t& r)
	{
		cdt[n++] = (r >> 8) & 0xFF;
		cdt[n++] = (uint8_t)r;
	}
	inline void FunV032(const float& r)
	{
		uint16_t t = r* DWIN_DOT_TEN_MUL;
		cdt[n++] = (t >> 8) & 0xFF;
		cdt[n++] = (uint8_t)t;
	}
	inline void FunV037(const int32_t& r)
	{
		cdt[n++] = (r >> 24) & 0xFF;
		cdt[n++] = (r >> 16) & 0xFF;
		cdt[n++] = (r >> 8) & 0xFF;
		cdt[n++] = (uint8_t)r;
	}
	inline void FunV03D(const float& r)
	{
		int32_t t = r*DWIN_DOT_TEN_MUL;
		FunV037(t);
	}
	inline void FunV03E(int16_t& r)
	{
		r = cdt[n++];
		r = ((r << 8) | cdt[n++]);
	}
	inline void FunV043(float& r)
	{
		int16_t t;
		FunV03E(t);
		r = t / DWIN_DOT_TEN_MUL;
	}
	inline void FunV044(int32_t& r)
	{
		r = cdt[n++];
		r = ((r << 8) | cdt[n++]);
		r = ((r << 8) | cdt[n++]);
		r = ((r << 8) | cdt[n++]);
	}
	inline void FunV047(float& r)
	{
		int32_t t;
		FunV044(t);
		r = t / DWIN_DOT_TEN_MUL;
	}
	void FunV049(millis_t& pt);
	void FunV04A(const char* str);
public:
	Ydwin():n(0){}
	~Ydwin(){ }
};
#endif 
