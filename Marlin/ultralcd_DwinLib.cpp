#include "temperature.h"
#include "ultralcd.h"
#include "yDwinAddrLayout.h"
#include "Marlin.h"
#include "language.h"
#include "cardreader.h"
#include "temperature.h"
#include "stepper.h"
#include "configuration_store.h"
#include <avr/wdt.h>
typedef void(*generalVoidFun)();
static uint8_t GLOBAL_var_V00A = 0x00;
#define	MACRO_var_V05A	0x01	
#define	MACRO_var_V05B		0x02	
#define	MACRO_var_V059			0x04	
#define	MACRO_var_V05C			0x08	
#define	MACRO_var_V05D			0x10	
#define	MACRO_var_V05E		0x20	
char tempChoice = 0;
char optionId = 0;
uint16_t GLOBAL_var_V00C = 0, GLOBAL_var_V00D = 0;
uint16_t file_num = 0;
int16_t lcd_preheat_hotend_temp[2] = { PREHEAT_1_TEMP_HOTEND, PREHEAT_2_TEMP_HOTEND },
lcd_preheat_bed_temp[2] = { PREHEAT_1_TEMP_BED, PREHEAT_2_TEMP_BED },
lcd_preheat_fan_speed[2] = { PREHEAT_1_FAN_SPEED, PREHEAT_2_FAN_SPEED };
float movDis = 5.0, feedrate = 0.0;
generalVoidFun periodFun = nullptr;
static Ydwin myDwin;
static void FunV03C();
static void FunV036();
static void FunV035();
static void FunV040();
static void FunV020();
static void FunV021();
#if HOTENDS > 1
static void FunV034();
static void FunV033();
static void FunV039();
static void FunV048();
static void FunV045();
static void FunV04B();
#endif
static void FunV038();
static void FunV04C();
static void FunV046();
static void FunV03A();
static void FunV030();
static void FunV02F();
static void FunV02D();
static void FunV041();
static void FunV054();
static void FunV055();
static void FunV01F(int16_t s);
static void FunV01B();
static void FunV014(millis_t& tNow);
static void FunV042();
static void dwinCheck();
static void dwinExeCmd(millis_t& tNow);
void lcd_init()
{
#if defined (SDSUPPORT) && PIN_EXISTS(SD_DETECT)
	pinMode(SD_DETECT_PIN, INPUT);
	WRITE(SD_DETECT_PIN, HIGH);
#endif
	Ydwin::FunV001();
	FunV029(); 
#if defined (SDSUPPORT) && PIN_EXISTS(SD_DETECT)
	if (READ(SD_DETECT_PIN) == 0)
	{
		GLOBAL_var_V00A |= MACRO_var_V05A;
	}
	else
	{
	}
	file_num = 0;
#endif
#if DWINPAGE_EXIST(MAIN)
	GLOBAL_var_V00D = DWINPAGE(MAIN);
#endif
	GLOBAL_var_V001 |= ((uint16_t)0x0001 << MACRO_var_V004);
	Ydwin::FunV015(VARADDR_VERSION_DATE, __DATE__, ATTACH_STR_LEN);
}
void lcd_update()
{
	millis_t t = millis();
	FunV042();
	dwinCheck();
	dwinExeCmd(t);
	FunV014(t);
}
static void FunV042()
{
	if (GLOBAL_var_V001 == 0x0000)return;
	uint8_t n;
	char sdFileName[DWIN_FILENAME_LEN],*t;
	for (n = 0; n < 16; n++)
	{
		if (GLOBAL_var_V001&(1 << n))
		{
			GLOBAL_var_V001 &= ~(uint16_t)(1 << n);
			break;
		}
	}
	switch (n)
	{
	case MACRO_var_V005:
		#if DWINPAGE_EXIST(INFO_POPUP)
		if (GLOBAL_var_V00C != DWINPAGE(INFO_POPUP))
			lcd_setPage(DWINPAGE(ASK_RESUM_PRINT));
		#endif
		break;
	case MACRO_var_V006:
		if (card.longFilename[0])strncpy(sdFileName, card.longFilename, DWIN_FILENAME_LEN);
		else strncpy(sdFileName, card.filename, DWIN_FILENAME_LEN);
		t = strchr(sdFileName, '.');
		while (*t)*t++ = '\0';
		Ydwin::FunV015(MACRO_var_V040, sdFileName, DWIN_FILENAME_LEN);
		break;
	case MACRO_var_V05F:
		GLOBAL_var_V00A &= ~MACRO_var_V05D;
	case MACRO_var_V004:
		GLOBAL_var_V00A |= MACRO_var_V059;
		FunV020();
		break;
	case MACRO_var_V024:
		#if DWINPAGE_EXIST(AUTO_LEVELING_COMPLETE)&&!DWINPAGE_EXIST(AUTO_LEVELING)
		FunV038();
		lcd_setPage(DWINPAGE(AUTO_LEVELING_COMPLETE));
		#endif
		disable_all_steppers();
		break;
	case MACRO_VAR_V058:
		#if DWINPAGE_EXIST(MAIN)
		lcd_setPage(DWINPAGE(MAIN));
		#endif
		break;
	case LCDEVT_M1104_NEED_ADJUST:
		break;
	}
}
static void dwinCheck()
{
#if defined (SDSUPPORT) && PIN_EXISTS(SD_DETECT)
	bool sdNow = (READ(SD_DETECT_PIN) == 0);
	bool sdOld = GLOBAL_var_V00A&MACRO_var_V05A;
	if (sdNow != sdOld)
	{
		if (sdNow)
		{
			GLOBAL_var_V00A |= MACRO_var_V05A;
			SERIAL_ECHOPGM("SD card inserted.");
			card.initsd();
			FunV006("SD card inserted.");
		}
		else
		{
			GLOBAL_var_V00A &= ~MACRO_var_V05A;
			SERIAL_ECHOPGM("SD card removed.");
			card.release();
			FunV006("SD card removed.");
		}
		file_num = 0;
	}
#endif
}
static void FunV014(millis_t& tNow)
{
	static millis_t period = 1000;
	if ((GLOBAL_var_V00A&MACRO_var_V05B) && (commands_in_queue < BUFSIZE))
	{
		enqueue_and_echo_commands_P(PSTR("M500"));
		GLOBAL_var_V00A &= ~MACRO_var_V05B;
	}
	if (tNow > period || (GLOBAL_var_V00A&MACRO_var_V059))
	{
		if (periodFun)periodFun();
		millis_t distance = 0;
#if defined(DEFAULT_ACTIVE_TIME_OVER)
		if (tNow<previous_cmd_ms)
		{
			previous_cmd_ms = tNow;
			distance = max_inactive_time;
		}
		else
		{
			distance = tNow - previous_cmd_ms;
			if (distance > max_inactive_time)distance = 0;
			else distance = max_inactive_time - distance;
		}
		if (distance<21000)
		{
			if (!(GLOBAL_var_V00A&MACRO_var_V05E))
			{
				GLOBAL_var_V00A |= MACRO_var_V05E;
				FunV052();
			}
		}
		else if (GLOBAL_var_V00A&MACRO_var_V05E)
		{
			GLOBAL_var_V00A &= ~MACRO_var_V05E;
			FunV052();
		}
#endif
		FunV01F(distance / 1000);
		period = tNow + 1000;
		GLOBAL_var_V00A &= ~MACRO_var_V059;
	}
#if defined(MACRO_var_V03E)&&MACRO_var_V03E>0
	static millis_t prompt = 200;
	if (tNow > prompt)
	{
		FunV01B();
		prompt = tNow + 200;
	}
#endif
}
static void dwinSave()
{
	if (commands_in_queue < BUFSIZE)
	{
		enqueue_and_echo_commands_P(PSTR("M500"));
		GLOBAL_var_V00A &= ~MACRO_var_V05B;
	}
	else 
		GLOBAL_var_V00A |= MACRO_var_V05B;
}
static inline void dwinPid_autoTune()
{
	char str[30];
	FunV021();
	sprintf(str, "M303 E%d C5 S%d U1", active_extruder, thermalManager.target_temperature[active_extruder]);
	enqueue_and_echo_command(str,true);
	GLOBAL_var_V00A |= MACRO_var_V05D;
}
static void moveAxis(AxisEnum axis, float val)
{
	current_position[axis] += val;
	if (current_position[axis] < 0)current_position[axis] = 0;
	planner.buffer_line(current_position[X_AXIS], current_position[Y_AXIS], current_position[Z_AXIS], current_position[E_AXIS], feedrate>0.0 ? feedrate : pgm_read_float(&homing_feedrate_mm_s[axis]), active_extruder);
}
static void manualLevelingMove(float x, float y)
{
	current_position[Z_AXIS] = Z_CLEARANCE_DEPLOY_PROBE;
	planner.buffer_line(current_position[X_AXIS], current_position[Y_AXIS], current_position[Z_AXIS], current_position[E_AXIS], MMM_TO_MMS(HOMING_FEEDRATE_Z), active_extruder);
	current_position[X_AXIS] = x;
	current_position[Y_AXIS] = y;
	planner.buffer_line(current_position[X_AXIS], current_position[Y_AXIS], current_position[Z_AXIS], current_position[E_AXIS], MMM_TO_MMS(HOMING_FEEDRATE_XY), active_extruder);
	current_position[Z_AXIS] = 0;
	planner.buffer_line(current_position[X_AXIS], current_position[Y_AXIS], current_position[Z_AXIS], current_position[E_AXIS], MMM_TO_MMS(HOMING_FEEDRATE_Z), active_extruder);
	GLOBAL_var_V00E = MACRO_VAR_V056;
}
static int16_t FunV058(bool ifHeatBed = true)
{
	int16_t tempe, tempb;
	if (tempChoice < 2)
	{
		tempe = lcd_preheat_hotend_temp[tempChoice];
		tempb = lcd_preheat_bed_temp[tempChoice];
	}
	else
	{
		tempe = 260;
		tempb = 100;
	}
	if (ifHeatBed)thermalManager.setTargetBed(tempb);
	thermalManager.setTargetHotend(tempe, active_extruder);
	#if FAN_COUNT > 0
	if(active_extruder<FAN_COUNT)fanSpeeds[active_extruder] = lcd_preheat_fan_speed[active_extruder];
	#endif
	return tempe;
}
static void FunV057()
{
	int16_t tempe = FunV058(false) - 5;
	if (!planner.is_full() && thermalManager.degHotend(active_extruder) > tempe)
	{
		current_position[E_AXIS] += 100.0;
		planner.buffer_line(current_position[X_AXIS], current_position[Y_AXIS], current_position[Z_AXIS], current_position[E_AXIS], 
			feedrate>0 ? feedrate : pgm_read_float(&homing_feedrate_mm_s[E_AXIS]), active_extruder);
	}
}
static void FunV056()
{
	int16_t tempe = FunV058(false) - 5;
	if (!planner.is_full() && thermalManager.degHotend(active_extruder) > tempe)
	{
		current_position[E_AXIS] -= 100.0;
		planner.buffer_line(current_position[X_AXIS], current_position[Y_AXIS], current_position[Z_AXIS], current_position[E_AXIS], 
			feedrate>0 ? feedrate : pgm_read_float(&homing_feedrate_mm_s[E_AXIS]), active_extruder);
	}
}
static void dwinExeCmd(millis_t& tNow)
{
	if (!Ydwin::GLOBAL_var_V006)return;
#if defined(DEFAULT_ACTIVE_TIME_OVER)
	previous_cmd_ms = tNow;
#endif
	Ydwin::GLOBAL_var_V006 = false;
#if DWIN_FILENAME_LEN>LONG_FILENAME_LENGTH
#define	DLCD_FILE_LEN LONG_FILENAME_LENGTH
#else
#define	DLCD_FILE_LEN DWIN_FILENAME_LEN
#endif
	char sdFileName[DLCD_FILE_LEN] = { 0 };
	uint16_t tval = Ydwin::dwinData[0];
	tval = (tval << 8) | Ydwin::dwinData[1];
	SERIAL_ECHOPGM("len:");
	MYSERIAL.print((int)Ydwin::dwinDataLen);
	SERIAL_ECHOPGM(" Addr:");
	for (uint8_t i = 0; i<4;i++)
	{
		char c = (Ydwin::dwinAddr >> ((3-i)<<2)) & 0x0F;
		if (c > 9)MYSERIAL.write(c - 10 + 'A');
		else MYSERIAL.write(c + '0');
	}
	SERIAL_ECHOPGM(" data:");
	for (uint8_t i = 0; i < Ydwin::dwinDataLen; i++)
	{
		char c = (Ydwin::dwinData[i] >> 4) & 0x0F;
		if (c>9)c = c - 10 + 'A';
		else c += '0';
		MYSERIAL.write(c);
		c = Ydwin::dwinData[i] & 0x0F;
		if (c>9)c = c - 10 + 'A';
		else c += '0';
		MYSERIAL.write(c);
		MYSERIAL.write(' ');
	}
	uint8_t cmd[2];
	switch (Ydwin::dwinAddr)
	{
	case MACRO_var_V068:
		switch (tval)
		{
		case MACRO_var_V069:
			FunV020();
			GLOBAL_var_V00A |= MACRO_var_V059;
			#if DWINPAGE_EXIST(TUNE_PID)
			lcd_setPage(DWINPAGE(TUNE_PID));
			#endif
			break;
		case MACRO_var_V06A:
			FunV021();
			break;
		case MACRO_var_V068B:
			FunV021();
			dwinSave();
			break;
		case MACRO_var_V06C:
			enqueue_and_echo_commands_P(PSTR("G28"));
			break;
		case MACRO_var_V06D:
			enqueue_and_echo_commands_P(PSTR("G28 X0"));
			break;
		case MACRO_var_V06E:
			enqueue_and_echo_commands_P(PSTR("G28 Y0"));
			break;
		case MACRO_var_V06F:
			enqueue_and_echo_commands_P(PSTR("G28 Z0"));
			break;
		case VARVAL_TOOL_HOME_XY:
			enqueue_and_echo_commands_P(PSTR("G28 X0 Y0"));
			break;
		case MACRO_var_V070:
			#if PIN_EXISTS(PS_ON)
			GLOBAL_var_V003 = MACRO_var_V00A;
			#endif
			break;
		case MACRO_var_V071:
			for (uint8_t e = 0; e<EXTRUDERS; e++)
				thermalManager.setTargetHotend(0, e);
			thermalManager.setTargetBed(0);
			break;
		case MACRO_var_V072:
			tempChoice = 0;
			FunV058();
			FunV020();
			break;
		case MACRO_var_V073:
			tempChoice = 1;
			FunV058();
			FunV020();
			break;
		case MACRO_var_V074:
			Running = true;
			MYSERIAL.flush();
			SERIAL_ECHOLNPGM("M999 ok.");
			break;
		case MACRO_var_V075:
			thermalManager.setTargetHotend(0, active_extruder);
			break;
		case MACRO_var_V076:
			dwinPid_autoTune();
			break;
		case MACRO_var_V077:
			myDwin.FunV02C(VARADDR_STATUS_AXIS_LOCK);
			if (X_ENABLE_READ == X_ENABLE_ON)
			{
				disable_all_steppers();
				myDwin.FunV026(2);
			}
			else
			{
				enable_all_steppers();
				myDwin.dwinPut16(1);
			}
			myDwin.FunV02E();
			break;
		#if FAN_COUNT>0
		case MACRO_var_V078:
			myDwin.FunV02C(VARADDR_STATUS_FAN);
			if (fanSpeeds[active_extruder]>0)
			{
				fanSpeeds[active_extruder]=0;
				myDwin.FunV026(2);
			}
			else
			{
				fanSpeeds[active_extruder]=255;
				myDwin.dwinPut16(1);
			}
			myDwin.FunV02E();
			break;
		#endif
		#if HAS_SERVOS
		case MACRO_var_V079:
			myDwin.FunV02C(VARADDR_STATUS_SERVO);
			if(GLOBAL_var_V00A&MACRO_var_V05C)
			{
				GLOBAL_var_V00A&=~MACRO_var_V05C;
				servo[0].move(90);
				myDwin.FunV026(2);
			}
			else
			{
				GLOBAL_var_V00A|=MACRO_var_V05C;
				servo[0].move(10);
				myDwin.dwinPut16(1);
			}
			myDwin.FunV02E();
			break;
		case MACRO_var_V07A:
			servo[0].move(160);
			break;
		#endif
		case MACRO_var_V07B:
			#if EXTRUDERS>1
			if (active_extruder + 1 < EXTRUDERS)
			{
				char str[3] = { 'T', active_extruder + '1', 0 };
				enqueue_and_echo_command(str);
			}
			else
			{
				char str[3] = { 'T', '0', 0 };
				enqueue_and_echo_command(str);
			}
			#endif
			break;
		case MACRO_var_V07C:
			wait_for_user = false;
			FunV052();
			break;
		case MACRO_var_V07D:
			if (periodFun == FunV057 || periodFun == FunV056)
			{
				periodFun = nullptr;
				tempChoice = 0;
			}
			stepper.quick_stop();
			thermalManager.setTargetHotend(0, active_extruder);
			FunV020();
			break;
		case MACRO_var_V07E:
			dwin_popup(PSTR("\nLeveling is in progress."),1);
			enqueue_and_echo_commands_P(PSTR("G28"));
			enqueue_and_echo_commands_P(PSTR("G29"));
			break;
		case MACRO_var_V07F:
			break;
		case MACRO_var_V080:
			#if DWINPAGE_EXIST(MANUAL_LEVELING)
			lcd_setPage(DWINPAGE(MANUAL_LEVELING));
			#endif
			enqueue_and_echo_commands_P(PSTR("G28"));
			#if HAS_LEVELING
			reset_bed_level();
			enqueue_and_echo_commands_P(PSTR("M500"));
			#endif
			break;
		#ifdef AUTO_BED_LEVELING_LINEAR
		case MACRO_var_V081:
			manualLevelingMove(X_MIN_POS + MESH_INSET, Y_MIN_POS + MESH_INSET);
			break;
		case MACRO_var_V082:
			manualLevelingMove(X_MAX_POS - MESH_INSET, Y_MIN_POS + MESH_INSET);
			break;
		case MACRO_var_V083:
			manualLevelingMove(X_MAX_POS - MESH_INSET, Y_MAX_POS - MESH_INSET);
			break;
		case MACRO_var_V084:
			manualLevelingMove(X_MIN_POS + MESH_INSET, Y_MAX_POS - MESH_INSET);
			break;
		#endif
		case MACRO_var_V085:
			planner.set_position_mm(current_position[X_AXIS], current_position[Y_AXIS], current_position[Z_AXIS], current_position[E_AXIS]);
			current_position[Z_AXIS] += 1;
			line_to_current_position();
			break;
		case MACRO_var_V086:
			planner.set_position_mm(current_position[X_AXIS], current_position[Y_AXIS], current_position[Z_AXIS], current_position[E_AXIS]);
			current_position[Z_AXIS] -= 1;
			line_to_current_position();
			break;
		case MACRO_var_V087:
			moveAxis(X_AXIS, -movDis);
			break;
		case MACRO_var_V088:
			moveAxis(X_AXIS, movDis);
			break;
		case MACRO_var_V089:
			moveAxis(Y_AXIS, -movDis);
			break;
		case MACRO_var_V08A:
			moveAxis(Y_AXIS, movDis);
			break;
		case MACRO_var_V08B:
			moveAxis(Z_AXIS, -movDis);
			break;
		case MACRO_var_V08C:
			moveAxis(Z_AXIS, movDis);
			break;
		case MACRO_var_V08D:
			moveAxis(E_AXIS, -movDis);
			break;
		case MACRO_var_V08E:
			moveAxis(E_AXIS, movDis);
			break;
		case MACRO_var_V08F:
			break;
		case VARVAL_TOOL_ENTER_PAPER_HEIGHT:
			#if DWINPAGE_EXIST(TOOL_PAPERHEIGHT)
			lcd_setPage(DWINPAGE(TOOL_PAPERHEIGHT));
			#endif
			break;
		case VARVAL_TOOL_RESET:
			wdt_enable(WDTO_30MS);
			while (1) {};
			break;
		case VARVAL_TOOL_OPTION_LEFT:
			FunV052();
			switch (optionId)
			{
			#if ENABLED(ADVANCED_PAUSE_FEATURE)
			case 1:
				advanced_pause_menu_response = ADVANCED_PAUSE_RESPONSE_RESUME_PRINT;
				break;
			#endif
			default:
				break;
			}
			break;
		case VARVAL_TOOL_OPTION_RIGHT:
			FunV052();
			switch (optionId)
			{
			#if ENABLED(ADVANCED_PAUSE_FEATURE)
			case 1:
				advanced_pause_menu_response = ADVANCED_PAUSE_RESPONSE_EXTRUDE_MORE;
				break;
			#endif
			default:
				break;
			}
			break;
		case VARVAL_TOOL_CONNECT_WIFI:
			break;
		}
		break;
	case MACRO_var_V026:
#if ENABLED(SDSUPPORT)
		if (card.cardOK)
		{
			if (tval == MACRO_var_V028 || tval == MACRO_var_V02C || tval == MACRO_var_V02A)
			{
				uint16_t fileCnt = card.getnrfilenames();
				if (file_num<1)file_num = fileCnt;
				card.getWorkDirName();
				switch (tval)
				{
				case MACRO_var_V028:
					#if DWINPAGE_EXIST(FILELIST)
					lcd_setPage(DWINPAGE(FILELIST));
					#endif
					break;
				case MACRO_var_V02A:
					if (file_num > FILES_NUM)
						file_num = file_num - FILES_NUM;
					break;
				case MACRO_var_V02C:
					if (file_num <= (fileCnt - FILES_NUM))
						file_num = file_num + FILES_NUM;
					break;
				}
				for (uint8_t i = 0; i<FILES_NUM; i++)
				{
					if (sdFileName[0])memset(sdFileName, 0, DWIN_FILENAME_LEN);
					if ((file_num - i)>0 && (file_num - i) < 60000)
					{
						card.getfilename(file_num - 1 - i);
						strncpy(sdFileName, card.longFilename, DWIN_FILENAME_LEN);
						char*t = strchr(sdFileName, '.');
						while (*t)*t++ = '\0';
					}
					Ydwin::FunV015(VARADDR_FILES_NAME[i], sdFileName, DWIN_FILENAME_LEN);
				}
			}
			else
				switch (tval)
			{
				case MACRO_var_V030:
					GLOBAL_var_V00E = MACRO_var_V021;
					break;
				case MACRO_var_V032:
					GLOBAL_var_V00E = MACRO_var_V020;
					break;
				case MACRO_var_V033:
					GLOBAL_var_V00E = MACRO_var_V022;
					wait_for_heatup = false;
					#if DWINPAGE_EXIST(MAIN)
					lcd_setPage(DWINPAGE(MAIN));
					#endif
					break;
				case MACRO_var_V035:
					FunV03C();
					#if DWINPAGE_EXIST(TUNE)
					lcd_setPage(DWINPAGE(TUNE));
					#endif
					break;
				case MACRO_var_V037:
					GLOBAL_var_V00D = PAGENUM_PRINT;
					GLOBAL_var_V00C = PAGENUM_PRINT;
					FunV036();
					break;
				#ifdef MACRO_var_V010
				case MACRO_var_V03B: 
					#if DWINPAGE_EXIST(PRINT)
					lcd_setPage(DWINPAGE(PRINT));
					#endif
					delay(20);
					gcode_M1101();
					break;
				case MACRO_var_V03D:  
					#if DWINPAGE_EXIST(MAIN)
					lcd_setPage(DWINPAGE(MAIN));
					#endif
					delay(20);
					gcode_M1103();
					break;
				#endif
				#if defined(FILE_PRINT_NEED_CONRIRM)
				case VARVAL_PRINT_CONFIRM:
					card.startFileprint();
					print_job_timer.start();
					#if DWINPAGE_EXIST(PRINT)
					lcd_setPage(DWINPAGE(PRINT));
					#endif
					break;
				#endif
				default:
					for (uint8_t i = 0; i < FILES_NUM; i++)
					if (tval == MACRO_var_V02E[i])
					{
						uint16_t sd_num = file_num - i;
						card.getfilename(sd_num - 1);
						#ifdef MACRO_var_V010
						strcpy(GLOBAL_var_V004, card.filename);
						#endif
						strcpy(sdFileName, card.longFilename);
						if (card.isFileOpen())card.closefile();
						card.openFile(card.filename, true);
						#if !defined(FILE_PRINT_NEED_CONRIRM)
						card.startFileprint();
						print_job_timer.start();
						#endif
						char*t = strchr(sdFileName, '.');
						while (*t)*t++ = '\0';
						Ydwin::FunV015(MACRO_var_V040, sdFileName, DWIN_FILENAME_LEN);
						#if !defined(FILE_PRINT_NEED_CONRIRM)&&DWINPAGE_EXIST(PRINT)
						lcd_setPage(DWINPAGE(PRINT));
						#endif
						break;
					}
					break;
			}
		}
		else
			card.initsd();
#endif
		break;
	case MACRO_var_V044:
		switch (tval)
		{
		case MACRO_var_V045:
			FunV035();
			#if DWINPAGE_EXIST(SETTING_MOTOR)
			lcd_setPage(DWINPAGE(SETTING_MOTOR));
			#endif
			break;
		case MACRO_var_V046:
			FunV040();
			break;
		case MACRO_var_V047:
			FunV040();
			dwinSave();
			break;
		case MACRO_var_V048:
			FunV038();
			#if DWINPAGE_EXIST(SETTING_LEVELING)
			lcd_setPage(DWINPAGE(SETTING_LEVELING));
			#endif
			break;
		case MACRO_var_V049:
			FunV04C();
			break;
		case MACRO_var_V04A:
			FunV04C();
			dwinSave();
			break;
		case MACRO_var_V04B:
			FunV046();
			#if DWINPAGE_EXIST(SETTING_TEMP)
			lcd_setPage(DWINPAGE(SETTING_TEMP));
			#endif
			break;
		case MACRO_var_V04C:
			FunV03A();
			break;
		case MACRO_var_V04D:
			FunV03A();
			dwinSave();
			break;
		case MACRO_var_V04E:
			FunV030();
			#if DWINPAGE_EXIST(SETTING_MATERIAL)
			lcd_setPage(DWINPAGE(SETTING_MATERIAL));
			#endif
			break;
		case MACRO_var_V04F:
			FunV02F();
			break;
		case MACRO_var_V050:
			FunV02F();
			dwinSave();
			break;
		case MACRO_var_V051:
			FunV02D();
			#if DWINPAGE_EXIST(SETTING_TMC2130)
			lcd_setPage(DWINPAGE(SETTING_TMC2130));
			#endif
			break;
		case MACRO_var_V052:
			FunV041();
			break;
		case MACRO_var_V053:
			FunV041();
			dwinSave();
			break;
		case MACRO_var_V054:
			dwinSave();
			break;
		case MACRO_var_V055:
			settings.reset();
			break;
		case MACRO_var_V063:
			settings.reset();
			dwinSave();
			break;
		case MACRO_var_V064:
			FunV054();
			#if DWINPAGE_EXIST(SETTING_SYSTEM)
			lcd_setPage(DWINPAGE(SETTING_SYSTEM));
			#endif
			break;
		case MACRO_var_V065:
			FunV055();
			break;
		case MACRO_var_V066:
			FunV055();
			dwinSave();
			break;
		}
		break;
#if HOTENDS > 1
	case MACRO_var_V091:
		switch (tval)
		{
		case MACRO_var_V092:
			FunV034();
			#if DWINPAGE_EXIST(SETTING_EXTRUDERS_OFFSET)
			lcd_setPage(DWINPAGE(SETTING_EXTRUDERS_OFFSET));
			#endif
			break;
		case MACRO_var_V093:
			FunV033();
			break;
		case MACRO_var_V094:
			FunV033();
			dwinSave();
			break;
		case MACRO_var_V095:
			FunV045();
			#if DWINPAGE_EXIST(SETTING_EXTRUDERS_MOTOR)
			lcd_setPage(DWINPAGE(SETTING_EXTRUDERS_MOTOR));
			#endif
			break;
		case MACRO_var_V096:
			FunV04B();
			break;
		case MACRO_var_V097:
			FunV04B();
			dwinSave();
			break;
		case MACRO_var_V098:
			FunV039();
			#if DWINPAGE_EXIST(SETTING_EXTRUDERS_TEMP)
			lcd_setPage(DWINPAGE(SETTING_EXTRUDERS_TEMP));
			#endif
			break;
		case MACRO_var_V099:
			FunV048();
			break;
		case MACRO_var_V09A:
			FunV048();
			dwinSave();
			break;
		}
		break;
#endif
	case MACRO_var_V09E:
	case MACRO_var_V09F:
		tempChoice = tval - 1;
		if (Ydwin::dwinAddr == MACRO_var_V09E)periodFun = FunV057;
		else periodFun = FunV056;
		break;
	case MACRO_var_V09C:
		movDis = tval;
		movDis /= DWIN_DOT_TEN_MUL;
		myDwin.FunV02C(VARADDR_MOVE_DIS_SIGN);
		myDwin.dwinPut16(tval);
		myDwin.FunV02E();
		break;
	case MACRO_var_V09D:
		feedrate = tval;
		feedrate /= DWIN_DOT_TEN_MUL;
		myDwin.FunV02C(VARADDR_MOVE_SPEED_SIGN);
		myDwin.dwinPut16(tval);
		myDwin.FunV02E();
		break;
	case VARADDR_JUMP_PAGE:
		GLOBAL_var_V00D = tval; 
		GLOBAL_var_V00C = tval;
		SERIAL_ECHOPGM("Page to");
		MYSERIAL.println((int)tval);
		break;
	default:
		break;;
	}
}
#if defined(MACRO_var_V03E)&&MACRO_var_V03E>0
static void FunV01B()
{
	myDwin.FunV02C(MACRO_var_V03E);
	#if HAS_X_MIN 
	if(READ(X_MIN_PIN) ^ X_MIN_ENDSTOP_INVERTING)myDwin.dwinPut16(1);
	else myDwin.FunV026(2);
	#else
	myDwin.FunV026(2);
	#endif
	#if HAS_X_MAX 
	if (READ(X_MAX_PIN) ^ X_MAX_ENDSTOP_INVERTING)myDwin.dwinPut16(1);
	else myDwin.FunV026(2);
	#else
	myDwin.FunV026(2);
	#endif
	#if HAS_Y_MIN 
	if(READ(Y_MIN_PIN) ^ Y_MIN_ENDSTOP_INVERTING)myDwin.dwinPut16(1);
	else myDwin.FunV026(2);
	#else
	myDwin.FunV026(2);
	#endif
	#if HAS_Y_MAX 
	if(READ(Y_MAX_PIN) ^ Y_MAX_ENDSTOP_INVERTING)myDwin.dwinPut16(1);
	else myDwin.FunV026(2);
	#else
	myDwin.FunV026(2);
	#endif
	#if HAS_Z_MIN 
	if(READ(Z_MIN_PIN) ^ Z_MIN_ENDSTOP_INVERTING)myDwin.dwinPut16(1);
	else myDwin.FunV026(2);
	#else
	myDwin.FunV026(2);
	#endif
	#if HAS_Z_MAX 
	if(READ(Z_MAX_PIN) ^ Z_MAX_ENDSTOP_INVERTING)myDwin.dwinPut16(1);
	else myDwin.FunV026(2);
	#else
	myDwin.FunV026(2);
	#endif
	myDwin.FunV026(2);
	static uint8_t dynamicIcon = 0;
	dynamicIcon++;
	if (dynamicIcon > 9)dynamicIcon = 0;
	#if FAN_COUNT > 0 
	if (fanSpeeds[active_extruder] > 0)
		myDwin.dwinPut16(dynamicIcon);
	else
		myDwin.FunV026(2);
	#else
	myDwin.FunV026(2);
	#endif
	if (card.sdprinting)
		myDwin.dwinPut16(dynamicIcon);
	else
		myDwin.FunV026(2);
	myDwin.FunV026(2);
	if (GLOBAL_var_V00A&MACRO_var_V05D)
		myDwin.dwinPut16(dynamicIcon);
	else
		myDwin.FunV026(2);
	myDwin.FunV02E();
}
#endif
static void FunV01F(int16_t s)
{
	uint8_t i;
	myDwin.FunV02C(MACRO_var_V039);
	myDwin.FunV032(thermalManager.degHotend(active_extruder));
	#if FAN_COUNT > 0
	myDwin.dwinPut16(fanSpeeds[active_extruder]);
	#else
	myDwin.FunV026(2);
	#endif
	#if HAS_TEMP_BED
	myDwin.FunV032(thermalManager.degBed());
	#else
	myDwin.FunV026(2);
	#endif
	myDwin.dwinPut16(feedrate_percentage);
	for (i = 0; i < 4;i++)
	{
		myDwin.FunV032(current_position[i]);
	}
	#if ENABLED(SDSUPPORT)
	static int progress = 0;
	if (IS_SD_PRINTING)progress = card.percentDone();
	if (progress> 100)progress = 0;
	myDwin.dwinPut16(progress);
	#else
	myDwin.FunV026(2);
	#endif
	#if defined(DEFAULT_ACTIVE_TIME_OVER)
	myDwin.dwinPut16(s);
	#else
	myDwin.FunV026(2);
	#endif
	myDwin.FunV02E();
	if (IS_SD_PRINTING)
	{
		myDwin.FunV02C(MACRO_var_V03C);
		millis_t pt = print_job_timer.duration();
		float total = float(pt) * float(card.getFileSize()) / float(card.getFilePos());
		static float smoothTotal = 0;
		myDwin.FunV049(pt);
		myDwin.FunV02E(8);
		smoothTotal = (smoothTotal * 999L + total) / 1000L;
		if (isinf(smoothTotal))smoothTotal = total;
		if (pt < 120)
		{
			myDwin.FunV04A(PSTR("Unknown."));
			smoothTotal = total;
		}
		else
		{
			pt = smoothTotal;
			myDwin.FunV049(pt);
		}
		myDwin.FunV02E();
	}
}
static void FunV020()
{
	myDwin.FunV02C(MACRO_var_V03A);
	int n = active_extruder;
	myDwin.dwinPut16(n);
	myDwin.dwinPut16(thermalManager.target_temperature[active_extruder]);
	myDwin.FunV03D(PID_PARAM(Kp, active_extruder));
	myDwin.FunV03D(unscalePID_i(PID_PARAM(Ki, active_extruder)));
	myDwin.FunV03D(unscalePID_d(PID_PARAM(Kd, active_extruder)));
	#if ENABLED(PID_EXTRUSION_SCALING)
	myDwin.FunV03D(PID_PARAM(Kc, active_extruder));
	#else
	myDwin.FunV026(4);
	#endif
	#if HAS_TEMP_BED
	myDwin.dwinPut16(thermalManager.target_temperature_bed);
	#else
	myDwin.FunV026(2);
	#endif
	myDwin.FunV02E();
}
static void FunV021()
{
	myDwin.FunV02C(MACRO_var_V03A);
	if (myDwin.FunV027(22))
	{
		myDwin.FunV026(2);
		int16_t t;
		myDwin.FunV03E(t);
		myDwin.FunV047(PID_PARAM(Kp, active_extruder));
		myDwin.FunV047(PID_PARAM(Ki, active_extruder));
		PID_PARAM(Ki, active_extruder) = scalePID_i(PID_PARAM(Ki, active_extruder));
		myDwin.FunV047(PID_PARAM(Kd, active_extruder));
		PID_PARAM(Kd, active_extruder) = scalePID_d(PID_PARAM(Kd, active_extruder));
	#if ENABLED(PID_EXTRUSION_SCALING)
		myDwin.FunV047(PID_PARAM(Kc, active_extruder));
	#else
		myDwin.FunV026(4);
	#endif
		thermalManager.setTargetHotend(t, active_extruder);
		myDwin.FunV03E(t);
		thermalManager.setTargetBed(t);
	}
}
static void FunV03C()
{
	uint8_t e;
	myDwin.FunV02C(MACRO_var_V027);
	#if HAS_TEMP_0
	myDwin.dwinPut16(thermalManager.target_temperature[0]);
	#else
	myDwin.FunV026(2);
	#endif
	#if HAS_TEMP_1
	myDwin.dwinPut16(thermalManager.target_temperature[1]);
	#else
	myDwin.FunV026(2);
	#endif
	#if HAS_TEMP_2
	myDwin.dwinPut16(thermalManager.target_temperature[2]);
	#else
	myDwin.FunV026(2);
	#endif
	#if HAS_TEMP_3
	myDwin.dwinPut16(thermalManager.target_temperature[3]);
	#else
	myDwin.FunV026(2);
	#endif
	#if HAS_TEMP_4
	myDwin.dwinPut16(thermalManager.target_temperature[4]);
	#else
	myDwin.FunV026(2);
	#endif
	#if HAS_TEMP_BED
	myDwin.dwinPut16(thermalManager.target_temperature_bed);
	#else
	myDwin.FunV026(2);
	#endif
	#if FAN_COUNT > 0
	for(e=0;e<FAN_COUNT;e++)
	{
		myDwin.dwinPut16(fanSpeeds[e]);
	}
	for (; e<5; e++)myDwin.FunV026(2);
	#else
	myDwin.FunV026(10);
	#endif
	myDwin.dwinPut16(feedrate_percentage);
	for (e = 0; e < EXTRUDERS; e++)
	{
		myDwin.dwinPut16(flow_percentage[e]);
	}
	for (; e < 5; e++)myDwin.FunV026(2);
	#if PIN_EXISTS(PS_ON)
	if (GLOBAL_var_V005)myDwin.dwinPut16(4);
	else myDwin.dwinPut16(5);
	#else
	myDwin.FunV026(2);
	#endif
	#ifdef DEFAULT_ENERGY_CONSERVE_HEIGHT
	if (ifEnergyConserve)myDwin.dwinPut16(4);
	else myDwin.dwinPut16(5);
	#else
	myDwin.FunV026(2);
	#endif
	myDwin.FunV02E();
}
static void FunV036()
{
	int16_t t;
	myDwin.FunV02C(MACRO_var_V027);
	if (myDwin.FunV027(38))
	{
		#if HAS_TEMP_0
		myDwin.FunV03E(t);
		thermalManager.setTargetHotend(t, 0);
		#else
		myDwin.FunV026(2);
		#endif
		#if HAS_TEMP_1
		myDwin.FunV03E(t);
		thermalManager.setTargetHotend(t, 1);
		#else
		myDwin.FunV026(2);
		#endif
		#if HAS_TEMP_2
		myDwin.FunV03E(t);
		thermalManager.setTargetHotend(t,2);
		#else
		myDwin.FunV026(2);
		#endif
		#if HAS_TEMP_3
		myDwin.FunV03E(t);
		thermalManager.setTargetHotend(t,3);
		#else
		myDwin.FunV026(2);
		#endif
		#if HAS_TEMP_4
		myDwin.FunV03E(t);
		thermalManager.setTargetHotend(t, 4);
		#else
		myDwin.FunV026(2);
		#endif
		#if HAS_TEMP_BED
		myDwin.FunV03E(t);
		thermalManager.setTargetBed(t);
		#else
		myDwin.FunV026(2);
		#endif
		uint8_t e;
		#if FAN_COUNT > 0
		for (e = 0; e<FAN_COUNT; e++)
		{
			myDwin.FunV03E(fanSpeeds[e]);
		}
		for (; e<5; e++)myDwin.FunV026(2);
		#else
		myDwin.FunV026(10);
		#endif
		myDwin.FunV03E(feedrate_percentage);
		for (e = 0; e < EXTRUDERS; e++)
		{
			myDwin.FunV03E(flow_percentage[e]);
		}
		for (; e<5; e++)myDwin.FunV026(2);
		int16_t n;
		#if PIN_EXISTS(PS_ON)
		myDwin.FunV03E(n);
		if (n == 4)GLOBAL_var_V005 = true;
		else GLOBAL_var_V005 = false;
		#endif
		#ifdef DEFAULT_ENERGY_CONSERVE_HEIGHT
		myDwin.FunV03E(n);
		if (n == 4)ifEnergyConserve = true;
		else ifEnergyConserve = false;
		#endif
	}
}
static void FunV035()
{
	myDwin.FunV02C(MACRO_var_V029);
	for (uint8_t i = 0; i < 4; i++) 
	{
		myDwin.FunV03D(planner.axis_steps_per_mm[i]);
	}
	for (uint8_t i = 0; i < 4; i++) 
	{
		myDwin.FunV03D(planner.max_feedrate_mm_s[i]);
	}
	myDwin.FunV02E();
	for (uint8_t i = 0; i < 4; i++) 
	{
		myDwin.FunV037((int32_t)planner.max_acceleration_mm_per_s2[i]);
	}
	myDwin.FunV03D(planner.acceleration);
	myDwin.FunV03D(planner.retract_acceleration);
	myDwin.FunV03D(planner.travel_acceleration);
	myDwin.FunV03D(planner.min_feedrate_mm_s);
	myDwin.FunV02E();
	myDwin.FunV037((int32_t)planner.min_segment_time); 
	myDwin.FunV03D(planner.min_travel_feedrate_mm_s);
	for(uint8_t i=0;i<4;i++) 
	{
		myDwin.FunV03D(planner.max_jerk[i]);
	}
	myDwin.FunV02E();
#if HAS_HOME_OFFSET
	#if ENABLED(DELTA)
	myDwin.FunV026(8);
	float f=DELTA_HEIGHT + home_offset[Z_AXIS];
	myDwin.FunV03D(f);
	#else
	for(uint8_t i=0;i<3;i++) 
	{
		myDwin.FunV03D(home_offset[i]);
	}
	#endif
#else
	myDwin.FunV026(12);
#endif
#if HAS_MOTOR_CURRENT_PWM
	for (uint8_t q = 0; q<3;q++) 
	{
		myDwin.FunV037((int32_t)stepper.motor_current_setting[q]);
	}
	myDwin.FunV037(MOTOR_CURRENT_PWM_RANGE);
#else
	myDwin.FunV026(16);
#endif
	myDwin.FunV02E();
}
static void FunV040()
{
	myDwin.FunV02C(MACRO_var_V029);
	if (myDwin.FunV027(32))
	{
		for (uint8_t i = 0; i < 4; i++)
		{
			myDwin.FunV047(planner.axis_steps_per_mm[i]);
		}
		for (uint8_t i = 0; i < 4; i++)
		{
			myDwin.FunV047(planner.max_feedrate_mm_s[i]);
		}
	}
	myDwin.dwinCmdClear();
	if (myDwin.FunV027(32))
	{
		int32_t t;
		for (uint8_t i = 0; i < 4; i++)
		{
			myDwin.FunV044(t);
			planner.max_acceleration_mm_per_s2[i] = t;
		}
		myDwin.FunV047(planner.acceleration);
		myDwin.FunV047(planner.retract_acceleration);
		myDwin.FunV047(planner.travel_acceleration);
		myDwin.FunV047(planner.min_feedrate_mm_s);
	}
	myDwin.dwinCmdClear();
	if (myDwin.FunV027(24))
	{
		int32_t t;
		myDwin.FunV044(t);
		planner.min_segment_time = t;
		myDwin.FunV047(planner.min_travel_feedrate_mm_s);
		myDwin.FunV047(planner.max_jerk[X_AXIS]);
		myDwin.FunV047(planner.max_jerk[Y_AXIS]);
		myDwin.FunV047(planner.max_jerk[Z_AXIS]);
		myDwin.FunV047(planner.max_jerk[E_AXIS]);
	}
	myDwin.dwinCmdClear();
	if (myDwin.FunV027(24))
	{
#if HAS_HOME_OFFSET
	#if ENABLED(DELTA)
		myDwin.FunV026(8);
		myDwin.FunV047(home_offset[Z_AXIS]);
		home_offset[Z_AXIS] -= DELTA_HEIGHT;
	#else
		for (uint8_t i = 0; i < 3; i++)
		{
			myDwin.FunV047(home_offset[i]);
		}
	#endif
#else
		myDwin.FunV026(12);
#endif
#if HAS_MOTOR_CURRENT_PWM
		int32_t t;
		for (uint8_t q = 0; q<3; q++) 
		{
			myDwin.FunV044(t);
			stepper.motor_current_setting[q]=t;
		}
#else
		myDwin.FunV026(12);
#endif
	}
}
#if HOTENDS > 1
static void FunV034()
{
	uint8_t e;
	myDwin.FunV02C(MACRO_var_V034);
#if EXTRUDERS==5
	for (e = 1; e < 4; e++)
	{
		for (uint8_t axis = 0; axis < 3; axis++)
		{
			myDwin.FunV03D(hotend_offset[axis][e]);
		}
	}
	myDwin.FunV02E(); 
	for (uint8_t axis = 0; axis < 3; axis++)
	{
		myDwin.FunV03D(hotend_offset[axis][4]);
	}
	myDwin.FunV02E(); 
#else
	for (e = 1; e < EXTRUDERS; e++)
	{
		for (uint8_t axis = 0; axis < 3; axis++)
		{
			myDwin.FunV03D(hotend_offset[axis][e]);
		}
	}
	myDwin.FunV02E();
#endif
}
static void FunV033()
{
	myDwin.FunV02C(MACRO_var_V034);
#if EXTRUDERS==5
	if (myDwin.FunV027(36))
	{
		for (uint8_t e = 1; e < 4; e++)
		for (uint8_t axis = 0; axis < 3; axis++)
		{
			myDwin.FunV047(hotend_offset[axis][e]);
		}
	}
	myDwin.dwinCmdClear();
	if (myDwin.FunV027(12))
	{
		for (uint8_t axis = 0; axis < 3; axis++)
		{
			myDwin.FunV047(hotend_offset[axis][4]);
		}
	}
#else
	if (myDwin.FunV027((EXTRUDERS - 1) * 12))
	{
		for (uint8_t e = 1; e < EXTRUDERS; e++)
		for (uint8_t axis = 0; axis < 3; axis++)
		{
			myDwin.FunV047(hotend_offset[axis][e]);
		}
	}
#endif
}
static void FunV045()
{
	myDwin.FunV02C(MACRO_var_V036);
#if EXTRUDERS==5
	for (uint8_t e = 1; e < 4; e++)
	{
		myDwin.FunV03D(planner.axis_steps_per_mm[3 + e]);
		myDwin.FunV03D(planner.max_feedrate_mm_s[3 + e]);
		myDwin.FunV03D(planner.max_acceleration_mm_per_s2[3 + e]);
	}
	myDwin.FunV02E();
	myDwin.FunV03D(planner.axis_steps_per_mm[7]);
	myDwin.FunV03D(planner.max_feedrate_mm_s[7]);
	myDwin.FunV03D(planner.max_acceleration_mm_per_s2[7]);
	myDwin.FunV02E();
#else
	for (uint8_t e = 1; e < EXTRUDERS; e++)
	{
		myDwin.FunV03D(planner.axis_steps_per_mm[3 + e]);
		myDwin.FunV03D(planner.max_feedrate_mm_s[3 + e]);
		myDwin.FunV03D(planner.max_acceleration_mm_per_s2[3 + e]);
	}
	myDwin.FunV02E();
#endif
}
static void FunV04B()
{
	int32_t t;
	myDwin.FunV02C(MACRO_var_V036);
#if EXTRUDERS==5
	if (myDwin.FunV027(36))
	{
		for (uint8_t e = 1; e < 4; e++)
		{
			myDwin.FunV047(planner.axis_steps_per_mm[3 + e]);
			myDwin.FunV047(planner.max_feedrate_mm_s[3 + e]);
			myDwin.FunV044(t);
			planner.max_acceleration_mm_per_s2[3 + e] = t;
		}
	}
	myDwin.dwinCmdClear();
	if (myDwin.FunV027(12))
	{
		myDwin.FunV047(planner.axis_steps_per_mm[7]);
		myDwin.FunV047(planner.max_feedrate_mm_s[7]);
		myDwin.FunV044(t);
		planner.max_acceleration_mm_per_s2[7] = t;
	}
#else
	if (myDwin.FunV027((EXTRUDERS - 1) * 12))
	{
		for (uint8_t e = 1; e < EXTRUDERS; e++)
		{
			myDwin.FunV047(planner.axis_steps_per_mm[3 + e]);
			myDwin.FunV047(planner.max_feedrate_mm_s[3 + e]);
			myDwin.FunV044(t);
			planner.max_acceleration_mm_per_s2[3 + e] = t;
		}
	}
#endif
}
static void FunV039()
{
	uint8_t e;
	myDwin.FunV02C(MACRO_var_V038);
#if EXTRUDERS>3
	for (e = 1; e < 3; e++)
	{
		myDwin.FunV03D(PID_PARAM(Kp, e));
		myDwin.FunV03D(unscalePID_i(PID_PARAM(Ki, e)));
		myDwin.FunV03D(unscalePID_d(PID_PARAM(Kd, e)));
	#if ENABLED(PID_EXTRUSION_SCALING)
		myDwin.FunV03D(PID_PARAM(Kc, e));
	#else
		myDwin.FunV026(4);
	#endif
	}
	myDwin.FunV02E();
	for (; e < EXTRUDERS; e++)
	{
		myDwin.FunV03D(PID_PARAM(Kp, e));
		myDwin.FunV03D(unscalePID_i(PID_PARAM(Ki, e)));
		myDwin.FunV03D(unscalePID_d(PID_PARAM(Kd, e)));
	#if ENABLED(PID_EXTRUSION_SCALING)
		myDwin.FunV03D(PID_PARAM(Kc, e));
	#else
		myDwin.FunV026(4);
	#endif
	}
	myDwin.FunV02E();
#else
	for (e = 1; e < EXTRUDERS; e++)
	{
		myDwin.FunV03D(PID_PARAM(Kp, e));
		myDwin.FunV03D(unscalePID_i(PID_PARAM(Ki, e)));
		myDwin.FunV03D(unscalePID_d(PID_PARAM(Kd, e)));
		#if ENABLED(PID_EXTRUSION_SCALING)
		myDwin.FunV03D(PID_PARAM(Kc, e));
		#else
		myDwin.FunV026(4);
		#endif
	}
	myDwin.FunV02E();
#endif
}
static void FunV048()
{
	uint8_t e;
	myDwin.FunV02C(MACRO_var_V038);
#if EXTRUDERS>3
	if (myDwin.FunV027(32))
	{
		for (e = 1; e < 3; e++)
		{
			myDwin.FunV047(PID_PARAM(Kp, e));
			myDwin.FunV047(PID_PARAM(Ki, e));
			PID_PARAM(Ki, e) = scalePID_i(PID_PARAM(Ki, e));
			myDwin.FunV047(PID_PARAM(Kd, e));
			PID_PARAM(Kd, e) = scalePID_d(PID_PARAM(Kd, e));
		#if ENABLED(PID_EXTRUSION_SCALING)
			myDwin.FunV047(PID_PARAM(Kc, e));
		#else
			myDwin.FunV026(4);
		#endif
		}
	}
	myDwin.dwinCmdClear();
	if (myDwin.FunV027(32))
	{
		for (; e < EXTRUDERS; e++)
		{
			myDwin.FunV047(PID_PARAM(Kp, e));
			myDwin.FunV047(PID_PARAM(Ki, e));
			PID_PARAM(Ki, e) = scalePID_i(PID_PARAM(Ki, e));
			myDwin.FunV047(PID_PARAM(Kd, e));
			PID_PARAM(Kd, e) = scalePID_d(PID_PARAM(Kd, e));
		#if ENABLED(PID_EXTRUSION_SCALING)
			myDwin.FunV047(PID_PARAM(Kc, e));
		#else
			myDwin.FunV026(4);
		#endif
		}
	}
#else
	if (myDwin.FunV027(16*(EXTRUDERS-1)))
	{
		for (e = 1; e < EXTRUDERS; e++)
		{
			myDwin.FunV047(PID_PARAM(Kp, e));
			myDwin.FunV047(PID_PARAM(Ki, e));
			PID_PARAM(Ki, e) = scalePID_i(PID_PARAM(Ki, e));
			myDwin.FunV047(PID_PARAM(Kd, e));
			PID_PARAM(Kd, e) = scalePID_d(PID_PARAM(Kd, e));
		#if ENABLED(PID_EXTRUSION_SCALING)
			myDwin.FunV047(PID_PARAM(Kc, e));
		#else
			myDwin.FunV026(4);
		#endif
		}
	}
#endif
}
#endif
static void FunV038()
{
	myDwin.FunV02C(MACRO_var_V02B);
#if ENABLED(ENABLE_LEVELING_FADE_HEIGHT)
	myDwin.FunV03D(planner.z_fade_height);
#else
	myDwin.FunV026(4);
#endif
	myDwin.FunV02E();
#if ABL_PLANAR
	for (char i = 0; i < 9; i++)
	{
		myDwin.FunV03D(planner.bed_level_matrix.matrix[i]);
	}
#else
	myDwin.FunV026(36);
#endif
	myDwin.FunV02E();
}
static void FunV04C()
{
	myDwin.FunV02C(MACRO_var_V02B);
	if (myDwin.FunV027(4))
	{
#if ENABLED(ENABLE_LEVELING_FADE_HEIGHT)
		myDwin.FunV047(planner.z_fade_height);
#endif
	}
	if (myDwin.FunV027(36))
	{
#if ABL_PLANAR
		for (char i = 0; i < 9; i++)
		{
			myDwin.FunV047(planner.bed_level_matrix.matrix[i]);
		}
#endif
	}
}
static void FunV046()
{
	myDwin.FunV02C(MACRO_var_V02D);
#if ENABLED(PIDTEMP)
	myDwin.FunV03D(PID_PARAM(Kp, 0));
	myDwin.FunV03D(unscalePID_i(PID_PARAM(Ki, 0)));
	myDwin.FunV03D(unscalePID_d(PID_PARAM(Kd, 0)));
	#if ENABLED(PID_EXTRUSION_SCALING)
	myDwin.FunV03D(PID_PARAM(Kc, 0));
	#else
	myDwin.FunV026(4);
	#endif
#else
	myDwin.FunV026(16);
#endif 
#if DISABLED(PIDTEMPBED)
	myDwin.FunV026(16);
#else
	myDwin.FunV03D(thermalManager.bedKp);
	myDwin.FunV03D(unscalePID_i(thermalManager.bedKi));
	myDwin.FunV03D(unscalePID_d(thermalManager.bedKd));
	myDwin.FunV026(4);
#endif
	myDwin.FunV02E();
	for (char i = 0; i < 2; i++)
	{
		myDwin.dwinPut16(lcd_preheat_hotend_temp[i]);
	}
	for (char i = 0; i < 2; i++)
	{
		myDwin.dwinPut16(lcd_preheat_bed_temp[i]);
	}
	for (char i = 0; i < 2; i++)
	{
		myDwin.dwinPut16(lcd_preheat_fan_speed[i]);
	}
#if ENABLED(PID_EXTRUSION_SCALING)
	myDwin.FunV037((int32_t)lpq_len);
#else
	myDwin.FunV026(4);
#endif
	myDwin.FunV02E();
}
static void FunV03A()
{
	myDwin.FunV02C(MACRO_var_V02D);
	if (myDwin.FunV027(32))
	{
	#if ENABLED(PIDTEMP)
		myDwin.FunV047(PID_PARAM(Kp, 0));
		myDwin.FunV047(PID_PARAM(Ki, 0));
		PID_PARAM(Ki, 0) = scalePID_i(PID_PARAM(Ki, 0));
		myDwin.FunV047(PID_PARAM(Kd, 0));
		PID_PARAM(Kd, 0) = scalePID_d(PID_PARAM(Kd, 0));
		#if ENABLED(PID_EXTRUSION_SCALING)
		myDwin.FunV047(PID_PARAM(Kc, 0));
		#else
		myDwin.FunV026(4);
		#endif
	#else
		myDwin.FunV026(16);
	#endif 
	#if DISABLED(PIDTEMPBED)
		myDwin.FunV026(16);
	#else
		myDwin.FunV047(thermalManager.bedKp);
		myDwin.FunV047(thermalManager.bedKi);
		thermalManager.bedKi = scalePID_i(thermalManager.bedKi);
		myDwin.FunV047(thermalManager.bedKd);
		thermalManager.bedKd = scalePID_d(thermalManager.bedKd);
		myDwin.FunV026(4);
	#endif
	}
	myDwin.dwinCmdClear();
	if (myDwin.FunV027(20))
	{
		for (char i = 0; i < 2; i++)
		{
			myDwin.FunV03E(lcd_preheat_hotend_temp[i]);
		}
		for (char i = 0; i < 2; i++)
		{
			myDwin.FunV03E(lcd_preheat_bed_temp[i]);
		}
		for (char i = 0; i < 2; i++)
		{
			myDwin.FunV03E(lcd_preheat_fan_speed[i]);
		}
	#if ENABLED(PID_EXTRUSION_SCALING)
		myDwin.FunV044((int32_t)lpq_len);
	#else
		myDwin.FunV026(4);
	#endif
	}
}
static void FunV030()
{
	myDwin.FunV02C(MACRO_var_V02F);
#if ENABLED(FWRETRACT)
	myDwin.dwinPut16(autoretract_enabled);
	myDwin.FunV03D(retract_length);
	#if EXTRUDERS > 1
	myDwin.FunV03D(retract_length_swap);
	#else
	myDwin.FunV026(4);
	#endif
	myDwin.FunV03D(retract_feedrate_mm_s);
	myDwin.FunV03D(retract_zlift);
	myDwin.FunV03D(retract_recover_length);
	#if EXTRUDERS > 1
	myDwin.FunV03D(retract_recover_length_swap);
	#else
	myDwin.FunV026(4);
	#endif
	myDwin.FunV03D(retract_recover_feedrate_mm_s);
#else
	myDwin.FunV026(30);
#endif 
	myDwin.dwinPut16(volumetric_enabled);
	myDwin.FunV02E();
	uint8_t e;
	for (e = 0; e<EXTRUDERS; e++)myDwin.FunV03D(filament_size[e]);
	for (; e < 5; e++)myDwin.FunV026(4);
#if ENABLED(LIN_ADVANCE)
	myDwin.FunV03D(planner.extruder_advance_k);
	myDwin.FunV03D(planner.advance_ed_ratio);
#else
	myDwin.FunV026(8);
#endif
	myDwin.FunV02E();
}
static void FunV02F()
{
	myDwin.FunV02C(MACRO_var_V02F);
	if(myDwin.FunV027(32))
	{
		int16_t t;
	#if ENABLED(FWRETRACT)
		myDwin.FunV03E(t);
		autoretract_enabled=t;
		myDwin.FunV047(retract_length);
		#if EXTRUDERS > 1
		myDwin.FunV047(retract_length_swap);
		#else
		myDwin.FunV026(4);
		#endif
		myDwin.FunV047(retract_feedrate_mm_s);
		myDwin.FunV047(retract_zlift);
		myDwin.FunV047(retract_recover_length);
		#if EXTRUDERS > 1
		myDwin.FunV047(retract_recover_length_swap);
		#else
		myDwin.FunV026(4);
		#endif
		myDwin.FunV047(retract_recover_feedrate_mm_s);
	#else
		myDwin.FunV026(30);
	#endif 
		myDwin.FunV03E(t);
		volumetric_enabled = t;
	}
	myDwin.dwinCmdClear();
	if(myDwin.FunV027(28))
	{
		uint8_t e;
		for (e = 0; e<EXTRUDERS; e++)myDwin.FunV047(filament_size[e]);
		for (; e < 5; e++)myDwin.FunV026(4);
	#if ENABLED(LIN_ADVANCE)
		myDwin.FunV047(planner.extruder_advance_k);
		myDwin.FunV047(planner.advance_ed_ratio);
	#endif
	}
}
static void FunV02D()
{
	int16_t t;
	myDwin.FunV02C(MACRO_var_V031);
#if ENABLED(HAVE_TMC2130)
    #if ENABLED(X_IS_TMC2130)
	t = stepperX.getCurrent();
	myDwin.dwinPut16(t);
    #else
	myDwin.FunV026(2);
	#endif
    #if ENABLED(Y_IS_TMC2130)
	t=stepperY.getCurrent();
	myDwin.dwinPut16(t);
    #else
	myDwin.FunV026(2);
    #endif
    #if ENABLED(Z_IS_TMC2130)
	t=stepperZ.getCurrent();
	myDwin.dwinPut16(t);
    #else
	myDwin.FunV026(2);
    #endif
    #if ENABLED(E0_IS_TMC2130)
	t=stepperE0.getCurrent();
	myDwin.dwinPut16(t);
    #else
	myDwin.FunV026(2);
    #endif
    #if ENABLED(X2_IS_TMC2130)
	t=stepperX2.getCurrent();
	myDwin.dwinPut16(t);
    #else
	myDwin.FunV026(2);
    #endif
    #if ENABLED(Y2_IS_TMC2130)
	t=stepperY2.getCurrent();
	myDwin.dwinPut16(t);
    #else
	myDwin.FunV026(2);
    #endif
    #if ENABLED(Z2_IS_TMC2130)
	t=stepperZ2.getCurrent();
	myDwin.dwinPut16(t);
    #else
	myDwin.FunV026(2);
    #endif
    #if ENABLED(E1_IS_TMC2130)
	t=stepperE1.getCurrent();
	myDwin.dwinPut16(t);
    #else
	myDwin.FunV026(2);
    #endif
    #if ENABLED(E2_IS_TMC2130)
	t=stepperE2.getCurrent();
	myDwin.dwinPut16(t);
    #else
	myDwin.FunV026(2);
    #endif
    #if ENABLED(E3_IS_TMC2130)
	t=stepperE3.getCurrent();
	myDwin.dwinPut16(t);
    #else
	myDwin.FunV026(2);
    #endif
    #if ENABLED(E4_IS_TMC2130)
	t=stepperE4.getCurrent();
	myDwin.dwinPut16(t);
    #else
	myDwin.FunV026(2);
    #endif
#else
	myDwin.FunV026(22);
#endif
	myDwin.FunV02E();
}
static void FunV041()
{
	int16_t t;
	myDwin.FunV02C(MACRO_var_V031);
	if (myDwin.FunV027(22))
	{
#if ENABLED(HAVE_TMC2130)
	#if ENABLED(X_IS_TMC2130)
		myDwin.FunV03E(t)
		stepperX.setCurrent(tar, R_SENSE, HOLD_MULTIPLIER);
	#else
		myDwin.FunV026(2);
	#endif
	#if ENABLED(Y_IS_TMC2130)
		myDwin.FunV03E(t)
		stepperY.setCurrent(tar, R_SENSE, HOLD_MULTIPLIER);
	#else
		myDwin.FunV026(2);
	#endif
	#if ENABLED(Z_IS_TMC2130)
		myDwin.FunV03E(t)
		stepperZ.setCurrent(tar, R_SENSE, HOLD_MULTIPLIER);
	#else
		myDwin.FunV026(2);
	#endif
	#if ENABLED(X2_IS_TMC2130)
		myDwin.FunV03E(t)
		stepperX2.setCurrent(tar, R_SENSE, HOLD_MULTIPLIER);
	#else
		myDwin.FunV026(2);
	#endif
	#if ENABLED(Y2_IS_TMC2130)
		myDwin.FunV03E(t)
		stepperY2.setCurrent(tar, R_SENSE, HOLD_MULTIPLIER);
	#else
		myDwin.FunV026(2);
	#endif
	#if ENABLED(Z2_IS_TMC2130)
		myDwin.FunV03E(t)
		stepperZ2.setCurrent(tar, R_SENSE, HOLD_MULTIPLIER);
	#else
		myDwin.FunV026(2);
	#endif
	#if ENABLED(E0_IS_TMC2130)
		myDwin.FunV03E(t)
		stepperE0.setCurrent(tar, R_SENSE, HOLD_MULTIPLIER);
	#else
		myDwin.FunV026(2);
	#endif
	#if ENABLED(E1_IS_TMC2130)
		myDwin.FunV03E(t)
		stepperE1.setCurrent(tar, R_SENSE, HOLD_MULTIPLIER);
	#else
		myDwin.FunV026(2);
	#endif
	#if ENABLED(E2_IS_TMC2130)
		myDwin.FunV03E(t)
		stepperE2.setCurrent(tar, R_SENSE, HOLD_MULTIPLIER);
	#else
		myDwin.FunV026(2);
	#endif
	#if ENABLED(E3_IS_TMC2130)
		myDwin.FunV03E(t)
		stepperE3.setCurrent(tar, R_SENSE, HOLD_MULTIPLIER);
	#else
		myDwin.FunV026(2);
	#endif
	#if ENABLED(E4_IS_TMC2130)
		myDwin.FunV03E(t)
		stepperE4.setCurrent(tar, R_SENSE, HOLD_MULTIPLIER);
	#else
		myDwin.FunV026(2);
	#endif
#else
		myDwin.FunV026(22);
#endif
	}
}
static void FunV054()
{
	myDwin.FunV02C(MACRO_var_V062);
#ifdef DEFAULT_ENERGY_CONSERVE_HEIGHT
	myDwin.FunV032(zEnergyHeight);
#else
	myDwin.FunV026(2);
#endif
#if PIN_EXISTS(PS_ON)&&defined(DEFAULT_ACTIVE_TIME_OVER)
	int16_t t=max_inactive_time/1000UL;
	myDwin.dwinPut16(t);
#else
	myDwin.FunV026(2);
#endif
	myDwin.FunV02E();
}
static void FunV055()
{
	myDwin.FunV02C(MACRO_var_V062);
	if (myDwin.FunV027(4))
	{
#ifdef DEFAULT_ENERGY_CONSERVE_HEIGHT
		myDwin.FunV043(zEnergyHeight);
#else
		myDwin.FunV026(2);
#endif
#if PIN_EXISTS(PS_ON)&&defined(DEFAULT_ACTIVE_TIME_OVER)
		int16_t t;
		myDwin.FunV03E(t);
		max_inactive_time = (millis_t)t* 1000UL;
#else
		myDwin.FunV026(2);
#endif
	}
}
void dwin_popup(const char* msg, char pageChoose, char funid)
{
	char str[INFO_POPUP_LEN + 1], i, j, ch;
	if (msg)
	for (j = 0; j < INFOS_NUM; j++)
	{
		memset(str, 0, INFO_POPUP_LEN);
		i = 0;
		while (ch = pgm_read_byte(msg))
		{
			if (ch == '\n')
			{
				msg++;
				break;
			}
			str[i++] = ch;
			msg++;
			if (i >= INFO_POPUP_LEN)break;
		}
		Ydwin::FunV015(MACRO_var_V0A0[j], str, INFO_POPUP_LEN);
	}
	switch (pageChoose)
	{
	case 0:
		#if DWINPAGE_EXIST(INFO_POPUP)
		if (GLOBAL_var_V00C != DWINPAGE(INFO_POPUP))lcd_popPage(DWINPAGE(INFO_POPUP));
		#endif
		break;
	case 1:
		#if DWINPAGE_EXIST(INFO_WAITING)
		if (GLOBAL_var_V00C != DWINPAGE(INFO_WAITING))lcd_popPage(DWINPAGE(INFO_WAITING));
		#endif
		break;
	case 2:
		switch (funid)
		{
		#if ENABLED(ADVANCED_PAUSE_FEATURE)
		case 1:
			optionId = 1;
			advanced_pause_menu_response = ADVANCED_PAUSE_RESPONSE_WAIT_FOR;
			Ydwin::dwinPutsPGM(VARADDR_QUESTION_LEFT, PSTR("Resume"),8);
			Ydwin::dwinPutsPGM(VARADDR_QUESTION_RIGHT, PSTR("Extrude"),8);
			break;
		#endif
		default:
			break;
		}
		#if DWINPAGE_EXIST(INFO_OPTION)
		if (GLOBAL_var_V00C != DWINPAGE(INFO_OPTION))lcd_popPage(DWINPAGE(INFO_OPTION));
		#endif
		break;
	default:
		break;
	}
}
void FunV052()
{
	if (
		#if DWINPAGE_EXIST(INFO_POPUP)
		GLOBAL_var_V00C == DWINPAGE(INFO_POPUP) || 
		#endif
		#if DWINPAGE_EXIST(INFO_WAITING)
		GLOBAL_var_V00C == DWINPAGE(INFO_WAITING) ||
		#endif
		#if DWINPAGE_EXIST(INFO_OPTION)
		GLOBAL_var_V00C == DWINPAGE(INFO_OPTION) ||
		#endif
		0)lcd_popPage(GLOBAL_var_V00D);
}
void FunV029()
{
	lcd_popPage(0x01);
	Ydwin::FunV028(0x00, 0x02, 0xFF);
	delay(100);
}
void lcd_shutDown()
{
	Ydwin::FunV028(0x05, 0x02, 0xFF);
	lcd_setPage(0x00);
}
void lcd_setstatus(const char* message, const bool persist)
{
	(void)persist;
#ifdef SERIAL_WIFI_ESP3D
	while (*message == ' ')message++;
	const char*t = message;
	bool tf = false;
	char n , d;
	for (n = 0, d = 0; (*t >= '0'&&*t <= '9') || *t == '.'; t++)
	{
		if (*t == '.')
		{
			if (tf)
			{
				d++;
				tf = false;
			}
			else break;
		}
		else
		{
			if (!tf)
			{
				tf = true;
				n++;
			}
		}
	}
	if (n == 4 && d == 3)
	{
		Ydwin::FunV015(VARADDR_WIFI_IP, message, ATTACH_STR_LEN);
		return;
	}
	t=message;
	const char* str = "SSID";
	for (d = 0; *t; t++)
	{
		if (*t == str[0])
		{
			const char *r=t;
			for(d=0;*r&&d<4;r++,d++)
			if (*r != str[d])break;
			if (d == 4)
			{
				t = r;
				t++;
				break;
			}
		}
	}
	if (*t)
	{
		Ydwin::FunV015(VARADDR_WIFI_SSID, t, ATTACH_STR_LEN);
		myDwin.FunV02C(VARADDR_STATUS_WIFI);
		myDwin.dwinPut16(1);
		myDwin.FunV02E();
		return;
	}
#else
	(void)message;
#endif
}
