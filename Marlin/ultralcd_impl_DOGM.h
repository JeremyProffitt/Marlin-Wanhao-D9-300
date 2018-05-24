#ifndef ULTRALCD_IMPL_DOGM_H
#define ULTRALCD_IMPL_DOGM_H
#include "MarlinConfig.h"
#include "ultralcd.h"
#include "ultralcd_st7920_u8glib_rrd.h"
#include "dogm_bitmaps.h"
#include "utility.h"
#include "duration_t.h"
#include <U8glib.h>
#if ENABLED(AUTO_BED_LEVELING_UBL)
  #include "ubl.h"
#endif
#if ENABLED(SHOW_BOOTSCREEN) && ENABLED(SHOW_CUSTOM_BOOTSCREEN)
  #include "_Bootscreen.h"
#endif
#if DISABLED(DISPLAY_CHARSET_ISO10646_1)
  #undef USE_BIG_EDIT_FONT
  #undef USE_SMALL_INFOFONT
#endif
#if ENABLED(USE_SMALL_INFOFONT)
  #include "dogm_font_data_6x9_marlin.h"
  #define FONT_STATUSMENU_NAME u8g_font_6x9
#else
  #define FONT_STATUSMENU_NAME FONT_MENU_NAME
#endif
#include "dogm_font_data_Marlin_symbols.h"   
#define FONT_SPECIAL_NAME Marlin_symbols
#if DISABLED(SIMULATE_ROMFONT)
  #if ENABLED(DISPLAY_CHARSET_ISO10646_1)
    #include "dogm_font_data_ISO10646_1.h"
    #define FONT_MENU_NAME ISO10646_1_5x7
  #elif ENABLED(DISPLAY_CHARSET_ISO10646_PL)
    #include "dogm_font_data_ISO10646_1_PL.h"
    #define FONT_MENU_NAME ISO10646_1_PL_5x7
  #elif ENABLED(DISPLAY_CHARSET_ISO10646_5)
    #include "dogm_font_data_ISO10646_5_Cyrillic.h"
    #define FONT_MENU_NAME ISO10646_5_Cyrillic_5x7
  #elif ENABLED(DISPLAY_CHARSET_ISO10646_KANA)
    #include "dogm_font_data_ISO10646_Kana.h"
    #define FONT_MENU_NAME ISO10646_Kana_5x7
  #elif ENABLED(DISPLAY_CHARSET_ISO10646_GREEK)
    #include "dogm_font_data_ISO10646_Greek.h"
    #define FONT_MENU_NAME ISO10646_Greek_5x7
  #elif ENABLED(DISPLAY_CHARSET_ISO10646_CN)
    #include "dogm_font_data_ISO10646_CN.h"
    #define FONT_MENU_NAME ISO10646_CN
    #define TALL_FONT_CORRECTION 1
  #elif ENABLED(DISPLAY_CHARSET_ISO10646_TR)
    #include "dogm_font_data_ISO10646_1_tr.h"
    #define FONT_MENU_NAME ISO10646_TR
  #elif ENABLED(DISPLAY_CHARSET_ISO10646_CZ)
    #include "dogm_font_data_ISO10646_CZ.h"
    #define FONT_MENU_NAME ISO10646_CZ
  #else 
    #include "dogm_font_data_ISO10646_1.h"
    #define FONT_MENU_NAME ISO10646_1_5x7
  #endif
#else 
  #if DISPLAY_CHARSET_HD44780 == JAPANESE
    #include "dogm_font_data_HD44780_J.h"
    #define FONT_MENU_NAME HD44780_J_5x7
  #elif DISPLAY_CHARSET_HD44780 == WESTERN
    #include "dogm_font_data_HD44780_W.h"
    #define FONT_MENU_NAME HD44780_W_5x7
  #elif DISPLAY_CHARSET_HD44780 == CYRILLIC
    #include "dogm_font_data_HD44780_C.h"
    #define FONT_MENU_NAME HD44780_C_5x7
  #else 
    #include "dogm_font_data_ISO10646_1.h"
    #define FONT_MENU_NAME ISO10646_1_5x7
  #endif
#endif 
#define FONT_STATUSMENU 1
#define FONT_SPECIAL 2
#define FONT_MENU_EDIT 3
#define FONT_MENU 4
#define DOG_CHAR_WIDTH         6
#define DOG_CHAR_HEIGHT        12
#if ENABLED(USE_BIG_EDIT_FONT)
  #define FONT_MENU_EDIT_NAME u8g_font_9x18
  #define DOG_CHAR_WIDTH_EDIT  9
  #define DOG_CHAR_HEIGHT_EDIT 13
  #define LCD_WIDTH_EDIT       14
#else
  #define FONT_MENU_EDIT_NAME FONT_MENU_NAME
  #define DOG_CHAR_WIDTH_EDIT  6
  #define DOG_CHAR_HEIGHT_EDIT 12
  #define LCD_WIDTH_EDIT       22
#endif
#ifndef TALL_FONT_CORRECTION
  #define TALL_FONT_CORRECTION 0
#endif
#define START_COL              0
#if ENABLED(REPRAPWORLD_GRAPHICAL_LCD)
  U8GLIB_ST7920_128X64_4X u8g(LCD_PINS_RS); 
#elif ENABLED(U8GLIB_ST7920)
  U8GLIB_ST7920_128X64_RRD u8g(0); 
#elif ENABLED(CARTESIO_UI)
  #if DOGLCD_MOSI != -1 && DOGLCD_SCK != -1
    U8GLIB_DOGM128_2X u8g(DOGLCD_SCK, DOGLCD_MOSI, DOGLCD_CS, DOGLCD_A0); 
  #else
    U8GLIB_DOGM128_2X u8g(DOGLCD_CS, DOGLCD_A0); 
  #endif
#elif ENABLED(U8GLIB_LM6059_AF)
  U8GLIB_LM6059_2X u8g(DOGLCD_CS, DOGLCD_A0); 
#elif ENABLED(MAKRPANEL) || ENABLED(VIKI2) || ENABLED(miniVIKI)
  U8GLIB_NHD_C12864_2X u8g(DOGLCD_CS, DOGLCD_A0); 
#elif ENABLED(U8GLIB_SSD1306)
  U8GLIB_SSD1306_128X64_2X u8g(U8G_I2C_OPT_NONE | U8G_I2C_OPT_FAST); 
#elif ENABLED(U8GLIB_SH1106)
  U8GLIB_SH1106_128X64_2X u8g(U8G_I2C_OPT_NONE | U8G_I2C_OPT_FAST); 
#elif ENABLED(MINIPANEL)
  U8GLIB_MINI12864_2X u8g(DOGLCD_CS, DOGLCD_A0); 
#else
  U8GLIB_DOGM128_2X u8g(DOGLCD_CS, DOGLCD_A0);  
#endif
#ifndef LCD_PIXEL_WIDTH
  #define LCD_PIXEL_WIDTH 128
#endif
#ifndef LCD_PIXEL_HEIGHT
  #define LCD_PIXEL_HEIGHT 64
#endif
#include "utf_mapper.h"
uint16_t lcd_contrast;
static char currentfont = 0;
u8g_page_t &page = ((u8g_pb_t *)((u8g.getU8g())->dev->dev_mem))->p;
#define PAGE_UNDER(yb) (u8g.getU8g()->current_page.y0 <= (yb))
#define PAGE_CONTAINS(ya, yb) (PAGE_UNDER(yb) && u8g.getU8g()->current_page.y1 >= (ya))
static void lcd_setFont(const char font_nr) {
  switch (font_nr) {
    case FONT_STATUSMENU : {u8g.setFont(FONT_STATUSMENU_NAME); currentfont = FONT_STATUSMENU;}; break;
    case FONT_MENU       : {u8g.setFont(FONT_MENU_NAME); currentfont = FONT_MENU;}; break;
    case FONT_SPECIAL    : {u8g.setFont(FONT_SPECIAL_NAME); currentfont = FONT_SPECIAL;}; break;
    case FONT_MENU_EDIT  : {u8g.setFont(FONT_MENU_EDIT_NAME); currentfont = FONT_MENU_EDIT;}; break;
    break;
  }
}
void lcd_print(const char c) {
  if (WITHIN(c, 1, LCD_STR_SPECIAL_MAX)) {
    u8g.setFont(FONT_SPECIAL_NAME);
    u8g.print(c);
    lcd_setFont(currentfont);
  }
  else charset_mapper(c);
}
char lcd_print_and_count(const char c) {
  if (WITHIN(c, 1, LCD_STR_SPECIAL_MAX)) {
    u8g.setFont(FONT_SPECIAL_NAME);
    u8g.print(c);
    lcd_setFont(currentfont);
    return 1;
  }
  else return charset_mapper(c);
}
void lcd_print(const char *str) { while (*str) lcd_print(*str++); }
void lcd_printPGM(const char *str) { while (const char c = pgm_read_byte(str)) lcd_print(c), ++str; }
void lcd_print_utf(const char *str, uint8_t n=LCD_WIDTH) {
  char c;
  while (n && (c = *str)) n -= charset_mapper(c), ++str;
}
void lcd_printPGM_utf(const char *str, uint8_t n=LCD_WIDTH) {
  char c;
  while (n && (c = pgm_read_byte(str))) n -= charset_mapper(c), ++str;
}
#if ENABLED(SHOW_BOOTSCREEN)
  #if ENABLED(SHOW_CUSTOM_BOOTSCREEN)
    void lcd_custom_bootscreen() {
      u8g.firstPage();
      do {
        u8g.drawBitmapP(
          (128 - (CUSTOM_BOOTSCREEN_BMPWIDTH))  /2,
          ( 64 - (CUSTOM_BOOTSCREEN_BMPHEIGHT)) /2,
          CEILING(CUSTOM_BOOTSCREEN_BMPWIDTH, 8), CUSTOM_BOOTSCREEN_BMPHEIGHT, custom_start_bmp);
      } while (u8g.nextPage());
    }
  #endif 
  void lcd_bootscreen() {
    static bool show_bootscreen = true;
    if (show_bootscreen) {
      show_bootscreen = false;
      #if ENABLED(START_BMPHIGH)
        constexpr uint8_t offy = 0;
      #else
        constexpr uint8_t offy = DOG_CHAR_HEIGHT;
      #endif
      const uint8_t offx = (u8g.getWidth() - (START_BMPWIDTH)) / 2,
                    txt1X = (u8g.getWidth() - (sizeof(STRING_SPLASH_LINE1) - 1) * (DOG_CHAR_WIDTH)) / 2;
      u8g.firstPage();
      do {
        u8g.drawBitmapP(offx, offy, START_BMPBYTEWIDTH, START_BMPHEIGHT, start_bmp);
        lcd_setFont(FONT_MENU);
        #ifndef STRING_SPLASH_LINE2
          u8g.drawStr(txt1X, u8g.getHeight() - (DOG_CHAR_HEIGHT), STRING_SPLASH_LINE1);
        #else
          const uint8_t txt2X = (u8g.getWidth() - (sizeof(STRING_SPLASH_LINE2) - 1) * (DOG_CHAR_WIDTH)) / 2;
          u8g.drawStr(txt1X, u8g.getHeight() - (DOG_CHAR_HEIGHT) * 3 / 2, STRING_SPLASH_LINE1);
          u8g.drawStr(txt2X, u8g.getHeight() - (DOG_CHAR_HEIGHT) * 1 / 2, STRING_SPLASH_LINE2);
        #endif
	  } while (u8g.nextPage());
    }
  }
#endif 
static void lcd_implementation_init() {
  #if PIN_EXISTS(LCD_BACKLIGHT) 
    OUT_WRITE(LCD_BACKLIGHT_PIN, HIGH);
  #endif
  #if PIN_EXISTS(LCD_RESET)
    OUT_WRITE(LCD_RESET_PIN, LOW); 
    _delay_ms(5);
    OUT_WRITE(LCD_RESET_PIN, HIGH);
    _delay_ms(5); 
    u8g.begin(); 
  #endif
#if HAS_LCD_CONTRAST
	u8g.setContrast(lcd_contrast);
#else 
  #if DISABLED(MINIPANEL) 
    u8g.setContrast(lcd_contrast);
  #endif
#endif
  #if ENABLED(LCD_SCREEN_ROT_90)
    u8g.setRot90();   
  #elif ENABLED(LCD_SCREEN_ROT_180)
    u8g.setRot180();  
  #elif ENABLED(LCD_SCREEN_ROT_270)
    u8g.setRot270();  
  #endif
  #if ENABLED(SHOW_BOOTSCREEN)
    #if ENABLED(SHOW_CUSTOM_BOOTSCREEN)
      lcd_custom_bootscreen();
    #else
      lcd_bootscreen();
    #endif
  #endif
}
void lcd_kill_screen() {
  lcd_setFont(FONT_MENU);
  u8g.setPrintPos(0, u8g.getHeight()/4*1);
  lcd_print_utf(lcd_status_message);
  u8g.setPrintPos(0, u8g.getHeight()/4*2);
  lcd_printPGM(PSTR(MSG_HALTED));
  u8g.setPrintPos(0, u8g.getHeight()/4*3);
  lcd_printPGM(PSTR(MSG_PLEASE_RESET));
}
void lcd_implementation_clear() { } 
FORCE_INLINE void _draw_centered_temp(const int16_t temp, const uint8_t x, const uint8_t y) {
  const uint8_t degsize = 6 * (temp >= 100 ? 3 : temp >= 10 ? 2 : 1); 
  u8g.setPrintPos(x - (18 - degsize) / 2, y); 
  lcd_print(itostr3(temp));
  lcd_printPGM(PSTR(LCD_STR_DEGREE " "));
}
FORCE_INLINE void _draw_heater_status(const uint8_t x, const int8_t heater, const bool blink) {
  #if HAS_TEMP_BED
    bool isBed = heater < 0;
  #else
    const bool isBed = false;
  #endif
  if (PAGE_UNDER(7)) {
    #if HEATER_IDLE_HANDLER
      const bool is_idle = (!isBed ? thermalManager.is_heater_idle(heater) :
      #if HAS_TEMP_BED
        thermalManager.is_bed_idle()
      #else
        false
      #endif
      );
      if (blink || !is_idle)
    #endif
    _draw_centered_temp((isBed ? thermalManager.degTargetBed() : thermalManager.degTargetHotend(heater)) + 0.5, x, 7); }
  if (PAGE_CONTAINS(21, 28))
    _draw_centered_temp((isBed ? thermalManager.degBed() : thermalManager.degHotend(heater)) + 0.5, x, 28);
  if (PAGE_CONTAINS(17, 20)) {
    const uint8_t h = isBed ? 7 : 8,
                  y = isBed ? 18 : 17;
    if (isBed ? thermalManager.isHeatingBed() : thermalManager.isHeatingHotend(heater)) {
      u8g.setColorIndex(0); 
      u8g.drawBox(x + h, y, 2, 2);
      u8g.setColorIndex(1); 
    }
    else {
      u8g.drawBox(x + h, y, 2, 2);
    }
  }
}
FORCE_INLINE void _draw_axis_label(const AxisEnum axis, const char* const pstr, const bool blink) {
  if (blink)
    lcd_printPGM(pstr);
  else {
    if (!axis_homed[axis])
      u8g.print('?');
    else {
      #if DISABLED(DISABLE_REDUCED_ACCURACY_WARNING)
        if (!axis_known_position[axis])
          u8g.print(' ');
        else
      #endif
      lcd_printPGM(pstr);
    }
  }
}
inline void lcd_implementation_status_message(const bool blink) {
  #if ENABLED(STATUS_MESSAGE_SCROLLING)
    static bool last_blink = false;
    const uint8_t slen = lcd_strlen(lcd_status_message);
    const char *stat = lcd_status_message + status_scroll_pos;
    if (slen <= LCD_WIDTH)
      lcd_print_utf(stat);                                      
    else {
      if (status_scroll_pos <= slen - LCD_WIDTH)
        lcd_print_utf(stat);                                    
      else {
        uint8_t chars = LCD_WIDTH;
        if (status_scroll_pos < slen) {                         
          lcd_print_utf(stat);                                  
          chars -= slen - status_scroll_pos;                    
        }
        u8g.print('.');                                         
        if (--chars) {
          if (status_scroll_pos < slen + 1)                     
            --chars, u8g.print('.');
          if (chars) lcd_print_utf(lcd_status_message, chars);  
        }
      }
      if (last_blink != blink) {
        last_blink = blink;
        if (status_scroll_pos < slen) while (!PRINTABLE(lcd_status_message[status_scroll_pos])) status_scroll_pos++;
        if (++status_scroll_pos >= slen + 2) status_scroll_pos = 0;
      }
    }
  #else
    lcd_print_utf(lcd_status_message);
  #endif
}
static void lcd_implementation_status_screen() {
  const bool blink = lcd_blink();
  lcd_setFont(FONT_STATUSMENU);
  if (PAGE_UNDER(STATUS_SCREENHEIGHT + 1)) {
    u8g.drawBitmapP(9, 1, STATUS_SCREENBYTEWIDTH, STATUS_SCREENHEIGHT,
      #if HAS_FAN0
        blink && fanSpeeds[0] ? status_screen0_bmp : status_screen1_bmp
      #else
        status_screen0_bmp
      #endif
    );
  }
  if (PAGE_UNDER(28)) {
    HOTEND_LOOP() _draw_heater_status(5 + e * 25, e, blink);
    #if HOTENDS < 4 && HAS_TEMP_BED
      _draw_heater_status(81, -1, blink);
    #endif
    #if HAS_FAN0
      if (PAGE_CONTAINS(20, 27)) {
        const int16_t per = ((fanSpeeds[0] + 1) * 100) / 256;
        if (per) {
          u8g.setPrintPos(104, 27);
          lcd_print(itostr3(per));
          u8g.print('%');
        }
      }
    #endif
  }
  #if ENABLED(SDSUPPORT)
    if (PAGE_CONTAINS(42 - (TALL_FONT_CORRECTION), 51 - (TALL_FONT_CORRECTION))) {
      u8g.drawBox(42, 42 - (TALL_FONT_CORRECTION), 8, 7);     
      u8g.drawBox(50, 44 - (TALL_FONT_CORRECTION), 2, 5);     
      u8g.drawFrame(42, 49 - (TALL_FONT_CORRECTION), 10, 4);  
      u8g.drawPixel(50, 43 - (TALL_FONT_CORRECTION));         
    }
    #define PROGRESS_BAR_X 54
    #define PROGRESS_BAR_WIDTH (LCD_PIXEL_WIDTH - PROGRESS_BAR_X)
    if (PAGE_CONTAINS(49, 52 - (TALL_FONT_CORRECTION)))       
      u8g.drawFrame(
        PROGRESS_BAR_X, 49,
        PROGRESS_BAR_WIDTH, 4 - (TALL_FONT_CORRECTION)
      );
    if (IS_SD_PRINTING) {
      if (PAGE_CONTAINS(50, 51 - (TALL_FONT_CORRECTION)))     
        u8g.drawBox(
          PROGRESS_BAR_X + 1, 50,
          (uint16_t)((PROGRESS_BAR_WIDTH - 2) * card.percentDone() * 0.01), 2 - (TALL_FONT_CORRECTION)
        );
      #if ENABLED(DOGM_SD_PERCENT)
        if (PAGE_CONTAINS(41, 48)) {
          u8g.setPrintPos(55, 48);
          u8g.print(itostr3(card.percentDone()));
          u8g.print('%');
        }
      #endif
    }
    #if DISABLED(DOGM_SD_PERCENT)
      #define SD_DURATION_X (PROGRESS_BAR_X + (PROGRESS_BAR_WIDTH / 2) - len * (DOG_CHAR_WIDTH / 2))
    #else
      #define SD_DURATION_X (LCD_PIXEL_WIDTH - len * DOG_CHAR_WIDTH)
    #endif
	if (PAGE_CONTAINS(41, 48)) {
      char buffer[10];
	  duration_t elapsed = print_job_timer.duration();
      bool has_days = (elapsed.value > 60*60*24L);
      uint8_t len = elapsed.toDigital(buffer, has_days);
	  u8g.setPrintPos(SD_DURATION_X, 48);
      lcd_print(buffer);
    }
  #endif
  #if ENABLED(USE_SMALL_INFOFONT)
    #define INFO_FONT_HEIGHT 7
  #else
    #define INFO_FONT_HEIGHT 8
  #endif
  #define XYZ_BASELINE (30 + INFO_FONT_HEIGHT)
  #define X_LABEL_POS  3
  #define X_VALUE_POS 11
  #define XYZ_SPACING 40
  #if ENABLED(XYZ_HOLLOW_FRAME)
    #define XYZ_FRAME_TOP 29
    #define XYZ_FRAME_HEIGHT INFO_FONT_HEIGHT + 3
  #else
    #define XYZ_FRAME_TOP 30
    #define XYZ_FRAME_HEIGHT INFO_FONT_HEIGHT + 1
  #endif
  static char xstring[5], ystring[5], zstring[7];
  #if ENABLED(FILAMENT_LCD_DISPLAY) && DISABLED(SDSUPPORT)
    static char wstring[5], mstring[4];
  #endif
  if (page.page == 0) {
    strcpy(xstring, ftostr4sign(current_position[X_AXIS]));
    strcpy(ystring, ftostr4sign(current_position[Y_AXIS]));
    strcpy(zstring, ftostr52sp(FIXFLOAT(current_position[Z_AXIS])));
    #if ENABLED(FILAMENT_LCD_DISPLAY) && DISABLED(SDSUPPORT)
      strcpy(wstring, ftostr12ns(filament_width_meas));
      strcpy(mstring, itostr3(100.0 * volumetric_multiplier[FILAMENT_SENSOR_EXTRUDER_NUM]));
    #endif
  }
  if (PAGE_CONTAINS(XYZ_FRAME_TOP, XYZ_FRAME_TOP + XYZ_FRAME_HEIGHT - 1)) {
    #if ENABLED(XYZ_HOLLOW_FRAME)
      u8g.drawFrame(0, XYZ_FRAME_TOP, LCD_PIXEL_WIDTH, XYZ_FRAME_HEIGHT); 
    #else
      u8g.drawBox(0, XYZ_FRAME_TOP, LCD_PIXEL_WIDTH, XYZ_FRAME_HEIGHT);   
    #endif
    if (PAGE_CONTAINS(XYZ_BASELINE - (INFO_FONT_HEIGHT - 1), XYZ_BASELINE)) {
      #if DISABLED(XYZ_HOLLOW_FRAME)
        u8g.setColorIndex(0); 
      #endif
      u8g.setPrintPos(0 * XYZ_SPACING + X_LABEL_POS, XYZ_BASELINE);
      _draw_axis_label(X_AXIS, PSTR(MSG_X), blink);
      u8g.setPrintPos(0 * XYZ_SPACING + X_VALUE_POS, XYZ_BASELINE);
      lcd_print(xstring);
      u8g.setPrintPos(1 * XYZ_SPACING + X_LABEL_POS, XYZ_BASELINE);
      _draw_axis_label(Y_AXIS, PSTR(MSG_Y), blink);
      u8g.setPrintPos(1 * XYZ_SPACING + X_VALUE_POS, XYZ_BASELINE);
      lcd_print(ystring);
      u8g.setPrintPos(2 * XYZ_SPACING + X_LABEL_POS, XYZ_BASELINE);
      _draw_axis_label(Z_AXIS, PSTR(MSG_Z), blink);
      u8g.setPrintPos(2 * XYZ_SPACING + X_VALUE_POS, XYZ_BASELINE);
      lcd_print(zstring);
      #if DISABLED(XYZ_HOLLOW_FRAME)
        u8g.setColorIndex(1); 
      #endif
    }
  }
  if (PAGE_CONTAINS(51 - INFO_FONT_HEIGHT, 49)) {
    lcd_setFont(FONT_MENU);
    u8g.setPrintPos(3, 50);
    lcd_print(LCD_STR_FEEDRATE[0]);
    lcd_setFont(FONT_STATUSMENU);
    u8g.setPrintPos(12, 50);
    lcd_print(itostr3(feedrate_percentage));
    u8g.print('%');
    #if DISABLED(SDSUPPORT) && ENABLED(FILAMENT_LCD_DISPLAY)
      u8g.setPrintPos(56, 50);
      lcd_print(wstring);
      u8g.setPrintPos(102, 50);
      lcd_print(mstring);
      u8g.print('%');
      lcd_setFont(FONT_MENU);
      u8g.setPrintPos(47, 50);
      lcd_print(LCD_STR_FILAM_DIA);
      u8g.setPrintPos(93, 50);
      lcd_print(LCD_STR_FILAM_MUL);
    #endif
  }
#define STATUS_BASELINE (55 + INFO_FONT_HEIGHT)
  if (PAGE_CONTAINS(STATUS_BASELINE - (INFO_FONT_HEIGHT - 1), STATUS_BASELINE)) {
    u8g.setPrintPos(0, STATUS_BASELINE);
    #if ENABLED(FILAMENT_LCD_DISPLAY) && ENABLED(SDSUPPORT)
      if (PENDING(millis(), previous_lcd_status_ms + 5000UL)) {  
        lcd_implementation_status_message(blink);
      }
      else {
        lcd_printPGM(PSTR(LCD_STR_FILAM_DIA));
        u8g.print(':');
        lcd_print(ftostr12ns(filament_width_meas));
        lcd_printPGM(PSTR("  " LCD_STR_FILAM_MUL));
        u8g.print(':');
        lcd_print(itostr3(100.0 * volumetric_multiplier[FILAMENT_SENSOR_EXTRUDER_NUM]));
        u8g.print('%');
      }
    #else
      lcd_implementation_status_message(blink);
    #endif
  }
}
#if ENABLED(ULTIPANEL)
  uint8_t row_y1, row_y2;
  uint8_t constexpr row_height = DOG_CHAR_HEIGHT + 2 * (TALL_FONT_CORRECTION);
  #if ENABLED(ADVANCED_PAUSE_FEATURE)
    static void lcd_implementation_hotend_status(const uint8_t row) {
      row_y1 = row * row_height + 1;
      row_y2 = row_y1 + row_height - 1;
      if (!PAGE_CONTAINS(row_y1 + 1, row_y2 + 2)) return;
      u8g.setPrintPos(LCD_PIXEL_WIDTH - 11 * (DOG_CHAR_WIDTH), row_y2);
      lcd_print('E');
      lcd_print((char)('1' + active_extruder));
      lcd_print(' ');
      lcd_print(itostr3(thermalManager.degHotend(active_extruder)));
      lcd_print('/');
      if (lcd_blink() || !thermalManager.is_heater_idle(active_extruder))
        lcd_print(itostr3(thermalManager.degTargetHotend(active_extruder)));
    }
  #endif 
  static void lcd_implementation_mark_as_selected(const uint8_t row, const bool isSelected) {
    row_y1 = row * row_height + 1;
    row_y2 = row_y1 + row_height - 1;
    if (!PAGE_CONTAINS(row_y1 + 1, row_y2 + 2)) return;
    if (isSelected) {
      #if ENABLED(MENU_HOLLOW_FRAME)
        u8g.drawHLine(0, row_y1 + 1, LCD_PIXEL_WIDTH);
        u8g.drawHLine(0, row_y2 + 2, LCD_PIXEL_WIDTH);
      #else
        u8g.setColorIndex(1); 
        u8g.drawBox(0, row_y1 + 2, LCD_PIXEL_WIDTH, row_height - 1);
        u8g.setColorIndex(0); 
      #endif
    }
    #if DISABLED(MENU_HOLLOW_FRAME)
      else {
        u8g.setColorIndex(1); 
      }
    #endif
    u8g.setPrintPos((START_COL) * (DOG_CHAR_WIDTH), row_y2);
  }
  static void lcd_implementation_drawmenu_static(const uint8_t row, const char* pstr, const bool center=true, const bool invert=false, const char* valstr=NULL) {
    lcd_implementation_mark_as_selected(row, invert);
    if (!PAGE_CONTAINS(row_y1, row_y2)) return;
    char c;
    int8_t n = LCD_WIDTH - (START_COL);
    if (center && !valstr) {
      int8_t pad = (LCD_WIDTH - lcd_strlen_P(pstr)) / 2;
      while (--pad >= 0) { u8g.print(' '); n--; }
    }
    while (n > 0 && (c = pgm_read_byte(pstr))) {
      n -= lcd_print_and_count(c);
      pstr++;
    }
    if (valstr) while (n > 0 && (c = *valstr)) {
      n -= lcd_print_and_count(c);
      valstr++;
    }
    while (n-- > 0) u8g.print(' ');
  }
  static void lcd_implementation_drawmenu_generic(const bool isSelected, const uint8_t row, const char* pstr, const char pre_char, const char post_char) {
    UNUSED(pre_char);
    lcd_implementation_mark_as_selected(row, isSelected);
    if (!PAGE_CONTAINS(row_y1, row_y2)) return;
    uint8_t n = LCD_WIDTH - (START_COL) - 2;
    while (char c = pgm_read_byte(pstr)) {
      n -= lcd_print_and_count(c);
      pstr++;
    }
    while (n--) u8g.print(' ');
    u8g.setPrintPos(LCD_PIXEL_WIDTH - (DOG_CHAR_WIDTH), row_y2);
    lcd_print(post_char);
    u8g.print(' ');
  }
#define lcd_implementation_drawmenu_levelComplete(sel, row, pstr, dummy) lcd_implementation_drawmenu_generic(sel, row, pstr, LCD_STR_UPLEVEL[0], LCD_STR_UPLEVEL[0])
#define lcd_implementation_drawmenu_back(sel, row, pstr, dummy) lcd_implementation_drawmenu_generic(sel, row, pstr, LCD_STR_UPLEVEL[0], LCD_STR_UPLEVEL[0])
#define lcd_implementation_drawmenu_submenu(sel, row, pstr, data) lcd_implementation_drawmenu_generic(sel, row, pstr, '>', LCD_STR_ARROW_RIGHT[0])
#define lcd_implementation_drawmenu_gcode(sel, row, pstr, gcode) lcd_implementation_drawmenu_generic(sel, row, pstr, '>', ' ')
#define lcd_implementation_drawmenu_function(sel, row, pstr, data) lcd_implementation_drawmenu_generic(sel, row, pstr, '>', ' ')
#define lcd_implementation_drawmenu_gotoScreen(sel, row, pstr, d1,d2,d3) lcd_implementation_drawmenu_generic(sel, row, pstr, '>', LCD_STR_ARROW_RIGHT[0])
  static void _drawmenu_setting_edit_generic(const bool isSelected, const uint8_t row, const char* pstr, const char* const data, const bool pgm) {
    lcd_implementation_mark_as_selected(row, isSelected);
    if (!PAGE_CONTAINS(row_y1, row_y2)) return;
    const uint8_t vallen = (pgm ? lcd_strlen_P(data) : (lcd_strlen((char*)data)));
    uint8_t n = LCD_WIDTH - (START_COL) - 2 - vallen;
    while (char c = pgm_read_byte(pstr)) {
      n -= lcd_print_and_count(c);
      pstr++;
    }
    u8g.print(':');
    while (n--) u8g.print(' ');
    u8g.setPrintPos(LCD_PIXEL_WIDTH - (DOG_CHAR_WIDTH) * vallen, row_y2);
    if (pgm)  lcd_printPGM(data);  else  lcd_print((char*)data);
  }
  #define lcd_implementation_drawmenu_setting_edit_generic(sel, row, pstr, data) _drawmenu_setting_edit_generic(sel, row, pstr, data, false)
  #define lcd_implementation_drawmenu_setting_edit_generic_P(sel, row, pstr, data) _drawmenu_setting_edit_generic(sel, row, pstr, data, true)
  #define DEFINE_LCD_IMPLEMENTATION_DRAWMENU_SETTING_EDIT_TYPE(_type, _name, _strFunc) \
    inline void lcd_implementation_drawmenu_setting_edit_ ## _name (const bool sel, const uint8_t row, const char* pstr, const char* pstr2, _type * const data, ...) { \
      lcd_implementation_drawmenu_setting_edit_generic(sel, row, pstr, _strFunc(*(data))); \
    } \
    inline void lcd_implementation_drawmenu_setting_edit_callback_ ## _name (const bool sel, const uint8_t row, const char* pstr, const char* pstr2, _type * const data, ...) { \
      lcd_implementation_drawmenu_setting_edit_generic(sel, row, pstr, _strFunc(*(data))); \
    } \
    inline void lcd_implementation_drawmenu_setting_edit_accessor_ ## _name (const bool sel, const uint8_t row, const char* pstr, const char* pstr2, _type (*pget)(), void (*pset)(_type), ...) { \
      lcd_implementation_drawmenu_setting_edit_generic(sel, row, pstr, _strFunc(pget())); \
    } \
    typedef void _name##_void
  DEFINE_LCD_IMPLEMENTATION_DRAWMENU_SETTING_EDIT_TYPE(int16_t, int3, itostr3);
  DEFINE_LCD_IMPLEMENTATION_DRAWMENU_SETTING_EDIT_TYPE(uint8_t, int8, i8tostr3);
  DEFINE_LCD_IMPLEMENTATION_DRAWMENU_SETTING_EDIT_TYPE(float, float3, ftostr3);
  DEFINE_LCD_IMPLEMENTATION_DRAWMENU_SETTING_EDIT_TYPE(float, float32, ftostr32);
  DEFINE_LCD_IMPLEMENTATION_DRAWMENU_SETTING_EDIT_TYPE(float, float43, ftostr43sign);
  DEFINE_LCD_IMPLEMENTATION_DRAWMENU_SETTING_EDIT_TYPE(float, float5, ftostr5rj);
  DEFINE_LCD_IMPLEMENTATION_DRAWMENU_SETTING_EDIT_TYPE(float, float51, ftostr51sign);
  DEFINE_LCD_IMPLEMENTATION_DRAWMENU_SETTING_EDIT_TYPE(float, float52, ftostr52sign);
  DEFINE_LCD_IMPLEMENTATION_DRAWMENU_SETTING_EDIT_TYPE(float, float62, ftostr62rj);
  DEFINE_LCD_IMPLEMENTATION_DRAWMENU_SETTING_EDIT_TYPE(uint32_t, long5, ftostr5rj);
  #define lcd_implementation_drawmenu_setting_edit_bool(sel, row, pstr, pstr2, data) lcd_implementation_drawmenu_setting_edit_generic_P(sel, row, pstr, (*(data))?PSTR(MSG_ON):PSTR(MSG_OFF))
  #define lcd_implementation_drawmenu_setting_edit_callback_bool(sel, row, pstr, pstr2, data, callback) lcd_implementation_drawmenu_setting_edit_generic_P(sel, row, pstr, (*(data))?PSTR(MSG_ON):PSTR(MSG_OFF))
  #define lcd_implementation_drawmenu_setting_edit_accessor_bool(sel, row, pstr, pstr2, pget, pset) lcd_implementation_drawmenu_setting_edit_generic_P(sel, row, pstr, (*(data))?PSTR(MSG_ON):PSTR(MSG_OFF))
  void lcd_implementation_drawedit(const char* const pstr, const char* const value=NULL) {
    const uint8_t labellen = lcd_strlen_P(pstr),
                  vallen = lcd_strlen(value);
    uint8_t rows = (labellen > LCD_WIDTH - 2 - vallen) ? 2 : 1;
    #if ENABLED(USE_BIG_EDIT_FONT)
      uint8_t lcd_width, char_width;
      if (labellen <= LCD_WIDTH_EDIT - 1) {
        if (labellen + vallen + 2 >= LCD_WIDTH_EDIT) rows = 2;
        lcd_width = LCD_WIDTH_EDIT + 1;
        char_width = DOG_CHAR_WIDTH_EDIT;
        lcd_setFont(FONT_MENU_EDIT);
      }
      else {
        lcd_width = LCD_WIDTH - (START_COL);
        char_width = DOG_CHAR_WIDTH;
        lcd_setFont(FONT_MENU);
      }
    #else
      constexpr uint8_t lcd_width = LCD_WIDTH - (START_COL),
                        char_width = DOG_CHAR_WIDTH;
    #endif
    const uint8_t segmentHeight = u8g.getHeight() / (rows + 1); 
    uint8_t baseline = segmentHeight + (DOG_CHAR_HEIGHT_EDIT + 1) / 2;
    bool onpage = PAGE_CONTAINS(baseline + 1 - (DOG_CHAR_HEIGHT_EDIT), baseline);
    if (onpage) {
      u8g.setPrintPos(0, baseline);
      lcd_printPGM(pstr);
    }
    if (value != NULL) {
      u8g.print(':');
      if (rows == 2) {
        baseline += segmentHeight;
        onpage = PAGE_CONTAINS(baseline + 1 - (DOG_CHAR_HEIGHT_EDIT), baseline);
      }
      if (onpage) {
        u8g.setPrintPos(((lcd_width - 1) - (vallen + 1)) * char_width, baseline); 
        u8g.print(' '); 
        lcd_print(value);
      }
    }
  }
  #if ENABLED(SDSUPPORT)
    static void _drawmenu_sd(const bool isSelected, const uint8_t row, const char* const pstr, const char* filename, char* const longFilename, const bool isDir) {
      UNUSED(pstr);
      lcd_implementation_mark_as_selected(row, isSelected);
      if (!PAGE_CONTAINS(row_y1, row_y2)) return;
      uint8_t n = LCD_WIDTH - (START_COL) - 1;
      if (longFilename[0]) {
        filename = longFilename;
        longFilename[n] = '\0';
      }
      if (isDir) lcd_print(LCD_STR_FOLDER[0]);
      while (char c = *filename) {
        n -= lcd_print_and_count(c);
        filename++;
      }
      while (n--) u8g.print(' ');
    }
    #define lcd_implementation_drawmenu_sdfile(sel, row, pstr, filename, longFilename) _drawmenu_sd(sel, row, pstr, filename, longFilename, false)
    #define lcd_implementation_drawmenu_sddirectory(sel, row, pstr, filename, longFilename) _drawmenu_sd(sel, row, pstr, filename, longFilename, true)
  #endif 
  #if ENABLED(AUTO_BED_LEVELING_UBL)
    #define MAP_UPPER_LEFT_CORNER_X 35  
    #define MAP_UPPER_LEFT_CORNER_Y  8  
    #define MAP_MAX_PIXELS_X        53
    #define MAP_MAX_PIXELS_Y        49
    void lcd_implementation_ubl_plot(const uint8_t x_plot, const uint8_t y_plot) {
      uint8_t x_map_pixels = ((MAP_MAX_PIXELS_X - 4) / (GRID_MAX_POINTS_X)) * (GRID_MAX_POINTS_X),
              y_map_pixels = ((MAP_MAX_PIXELS_Y - 4) / (GRID_MAX_POINTS_Y)) * (GRID_MAX_POINTS_Y),
              pixels_per_X_mesh_pnt = x_map_pixels / (GRID_MAX_POINTS_X),
              pixels_per_Y_mesh_pnt = y_map_pixels / (GRID_MAX_POINTS_Y),
              x_offset = MAP_UPPER_LEFT_CORNER_X + 1 + (MAP_MAX_PIXELS_X - x_map_pixels - 2) / 2,
              y_offset = MAP_UPPER_LEFT_CORNER_Y + 1 + (MAP_MAX_PIXELS_Y - y_map_pixels - 2) / 2;
      if (PAGE_CONTAINS(y_offset - 2, y_offset + y_map_pixels + 4)) {
        u8g.setColorIndex(1);  
        u8g.drawBox(x_offset - 2, y_offset - 2, x_map_pixels + 4, y_map_pixels + 4);
        if (PAGE_CONTAINS(y_offset, y_offset + y_map_pixels)) {
          u8g.setColorIndex(0);  
          u8g.drawBox(x_offset, y_offset, x_map_pixels, y_map_pixels);
        }
      }
      u8g.setColorIndex(1);
      const uint8_t sx = x_offset + pixels_per_X_mesh_pnt / 2;
            uint8_t  y = y_offset + pixels_per_Y_mesh_pnt / 2;
      for (uint8_t j = 0; j < GRID_MAX_POINTS_Y; j++, y += pixels_per_Y_mesh_pnt)
        if (PAGE_CONTAINS(y, y))
          for (uint8_t i = 0, x = sx; i < GRID_MAX_POINTS_X; i++, x += pixels_per_X_mesh_pnt)
            u8g.drawBox(sx, y, 1, 1);
      uint8_t inverted_y = GRID_MAX_POINTS_Y - y_plot - 1;  
      const uint8_t by = y_offset + inverted_y * pixels_per_Y_mesh_pnt;
      if (PAGE_CONTAINS(by, by + pixels_per_Y_mesh_pnt))
        u8g.drawBox(
          x_offset + x_plot * pixels_per_X_mesh_pnt, by,
          pixels_per_X_mesh_pnt, pixels_per_Y_mesh_pnt
        );
      u8g.setColorIndex(1);
      if (PAGE_UNDER(7)) {
        u8g.setPrintPos(5, 7);
        lcd_print("X:");
        lcd_print(ftostr32(LOGICAL_X_POSITION(pgm_read_float(&ubl._mesh_index_to_xpos[x_plot]))));
        u8g.setPrintPos(74, 7);
        lcd_print("Y:");
        lcd_print(ftostr32(LOGICAL_Y_POSITION(pgm_read_float(&ubl._mesh_index_to_ypos[y_plot]))));
      }
      if (PAGE_CONTAINS(64 - (INFO_FONT_HEIGHT - 1), 64)) {
        u8g.setPrintPos(5, 64);
        lcd_print('(');
        u8g.print(x_plot);
        lcd_print(',');
        u8g.print(y_plot);
        lcd_print(')');
        u8g.setPrintPos(74, 64);
        lcd_print("Z:");
        if (!isnan(ubl.z_values[x_plot][y_plot]))
          lcd_print(ftostr43sign(ubl.z_values[x_plot][y_plot]));
        else
          lcd_printPGM(PSTR(" -----"));
      }
    }
  #endif 
#endif 
#endif 
