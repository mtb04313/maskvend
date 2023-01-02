/** THE SOURCE CODE AND ITS RELATED DOCUMENTATION IS PROVIDED "AS IS". INFINEON
TECHNOLOGIES MAKES NO OTHER WARRANTY OF ANY KIND,WHETHER EXPRESS,IMPLIED OR,
STATUTORY AND DISCLAIMS ANY AND ALL IMPLIED WARRANTIES OF MERCHANTABILITY,
SATISFACTORY QUALITY, NON INFRINGEMENT AND FITNESS FOR A PARTICULAR PURPOSE.

THE SOURCE CODE AND DOCUMENTATION MAY INCLUDE ERRORS. INFINEON TECHNOLOGIES
RESERVES THE CAPSENSE_SLIDERRIGHT TO INCORPORATE MODIFICATIONS TO THE SOURCE CODE IN LATER
REVISIONS OF IT, AND TO MAKE IMPROVEMENTS OR CHANGES IN THE DOCUMENTATION OR
THE PRODUCTS OR TECHNOLOGIES DESCRIBED THEREIN AT ANY TIME.

INFINEON TECHNOLOGIES SHALL NOT BE LIABLE FOR ANY DIRECT, INDIRECT OR
CONSEQUENTIAL DAMAGE OR LIABILITY ARISING FROM YOUR USE OF THE SOURCE CODE OR
ANY DOCUMENTATION, INCLUDING BUT NOT LIMITED TO, LOST REVENUES, DATA OR
PROFITS, DAMAGES OF ANY SPECIAL, INCIDENTAL OR CONSEQUENTIAL NATURE, PUNITIVE
DAMAGES, LOSS OF PROPERTY OR LOSS OF PROFITS ARISING OUT OF OR IN CONNECTION
WITH THIS AGREEMENT, OR BEING UNUSABLE, EVEN IF ADVISED OF THE POSSIBILITY OR
PROBABILITY OF SUCH DAMAGES AND WHETHER A CLAIM FOR SUCH DAMAGE IS BASED UPON
WARRANTY, CONTRACT, TORT, NEGLIGENCE OR OTHERWISE.

(C)CopyCAPSENSE_SLIDERRIGHT INFINEON TECHNOLOGIES All CAPSENSE_SLIDERRIGHTs reserved
*/

#ifndef __GUI_DEFINITIONS_H__
#define __GUI_DEFINITIONS_H__

#ifdef __cplusplus
extern "C"
{
#endif

/*-- Public Definitions -------------------------------------------------*/

#define SCREEN_WIDTH  320
#define SCREEN_HEIGHT 240
#define SCREEN_CENTER_X_COORD       159
#define SCREEN_CENTER_Y_COORD       119

#define TEXT_BOX_X1_COORD           0
#define TEXT_BOX_Y1_COORD           89
#define TEXT_BOX_X2_COORD           319
#define TEXT_BOX_Y2_COORD           149

#define MASK_DISPENSED_MSG_X_COORD  210
#define MASK_DISPENSED_MSG_Y_COORD  100
#define OUT_OF_STOCK_MSG_X_COORD    210
#define OUT_OF_STOCK_MSG_Y_COORD    100


/*-- GUI OBJECT POSITIONS -------------------------------------------------*/
#define SCREEN_X_COORD_0 0
#define SCREEN_Y_COORD_0 0
#define SCREEN_X_COORD_319 319
#define SCREEN_Y_COORD_239 239

#define BATTERY_TOP_LOGO_X_COORD 295
#define BATTERY_TOP_LOGO_Y_COORD 5
#define ESIM_LOGO_SETTINGS_X_COORD	137
#define ESIM_LOGO_SETTINGS_Y_COORD	140
#define BLUETOOTH_LOGO_SETTINGS_X_COORD	67
#define BLUETOOTH_LOGO_SETTINGS_Y_COORD	140
#define BATTERY_LOGO_SETTINGS_X_COORD	200
#define BATTERY_LOGO_SETTINGS_Y_COORD	140
#define WORD_ICONNAME_SETTINGS_X_COORD	160
#define WORD_ICONNAME_SETTINGS_DISPLAY_Y_COORD	20
#define WORD_ICONNAME_SETTINGS_Y_COORD	80
#define ESIM_SETTINGS_FIRST_ITEM_Y_COORD 95
#define ESIM_SETTINGS_ARROW_POINTER_X_COORD 28
#define ESIM_SETTINGS_SHIFT_BY_30_Y_COORD 30
#define TIME_LINE_X_COORD 40
#define TIME_LINE_1_Y_COORD 100
#define TIME_LINE_2_Y_COORD 115

#define PROFILE_ESIM_AVAILABLE_1_X_COORD 133
#define PROFILE_ESIM_AVAILABLE_2_X_COORD 97
#define PROFILE_ESIM_AVAILABLE_3_X_COORD 63
#define PROFILE_ESIM_AVAILABLE_4_X_COORD 27
#define PROFILE_ESIM_LOGO_BORDER_SHIFT_XY_COORD 2
#define PROFILE_ESIM_ENABLED_ASTERISK_SHIFT_X_COORD 26
#define PROFILE_ESIM_ENABLED_ASTERISK_SHIFT_X_COORD 26
#define PROFILE_ESIM_ENABLED_ASTERISK_Y_COORD 122
#define PROFILE_ESIM_NAME_DISPLAY_X_COORD 160
#define PROFILE_ESIM_NAME_DISPLAY_Y_COORD 70
#define PROFILE_ESIM_ROW_1_LOGO_Y_COORD 140
#define PROFILE_ESIM_SHIFT_BY_70_X_COORD 70

#define PROFILE_ENABLE_BUTTON_X_COORD 110
#define PROFILE_ENABLE_BUTTON_Y_COORD 100
#define PROFILE_DISABLE_BUTTON_X_COORD 210
#define PROFILE_DISABLE_BUTTON_Y_COORD 100
#define PROFILE_ENABLE_DISABLE_NOTIF_X_COORD 160
#define PROFILE_ENABLE_DISABLE_NOTIF_Y_COORD 200

#define EID_DISPLAY_X_COORD 160
#define EID_DISPLAY_Y_COORD 20

#define ANDROID_CONNETCION_NOTIF_X_COORD 160
#define ANDROID_CONNETCION_NOTIF_Y_COORD 200

/*-- SELECTIONS -------------------------------------------------*/
#define EXIT_BUTTON 'X'
#define ESIM_SETTINGS '2'
#define LOWER_LIMIT_BUTTON_SELECTION 1
#define UPPER_LIMIT_BUTTON_SELECTION_SETTINGS_MENU 3
#define UPPER_LIMIT_BUTTON_SELECTION_ESIM_SETTINGS_MENU 2
#define UPPER_LIMIT_BUTTON_SELECTION_ESIM_PROFILE_ENABLE_DISABLE 2
#define INPUT_RECIEVED true
#define NO_INPUT_RECIEVED false

#ifdef __cplusplus
}
#endif

#endif // __GUI_DEFINITIONS_H__
