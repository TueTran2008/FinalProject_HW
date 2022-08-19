#ifndef __DATA_DEFINE_H__
#define __DATA_DEFINE_H__

#include <stdint.h>
#include <ctype.h>
#include <math.h>
#include "esp_system.h"

#define __FIRMWARE_VERSION__ "SL1.8.9"

//typedef union
//{
//    float value;
//    uint8_t bytes[4];
//} Float_t;

typedef union
{
    uint32_t value;
    uint8_t bytes[4];
} Long_t;

typedef union
{
    uint16_t value;
    uint8_t bytes[2];
} Int_t;

typedef struct
{
    uint8_t Year;
    uint8_t Month;
    uint8_t Day;
    uint8_t Hour;
    uint8_t Minute;
    uint8_t Second;
} DateTime_t;

typedef struct
{
    uint8_t Sub[4];
    uint16_t Port;
} IPStruct_t;


typedef enum {
	IO_OFF = 0,
	IO_ON = 1
} IOState_e;

typedef union {
	struct _State_t {
		uint8_t isEthernetUp : 1;
		uint8_t isWiFiUp : 1;
		uint8_t isPPPUp : 1;
		uint8_t NA : 5;
	} Name;
	uint8_t Value;
} Network_State_t;


typedef enum {
	NO_STREAM  = 0,
	LIVE_MASTER = 1,
	SCHEDULE_LINK = 2,
	LAPTOP = 3,
	MOBILE = 4
} MasterStreamType_t;






typedef struct {	
    // char WiFiName[30];
	// char WiFiPass[30];
	// uint8_t WiFiEnable;
    // uint8_t SendInfoPeriod;	/* Second */

	// char GSM_IMEI[25];
	// char SIM_IMEI[25];

	// Dành cho cấu trúc nhóm master (T1,T2,T3 -> H1,H2,H3 -> X1,X2,X3)
	// char MASTER[10][25];	/* IMEI of list master */

	
	// char MASTER_IMEI[25];	/* GSM IMEI of Master */
	// uint8_t UID[6];	/* Unique MAC of ESP32 */

	// uint8_t CurrentMaster;	/* Đang thu từ master nào, sau reset còn biết để resume */
	// uint8_t OperateMode;	/* None - Disable all receiver, FM - receive FM */
	// uint8_t Volume;		/* Volume chung cho cả audio codec và FM receiver */
	// uint32_t FMFreq1;	/* KHz */
	// uint32_t FMFreq2;	/* KHz */
	// uint32_t FMFreq3;	/* KHz */
	// uint32_t FMFreqRun;	/* KHz: Tần số FM được chọn để thu khi chạy chế độ FM */
	// /* Stream start -> turn on relay1 -> delay1 -> turn on relay2 */
	// uint8_t delayTurnOnRelay;	
	// /* Stream finished -> delay2 -> off relay2 -> delay1 -> off relay1 */
	// uint8_t delayTurnOffRelay1;
	// uint16_t delayTurnOffRelay2;
    // uint8_t speaker_detect;
	// char HttpUrlHeader[128];		/* Url stream server */
} Parameters_t;

typedef struct
{
	/* Phiên bản phần cứng */
	uint8_t hwVersion;

	/* Version mạch MCU_Expander */
	// uint8_t workerVersion;
	
	/* Trạng thái CPU */
	// uint32_t resetCounter;

	/* Trạng thái internet của các interface mạng */
	// Network_State_t	Network;

	/* Detect trạng thái loa */
	// app_io_speaker_detect_state_t Speaker;
	
	/* Trạng thái codec mode */
	// uint8_t codecMode;
	
	/* Trạng thái ngoại vi */
	// bool isMICPlugged;
	// uint8_t Volume;		/* Giá trị volume hiện tại đang chạy: của codec/của mạch FM */

	/* Trạng thái streaming của các đài Tỉnh, Huyện, Xã */
	// uint8_t httpReceivedDataTimeout;	/* Timeout thời gian nhận được stream data http */

	/* Trạng thái module FM */
	// uint8_t FMModuleMode;
	// uint32_t FMFreqRunning;		/* Tần số module FM đang thu */
	// uint8_t FMRSSI;
	// uint8_t FMSNR;
	// int FMRSSIdBm;
	// char gpsLatLng[20];
} Status_t;

typedef struct
{
	// Parameters_t Parameters;	
	Status_t Status;
}System_t;

#endif // __DATA_DEFINE_H__