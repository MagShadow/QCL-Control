#ifndef DLS_SCI_CMD_PUB_H
#define DLS_SCI_CMD_PUB_H

#include <stdint.h>

#ifdef __cplusplus
extern "C"
{
#endif

// Title: DlsSciCmdPub.h

/* Topic: Purpose
	This file contains definitions used by the internal command protocol of the Sidekick Controller that need to be accessible by users of the
	SidekickSDK.
*/

#ifdef _WIN32
#pragma pack(push, 1)
#endif

/* String Lengths */
#define DLS_SCI_MFG_DATE_STR_LEN						10
#define DLS_SCI_MODEL_NUM_STR_LEN						24
#define DLS_SCI_SERIAL_NUM_STR_LEN						24

#define DLS_SCI_INFO_SYS_MAX_NUM_ERRORS 20

typedef struct _DLS_SCI_INFO_SYSTEM_ERROR_LIST	//Same as MIRcat
{
	uint16_t	status_list[DLS_SCI_INFO_SYS_MAX_NUM_ERRORS];
	uint16_t	num_items;
	uint16_t	index;
}DLS_SCI_INFO_SYSTEM_ERROR_LIST;

typedef struct _DLS_SCI_INFO_SYSTEM_ERRORS		//Same as MIRcat
{
	DLS_SCI_INFO_SYSTEM_ERROR_LIST	system_errors;
	DLS_SCI_INFO_SYSTEM_ERROR_LIST	system_warnings;
}DLS_SCI_INFO_SYSTEM_ERRORS;

typedef union _DLS_SCI_SCAN_WAVE_TYPE_UNION
{
	int32_t    encValue;
	float	   waveValue;
}DLS_SCI_SCAN_WAVE_TYPE_UNION;

#ifdef _WIN32
#pragma pack(pop)
#endif

#ifdef __cplusplus
}   // End extern C
#endif

#endif