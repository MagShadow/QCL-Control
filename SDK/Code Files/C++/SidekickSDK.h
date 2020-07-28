//@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@
// SidekickSDK.h
//
// Copyright (c) 2015.   All rights reserved.
// Daylight Solutions
// 15378 Avenue of Science, Suite 200
// San Diego, CA  92128
//@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@


#ifndef _SIDEKICKSDK_H
#define _SIDEKICKSDK_H
#pragma once
#include <stdint.h>
#include "SidekickSDKConstants.h"
#include "SidekickSDKTypes.h"

#ifdef SIDEKICKSDK_EXPORTS
#define SIDEKICK_SDK_DLL	__declspec(dllexport)
#else
#define SIDEKICK_SDK_DLL	__declspec(dllimport)
#endif

#ifdef __cplusplus
extern "C"
{
#else
#include <stdbool.h>
#endif

// Title: SidekickSDK.h

/* Topic: Documentation and Code Conventions

	- Every function the SDK starts with *SidekickSDK_*. This prefix is omitted from the much of the documentation.
	- Almost every function returns one of the SDK <Error Codes>, so the return values are omitted from the documentation for
		most functions. Return values appear for the exceptions.
	- Every function that communicates with a connected controller takes a <DLS_SCI_DEVICE_HANDLE> as the first
	parameter. This parameter is omitted from the documentation parameter lists, except for the documentation of
	the connection functions.
	- Since most functions return <Error Codes>, functions that require data output use out parameters. The user must
		pass a pointer to enough storage space to hold the data. Out parameters are noted in the documentation by
		*(out)*.
	- Getting data from the system often requires using two functions: one function sends the command to retrieve
		the data from the system and the SDK stores the data internally, the second function gets the internally stored
		values from the SDK. The user must make sure to use the first function to update the data if they want
		up to date information.
	- Similar to getting data from the system, sending data to the head often requires two functions: one to set
	the values to send, then another to send the values to the system.
	- In general, functions that are prefixed with Read, ReadWrite, or Exec perform the actual communication with the laser
		controller.  These should be used with the functions prefixed with Get or Set to get or set the actual data that 
		will be read/written.
	- For example, to read data from the laser, the Read function would first be called.  If successful, the associated
		Get function would be used to get the data that was returned from the laser.  Likewise, to write data to the laser,
		the Set function would first be called to set the local data structure.  Then, the ReadWrite(true) function would 
		be called to actually send the data to the laser.
*/


/************************************************************************
<summary>This function currently does nothing. It is a placeholder for possible future development</summary>
<returns>Error code.  Possible codes: SIDEKICK_SDK_RET_SUCCESS</returns>
*/
SIDEKICK_SDK_DLL uint32_t SidekickSDK_Initialize();

//---------------------------------------------------------------------------------------------------------//
/* Group: Miscellaneous Functions */

/* Function: GetAPIVersion
*	Get Version of the API.
*
*	Parameters:
*		papiVersionMajor - (out) Major Version of Sidekick API.
*		papiVersionMinor - (out) Minor Version of Sidekick API.
*		papiVersionPatch - (out) Patch Version of Sidekick API.
*/
SIDEKICK_SDK_DLL uint32_t SidekickSDK_GetAPIVersion(uint16_t * papiVersionMajor, uint16_t * papiVersionMinor, uint16_t * papiVersionPatch);


/* Function: printDeviceInfo
*	Prints the provided <DEVICE_INFO> structure to STDOUT. This function is meant for debugging console programs.
*
*	Parameters:
*		pDeviceInfo - Pointer to <DEVICE_INFO> structure that contains device information to print.
*/
SIDEKICK_SDK_DLL void SidekickSDK_printDeviceInfo(const DLS_SCI_DEVICE_INFO *pDeviceInfo);

/* Function: GetCommErrDesc
*	Gets a C style string description from an error code returned from SDK function.
*
*	Parameters:
*		errCode - Error code returned from SDK function.
*
*	Returns:
*		C-style string describing the error.
*/
SIDEKICK_SDK_DLL const char *SidekickSDK_GetCommErrDesc(uint32_t errCode);

//---------------------------------------------------------------------------------------------------------//
/* Group:  Search and Connect Functions
	
 These functions are for searching for controllers and connecting to them.

 Topic: Searching for Devices

	Before connection to a controller, one must call a search function to find available controllers. The SDK can perform a search
	for devices connected to a host computer using USB, or for devices that are present on the network. The user can choose to
	perform the search over one or both transport types (USB/Network) using different functions. The SDK searches USB by enumerating
	the host computer's USB bus. The SDK searches the network by using a discovery protocol build on top of UDP. For the network
	search, the user must specify a multicast group address and port number. The default discovery address is *239.255.101.224:8383*.
	To change this address, see <Network Setup Functions>.

	During the user commanded search, the SDK populates an internal list of devices. For each device found, the SDK attempts to
	connect to the device and request information about the laser head connected to the controller. The SDK then fills in a
	<DLS_SCI_DEVICE_INFO> structure and adds it to the internal list of devices.

	After the SDK completes the search, the user can enumerate the list of found devices with <GetNumOfDevices> and <GetDeviceInfo>.

	(start code)
	// Search example
	DLS_SCI_DEVICE_INFO deviceInfoList[10];
	uint32_t ret = 0;
	uint16_t numberOfDevices = 0;
	int i = 0;

	// Search over both USB and network transports
	ret = SidekickSDK_SearchForDevices("239.255.101.224", 8383);
	if(SIDEKICK_SDK_RET_SUCCESS == ret)
	{
		ret = SidekickSDK_GetNumOfDevices(&numberOfDevices);
		if ((SIDEKICK_SDK_RET_SUCCESS == ret) && numberOfDevices > 0)
		{
			// Enumerate device list
			for(i = 0; i < numberOfDevices; i++)
			{
				SidekickSDK_GetDeviceInfo(i, deviceInfoList[i]);
				SidekickSDK_printDeviceInfo(&deviceInfoList[i]);
			}
		}
	}
	(end code)
	
 Topic: Connecting to Devices

	Using the information retrieved from the list populated by the search, the user can then command the SDK to connect to the 
	desired device. Functions exist to specify connection transport type (USB/network), or to connect to a device using any available
	transport.

	The SDK can connect to multiple devices simultaneously. To keep track of multiple connections, the SDK uses
	<DLS_SCI_DEVICE_HANDLE>. Each connect function takes an out parameter for holding a <DLS_SCI_DEVICE_HANDLE>. If the connection is
	successful, the SDK outputs the	handle for that device to the out parameter. The user then saves that handle and uses it for all
	communications with that specific device. When the User is done with the device, they should pass the handle to the <Disconnect>
	function to close the connection.

	(start code)
	// Connection Example
	DLS_SCI_DEVICE_HANDLE handle = DLS_SCI_DEVICE_NULL_HANDLE; // no connected device should have this handle
	uint32_t StatusWord = 0, ret = 0;
	uint16_t WarningWord = 0, ErrorWord = 0;

	// Note that the address of handle is passed (pass by reference)
	ret = SidekickSDK_ConnectToDeviceNumber(&handle, 0); // connect to first device found
	if(SIDEKICK_SDK_RET_SUCCESS == ret)
	{
		// Connection success, read status
		// Note that the handle is now passed by value instead of reference
		ret = SidekickSDK_ReadStatusMask(handle, &StatusWord, &ErrorWord, &WarningWord);
		SidekickSDK_Disconnect(handle);
	}
	(end code)
*/

/* Function: SearchForDevices
*	This function searches for USB FTDI devices and TCP attached devices.
*
*	Parameters:
*		ipAddress - String in dot format containing the Group IP Address.
*		port - Port number to use for discovery protocol.
*/
SIDEKICK_SDK_DLL uint32_t SidekickSDK_SearchForDevices(const char *ipAddress, unsigned int port);

/* Function: SearchForTCPDevices
*	Search for TCP devices using the discovery protocol. Typically the IP Address is a multicast(group) address,
*	but it can be a known device address. The the function sends a query to the group address, and controllers
*	that are members of the multicast group associated with the address will respond with their address and port.
*
*	Parameters:
*		ipAddress - String in dot format containing the Group IP Address.
*		port - Port number to use for discovery protocol.
*/
SIDEKICK_SDK_DLL uint32_t SidekickSDK_SearchForTCPDevices(const char *ipAddress, unsigned int port);

/* Function: SearchForUsbDevices
*	Search for USB devices attached to the system.
*
*	Returns:
*		Error code. Possible codes: SIDEKICK_SDK_RET_SEARCH_ERROR, SIDEKICK_SDK_RET_SUCCESS on success.
*/
SIDEKICK_SDK_DLL uint32_t SidekickSDK_SearchForUsbDevices(void);

/* Function: SetSearchTimeout
	Sets the timeout value of the search functions. Default is <SIDEKICK_SDK_SEARCH_TIMEOUT>. Only call this function if a change from the
	default is desired.

	Parameters:
		timeoutms - Desired timeout in milliseconds.
*/
SIDEKICK_SDK_DLL uint32_t SidekickSDK_SetSearchTimeout(uint32_t timeoutms);

/* Function: GetSearchTimeout
Gets the timeout value of the search functions.

	Parameters:
	timeoutms - (out) timeout in milliseconds.
*/

SIDEKICK_SDK_DLL uint32_t SidekickSDK_GetSearchTimeout(uint32_t *timeoutms);

/* Function: ClearDeviceList
	Clears the cached search data. After calling this function, the internal search list will be empty and a new search is necessary
	to populate the list.
*/
SIDEKICK_SDK_DLL uint32_t SidekickSDK_ClearDeviceList(void);

/* Function: GetNumOfDevices
*	Gets the number of devices attached to the system.  Must run a search
*	over TCP or USB before calling this function.
*
*	Parameters:
*		numDevices - (out) Number of devices found during previous search.
*/
SIDEKICK_SDK_DLL uint32_t SidekickSDK_GetNumOfDevices(uint16_t *pnumDevices);

/* Function: GetNumTcpDevices
*	Gets the number of TCP devices attached to the system.  Must run a search
*	over TCP before calling this function.
*
*	Parameters:
*		pnumDevices - (out) Number of network devices found during previous search.
*/
SIDEKICK_SDK_DLL uint32_t SidekickSDK_GetNumTcpDevices(uint16_t *pnumDevices);

/* Function: GetNumUsbDevices
*	Gets the number of USB FTDI devices attached to the system.  A call to
*	a search function that searches for TCP devices must be called first.
*
*	Parameter:
*		pnumDevices - (out) Number of USB devices found during previous search.
*/
SIDEKICK_SDK_DLL uint32_t SidekickSDK_GetNumUsbDevices(uint16_t * pnumDevices);

/* Function: GetDeviceInfo
*	Gets the information for a device with the provided number. Call a search function to populate
*	the device list prior to calling this function. See <DlsSciDeviceTypes.h> for the data structure format.
*
*	Parameters:
*		deviceNum - Device Instance.  Should be less than number of devices from call to <GetNumOfDevices>.
*		pDeviceInfo - (out) <DEVICE_INFO> structure that contains all of the device information.
*/
SIDEKICK_SDK_DLL uint32_t SidekickSDK_GetDeviceInfo(uint16_t deviceNum, DLS_SCI_DEVICE_INFO *pDeviceInfo);

/* Function: GetDeviceInfoFromHandle
	Gets the information for a connected device with the specified handle.

	Parameters:
		pDeviceInfo - (out) <DEVICE_INFO> that contains the device information.
*/
SIDEKICK_SDK_DLL uint32_t SidekickSDK_GetDeviceInfoFromHandle(DLS_SCI_DEVICE_HANDLE deviceHandle, DLS_SCI_DEVICE_INFO *pDeviceInfo);

/* Function: ConnectToDeviceNumber
*	Attempts to connect to a device with specified number from device list. If both USB and Ethernet are available,
*	connection will be made over USB.
*
*	Parameters:
*		p_deviceHandle - (out) The handle of the device on connection success.
*		deviceNum - Device number in the search list.  Should be less number of devices from call to
*					<GetNumOfDevices>.
*/
SIDEKICK_SDK_DLL uint32_t SidekickSDK_ConnectToDeviceNumber(DLS_SCI_DEVICE_HANDLE *p_deviceHandle, uint16_t deviceNum);

/* Function: ConnectToUsbDevice
*	Attempts to connect to a USB device using the USB device number, which is different than the device number
*	from the search list. To retrieve USB device number, <GetDeviceInfo> for the device, check that USB information
*	is valid, then read the usb device number. Because these semantics can be confusing, it is recommended that
*	<ConnectToDeviceNumber> is used instead.
*
*	Parameters:
*		p_deviceHandle - (out) The handle of the device on connection success.
*		usbDeviceNum - Device Instance.  Should be a number from the usbFtdiDeviceNumber field of the DLS_SCI_DEVICE_INFO structure.
*/
SIDEKICK_SDK_DLL uint32_t SidekickSDK_ConnectToUsbDevice(DLS_SCI_DEVICE_HANDLE *p_deviceHandle, uint16_t usbDeviceNum);

/* Function: ConnectToTcpDevice
*	Attempts to connect to a TCP device with a known address and port number.
*
*	Parameters:
*		p_deviceHandle - (out) The handle of the device on connection success.
*		p_sIpAddress - Address in dotted decimal format of TCP device to connect to.
*		nComPort - Port of TCP device to connect to.
*/
SIDEKICK_SDK_DLL uint32_t SidekickSDK_ConnectToTcpDevice(DLS_SCI_DEVICE_HANDLE *p_deviceHandle, const char * p_sIpAddress, unsigned short nComPort);

/* Function: Disconnect
*	Disconnects from device associated with deviceHandle.
*
*	Parameters:
*		deviceHandle - Handle of the device to disconnect from.
*/
SIDEKICK_SDK_DLL uint32_t SidekickSDK_Disconnect(DLS_SCI_DEVICE_HANDLE deviceHandle);

//-------------------------------------------------------------------------------------------------------------------------------------------//
/* Group: Status Functions
	Functions for reading status, errors, and warnings. Use with <Status Helper Functions> or constants defined in
	SidekickSDKConstants.h.
*/

/* Topic: Status, Error, and Warning words
	The controller can retrieve these three words from the laser head. They must be decoded with bit masks or helper functions.

	Status Word gives various system status. Use <Status Helper Functions> or <System Status Masks> to decode the meaning of this
	word.

	Error Word gives various system errors. See <Errors and Warnings> for the codes.

	Warning Word gives various system warnings. See <Errors and Warnings> for the codes.
*/

/* Function: ReadStatusMask
*	Read the system status data from the unit.  Returns the 32 bit status word, 16 bit error word and a
*	16 bit warning word. Individual status states can be decoded using status mask helper functions.
*	This function is equivalent to calling <ReadInfoStatusMask> immediately followed by <GetInfoStatusMask>.
*
*	Parameters:
*		p_StatusMask - (out) Status Mask.
*		p_SysErrorWord - (out) System Error Word
*		p_SysWarningWord - (out) System Warning Word.
*/
SIDEKICK_SDK_DLL uint32_t SidekickSDK_ReadStatusMask(DLS_SCI_DEVICE_HANDLE deviceHandle, uint32_t * p_StatusMask, uint16_t * p_SysErrorWord, uint16_t * p_SysWarningWord);

/* Function: ReadInfoStatusMask
	Read status information from laser. Retrieve data with <GetInfoStatusMask>.
*/
SIDEKICK_SDK_DLL uint32_t SidekickSDK_ReadInfoStatusMask(DLS_SCI_DEVICE_HANDLE deviceHandle);
/* Function: GetInfoStatusMask
	Get Status word, error word, and warning word retrieved from previous <GetInfoStatusMask> or <ReadStatusMask> call.

	Parameters:
		p_statusMask - (out) Status word.
		p_sysErrorWord - (out) Error word.
		p_sysWarningWord - (out) Warning word.
*/
SIDEKICK_SDK_DLL uint32_t SidekickSDK_GetInfoStatusMask(DLS_SCI_DEVICE_HANDLE deviceHandle, uint32_t *p_statusMask, uint16_t *p_sysErrorWord, uint16_t *p_sysWarningWord);

/* Function: getSystemErrorWord 
	Gets system error word retrieved from previous <GetInfoStatusMask> or <ReadStatusMask> call.

	Parameters:
		p_SystemErrorWord - (out) System Error Word.
*/
SIDEKICK_SDK_DLL uint32_t SidekickSDK_getSystemErrorWord(DLS_SCI_DEVICE_HANDLE deviceHandle, uint16_t *p_SystemErrorWord);		// returns the 16 bit error status

/* Function: getSystemWarningWord
	Gets system warning word retrieved from previous <GetInfoStatusMask> or <ReadStatusMask> call.

	Parameters:
		p_SystemWarningWord - (out) System Warning Word.
*/
SIDEKICK_SDK_DLL uint32_t SidekickSDK_getSystemWarningWord(DLS_SCI_DEVICE_HANDLE deviceHandle, uint16_t *p_SystemWarningWord);		// returns the 16 bit warning status

//-------------------------------------------------------------------------------------------------------------------------------------------//
/* Group: Status Helper Functions

	These functions decode the status, error, and warning words retrieved from a <ReadInfoStatusMask> or <ReadStatusMask> call.
	Call <ReadInfoStatusMask> or <ReadStatusMask> before calling any of the functions in this group to have the most up to date
   status.
*/

/* Function: isInterlockedStatusSet 
	Checks Interlock state.

	Parameters:
		p_InterlockedStatusSet - (out) true when interlock will allow laser to fire.
*/
SIDEKICK_SDK_DLL uint32_t SidekickSDK_isInterlockedStatusSet(DLS_SCI_DEVICE_HANDLE deviceHandle, bool *p_InterlockedStatusSet);

/* Function: isKeySwitchStatusSet
	Checks keyswitch state.

	Parameters:
		p_KeySwitchStatusSet - (out) true when keyswitch is in the enable position.
*/
SIDEKICK_SDK_DLL uint32_t SidekickSDK_isKeySwitchStatusSet(DLS_SCI_DEVICE_HANDLE deviceHandle, bool *p_KeySwitchStatusSet);

/* Function: isTempStatusSet
	Checks status of laser head temperature control.

	Parameters:
		p_TempStatusSet - (out) true when laser is within desired temperature range.
*/
SIDEKICK_SDK_DLL uint32_t SidekickSDK_isTempStatusSet(DLS_SCI_DEVICE_HANDLE deviceHandle, bool *p_TempStatusSet);

/* Function: isPostInProgressSet
	Checks if Power On Self Test is in progress.

	Parameters:
		p_PostInProgressSet - (out) true if POST is in progress.
*/
SIDEKICK_SDK_DLL uint32_t SidekickSDK_isPostInProgressSet(DLS_SCI_DEVICE_HANDLE deviceHandle, bool *p_PostInProgressSet);

/* Function: isPostSuccessfulSet
	Checks if Power On Self Test passed.

	Parameter:
		p_PostSuccessfulSet - (out) true of POST was successful.
*/
SIDEKICK_SDK_DLL uint32_t SidekickSDK_isPostSuccessfulSet(DLS_SCI_DEVICE_HANDLE deviceHandle, bool *p_PostSuccessfulSet);

/* Function: isScanningSet
	Checks if system is currently performing a scan.

	Parameters:
		p_ScanningSet - (out) true while system is scanning.
*/
SIDEKICK_SDK_DLL uint32_t SidekickSDK_isScanningSet(DLS_SCI_DEVICE_HANDLE deviceHandle, bool *p_ScanningSet);

/* Function: isManualTuning
	Checks if the system is tuning.

	Parameters:
		p_ManualTuning - (out) true if system is currently tuning to a commanded wavelength.
*/
SIDEKICK_SDK_DLL uint32_t SidekickSDK_isManualTuning(DLS_SCI_DEVICE_HANDLE deviceHandle, bool *p_ManualTuning);

/* Function: isTuned
	Checks to see if tuning is complete.

	Parameters:
	p_Tuned - (out) true if system has finished tuning to command wavelength.
*/
SIDEKICK_SDK_DLL uint32_t SidekickSDK_isTuned(DLS_SCI_DEVICE_HANDLE deviceHandle, bool *p_Tuned);

/* Function: isSystemError
	Checks if an error has occurred. If a system error has occurred the error code(s) can be read in one of two ways.
	The user can use the <ReadInfoSysInfo> and <GetInfoSysInfo> functions to read the last error and warning code that 
	were detected by the laser.  Alternatively, the user can use the <GetInfoSystemErrorsAndWarnings> and <GetInfoSystemErrorsList>
	functions to get a list of up to 20 errors that were detected by the laser.

	If an error occurs the error can be called by calling either <ClearFault> or <ExecInfoClearFault>.  See definitions below for 
	descriptions of each function.

	Parameters:
		p_SystemError - (out) true if an error has occurred.
*/
SIDEKICK_SDK_DLL uint32_t SidekickSDK_isSystemError(DLS_SCI_DEVICE_HANDLE deviceHandle, bool *p_SystemError);

/* Function: isSystemWarning
	Checks if a warning has occurred. If a system warning has occurred the warning code(s) can be read in one of two ways.
	The user can use the <ReadInfoSysInfo> and <GetInfoSysInfo> functions to read the last error and warning code that 
	were detected by the laser.  Alternatively, the user can use the <GetInfoSystemErrorsAndWarnings> and <GetInfoSystemWarningsList>
	functions to get a list of up to 20 warnings that were detected by the laser.

	If an error occurs the error can be called by calling either <ClearFault> or <ExecInfoClearFault>.  See definitions below for 
	descriptions of each function.

	Parameters:
		p_SystemWarning - (out) true if there is a system warning present.
*/
SIDEKICK_SDK_DLL uint32_t SidekickSDK_isSystemWarning(DLS_SCI_DEVICE_HANDLE deviceHandle, bool *p_SystemWarning);

/* Function: isEmissionOn
	Checks if laser is emission is enabled. If this status is true the laser should be considered to be emitting light.
	This function provides the status of the laser emission enable signal from the controller.

	Parameters:
		p_EmissionOn - (out) true if laser is emitting.
*/
SIDEKICK_SDK_DLL uint32_t SidekickSDK_isEmissionOn(DLS_SCI_DEVICE_HANDLE deviceHandle, bool *p_EmissionOn);

/* Function: isLaserArmed
	Checks laser arm status.

	Parameters:
		p_LaserArmed - (out) true if laser is armed.
*/
SIDEKICK_SDK_DLL uint32_t SidekickSDK_isLaserArmed(DLS_SCI_DEVICE_HANDLE deviceHandle, bool *p_LaserArmed);

/* Function: isScanSetupInProgress
	Checks if the system is preparing to start a scan.

	Parameters:
		p_ScanSetupInProgress - (out) true if the system is preparing to start a scan.
*/
SIDEKICK_SDK_DLL uint32_t SidekickSDK_isScanSetupInProgress(DLS_SCI_DEVICE_HANDLE deviceHandle, bool *p_ScanSetupInProgress);

/* Function: isLaserFiring
	Checks if the laser is firing. This provides the same status as and is redundant with <isEmissionOn>.

	Parameters:
		p_LaserFiring - (out) true if laser is firing.
*/
SIDEKICK_SDK_DLL uint32_t SidekickSDK_isLaserFiring(DLS_SCI_DEVICE_HANDLE deviceHandle, bool *p_LaserFiring);

/* Function: isInitDone
	Checks if system initialization is complete.

	Parameters:
		p_InitDone - (out) true if initialization is complete.
*/
//SIDEKICK_SDK_DLL uint32_t SidekickSDK_isInitDone(DLS_SCI_DEVICE_HANDLE deviceHandle, bool *p_InitDone);

/* Function: isBenchTemp1Faulty
	Checks if the bench temperature measurement is not working correctly.  Not used for Sidekick.

	Parameter:
		p_BenchTemp1Faulty - (out) true if bench temperature measurement is not working correctly.
*/
SIDEKICK_SDK_DLL uint32_t SidekickSDK_isBenchTemp1Faulty(DLS_SCI_DEVICE_HANDLE deviceHandle, bool *p_BenchTemp1Faulty);

/* Function: isBenchTemp2Faulty
	Not used for Sidekick.
*/
SIDEKICK_SDK_DLL uint32_t SidekickSDK_isBenchTemp2Faulty(DLS_SCI_DEVICE_HANDLE deviceHandle, bool *p_BenchTemp2Faulty);

/* Function: isPcbTempFaulty
	Checks for a PCB temperature fault. This indicates that the controller cannot read a valid temperature from the PCB
	temperature sensor.

	Parameter:
		p_PcbTempFaulty - (out) true if there was a PCB temperature fault.
*/
SIDEKICK_SDK_DLL uint32_t SidekickSDK_isPcbTempFaulty(DLS_SCI_DEVICE_HANDLE deviceHandle, bool *p_PcbTempFaulty);

/* Function: isMotionFaulted
	Indicates a fault with the motor drivetrain has occurred.  Possible faults are motor stall or motor overcurrent.

	Parameter:
		p_MotionFaulted - (out) true if there was a motion fault.
*/
SIDEKICK_SDK_DLL uint32_t SidekickSDK_isMotionFaulted(DLS_SCI_DEVICE_HANDLE deviceHandle, bool *p_MotionFaulted);

/* Function: isHeadInstalled
	Indicates whether or not a laser head is installed or not.

	Parameter:
		p_HeadInstalled - (out) true if a laser head is connected to and detected by the Sidekick controller.
*/
SIDEKICK_SDK_DLL uint32_t SidekickSDK_isHeadInstalled(DLS_SCI_DEVICE_HANDLE deviceHandle, bool *p_HeadInstalled);

/* Function: isVsrcGood
	Indicates if the status of the laser voltage supply is good.

	Parameter:
		p_VsrcGood - (out) true if the laser voltage supply is both ON and within +/- 1.0V of the target voltage.
*/
SIDEKICK_SDK_DLL uint32_t SidekickSDK_isVsrcGood(DLS_SCI_DEVICE_HANDLE deviceHandle, bool *p_VsrcGood);

/* Function: isHomingCompleted
	Used to determine if the tuning mechanism in the laser head was successfully initialized during the system 
	startup sequence.

	Parameter:
		p_HomingCompleted - (out) true if the laser head successfully initialized the tuning mechanism.
*/
SIDEKICK_SDK_DLL uint32_t SidekickSDK_isHomingCompleted(DLS_SCI_DEVICE_HANDLE deviceHandle, bool *p_HomingCompleted);

//-------------------------------------------------------------------------------------------------------------------------------------------//
/* Group: Laser System Information Functions
	These functions provide information about the Sidekick system and provide status for system errors/warnings, 
	laser status, temperature status, I/O status, etc.  
*/

/* Function: ReadInfoSysInfo
*	Queries the laser to receive system information. Call this function before GetInfoSysInfo.
*/
SIDEKICK_SDK_DLL uint32_t SidekickSDK_ReadInfoSysInfo(DLS_SCI_DEVICE_HANDLE deviceHandle);

/* Function: GetInfoSysInfo
*	Gets the laser system information that was obtained from the laser head from the previous ReadInfoSysInfo().
*
*	Parameters:
*		p_supportedIfc - (out) This is the command protocol version that is supported by the controller.  
*		p_sysInfo - (out) This is the type of system to which the SDK is connected. 
*		p_reserved - (out) Not currently used.
*/
SIDEKICK_SDK_DLL uint32_t SidekickSDK_GetInfoSysInfo(DLS_SCI_DEVICE_HANDLE deviceHandle, uint8_t *p_supportedIfc, uint8_t *p_sysInfo, uint32_t *p_reserved);

/* Function: ExecInfoClearFault
*	Clears different types of laser faults depending on the passed in flags.  Multiple flags can be bitwise ORed together to clear multiple fault types.
*
*	Parameters:
*		flags - Type of system fault to be cleared.  Possible flags are <SIDEKICK_SDK_CLEAR_ERROR_FAULT>, <SIDEKICK_SDK_CLEAR_WARNING_FAULT>,
*				<SIDEKICK_SDK_CLEAR_ERROR_LIST>, and <SIDEKICK_SDK_CLEAR_WARNING_LIST>
*/
SIDEKICK_SDK_DLL uint32_t SidekickSDK_ExecInfoClearFault(DLS_SCI_DEVICE_HANDLE deviceHandle, uint8_t flags);

/* Function: ClearFault
*	Clears the most recent Error and Warning from the system.  These are the Errors and Warnings that are reported by 
*	<getSystemErrorWord> and <getSystemWarningWord> functions.
*/
SIDEKICK_SDK_DLL uint32_t SidekickSDK_ClearFault(DLS_SCI_DEVICE_HANDLE deviceHandle);

/* Function: ClearErrorList
*	Clears the system error list which stores up to the last 20 system errors.	
*/
SIDEKICK_SDK_DLL uint32_t SidekickSDK_ClearErrorList(DLS_SCI_DEVICE_HANDLE deviceHandle);

/* Function: ClearWarningList
*	Clears the system warning list which stores up to the last 20 system warnings. 
*/
SIDEKICK_SDK_DLL uint32_t SidekickSDK_ClearWarningList(DLS_SCI_DEVICE_HANDLE deviceHandle);

/* Function: GetInfoSystemErrorsAndWarnings
	Depracated: Please use <GetInfoSystemErrorsList> instead.
	Reads the system error and warning lists from the laser controller and stores them in a local data structure. 
*/
SIDEKICK_SDK_DLL uint32_t SidekickSDK_GetInfoSystemErrorsAndWarnings(DLS_SCI_DEVICE_HANDLE deviceHandle);

/* Function: GetInfoSystemErrorsList
*	Reads the system errors and warnings from the laser and copies the system error and warning list from the 
	local data structure to the structure pointed to by the passed in parameter.

	Parameters:
		sysErrorsAndWarnings - Pointer to structure of type DLS_SCI_INFO_SYSTEM_ERRORS
*/
SIDEKICK_SDK_DLL uint32_t SidekickSDK_GetInfoSystemErrorsList(DLS_SCI_DEVICE_HANDLE deviceHandle, DLS_SCI_INFO_SYSTEM_ERRORS * sysErrorsAndWarnings);

/* Function: GetInfoSystemWarningsList
	Depracated: Please use <GetInfoSystemErrorsList> instead.
	Reads the system errors and warnings from the laser and copies the system error and warning list from the 
	local data structure to the structure pointed to by the passed in parameter.

	Parameters:
		sysWarningsList - Pointer to structure of type DLS_SCI_INFO_SYSTEM_ERRORS
*/
SIDEKICK_SDK_DLL uint32_t SidekickSDK_GetInfoSystemWarningsList(DLS_SCI_DEVICE_HANDLE deviceHandle, DLS_SCI_INFO_SYSTEM_ERRORS * sysWarningsList);

/* Function: ReadInfoLight
*	Queries the laser to read information pertaining to emission status and wavelength. Call this function before <GetInfoLight>.
*/
SIDEKICK_SDK_DLL uint32_t SidekickSDK_ReadInfoLight(DLS_SCI_DEVICE_HANDLE deviceHandle);

/* Function: GetInfoLight
*	Gets the information received from the last <ReadInfoLight> call.
*
*	Parameters:
*		p_lightStatus - (out) true if laser emission is enabled, false otherwise.
*		p_currentWW - (out) Current wavelength as measured internally by the laser motion control system.
*		p_currentWWUnit - (out) <Wavelength Units> of the wavelength value.
*		p_curQcl - (out) Current emitting QCL.  For Sidekick this is always 1, but for MIRcat this can be 1-4.
*/
SIDEKICK_SDK_DLL uint32_t SidekickSDK_GetInfoLight(DLS_SCI_DEVICE_HANDLE deviceHandle, uint8_t *p_lightStatus, float *p_currentWW, uint8_t *p_currentWWUnit, uint8_t *p_curQcl);

/* Function: ReadInfoQclInfo
*	Queries the laser to read QCL status. Call before <GetInfoQclInfo>.
*
*	Parameters:
*		qcl_slot - qcl_slot to query. Sidekick only supports one slot, so the value does not matter.
*/
SIDEKICK_SDK_DLL uint32_t SidekickSDK_ReadInfoQclInfo(DLS_SCI_DEVICE_HANDLE deviceHandle, uint8_t qcl_slot);

/* Function: GetInfoQclInfo
*	Gets the information received from last <ReadInfoQclInfo> call.
*
*	Parameters:
*		p_channel - (out) For Sidekick this does not matter (included for MIRcat or other multi-channel systems).
*		p_status - (out) Status mask that indicates different QCL statuses.  See <QCL Statuses>.
*		p_qclVoltage - (out) Measured QCL Voltage in Volts.  This measurement is not supported on legacy heads (HFQD, MHF).
*		p_qclVfet - (out) Measured drive FET Voltage in Volts.  This measurement is not supported on legacy heads (HFQD, MHF).
*		p_qclVsrc - (out) Measured QCL Voltage source output in Volts.
*		p_qclCurrent - (out) Measured QCL Current in Amps.  This measurement is not supported on legacy heads (HFQD, MHF).
*		p_iSns - (out) Measured current out of VSRC power supply module in Amps.
*		p_currentSrc - (out) Measured current out of MHF current source in Amps.
*		p_qclTemp - (out) QCL Temperature in degrees C.
*		p_vQcHi - (out) Secondary voltage generated from p_qclVsrc.
*/
SIDEKICK_SDK_DLL uint32_t SidekickSDK_GetInfoQclInfo(DLS_SCI_DEVICE_HANDLE deviceHandle, uint8_t *p_channel, uint8_t *p_status,
							float *p_qclVoltage, float *p_qclVfet, float *p_qclVsrc,
							float *p_qclCurrent, float *p_iSns, float *p_currentSrc,
							float *p_qclTemp, float * p_vQcHi);
							
/* Function: ReadInfoTecInfo
*	Queries the laser to read the status of the thermal control system (TEC).  Call this prior to <GetInfoTecInfo>.
*
*	Parameters:
*		channel - For Sidekick this can be set to any value.  For MIRcat or other multi-channel systems this must be set to the 
*				  channel that is to be read.
*/
SIDEKICK_SDK_DLL uint32_t SidekickSDK_ReadInfoTecInfo(DLS_SCI_DEVICE_HANDLE deviceHandle, uint8_t channel);

/* Function: GetInfoTecInfo
*	Gets the TEC info from the last <ReadInfoTecInfo> call.
*
*	Parameters:
*		p_channel - (out) The channel for which the TEC information was read by the last <ReadInfoTecInfo> call.
*		p_status - (out) Status of the TEC and thermal management system.  See <TEC Statuses>.
*		p_tecTemp - (out) TEC Temperature in degrees C.
*		p_tecCurrent - (out) Magnitude of TEC Current in Amps.
*		p_tecVoltage - (out) TEC Voltage in Volts.
*/
SIDEKICK_SDK_DLL uint32_t SidekickSDK_GetInfoTecInfo(DLS_SCI_DEVICE_HANDLE deviceHandle, uint8_t *p_channel, uint8_t *p_status,
														float *p_tecTemp, float *p_tecCurrent, float *p_tecVoltage);

/* Function: ReadInfoSysTemperatures
*	Queries the laser to read the temperatures. Temperatures can be retrieved with the next <GetInfoSysTemperatures> call.
*/
SIDEKICK_SDK_DLL uint32_t SidekickSDK_ReadInfoSysTemperatures(DLS_SCI_DEVICE_HANDLE deviceHandle);
/* Function: GetInfoSysTemperatures
*	Gets the temperature data retrieved from the laser during the last <ReadInfoSysTemperatures> call.
*
*	Parameters:
*		p_temp1 - (out) Controller PCB Temperature, degrees Celsius.
*		p_temp2 - (out) Laser head case temperature, degrees Celsius.
*		p_temp3 - (out) None for Sidekick.
*		p_humidity1 - (out) Relative Humidity on the Sidekick PCB in percentage units.
*		p_humidity2 - (out) None for Sidekick.
*		p_aux_temp1 - (out) None for Sidekick.
*		p_aux_temp2 - (out) None for Sidekick.
*/
SIDEKICK_SDK_DLL uint32_t SidekickSDK_GetInfoSysTemperatures(DLS_SCI_DEVICE_HANDLE deviceHandle, float *p_temp1, float *p_temp2, float *p_temp3,
						 float *p_humidity1, float *p_humidity2, 
						 float *p_aux_temp1, float *p_aux_temp2 );

/* Function: ReadInfoSysVoltages
*	Reads the voltages on the board. Retrieve data read using <GetInfoSysVoltages>.
*/
SIDEKICK_SDK_DLL uint32_t SidekickSDK_ReadInfoSysVoltages(DLS_SCI_DEVICE_HANDLE deviceHandle);

/* Function: GetInfoSysVoltages
*	Gets the voltages read from the last ReadInfoSysVoltages() call. The function retrieves voltages one at a time using the predefined voltage bus <Voltage Indexes>.
*
*	Parameters:
*		index - Index of the voltage to retrieve.  See <Voltage Indexes>.
*		p_voltage - (out) Voltage value read.
*/
SIDEKICK_SDK_DLL uint32_t SidekickSDK_GetInfoSysVoltages(DLS_SCI_DEVICE_HANDLE deviceHandle, uint8_t index, float *p_voltage);

/* Function: ReadInfoIoStatus
*	Reads the status of the I/O signals on the Sidekick DB-15 connector.
*/
SIDEKICK_SDK_DLL uint32_t SidekickSDK_ReadInfoIoStatus(DLS_SCI_DEVICE_HANDLE deviceHandle);
/* Function: GetInfoIoStatus
*	Gets the value of the I/O statuses as read from <ReadInfoIoStatus>.
*
*	Parameters:
*		p_io_status - (out) Bitmask that indicates the state of the DB-15 I/O signals.  Use with <IO Status Masks>. 
*/
SIDEKICK_SDK_DLL uint32_t SidekickSDK_GetInfoIoStatus(DLS_SCI_DEVICE_HANDLE deviceHandle, uint32_t *p_io_status);

//-------------------------------------------------------------------------------------------------------------------------------------------------------------//
/* Group: Tuning and Scaning Functions
	These functions command manual tuning and scanning of laser wavelength. Scanning functions use three functions: *ReadWrite*, *Set*, and *Get*. For setting scan
	 parameters, call *Set* first, then call *ReadWrite* with fWrite = true. For getting scan parameters, call *ReadWrite* with fWrite = false, then call *Get*.
	 Every call of the *ReadWrite* functions will update the internal SDK scan parameters corresponding that function.
*/

/* Function: SetTuneToWW
	Manual tuning setup command. Set the wavelength to tune to. Will tune the the specified wavelength next ExecTuneToWW() call.

	Parameters:
		ww_unit - <Wavelength Units> for the wave parameter.
		wave - Wavelength to tune to, in units specified by ww_unit.
		preferred_qcl - For Sidekick this parameter can be set to 0.  For MIRcat systems it is used to always tune to a specific channel if the wavelength is supported on that channel.
*/
SIDEKICK_SDK_DLL uint32_t SidekickSDK_SetTuneToWW(DLS_SCI_DEVICE_HANDLE deviceHandle, uint8_t ww_unit, float wave, uint8_t preferred_qcl);

/* Function: ExecTuneToWW
	Executes the manual tuning commmand set up in the <SetTuneToWW> call.
*/
SIDEKICK_SDK_DLL uint32_t SidekickSDK_ExecTuneToWW(DLS_SCI_DEVICE_HANDLE deviceHandle);

/* Function: ReadWriteScanMode
	 Reads or Writes scan mode from or to laser head. For use with <GetScanMode> and <SetScanMode>.

	 Parameters:
		fWrite - set to true to write parameters, false to read.
*/
SIDEKICK_SDK_DLL uint32_t SidekickSDK_ReadWriteScanMode(DLS_SCI_DEVICE_HANDLE deviceHandle, bool fWrite);

/* Function: GetScanMode
	Get scan mode internally stored in SDK. To make sure that the scan mode in SDK is up to date, call <ReadWriteScanMode> with
	fWrite = false before calling this function.

	Parameters:
		p_scanMode - (out) Scan mode. Decode with <Scan Operations>.
*/
SIDEKICK_SDK_DLL uint32_t SidekickSDK_GetScanMode(DLS_SCI_DEVICE_HANDLE deviceHandle, uint8_t *p_scanMode);

/* Function: SetScanMode
	Sets the scan mode internally stored in SDK. Call <ReadWriteScanMode> with fWrite = true to send to laser head.

	Parameters:
		scanMode - Scan mode, use <Scan Operations>.
*/
SIDEKICK_SDK_DLL uint32_t SidekickSDK_SetScanMode(DLS_SCI_DEVICE_HANDLE deviceHandle, uint8_t scanMode);

/* Function: ReadWriteSingleWWParams
	Read/Write single wavelength params is not used for Sidekick, only for MIRcat.

	Parameters:
		fWrite - set to true to write parameters, false to read.
*/
SIDEKICK_SDK_DLL uint32_t SidekickSDK_ReadWriteSingleWWParams(DLS_SCI_DEVICE_HANDLE deviceHandle, bool fWrite);

/* Function: GetSingleWWParams
	Get Single Wavelength parameters, used with <ReadWriteSingleWWParams>.  This is not used for Sidekick, only for MIRcat.

	Parameters:
		p_units - (out) <Wavelength Units>.
		p_wave - (out) Wavelength in units of p_units.
		p_duration - (out) Depracated, no longer used.
		p_preferredQcl - (out) For MIRcat, the preferred QCL is the desired QCL to use.  If the wavelength is supported by the preferred QCL, it will always tune to the preferred QCL.
*/
SIDEKICK_SDK_DLL uint32_t SidekickSDK_GetSingleWWParams(DLS_SCI_DEVICE_HANDLE deviceHandle, uint8_t *p_units, float *p_wave, uint32_t *p_duration, uint8_t *p_preferredQcl);

/* Function: SetSingleWWParams
	Set Single Wavelength parameters, used with <ReadWriteSingleWWParams>.  This is not used for Sidekick, only for MIRcat.

	Parameters:
		units - <Wavelength Units>
		wave - Wavelength in units of units.
		duration - Depracated, no longer used.
		preferredQcl - For MIRcat, the preferred QCL is the desired QCL to use.  If the wavelength is supported by the preferred QCL, it will always tune to the preferred QCL.
*/
SIDEKICK_SDK_DLL uint32_t SidekickSDK_SetSingleWWParams(DLS_SCI_DEVICE_HANDLE deviceHandle, uint8_t units, float wave, uint32_t duration, uint8_t preferredQcl);

/* Function: ReadWriteSweepParams
	Reads or Writes Sweep parameters from to to laser head. For use with <GetSweepParams> and <SetSweepParams>.

	Parameters:
		fWrite - set to true to write parameters, false to read.
*/
SIDEKICK_SDK_DLL uint32_t SidekickSDK_ReadWriteSweepParams(DLS_SCI_DEVICE_HANDLE deviceHandle, bool fWrite);

/* Function: GetSweepParams
	Gets the sweep parameters obtained from the last <ReadWriteSweepParams> call.

	Parameters:
		p_units - (out) <Wavelength Units>.
		p_start_ww - (out) Starting wavelength for scan.
		p_stop_ww - (out) End wavelength for scan.
		p_speed - (out) Scan speed in <Wavelength Units> per second.
		p_num_scans - (out) Number of times to repeat the scan.  Set to 0 to perform an infinite number of scans.
		p_pref_qcl - (out) For Sidekick this can just be set to 0 (is not used).  For MIRcat this is the preferred channel to use during the scan.
		p_bidirectional - (out) 0 = single direction scan, non-zero = bidirectional scan that sweeps from start to stop and stop to start for one scan.
*/
SIDEKICK_SDK_DLL uint32_t SidekickSDK_GetSweepParams(DLS_SCI_DEVICE_HANDLE deviceHandle, uint8_t *p_units, float *p_start_ww, float *p_stop_ww, float *p_speed,
															uint16_t *p_num_scans, uint8_t *p_pref_qcl, uint8_t *p_bidirectional );
/* Function: SetSweepParams
	Sets the sweep parameters. Send to laser head using <ReadWriteSweepParams> with fWrite = true;

	Parameters:
		units - <Wavelength Units>.
		start_ww - Starting wavelength for scan.
		stop_ww - End wavelength for scan.
		speed - Scan speed in <Wavelength Units> per second.
		num_scans - Number of times to repeat the scan.  Set to 0 to perform an infinite number of scans.
		pref_qcl - For Sidekick this can just be set to 0 (is not used).  For MIRcat this is the preferred channel to use during the scan.
		bidirectional - 0 = single direction scan, non-zero = bidirectional scan that sweeps from start to stop and stop to start for one scan.
*/
SIDEKICK_SDK_DLL uint32_t SidekickSDK_SetSweepParams(DLS_SCI_DEVICE_HANDLE deviceHandle, uint8_t units, float start_ww, float stop_ww, float speed,
															uint16_t num_scans, uint8_t pref_qcl, uint8_t bidirectional );
/* Function: ReadWriteWavelengthTrigParams
	Reads or Writes wavelength trigger parameters from/to laser head.

	Parameters:
		fWrite - set to true to write parameters, false to read.
*/
SIDEKICK_SDK_DLL uint32_t SidekickSDK_ReadWriteWavelengthTrigParams(DLS_SCI_DEVICE_HANDLE deviceHandle, bool fWrite);

/* Function: GetWavelengthTrigParams
	Gets the wavelength trigger parameters from the last call to <ReadWriteWavelengthTrigParams>.

	Parameters:
		p_units - (out) <Wavelength Units>.
		p_start_ww - (out) Starting wavelength
		p_stop_ww - (out) Ending wavelength
		p_spacing - (out) Interval between each wavelength trigger in <Wavelength Units>.
		p_qcl - (out) QCL is used for MIRcat systems in order to get the wavelength trigger parameters for a specific channel. 
*/
SIDEKICK_SDK_DLL uint32_t SidekickSDK_GetWavelengthTrigParams(DLS_SCI_DEVICE_HANDLE deviceHandle, uint8_t *p_units, float *p_start_ww, float *p_stop_ww, float *p_spacing, uint8_t *p_qcl);

/* Function: SetWavelengthTrigParams
	Sets the wavelength trigger parameters prior to calling <ReadWriteWavelengthTrigParams> with parameter true.

	Parameters:
		units - <Wavelength Units>.
		start_ww - Starting wavelength
		stop_ww - Ending wavelength
		spacing - Interval between each wavelength trigger in <Wavelength Units>.
		qcl - QCL is used for MIRcat systems in order to set the wavelength trigger parameters for a specific channel.
*/
SIDEKICK_SDK_DLL uint32_t SidekickSDK_SetWavelengthTrigParams(DLS_SCI_DEVICE_HANDLE deviceHandle, uint8_t units, float start_ww, float stop_ww, float spacing, uint8_t qcl);


SIDEKICK_SDK_DLL uint32_t SidekickSDK_ReadNumWlTrigMarkers(DLS_SCI_DEVICE_HANDLE deviceHandle, uint8_t type, uint8_t start_stop_units);
SIDEKICK_SDK_DLL uint32_t SidekickSDK_GetNumWlTrigMarkersValues(DLS_SCI_DEVICE_HANDLE deviceHandle, uint16_t * num_markers, DLS_SCI_SCAN_WAVE_TYPE_UNION * start, DLS_SCI_SCAN_WAVE_TYPE_UNION * stop);
SIDEKICK_SDK_DLL uint32_t SidekickSDK_ReadWlTrigValues(DLS_SCI_DEVICE_HANDLE deviceHandle, uint8_t type, uint16_t startAddress, uint16_t numMarkers, uint8_t markerUnits);
SIDEKICK_SDK_DLL uint32_t SidekickSDK_GetWlTrigValue(DLS_SCI_DEVICE_HANDLE deviceHandle, uint8_t type, uint16_t address, DLS_SCI_SCAN_WAVE_TYPE_UNION * markerValue);


/* Function: ReadWriteProcessTrigParams
	Reads or Writes Process Trigger parameters to laser head/controller.  Process trigger parameters are used for Step & Measure and Multi-Spectral scans to
	control when the laser moves to the next step in the scan.

	Parameters:
		fWrite - set to true to write parameters, false to read.
*/
SIDEKICK_SDK_DLL uint32_t SidekickSDK_ReadWriteProcessTrigParams(DLS_SCI_DEVICE_HANDLE deviceHandle, bool fWrite);

/* Function: GetProcessTrigParams
	Gets the process trigger mode for step scans.

	Parameters:
		p_procTrigMode - (out) Process Trigger Mode.  See <Process Trigger Mode>.
*/
SIDEKICK_SDK_DLL uint32_t SidekickSDK_GetProcessTrigParams(DLS_SCI_DEVICE_HANDLE deviceHandle, uint8_t *p_procTrigMode);

/* Function: SetProcessTrigParams
	Sets the process trigger mode for Step scans.  

	Parameters:
		procTrigMode - Process Trigger Mode.  See <Process Trigger Mode>.
*/
SIDEKICK_SDK_DLL uint32_t SidekickSDK_SetProcessTrigParams(DLS_SCI_DEVICE_HANDLE deviceHandle, uint8_t procTrigMode);

/* Function: ReadWriteStepMeasureParams
	Reads or Writes Step & Measure Scan parameters to laser head/controller.  Used with <GetStepMeasureParams> amd <SetStepMeasureParams>.

	Parameters:
		fWrite - set to true to write parameters, false to read.
*/
SIDEKICK_SDK_DLL uint32_t SidekickSDK_ReadWriteStepMeasureParams(DLS_SCI_DEVICE_HANDLE deviceHandle, bool fWrite);

/* Function: GetStepMeasureParams
	Gets the parameters for a Step & Measure scan.  A Step & Measure scan is one in which the laser is given a start and stop wavelength
		and it will step from start to stop in increments of p_step.  Call after <ReadWriteStepMeasureParams>.

	Parameters:
		p_units - (out) <Wavelength Units>.
		p_start_ww - (out) Starting wavelength.
		p_stop_ww - (out) Ending wavelength.
		p_step - (out) Wavelength is incremented/decremented by Step Size for each step.  
		p_num_scans - (out) Number of scans that will be performed.  Setting this to 0 means infinite scans.		
		p_keep_on - (out) Non-Zero value Keeps the laser on between steps, zero turns laser emission off between steps.
		p_bidirectional - (out) Non-Zero value sets bi-directional scan true, Zero false.  If true, scan will increment 
							from start to stop and then stop to start for one scan.
		p_dwell_time_ms - (out) For each step, once the laser is tuned and turned on, the scan will stay at this wavelength for this time (in ms).
		p_trans_time_ms - (out) After dwell time, laser will start a timer as it starts tuning.  If keep_on is 0, laser will be off for this duration.  
							Tuned signal will be low during this time.  This can be set to 0 and time in between lasers will be limited only by mechanical tuning time.
*/
SIDEKICK_SDK_DLL uint32_t SidekickSDK_GetStepMeasureParams(DLS_SCI_DEVICE_HANDLE deviceHandle,
																uint8_t *p_units, float *p_start_ww, float *p_stop_ww, float *p_step,
																uint16_t *p_num_scans, uint8_t *p_keep_on, uint8_t *p_bidirectional,
																uint32_t *p_dwell_time_ms, uint32_t *p_trans_time_ms);
/* Function: SetStepMeasureParams
	Sets the parameters for a Step & Measure scan.  A Step & Measure scan is one in which the laser is given a start and stop wavelength
		and it will step from start to stop in increments of p_step.  Call before <ReadWriteStepMeasureParams> true.

	Parameters:
		units - <Wavelength Units>.
		start_ww - Starting wavelength.
		stop_ww - Ending wavelength.
		step - Wavelength is incremented/decremented by Step Size for each step.  
		num_scans - Number of scans that will be performed.  Setting this to 0 means infinite scans.		
		keep_on - Non-Zero value Keeps the laser on between steps, zero turns laser emission off between steps.
		bidirectional - Non-Zero value sets bi-directional scan true, Zero false.  If true, scan will increment 
			from start to stop and then stop to start for one scan.
		dwell_time_ms - For each step, once the laser is tuned and turned on, the scan will stay at this wavelength for this time (in ms).
		trans_time_ms - After dwell time, laser will start a timer as it starts tuning.  If keep_on is 0, laser will be off for this duration.  
			Tuned signal will be low during this time.  This can be set to 0 and time in between lasers will be gated by mechanical tuning time.
*/
SIDEKICK_SDK_DLL uint32_t SidekickSDK_SetStepMeasureParams(DLS_SCI_DEVICE_HANDLE deviceHandle,
															uint8_t units, float start_ww, float stop_ww, 
															float step, uint16_t num_scans, uint8_t keep_on, uint8_t bidirectional, 
															uint32_t dwell_time_ms, uint32_t trans_time_ms );

/* Function: ReadWriteMultispectralParams
	Used to read/write the global parameters for a Multi-Spectral scan.  Use with <GetMultiSpectralParams> and <SetMultiSpectralParams>.

	Parameters:
		fWrite - set to true to write parameters, false to read.
*/
SIDEKICK_SDK_DLL uint32_t SidekickSDK_ReadWriteMultiSpectralParams(DLS_SCI_DEVICE_HANDLE deviceHandle, bool fWrite);
/* Function: GetMultiSpectralParams
	Get parameters read from get during last call to <ReadWriteMultiSpectralParams>.

	Parameters:
		p_units - (out) <Wavelength Units> to use for this scan.
		p_numSteps - (out) Number of wavelength steps in this scan.		
		p_numScans - (out) Number of scans.
		p_bidirectional - (out) Used to perform a bidirectional scan that goes from start to stop, then stop to start for a single scan.
		p_dataAllocated - (out) If a data structure for this size scan was successfully allocated.  This must be true in order to set individual step parameters.
*/
SIDEKICK_SDK_DLL uint32_t SidekickSDK_GetMultiSpectralParams(DLS_SCI_DEVICE_HANDLE deviceHandle, uint8_t *p_units,
																uint16_t *p_numSteps, uint16_t *p_numScans, bool *p_bidirectional,
																bool *p_dataAllocated);
/* Function: SetMultiSpectralParams
	Set parameters to be written to head during next call to <ReadWriteMultiSpectralParams>.

	Parameters:
		units - <Wavelength Units> to use for this scan.
		numSteps - Number of wavelength steps in this scan.		
		numScans - Number of scans.
		bidirectional - Used to perform a bidirectional scan that goes from start to stop, then stop to start for a single scan.
*/
SIDEKICK_SDK_DLL uint32_t SidekickSDK_SetMultiSpectralParams(DLS_SCI_DEVICE_HANDLE deviceHandle, uint8_t units, 
															uint16_t numSteps, uint16_t numScans, bool bidirectional);


/* Function: ReadWriteMultiSpectralElementParams
	Read/Write parameters for a specific Multi-Spectral scan step.  Individual step parameters can only be setup after successfully 
	initializing the Multi-Spectral scan using <SetMultiSpectralParams> and <ReadWriteMultispectralParams>.

	Parameters:
		fWrite - set to true to write parameters, false to read.
*/
SIDEKICK_SDK_DLL uint32_t SidekickSDK_ReadWriteMultiSpectralElementParams(DLS_SCI_DEVICE_HANDLE deviceHandle, bool fWrite);

/* Function: GetMultiSpectralElement
	Get the parameters from the last call to <ReadWriteMultiSpectralElementParams> (true).  The index returned is the index of the step
	that was last configured.  

	Parameters:
		p_idx - (out) Index of the last Multi-Spectral scan step that was configured.
		p_scan_ww - (out) Wavelength for this step in the same units that were used in <SetMultiSpectralParams> when setting up a scan.
		p_dwell_time_ms - (out) Once the laser is tuned and turned on, the scan will stay at this wavelength for this time (in ms). 
		p_transition_time_ms - (out) After dwell time, laser will start a timer as it starts tuning.  If keep_on is 0, laser will be off for this duration.  
							Tuned signal will be low during this time.  This can be set to 0 and time in between lasers will be gated by mechanical tuning time.
		p_keepLaserOn - (out) Once the dwell time for this step is complete, the system will leave the laser on if this is set true, 
						or will turn the laser off if this is set false.
		p_stepInitialized - (out) True if this step was successfully initialized, false otherwise.  Initialization can fail if there is not enough memory 
							on the Sidekick controller for this step or if the desired wavelength is out of the valid range.
*/
SIDEKICK_SDK_DLL uint32_t SidekickSDK_GetMultiSpectralElement(DLS_SCI_DEVICE_HANDLE deviceHandle, uint16_t *p_idx,
															float *p_scan_ww, uint32_t *p_dwell_time_ms, uint32_t *p_transition_time_ms,
															bool *p_keepLaserOn, bool *p_stepInitialized);

/* Function: SetMultiSpectralElement
	Set the parameters for a particular step in a Multi-Spectral scan.  

	Parameters:
		idx - Index of the last Multi-Spectral scan step that was configured.
		scan_ww - Wavelength for this step in the same units that were used in <SetMultiSpectralParams> when setting up a scan.
		dwell_time_ms - Once the laser is tuned and turned on, the scan will stay at this wavelength for this time (in ms). 
		transition_time_ms - After dwell time, laser will start a timer as it starts tuning.  If keep_on is 0, laser will be off for this duration.  
							Tuned signal will be low during this time.  This can be set to 0 and time in between lasers will be gated by mechanical tuning time.
		keepLaserOn - Once the dwell time for this step is complete, the system will leave the laser on if this is set true, 
						or will turn the laser off if this is set false.
*/
SIDEKICK_SDK_DLL uint32_t SidekickSDK_SetMultiSpectralElement(DLS_SCI_DEVICE_HANDLE deviceHandle, uint16_t idx, float scan_ww,
																uint32_t dwell_time_ms, uint32_t transition_time_ms, bool keepLaserOn);

/* Function: ExecuteScanOperation
	Execute the scan operation currently set.
*/
SIDEKICK_SDK_DLL uint32_t SidekickSDK_ExecuteScanOperation(DLS_SCI_DEVICE_HANDLE deviceHandle);
/* Function: GetScanOperation
	Get the currently set scan operation.

	Parameters:
		operation - (out) <Scan Operations>
*/
SIDEKICK_SDK_DLL uint32_t SidekickSDK_GetScanOperation(DLS_SCI_DEVICE_HANDLE deviceHandle, uint8_t *operation);
/* Function: SetScanOperation
	Set the scan operation.

	Parameters:
		operation - <Scan Operations>
*/
SIDEKICK_SDK_DLL uint32_t SidekickSDK_SetScanOperation(DLS_SCI_DEVICE_HANDLE deviceHandle, uint8_t operation);

/* Function: ReadScanProgress
	Read progress of current scan operation from the head. Follow with <GetScanProgress>.
*/
SIDEKICK_SDK_DLL uint32_t SidekickSDK_ReadScanProgress(DLS_SCI_DEVICE_HANDLE deviceHandle);
/* Function: GetScanProgress
	Gets the scan progress information read from the head during last <ReadScanProgress>.

	Parameters:
		p_scan_progress_mask - (out) Bitmask that provides different <Scan statuses>.
		p_cur_scan_num - (out) number of scan currently being executed.
		p_cur_scan_perecent - (out) percentage complete of the current scan number. 
*/
SIDEKICK_SDK_DLL uint32_t SidekickSDK_GetScanProgress(DLS_SCI_DEVICE_HANDLE deviceHandle, uint8_t *p_scan_progress_mask, uint16_t *p_cur_scan_num, uint16_t *p_cur_scan_percent);


/* Function: ReadWriteVcmModulationParams
	Read/Write parameters for VCM wavelength modulation.

	Parameters:
		fWrite - set to true to write parameters, false to read.
*/
SIDEKICK_SDK_DLL uint32_t SidekickSDK_ReadWriteVcmModulationParams(DLS_SCI_DEVICE_HANDLE deviceHandle, bool fWrite);

/* Function: GetVcmModulationParams
	Gets the VCM wavelength modulation parameters from the last call to <ReadWriteVcmModulationParams>.

	Parameters:
		p_enable - (out) Enable/Disable the specified type of VCM modulation.
		p_type - (out) Modulation type <VCM Modulation>.
		p_frequencyHz - (out) Frequency (Hz) of the specified type of VCM modulation.
		p_curModCounts - (out) Amplitude of current modulation in DAC counts for VCM Current Modulation.
		p_posModCounts - (out) Amplitude in encoder counts for VCM Position Modulation.
*/
SIDEKICK_SDK_DLL uint32_t SidekickSDK_GetVcmModulationParams(DLS_SCI_DEVICE_HANDLE deviceHandle, bool * p_enable, uint8_t * p_type,
															 uint32_t * p_frequencyHz, uint16_t * p_curModCounts, uint32_t * p_posModCounts);

/* Function: SetVcmModulationParams
	Sets the VCM wavelength modulation parameters for the next call to <ReadWriteVcmModulationParams>.

	Parameters:
		enable - Enable/Disable the specified type of VCM modulation.
		type - Modulation type <VCM Modulation>.
		frequencyHz - Frequency (Hz) of the specified type of VCM modulation.
		curModCounts - Amplitude of current modulation in DAC counts for VCM Current Modulation.
		posModCounts - Amplitude in encoder counts for VCM Position Modulation.
*/
SIDEKICK_SDK_DLL uint32_t SidekickSDK_SetVcmModulationParams(DLS_SCI_DEVICE_HANDLE deviceHandle, bool enable, uint8_t type,
															 uint32_t frequencyHz, uint16_t curModCounts, uint32_t posModCounts);


//-------------------------------------------------------------------------------------------------------------------------------------------------------------//
/* Group: Laser Command Functions
	Functions for setting up QCL parameters and enabling laser emission. Some of these functions take QCL Slot number as a parameter.
	Sidekick only supports one head at a time, so this parameter is not used. Set this parameter to 0.
*/

/* Function: ReadWriteLaserQclParams
	Read or Write laser parameters to given QCL Slot. Sidekick only supports one laser head, so qcl_slot does not matter.

	Parameters:
		fWrite - Set to true to write parameters, false to read.
		qcl_slot - Set to 0.
*/
SIDEKICK_SDK_DLL uint32_t SidekickSDK_ReadWriteLaserQclParams(DLS_SCI_DEVICE_HANDLE deviceHandle, bool fWrite, uint8_t qcl_slot);

/* Function: GetLaserQclParams
	Get laser parameters read from last <ReadWriteLaserQclParams> call.

	Parameters:
		p_qclSlot - (out) Not used by Sidekick.
		p_pulseRateHz - (out) Pulse Rate in hertz.
		p_pulseWidthNs - (out) Pulse Width in nanoseconds.
		p_currentMa - (out) QCL current in milliamperes.
		p_temperature - (out) Target temperature in degrees C.
		p_laserMode - (out) <Laser Mode>.
		p_pulseMode - (out) <Trigger Mode>.
		p_vsrc - (out) QCL Voltage Source voltage, in volts.
*/
SIDEKICK_SDK_DLL uint32_t SidekickSDK_GetLaserQclParams(DLS_SCI_DEVICE_HANDLE deviceHandle, uint8_t *p_qclSlot, uint32_t *p_pulseRateHz, uint32_t *p_ulseWidthNs, 
														uint16_t *p_currentMa, float *p_temperature, uint8_t *p_laserMode, uint8_t *p_pulseMode, float *p_vsrc);

/* Function: SetLaserQclParams
	Set laser parameters to write to laser system with next <ReadWriteLaserQclParams> call.

	Parameters:
		qclSlot - Not used by Sidekick, set to 0.
		pulseRateHz - Pulse Rate in hertz.
		pulseWidthNs - Pulse Width in nanoseconds.
		currentMa - QCL current in milliamperes.
		temperature - Target temperature in degrees C.
		laserMode - <Laser Mode>.
		pulseMode - <Trigger Mode>.
		vsrc - QCL Voltage Source voltage, in volts.
*/
SIDEKICK_SDK_DLL uint32_t SidekickSDK_SetLaserQclParams(DLS_SCI_DEVICE_HANDLE deviceHandle, uint8_t qclSlot, uint32_t pulseRateHz, uint32_t pulseWidthNs, 
														uint16_t currentMa, float temperature, uint8_t laserMode, uint8_t pulseMode, float vsrc );

/* Function: ExecLaserOnOff
	Command the laser to turn on or off based on last <SetLaserOnOff> call.
*/
SIDEKICK_SDK_DLL uint32_t SidekickSDK_ExecLaserOnOff(DLS_SCI_DEVICE_HANDLE deviceHandle);

/* Function: SetLaserOnOff
	Set On or Off command of the laser, write to system with <ExecLaserOnOff>.

	Parameters:
		qcl_slot - Set to 0.
		on - true for on, false for off.
*/
SIDEKICK_SDK_DLL uint32_t SidekickSDK_SetLaserOnOff(DLS_SCI_DEVICE_HANDLE deviceHandle,  uint8_t qcl_slot, bool on);

/* Function: ExecLaserArmDisarm
	Send Arm or Disarm command to system from last <SetLaserArmDisarm>.
*/
SIDEKICK_SDK_DLL uint32_t SidekickSDK_ExecLaserArmDisarm(DLS_SCI_DEVICE_HANDLE deviceHandle);

/* Function: SetLaserArmDisarm
	Set Arm or Not Armed status for laser. Write to system with <ExecLaserArmDisarm>.

	Parameters:
		arm - true to arm laser system, false to disarm laser system.
*/
SIDEKICK_SDK_DLL uint32_t SidekickSDK_SetLaserArmDisarm(DLS_SCI_DEVICE_HANDLE deviceHandle, bool arm);

/* Function: SetForceFanOff
Sets the override state of the cooling fan in Aries laser.

Parameters:
force_fan_off - true to disable fans
reserved	- unused at this time
*/
SIDEKICK_SDK_DLL uint32_t SidekickSDK_SetForceFanOff(DLS_SCI_DEVICE_HANDLE deviceHandle, uint8_t qcl_slot, bool force_fan_off, uint8_t reserved);

/* Function: GetForceFanOff
Sets the override state of the cooling fan in Aries laser.

Parameters:
force_fan_off - true to disable fans
reserved	- unused at this time
*/
SIDEKICK_SDK_DLL uint32_t SidekickSDK_GetForceFanOff(DLS_SCI_DEVICE_HANDLE deviceHandle, uint8_t qcl_slot, bool *force_fan_off, uint8_t *reserved);

/* Function: ReadWriteForceFanOff
Writes the mode for fan operation of the Aries heads.

Parameters:
bWrite - true to write condition, false to read status
*/
SIDEKICK_SDK_DLL uint32_t SidekickSDK_ReadWriteForceFanOff(DLS_SCI_DEVICE_HANDLE deviceHandle, uint8_t qcl_slot, bool bWrite);

/* Function: SetEnableQclRamp
Sets SDK variable to enable/disable QCL current ramping at laser turn-on.

Parameters:
qcl_ramp_enable - true to enable QCL current ramping, false to turn on at full current
*/
SIDEKICK_SDK_DLL uint32_t SidekickSDK_SetEnableQclRamp(DLS_SCI_DEVICE_HANDLE deviceHandle, bool qcl_ramp_enable);

/* Function: GetEnableQclRamp
Gets SDK variable to enable/disable QCL current ramping at laser turn-on.

Parameters:
qcl_ramp_enable - true to enable QCL current ramping, false to turn on at full current
*/
SIDEKICK_SDK_DLL uint32_t SidekickSDK_GetEnableQclRamp(DLS_SCI_DEVICE_HANDLE deviceHandle, bool *qcl_ramp_enable);

/* Function: ReadWriteEnableQclRamp
Enables or disables QCL Current Ramping at laser turn-on.  Sends the command to the Sidekick.

Parameters:
bWrite - true to write condition, false to read status
*/
SIDEKICK_SDK_DLL uint32_t SidekickSDK_ReadWriteEnableQclRamp(DLS_SCI_DEVICE_HANDLE deviceHandle, bool bWrite);

/* Function: ExecModeRelease
Execute a mode release event on the laser.  This is only supported by some lasers.  If a laser does not support mode release it will do nothing.

Parameters:

*/
SIDEKICK_SDK_DLL uint32_t SidekickSDK_ExecModeRelease(DLS_SCI_DEVICE_HANDLE deviceHandle);

//-------------------------------------------------------------------------------------------------------------------------------------------------------------//
// Group: System Administration Functions

/* Function: ReadAdminSysParams
	Read System Administration parameters from laser system. Use <System Administration Helper Functions> to get the data after
	calling this function.
*/
SIDEKICK_SDK_DLL uint32_t SidekickSDK_ReadAdminSysParams(DLS_SCI_DEVICE_HANDLE deviceHandle);

/* Function: ExecShutdownLaser
	Command that turns off the Sidekick controller.

	Parameters:
		type - Not used.
		reason - Not used.
		delay - Not used.
*/
SIDEKICK_SDK_DLL uint32_t SidekickSDK_ExecShutdownLaser(DLS_SCI_DEVICE_HANDLE deviceHandle, uint8_t type, uint8_t reason, uint32_t delay);

//-------------------------------------------------------------------------------------------------------------------------------------------------------------//
/* Group: System Administration Helper Functions
	These functions are called after a call to <ReadAdminSysParams>.  This call copies these parameters to a local variable that can be 
	retrieved by one of the helper functions.	
*/

/* Function: AdminGetSystemType
	Get <System Type>.

	Parameters:
		p_SystemType - (out) <System Type>.
*/
SIDEKICK_SDK_DLL uint32_t SidekickSDK_AdminGetSystemType(DLS_SCI_DEVICE_HANDLE deviceHandle, uint8_t *p_SystemType);

/* Function: AdminGetMfgDateStr
	Gets the Sidekick Manufacturing Date String.

	Parameters:
		destString - (out) Destination string (char array pointer), should be at least <SIDEKICK_SDK_STRLEN_MAX_MFG_DATE> characters.
		strLen - (out) Length.
*/

SIDEKICK_SDK_DLL uint32_t SidekickSDK_AdminGetMfgDateStr(DLS_SCI_DEVICE_HANDLE deviceHandle, char * destString, uint8_t *strLen);

/* Function: AdminGetModelNumStr
	Gets the Sidekick Model Number String.

	Parameters:
		destString - (out) Destination string (char array pointer), should be at least <SIDEKICK_SDK_STRLEN_MAX_MODEL_NUM> characters.
		strLen - (out) Length.
*/
SIDEKICK_SDK_DLL uint32_t SidekickSDK_AdminGetModelNumStr(DLS_SCI_DEVICE_HANDLE deviceHandle, char * destString, uint8_t *strLen);

/* Function: AdminGetSerialNumStr
	Gets the Sidekick Serial Number String

	Parameters:
		destString - (out) Destination string (char array pointer), should be at least <SIDEKICK_SDK_STRLEN_MAX_SERIAL_NUM> characters.
		strLen - (out) Length.
*/
SIDEKICK_SDK_DLL uint32_t SidekickSDK_AdminGetSerialNumStr(DLS_SCI_DEVICE_HANDLE deviceHandle, char * destString, uint8_t *strLen);

/* Function: AdminGetNumQcls
	Gets number of QCL slots present. Sidekick only supports one laser head, so it should only return 0 for no head connected, or 1.

	Parameters:
		p_NumQcls - (out) Number of laser heads present.
*/
SIDEKICK_SDK_DLL uint32_t SidekickSDK_AdminGetNumQcls(DLS_SCI_DEVICE_HANDLE deviceHandle, uint8_t *p_NumQcls);

/* Function: AdminGetFwVersion
	Get firmware version of the controller.

	Parameters:
		major - (out) major version number.
		minor - (out) minor version number.
		patch - (out) patch version number.
*/
SIDEKICK_SDK_DLL uint32_t SidekickSDK_AdminGetFwVersion(DLS_SCI_DEVICE_HANDLE deviceHandle, uint8_t *major, uint8_t *minor, uint8_t *patch);

/* Function: AdminGetEthernetFwVersion
Get firmware version of the Ethernet Driver.

Parameters:
major - (out) major version number.
minor - (out) minor version number.
patch - (out) patch version number.
*/
SIDEKICK_SDK_DLL uint32_t SidekickSDK_AdminGetEthernetFwVersion(DLS_SCI_DEVICE_HANDLE deviceHandle, uint8_t *major, uint8_t *minor, uint8_t *patch);


/* Function: AdminGetHoursOfOperation
	Get the number of hours of operation for the currently connected laser head.  This is laser on (emission) time in hours.

	Parameters:
		p_HoursOfOperation - (out) Hours.
*/
SIDEKICK_SDK_DLL uint32_t SidekickSDK_AdminGetHoursOfOperation(DLS_SCI_DEVICE_HANDLE deviceHandle, float *p_HoursOfOperation);

/* Function: AdminGetControllerPcbVer
	Get the controller PCB version.

	Parameters:
		p_ControllerPcbVer - (out) Controller PCB version.
*/

SIDEKICK_SDK_DLL uint32_t SidekickSDK_AdminGetControllerPcbVer(DLS_SCI_DEVICE_HANDLE deviceHandle, uint8_t *p_ControllerPcbVer);

/* Function: AdminGetMotionVersions
	Get the Motion (tuning and scanning) versions.

	Parameters:
		modType - (out) Not used at this time.
		hwType - (out) Not used at this time.
		fwVer - (out) Not used at this time.
*/
SIDEKICK_SDK_DLL uint32_t SidekickSDK_AdminGetMotionVersions(DLS_SCI_DEVICE_HANDLE deviceHandle, uint16_t *modType, uint16_t *hwType, uint16_t *fwVer);

//-------------------------------------------------------------------------------------------------------------------------------------------------------------//
/* Group: QCL Administration Functions
	These functions read data about the attached laser head from the system. Use the helper functions after calling the *Read* function to get the information.
*/

/* Function: ReadAdminQclParams
	Read Admin QCL parameters from laser system. These include things like supported wavelength range, maximum QCL current, etc.
	Use the AdminQcl helper functions to retrieve the actual data.

	Parameters:
		qcl_slot - Set to 0.
*/
SIDEKICK_SDK_DLL uint32_t SidekickSDK_ReadAdminQclParams(DLS_SCI_DEVICE_HANDLE deviceHandle, uint8_t qcl_slot);

/* Function: ReadAdminFirmwareVersion
	Read firmware version from the controller. Get the values with <GetAdminFirmwareVersions> and <AdminGetEthernetFwVersion>. 
*/
SIDEKICK_SDK_DLL uint32_t SidekickSDK_ReadAdminFirmwareVersion(DLS_SCI_DEVICE_HANDLE deviceHandle);

//-------------------------------------------------------------------------------------------------------------------------------------------------------------//
/* Group: QCL Administration Helper Functions
	Call functions in the <QCL Administration Functions> group before calling these.
*/

/* Function: GetAdminFirmwareVersions
	Get firmware version information from last <ReadAdminFirmwareVersion> call. This is the firmware and PCB version of the Sidekick controller.
	The command that implements this function was intended to never change for different lasers, which is why this command exists in addition to 
	the <AdminGetFwVersion> function.

	Parameters:
		major - (out) Major version number.
		minor - (out) Minor version number.
		patch - (out) Patch version number.
		pcbVer - (out) Hardware (PCB) Version number.
*/
SIDEKICK_SDK_DLL uint32_t SidekickSDK_GetAdminFirmwareVersions(DLS_SCI_DEVICE_HANDLE deviceHandle, uint8_t *major, uint8_t *minor, uint8_t *patch, uint8_t *pcbVer);

/* Function: AdminQclGetSlot
	QCL Slot from the previous call to <ReadAdminQclParams>.

	Parameters:
		p_Slot - (out) For Sidekick this is not used.
*/
SIDEKICK_SDK_DLL uint32_t SidekickSDK_AdminQclGetSlot(DLS_SCI_DEVICE_HANDLE deviceHandle, uint8_t *p_Slot);

/* Function: AdminQclIsAvailable
	Is a QCL installed and detected by the system.

	Parameters:
		p_QclIsAvailable - (out) true if a QCL has been detected by the Sidekick system.
*/
SIDEKICK_SDK_DLL uint32_t SidekickSDK_AdminQclIsAvailable(DLS_SCI_DEVICE_HANDLE deviceHandle, bool *p_QclIsAvailable);

/* Function: AdminQclGetLaserType
	This is the motion drivetrain hardware for the laser head.

	Parameters:
		p_LaserType - (out) See <Laser Type>.
*/
SIDEKICK_SDK_DLL uint32_t SidekickSDK_AdminQclGetLaserType(DLS_SCI_DEVICE_HANDLE deviceHandle, uint8_t *p_LaserType);

/* Function: AdminQclGetSerialNumStr
	Gets the Laser Head Serial Number.

	Parameters:
		destString - (out) Destination string (char array pointer) that should be at least <SIDEKICK_SDK_STRLEN_MAX_SERIAL_NUM> characters.
		strLen - (out) Length.
*/
SIDEKICK_SDK_DLL uint32_t SidekickSDK_AdminQclGetSerialNumStr(DLS_SCI_DEVICE_HANDLE deviceHandle, char * destString, uint8_t *strLen);

/* Function: AdminQclGetModelNumStr
	Gets the Laser Head Model Number String

	Parameters:
		destString - (out) Destination string (char array pointer) that should be at least <SIDEKICK_SDK_STRLEN_MAX_MODEL_NUM> characters.
		strLen - (out) Length.
*/
SIDEKICK_SDK_DLL uint32_t SidekickSDK_AdminQclGetModelNumStr(DLS_SCI_DEVICE_HANDLE deviceHandle, char * destString, uint8_t *strLen);

/* Function: AdminQclGetMfgDateStr
	Gets the Laser Head Manufacturing Date String.

	Parameters:
	destString - (out) Destination string (char array pointer) that should be at least <SIDEKICK_SDK_STRLEN_MAX_MFG_DATE> characters.
	strLen - (out) Length.
*/
SIDEKICK_SDK_DLL uint32_t SidekickSDK_AdminQclGetMfgDateStr(DLS_SCI_DEVICE_HANDLE deviceHandle, char *destString, uint8_t *strLen);

/* Function: AdminQclGetCurrentLimits
	Gets the current limits.

	Parameters:
		pulseCurMax - (out) Maximum current for pulsed mode in mA.
		cwCurMax - (out) Current limit for CW mode in mA.
*/
SIDEKICK_SDK_DLL uint32_t SidekickSDK_AdminQclGetCurrentLimits(DLS_SCI_DEVICE_HANDLE deviceHandle, uint16_t *pulseCurMax, uint16_t *cwCurMax);

/* Function: AdminQclGetMhfCurrentLimits
	Gets the current limits for Mode Hop Free head.

	Parameters:
		mhfCurMin - (out) Minimum current in mA.
		mhfCurMax - (out) Maximum current in mA.
*/
SIDEKICK_SDK_DLL uint32_t SidekickSDK_AdminQclGetMhfCurrentLimits(DLS_SCI_DEVICE_HANDLE deviceHandle, uint16_t *mhfCurMin, uint16_t *mhfCurMax);

/* Function: AdminQclGetTempLimits
	This is the nominal, min, and max temperatures for the QCL target temperature.

	Parameters:
		minTemp - (out) Minimum QCL temperature.
		nomTemp - (out) Nominal QCL temperature.
		maxTemp - (out) Maximum QCL temperature.
*/
SIDEKICK_SDK_DLL uint32_t SidekickSDK_AdminQclGetTempLimits(DLS_SCI_DEVICE_HANDLE deviceHandle, float *minTemp, float *nomTemp, float *maxTemp);

/* Function: AdminQclGetPulseLimits
	Pulse limits for pulsed mode.

	Parameters:
		min_pls_rate_hz - (out) Minimum laser pulse rate in Hz.
		max_pls_rate_hz - (out) Maximum laser pulse rate in Hz.
		min_pls_width_ns - (out) Minumum pulse width in ns.
		max_pls_width_ns - (out) Maximum pulse width in ns.
*/
SIDEKICK_SDK_DLL uint32_t SidekickSDK_AdminQclGetPulseLimits(DLS_SCI_DEVICE_HANDLE deviceHandle, uint32_t *min_pls_rate_hz, uint32_t *max_pls_rate_hz,
							 uint32_t *min_pls_width_ns, uint32_t *max_pls_width_ns );
SIDEKICK_SDK_DLL uint32_t SidekickSDK_AdminQclGetPulseParams(DLS_SCI_DEVICE_HANDLE deviceHandle, uint32_t *min_pls_rate_hz, uint32_t *max_pls_rate_hz,
							 uint32_t *min_pls_width_ns, uint32_t *max_pls_width_ns, float *max_duty_cycle);

/* Function: AdminQclGetScanRateLimits
	Get the scan rate limits.

	Parameters:
		min_scan_rate - (out) Minimum scan rate.
		max_scan_rate - (out) Maximum scan rate.
		scan_rate_units - <Wavelength Units> of scan rate per second.  This is always reported in <SIDEKICK_SDK_UNITS_MICRONS> per second.
*/
SIDEKICK_SDK_DLL uint32_t SidekickSDK_AdminQclGetScanRateLimits(DLS_SCI_DEVICE_HANDLE deviceHandle, float *min_scan_rate, float *max_scan_rate, uint8_t *scan_rate_units);

/* Function: AdminQclGetDwellTimeLimits
	Dwell time is used during Step & Measure or Multi-Spectral scans as the duration to stay on each wavelength during a step.  

	Parameters:
		min_dwell_time_ms - Minimum dwell time, milliseconds.
		max_dwell_time_ms - Maxmimum dwell time, milliseconds.
*/
SIDEKICK_SDK_DLL uint32_t SidekickSDK_AdminQclGetDwellTimeLimits(DLS_SCI_DEVICE_HANDLE deviceHandle, uint32_t *min_dwell_time_ms, uint32_t *max_dwell_time_ms);

/* Function: AdminQclGetWavelengthLimits
	Get wavelength limits for tunable head.

	Parameters:
		min_ww - (out) Minimum wavelength.
		max_ww - (out) Maxmimum wavelength.
		ww_units - (out) <Wavelength Units>.
*/
SIDEKICK_SDK_DLL uint32_t SidekickSDK_AdminQclGetWavelengthLimits(DLS_SCI_DEVICE_HANDLE deviceHandle, float *min_ww, float *max_ww, uint8_t *ww_units);
/* Function: AdminQclGetMhfWavelengthLimits
	Get wavelength limits for a Mode Hop Free head.

	Parameters:
		min_ww - (out) Minimum wavelength.
		max_ww - (out) Maxmimum wavelength.
		ww_units - (out) <Wavelength Units>.
*/
SIDEKICK_SDK_DLL uint32_t SidekickSDK_AdminQclGetMhfWavelengthLimits(DLS_SCI_DEVICE_HANDLE deviceHandle, float *min_ww, float *max_ww, uint8_t *ww_units);
/* Function: AdminQclGetCwAllowed
	Is CW mode supported by this laser head?

	Parameters:
		p_CwAllowed - (out) true if CW mode is allowed for use on this laser.
*/
SIDEKICK_SDK_DLL uint32_t SidekickSDK_AdminQclGetCwAllowed(DLS_SCI_DEVICE_HANDLE deviceHandle, bool *p_CwAllowed);

/* Function: AdminQclGetFiltersInstalled
Are CW filters installed on this laser head?

Parameters:
p_FiltersInstalled - (out) true if CW filters are installed in the laser head.
*/
SIDEKICK_SDK_DLL uint32_t SidekickSDK_AdminQclGetFiltersInstalled(DLS_SCI_DEVICE_HANDLE deviceHandle, bool *p_FiltersInstalled);

/* Function: AdminQclGetLaserTimeMinutes
	This is the laser on time in minutes for the currently attached laser head.

	Parameters:
		p_LaserTimeMinutes - (out) The accumulated time the laser has been on (emitting).
*/
SIDEKICK_SDK_DLL uint32_t SidekickSDK_AdminQclGetLaserTimeMinutes(DLS_SCI_DEVICE_HANDLE deviceHandle, uint32_t *p_LaserTimeMinutes);

/* Function: AdminQclGetHeadType
	This is the head type of the attached laser as defined by <Head Type1>.

	Parameters:
		headType - (out) See <Head Type1>.
*/
SIDEKICK_SDK_DLL uint32_t SidekickSDK_AdminQclGetHeadType(DLS_SCI_DEVICE_HANDLE deviceHandle, uint8_t *headType);

/* Function: AdminQclGetHeadElectronics
	Gets the head electronics type.

	Parameters:
		headElectronics - (out) See <Head Electronics>.
*/
SIDEKICK_SDK_DLL uint32_t SidekickSDK_AdminQclGetHeadElectronics(DLS_SCI_DEVICE_HANDLE deviceHandle, uint8_t *headElectronics);

/* Function: AdminQclGetFixedWavelengthLaser
	Gets whether or not laser is fixed wavelength (not tunable).

	Parameters:
		fixedWavelengthLaser - (out) true if laser is not tunable, false for tunable laser head.
*/
SIDEKICK_SDK_DLL uint32_t SidekickSDK_AdminQclGetFixedWavelengthLaser(DLS_SCI_DEVICE_HANDLE deviceHandle, bool *fixedWavelengthLaser );

//-------------------------------------------------------------------------------------------------------------------------------------------------------------//
// Group: Display Functions

/* Function: ReadWriteSysDisplayUnits
	This is not currently used by Sidekick.

	Parameters:
		fWrite - true for write, false for read.
*/
SIDEKICK_SDK_DLL uint32_t SidekickSDK_ReadWriteSysDisplayUnits(DLS_SCI_DEVICE_HANDLE deviceHandle, bool fWrite);
/* Function: GetDisplayUnits
	This is not currently used by Sidekick.

	Parameters:
		p_wwUnits - (out) <Wavelength Units>.
		p_pulseRateUnits - (out) This is not currently used by Sidekick.
*/
SIDEKICK_SDK_DLL uint32_t SidekickSDK_GetDisplayUnits(DLS_SCI_DEVICE_HANDLE deviceHandle, uint8_t *p_wwUnits, uint8_t *p_pulseRateUnits);
/* Function: SetDisplayUnits
	This is not currently used by Sidekick.

	Parameters:
		p_wwUnits - <Wavelength Units>.
		p_pulseRateUnits - This is not currently used by Sidekick.
*/
SIDEKICK_SDK_DLL uint32_t SidekickSDK_SetDisplayUnits(DLS_SCI_DEVICE_HANDLE deviceHandle, uint8_t wwUnits, uint8_t pulseRateUnits);
/* Function: ExecuteSaveStateOperation
	This is not currently used by Sidekick.
*/

//-------------------------------------------------------------------------------------------------------------------------------------------------------------//
/* Group: State Management Functions
	Functions for Saving and Recalling states. This is not currently used by Sidekick.
*/

SIDEKICK_SDK_DLL uint32_t SidekickSDK_ExecuteSaveStateOperation(DLS_SCI_DEVICE_HANDLE deviceHandle);
/* Function: SaveState
	This is not currently used by Sidekick.

	Parameters:
		p_newName - Saved state name.
		newId - Saved state ID.
*/
SIDEKICK_SDK_DLL uint32_t SidekickSDK_SaveState(DLS_SCI_DEVICE_HANDLE deviceHandle, const char *p_newName, int *newId);
/* Function: OverwriteState
	This is not currently used by Sidekick.

	Parameters:
		p_name - Saved state name.
		id - Saved state ID.
*/
SIDEKICK_SDK_DLL uint32_t SidekickSDK_OverwriteState(DLS_SCI_DEVICE_HANDLE deviceHandle, const char *p_name, int id);

/* Function: ExecuteRestoreStateOperation
	This is not currently used by Sidekick.
*/
SIDEKICK_SDK_DLL uint32_t SidekickSDK_ExecuteRestoreStateOperation(DLS_SCI_DEVICE_HANDLE deviceHandle);

/* Function: RecallState
	This is not currently used by Sidekick.

	Parameters:
		id - Saved state ID.
*/
SIDEKICK_SDK_DLL uint32_t SidekickSDK_RecallState(DLS_SCI_DEVICE_HANDLE deviceHandle, int id);
/* Function: ExecuteEraseStateOperation
	This is not currently used by Sidekick.
*/
SIDEKICK_SDK_DLL uint32_t SidekickSDK_ExecuteEraseStateOperation(DLS_SCI_DEVICE_HANDLE deviceHandle);

/* Function: EraseState
	This is not currently used by Sidekick.

	Parameters:
		id - Saved state ID.
*/
SIDEKICK_SDK_DLL uint32_t SidekickSDK_EraseState(DLS_SCI_DEVICE_HANDLE deviceHandle, int id);

/* Function: ReadNumSavedStates
	This is not currently used by Sidekick.
*/
SIDEKICK_SDK_DLL uint32_t SidekickSDK_ReadNumSavedStates(DLS_SCI_DEVICE_HANDLE deviceHandle);	

/* Function: ReadSavedStateName
	This is not currently used by Sidekick.
*/
SIDEKICK_SDK_DLL uint32_t SidekickSDK_ReadSavedStateName(DLS_SCI_DEVICE_HANDLE deviceHandle);	

//-------------------------------------------------------------------------------------------------------------------------------------------------------------//
/* Group: Network Setup Functions
	Functions for setting up network interface.
*/

/* Function: ReadWriteIPAddressProvision
	Read or Write network set up of controller. Configuration set using <SetProvisionIPAddr>, and can be read using <GetProvisionIPAddr>. Changes are read at
	startup, so the controller must be power cycled to use new settings.

	Parameters:
		fWrite - true to write configuration to controller, false to read only.
*/
SIDEKICK_SDK_DLL uint32_t SidekickSDK_ReadWriteIPAddressProvision(DLS_SCI_DEVICE_HANDLE deviceHandle, bool fWrite);

/* Function: GetProvisionIPAddr
	Get network configuration read during the last call of <ReadWriteIPAddressProvision>.

	Parameters:
		ipAddr - (out) Numeric (host byte order) IP address for controller. Convert to string using socket functions.
		subnetMask - (out) Numeric subnet mask.
		defGateway - (out) Default gateway.
		port - (out) Port to use with IP Address.
		useDhcp - (out) true when using DHCP.
*/
SIDEKICK_SDK_DLL uint32_t SidekickSDK_GetProvisionIPAddr(DLS_SCI_DEVICE_HANDLE deviceHandle,
										uint32_t *ipAddr, uint32_t *subnetMask, uint32_t *defGateway, uint16_t *port, bool *useDhcp);
/* Function: SetProvisionIPAddr
	Set network configuration. Will be written during next call to <ReadWriteIPAdressProvision> with fWrite = true.

	Parameters:
		ipAddr - Numeric (host byte order) IP address. Convert from string using socket functions.
		subnetMask - Numeric subnet mask.
		defGateway - Default gateway.
		port - Port to use with IP Address.
		useDhcp - true when using DHCP.
*/
SIDEKICK_SDK_DLL uint32_t SidekickSDK_SetProvisionIPAddr(DLS_SCI_DEVICE_HANDLE deviceHandle, uint32_t ipAddr, uint32_t subnetMask, uint32_t defGateway, uint16_t port, bool useDhcp);

/* Function: ReadConfiguredProvisionIPAddr
Get configured network configuration. If configured was not successful, zeros will be returned for all values.

Parameters:
ipAddr - (out) Numeric (host byte order) IP address for controller. Convert to string using socket functions.
subnetMask - (out) Numeric subnet mask.
defGateway - (out) Default gateway.
port - (out) Port to use with IP Address.
useDhcp - (out) true when using DHCP.
*/
SIDEKICK_SDK_DLL uint32_t SidekickSDK_ReadConfiguredProvisionIPAddr(DLS_SCI_DEVICE_HANDLE deviceHandle,
	uint32_t *ipAddr, uint32_t *subnetMask, uint32_t *defGateway, uint16_t *port, bool *useDhcp);

/* Function: ReadWriteDiscoveryIPAddressProvision
	Read or Write discovery protocol configuration. Set using <SetDiscoveryProvisionIPAddr>, get using <GetDiscoveryProvisionIPAddr>. Power cycle controller after changing
	setup or call <EthernetReset> to use new configuration.

	Parameters:
		fWrite - true to write new parameters, false to only read parameters.
*/
SIDEKICK_SDK_DLL uint32_t SidekickSDK_ReadWriteDiscoveryIPAddressProvision(DLS_SCI_DEVICE_HANDLE deviceHandle, bool fWrite);
/* Function: GetDiscoveryProvisionIPAddr
	Get discovery protocol configuration read from last <ReadWriteDiscoveryIPAddressProvision> call.

	Parameters:
		ipAddr - (out) Numeric (host byte order) IP address for controller. Convert to string using socket functions.
		port - (out) Port to use with IP Address.
*/
SIDEKICK_SDK_DLL uint32_t SidekickSDK_GetDiscoveryProvisionIPAddr(DLS_SCI_DEVICE_HANDLE deviceHandle, uint32_t *ipAddr, uint16_t *port);
/* Function: SetDiscoveryProvisionIPAddr
	Set discovery protocol setup. Will be written with next <ReadWriteDiscoveryIPAddressProvision> call.

	Parameters:
		ipAddr - IP Address (host byte order) for the discovery protocol. This is usually a multicast group address, default 239.255.101.224.
		port - Port to use for discovery protocol. Default is 8383.
*/
SIDEKICK_SDK_DLL uint32_t SidekickSDK_SetDiscoveryProvisionIPAddr(DLS_SCI_DEVICE_HANDLE deviceHandle, uint32_t ipAddr, uint16_t port);

/* Function: ReadWriteEthernetTimeout
	Read or Write Ethernet enable and timeout. See <SetEthernetTimeout> and <GetEthernetTimeout>. 
	
	Parameters:
		fWrite - true to write new parameters, false to only read parameters.
*/
SIDEKICK_SDK_DLL uint32_t SidekickSDK_ReadWriteEthernetTimeout(DLS_SCI_DEVICE_HANDLE deviceHandle, bool fWrite);

/* Function: SetEthernetTimeout
	Set Ethernet timeout in tens of seconds. For values strictly between 0-255, Sidekick will attempt to establish
	a connection, then disable Ethernet after timeout passes. 0 and 255 have special meaning. Call <EthernetReset> for new settings
	to take effect.

	Parameters
		timeoutTens - Timeout multiple of ten seconds (1 means 10 seconds, 2 means 20 seconds). 0 disables Ethernet. 255 means no timeout.
*/
SIDEKICK_SDK_DLL uint32_t SidekickSDK_SetEthernetTimeout(DLS_SCI_DEVICE_HANDLE deviceHandle, uint8_t timeoutTens);

/* Function: GetEthernetTimeout
	Get Ethernet timeout in tens of seconds. For values strictly between 0-255, Sidekick will attempt to establish
	a connection, then disable Ethernet after timeout passes. 0 and 255 have special meaning.

	Parameters
		timeoutTens - (out) Timeout multiple of ten seconds (1 means 10 seconds, 2 means 20 seconds). 0 disables Ethernet. 255 means no timeout.	
*/
SIDEKICK_SDK_DLL uint32_t SidekickSDK_GetEthernetTimeout(DLS_SCI_DEVICE_HANDLE deviceHandle, uint8_t *timeoutTens);


/* Function: EthernetReset
	Reset Ethernet to use new Ethernet settings.
*/
SIDEKICK_SDK_DLL uint32_t SidekickSDK_ResetEthernet(DLS_SCI_DEVICE_HANDLE deviceHandle);

//-------------------------------------------------------------------------------------------------------------------------------------------------------------//
/* Group: Miscellaneous System Configuration Functions
*/

/* Function: ReadWriteAudioNotificationPreference
	Read or Write audio notification preferences.  This is currently not implemented in Sidekick.

	Parameters:
		fWrite - true for writing, false for reading.
*/
SIDEKICK_SDK_DLL uint32_t SidekickSDK_ReadWriteAudioNotificationPreference(DLS_SCI_DEVICE_HANDLE deviceHandle, bool fWrite);

/* Function: GetAudioNotificationPreferences
	Get audio notification preferences read during the last <ReadWriteAudioNotificationPreference> call.  
	This is currently not implemented in Sidekick.

	Parameters:
		audioOn - (out) Turn beeper on when laser is firing.
		flashLed - (out) Flash emission LED when laser is firing.
*/
SIDEKICK_SDK_DLL uint32_t SidekickSDK_GetAudioNotificationPreferences(DLS_SCI_DEVICE_HANDLE deviceHandle, bool *audioOn, bool *flashLed);

/* Function: SetAudioNotificationPreferences
	Set audio notification preferences. Write to system using <ReadWriteAudioNotificationPreference>.
	This is currently not implemented in Sidekick.

	Parameters:
		audioOn - Turn beeper on when laser is firing.
		flashLed - Flash emission LED when laser is firing.
*/
SIDEKICK_SDK_DLL uint32_t SidekickSDK_SetAudioNotificationPreferences(DLS_SCI_DEVICE_HANDLE deviceHandle, bool audioOn, bool flashLed);

/* Function: ReadWriteSysGlobalPreferences
	Read or Write global system preferences. Use with <GetSysGlobalPreferences> and <SetSysGlobalPreferences>.
	This is currently not implemented in Sidekick.

	Parameters:
		fWrite - true for writing, false for reading.
*/
SIDEKICK_SDK_DLL uint32_t SidekickSDK_ReadWriteSysGlobalPreferences(DLS_SCI_DEVICE_HANDLE deviceHandle, bool fWrite);

/* Function: GetSysGlobalPreferences
	Get preferences read during last <ReadWriteSysGlobalPreferences> call.
	This is currently not implemented in Sidekick.

	Parameters:
		alwaysUseCrossover - (out) Not applicable to Sidekick.
*/
SIDEKICK_SDK_DLL uint32_t SidekickSDK_GetSysGlobalPreferences(DLS_SCI_DEVICE_HANDLE deviceHandle, bool *alwaysUseCrossover);

/* Function: SetSysGlobalPreferences
	Set preferences to write during next <ReadWriteSysGlobalPreferences> call.
	This is currently not implemented in Sidekick.

	Parameters:
		alwaysUseCrossover - Not applicable to Sidekick.
*/
SIDEKICK_SDK_DLL uint32_t SidekickSDK_SetSysGlobalPreferences(DLS_SCI_DEVICE_HANDLE deviceHandle, bool alwaysUseCrossover);



#ifdef __cplusplus
}
#endif

#endif //_SIDEKICKSDK_H