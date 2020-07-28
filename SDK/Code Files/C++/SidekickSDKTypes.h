#ifndef DLS_SCI_DEVICE_TYPES_H
#define	DLS_SCI_DEVICE_TYPES_H


#include <stdint.h>
#include "SidekickSDKConstants.h"
#include "DlsSciCmdPub.h"

#ifdef __cplusplus
extern "C"
{
#else
#ifdef _WIN32
#include <stdbool.h>
#else
#include "DlsSciBool.h"
#endif
#endif

// Title: SidekickSDKTypes.h

/* Topic: Documentation conventions 
	All types defined in this file contain a *DLS_SCI_* prefix that may be omitted from the documentation.
*/

/* typedef: DLS_SCI_DEVICE_HANDLE
	The device handle type for use with all functions that communicate with a connected device.
*/
typedef unsigned int DLS_SCI_DEVICE_HANDLE;

/* Constant: DLS_SCI_DEVICE_NULL_HANDLE
	The null value for <DLS_SCI_DEVICE_HANDLE>. A connected device should never have this value.
*/
#define DLS_SCI_DEVICE_NULL_HANDLE			(0x0)

/* enum: DLS_SCI_DEVICE_TRANSPORT
	Used internally to SDK.
*/
typedef enum DLS_SCI_DEVICE_TRANSPORT
{
	DLS_NUL_TRANSPORT,
	DLS_USB_TRANSPORT,
	DLS_UDP_TRANSPORT,
	DLS_TCP_TRANSPORT
} DLS_SCI_DEVICE_TRANSPORT;

/* struct: SYSTEM_INFO
	Contains information about the controller and connected laser head that is filled in during a device search.

	Fields:
		SystemSerialNumber - Controller Serial Number.
		SystemModelNumber - Controller Model Number.
		SystemType - <System Type>.
		StatusWord - Status word from <ReadStatusMask>.
		ErrorWord - Error word from <ReadStatusMask>.
		WarningWord - Warning word from <ReadStatusMask>.
		HeadConnected - true if laser head is connected.
		HeadSerialNumber - Head Serial Number.
		HeadModelNumber - Head Model Number.
		HeadType - <Head Type1>
		min_ww - Minimum wavelength.
		max_ww - Maximum wavelength.
		ww_units - <Wavelength Units>.
		mhf_min_ww - Mode Hop Free minimum wavelength.
		mhf_max_ww - Mode Hop Free maximum wavelength.
		mhf_ww_units - <Wavelength Units>.
*/
typedef struct _DLS_SCI_SYSTEM_INFO
{
	char	    SystemSerialNumber[SIDEKICK_SDK_STRLEN_MAX_SERIAL_NUM];
	char        SystemModelNumber[SIDEKICK_SDK_STRLEN_MAX_MODEL_NUM];
	uint8_t     SystemType;
	uint32_t	StatusWord;
	uint16_t	ErrorWord;
	uint16_t	WarningWord;

	bool		HeadConnected;
	char    	HeadSerialNumber[SIDEKICK_SDK_STRLEN_MAX_SERIAL_NUM];
	char        HeadModelNumber[SIDEKICK_SDK_STRLEN_MAX_MODEL_NUM];
	uint8_t     HeadType;				// tuneable or Aries
	float		min_ww;
	float		max_ww;
	uint8_t		ww_units;
	float		mhf_min_ww;
	float		mhf_max_ww;
	uint8_t		mhf_ww_units;
	uint8_t     DriveTrainType;			// CWP, MHF, Voice Coil
	uint8_t     HeadElectronicsType;	// Gen 1, HFQD, MHF, HFQD2
	uint8_t     DeviceType;				// QC or IC
	bool		FixedWavelength;			// tuneable or not (redundant)
} DLS_SCI_SYSTEM_INFO;

/* struct: IP_TRANSPORT_INFO
	Contains information about network transport.

	Fields:
		ipAddressString - IP Address in dotted decimal format.
		ipAddress - IP Address in numeric format.
		tcpCommandPort - Command port to use with connection functions.
		udpDiscoveryPort - not used.
*/
typedef struct _DLS_SCI_IP_TRANSPORT_INFO
{
	char		ipAddressString[48];
	uint32_t	ipAddress;
	uint16_t	tcpCommandPort;
} DLS_SCI_IP_TRANSPORT_INFO;

/* struct: USB_TRANSPORT_INFO
	Contains information about USB Transport information.

	Fields:
		ID - USB device ID.
		SerialNumber - Serial Number programmed into USB controller.
		Description - Description programmed into USB controller.
*/
typedef struct _DLS_SCI_USB_TRANSPORT_INFO
{
	uint32_t	ID;
	char SerialNumber[16];
	char Description[64];
} DLS_SCI_USB_TRANSPORT_INFO;

/* struct: DEVICE_INFO
	Complete structure containing system information and transport information. This structure is filled in during device search.

	Fields:
		systemInfoValid - true if the systemInfo field contains valid information.
		systemInfo - <SYSTEM_INFO> structure.
		ipTransportValid - true if the ipTransportInfo field contains valid information.
		ipTransportConnected - true if the device is currently connected over the network.
		ipTransportHandle - <DLS_SCI_DEVICE_HANDLE> that device is connected over, if ipTransportConnected == true.
		ipTransportInfo - <IP_TRANSPORT_INFO> structure.
		usbFtdiTransportValid - true if usbFtdiTransportInfo field contains valid information.
		usbFtdiConnected - true if the device is currently connected over USB.
		usbFtdiHandle - if usbFtdiConnected == true, contains <DLS_SCI_DEVICE_HANDLE> that this device is connected over.
		usbFtdiDeviceNumber - Device number on USB. For use with <ConnectToUsbDevice>
		usbFtdiTransportInfo - <USB_TRANSPORT_INFO> structure.
*/
typedef struct _DLS_SCI_DEVICE_INFO
{
	bool						systemInfoValid;
	DLS_SCI_SYSTEM_INFO         systemInfo;

    bool		                ipTransportValid;
	bool						ipTransportConnected;
	DLS_SCI_DEVICE_HANDLE		ipTransportHandle;
	DLS_SCI_IP_TRANSPORT_INFO	ipTransportInfo;

	bool		                usbFtdiTransportValid;
	bool						usbFtdiConnected;
	DLS_SCI_DEVICE_HANDLE		usbFtdiHandle;
	uint32_t	                usbFtdiDeviceNumber;
	DLS_SCI_USB_TRANSPORT_INFO	usbFtdiTransportInfo;
} DLS_SCI_DEVICE_INFO;

#ifdef __cplusplus
}
#endif


#endif // !DLS_SCI_DEVICE_TYPES_H
