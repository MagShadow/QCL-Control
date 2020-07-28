#ifndef SIDEKICK_SDK_CONSTANTS_H
#define SIDEKICK_SDK_CONSTANTS_H
#pragma once

#include "DlsSciCmdPub.h"
// Title: SidekickSDKConstants.h

//----------------------------------------------------------------------------------------------------------------//
/* Constants: Error Codes
	The SDK can return several possible error codes. See SidekickSDKConstants.h for the full list. If the error code
	is not success, the user can use <GetCommErrDesc> to get a text string description of the error.

	SIDEKICK_SDK_RET_SUCCESS - Success error code. Compare return value from function with this value to check for success.
*/
#define SIDEKICK_SDK_RET_SUCCESS					((uint32_t)0)

// Communication and Transport Errors
#define SIDEKICK_SDK_RET_UNSUPPORTED_TRANSPORT		((uint32_t)10)
#define SIDEKICK_SDK_RET_INVALID_DEVICE_NUM			((uint32_t)11)
#define SIDEKICK_SDK_RET_DEVICE_INFO_ERROR			((uint32_t)12)
#define SIDEKICK_SDK_RET_CONNECT_ERROR				((uint32_t)13)
#define SIDEKICK_SDK_RET_COMM_ERROR					((uint32_t)14)
#define SIDEKICK_SDK_RET_CONNECTION_LOST			((uint32_t)15)
#define SIDEKICK_SDK_RET_SEARCH_ERROR				((uint32_t)16)
#define SIDEKICK_SDK_RET_INVALID_DEVICE_HANDLE		((uint32_t)17)

// Initialization Errors
#define SIDEKICK_SDK_RET_INIT_FAILURE				((uint32_t)20)

// Function return Errors
#define SIDEKICK_SDK_RET_LASER_NOT_ARMED			((uint32_t)30)
#define SIDEKICK_SDK_RET_INVALID_CHAN_NUM			((uint32_t)31)

#define SIDEKICK_SDK_RET_DATA_TOO_LARGE_ERROR		((uint32_t)32)
#define SIDEKICK_SDK_RET_DATA_NULL_PTR_ERROR		((uint32_t)33)

//----------------------------------------------------------------------------------------------------------------//
/* Constants: System Status Masks
Use with System Status Word in <GetInfoStatusMask>. See SidekickSDKConstants.h for full list.
*/
#define SIDEKICK_SDK_IDX_STATUS_MASK_INTERLOCK_SET			((uint32_t)0x00000001)
#define SIDEKICK_SDK_IDX_STATUS_MASK_KEYSWITCH_SET			((uint32_t)0x00000002)
#define SIDEKICK_SDK_IDX_STATUS_MASK_AT_TEMPERATURE			((uint32_t)0x00000004)
#define SIDEKICK_SDK_IDX_STATUS_MASK_POST_IN_PROGRESS		((uint32_t)0x00000008)
#define SIDEKICK_SDK_IDX_STATUS_MASK_POST_SUCCESS			((uint32_t)0x00000010)
#define SIDEKICK_SDK_IDX_STATUS_MASK_SCAN_IN_PROGRESS		((uint32_t)0x00000020)
#define SIDEKICK_SDK_IDX_STATUS_MASK_MOTION_IN_PROGRESS		((uint32_t)0x00000040)
#define SIDEKICK_SDK_IDX_STATUS_MASK_TUNED_ON_TARGET		((uint32_t)0x00000080) 
#define SIDEKICK_SDK_IDX_STATUS_MASK_SYSTEM_ERROR			((uint32_t)0x00000100)
#define SIDEKICK_SDK_IDX_STATUS_MASK_SYSTEM_WARNING			((uint32_t)0x00000200)
#define SIDEKICK_SDK_IDX_STATUS_MASK_LASER_ARMED			((uint32_t)0x00000400)
#define SIDEKICK_SDK_IDX_STATUS_MASK_SCAN_SETUP				((uint32_t)0x00000800)
#define SIDEKICK_SDK_IDX_STATUS_MASK_LASER_FIRING			((uint32_t)0x00001000)
#define SIDEKICK_SDK_IDX_STATUS_MASK_BTEMP1_FAULT			((uint32_t)0x00002000)
#define SIDEKICK_SDK_IDX_STATUS_MASK_BTEMP2_FAULT			((uint32_t)0x00004000)
#define SIDEKICK_SDK_IDX_STATUS_MASK_PCB_TEMP_FAULT			((uint32_t)0x00008000)
#define SIDEKICK_SDK_IDX_STATUS_MASK_POINTER_INSTALLED		((uint32_t)0x00010000)
#define SIDEKICK_SDK_IDX_STATUS_MASK_POINTER_ENABLED		((uint32_t)0x00020000)
#define SIDEKICK_SDK_IDX_STATUS_MASK_MOTION_FAULTED			((uint32_t)0x00040000)
#define SIDEKICK_SDK_IDX_STATUS_MASK_HEAD_INSTALLED			((uint32_t)0x00080000)
#define SIDEKICK_SDK_IDX_STATUS_MASK_VSRC_GOOD				((uint32_t)0x00100000)
#define SIDEKICK_SDK_IDX_STATUS_MASK_HOMING_COMPLETED		((uint32_t)0x00200000)

//----------------------------------------------------------------------------------------------------------------//
/* Constants: IO Status Masks
	Use with <GetInfoIoStatus>. See SidekickSDKConstants.h for full list.
*/
#define SIDEKICK_SDK_IDX_IO_SCAN_DIR        ((uint32_t)0x00000001) //System Output
#define SIDEKICK_SDK_IDX_IO_WL_TRIG         ((uint32_t)0x00000002) //System Output
#define SIDEKICK_SDK_IDX_IO_GATE            ((uint32_t)0x00000004) //System Input
#define SIDEKICK_SDK_IDX_IO_TRIG_OUT        ((uint32_t)0x00000008) //System Output
#define SIDEKICK_SDK_IDX_IO_TUNED           ((uint32_t)0x00000010) //System Output
#define SIDEKICK_SDK_IDX_IO_PROCESS_TRIG    ((uint32_t)0x00000020) //System Input
#define SIDEKICK_SDK_IDX_IO_INTERLOCK       ((uint32_t)0x00000040) //System Input
#define SIDEKICK_SDK_IDX_IO_TRIG_IN         ((uint32_t)0x00000080) //System Input
#define SIDEKICK_SDK_IDX_IO_KEY_SWITCH      ((uint32_t)0x00000100) //System Input
#define SIDEKICK_SDK_IDX_IO_PUSH_BUTTON     ((uint32_t)0x00000200) //System Input
#define SIDEKICK_SDK_IDX_IO_LED3		    ((uint32_t)0x00000800) //System Output, Open Collector
#define SIDEKICK_SDK_IDX_IO_ON_OFF_CTRL     ((uint32_t)0x00001000) //System Input

//----------------------------------------------------------------------------------------------------------------//
/* Constants: Errors and Warnings flags
	Flags used to clear the different types of system faults.  Used with <ExecInfoClearFault>.

	SIDEKICK_SDK_CLEAR_ERROR_FAULT - Clears most recent system error.
	SIDEKICK_SDK_CLEAR_WARNING_FAULT - Clears most recent system warning.
	SIDEKICK_SDK_CLEAR_ERROR_LIST - Clears system error list.
	SIDEKICK_SDK_CLEAR_WARNING_LIST - Clears system warning list.
*/
#define SIDEKICK_SDK_CLEAR_ERROR_FAULT			((uint8_t)0x01)
#define SIDEKICK_SDK_CLEAR_WARNING_FAULT        ((uint8_t)0x02)
#define SIDEKICK_SDK_CLEAR_ERROR_LIST			((uint8_t)0x04)
#define SIDEKICK_SDK_CLEAR_WARNING_LIST			((uint8_t)0x08)

//----------------------------------------------------------------------------------------------------------------//
/* Constants: Errors and Warnings
	System error codes. See SidekickSDKConstants.h for full list.
*/
#define SIDEKICK_SDK_FAULT_TEC_ERROR					((uint16_t)20)
#define SIDEKICK_SDK_FAULT_BTEMP_HIGH					((uint16_t)24)
#define SIDEKICK_SDK_FAULT_BTEMP_LOW					((uint16_t)25)
#define SIDEKICK_SDK_FAULT_TEC_RWAY						((uint16_t)26)
#define SIDEKICK_SDK_FAULT_MOTION_FAULT					((uint16_t)30)
#define SIDEKICK_SDK_FAULT_MOTION_STALL					((uint16_t)35)
#define SIDEKICK_SDK_FAULT_MOTION_UNINIT_CHAN			((uint16_t)40)
#define SIDEKICK_SDK_FAULT_MOTION_ERROR_CHAN			((uint16_t)45)
#define SIDEKICK_SDK_FAULT_EEPROM_CRC_MISMATCH			((uint16_t)50)
#define SIDEKICK_SDK_FAULT_CANNOT_GEN_DEFAULT_FAV		((uint16_t)55)
#define SIDEKICK_SDK_FAULT_MOTION_HOME_FAULT_CHAN		((uint16_t)60)
#define SIDEKICK_SDK_FAULT_CASE_TEMP1_FAULT				((uint16_t)65)
#define SIDEKICK_SDK_FAULT_CASE_TEMP2_FAULT				((uint16_t)66)
#define SIDEKICK_SDK_FAULT_CASE_TEMPS_BAD				((uint16_t)67)  //Both case temps are bad, so we can't determine case temp
#define SIDEKICK_SDK_FAULT_VSRC_ERROR					((uint16_t)70)
#define SIDEKICK_SDK_FAULT_SWEEP_ERROR					((uint16_t)71)
#define SIDEKICK_SDK_FAULT_HEAD_COMM_ERROR				((uint16_t)72)
#define SIDEKICK_SDK_FAULT_HEAD_READ_DATA_ERROR			((uint16_t)73)
#define SIDEKICK_SDK_FAULT_UNSUPPORTED_HEAD_ERROR		((uint16_t)74)
#define SIDEKICK_SDK_FAULT_HEAD_CABLE_MISMATCH			((uint16_t)75)
#define SIDEKICK_SDK_FAULT_INIT_WAVE_ERROR				((uint16_t)80)

//----------------------------------------------------------------------------------------------------------------//
/* Constants: Wavelength Units
	Units for functions that use wavelength values.

	SIDEKICK_SDK_UNITS_MICRONS - Micrometers, 1 x 10^-6 meters
	SIDEKICK_SDK_UNITS_CM1 - Wavenumbers in cm^-1 units.  This is the spatial frequency of the wavelength and is in cycles per cm.  
	SIDEKICK_SDK_UNITS_ENCODER - Encoder count (only used for reading out wavelength trigger values)
*/
#define SIDEKICK_SDK_UNITS_MICRONS			((uint8_t)1)
#define SIDEKICK_SDK_UNITS_CM1				((uint8_t)2)
#define SIDEKICK_SDK_UNITS_ENCODER			((uint8_t)3)

//----------------------------------------------------------------------------------------------------------------//
/* Constants: Laser Mode
	This is the mode the laser uses for emission.  Not all modes are supported by all laser heads.
	Modulation modes are only supported by some heads (namely, Hedgehog does NOT support current modulation).
	Low Noise Filter modes are only supported by Hedgehog heads (and some MIRcat sleds).
	Mode release mode is only supported in Sidekick at this time.

	SIDEKICK_SDK_MODE_ERROR	- Invalid laser mode.
	SIDEKICK_SDK_MODE_PULSED - Pulsed laser mode.  The laser pulses on/off at the set repetition rate and pulse width.
	SIDEKICK_SDK_MODE_CW - Continuous Waveform Mode.  In this mode the laser emission is continuously on.  
	SIDEKICK_SDK_MODE_CW_MOD - Same as CW mode but with an analog modulation enable signal enabled.  This is only supported
							   by laser heads that have a modulation enable input (such as MIRcat sleds).
	SIDEKICK_SDK_MODE_MHF - Same as CW mode but the MHF-II system is limited to current and wavelength limits that allow
							for mode-hop-free operation.
	SIDEKICK_SDK_MODE_MHF_MOD - Same as MHF mode but with an analog modulation signal enabled.
	SIDEKICK_SDK_MODE_CW_MR - CW Mode Release Mode.  This mode uses unfiltered CW mode and turns the laser off and back on
							each time the laser finishes a tune.  This "releases" any "pulled" modes and turns back on at 
							the natural mode.  The fast turn off/on reduces any temperature transients.
	SIDEKICK_SDK_MODE_CW_MOD_MR - Same as CW_MR but with modulation enabled.  Only applies to head types that allow current modulation.
	SIDEKICK_SDK_MODE_CW_LN_1 - CW Low Noise Filter 1.  This implements low pass filtering with a cutoff frequency of ~20 KHz to the QCL driver.
	SIDEKICK_SDK_MODE_CW_LN_2 - CW Low Noise Filter 2.  This severely low pass filters the QCL driver with a cutoff frequency of ~0.3 Hz.
	SIDEKICK_SDK_MODE_CW_LN_1_MOD - CW Low Noise Filter 1 with Modulation.  This is the same as CW_LN_1 but allows QCL current modulation on 
									supported heads.
	SIDEKICK_SDK_MODE_CW_LN_2_MR - CW Low Noise Filter 2 with Mode Release.  This is the same as CW_LN_2, but implements Mode Release functionality.
								The turn off/on is controlled in firmware for the shortest possible off time.

*/
#define SIDEKICK_SDK_MODE_ERROR				((uint8_t)0)
#define SIDEKICK_SDK_MODE_PULSED			((uint8_t)1)
#define SIDEKICK_SDK_MODE_CW				((uint8_t)2)
#define SIDEKICK_SDK_MODE_CW_MOD			((uint8_t)3)
#define SIDEKICK_SDK_MODE_MHF				((uint8_t)4)
#define SIDEKICK_SDK_MODE_MHF_MOD			((uint8_t)5)
#define SIDEKICK_SDK_MODE_CW_MR				((uint8_t)6)
#define SIDEKICK_SDK_MODE_CW_MOD_MR			((uint8_t)7)
#define SIDEKICK_SDK_MODE_CW_LN_1			((uint8_t)8)
#define SIDEKICK_SDK_MODE_CW_LN_2			((uint8_t)9)
#define SIDEKICK_SDK_MODE_CW_LN_1_MOD		((uint8_t)10)
#define SIDEKICK_SDK_MODE_CW_LN_2_MR		((uint8_t)11)

//----------------------------------------------------------------------------------------------------------------//
/* Constants: Trigger Mode
	This is the laser triggering mode for controlling QCL on/off.  

	SIDEKICK_SDK_TRIG_INTERNAL - The laser internally controls pulse triggering based on set parameters.
	SIDEKICK_SDK_TRIG_EXT_TRIG - The laser uses an external TTL trigger signal to control the start of a laser pulse.
								 The duration of the laser pulse is controlled by the laser pulse width setting.  A pulse
								 is started on the rising edge of the external TTL signal with a jitter of up to 20 ns.
	SIDEKICK_SDK_TRIG_EXT_PASSTHRU - This is similar to external trigger mode, but the laser output simply follows the 
									 external TTL signal, with limits.  CW lasers are not limited.  Pulsed only lasers 
									 are limited to BOTH their maximum pulse width and duty cycle.  This mode only uses
									 combinational logic between the external TTL signal and the laser trigger enable to
									 limit jitter to near zero.
*/
#define SIDEKICK_SDK_TRIG_INTERNAL			((uint8_t)1)
#define SIDEKICK_SDK_TRIG_EXT_TRIG			((uint8_t)2)
#define SIDEKICK_SDK_TRIG_EXT_PASSTHRU		((uint8_t)3)

//----------------------------------------------------------------------------------------------------------------//
/* Constants: Process Trigger Mode
	For step scan modes (Step & Measure and Multi-Spectral) a process trigger is used to go to the next step in the scan.
	The system provides the option to use three different types of process trigger modes below.

	SIDEKICK_SDK_PROCESS_TRIGGER_INTERNAL - Laser controller controls all timing for step scan modes.
	SIDEKICK_SDK_PROCESS_TRIGGER_EXTERNAL - External trigger on Sidekick 15-pin I/O connector must be provided to advance to next step.  
										Signal must be pulled low for ~250 ms to trigger a step.
	SIDEKICK_SDK_PROCESS_TRIGGER_MANUAL - Manual trigger command from PC must be sent to advance to next step.
*/
#define SIDEKICK_SDK_PROCESS_TRIGGER_INTERNAL		((uint8_t)1)		
#define SIDEKICK_SDK_PROCESS_TRIGGER_EXTERNAL		((uint8_t)2)		
#define SIDEKICK_SDK_PROCESS_TRIGGER_MANUAL			((uint8_t)3)

//----------------------------------------------------------------------------------------------------------------//
/* Constants: Voltage Indexes
	These constants are used with <GetInfoSysVoltages> to retrieve a specifically desired voltage.

	SIDEKICK_SDK_INFO_NUM_SYS_VOLTAGES - Maximum number of voltages supported by <ReadInfoSysVoltages> command.
	SIDEKICK_SDK_IDX_VBUS_SENSE - Sidekick input bus voltage (nominally 24V).
	SIDEKICK_SDK_IDX_IBUS_SENSE - Current being drawn from the input bus voltage.
	SIDEKICK_SDK_IDX_VOLTAGES_12V - 12V bus.
	SIDEKICK_SDK_IDX_VOLTAGES_3V3 - 3.3V bus.
	SIDEKICK_SDK_IDX_VOLTAGES_5V - +5V bus.
	SIDEKICK_SDK_IDX_VOLTAGES_N5V - -5V bus
	SIDEKICK_SDK_IDX_VOLTAGES_VQC_HI - VQC Hi voltage
	SIDEKICK_SDK_IDX_VOLTAGES_CABLE_ID - Cable Identification voltage
*/
#define SIDEKICK_SDK_INFO_NUM_SYS_VOLTAGES	8
#define SIDEKICK_SDK_IDX_VBUS_SENSE			((uint8_t)0)
#define SIDEKICK_SDK_IDX_IBUS_SENSE			((uint8_t)1)
#define SIDEKICK_SDK_IDX_VOLTAGES_12V       ((uint8_t)2)
#define SIDEKICK_SDK_IDX_VOLTAGES_3V3       ((uint8_t)3)
#define SIDEKICK_SDK_IDX_VOLTAGES_5V        ((uint8_t)4)
#define SIDEKICK_SDK_IDX_VOLTAGES_N5V       ((uint8_t)5)
#define SIDEKICK_SDK_IDX_VOLTAGES_VQC_HI    ((uint8_t)6)
#define SIDEKICK_SDK_IDX_VOLTAGES_CABLE_ID  ((uint8_t)7)

//---------------------------------------------------------------------------------------------------------------//
/* Constants: Scan Operations
	Used with <GetScanOperation> and <SetScanOperation>.

	SIDEKICK_SDK_SCAN_START					- Legacy scan operation from MIRcat not used in Sidekick.
	SIDEKICK_SDK_SCAN_STOP					- Stop scan.
	SIDEKICK_SDK_SCAN_PAUSE					- Not supported.
	SIDEKICK_SDK_SCAN_RESUME				- Not supported.
	SIDEKICK_SDK_SCAN_MAUAL_STEP			- This operation injects a manual step to the controller for use with manual process trigger mode.
	SIDEKICK_SDK_SCAN_START_SWEEP			- Start sweep scan for Sidekick controller.
	SIDEKICK_SDK_SCAN_START_STEP_MEASURE	- Start Step and Measure scan for Sidekick controller.
	SIDEKICK_SDK_SCAN_START_MULTI_SPECTRAL	- Start Multispectral scan for Sidekick controller.
*/
#define SIDEKICK_SDK_SCAN_START					((uint8_t)1)
#define SIDEKICK_SDK_SCAN_STOP					((uint8_t)2)
#define SIDEKICK_SDK_SCAN_PAUSE					((uint8_t)3)
#define SIDEKICK_SDK_SCAN_RESUME				((uint8_t)4)
#define SIDEKICK_SDK_SCAN_MAUAL_STEP			((uint8_t)5)
//Added for Sidekick
#define SIDEKICK_SDK_SCAN_START_SWEEP			((uint8_t)6)
#define SIDEKICK_SDK_SCAN_START_STEP_MEASURE	((uint8_t)7)
#define SIDEKICK_SDK_SCAN_START_MULTI_SPECTRAL	((uint8_t)8)

/* Constants: VCM Modulation
	Used with <GetVcmModulationParams> and <SetVcmModulationParams>.

	SIDEKICK_SDK_SCAN_VCM_MOD_TYPE_CURRENT	- VCM Current Modulation.
	SIDEKICK_SDK_SCAN_VCM_MOD_TYPE_POSITION - VCM Position Modulation.
*/
#define SIDEKICK_SDK_SCAN_VCM_MOD_TYPE_CURRENT	((uint8_t)1)
#define SIDEKICK_SDK_SCAN_VCM_MOD_TYPE_POSITION	((uint8_t)2)

//----------------------------------------------------------------------------------------------------------------//
/* Constants: System Communication Protocol Version
	Version of communications protocol used by the system.

	SIDEKICK_SDK_SYS_COMM_PROT_V0		- Used by MIRcat laser systems.
	SIDEKICK_SDK_SYS_COMM_PROT_V1		- Used by Sidekick laser controllers.	
*/
#define SIDEKICK_SDK_SYS_COMM_PROT_V0		((uint8_t)0)
#define SIDEKICK_SDK_SYS_COMM_PROT_V1		((uint8_t)1)


//----------------------------------------------------------------------------------------------------------------//
/* Constants: System Type
	Type of controller in laser system.
	
	SIDEKICK_SDK_SYS_TYPE_UNKNOWN		- Unknown system.
	SIDEKICK_SDK_SYS_TYPE_MIRCAT		- MIRCAT system.
	SIDEKICK_SDK_SYS_TYPE_SIDEKICK		- Sidekick Controller.
*/
#define SIDEKICK_SDK_SYS_TYPE_UNKNOWN		((uint8_t)0)
#define SIDEKICK_SDK_SYS_TYPE_MIRCAT		((uint8_t)1)
#define SIDEKICK_SDK_SYS_TYPE_SIDEKICK		((uint8_t)3)
#define SIDEKICK_SDK_SYS_TYPE_MIRCAT_V2		((uint8_t)4)
#define SIDEKICK_SDK_SYS_TYPE_SYS_BOARD		((uint8_t)5)

//----------------------------------------------------------------------------------------------------------------//
/* Constants: String Lengths
	String length constants.
	
	SIDEKICK_USB_SERIAL_NO_MAX_LEN		- Maximum length of the serial number programmed into the USB controller.
	SIDEKICK_USB_DESCRIPTION_MAX_LEN	- Maximum length of the description programmed into the USB controller.
	SIDEKICK_SDK_STRLEN_MAX_MFG_DATE	- Maximum Length of the controller manufacturing date string.
	SIDEKICK_SDK_STRLEN_MAX_MODEL_NUM	- Maximum Length of the controller model number string.
	SIDEKICK_SDK_STRLEN_MAX_SERIAL_NUM	- Maximum Length of the controller serial number string.
*/
#define SIDEKICK_USB_SERIAL_NO_MAX_LEN		((uint16_t)16)
#define SIDEKICK_USB_DESCRIPTION_MAX_LEN	((uint16_t)64)

#define SIDEKICK_SDK_STRLEN_MAX_MFG_DATE	(DLS_SCI_MFG_DATE_STR_LEN + 1)
#define SIDEKICK_SDK_STRLEN_MAX_MODEL_NUM	(DLS_SCI_MODEL_NUM_STR_LEN + 1)
#define SIDEKICK_SDK_STRLEN_MAX_SERIAL_NUM	(DLS_SCI_SERIAL_NUM_STR_LEN + 1)
//----------------------------------------------------------------------------------------------------------------//
/* Constants: Head Electronics
	Defines the QCL drive electronics version in the connected laser head.

SIDEKICK_SDK_HEAD_ELECT_GEN1 - Laser head electronics type that is unsupported by Sidekick.  These are the original DLS shunt-switched laser heads.
SIDEKICK_SDK_HEAD_ELECT_HFQD - Lasers such as CW/Pulsed, UT, Unicorn, Aries, use HFQD drive electronics.
SIDEKICK_SDK_HEAD_ELECT_MHF - Mode Hop Free lasers.
*/
#define SIDEKICK_SDK_HEAD_ELECT_GEN1			0
#define SIDEKICK_SDK_HEAD_ELECT_HFQD			1
#define SIDEKICK_SDK_HEAD_ELECT_MHF				2
#define SIDEKICK_SDK_HEAD_ELECT_HFQD2			3

//----------------------------------------------------------------------------------------------------------------//
/* Constants: Laser Type
	This is the motion type for the connected laser head.
	
	SIDEKICK_SDK_LASER_TYPE_GEN1_PULSE	- Drivetrain for Gen1, HFQD, and MIRcat lasers.
	SIDEKICK_SDK_LASER_TYPE_MHF			- Mode-Hop-Free Gen1 motion type.
	SIDEKICK_SDK_LASER_TYPE_MHF2		- Mode-Hop-Free Gen2 motion type.
*/
//Laser Type Defines (Corresponds to drivetrain type in laser head)
#define SIDEKICK_SDK_LASER_TYPE_GEN1_PULSE		0
#define SIDEKICK_SDK_LASER_TYPE_MHF				1
#define SIDEKICK_SDK_LASER_TYPE_MHF2			2
#define SIDEKICK_SDK_LASER_TYPE_VOICECOIL		3

//----------------------------------------------------------------------------------------------------------------//
/* Constants: Head Type1
	This is the laser head type (tunable or fixed wavelength).

	SIDEKICK_SDK_HEAD_TYPE_TUNABLE - Indicates the head supports wavelength tuning.
	SIDEKICK_SDK_HEAD_TYPE_ARIES - Indicates the head is a fixed wavelength laser.
*/
#define SIDEKICK_SDK_HEAD_TYPE_TUNABLE			0
#define SIDEKICK_SDK_HEAD_TYPE_ARIES			1

//----------------------------------------------------------------------------------------------------------------//
/* Constant: SIDEKICK_SDK_SEARCH_TIMEOUT
	Default search timeout, 500 milliseconds.	
*/
#define SIDEKICK_SDK_SEARCH_TIMEOUT 500		// ms

//----------------------------------------------------------------------------------------------------------------//
/* Constants: Scan statuses
	These defines indicate the state of a current scan.

	SIDEKICK_SDK_SCAN_PROG_IND_TEC_INPROGRESS			- Waiting for TEC to get to correct temp.
	SIDEKICK_SDK_SCAN_PROG_IND_MOTION_INPROGRESS		- Waiting for grating to move to correct position.
	SIDEKICK_SDK_SCAN_PROG_IND_DEALY_TIMER_INPROGRESS	- Waiting for delay timer.
	SIDEKICK_SDK_SCAN_PROG_IND_SCAN_ACTIVE				- Scanning is active - all the preprocessing is done.
	SIDEKICK_SDK_SCAN_PROG_IND_SCAN_INACTIVE			- This status is no longer used.
	SIDEKICK_SDK_SCAN_PROG_IND_WAITING_PROC_TRIG		- The system is waiting for a process trigger (for external and manual process trigger modes).
*/
#define SIDEKICK_SDK_SCAN_PROG_IND_TEC_INPROGRESS			((uint8_t)0x01)     
#define SIDEKICK_SDK_SCAN_PROG_IND_MOTION_INPROGRESS		((uint8_t)0x02)     
#define SIDEKICK_SDK_SCAN_PROG_IND_DEALY_TIMER_INPROGRESS	((uint8_t)0x04)		
#define SIDEKICK_SDK_SCAN_PROG_IND_SCAN_ACTIVE				((uint8_t)0x08)		
#define SIDEKICK_SDK_SCAN_PROG_IND_SCAN_INACTIVE			((uint8_t)0x10)		
#define SIDEKICK_SDK_SCAN_PROG_IND_WAITING_PROC_TRIG		((uint8_t)0x20)		

//----------------------------------------------------------------------------------------------------------------//
/* Constant: QCL Statuses
	These bitmask status defines indicate the current QCL state.  Used with <GetInfoQclInfo>.

	SIDEKICK_SDK_INFO_QCL_EMISSION_ON			- Indicates if laser emission is on.
	SIDEKICK_SDK_INFO_QCL_MODE_PULSED			- Laser mode set to pulsed mode.
	SIDEKICK_SDK_INFO_QCL_MODE_CW				- Laser mode set to CW mode.
	SIDEKICK_SDK_INFO_QCL_MOD_ENABLED			- Laser modulation enabled.
	SIDEKICK_SDK_INFO_QCL_CURSRC_CMD_ENABLED	- Mode-Hop-Free current source is enabled.
	SIDEKICK_SDK_INFO_QCL_CURSRC_MOD_ENABLED	- Mode-Hop-Free current source modulation is enabled.
*/
#define SIDEKICK_SDK_INFO_QCL_EMISSION_ON         ((uint8_t)0x01)
#define SIDEKICK_SDK_INFO_QCL_MODE_PULSED         ((uint8_t)0x02)
#define SIDEKICK_SDK_INFO_QCL_MODE_CW             ((uint8_t)0x04)
#define SIDEKICK_SDK_INFO_QCL_MOD_ENABLED         ((uint8_t)0x08)
#define SIDEKICK_SDK_INFO_QCL_CURSRC_CMD_ENABLED  ((uint8_t)0x10)
#define SIDEKICK_SDK_INFO_QCL_CURSRC_MOD_ENABLED  ((uint8_t)0x20)

//----------------------------------------------------------------------------------------------------------------//
/* Constant: TEC Statuses
	These bitmask status defines indicate the current state of the laser Thermo-Electric Cooler (TEC).  Used with <GetInfoTecInfo>.

	SIDEKICK_SDK_INFO_TEC_ENABLED				- Indicates if the TEC is enabled.
	SIDEKICK_SDK_INFO_TEC_ON_TARGET				- Temperature control is on target to the set temperature.
	SIDEKICK_SDK_INFO_TEC_FAULTED				- Thermal control system is faulted.
	SIDEKICK_SDK_INFO_TEC_THERM_RUNAWAY			- Controller has detected a thermal runaway condition.
	SIDEKICK_SDK_INFO_QCL_ON_TEC_OUT_OF_RANGE	- The QCL is on but the thermal control system is unable to maintain temperature and the ABS(QCL Temp - Target Temp) > 15 degrees C.
*/
#define SIDEKICK_SDK_INFO_TEC_ENABLED              ((uint8_t)0x01)
#define SIDEKICK_SDK_INFO_TEC_ON_TARGET			   ((uint8_t)0x02)
#define SIDEKICK_SDK_INFO_TEC_FAULTED              ((uint8_t)0x04)
#define SIDEKICK_SDK_INFO_TEC_THERM_RUNAWAY        ((uint8_t)0x08)
#define SIDEKICK_SDK_INFO_QCL_ON_TEC_OUT_OF_RANGE  ((uint8_t)0x10)

//----------------------------------------------------------------------------------------------------------------//
/* Constant: Wavelength Trigger Type
	These defines indicate the type of wavelength trigger value to read out from the laser.  Used with .

	SIDEKICK_SDK_SCAN_MARKER_TYPE_TARGET		- Wavelength trigger target value
	SIDEKICK_SDK_SCAN_MARKER_TYPE_CAPTURED		- Actual wavelength trigger value that was captured during a scan
*/
#define SIDEKICK_SDK_SCAN_MARKER_TYPE_TARGET		((uint8_t)1)
#define SIDEKICK_SDK_SCAN_MARKER_TYPE_CAPTURED		((uint8_t)2)

//----------------------------------------------------------------------------------------------------------------//
/* Constant: Wavelength Trigger Maximum Values
	These defines indicate the maximum values for some fields in the wavelength trigger reading data structures.

	SIDEKICK_SDK_SCAN_MAX_READ_MARKERS		- Maximum number of marker values that can be read out for each command.
	SIDEKICK_SDK_SCAN_MAX_TOTAL_MARKERS		- Total number of markers that can be generated/captured by Sidekick
*/
#define SIDEKICK_SDK_SCAN_MAX_READ_MARKERS		64
#define SIDEKICK_SDK_SCAN_MAX_TOTAL_MARKERS		2048


#endif