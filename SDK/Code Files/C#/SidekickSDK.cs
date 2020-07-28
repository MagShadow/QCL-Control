using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Runtime.InteropServices;
using Sidekick_Control;


namespace Sidekick_Control
{   
    public enum SDKConstants : uint
    {   // Return Values for API function calls
        SIDEKICK_SDK_RET_SUCCESS = 0,

        // Communication and Transport Errors
        SIDEKICK_SDK_RET_UNSUPPORTED_TRANSPORT      = 10,
        SIDEKICK_SDK_RET_INVALID_DEVICE_NUM         = 11,
        SIDEKICK_SDK_RET_DEVICE_INFO_ERROR          = 12,
        SIDEKICK_SDK_RET_CONNECT_ERROR              = 13,
        SIDEKICK_SDK_RET_COMM_ERROR                 = 14,
        SIDEKICK_SDK_RET_CONNECTION_LOST			= 15,
        SIDEKICK_SDK_RET_INVALID_DEVICE_HANDLE		= 17,

        // Initialization Errors
        SIDEKICK_SDK_RET_INIT_FAILURE = 20,

        // Function return Errors
        SIDEKICK_SDK_RET_LASER_NOT_ARMED = 30,
    
        DLS_SCI_INFO_SYS_MAX_NUM_ERRORS = 20
    }

    // Units
    // Wavelength & Wavenumber Units
    public enum Units:uint
    {   SIDEKICK_SDK_UNITS_MICRONS = 1,
        SIDEKICK_SDK_UNITS_CM1 = 2
    }

    // Laser Modes
    enum LaserModes: uint
    {   SIDEKICK_SDK_MODE_ERROR = 0,
        SIDEKICK_SDK_MODE_PULSED = 1,
        SIDEKICK_SDK_MODE_CW = 2,
        SIDEKICK_SDK_MODE_CW_MOD = 3,
        SIDEKICK_SDK_MODE_MHF = 4,
        SIDEKICK_SDK_MODE_MHF_MOD = 5,
        SIDEKICK_SDK_MODE_CW_MR = 6,
        SIDEKICK_SDK_MODE_CW_MOD_MR = 7,
        SIDEKICK_SDK_MODE_LN_1 = 8,
        SIDEKICK_SDK_MODE_LN_2 = 9,
        SIDEKICK_SDK_MODE_LN_1_MOD = 10,
        SIDEKICK_SDK_MODE_LN_2_MR = 11
    }

    // Pulse Triggering Modes
    enum TriggerModes: uint
    {   SIDEKICK_SDK_TRIG_INTERNAL = 1,
        SIDEKICK_SDK_TRIG_EXT_TRIG = 2,
        SIDEKICK_SDK_TRIG_EXT_PASSTHRU = 3,
        SIDEKICK_SDK_TRIG_WAVELENGTH = 4
    }

    //Process Triggering Modes
    enum ProcessTriggerModes: uint
    {
        SIDEKICK_SDK_PROC_TRIG_INTERNAL = 1,
        SIDEKICK_SDK_PROC_TRIG_EXTERNAL = 2,
        SIDEKICK_SDK_PROC_TRIG_MANUAL   = 3
    }

    // System Voltage Indexes
    public enum SystemVoltageIndex: byte
    {
        SIDEKICK_SDK_IDX_VBUS_SENSE			=0,
        SIDEKICK_SDK_IDX_IBUS_SENSE			=1,
        SIDEKICK_SDK_IDX_VOLTAGES_12V       =2,
        SIDEKICK_SDK_IDX_VOLTAGES_3V3       =3,
        SIDEKICK_SDK_IDX_VOLTAGES_5V        =4,
        SIDEKICK_SDK_IDX_VOLTAGES_N5V       =5,
		SIDEKICK_SDK_IDX_VOLTAGES_VQC_HI    =6,
		SIDEKICK_SDK_IDX_VOLTAGES_CABLE_ID 	=7
    }

    //VCM Modulation Type
    public enum VcmModulationType: byte
    {
        SIDEKICK_SDK_VCM_MOD_TYPE_CURRENT   = 1,
        SIDEKICK_SDK_VCM_MOD_TYPE_POSITION  = 2
    }
        
    //System Types
    enum SystemTypes: uint
    {   SIDEKICK_SDK_SYS_TYPE_UNKNOWN = 0,
        SIDEKICK_SDK_SYS_TYPE_MIRCAT = 1,
        SIDEKICK_SDK_SYS_TYPE_SIDEKICK = 3
    }

    //HEAD Types
    public enum HeadTypes: uint
    {   SIDEKICK_SDK_HEAD_TYPE_CW_PLS = 0,
        SIDEKICK_SDK_HEAD_TYPE_MHF_GEN1 = 1,
        SIDEKICK_SDK_HEAD_TYPE_MHF_GEN2=2,
        SIDEKICK_SDK_LASER_TYPE_VOICECOIL=3
    }

    // Communication Parameters
    enum CommParams: uint
    {   SIDEKICK_SDK_COMM_USB = 1,
        SIDEKICK_SDK_COMM_TCP = 2,
        SIDEKICK_SDK_COMM_DEFAULT = SIDEKICK_SDK_COMM_USB
    }

    // IO Status Mask Indexes
    enum IOStatusMask: uint
    {   SIDEKICK_SDK_IDX_IO_SCAN_DIR        =0x00000001, //System Output
        SIDEKICK_SDK_IDX_IO_WL_TRIG         =0x00000002, //System Output
        SIDEKICK_SDK_IDX_IO_GATE            =0x00000004, //System Input
        SIDEKICK_SDK_IDX_IO_TRIG_OUT        =0x00000008, //System Output
        SIDEKICK_SDK_IDX_IO_TUNED           =0x00000010, //System Output
        SIDEKICK_SDK_IDX_IO_PROCESS_TRIG    =0x00000020, //System Input
        SIDEKICK_SDK_IDX_IO_INTERLOCK       =0x00000040, //System Input
        SIDEKICK_SDK_IDX_IO_TRIG_IN         =0x00000080, //System Input
        SIDEKICK_SDK_IDX_IO_KEY_SWITCH      =0x00000100, //System Input
        SIDEKICK_SDK_IDX_IO_PUSH_BUTTON     =0x00000200, //System Input
        SIDEKICK_SDK_IDX_IO_LED3            =0x00000800, //System Output, Open Collector
        SIDEKICK_SDK_IDX_IO_ON_OFF_CTRL     =0x00001000  //System Input
    }

    //TEC status mask defines
    enum TECStatus: uint
    {   SIDEKICK_SDK_TEC_ENABLED = 0x01,
        SIDEKICK_SDK_TEC_ON_TARGET          =0x02,
        SIDEKICK_SDK_TEC_FAULTED            =0x04,
        SIDEKICK_SDK_TEC_THERM_RUNAWAY      =0x08
    }

    //Scan Operations
    public enum ScanOps: uint
    {
        SIDEKICK_SDK_SCAN_START					=1,
        SIDEKICK_SDK_SCAN_STOP					=2,
        SIDEKICK_SDK_SCAN_PAUSE					=3,
        SIDEKICK_SDK_SCAN_RESUME				=4,
        SIDEKICK_SDK_SCAN_MAUAL_STEP			=5,
        SIDEKICK_SDK_SCAN_START_SWEEP			=6,
        SIDEKICK_SDK_SCAN_START_STEP_MEASURE	=7,
        SIDEKICK_SDK_SCAN_START_MULTI_SPECTRAL	=8
    }

    enum StatusMask : uint
    {  // System Status Mask Indexes
        SIDEKICK_SDK_IDX_STATUS_MASK_INTERLOCK_SET			=0x00000001,
        SIDEKICK_SDK_IDX_STATUS_MASK_KEYSWITCH_SET			=0x00000002,
        SIDEKICK_SDK_IDX_STATUS_MASK_AT_TEMPERATURE			=0x00000004,
        SIDEKICK_SDK_IDX_STATUS_MASK_POST_IN_PROGRESS		=0x00000008,
        SIDEKICK_SDK_IDX_STATUS_MASK_POST_SUCCESS			=0x00000010,
        SIDEKICK_SDK_IDX_STATUS_MASK_SCAN_IN_PROGRESS		=0x00000020,
        SIDEKICK_SDK_IDX_STATUS_MASK_MOTION_IN_PROGRESS		=0x00000040,
        SIDEKICK_SDK_IDX_STATUS_MASK_TUNED_ON_TARGET		=0x00000080,
        SIDEKICK_SDK_IDX_STATUS_MASK_SYSTEM_ERROR			=0x00000100,
        SIDEKICK_SDK_IDX_STATUS_MASK_SYSTEM_WARNING			=0x00000200,
        SIDEKICK_SDK_IDX_STATUS_MASK_LASER_ARMED			=0x00000400,
        SIDEKICK_SDK_IDX_STATUS_MASK_SCAN_SETUP				=0x00000800,
        SIDEKICK_SDK_IDX_STATUS_MASK_LASER_FIRING			=0x00001000,
        SIDEKICK_SDK_IDX_STATUS_MASK_BTEMP1_FAULT		 	=0x00002000,
        SIDEKICK_SDK_IDX_STATUS_MASK_BTEMP2_FAULT           =0x00004000,
        SIDEKICK_SDK_IDX_STATUS_MASK_PCB_TEMP_FAULT			=0x00008000,
        SIDEKICK_SDK_IDX_STATUS_MASK_POINTER_INSTALLED		=0x00010000,
        SIDEKICK_SDK_IDX_STATUS_MASK_POINTER_ENABLED		=0x00020000,
        SIDEKICK_SDK_IDX_STATUS_MASK_MOTION_FAULTED         =0x00040000,
        SIDEKICK_SDK_IDX_STATUS_MASK_HEAD_INSTALLED			=0x00080000,
        SIDEKICK_SDK_IDX_STATUS_MASK_VSRC_GOOD				=0x00100000,
        SIDEKICK_SDK_IDX_STATUS_MASK_HOMING_COMPLETED       =0x00200000
    }

    enum SystemErrorList: uint
    {
        //System Errors/Warnings and Fault List flags
        SIDEKICK_SDK_CLEAR_ERROR_FAULT			=0x01,
        SIDEKICK_SDK_CLEAR_WARNING_FAULT        =0x02,
        SIDEKICK_SDK_CLEAR_ERROR_LIST			=0x04,
        SIDEKICK_SDK_CLEAR_WARNING_LIST			=0x08,
    }

    enum SystemErrorMask: uint
    {   //System Errors/Warnings
        SIDEKICK_SDK_FAULT_TEC_ERROR					=20,
        SIDEKICK_SDK_FAULT_BTEMP_HIGH					=24,
        SIDEKICK_SDK_FAULT_BTEMP_LOW					=25,
        SIDEKICK_SDK_FAULT_TEC_RWAY						=26,
        SIDEKICK_SDK_FAULT_MOTION_FAULT					=30,
        SIDEKICK_SDK_FAULT_MOTION_STALL					=35,
        SIDEKICK_SDK_FAULT_MOTION_UNINIT_CHAN			=40,
        SIDEKICK_SDK_FAULT_MOTION_ERROR_CHAN			=45,
        SIDEKICK_SDK_FAULT_EEPROM_CRC_MISMATCH			=50,
        SIDEKICK_SDK_FAULT_CANNOT_GEN_DEFAULT_FAV		=55,
        SIDEKICK_SDK_FAULT_MOTION_HOME_FAULT_CHAN		=60,
        SIDEKICK_SDK_FAULT_CASE_TEMP1_FAULT				=65,
        SIDEKICK_SDK_FAULT_CASE_TEMP2_FAULT				=66,
        SIDEKICK_SDK_FAULT_CASE_TEMPS_BAD				=67,  //Both case temps are bad, so we can't determine case temp
        SIDEKICK_SDK_FAULT_VSRC_ERROR					=70,
        SIDEKICK_SDK_FAULT_SWEEP_ERROR					=71,
        SIDEKICK_SDK_FAULT_HEAD_I2C_ERROR               =72,
        SIDEKICK_SDK_FAULT_EEPROM_READ_ERROR            =73,
        SIDEKICK_SDK_FAULT_UNSUPPORTED_HEAD_ERROR       =74,
        SIDEKICK_SDK_FAULT_HEAD_CABLE_MISMATCH          =75,
        SIDEKICK_SDK_FAULT_INIT_WAVE_ERROR              =80,
        SIDEKICK_SDK_FAULT_UNKNOWN_ERROR                =2001
    }

    //Head Type Defines (Corresponds to TheadType in laser head)
    enum HeadType: uint 
    {   
        SIDEKICK_SDK_HEAD_TYPE_TUNABLE			=0,
        SIDEKICK_SDK_HEAD_TYPE_ARIES			=1,
    }

    //Laser Type Defines (Corresponds to drivetrain type in laser head)
    public enum LaserType: uint
    {
        SIDEKICK_SDK_LASER_TYPE_GEN1_PULSE		=0,
        SIDEKICK_SDK_LASER_TYPE_MHF				=1,
        SIDEKICK_SDK_LASER_TYPE_MHF2			=2,
        SIDEKICK_SDK_LASER_TYPE_VOICECOIL       =3
    }

    //Head Electronics Defines (Corresponds to TheadElect in laser head)
    enum ElectrType : uint
    {
        SIDEKICK_SDK_HEAD_ELECT_GEN1 = 0,
        SIDEKICK_SDK_HEAD_ELECT_HFQD = 1,
        SIDEKICK_SDK_HEAD_ELECT_MHF = 2,
        SIDEKICK_SDK_HEAD_ELECT_HFQD2 = 3
    }

    public class StringLengths
    {
        public const uint SIDEKICK_SDK_STRLEN_MAX_MFG_DATE = 10;
        public const uint SIDEKICK_SDK_STRLEN_MAX_MODEL_NUM = 24;
        public const uint SIDEKICK_SDK_STRLEN_MAX_SERIAL_NUM = 24;
        
        public const uint SIDEKICK_USB_SERIAL_NO_MAX_LEN = 16;
        public const uint SIDEKICK_USB_DESCRIPTION_MAX_LEN = 64;
    }

    enum DLS_SCI_DEVICE_TRANSPORT : uint
    {
	    DLS_NUL_TRANSPORT=0,
	    DLS_USB_TRANSPORT=1,
	    DLS_UDP_TRANSPORT=2,
	    DLS_TCP_TRANSPORT=3
    }

    [StructLayoutAttribute(LayoutKind.Sequential, CharSet = CharSet.Ansi)] public struct DLS_SCI_SYSTEM_INFO
    {
        [MarshalAsAttribute(UnmanagedType.ByValTStr, SizeConst = 24 + 1)] public string SystemSerialNumber;
        [MarshalAsAttribute(UnmanagedType.ByValTStr, SizeConst = 24 + 1)] public string SystemModelNumber;
        public byte SystemType;
        public UInt32 StatusWord;
        public UInt16 ErrorWord;
        public UInt16 WarningWord;

        [MarshalAsAttribute(UnmanagedType.I1)] public bool HeadConnected;
        [MarshalAsAttribute(UnmanagedType.ByValTStr, SizeConst = 24 + 1)] public string HeadSerialNumber;
        [MarshalAsAttribute(UnmanagedType.ByValTStr, SizeConst = 24 + 1)] public string HeadModelNumber;
        public byte HeadType;
        public float min_ww;
        public float max_ww;
        public byte ww_units;
        public float mhf_min_ww;
        public float mhf_max_ww;
        public byte mhf_ww_units;
        public byte DriveTrainType;			// CWP, MHF, Voice Coil
	    public byte HeadElectronicsType;	// Gen 1, HFQD, MHF, HFQD2
	    public byte DeviceType;				// QC or IC
        public bool FixedWavelength;		// tuneable or not (redundant)

        public override string ToString()
        {
            string returnStr = "System Information\n" +
                "Serial Number:\t" + SystemSerialNumber + "\n" +
                "Model Number:\t" + SystemModelNumber + "\n" +
                "Type:\t\t" + SystemType + "\n";

            string statusString = String.Format("Status:\t{0:X}\nWarning:\t{1:X}\nError:\t{2:X}\n", StatusWord, WarningWord, ErrorWord);

            returnStr += statusString;

            if (HeadConnected)
            {
                string headStr = "Head Serial Number:\t" + HeadSerialNumber + "\n" +
                    "Head Model Number:\t" + HeadModelNumber + "\n" +
                    "Head Type:\t\t" + HeadType + "\n";

                returnStr += headStr;
            }
            else returnStr += "No Head Connected.\n";

            return returnStr;
        }
    }

    [StructLayoutAttribute(LayoutKind.Sequential, CharSet = CharSet.Ansi)] public struct DLS_SCI_IP_TRANSPORT_INFO
    {   [MarshalAsAttribute(UnmanagedType.ByValTStr, SizeConst = 48)] public string ipAddressString;
        public UInt32 ipAddress;
        public UInt16 tcpCommandPort;

        public override string ToString()
        {
            string returnStr = "IP Transport\n" + "TCP Command Address:Port:\t" + ipAddressString + ":" + tcpCommandPort + "\n";

            return returnStr;
        }
    }

    [StructLayoutAttribute(LayoutKind.Sequential, CharSet = CharSet.Ansi)]
    public struct DLS_SCI_USB_TRANSPORT_INFO
    {
        UInt32 ID;
        [MarshalAsAttribute(UnmanagedType.ByValTStr, SizeConst = 16)] public string SerialNumber;
        [MarshalAsAttribute(UnmanagedType.ByValTStr, SizeConst = 64)] public string Description;

        public override string ToString()
        {
            string returnStr = "USB Transport\n" + "Serial Number:\t" + SerialNumber + "\n" +
                "Description:\t" + Description + "\n";

            return returnStr;
        }
    }

    [StructLayoutAttribute(LayoutKind.Sequential)]
    public unsafe struct DLS_SCI_INFO_SYSTEM_ERROR_LIST	//Same as MIRcat
    {
        public fixed UInt16 status_list[(int)SDKConstants.DLS_SCI_INFO_SYS_MAX_NUM_ERRORS];
        public UInt16 num_items;
        public UInt16 index;
    }

    [StructLayoutAttribute(LayoutKind.Sequential)]
    public unsafe struct DLS_SCI_INFO_SYSTEM_ERRORS
    {
        public unsafe DLS_SCI_INFO_SYSTEM_ERROR_LIST system_errors;
        public unsafe DLS_SCI_INFO_SYSTEM_ERROR_LIST system_warnings;
    }

    [StructLayoutAttribute(LayoutKind.Sequential, CharSet = CharSet.Ansi)] public struct DLS_SCI_DEVICE_INFO
    {   [MarshalAsAttribute(UnmanagedType.I1)] public bool systemInfoValid;
        public DLS_SCI_SYSTEM_INFO systemInfo;
        [MarshalAsAttribute(UnmanagedType.I1)] public bool ipTransportValid;
        [MarshalAsAttribute(UnmanagedType.I1)] public bool ipTransportConnected;
        public uint ipTransportHandle;
        public DLS_SCI_IP_TRANSPORT_INFO ipTransportInfo;
        
        [MarshalAsAttribute(UnmanagedType.I1)] public bool usbFtdiTransportValid;
        [MarshalAsAttribute(UnmanagedType.I1)] public bool usbFtdiConnected;
        public uint usbFtdiHandle;
        public UInt32 usbFtdiDeviceNumber;
        public DLS_SCI_USB_TRANSPORT_INFO usbFtdiTransportInfo;

        public override string ToString()
        {
            string returnStr = "DEVICE INFORMATION\n";
            if ((ipTransportConnected) || (usbFtdiConnected))
            {
                string connectionStr = "";

                if(usbFtdiConnected) connectionStr += "Connected over USB on handle " + usbFtdiHandle + "\n";
                if (ipTransportConnected) connectionStr += "Connected over TCP on handle " + ipTransportHandle + "\n";

                returnStr += connectionStr;
            }

            if (systemInfoValid) returnStr += systemInfo.ToString();
            if (usbFtdiTransportValid) returnStr += "\n" + usbFtdiTransportInfo.ToString();
            if (ipTransportValid) returnStr += "\n" + ipTransportInfo.ToString();

            return returnStr;
        }
    }
    
    sealed class SidekickSDK
    {
        [DllImport("SidekickSDK.dll", CallingConvention = CallingConvention.Cdecl)] public static extern IntPtr SidekickSDK_GetCommErrDesc(UInt32 errCode);
        
        // API Functions Declared Below
	    /* COMMUNICATION FUNCTIONS */

	    /** 
	    <summary>Get Version of the API.</summary>
	    <param name="papiVersionMajor">Major Version of Sidekick API.</param>
	    <param name="papiVersionMinor">Minor Version of Sidekick API.</param>
	    <param name="papiVersionPatch">Patch Version of Sidekick API.</param>
	    <returns>Error code.  Possible codes: SIDEKICK_SDK_RET_SUCCESS</returns>
	    */
	    [DllImport("SidekickSDK.dll", CallingConvention = CallingConvention.Cdecl)] public static extern UInt32 SidekickSDK_GetAPIVersion( ref UInt16 papiVersionMajor, ref UInt16 papiVersionMinor, ref UInt16 papiVersionPatch);

	    /** 
	    <summary>Initialize the SidekickController object and enumerate USB objects.</summary>
	    <returns>Error code.  Possible codes: SIDEKICK_SDK_RET_SUCCESS</returns>
         * OBSOLETE *
	    */
	    [DllImport("SidekickSDK.dll", CallingConvention = CallingConvention.Cdecl)] public static extern UInt32 SidekickSDK_Initialize();


        [DllImport("SidekickSDK.dll", CallingConvention = CallingConvention.Cdecl)] public static extern UInt32 SidekickSDK_SearchForDevices(StringBuilder ipAddress, UInt16 port);
	    /** 
	    <summary>Re-initialize list of USB devices attached to the system.</summary>
	    <returns>Error code.  Possible codes: SIDEKICK_SDK_RET_SUCCESS</returns>
	    */
        [DllImport("SidekickSDK.dll", CallingConvention = CallingConvention.Cdecl)] public static extern UInt32 SidekickSDK_SearchForTCPDevices(StringBuilder ipAddress, UInt16 port);
        [DllImport("SidekickSDK.dll", CallingConvention = CallingConvention.Cdecl)] public static extern UInt32 SidekickSDK_SearchForUsbDevices();
        [DllImport("SidekickSDK.dll", CallingConvention = CallingConvention.Cdecl)] public static extern UInt32 SidekickSDK_ClearDeviceList();

	    /************************************************************************
	    <summary>Gets the number of devices attached to the system.  Must run a search
	    over TCP or USB before calling this function.</summary>
	    <param name="pnumDevices">Pointer to store number of devices attached to the system.</param>
	    <returns>Error code.  Possible codes: SIDEKICK_SDK_RET_SUCCESS</returns>
	    */
	    [DllImport("SidekickSDK.dll", CallingConvention = CallingConvention.Cdecl)] public static extern UInt32 SidekickSDK_GetNumOfDevices(ref UInt16 pnumDevices);

	    /************************************************************************
	    <summary>Gets the number of TCP devices attached to the system.  Must run a search
	    over TCP before calling this function.</summary>
	    <param name="pnumDevices">Pointer to store number of devices attached to the system.</param>
	    <returns>Error code.  Possible codes: SIDEKICK_SDK_RET_SUCCESS</returns>
	    */
        [DllImport("SidekickSDK.dll", CallingConvention = CallingConvention.Cdecl)] public static extern UInt32 SidekickSDK_GetNumTcpDevices(ref UInt16 pnumDevices);
	    /** 
	    <summary>Gets the number of USB FTDI devices attached to the system.  A call to
	    SidekickSDK_Initialize() or SidekickSDK_RescanUsbDevices() needs to be done prior
	    to calling this function.</summary>
	    <param name="pnumDevices">Number of USB devices attached to the system.</param>
	    <returns>Error code.  Possible codes: SIDEKICK_SDK_RET_SUCCESS</returns>
	    */
        [DllImport("SidekickSDK.dll", CallingConvention = CallingConvention.Cdecl)] public static extern UInt32 SidekickSDK_GetNumUsbDevices(ref UInt16 pnumDevices);

	    /** d
	    <summary>Gets the information for a USB device at the location provided.  A call to
	    SidekickSDK_Initialize() or SidekickSDK_RescanUsbDevices() needs to be done prior
	    to calling this function.</summary>
	    <param name="deviceNum">Device Instance.  Should be < number of devices from call to SidekickSDK_GetNumFtdiDevices().</param>
	    <param name="pFlags">USB device flags.</param>
	    <param name="pType">USB device type.</param>
	    <param name="pId">USB device ID.</param>
	    <param name="pLocId">USB device location ID.</param>
	    <param name="pSerialNum">16 character device serial number string.  Enough space should be allocated prior to calling this function.</param>
	    <param name="pDescription">64 character device description string.  Enough space should be allocated prior to calling this function.</param>
	    <returns>Error code.  Possible codes: SIDEKICK_SDK_RET_SUCCESS, SIDEKICK_SDK_RET_INVALID_DEVICE_NUM, SIDEKICK_SDK_RET_DEVICE_INFO_ERROR</returns>
	    */
        [DllImport("SidekickSDK.dll", CallingConvention = CallingConvention.Cdecl)] public static extern UInt32 SidekickSDK_GetDeviceInfo(UInt16 deviceNum, ref DLS_SCI_DEVICE_INFO pDeviceInfo);
        [DllImport("SidekickSDK.dll", CallingConvention = CallingConvention.Cdecl)] public static extern UInt32 SidekickSDK_GetDeviceInfoFromHandle(UInt32 p_deviceHandle, ref DLS_SCI_DEVICE_INFO pDeviceInfo);
        /************************************************************************ 
	    <summary>Print DLS_SCI_DEVICE_INFO to screen.</summary>
	    <param name="pDeviceInfo">Pointer to DLS_SCI_DEVICE_INFO structure that contains device information to print.</param>
	    */
	    [DllImport("SidekickSDK.dll", CallingConvention = CallingConvention.Cdecl)] public static extern void printDeviceInfo(ref DLS_SCI_DEVICE_INFO pDeviceInfo);

        /************************************************************************
	    <summary>Attempts to connect to a device with specified number from device list.</summary>
	    <param name="deviceNum">Device Instance.  Should be < number of devices from call to SidekickSDK_GetNumOfDevices().</param>
	    <returns>Error code.  Possible codes: SIDEKICK_SDK_RET_SUCCESS, SIDEKICK_SDK_RET_INVALID_DEVICE_NUM, SIDEKICK_SDK_RET_CONNECT_ERROR</returns>
	    */
	    [DllImport("SidekickSDK.dll", CallingConvention = CallingConvention.Cdecl)] public static extern UInt32 SidekickSDK_ConnectToDeviceNumber(ref UInt32 p_deviceHandle, UInt16 deviceNumber);

	    /** 
	    <summary>Attempts to connect to a USB device.</summary>
	    <param name="deviceNum">Device Instance.  Should be < number of devices from call to SidekickSDK_GetNumFtdiDevices().</param>
	    <returns>Error code.  Possible codes: SIDEKICK_SDK_RET_SUCCESS, SIDEKICK_SDK_RET_INVALID_DEVICE_NUM, SIDEKICK_SDK_RET_CONNECT_ERROR</returns>C
	    */
	    [DllImport("SidekickSDK.dll", CallingConvention = CallingConvention.Cdecl)] public static extern UInt32 SidekickSDK_ConnectToUsbDevice(ref UInt32 p_deviceHandle, UInt16 deviceNum);

	    /************************************************************************
	    <summary>Attempts to connect to a TCP device.</summary>
	    <param name="p_sIpAddress">Address in dot format of TCP device to connect to.</param>
	    <param name="n_nComPort">Port of TCP device to connect to.</param>
	    <returns>Error code.  Possible codes: SIDEKICK_SDK_RET_SUCCESS, SIDEKICK_SDK_RET_CONNECT_ERROR</returns>
	    */
       [DllImport("SidekickSDK.dll", CallingConvention = CallingConvention.Cdecl)] public static extern UInt32 SidekickSDK_ConnectToTcpDevice(ref UInt32 p_deviceHandle, string p_sIpAddress, ushort n_nComPort);
        /**
	    <summary>Closes connection to a USB device.</summary>
	    <returns>Error code.  Possible codes: SIDEKICK_SDK_RET_SUCCESS</returns>
	    */
	    [DllImport("SidekickSDK.dll", CallingConvention = CallingConvention.Cdecl)] public static extern UInt32 SidekickSDK_Disconnect(UInt32 p_deviceHandle);

	    [DllImport("SidekickSDK.dll", CallingConvention = CallingConvention.Cdecl)] public static extern UInt32 SidekickSDK_ReadStatusMask(UInt32 p_deviceHandle, ref UInt32 pStatusMask, ref UInt16 pSysErrorWord, ref UInt16 pSysWarningWord );
	
	    //Status Mask Helper Functions
	    [DllImport("SidekickSDK.dll", CallingConvention = CallingConvention.Cdecl)] public static extern UInt32 SidekickSDK_isInterlockedStatusSet(UInt32 p_deviceHandle, ref bool p_InterlockedStatusSet);
	    [DllImport("SidekickSDK.dll", CallingConvention = CallingConvention.Cdecl)] public static extern UInt32 SidekickSDK_isKeySwitchStatusSet(UInt32 p_deviceHandle, ref bool p_KeySwitchStatusSet);
	    [DllImport("SidekickSDK.dll", CallingConvention = CallingConvention.Cdecl)] public static extern UInt32 SidekickSDK_isTempStatusSet(UInt32 p_deviceHandle, ref bool p_TempStatusSet);
	    [DllImport("SidekickSDK.dll", CallingConvention = CallingConvention.Cdecl)] public static extern UInt32 SidekickSDK_isPostInProgressSet(UInt32 p_deviceHandle, ref bool p_PostInProgressSet);
	    [DllImport("SidekickSDK.dll", CallingConvention = CallingConvention.Cdecl)] public static extern UInt32 SidekickSDK_isPostSuccessfulSet(UInt32 p_deviceHandle, ref bool p_PostSuccessfulSet);
	    [DllImport("SidekickSDK.dll", CallingConvention = CallingConvention.Cdecl)] public static extern UInt32 SidekickSDK_isScanningSet(UInt32 p_deviceHandle, ref bool p_ScanningSet);
	    [DllImport("SidekickSDK.dll", CallingConvention = CallingConvention.Cdecl)] public static extern UInt32 SidekickSDK_isManualTuning(UInt32 p_deviceHandle, ref bool p_ManualTuning);
        [DllImport("SidekickSDK.dll", CallingConvention = CallingConvention.Cdecl)] public static extern UInt32 SidekickSDK_isTuned(UInt32 p_deviceHandle, ref bool p_Tuned);						// Tuned to desired wavelength?
	    [DllImport("SidekickSDK.dll", CallingConvention = CallingConvention.Cdecl)] public static extern UInt32 SidekickSDK_isSystemError(UInt32 p_deviceHandle, ref bool p_SystemError);
	    [DllImport("SidekickSDK.dll", CallingConvention = CallingConvention.Cdecl)] public static extern UInt32 SidekickSDK_isSystemWarning(UInt32 p_deviceHandle, ref bool p_SystemWarning);
	    [DllImport("SidekickSDK.dll", CallingConvention = CallingConvention.Cdecl)] public static extern UInt32 SidekickSDK_isEmissionOn(UInt32 p_deviceHandle, ref bool p_EmissionOn);
	    [DllImport("SidekickSDK.dll", CallingConvention = CallingConvention.Cdecl)] public static extern UInt32 SidekickSDK_isLaserArmed(UInt32 p_deviceHandle, ref bool p_LaserArmed);
	    [DllImport("SidekickSDK.dll", CallingConvention = CallingConvention.Cdecl)] public static extern UInt32 SidekickSDK_isScanSetupInProgressd(UInt32 p_deviceHandle, ref bool p_ScanSetupInProgress);
	    [DllImport("SidekickSDK.dll", CallingConvention = CallingConvention.Cdecl)] public static extern UInt32 SidekickSDK_isLaserFiring(UInt32 p_deviceHandle, ref bool p_LaserFiring);
	    //[DllImport("SidekickSDK.dll", CallingConvention = CallingConvention.Cdecl)] public static extern UInt32 SidekickSDK_isInitDone(UInt32 p_deviceHandle, ref bool p_InitDone);
	    [DllImport("SidekickSDK.dll", CallingConvention = CallingConvention.Cdecl)] public static extern UInt32 SidekickSDK_isBenchTemp1Faulty(UInt32 p_deviceHandle, ref bool p_BenchTemp1Faulty);
	    [DllImport("SidekickSDK.dll", CallingConvention = CallingConvention.Cdecl)] public static extern UInt32 SidekickSDK_isBenchTemp2Faulty(UInt32 p_deviceHandle, ref bool p_BenchTemp2Faulty);
	    [DllImport("SidekickSDK.dll", CallingConvention = CallingConvention.Cdecl)] public static extern UInt32 SidekickSDK_isPcbTempFaulty(UInt32 p_deviceHandle, ref bool p_PcbTempFaulty);
        [DllImport("SidekickSDK.dll", CallingConvention = CallingConvention.Cdecl)] public static extern UInt32 SidekickSDK_isMotionFaulted(UInt32 p_deviceHandle, ref bool p_MotionFaulted);
        [DllImport("SidekickSDK.dll", CallingConvention = CallingConvention.Cdecl)] public static extern UInt32 SidekickSDK_isHeadInstalled(UInt32 p_deviceHandle, ref bool p_HeadInstalled);
        [DllImport("SidekickSDK.dll", CallingConvention = CallingConvention.Cdecl)] public static extern UInt32 SidekickSDK_isVsrcGood(UInt32 p_deviceHandle, ref bool p_VsrcGood);
        [DllImport("SidekickSDK.dll", CallingConvention = CallingConvention.Cdecl)] public static extern UInt32 SidekickSDK_isHomingCompleted(UInt32 p_deviceHandle, ref bool p_HomingCompleted);
	    [DllImport("SidekickSDK.dll", CallingConvention = CallingConvention.Cdecl)] public static extern UInt16 SidekickSDK_getSystemErrorWord(UInt32 p_deviceHandle, ref UInt16 p_SystemErrorWord);
	    [DllImport("SidekickSDK.dll", CallingConvention = CallingConvention.Cdecl)] public static extern UInt16 SidekickSDK_getSystemWarningWord(UInt32 p_deviceHandle, ref UInt16 p_SystemWarningWord);

    ////////////////////////////////////////////////////////////
    // Info Commands
    ////////////////////////////////////////////////////////////
	    [DllImport("SidekickSDK.dll", CallingConvention = CallingConvention.Cdecl)] public static extern UInt32 SidekickSDK_ReadInfoSysInfo(UInt32 p_deviceHandle);
        [DllImport("SidekickSDK.dll", CallingConvention = CallingConvention.Cdecl)] public static extern UInt32 SidekickSDK_GetInfoSysInfo(UInt32 p_deviceHandle, ref byte p_supportedIfc, ref byte p_sysInfo, ref UInt32 p_reserved);
	    
        [DllImport("SidekickSDK.dll", CallingConvention = CallingConvention.Cdecl)] public static extern UInt32 SidekickSDK_ReadInfoStatusMask(UInt32 p_deviceHandle);
	    [DllImport("SidekickSDK.dll", CallingConvention = CallingConvention.Cdecl)] public static extern UInt32 SidekickSDK_GetInfoStatusMask(UInt32 p_deviceHandle, ref UInt32 p_statusMask, ref UInt16 p_sysErrorWord, ref UInt16 p_sysWarningWord );
        
        [DllImport("SidekickSDK.dll", CallingConvention = CallingConvention.Cdecl)] public static extern UInt32 SidekickSDK_ExecInfoClearFault(UInt32 p_deviceHandle,byte flags);
	    [DllImport("SidekickSDK.dll", CallingConvention = CallingConvention.Cdecl)] public static extern UInt32 SidekickSDK_ClearFault(UInt32 p_deviceHandle);
	    [DllImport("SidekickSDK.dll", CallingConvention = CallingConvention.Cdecl)] public static extern UInt32 SidekickSDK_ClearErrorList(UInt32 p_deviceHandle);
	    [DllImport("SidekickSDK.dll", CallingConvention = CallingConvention.Cdecl)] public static extern UInt32 SidekickSDK_ClearWarningList(UInt32 p_deviceHandle);
	    [DllImport("SidekickSDK.dll", CallingConvention = CallingConvention.Cdecl)] public static extern UInt32 SidekickSDK_GetInfoSystemErrorsAndWarnings(UInt32 p_deviceHandle);
        [DllImport("SidekickSDK.dll", CallingConvention = CallingConvention.Cdecl)] public static extern unsafe UInt32 SidekickSDK_GetInfoSystemErrorsList( UInt32 p_deviceHandle, ref DLS_SCI_INFO_SYSTEM_ERRORS sysErrorsAndWarnings );
        [DllImport("SidekickSDK.dll", CallingConvention = CallingConvention.Cdecl)] public static extern unsafe UInt32 SidekickSDK_GetInfoSystemWarningsList( UInt32 p_deviceHandle, ref DLS_SCI_INFO_SYSTEM_ERRORS sysWarningsList );
        	
        [DllImport("SidekickSDK.dll", CallingConvention = CallingConvention.Cdecl)] public static extern UInt32 SidekickSDK_ReadInfoLight(UInt32 p_deviceHandle);
        [DllImport("SidekickSDK.dll", CallingConvention = CallingConvention.Cdecl)] public static extern UInt32 SidekickSDK_GetInfoLight(UInt32 p_deviceHandle, ref byte p_lightStatus, ref float p_currentWW, ref byte p_currentWWUnit, ref byte p_curQcl);

	    [DllImport("SidekickSDK.dll", CallingConvention = CallingConvention.Cdecl)] public static extern UInt32 SidekickSDK_ReadInfoQclInfo(UInt32 p_deviceHandle, byte qcl_slot );
        [DllImport("SidekickSDK.dll", CallingConvention = CallingConvention.Cdecl)] public static extern UInt32 SidekickSDK_GetInfoQclInfo(UInt32 p_deviceHandle, ref byte p_channel, ref byte p_status, ref float p_qclVoltage, ref float p_qclVfet, ref float p_qclVsrc, 
														                                                                            ref float p_qclCurrent, ref float p_iSns, ref float p_currentSrc, ref float p_qclTemp, ref float p_vQcHi );
	    
        [DllImport("SidekickSDK.dll", CallingConvention = CallingConvention.Cdecl)] public static extern UInt32 SidekickSDK_ReadInfoTecInfo(UInt32 p_deviceHandle, byte channel);
        [DllImport("SidekickSDK.dll", CallingConvention = CallingConvention.Cdecl)] public static extern UInt32 SidekickSDK_GetInfoTecInfo(UInt32 p_deviceHandle,ref byte p_channel, ref byte p_status, ref float p_tecTemp, ref float p_tecCurrent, ref float p_tecVoltage);
	    [DllImport("SidekickSDK.dll", CallingConvention = CallingConvention.Cdecl)] public static extern UInt32 SidekickSDK_ReadInfoSysTemperatures(UInt32 p_deviceHandle);

        // temp1=PCB, temp2=Head/Case Temp, humidity1 = pcb humid sensor, others not used
        [DllImport("SidekickSDK.dll", CallingConvention = CallingConvention.Cdecl)] public static extern UInt32 SidekickSDK_GetInfoSysTemperatures(UInt32 p_deviceHandle, ref float p_temp1, ref float p_temp2, ref float p_temp3, 
							                                                                         ref float p_humidity1, ref float p_humidity2, 
							                                                                         ref float p_aux_temp1, ref float p_aux_temp2 );
	    [DllImport("SidekickSDK.dll", CallingConvention = CallingConvention.Cdecl)] public static extern UInt32 SidekickSDK_ReadInfoSysVoltages(UInt32 p_deviceHandle);
        [DllImport("SidekickSDK.dll", CallingConvention = CallingConvention.Cdecl)] public static extern UInt32 SidekickSDK_GetInfoSysVoltages(UInt32 p_deviceHandle,byte index, ref float p_voltage);
	    [DllImport("SidekickSDK.dll", CallingConvention = CallingConvention.Cdecl)] public static extern UInt32 SidekickSDK_ReadInfoIoStatus(UInt32 p_deviceHandle);
	    [DllImport("SidekickSDK.dll", CallingConvention = CallingConvention.Cdecl)] public static extern UInt32 SidekickSDK_GetInfoIoStatus(UInt32 p_deviceHandle, ref UInt32 p_io_status );

    ////////////////////////////////////////////////////////////
    //Scan Command Functions              
    ////////////////////////////////////////////////////////////

        //Single Tune Command Functions
	    [DllImport("SidekickSDK.dll", CallingConvention = CallingConvention.Cdecl)] public static extern UInt32 SidekickSDK_ExecTuneToWW(UInt32 p_deviceHandle);
        [DllImport("SidekickSDK.dll", CallingConvention = CallingConvention.Cdecl)] public static extern UInt32 SidekickSDK_SetTuneToWW(UInt32 p_deviceHandle, byte ww_unit, float wave, byte preferred_qcl);

	    [DllImport("SidekickSDK.dll", CallingConvention = CallingConvention.Cdecl)] public static extern UInt32 SidekickSDK_ReadWriteScanMode(UInt32 p_deviceHandle, bool fWrite);
        [DllImport("SidekickSDK.dll", CallingConvention = CallingConvention.Cdecl)] public static extern UInt32 SidekickSDK_GetScanMode(UInt32 p_deviceHandle, ref byte p_scanMode);
	    [DllImport("SidekickSDK.dll", CallingConvention = CallingConvention.Cdecl)] public static extern UInt32 SidekickSDK_SetScanMode(UInt32 p_deviceHandle, byte scanMode );
	    [DllImport("SidekickSDK.dll", CallingConvention = CallingConvention.Cdecl)] public static extern UInt32 SidekickSDK_ReadWriteSingleWWParams(UInt32 p_deviceHandle, bool fWrite);
	    [DllImport("SidekickSDK.dll", CallingConvention = CallingConvention.Cdecl)] public static extern UInt32 SidekickSDK_GetSingleWWParams(UInt32 p_deviceHandle, ref byte p_units, ref float p_wave, ref UInt32 p_duration, ref byte p_preferredQcl );
	    [DllImport("SidekickSDK.dll", CallingConvention = CallingConvention.Cdecl)] public static extern UInt32 SidekickSDK_SetSingleWWParams(UInt32 p_deviceHandle, byte units, float wave, UInt32 duration, byte preferredQcl );

        //Sweep Scan Command Functions
	    [DllImport("SidekickSDK.dll", CallingConvention = CallingConvention.Cdecl)] public static extern UInt32 SidekickSDK_ReadWriteSweepParams(UInt32 p_deviceHandle, bool fWrite);
	    [DllImport("SidekickSDK.dll", CallingConvention = CallingConvention.Cdecl)] public static extern UInt32 SidekickSDK_GetSweepParams(UInt32 p_deviceHandle, ref byte p_units, ref float p_start_ww, ref float p_stop_ww, ref float p_speed, ref UInt16 p_num_scans, ref byte p_pref_qcl, ref byte bidirectional );
	    [DllImport("SidekickSDK.dll", CallingConvention = CallingConvention.Cdecl)] public static extern UInt32 SidekickSDK_SetSweepParams(UInt32 p_deviceHandle, byte units, float start_ww, float stop_ww, float speed, UInt16 num_scans, byte pref_qcl, byte bidirectional );
	    [DllImport("SidekickSDK.dll", CallingConvention = CallingConvention.Cdecl)] public static extern UInt32 SidekickSDK_ReadWriteWavelengthTrigParams(UInt32 p_deviceHandle, bool fWrite);
	    [DllImport("SidekickSDK.dll", CallingConvention = CallingConvention.Cdecl)] public static extern UInt32 SidekickSDK_GetWavelengthTrigParams(UInt32 p_deviceHandle, ref byte units, ref float start_ww, ref float stop_ww, ref float spacing, ref byte qcl );
	    [DllImport("SidekickSDK.dll", CallingConvention = CallingConvention.Cdecl)] public static extern UInt32 SidekickSDK_SetWavelengthTrigParams(UInt32 p_deviceHandle, byte units, float start_ww, float stop_ww, float spacing, byte qcl );

        //Step Scan Command Functions
        [DllImport("SidekickSDK.dll", CallingConvention = CallingConvention.Cdecl)] public static extern UInt32 SidekickSDK_ReadWriteProcessTrigParams(UInt32 deviceHandle, bool fWrite);
        [DllImport("SidekickSDK.dll", CallingConvention = CallingConvention.Cdecl)] public static extern UInt32 SidekickSDK_GetProcessTrigParams(UInt32 deviceHandle, ref byte procTrigMode);
        [DllImport("SidekickSDK.dll", CallingConvention = CallingConvention.Cdecl)] public static extern UInt32 SidekickSDK_SetProcessTrigParams(UInt32 deviceHandle, byte procTrigMode);
        //Step & Measure Specific Command Functions
	    [DllImport("SidekickSDK.dll", CallingConvention = CallingConvention.Cdecl)] public static extern UInt32 SidekickSDK_ReadWriteStepMeasureParams(UInt32 p_deviceHandle, bool fWrite);    
	    [DllImport("SidekickSDK.dll", CallingConvention = CallingConvention.Cdecl)] public static extern UInt32 SidekickSDK_GetStepMeasureParams(UInt32 p_deviceHandle, ref byte p_units, ref float p_start_ww, ref float p_stop_ww, ref float p_step, 
                                                                                                                                                 ref UInt16 p_num_scans, ref byte p_keep_on, ref byte p_bidirectional, 
                                                                                                                                                 ref UInt32 dwell_time_ms, ref UInt32 trans_time_ms );
	    [DllImport("SidekickSDK.dll", CallingConvention = CallingConvention.Cdecl)] public static extern UInt32 SidekickSDK_SetStepMeasureParams(UInt32 p_deviceHandle, byte units, float start_ww, float stop_ww, float step, 
                                                                                                                                                 UInt16 num_scans, byte keep_on, byte bidirectional, 
																                                                                                 UInt32 dwell_time_ms, UInt32 trans_time_ms );
        //Multi-Spectral Specific Command Functions
	    [DllImport("SidekickSDK.dll", CallingConvention = CallingConvention.Cdecl)] public static extern UInt32 SidekickSDK_ReadWriteMultiSpectralParams(UInt32 p_deviceHandle, bool fWrite);
	    [DllImport("SidekickSDK.dll", CallingConvention = CallingConvention.Cdecl)] public static extern UInt32 SidekickSDK_GetMultiSpectralParams(UInt32 p_deviceHandle, ref byte scan_ww_unit, ref UInt16 num_steps, ref UInt16 num_scans,
                                                                                                                                                    ref bool bidirectional, ref bool data_allocated);
        [DllImport("SidekickSDK.dll", CallingConvention = CallingConvention.Cdecl)] public static extern UInt32 SidekickSDK_SetMultiSpectralParams(UInt32 p_deviceHandle, byte scan_ww_unit, UInt16 num_steps, UInt16 num_scans, bool bidirectional);
        [DllImport("SidekickSDK.dll", CallingConvention = CallingConvention.Cdecl)] public static extern UInt32 SidekickSDK_ReadWriteMultiSpectralElementParams(UInt32 p_deviceHandle, bool fWrite);                                                                                                        
        [DllImport("SidekickSDK.dll", CallingConvention = CallingConvention.Cdecl)] public static extern UInt32 SidekickSDK_GetMultiSpectralElement(UInt32 p_deviceHandle, ref UInt16 idx, ref float scan_ww, ref UInt32 dwell_time_ms, 
                                                                                                                                                    ref UInt32 transition_time_ms, ref bool keepLaserOn, ref bool stepInitialized);
        [DllImport("SidekickSDK.dll", CallingConvention = CallingConvention.Cdecl)] public static extern UInt32 SidekickSDK_SetMultiSpectralElement(UInt32 p_deviceHandle, UInt16 idx, float scan_ww, UInt32 dwell_time_ms, 
                                                                                                                                                    UInt32 transition_time_ms, bool keepLaserOn);
        //Scan Operation Command Functions
	    [DllImport("SidekickSDK.dll", CallingConvention = CallingConvention.Cdecl)] public static extern UInt32 SidekickSDK_ExecuteScanOperation(UInt32 p_deviceHandle);
	    [DllImport("SidekickSDK.dll", CallingConvention = CallingConvention.Cdecl)] public static extern UInt32 SidekickSDK_GetScanOperation(UInt32 p_deviceHandle, ref byte operation );
	    [DllImport("SidekickSDK.dll", CallingConvention = CallingConvention.Cdecl)] public static extern UInt32 SidekickSDK_SetScanOperation(UInt32 p_deviceHandle, byte operation );
        //Scan Status Command Functions
	    [DllImport("SidekickSDK.dll", CallingConvention = CallingConvention.Cdecl)] public static extern UInt32 SidekickSDK_ReadScanProgress(UInt32 p_deviceHandle);
	    [DllImport("SidekickSDK.dll", CallingConvention = CallingConvention.Cdecl)] public static extern UInt32 SidekickSDK_GetScanProgress(UInt32 p_deviceHandle, ref byte scan_progress_mask, ref UInt16 cur_scan_num, ref UInt16 cur_scan_percent );
        //VCM Modulation Commands
        [DllImport("SidekickSDK.dll", CallingConvention = CallingConvention.Cdecl)] public static extern UInt32 SidekickSDK_ReadWriteVcmModulationParams(UInt32 p_deviceHandle, bool fWrite);
        [DllImport("SidekickSDK.dll", CallingConvention = CallingConvention.Cdecl)] public static extern UInt32 SidekickSDK_GetVcmModulationParams(UInt32 p_deviceHandle, ref bool p_enable, ref byte p_type, ref UInt32 p_frequencyHz, ref UInt16 p_curModCounts, ref UInt32 p_posModCounts);
        [DllImport("SidekickSDK.dll", CallingConvention = CallingConvention.Cdecl)] public static extern UInt32 SidekickSDK_SetVcmModulationParams(UInt32 p_deviceHandle, bool enable, byte type, UInt32 frequencyHz, UInt16 curModCounts, UInt32 posModCounts);

    ////////////////////////////////////////////////////////////
    //Laser Command Functions
    ////////////////////////////////////////////////////////////
	    [DllImport("SidekickSDK.dll", CallingConvention = CallingConvention.Cdecl)] public static extern UInt32 SidekickSDK_ReadWriteLaserQclParams(UInt32 p_deviceHandle, bool fWrite, byte qcl_slot);
	    [DllImport("SidekickSDK.dll", CallingConvention = CallingConvention.Cdecl)] public static extern UInt32 SidekickSDK_GetLaserQclParams(UInt32 p_deviceHandle, ref byte qclSlot, ref UInt32 pulseRateHz, ref UInt32 pulseWidthNs, ref UInt16 currentMa,
						    ref float temperature, ref byte laserMode, ref byte pulseMode, ref float vsrc );
	    [DllImport("SidekickSDK.dll", CallingConvention = CallingConvention.Cdecl)] public static extern UInt32 SidekickSDK_SetLaserQclParams( UInt32 p_deviceHandle, byte qclSlot, UInt32 pulseRateHz, UInt32 pulseWidthNs, UInt16 currentMa,
						    float temperature, byte laserMode, byte pulseMode, float vsrc );
	    [DllImport("SidekickSDK.dll", CallingConvention = CallingConvention.Cdecl)] public static extern UInt32 SidekickSDK_ExecLaserOnOff(UInt32 p_deviceHandle);
	    [DllImport("SidekickSDK.dll", CallingConvention = CallingConvention.Cdecl)] public static extern UInt32 SidekickSDK_SetLaserOnOff(UInt32 p_deviceHandle, byte qcl_slot, bool on   );
	    [DllImport("SidekickSDK.dll", CallingConvention = CallingConvention.Cdecl)] public static extern UInt32 SidekickSDK_ExecLaserArmDisarm(UInt32 p_deviceHandle);
	    [DllImport("SidekickSDK.dll", CallingConvention = CallingConvention.Cdecl)] public static extern UInt32 SidekickSDK_SetLaserArmDisarm(UInt32 p_deviceHandle, bool arm );
	    [DllImport("SidekickSDK.dll", CallingConvention = CallingConvention.Cdecl)] public static extern UInt32 SidekickSDK_ReadWriteLaserQclVsrcParams(UInt32 p_deviceHandle, bool fWrite, byte qcl_slot );
	    [DllImport("SidekickSDK.dll", CallingConvention = CallingConvention.Cdecl)] public static extern UInt32 SidekickSDK_GetLaserQclVsrcParams(UInt32 p_deviceHandle, ref byte qcl_slot, ref float vsrc, ref float vsrcMaxPulsed, ref float vsrcMaxCw );
	    [DllImport("SidekickSDK.dll", CallingConvention = CallingConvention.Cdecl)] public static extern UInt32 SidekickSDK_SetLaserQclVsrcParams(UInt32 p_deviceHandle, byte qcl_slot, float vsrc, float vsrcMaxPulsed, float vsrcMaxCw );
        [DllImport("SidekickSDK.dll", CallingConvention = CallingConvention.Cdecl)] public static extern UInt32 SidekickSDK_ReadWriteForceFanOff(UInt32 deviceHandle, byte qcl_slot, bool bWrite);
        [DllImport("SidekickSDK.dll", CallingConvention = CallingConvention.Cdecl)] public static extern UInt32 SidekickSDK_SetForceFanOff(UInt32 deviceHandle, byte qcl_slot, bool force_fan_off, byte reserved);
        [DllImport("SidekickSDK.dll", CallingConvention = CallingConvention.Cdecl)] public static extern UInt32 SidekickSDK_GetForceFanOff(UInt32 deviceHandle, byte qcl_slot, ref bool force_fan_off, ref byte reserved);
        [DllImport("SidekickSDK.dll", CallingConvention = CallingConvention.Cdecl)] public static extern UInt32 SidekickSDK_SetEnableQclRamp(UInt32 deviceHandle, bool qcl_ramp_enable);
        [DllImport("SidekickSDK.dll", CallingConvention = CallingConvention.Cdecl)] public static extern UInt32 SidekickSDK_GetEnableQclRamp(UInt32 deviceHandle, ref bool qcl_ramp_enable);
        [DllImport("SidekickSDK.dll", CallingConvention = CallingConvention.Cdecl)] public static extern UInt32 SidekickSDK_ReadWriteEnableQclRamp(UInt32 deviceHandle, bool bWrite);

    ////////////////////////////////////////////////////////////	
    //Admin Command Functions
    ////////////////////////////////////////////////////////////	
	    [DllImport("SidekickSDK.dll", CallingConvention = CallingConvention.Cdecl)] public static extern UInt32 SidekickSDK_ReadAdminSysParams(UInt32 p_deviceHandle);
	    [DllImport("SidekickSDK.dll", CallingConvention = CallingConvention.Cdecl)] public static extern UInt32 SidekickSDK_ReadAdminQclParams(UInt32 p_deviceHandle, byte qcl_slot );
	    [DllImport("SidekickSDK.dll", CallingConvention = CallingConvention.Cdecl)] public static extern UInt32 SidekickSDK_ReadAdminFirmwareVersion(UInt32 p_deviceHandle);
        [DllImport("SidekickSDK.dll", CallingConvention = CallingConvention.Cdecl)] public static extern UInt32 SidekickSDK_GetAdminFirmwareVersions(UInt32 p_deviceHandle, ref byte major, ref byte minor, ref byte patch, ref byte pcbVer);
        [DllImport("SidekickSDK.dll", CallingConvention = CallingConvention.Cdecl)] public static extern UInt32 SidekickSDK_AdminGetEthernetFwVersion(UInt32 p_deviceHandle, ref byte major, ref byte minor, ref byte patch);
	    [DllImport("SidekickSDK.dll", CallingConvention = CallingConvention.Cdecl)] public static extern UInt32 SidekickSDK_ExecShutdownLaser(UInt32 p_deviceHandle, byte type, byte reason, UInt32 delay);

    //Admin Sys Helper Functions
	    [DllImport("SidekickSDK.dll", CallingConvention = CallingConvention.Cdecl)] public static extern UInt32 SidekickSDK_AdminGetSystemType(UInt32 p_deviceHandle, ref byte p_SystemType);
	    [DllImport("SidekickSDK.dll", CallingConvention = CallingConvention.Cdecl)] public static extern UInt32 SidekickSDK_AdminGetMfgDateStr(UInt32 p_deviceHandle, StringBuilder destString, ref byte strLen );
	    [DllImport("SidekickSDK.dll", CallingConvention = CallingConvention.Cdecl)] public static extern UInt32 SidekickSDK_AdminGetModelNumStr(UInt32 p_deviceHandle, StringBuilder destString, ref byte strLen);
	    [DllImport("SidekickSDK.dll", CallingConvention = CallingConvention.Cdecl)] public static extern UInt32 SidekickSDK_AdminGetSerialNumStr(UInt32 p_deviceHandle, StringBuilder destString, ref byte strLen );
	    [DllImport("SidekickSDK.dll", CallingConvention = CallingConvention.Cdecl)] public static extern UInt32 SidekickSDK_AdminGetNumQcls(UInt32 p_deviceHandle, ref byte p_NumQcls);
	    [DllImport("SidekickSDK.dll", CallingConvention = CallingConvention.Cdecl)] public static extern UInt32 SidekickSDK_AdminGetFwVersion(UInt32 p_deviceHandle, ref byte major, ref byte minor, ref byte patch );
	    [DllImport("SidekickSDK.dll", CallingConvention = CallingConvention.Cdecl)] public static extern UInt32 SidekickSDK_AdminGetHoursOfOperation(UInt32 p_deviceHandle, ref float p_HoursOfOperation);
	    [DllImport("SidekickSDK.dll", CallingConvention = CallingConvention.Cdecl)] public static extern UInt32 SidekickSDK_AdminGetControllerPcbVer(UInt32 p_deviceHandle, ref byte p_ControllerPcbVer);
	    [DllImport("SidekickSDK.dll", CallingConvention = CallingConvention.Cdecl)] public static extern UInt32 SidekickSDK_AdminGetMotionVersions(UInt32 p_deviceHandle, ref UInt16 modType, ref UInt16 hwType, ref UInt16 fwVer );

    //Admin QCL Helper Functions
	    [DllImport("SidekickSDK.dll", CallingConvention = CallingConvention.Cdecl)] public static extern UInt32 SidekickSDK_AdminQclGetSlot(UInt32 p_deviceHandle, ref byte p_Slot);
	    [DllImport("SidekickSDK.dll", CallingConvention = CallingConvention.Cdecl)] public static extern UInt32 SidekickSDK_AdminQclIsAvailable(UInt32 p_deviceHandle, ref bool p_QclIsAvailable);
	    [DllImport("SidekickSDK.dll", CallingConvention = CallingConvention.Cdecl)] public static extern UInt32 SidekickSDK_AdminQclGetLaserType(UInt32 p_deviceHandle, ref byte pLaserType);
	    
        [DllImport("SidekickSDK.dll", CallingConvention = CallingConvention.Cdecl)] public static extern UInt32 SidekickSDK_AdminQclGetSerialNumStr(UInt32 p_deviceHandle, StringBuilder destString, ref byte strLen );
	    [DllImport("SidekickSDK.dll", CallingConvention = CallingConvention.Cdecl)] public static extern UInt32 SidekickSDK_AdminQclGetModelNumStr(UInt32 p_deviceHandle, StringBuilder destString, ref byte strLen );
	    [DllImport("SidekickSDK.dll", CallingConvention = CallingConvention.Cdecl)] public static extern UInt32 SidekickSDK_AdminQclGetMfgDateStr(UInt32 p_deviceHandle, StringBuilder destString, ref byte strLen );
	    [DllImport("SidekickSDK.dll", CallingConvention = CallingConvention.Cdecl)] public static extern UInt32 SidekickSDK_AdminQclGetCurrentLimits(UInt32 p_deviceHandle, ref UInt16 pulseCurMax, ref UInt16 cwCurMax );
	    [DllImport("SidekickSDK.dll", CallingConvention = CallingConvention.Cdecl)] public static extern UInt32 SidekickSDK_AdminQclGetMhfCurrentLimits(UInt32 p_deviceHandle, ref UInt16 mhfCurMin, ref UInt16 mhfCurMax );
	    [DllImport("SidekickSDK.dll", CallingConvention = CallingConvention.Cdecl)] public static extern UInt32 SidekickSDK_AdminQclGetTempLimits(UInt32 p_deviceHandle, ref float minTemp, ref float nomTemp, ref float maxTemp );
	    [DllImport("SidekickSDK.dll", CallingConvention = CallingConvention.Cdecl)] public static extern UInt32 SidekickSDK_AdminQclGetPulseLimits(UInt32 p_deviceHandle, ref UInt32 min_pls_rate_hz, ref UInt32 max_pls_rate_hz, 
								                                                                                                                    ref UInt32 min_pls_width_ns, ref UInt32 max_pls_width_ns );
        [DllImport("SidekickSDK.dll", CallingConvention = CallingConvention.Cdecl)] public static extern UInt32 SidekickSDK_AdminQclGetPulseParams(UInt32 p_deviceHandle, ref UInt32 min_pls_rate_hz, ref UInt32 max_pls_rate_hz, 
								                                                                                                                    ref UInt32 min_pls_width_ns, ref UInt32 max_pls_width_ns, ref float max_duty_cycle);
	    [DllImport("SidekickSDK.dll", CallingConvention = CallingConvention.Cdecl)] public static extern UInt32 SidekickSDK_AdminQclGetScanRateLimits(UInt32 p_deviceHandle, ref float min_scan_rate, ref float max_scan_rate, ref byte scan_rate_units );
	    [DllImport("SidekickSDK.dll", CallingConvention = CallingConvention.Cdecl)] public static extern UInt32 SidekickSDK_AdminQclGetDwellTimeLimits(UInt32 p_deviceHandle, ref UInt32 min_dwell_time_us, ref UInt32 max_dwell_time_us );
	    [DllImport("SidekickSDK.dll", CallingConvention = CallingConvention.Cdecl)] public static extern UInt32 SidekickSDK_AdminQclGetWavelengthLimits(UInt32 p_deviceHandle, ref float min_ww, ref float max_ww, ref byte ww_units );
	    [DllImport("SidekickSDK.dll", CallingConvention = CallingConvention.Cdecl)] public static extern UInt32 SidekickSDK_AdminQclGetMhfWavelengthLimits(UInt32 p_deviceHandle, ref float min_ww, ref float max_ww, ref byte ww_units );	
	    [DllImport("SidekickSDK.dll", CallingConvention = CallingConvention.Cdecl)] public static extern UInt32 SidekickSDK_AdminQclGetCwAllowed(UInt32 p_deviceHandle, ref bool p_CwAllowed);
        [DllImport("SidekickSDK.dll", CallingConvention = CallingConvention.Cdecl)] public static extern UInt32 SidekickSDK_AdminQclGetFiltersInstalled(UInt32 p_deviceHandle, ref bool p_FiltersInstalled);
        [DllImport("SidekickSDK.dll", CallingConvention = CallingConvention.Cdecl)] public static extern UInt32 SidekickSDK_AdminQclGetLaserTimeMinutes(UInt32 p_deviceHandle, ref UInt32 p_LaserTimeMinutes);
        [DllImport("SidekickSDK.dll", CallingConvention = CallingConvention.Cdecl)] public static extern UInt32 SidekickSDK_AdminQclGetHeadType(UInt32 p_deviceHandle, ref byte headType);
	    [DllImport("SidekickSDK.dll", CallingConvention = CallingConvention.Cdecl)] public static extern UInt32 SidekickSDK_AdminQclGetHeadElectronics(UInt32 p_deviceHandle, ref byte headElectronics);
	    [DllImport("SidekickSDK.dll", CallingConvention = CallingConvention.Cdecl)] public static extern UInt32 SidekickSDK_AdminQclGetFixedWavelengthLaser(UInt32 p_deviceHandle, ref bool fixedWavelengthLaser );
    //End of Admin Command Functions

    ////////////////////////////////////////////////////////////
    //System Command Functions
    ////////////////////////////////////////////////////////////
	    [DllImport("SidekickSDK.dll", CallingConvention = CallingConvention.Cdecl)] public static extern UInt32 SidekickSDK_ReadWriteSysDisplayUnits(UInt32 p_deviceHandle, bool fWrite);    
	    [DllImport("SidekickSDK.dll", CallingConvention = CallingConvention.Cdecl)] public static extern UInt32 SidekickSDK_GetDisplayUnits(UInt32 p_deviceHandle, ref byte p_wwUnits, ref byte p_pulseRateUnits );
	    [DllImport("SidekickSDK.dll", CallingConvention = CallingConvention.Cdecl)] public static extern UInt32 SidekickSDK_SetDisplayUnits(UInt32 p_deviceHandle, byte p_wwUnits, byte p_pulseRateUnits );	
	    [DllImport("SidekickSDK.dll", CallingConvention = CallingConvention.Cdecl)] public static extern UInt32 SidekickSDK_ExecuteSaveStateOperation(UInt32 p_deviceHandle);
	    [DllImport("SidekickSDK.dll", CallingConvention = CallingConvention.Cdecl)] public static extern UInt32 SidekickSDK_SaveState(UInt32 p_deviceHandle, string newName, ref int newId );
	    [DllImport("SidekickSDK.dll", CallingConvention = CallingConvention.Cdecl)] public static extern UInt32 SidekickSDK_OverwriteState(UInt32 p_deviceHandle, string name, int id);
	    [DllImport("SidekickSDK.dll", CallingConvention = CallingConvention.Cdecl)] public static extern UInt32 SidekickSDK_ExecuteRestoreStateOperation(UInt32 p_deviceHandle);
	    [DllImport("SidekickSDK.dll", CallingConvention = CallingConvention.Cdecl)] public static extern UInt32 SidekickSDK_RecallState(UInt32 p_deviceHandle, int id);
	    [DllImport("SidekickSDK.dll", CallingConvention = CallingConvention.Cdecl)] public static extern UInt32 SidekickSDK_ExecuteEraseStateOperation(UInt32 p_deviceHandle);
	    [DllImport("SidekickSDK.dll", CallingConvention = CallingConvention.Cdecl)] public static extern UInt32 SidekickSDK_EraseState(UInt32 p_deviceHandle, int id);
	    [DllImport("SidekickSDK.dll", CallingConvention = CallingConvention.Cdecl)] public static extern UInt32 SidekickSDK_ReadNumSavedStates(UInt32 p_deviceHandle);	//Call once on startup
	    [DllImport("SidekickSDK.dll", CallingConvention = CallingConvention.Cdecl)] public static extern UInt32 SidekickSDK_ReadSavedStateName(UInt32 p_deviceHandle);	//Call once on startup
	    [DllImport("SidekickSDK.dll", CallingConvention = CallingConvention.Cdecl)] public static extern UInt32 SidekickSDK_ReadWriteIPAddressProvision(UInt32 p_deviceHandle, bool fWrite);
	    [DllImport("SidekickSDK.dll", CallingConvention = CallingConvention.Cdecl)] public static extern UInt32 SidekickSDK_GetProvisionIPAddr(UInt32 p_deviceHandle, ref UInt32 ipAddr, ref UInt32 subnetMask, ref UInt32 defGateway, ref UInt16 port, ref bool useDhcp );
	    [DllImport("SidekickSDK.dll", CallingConvention = CallingConvention.Cdecl)] public static extern UInt32 SidekickSDK_SetProvisionIPAddr(UInt32 p_deviceHandle, UInt32 ipAddr, UInt32 subnetMask, UInt32 defGateway, UInt16 port, bool useDhcp  );
        [DllImport("SidekickSDK.dll", CallingConvention = CallingConvention.Cdecl)] public static extern UInt32 SidekickSDK_ReadConfiguredProvisionIPAddr(UInt32 p_deviceHandle, ref UInt32 ipAddr, ref UInt32 subnetMask, ref UInt32 defGateway, ref UInt16 port, ref bool useDhcp);
        [DllImport("SidekickSDK.dll", CallingConvention = CallingConvention.Cdecl)] public static extern UInt32 SidekickSDK_ReadWriteDiscoveryIPAddressProvision(UInt32 p_deviceHandle, [MarshalAs(UnmanagedType.U1)] bool fWrite);
        [DllImport("SidekickSDK.dll", CallingConvention = CallingConvention.Cdecl)] public static extern UInt32 SidekickSDK_GetDiscoveryProvisionIPAddr(UInt32 p_deviceHandle, ref UInt32 ipAddr, ref UInt16 port);
        [DllImport("SidekickSDK.dll", CallingConvention = CallingConvention.Cdecl)] public static extern UInt32 SidekickSDK_SetDiscoveryProvisionIPAddr(UInt32 p_deviceHandle, UInt32 ipAddr, UInt16 port);
        [DllImport("SidekickSDK.dll", CallingConvention = CallingConvention.Cdecl)] public static extern UInt32 SidekickSDK_ReadWriteEthernetTimeout(UInt32 p_deviceHandle, [MarshalAs(UnmanagedType.U1)] bool fWrite);
        [DllImport("SidekickSDK.dll", CallingConvention = CallingConvention.Cdecl)] public static extern UInt32 SidekickSDK_SetEthernetTimeout(UInt32 p_deviceHandle, byte timeoutTens);
        [DllImport("SidekickSDK.dll", CallingConvention = CallingConvention.Cdecl)] public static extern UInt32 SidekickSDK_GetEthernetTimeout(UInt32 p_deviceHandle, ref byte timeoutTens);
        [DllImport("SidekickSDK.dll", CallingConvention = CallingConvention.Cdecl)] public static extern UInt32 SidekickSDK_ResetEthernet(UInt32 p_deviceHandle);
        [DllImport("SidekickSDK.dll", CallingConvention = CallingConvention.Cdecl)] public static extern UInt32 SidekickSDK_ReadWriteAudioNotificationPreference(UInt32 p_deviceHandle, bool fWrite);
        [DllImport("SidekickSDK.dll", CallingConvention = CallingConvention.Cdecl)] public static extern UInt32 SidekickSDK_GetAudioNotificationPreferences(UInt32 p_deviceHandle, [MarshalAs(UnmanagedType.U1)] ref bool audioOn, [MarshalAs(UnmanagedType.U1)] ref bool flashLed);
        [DllImport("SidekickSDK.dll", CallingConvention = CallingConvention.Cdecl)] public static extern UInt32 SidekickSDK_SetAudioNotificationPreferences(UInt32 p_deviceHandle, [MarshalAs(UnmanagedType.U1)] bool audioOn, [MarshalAs(UnmanagedType.U1)] bool flashLed);
	    [DllImport("SidekickSDK.dll", CallingConvention = CallingConvention.Cdecl)] public static extern UInt32 SidekickSDK_ReadWriteSysGlobalPreferences(UInt32 p_deviceHandle, bool fWrite);
	    [DllImport("SidekickSDK.dll", CallingConvention = CallingConvention.Cdecl)] public static extern UInt32 SidekickSDK_GetSysGlobalPreferences(UInt32 p_deviceHandle, ref bool alwaysUseCrossover );
	    [DllImport("SidekickSDK.dll", CallingConvention = CallingConvention.Cdecl)] public static extern UInt32 SidekickSDK_SetSysGlobalPreferences(UInt32 p_deviceHandle, bool alwaysUseCrossover );
	    

    
    }
}
