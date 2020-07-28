using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;
using Sidekick_Control;

namespace Sidekick_Control
{
    public class SidekickController : IDisposable
    {
        public static ushort SearchForUsbDevices()
        {
            ushort[] numberOfDevices = null;
            SDKConstants errCode = (SDKConstants)Sidekick_Control.SidekickSDK.SidekickSDK_SearchForUsbDevices();
            if(Sidekick_Control.SDKConstants.SIDEKICK_SDK_RET_SUCCESS == errCode)
            {
                numberOfDevices = GetNumberOfDevices();
            }
            return numberOfDevices[1];
        }

        public static ushort SearchForTcpDevices(string ipAddress, ushort port)
        {
            ushort[] numberOfDevices = null;
            SDKConstants errCode = (SDKConstants)Sidekick_Control.SidekickSDK.SidekickSDK_SearchForTCPDevices(new StringBuilder(ipAddress), port);
            if (Sidekick_Control.SDKConstants.SIDEKICK_SDK_RET_SUCCESS == errCode)
            {
                numberOfDevices = GetNumberOfDevices();
            }
            return numberOfDevices[2];
        }

        public static void ClearDeviceList()
        {
            Sidekick_Control.SidekickSDK.SidekickSDK_ClearDeviceList();
        }

        public static ushort[] GetNumberOfDevices()
        {
            var numberOfDevices = new ushort[3];
            Sidekick_Control.SidekickSDK.SidekickSDK_GetNumOfDevices(ref numberOfDevices[0]);
            Sidekick_Control.SidekickSDK.SidekickSDK_GetNumUsbDevices(ref numberOfDevices[1]);
            Sidekick_Control.SidekickSDK.SidekickSDK_GetNumTcpDevices(ref numberOfDevices[2]);

            return numberOfDevices;
        }

        public static List<DLS_SCI_DEVICE_INFO> GetDeviceInfo()
        {
            var returnList = new List<DLS_SCI_DEVICE_INFO>();
            var numberOfDevices = SidekickController.GetNumberOfDevices();
            DLS_SCI_DEVICE_INFO deviceInfo;
            for (ushort i=0; i < numberOfDevices[0]; i++)
            {
                deviceInfo = new DLS_SCI_DEVICE_INFO();
                Sidekick_Control.SidekickSDK.SidekickSDK_GetDeviceInfo(i, ref deviceInfo);
                returnList.Add(deviceInfo);
            }

            return returnList;
        }

        public static SidekickController Connect(ushort deviceNumber)
        {
            uint deviceHandle = 0;
            SidekickController returnController = null;
            SDKConstants errCode = (SDKConstants)Sidekick_Control.SidekickSDK.SidekickSDK_ConnectToDeviceNumber(ref deviceHandle, deviceNumber);
            if (Sidekick_Control.SDKConstants.SIDEKICK_SDK_RET_SUCCESS == errCode)
            {
                returnController = new SidekickController(deviceHandle);
            }

            return returnController;
        }
        
        private SidekickController(uint deviceHandle)
        {
            this.handle = deviceHandle;
        }

        public Tuple<uint, uint, uint, uint> GetFirmwareVersions()
        {
            byte major = 0, minor = 0, patch = 0, pcb = 0;
            Tuple<uint, uint, uint, uint> version = null;
            SDKConstants errCode = (SDKConstants)Sidekick_Control.SidekickSDK.SidekickSDK_ReadAdminFirmwareVersion(handle);
            
            if (SDKConstants.SIDEKICK_SDK_RET_SUCCESS == errCode)
            {
                errCode = (SDKConstants)Sidekick_Control.SidekickSDK.SidekickSDK_GetAdminFirmwareVersions(handle, ref major, ref minor, ref patch, ref pcb);
                version = new Tuple<uint, uint, uint, uint>(major, minor, patch, pcb);
            }
            else HandleErrorCode(errCode);

            return version;
        }

        public List<UInt32> ReadStatusMask()
        {
            List<UInt32> retList = null;

            UInt32 StatusMask = 0;
            UInt16 SysErrorWord = 0, SysWarningWord = 0;
            SDKConstants errCode = (SDKConstants)SidekickSDK.SidekickSDK_ReadStatusMask(this.handle, ref StatusMask, ref SysErrorWord, ref SysWarningWord);
            if (SDKConstants.SIDEKICK_SDK_RET_SUCCESS == errCode)
            {
                retList = new List<UInt32>();
                retList.Add(StatusMask);
                retList.Add(SysErrorWord);
                retList.Add(SysWarningWord);
            }
            else HandleErrorCode(errCode);

            return retList;
        }

        public List<float> GetInfoSysTemperatures()
        {
            List<float> retList = null;
            SDKConstants errCode = (SDKConstants)Sidekick_Control.SidekickSDK.SidekickSDK_ReadInfoSysTemperatures(handle);
            if (SDKConstants.SIDEKICK_SDK_RET_SUCCESS == errCode)
            {
                float p_temp1 = 0, p_temp2 = 0, p_temp3 = 0, p_humidity1 = 0, p_humidity2 = 0, p_aux_temp1 = 0, p_aux_temp2 = 0;
                if ((uint)SDKConstants.SIDEKICK_SDK_RET_SUCCESS ==
                        Sidekick_Control.SidekickSDK.SidekickSDK_GetInfoSysTemperatures(handle, ref p_temp1, ref p_temp2, ref p_temp3,
                        ref p_humidity1, ref p_humidity2,
                        ref p_aux_temp1, ref p_aux_temp2))
                {
                    retList = new List<float>();
                    retList.Add(p_temp1);
                    retList.Add(p_temp2);
                    retList.Add(p_temp3);
                    retList.Add(p_humidity1);
                    retList.Add(p_humidity2);
                    retList.Add(p_aux_temp1);
                    retList.Add(p_aux_temp2);
                }
            }
            else HandleErrorCode(errCode);

            return retList;
        }

        public List<float> GetInfoQclInfo()
        {
            List<float> retList = null;
            byte p_channel = 0, p_status = 0;
            float p_qclVoltage = 0, p_qclVfet = 0, p_qclVsrc = 0, p_qclCurrent = 0, p_iSns = 0, p_currentSrc = 0, p_qclTemp = 0, p_vQcHi = 0;

            System.Diagnostics.Debug.WriteLine("ReadInfoQclInfo({0})", 1);
            SDKConstants errCode = (SDKConstants)Sidekick_Control.SidekickSDK.SidekickSDK_ReadInfoQclInfo(this.handle, 1);
            if (SDKConstants.SIDEKICK_SDK_RET_SUCCESS == errCode)
            {
                System.Diagnostics.Debug.WriteLine(String.Format("ReadInfoQclInfo({0}, {1}, {2}, {3}, {4}, {5}, {6}, {7}, {8} {9})",
                    p_channel, p_status, p_qclVoltage, p_qclVfet, p_qclVsrc, p_qclCurrent, p_iSns, p_currentSrc, p_qclTemp, p_vQcHi));
                if ((uint)SDKConstants.SIDEKICK_SDK_RET_SUCCESS == Sidekick_Control.SidekickSDK.SidekickSDK_GetInfoQclInfo(
                    this.handle, ref p_channel, ref p_status, ref p_qclVoltage, ref p_qclVfet, ref p_qclVsrc,
                    ref p_qclCurrent, ref p_iSns, ref p_currentSrc, ref p_qclTemp, ref p_vQcHi))
                {
                    retList = new List<float>();
                    retList.Add((float)p_channel);
                    retList.Add((float)p_status);
                    retList.Add(p_qclVoltage);
                    retList.Add(p_qclVfet);
                    retList.Add(p_qclVsrc);
                    retList.Add(p_qclCurrent);
                    retList.Add(p_iSns);
                    retList.Add(p_currentSrc);
                    retList.Add(p_qclTemp);
                    retList.Add(p_vQcHi);

                }
            }
            else HandleErrorCode(errCode);

            return retList;
        }

        public Dictionary<Sidekick_Control.SystemVoltageIndex, float> GetInfoSysVoltages()
        {
            Dictionary<Sidekick_Control.SystemVoltageIndex, float> retList = null;
            SDKConstants errCode = (SDKConstants)Sidekick_Control.SidekickSDK.SidekickSDK_ReadInfoSysVoltages(handle);
            if (SDKConstants.SIDEKICK_SDK_RET_SUCCESS == errCode)
            {
                retList = new Dictionary<Sidekick_Control.SystemVoltageIndex, float>();

                foreach (Sidekick_Control.SystemVoltageIndex channel in Enum.GetValues(typeof(Sidekick_Control.SystemVoltageIndex)))
                {
                    float voltage = float.NaN;
                    Sidekick_Control.SidekickSDK.SidekickSDK_GetInfoSysVoltages(this.handle, (byte)channel, ref voltage);
                    retList.Add(channel, voltage);
                }
            }
            else HandleErrorCode(errCode);

            return retList;
        }

        public float GetInfoSysVoltageChannel(Sidekick_Control.SystemVoltageIndex channel)
        {
            SDKConstants errCode = (SDKConstants)Sidekick_Control.SidekickSDK.SidekickSDK_ReadInfoSysVoltages(handle);
            float voltage = float.NaN;
            if (SDKConstants.SIDEKICK_SDK_RET_SUCCESS == errCode)
            {
                Sidekick_Control.SidekickSDK.SidekickSDK_GetInfoSysVoltages(this.handle, (byte)channel, ref voltage);
            }
            else HandleErrorCode(errCode);

            return voltage;
        }
        
        public List<double> GetLaserQclVsrcParams(byte qcl_slot)
        {
            List<double> retList = null;
            SDKConstants errCode = (SDKConstants)SidekickSDK.SidekickSDK_ReadWriteLaserQclVsrcParams(this.handle, false, qcl_slot);
            if (SDKConstants.SIDEKICK_SDK_RET_SUCCESS == errCode)
            {
                float vsrc = 0, vsrcMaxPulsed = 0, vsrcMaxCw = 0;
                if((uint)SDKConstants.SIDEKICK_SDK_RET_SUCCESS == SidekickSDK.SidekickSDK_GetLaserQclVsrcParams(this.handle, ref qcl_slot,
                    ref vsrc, ref vsrcMaxPulsed, ref vsrcMaxCw))
                {
                    retList = new List<double>();
                    retList.Add((double)qcl_slot);
                    retList.Add(vsrc);
                    retList.Add(vsrcMaxPulsed);
                    retList.Add(vsrcMaxCw);
                }
            }
            else HandleErrorCode(errCode);

            return retList;
        }
        
        public List<double> GetLaserQclParams(byte qcl_slot)
        {
            List<double> retList = null;
            UInt32 pulseRateHz = 0, pulseWidthNs = 0;
            UInt16 currentMa = 0;
			float temperature = 0;
            byte laserMode = 0, pulseMode = 0;
            float vsrc = 0;

            SDKConstants errCode = (SDKConstants)SidekickSDK.SidekickSDK_ReadWriteLaserQclParams(this.handle, false, qcl_slot);
            if (SDKConstants.SIDEKICK_SDK_RET_SUCCESS == errCode)
            {
                if ((uint)SDKConstants.SIDEKICK_SDK_RET_SUCCESS == SidekickSDK.SidekickSDK_GetLaserQclParams(this.handle, ref qcl_slot,
                    ref pulseRateHz, ref pulseWidthNs, ref currentMa,
                    ref temperature, ref laserMode, ref pulseMode, ref vsrc))
                {
                    retList = new List<double>();
                    retList.Add(qcl_slot);
                    retList.Add(pulseRateHz);
                    retList.Add(pulseWidthNs);
                    retList.Add(currentMa);
                    retList.Add(temperature);
                    retList.Add(laserMode);
                    retList.Add(pulseMode);
                    retList.Add(vsrc);
                }
            }
            else HandleErrorCode(errCode);

            return retList;
        }
        
        public bool ReadAdminSysParams()
        {
            bool retVal = false;
            SDKConstants errCode = (SDKConstants)SidekickSDK.SidekickSDK_ReadAdminSysParams(this.handle);
            if (SDKConstants.SIDEKICK_SDK_RET_SUCCESS == errCode) retVal = true;
            else HandleErrorCode(errCode);
            return  retVal;
        }

        public bool IsInterlockedStatusSet()
        {
            bool retVal = false;
            SDKConstants errCode = (SDKConstants)SidekickSDK.SidekickSDK_isInterlockedStatusSet(this.handle, ref retVal);
            if (SDKConstants.SIDEKICK_SDK_RET_SUCCESS != errCode) HandleErrorCode(errCode);
            return retVal;
        }

        public bool IsLaserArmed()
        {
            bool retVal = false;
            SDKConstants errCode = (SDKConstants)SidekickSDK.SidekickSDK_isLaserArmed(this.handle, ref retVal);
            if (SDKConstants.SIDEKICK_SDK_RET_SUCCESS != errCode) HandleErrorCode(errCode);
            return retVal;
        }

        public bool IsKeySwitchStatusSet()
        {
            bool retVal = false;
            SDKConstants errCode = (SDKConstants)SidekickSDK.SidekickSDK_isKeySwitchStatusSet(this.handle, ref retVal);
            if (SDKConstants.SIDEKICK_SDK_RET_SUCCESS != errCode) HandleErrorCode(errCode);
            return retVal;
        }

        public bool IsHomingCompleted()
        {
            bool retVal = false;
            SDKConstants errCode = (SDKConstants)SidekickSDK.SidekickSDK_isHomingCompleted(this.handle, ref retVal);
            if (SDKConstants.SIDEKICK_SDK_RET_SUCCESS != errCode) HandleErrorCode(errCode);
            return retVal;
        }

        public string AdminGetSerialNumStr()
        {
            StringBuilder retStr = new StringBuilder((ushort)StringLengths.SIDEKICK_SDK_STRLEN_MAX_SERIAL_NUM);
            byte strLen = 0;
            SidekickSDK.SidekickSDK_AdminGetSerialNumStr(this.handle, retStr, ref strLen);
            return retStr.ToString();
        }
        /*
        public bool SetAndWriteAudioNotificationPreferences(bool audioOn, bool flashLED)
        {
            bool prefSetSuccess = false;

            SDKConstants errCode = (SDKConstants)SidekickSDK.SidekickSDK_SetAudioNotificationPreferences(this.handle, audioOn, flashLED);
            if (SDKConstants.SIDEKICK_SDK_RET_SUCCESS == errCode)
            {
                errCode = (SDKConstants)SidekickSDK.SidekickSDK_ReadWriteAudioNotificationPreference(this.handle, true);
                if (SDKConstants.SIDEKICK_SDK_RET_SUCCESS == errCode) prefSetSuccess = true;
            }

            return prefSetSuccess;
        }
         */

        public bool ArmLaser()
        {
            return ExecLaserArmDisarm(true);
        }

        public bool DisarmLaser()
        {
            return ExecLaserArmDisarm(false);
        }

        public bool ExecLaserArmDisarm(bool arm)
        {
            bool armDisarmSuccess = false;
            System.Diagnostics.Debug.WriteLine(String.Format("C# SetLaserArmDisarm({0})", arm));
            SDKConstants errCode = (SDKConstants)SidekickSDK.SidekickSDK_SetLaserArmDisarm(this.handle, arm);
            if (errCode == SDKConstants.SIDEKICK_SDK_RET_SUCCESS)
            {
                System.Diagnostics.Debug.WriteLine(String.Format("C# ExecLaserArmDisarm())"));
                errCode = (SDKConstants)SidekickSDK.SidekickSDK_ExecLaserArmDisarm(this.handle);
                if (errCode == SDKConstants.SIDEKICK_SDK_RET_SUCCESS) armDisarmSuccess = true;
            }

            HandleErrorCode(errCode);

            return armDisarmSuccess;
        }

        public uint? AdminGetNumQcls()
        {
            byte numOfQcls = 0;
            uint? retVal = null;

            if(ReadAdminSysParams())
            {
                Sidekick_Control.SidekickSDK.SidekickSDK_AdminGetNumQcls(this.handle, ref numOfQcls);
                retVal = (uint)numOfQcls;
            }

            return retVal;
        }

        public bool ReadAdminQclParams(byte qclSlot)
        {
            bool retVal = false;;
            SDKConstants errCode = (SDKConstants)SidekickSDK.SidekickSDK_ReadAdminQclParams(this.handle, qclSlot);
            if (SDKConstants.SIDEKICK_SDK_RET_SUCCESS == errCode) retVal = true;
            else HandleErrorCode(errCode);
            return retVal;
        }

        public LaserType? AdminQclGetHeadType()
        {
            byte head = 0;
            LaserType? retVal = null;
            SDKConstants errCode = (SDKConstants)SidekickSDK.SidekickSDK_AdminQclGetHeadType(this.handle, ref head);
            if ((uint)SDKConstants.SIDEKICK_SDK_RET_SUCCESS == errCode)
            {
                retVal = (LaserType)head;
            }
            else HandleErrorCode(errCode);

            return retVal;
        }

        public List<double> AdminGetWavelengthLimits()
        {
            float lowLimit = 0, highLimit = 0;
            byte units = 0;
            List<double> retList = null;
            if(
                (ReadAdminSysParams()) &&
                (ReadAdminQclParams(1))
            )
            {   
                LaserType? head = AdminQclGetHeadType();
                if (null != head)
                {
                    if (LaserType.SIDEKICK_SDK_LASER_TYPE_GEN1_PULSE == head)
                    {
                        Sidekick_Control.SidekickSDK.SidekickSDK_AdminQclGetWavelengthLimits(this.handle, ref lowLimit, ref highLimit, ref units);
                    }
                    else
                    {
                        Sidekick_Control.SidekickSDK.SidekickSDK_AdminQclGetMhfWavelengthLimits(this.handle, ref lowLimit, ref highLimit, ref units);
                    }
                    retList = new List<double>();
                    retList.Add(lowLimit);
                    retList.Add(highLimit);
                    retList.Add(units);
                }
            }
            return retList;
        }

        public bool SetSweepParams(Units limitUnits, double startWW, double stopWW, double speed, ushort numberOfScans)
        {
            bool retVal = false;
            SDKConstants errCode = (SDKConstants)Sidekick_Control.SidekickSDK.SidekickSDK_SetSweepParams(this.handle, (byte)limitUnits, (float)startWW, (float)stopWW, (float)speed,
                        numberOfScans, 0, 0);
            if (SDKConstants.SIDEKICK_SDK_RET_SUCCESS == errCode)
            {
                if (WriteSweepParams()) retVal = true;
            }
            else HandleErrorCode(errCode);

            return retVal;
        }
        
        public bool ReadSweepParams()
        {
            return this.ReadWriteSweepParams(false);
        }

        public bool WriteSweepParams()
        {
            return this.ReadWriteSweepParams(true);
        }

        private bool ReadWriteSweepParams(bool write)
        {
            bool retVal = false;

            SDKConstants errCode = (SDKConstants)Sidekick_Control.SidekickSDK.SidekickSDK_ReadWriteSweepParams(this.handle, write);
            if (SDKConstants.SIDEKICK_SDK_RET_SUCCESS == errCode) retVal = true;
            else HandleErrorCode(errCode);
            return retVal;
        }

        public bool SetAndExecScanOperation(ScanOps scanOp)
        {
            bool ret = false;

            SDKConstants errCode = (SDKConstants)SidekickSDK.SidekickSDK_SetScanOperation(this.handle, (byte)scanOp);
            if (SDKConstants.SIDEKICK_SDK_RET_SUCCESS == errCode)
            {
                if ((uint)SDKConstants.SIDEKICK_SDK_RET_SUCCESS == SidekickSDK.SidekickSDK_ExecuteScanOperation(this.handle)) ret = true;
            }
            else HandleErrorCode(errCode);

            return ret;
        }

        public List<UInt16> GetScanProgress()
        {
            List<UInt16> retList = null;

            SDKConstants errCode = (SDKConstants)SidekickSDK.SidekickSDK_ReadScanProgress(this.handle);
            if (SDKConstants.SIDEKICK_SDK_RET_SUCCESS == errCode)
            {
                retList = new List<UInt16>();
                byte scanProgressMask = 0;
                ushort scanNum = 0, scanPercent = 0;
                SidekickSDK.SidekickSDK_GetScanProgress(this.handle, ref scanProgressMask, ref scanNum, ref scanPercent);
                retList.Add(scanProgressMask);
                retList.Add(scanNum);
                retList.Add(scanPercent);
            }
            else HandleErrorCode(errCode);

            return retList;
        }

        public class MfgStrings
        {
            public readonly string SerialNumber;
            public readonly string ModelNumber;
            public readonly string ManufactureDate;

            public MfgStrings(string serial, string model, string manufacture)
            {
                SerialNumber = serial;
                ModelNumber = model;
                ManufactureDate = manufacture;
            }
        }

        public MfgStrings GetManufacturingStrings()
        {
            MfgStrings retVal = null;
            var errCode = (SDKConstants)SidekickSDK.SidekickSDK_ReadAdminSysParams(handle);
            if (errCode == SDKConstants.SIDEKICK_SDK_RET_SUCCESS)
            {
                byte length = 0;
                var serialNumber = new StringBuilder();
                var manufactureDate = new StringBuilder();
                var modelNumber = new StringBuilder();

                SidekickSDK.SidekickSDK_AdminGetSerialNumStr(handle, serialNumber, ref length);
                SidekickSDK.SidekickSDK_AdminGetModelNumStr(handle, modelNumber, ref length);
                SidekickSDK.SidekickSDK_AdminGetMfgDateStr(handle, manufactureDate, ref length);

                retVal = new MfgStrings(serialNumber.ToString(), modelNumber.ToString(), manufactureDate.ToString());
            }
            else HandleErrorCode(errCode);

            return retVal;
        }
  

        public void ExecShutdownLaser(byte type, byte reason, UInt32 delay)
        {
            SidekickSDK.SidekickSDK_ExecShutdownLaser(handle, type, reason, delay);
        }

        public void HandleErrorCode(SDKConstants errorCode)
        {
            if (SDKConstants.SIDEKICK_SDK_RET_SUCCESS == errorCode)
            {
                // no op
            }
            else
            {
                string errorDescription = System.Runtime.InteropServices.Marshal.PtrToStringAnsi(SidekickSDK.SidekickSDK_GetCommErrDesc((uint)errorCode));
                throw new System.InvalidOperationException(errorDescription);
            }
        }

/****************************************************************************************************************************/

        // Call this function when you are done with the object to release underlying unmanaged resources.
        public void Dispose()
        {
            Dispose(true);
            GC.SuppressFinalize(this);
        }

        protected virtual void Dispose(bool disposing)
        {
            if(disposing)
            {
                Sidekick_Control.SidekickSDK.SidekickSDK_Disconnect(this.handle);
            }
        }

        public readonly uint handle;
    }
}
