// COPYRIGHT 2020 by the Open Rails project.
// 
// This file is part of Open Rails.
// 
// Open Rails is free software: you can redistribute it and/or modify
// it under the terms of the GNU General Public License as published by
// the Free Software Foundation, either version 3 of the License, or
// (at your option) any later version.
// 
// Open Rails is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU General Public License for more details.
// 
// You should have received a copy of the GNU General Public License
// along with Open Rails.  If not, see <http://www.gnu.org/licenses/>.

using Orts.Simulation;
using Orts.Simulation.RollingStocks;
using ORTS.Common;
using ORTS.Scripting.Api;
using System;
using System.Collections.Generic;
using System.Globalization;
using System.IO;
using System.Linq;

namespace ORTS.Scripting.Script
{
    public class TCS_France : TrainControlSystem
    {
        // Helper functions
        public static T Min<T>(T a, T b) where T : IComparable
        {
            return a.CompareTo(b) <= 0 ? a : b;
        }

        public static T Max<T>(T a, T b) where T : IComparable
        {
            return a.CompareTo(b) >= 0 ? a : b;
        }

        // Cabview control number
        const int BP_AC_SF = 0;
        const int BP_A_LS_SF = 2;
        const int Z_ES_VA = 3;
        const int BP_AM_V1 = 9;
        const int BP_AM_V2 = 10;
        const int BP_DM = 11;
        const int VY_CV = 23;
        const int VY_SECT = 24;
        const int VY_SECT_AU = 25;
        const int VY_BPT = 26;
        const int TVM_VL = 27;
        const int TVM_Ex1 = 28;
        const int TVM_Ex2 = 29;
        const int TVM_An1 = 30;
        const int TVM_An2 = 31;
        const int LS_SF = 32;
        const int VY_SOS_RSO = 33;
        const int VY_SOS_VAC = 34;
        const int VY_ES_FU = 35;
        const int VY_SOS_KVB = 36;
        const int VY_VTE = 37;
        const int VY_FU = 38;
        const int KVB_Principal1 = 41;
        const int KVB_Principal2 = 42;
        const int KVB_Auxiliary = 43;
        const int TVM_Mask = 47;

        enum ETCSLevel
        {
            L0,         // Unfitted (national system active)
            NTC,        // Specific Transmission Module (national system information transmitted to ETCS)
            L1,         // Level 1 : Beacon transmission, loop transmission and radio in-fill
            L2,         // Level 2 : Radio transmission, beacon positionning
            L3          // Level 3 : Same as level 2 + moving block
        }

        enum RSOStateType
        {
            Init,
            Off,
            TriggeredPressed,
            TriggeredBlinking,
            TriggeredFixed
        }

        enum KVBStateType
        {
            Normal,
            Alert,
            Emergency
        }

        enum KVBPreAnnounceType
        {
            Deactivated,
            Armed,
            Triggered
        }

        enum KVBModeType
        {
            ConventionalLine,
            HighSpeedLine,
            Shunting
        }

        enum KVBReleaseSpeed
        {
            V30,
            V10
        }

        enum KVBPrincipalDisplayStateType
        {
            Empty,
            FU,
            V000,
            V00,
            L,
            b,
            p,
            Dashes3,
            Dashes9,
            Test
        }

        enum KVBAuxiliaryDisplayStateType
        {
            Empty,
            V000,
            V00,
            L,
            p,
            Dashes3,
            Test
        }

        enum QBalType
        {
            LC,
            LGV,
        }

        public enum TVMModelType
        {
            None,
            TVM300,
            TVM430_V300,
            TVM430_V320
        }

        public enum TVMSpeedType
        {
            _RRR,
            _000,
            _30E,
            _30,
            _60E,
            _60,
            _80E,
            _80,
            _100E,
            _100,
            _130E,
            _130,
            _160E,
            _160,
            _170E,
            _170,
            _200V,
            _200,
            _220E,
            _220V,
            _220,
            _230E,
            _230V,
            _230,
            _270V,
            _270,
            _300V,
            _300,
            _320V,
            _320,
            Any
        }

        public enum TVMAspectType
        {
            None,
            _RRR,
            _000,
            _30E,
            _30A,
            _60E,
            _60A,
            _80E,
            _80A,
            _100E,
            _100A,
            _130E,
            _130A,
            _160E,
            _160A,
            _170E,
            _170A,
            _200V,
            _200A,
            _220E,
            _220V,
            _220A,
            _230E,
            _230V,
            _230A,
            _270V,
            _270A,
            _300V,
            _300A,
            _320V
        }

        ETCSLevel CurrentETCSLevel = ETCSLevel.L0;

        // Properties
        bool RearmingButton { get; set; } = false;

    // Train parameters
        bool VACMAPresent;                                  // VACMA
        bool RSOPresent;                                    // RSO
        bool DAATPresent;                                   // DAAT
        bool KVBPresent;                                    // KVB
        bool TVM300Present;                                 // TVM300
        bool TVM430Present;                                 // TVM430
        bool ETCSPresent;                                   // ETCS (Not implemented)
        ETCSLevel ETCSMaxLevel = ETCSLevel.L0;              // ETCS maximum level (Not implemented)
        bool ElectroPneumaticBrake;                         // EP
        bool HeavyFreightTrain;                             // MA train only
        float SafeDecelerationMpS2;                         // Gamma

    // RSO (Répétition Optique des Signaux / Optical Signal Repetition)
        // Parameters
        float RSODelayBeforeEmergencyBrakingS;
        float RSOBlinkerFrequencyHz;

        // Variables
        RSOStateType RSOState = RSOStateType.Init;
        Aspect RSOLastSignalAspect = Aspect.Clear_1;
        bool RSOEmergencyBraking = true;
        bool RSOPressed = false;
        bool RSOPreviousPressed = false;
        bool RSOCancelPressed = false;
        bool RSOType1Inhibition = false;                    // Inhibition 1 : Reverse
        bool RSOType2Inhibition = false;                    // Inhibition 2 : KVB not inhibited and train on HSL
        bool RSOType3Inhibition = false;                    // Inhibition 3 : TVM COVIT not inhibited
        bool RSOClosedSignal = false;
        bool RSOPreviousClosedSignal = false;
        bool RSOOpenedSignal = false;
        Blinker RSOBlinker;
        Timer RSOEmergencyTimer;

    // DAAT (Dispositif d'Arrêt Automatique des Trains / Automatic Train Stop System)
        // Not implemented

    // KVB (Contrôle de Vitesse par Balises / Beacon speed control)
        // Parameters
        bool KVBInhibited;
        const float KVBDelayBeforeEmergencyBrakingS = 5f;   // Tx
        float KVBTrainSpeedLimitMpS;                        // VT
        float KVBTrainLengthM;                              // L
        float KVBDelayBeforeBrakingEstablishedS;            // Tbo

        // Variables
        bool KVBInit = true;
        bool KVBSpadEmergency = false;
        bool KVBOverspeedEmergency = false;
        KVBStateType KVBState = KVBStateType.Emergency;
        bool KVBEmergencyBraking = true;
        KVBPreAnnounceType KVBPreAnnounce = KVBPreAnnounceType.Deactivated;
        KVBModeType KVBMode = KVBModeType.ConventionalLine;
        KVBPrincipalDisplayStateType KVBPrincipalDisplayState = KVBPrincipalDisplayStateType.Empty;
        bool KVBPrincipalDisplayBlinking = false;
        KVBAuxiliaryDisplayStateType KVBAuxiliaryDisplayState = KVBAuxiliaryDisplayStateType.Empty;

        Blinker KVBPrincipalDisplayBlinker;
        OdoMeter KVBInitOdometer;

        Aspect KVBLastSignalAspect = Aspect.Clear_1;
        float KVBLastSignalSpeedLimitMpS = float.PositiveInfinity;

        int KVBStopTargetSignalNumber = -1;
        float KVBStopTargetDistanceM = float.PositiveInfinity;
        KVBReleaseSpeed KVBStopTargetReleaseSpeed = KVBReleaseSpeed.V30;
        bool KVBOnSight = false;

        int KVBSpeedRestrictionTargetSignalNumber = -1;
        float KVBSpeedRestrictionTargetDistanceM = float.PositiveInfinity;
        float KVBSpeedRestrictionTargetSpeedMpS = float.PositiveInfinity;

        float KVBDeclivity = 0f;                            // i

        float KVBCurrentLineSpeedLimitMpS = float.PositiveInfinity;
        float KVBNextLineSpeedLimitMpS = float.PositiveInfinity;
        float KVBNextLineSpeedDistanceM = float.PositiveInfinity;

        bool KVBSpeedTooHighLight = false;
        bool KVBEmergencyBrakeLight = false;

        // TVM arming check
        OdoMeter KarmStartOdometer;
        bool KarmEmergencyBraking = false;
        QBalType QBal = QBalType.LC;    // Q-BAL

    // TVM COVIT common
        // Constants
        const int TVMNumberOfBlockSections = 10;

        // Parameters
        TVMModelType TVMModel = TVMModelType.None;
        bool TVMCOVITInhibited = false;

        // Variables
        bool TVMArmed = false;
        bool TVMCOVITEmergencyBraking = false;
        bool TVMOpenCircuitBreaker = false;
        bool TVMOpenCircuitBreakerAutomatic = false;
        bool TVMLowerPantograph = false;

        int[] SpeedSequence = new int[TVMNumberOfBlockSections];
        Aspect[] AspectSequence = new Aspect[TVMNumberOfBlockSections];
        int PreviousSectionSpeed = 0;
        Aspect PreviousSectionAspect = Aspect.None;
        TVMSpeedType PreviousVcond = TVMSpeedType.Any;

        TVMSpeedType[] Vcond = new TVMSpeedType[TVMNumberOfBlockSections];
        TVMSpeedType[] Ve = new TVMSpeedType[TVMNumberOfBlockSections];
        TVMSpeedType[] Vc = new TVMSpeedType[TVMNumberOfBlockSections];
        TVMSpeedType[] Va = new TVMSpeedType[TVMNumberOfBlockSections];

        TVMAspectType TVMAspectCommand = TVMAspectType.None;
        TVMAspectType TVMAspectCurrent = TVMAspectType.None;
        TVMAspectType TVMAspectPreviousCycle = TVMAspectType.None;
        bool TVMBlinkingCommand = false;
        bool TVMBlinkingCurrent = false;
        bool TVMBlinkingPreviousCycle = false;
        Blinker TVMBlinker;

        float TVMStartControlSpeedMpS = 0f;
        float TVMEndControlSpeedMpS = 0f;
        float TVMDecelerationMpS2 = 0f;

        bool TVMClosedSignal;
        bool TVMPreviousClosedSignal;
        bool TVMOpenedSignal;
        bool TVMPreviousOpenedSignal;

    // TVM300 COVIT (Transmission Voie Machine 300 COntrôle de VITesse / Track Machine Transmission 300 Speed control)
        // Constants
        string TVM300DecodingFileName;
        Dictionary<Tuple<TVMSpeedType, TVMSpeedType, TVMSpeedType>, Tuple<TVMAspectType, bool, float>> TVM300DecodingTable = new Dictionary<Tuple<TVMSpeedType, TVMSpeedType, TVMSpeedType>, Tuple<TVMAspectType, bool, float>>();

        Dictionary<TVMAspectType, Aspect> TVM300MstsTranslation = new Dictionary<TVMAspectType, Aspect>
        {
            { TVMAspectType.None, Aspect.None  },
            { TVMAspectType._300V, Aspect.Clear_2 },
            { TVMAspectType._270A, Aspect.Clear_1  },
            { TVMAspectType._270V, Aspect.Approach_3 },
            { TVMAspectType._220A, Aspect.Approach_2 },
            { TVMAspectType._220E, Aspect.Approach_1 },
            { TVMAspectType._160A, Aspect.Restricted },
            { TVMAspectType._160E, Aspect.StopAndProceed },
            { TVMAspectType._80A, Aspect.Restricted },
            { TVMAspectType._80E, Aspect.StopAndProceed },
            { TVMAspectType._000, Aspect.Stop },
            { TVMAspectType._RRR, Aspect.Permission }
        };

        Dictionary<TVMSpeedType, TVMSpeedType> TVM300Tab1 = new Dictionary<TVMSpeedType, TVMSpeedType>
        {
            { TVMSpeedType._RRR,   TVMSpeedType._000 },
            { TVMSpeedType._000,  TVMSpeedType._160 },
            { TVMSpeedType._80E,  TVMSpeedType._80 },
            { TVMSpeedType._80,   TVMSpeedType._160 },
            { TVMSpeedType._160E, TVMSpeedType._160 },
            { TVMSpeedType._160,  TVMSpeedType._220 },
            { TVMSpeedType._220E, TVMSpeedType._220 },
            { TVMSpeedType._220,  TVMSpeedType._270 },
            { TVMSpeedType._270V, TVMSpeedType._270 },
            { TVMSpeedType._270,  TVMSpeedType._300 },
            { TVMSpeedType._300V, TVMSpeedType._300 },
            { TVMSpeedType._300,  TVMSpeedType._000 }
        };
        Dictionary<TVMSpeedType, TVMSpeedType> TVM300Tab2 = new Dictionary<TVMSpeedType, TVMSpeedType>
        {
            { TVMSpeedType._RRR,   TVMSpeedType._000 },
            { TVMSpeedType._000,  TVMSpeedType._000 },
            { TVMSpeedType._80E,  TVMSpeedType._80 },
            { TVMSpeedType._80,   TVMSpeedType._80 },
            { TVMSpeedType._160E, TVMSpeedType._160 },
            { TVMSpeedType._160,  TVMSpeedType._160 },
            { TVMSpeedType._220E, TVMSpeedType._220 },
            { TVMSpeedType._220,  TVMSpeedType._220 },
            { TVMSpeedType._270V, TVMSpeedType._270 },
            { TVMSpeedType._270,  TVMSpeedType._270 },
            { TVMSpeedType._300V, TVMSpeedType._300 },
            { TVMSpeedType._300,  TVMSpeedType._000 }
        };

    // TVM430 COVIT (Transmission Voie Machine 430 COntrôle de VITesse / Track Machine Transmission 430 Speed control)
        // Constants
        // TVM430 300 km/h
        string TVM430DecodingFileName;
        Dictionary<Tuple<TVMSpeedType, TVMSpeedType, TVMSpeedType>, Tuple<TVMAspectType, bool, float, float, float>> TVM430DecodingTable = new Dictionary<Tuple<TVMSpeedType, TVMSpeedType, TVMSpeedType>, Tuple<TVMAspectType, bool, float, float, float>>();

        Dictionary<TVMAspectType, Aspect> TVM430S300MstsTranslation = new Dictionary<TVMAspectType, Aspect>
        {
            { TVMAspectType.None, Aspect.None  },
            { TVMAspectType._300V, Aspect.Clear_2 },
            { TVMAspectType._270A, Aspect.Clear_1 },
            { TVMAspectType._270V, Aspect.Clear_1  },
            { TVMAspectType._230A, Aspect.Approach_3 },
            { TVMAspectType._230V, Aspect.Approach_3 },
            { TVMAspectType._230E, Aspect.Approach_3 },
            { TVMAspectType._220A, Aspect.Approach_3 },
            { TVMAspectType._220V, Aspect.Approach_3 },
            { TVMAspectType._220E, Aspect.Approach_3 },
            { TVMAspectType._200A, Aspect.Approach_2 },
            { TVMAspectType._200V, Aspect.Approach_2 },
            { TVMAspectType._170A, Aspect.Approach_2 },
            { TVMAspectType._170E, Aspect.Approach_2 },
            { TVMAspectType._160A, Aspect.Approach_1 },
            { TVMAspectType._160E, Aspect.Approach_1 },
            { TVMAspectType._130A, Aspect.Restricted },
            { TVMAspectType._130E, Aspect.Restricted },
            { TVMAspectType._80A, Aspect.Restricted },
            { TVMAspectType._80E, Aspect.Restricted },
            { TVMAspectType._60A, Aspect.Restricted },
            { TVMAspectType._60E, Aspect.Restricted },
            { TVMAspectType._000, Aspect.Stop },
            { TVMAspectType._RRR, Aspect.Permission }
        };
        Dictionary<TVMAspectType, Aspect> TVM430S320MstsTranslation = new Dictionary<TVMAspectType, Aspect>
        {
            { TVMAspectType.None, Aspect.None  },
            { TVMAspectType._320V, Aspect.Clear_2 },
            { TVMAspectType._300A, Aspect.Clear_1  },
            { TVMAspectType._300V, Aspect.Clear_1  },
            { TVMAspectType._270A, Aspect.Approach_3 },
            { TVMAspectType._270V, Aspect.Approach_3 },
            { TVMAspectType._230A, Aspect.Approach_2 },
            { TVMAspectType._230E, Aspect.Approach_2 },
            { TVMAspectType._220A, Aspect.Approach_2 },
            { TVMAspectType._220E, Aspect.Approach_2 },
            { TVMAspectType._200A, Aspect.Approach_1 },
            { TVMAspectType._200V, Aspect.Approach_1 },
            { TVMAspectType._170A, Aspect.Approach_1 },
            { TVMAspectType._170E, Aspect.Approach_1 },
            { TVMAspectType._160A, Aspect.Approach_1 },
            { TVMAspectType._160E, Aspect.Approach_1 },
            { TVMAspectType._130A, Aspect.Restricted },
            { TVMAspectType._130E, Aspect.Restricted },
            { TVMAspectType._80A, Aspect.Restricted },
            { TVMAspectType._80E, Aspect.Restricted },
            { TVMAspectType._60A, Aspect.Restricted },
            { TVMAspectType._60E, Aspect.Restricted },
            { TVMAspectType._000, Aspect.Stop },
            { TVMAspectType._RRR, Aspect.Permission }
        };

        Dictionary<TVMSpeedType, TVMSpeedType> TVM430SncfTab1 = new Dictionary<TVMSpeedType, TVMSpeedType>
        {
            { TVMSpeedType._RRR,   TVMSpeedType._000 },
            { TVMSpeedType._000,  TVMSpeedType._170 },
            { TVMSpeedType._60E,  TVMSpeedType._60 },
            { TVMSpeedType._60,   TVMSpeedType._170 },
            { TVMSpeedType._80E,  TVMSpeedType._80 },
            { TVMSpeedType._80,   TVMSpeedType._170 },
            { TVMSpeedType._130E, TVMSpeedType._130 },
            { TVMSpeedType._130,  TVMSpeedType._200 },
            { TVMSpeedType._160E, TVMSpeedType._160 },
            { TVMSpeedType._160,  TVMSpeedType._230 },
            { TVMSpeedType._170E, TVMSpeedType._170 },
            { TVMSpeedType._170,  TVMSpeedType._230 },
            { TVMSpeedType._200V, TVMSpeedType._200 },
            { TVMSpeedType._200,  TVMSpeedType._230 },
            { TVMSpeedType._220E, TVMSpeedType._220 },
            { TVMSpeedType._220V, TVMSpeedType._220 },
            { TVMSpeedType._220,  TVMSpeedType._270 },
            { TVMSpeedType._230E, TVMSpeedType._230 },
            { TVMSpeedType._230V, TVMSpeedType._230 },
            { TVMSpeedType._230,  TVMSpeedType._270 },
            { TVMSpeedType._270V, TVMSpeedType._270 },
            { TVMSpeedType._270,  TVMSpeedType._300 },
            { TVMSpeedType._300V, TVMSpeedType._300 },
            { TVMSpeedType._300,  TVMSpeedType._320 },
            { TVMSpeedType._320V, TVMSpeedType._320 },
            { TVMSpeedType._320,  TVMSpeedType._000 }
        };
        Dictionary<TVMSpeedType, TVMSpeedType> TVM430SncfTab2 = new Dictionary<TVMSpeedType, TVMSpeedType>
        {
            { TVMSpeedType._RRR,   TVMSpeedType._000 },
            { TVMSpeedType._000,  TVMSpeedType._000 },
            { TVMSpeedType._60E,  TVMSpeedType._60 },
            { TVMSpeedType._60,   TVMSpeedType._60 },
            { TVMSpeedType._80E,  TVMSpeedType._80 },
            { TVMSpeedType._80,   TVMSpeedType._80 },
            { TVMSpeedType._130E, TVMSpeedType._130 },
            { TVMSpeedType._130,  TVMSpeedType._130 },
            { TVMSpeedType._160E, TVMSpeedType._160 },
            { TVMSpeedType._160,  TVMSpeedType._160 },
            { TVMSpeedType._170E, TVMSpeedType._170 },
            { TVMSpeedType._170,  TVMSpeedType._170 },
            { TVMSpeedType._200V, TVMSpeedType._200 },
            { TVMSpeedType._200,  TVMSpeedType._200 },
            { TVMSpeedType._220E, TVMSpeedType._220 },
            { TVMSpeedType._220V, TVMSpeedType._220 },
            { TVMSpeedType._220,  TVMSpeedType._220 },
            { TVMSpeedType._230E, TVMSpeedType._230 },
            { TVMSpeedType._230V, TVMSpeedType._230 },
            { TVMSpeedType._230,  TVMSpeedType._230 },
            { TVMSpeedType._270V, TVMSpeedType._270 },
            { TVMSpeedType._270,  TVMSpeedType._270 },
            { TVMSpeedType._300V, TVMSpeedType._300 },
            { TVMSpeedType._300,  TVMSpeedType._300 },
            { TVMSpeedType._320V, TVMSpeedType._320 },
            { TVMSpeedType._320,  TVMSpeedType._000 }
        };

        // Parameters
        float TVM430TrainSpeedLimitMpS;

        // Variables
        Timer TVM430AspectChangeTimer;

    // Vigilance monitoring (VACMA)
        // Parameters
        float VACMAActivationSpeedMpS;
        float VACMAReleasedAlertDelayS;
        float VACMAReleasedEmergencyDelayS;
        float VACMAPressedAlertDelayS;
        float VACMAPressedEmergencyDelayS;

        // Variables
        bool VACMAEmergencyBraking = true;
        bool VACMATest = false;
        bool VACMAPressed = false;
        Timer VACMAPressedAlertTimer;
        Timer VACMAPressedEmergencyTimer;
        Timer VACMAReleasedAlertTimer;
        Timer VACMAReleasedEmergencyTimer;

    // Other variables
        float InitCount = 0;

        bool EmergencyBraking = false;
        bool ExternalEmergencyBraking = false;

        float PreviousNormalSignalDistanceM = 0f;
        bool NormalSignalPassed = false;

        float PreviousDistantSignalDistanceM = 0f;
        bool DistantSignalPassed = false;

        float PreviousLineSpeed = 0f;

        public TCS_France() { }

        public override void Initialize()
        {
            // General section
            VACMAPresent = GetBoolParameter("General", "VACMAPresent", true);
            RSOPresent = GetBoolParameter("General", "RSOPresent", true);
            DAATPresent = GetBoolParameter("General", "DAATPresent", false);
            KVBPresent = GetBoolParameter("General", "KVBPresent", false);
            TVM300Present = GetBoolParameter("General", "TVM300Present", false);
            TVM430Present = GetBoolParameter("General", "TVM430Present", false);
            ETCSPresent = GetBoolParameter("General", "ETCSPresent", false);
            ElectroPneumaticBrake = GetBoolParameter("General", "ElectroPneumaticBrake", false);
            HeavyFreightTrain = GetBoolParameter("General", "HeavyFreightTrain", false);
            SafeDecelerationMpS2 = GetFloatParameter("General", "SafeDecelerationMpS2", 0.7f);

            // RSO section
            RSODelayBeforeEmergencyBrakingS = GetFloatParameter("RSO", "DelayBeforeEmergencyBrakingS", 4f);
            RSOBlinkerFrequencyHz = GetFloatParameter("RSO", "BlinkerFrequencyHz", 6f);

            // KVB section
            KVBInhibited = GetBoolParameter("KVB", "Inhibited", false);
            KVBTrainSpeedLimitMpS = MpS.FromKpH(GetFloatParameter("KVB", "TrainSpeedLimitKpH", 160f));

            KVBPrincipalDisplayBlinker = new Blinker(this);
            KVBPrincipalDisplayBlinker.Setup(2f);

            KVBInitOdometer = new OdoMeter(this);
            KVBInitOdometer.Setup(4400f);

            KarmStartOdometer = new OdoMeter(this);
            KarmStartOdometer.Setup(450f);

            // TVM common section
            TVMCOVITInhibited = GetBoolParameter("TVM", "CovitInhibited", false);
            TVMBlinker = new Blinker(this);
            TVMBlinker.Setup(1f);

            // TVM300 section
            TVM300DecodingFileName = GetStringParameter("TVM300", "DecodingFileName", "..\\..\\common.script\\TGVR_TVM300.csv");

            {
                string path = Path.Combine(Path.GetDirectoryName(Locomotive().WagFilePath), "Script", TVM300DecodingFileName);

                if (File.Exists(path))
                {
                    using (StreamReader reader = new StreamReader(path))
                    {
                        string line;
                        while ((line = reader.ReadLine()) != null)
                        {
                            string[] parts = line.Split(';');
                            
                            Tuple<TVMSpeedType, TVMSpeedType, TVMSpeedType> triplet = new Tuple<TVMSpeedType, TVMSpeedType, TVMSpeedType>
                                (
                                    (TVMSpeedType)Enum.Parse(typeof(TVMSpeedType), "_" + parts[0]),
                                    (TVMSpeedType)Enum.Parse(typeof(TVMSpeedType), "_" + parts[1]),
                                    (TVMSpeedType)Enum.Parse(typeof(TVMSpeedType), (parts[2] == "---" ? "Any" : "_" + parts[2]))
                                );
                            Tuple<TVMAspectType, bool, float> onBoardValues = new Tuple<TVMAspectType, bool, float>
                                (
                                    (TVMAspectType)Enum.Parse(typeof(TVMAspectType), "_" + parts[3]),
                                    bool.Parse(parts[4]),
                                    float.Parse(parts[5], CultureInfo.InvariantCulture)
                                );
                            TVM300DecodingTable.Add(triplet, onBoardValues);
                        }
                    }
                }
                else
                {
                    throw new FileNotFoundException(string.Format("File {0} has not been found", path));
                }
            }


            // TVM430 section
            TVM430TrainSpeedLimitMpS = MpS.FromKpH(GetFloatParameter("TVM430", "TrainSpeedLimitKpH", 320f));
            TVM430DecodingFileName = GetStringParameter("TVM430", "DecodingFileName", "..\\..\\common.script\\TGVR_TVM430.csv");

            {
                string path = Path.Combine(Path.GetDirectoryName(Locomotive().WagFilePath), "Script", TVM430DecodingFileName);

                if (File.Exists(path))
                {
                    using (StreamReader reader = new StreamReader(path))
                    {
                        string line;
                        while ((line = reader.ReadLine()) != null)
                        {
                            string[] parts = line.Split(';');

                            Tuple<TVMSpeedType, TVMSpeedType, TVMSpeedType> triplet = new Tuple<TVMSpeedType, TVMSpeedType, TVMSpeedType>
                                (
                                    (TVMSpeedType)Enum.Parse(typeof(TVMSpeedType), "_" + parts[0]),
                                    (TVMSpeedType)Enum.Parse(typeof(TVMSpeedType), "_" + parts[1]),
                                    (TVMSpeedType)Enum.Parse(typeof(TVMSpeedType), (parts[2] == "---" ? "Any" : "_" + parts[2]))
                                );
                            Tuple<TVMAspectType, bool, float, float, float> onBoardValues = new Tuple<TVMAspectType, bool, float, float, float>
                                (
                                    (TVMAspectType)Enum.Parse(typeof(TVMAspectType), "_" + parts[3]),
                                    bool.Parse(parts[4]),
                                    float.Parse(parts[5], CultureInfo.InvariantCulture),
                                    float.Parse(parts[6], CultureInfo.InvariantCulture),
                                    float.Parse(parts[7], CultureInfo.InvariantCulture)
                                );
                            TVM430DecodingTable.Add(triplet, onBoardValues);
                        }
                    }
                }
                else
                {
                    throw new FileNotFoundException(string.Format("File {0} has not been found", path));
                }
            }

            if (TVM300Present)
            {
                TVMModel = TVMModelType.TVM300;
            }
            else if (TVM430Present)
            {
                if (TVM430TrainSpeedLimitMpS == MpS.FromKpH(300f))
                {
                    TVMModel = TVMModelType.TVM430_V300;
                }
                else
                {
                    TVMModel = TVMModelType.TVM430_V320;
                }
            }

            // VACMA section
            VACMAActivationSpeedMpS = MpS.FromKpH(GetFloatParameter("VACMA", "ActivationSpeedKpH", 3f));
            VACMAReleasedAlertDelayS = GetFloatParameter("VACMA", "ReleasedAlertDelayS", 2.5f);
            VACMAReleasedEmergencyDelayS = GetFloatParameter("VACMA", "ReleasedEmergencyDelayS", 5f);
            VACMAPressedAlertDelayS = GetFloatParameter("VACMA", "PressedAlertDelayS", 55f);
            VACMAPressedEmergencyDelayS = GetFloatParameter("VACMA", "PressedEmergencyDelayS", 60f);

            // Variables initialization
            RSOBlinker = new Blinker(this);
            RSOBlinker.Setup(RSOBlinkerFrequencyHz);
            RSOBlinker.Start();
            RSOEmergencyTimer = new Timer(this);
            RSOEmergencyTimer.Setup(RSODelayBeforeEmergencyBrakingS);

            VACMAPressedAlertTimer = new Timer(this);
            VACMAPressedAlertTimer.Setup(VACMAPressedAlertDelayS);
            VACMAPressedEmergencyTimer = new Timer(this);
            VACMAPressedEmergencyTimer.Setup(VACMAPressedEmergencyDelayS);
            VACMAReleasedAlertTimer = new Timer(this);
            VACMAReleasedAlertTimer.Setup(VACMAReleasedAlertDelayS);
            VACMAReleasedEmergencyTimer = new Timer(this);
            VACMAReleasedEmergencyTimer.Setup(VACMAReleasedEmergencyDelayS);

            TVM430AspectChangeTimer = new Timer(this);
            TVM430AspectChangeTimer.Setup(4.7f);

            // Cabview control names initialization
            SetCustomizedCabviewControlName(BP_AC_SF, "BP (AC) SF : Acquittement / Acknowledge");
            SetCustomizedCabviewControlName(BP_A_LS_SF, "BP (A) LS (SF) : Annulation LS (SF) / Cancel LS (SF)");
            SetCustomizedCabviewControlName(Z_ES_VA, "Z (ES) VA : Essai VACMA / Alerter test");
            SetCustomizedCabviewControlName(BP_AM_V1, "BP AM V1 : Armement manuel TVM voie 1 / TVM manual arming track 1");
            SetCustomizedCabviewControlName(BP_AM_V2, "BP AM V2 : Armement manuel TVM voie 2 / TVM manual arming track 2");
            SetCustomizedCabviewControlName(BP_DM, "BP DM : Désarmement manuel TVM / TVM manual dearming");
            SetCustomizedCabviewControlName(VY_CV, "VY CV : COVIT (freinage d'urgence TVM) / TVM emergency braking");
            SetCustomizedCabviewControlName(VY_SECT, "VY SECT : Sectionnement / Open circuit breaker");
            SetCustomizedCabviewControlName(VY_SECT_AU, "VY SECT AU : Sectionnement automatique / Automatic circuit breaker opening");
            SetCustomizedCabviewControlName(VY_BPT, "VY BPT : Baissez Panto / Lower pantograph");
            SetCustomizedCabviewControlName(TVM_VL, "Visualisateur TVM / TVM display");
            SetCustomizedCabviewControlName(TVM_An1, "Visualisateur TVM / TVM display");
            SetCustomizedCabviewControlName(TVM_An2, "Visualisateur TVM / TVM display");
            SetCustomizedCabviewControlName(TVM_Ex1, "Visualisateur TVM / TVM display");
            SetCustomizedCabviewControlName(TVM_Ex2, "Visualisateur TVM / TVM display");
            SetCustomizedCabviewControlName(LS_SF, "LS (SF) : Signal Fermé / Closed Signal");
            SetCustomizedCabviewControlName(VY_SOS_RSO, "VY SOS RSO : FU RSO / RSO EB");
            SetCustomizedCabviewControlName(VY_SOS_VAC, "VY SOS VAC : FU VACMA / Alerter EB");
            SetCustomizedCabviewControlName(VY_ES_FU, "VY ES FU : Essai FU / EB test");
            SetCustomizedCabviewControlName(VY_SOS_KVB, "VY SOS KVB : FU KVB / KVB EB");
            SetCustomizedCabviewControlName(VY_VTE, "VY VTE : Vitesse Trop Elevée / Speed too high");
            SetCustomizedCabviewControlName(VY_FU, "VY FU : FU KVB / KVB EB");
            SetCustomizedCabviewControlName(KVB_Principal1, "Visualisateur principal KVB");
            SetCustomizedCabviewControlName(KVB_Principal2, "Visualisateur principal KVB");
            SetCustomizedCabviewControlName(KVB_Auxiliary, "Visualisateur auxiliaire KVB");
            SetCustomizedCabviewControlName(TVM_Mask, "Masque TVM / TVM mask");

            Activated = true;

            SetNextSignalAspect(Aspect.Clear_1);
        }

        public override void InitializeMoving()
        {
            RSOState = RSOStateType.Off;
            RSOEmergencyBraking = false;
            KVBInit = false;
            KVBState = KVBStateType.Normal;
            KVBEmergencyBraking = false;
            VACMAEmergencyBraking = false;

            if (CurrentPostSpeedLimitMpS() > MpS.FromKpH(221f))
            {
                KVBMode = KVBModeType.HighSpeedLine;
                TVMArmed = true;
            }
        }

        public override void Update()
        {
            if (IsTrainControlEnabled())
            {
                if (InitCount < 5)
                {
                    InitCount++;

                    if (InitCount == 5 && CurrentPostSpeedLimitMpS() > MpS.FromKpH(221f))
                    {
                        KVBMode = KVBModeType.HighSpeedLine;
                        QBal = QBalType.LGV;
                    }

                    return;
                }

                UpdateSignalPassed();

                UpdateVACMA();
                UpdateRSO();
                UpdateTVM();
                UpdateKVB();

                if (RSOEmergencyBraking
                    || KVBEmergencyBraking
                    || KarmEmergencyBraking
                    || TVMCOVITEmergencyBraking
                    || VACMAEmergencyBraking
                    || ExternalEmergencyBraking)
                {
                    EmergencyBraking = true;
                }
                else if (RearmingButton)
                {
                    EmergencyBraking = false;
                }

                SetEmergencyBrake(EmergencyBraking);

                SetPenaltyApplicationDisplay(IsBrakeEmergency());

                SetPowerAuthorization(!EmergencyBraking);

                RSOType1Inhibition = IsDirectionReverse();
                RSOType2Inhibition = !KVBInhibited && ((TVM300Present || TVM430Present) && TVMArmed);
                RSOType3Inhibition = (!TVM300Present && !TVM430Present) || !TVMCOVITInhibited;

                PreviousLineSpeed = CurrentPostSpeedLimitMpS();
            }
        }

        public override void SetEmergency(bool emergency)
        {
            ExternalEmergencyBraking = emergency;
        }

        protected void UpdateRSO()
        {
            if (RSOPresent && IsSpeedControlEnabled())
            {
                // If train is about to cross a normal signal, get its information.
                float nextNormalSignalDistance = NextSignalDistanceM(0);
                Aspect nextNormalSignalAspect = Aspect.None;
                if (nextNormalSignalDistance <= 5f)
                {
                    nextNormalSignalAspect = NextSignalAspect(0);
                }

                // If train is about to cross a distant signal, get its information.
                float nextDistantSignalDistance = NextDistanceSignalDistanceM();
                Aspect nextDistantSignalAspect = Aspect.None;
                if (nextDistantSignalDistance <= 5f)
                {
                    nextDistantSignalAspect = NextDistanceSignalAspect();
                    // Hack for Swiss signals : only approach and clear aspects are allowed on distant signals
                    if (nextDistantSignalAspect > Aspect.Approach_1)
                    {
                        nextDistantSignalAspect = Aspect.Approach_1;
                    }
                }

                if (nextNormalSignalAspect != Aspect.None || nextDistantSignalAspect != Aspect.None)
                {
                    RSOLastSignalAspect = Max(nextNormalSignalAspect, nextDistantSignalAspect);
                }

                RSOClosedSignal = RSOOpenedSignal = false;

                if ((NormalSignalPassed || DistantSignalPassed)
                    && !RSOType1Inhibition
                    && !TVMArmed
                    && SpeedMpS() > 0.1f)
                {
                    if (RSOLastSignalAspect == Aspect.Stop
                        || RSOLastSignalAspect == Aspect.StopAndProceed
                        || RSOLastSignalAspect == Aspect.Restricted
                        || RSOLastSignalAspect == Aspect.Approach_1
                        || RSOLastSignalAspect == Aspect.Approach_2
                        || RSOLastSignalAspect == Aspect.Approach_3
                        )
                    {
                        RSOClosedSignal = true;
                    }
                    else
                    {
                        RSOOpenedSignal = true;
                    }
                }

                if ((RSOClosedSignal && !RSOType2Inhibition) || (TVMClosedSignal && !RSOType3Inhibition))
                {
                    RSOEmergencyTimer.Start();

                    if (RSOPressed)
                    {
                        RSOState = RSOStateType.TriggeredPressed;
                    }
                    else
                    {
                        RSOState = RSOStateType.TriggeredBlinking;
                    }
                }

                if (RSOOpenedSignal || TVMOpenedSignal || RSOCancelPressed)
                {
                    RSOEmergencyTimer.Stop();
                    RSOState = RSOStateType.Off;
                }

                switch (RSOState)
                {
                    case RSOStateType.Init:
                        if (!RSOBlinker.Started)
                        {
                            RSOBlinker.Start();
                        }
                        SetCabDisplayControl(LS_SF, RSOBlinker.On || RSOPressed ? 1 : 0);
                        break;

                    case RSOStateType.Off:
                        if (RSOBlinker.Started)
                        {
                            RSOBlinker.Stop();
                        }
                        SetCabDisplayControl(LS_SF, RSOPressed ? 1 : 0);
                        break;

                    case RSOStateType.TriggeredPressed:
                        SetCabDisplayControl(LS_SF, 0);

                        if (!RSOPressed)
                        {
                            RSOState = RSOStateType.TriggeredFixed;
                            RSOEmergencyTimer.Stop();
                        }
                        break;

                    case RSOStateType.TriggeredBlinking:
                        if (!RSOBlinker.Started)
                        {
                            RSOBlinker.Start();
                        }
                        SetCabDisplayControl(LS_SF, RSOBlinker.On ? 1 : 0);

                        if (!RSOPressed && RSOPreviousPressed)
                        {
                            RSOState = RSOStateType.TriggeredFixed;
                            RSOEmergencyTimer.Stop();
                        }
                        break;

                    case RSOStateType.TriggeredFixed:
                        SetCabDisplayControl(LS_SF, 1);
                        break;
                }

                if (RSOEmergencyTimer.Triggered)
                {
                    RSOEmergencyBraking = true;
                }
                else if (RearmingButton)
                {
                    RSOEmergencyBraking = false;
                }

                SetCabDisplayControl(VY_SOS_RSO, RSOEmergencyBraking ? 1 : 0);

                if (RSOClosedSignal && !RSOPreviousClosedSignal && !RSOType1Inhibition)
                {
                    TriggerSoundInfo1();
                }

                RSOPreviousClosedSignal = RSOClosedSignal;

                if (TVM300Present || TVM430Present)
                {
                    if (TVMClosedSignal && !TVMPreviousClosedSignal)
                    {
                        TriggerSoundInfo1();
                    }

                    if (TVMOpenedSignal && !TVMPreviousOpenedSignal)
                    {
                        TriggerSoundInfo1();
                    }

                    TVMPreviousClosedSignal = TVMClosedSignal;
                    TVMPreviousOpenedSignal = TVMOpenedSignal;
                }

                RSOPreviousPressed = RSOPressed;
            }
        }

        protected void UpdateKVB()
        {
            if (KVBPresent && IsSpeedControlEnabled())
            {
                if (CurrentPostSpeedLimitMpS() > MpS.FromKpH(221f) && PreviousLineSpeed <= MpS.FromKpH(221f) && SpeedMpS() > 0f)
                {
                    KarmStartOdometer.Start();

                    KVBSpadEmergency = false;
                    KVBOverspeedEmergency = false;
                    KVBSpeedTooHighLight = false;
                    KVBMode = KVBModeType.HighSpeedLine;
                }
                else if (NextPostSpeedLimitMpS(0) <= MpS.FromKpH(221f) && NextPostDistanceM(0) < 5f && PreviousLineSpeed > MpS.FromKpH(221f) && SpeedMpS() > 0f)
                {
                    KVBMode = KVBModeType.ConventionalLine;
                }

                if (KarmStartOdometer.Triggered)
                {
                    KarmStartOdometer.Stop();
                    QBal = QBalType.LGV;
                }
                else if (NextPostSpeedLimitMpS(0) <= MpS.FromKpH(221f) && NextPostDistanceM(0) < 60f && PreviousLineSpeed > MpS.FromKpH(221f) && SpeedMpS() > 0f)
                {
                    QBal = QBalType.LC;
                }

                if (QBal == QBalType.LGV && (TVM300Present || TVM430Present))
                {
                    if (!KVBInhibited && !TVMArmed)
                    {
                        KarmEmergencyBraking = true;
                    }
                    else if (RearmingButton)
                    {
                        KarmEmergencyBraking = false;
                    }
                }
                else
                {
                    KarmEmergencyBraking = false;
                }

                switch (KVBMode)
                {
                    case KVBModeType.HighSpeedLine:
                        ResetKVBTargets();

                        UpdateKVBInit();

                        UpdateKVBEmergencyBraking();

                        UpdateKVBDisplay();
                        break;

                    case KVBModeType.ConventionalLine:
                        KVBMode = KVBModeType.ConventionalLine;

                        UpdateKVBParameters();

                        UpdateKVBInit();

                        UpdateKVBTargets();

                        UpdateKVBSpeedControl();

                        UpdateKVBEmergencyBraking();

                        UpdateKVBDisplay();

                        // Send data to the simulator
                        if (KVBStopTargetSignalNumber == 0)
                        {
                            SetNextSpeedLimitMpS(0f);
                        }
                        else if (KVBSpeedRestrictionTargetSignalNumber == 0)
                        {
                            SetNextSpeedLimitMpS(KVBSpeedRestrictionTargetSpeedMpS);
                        }
                        else
                        {
                            SetNextSpeedLimitMpS(KVBNextLineSpeedLimitMpS);
                        }
                        SetCurrentSpeedLimitMpS(Math.Min(KVBLastSignalSpeedLimitMpS, KVBCurrentLineSpeedLimitMpS));
                        break;
                }
            }
            else
            {
                KVBEmergencyBraking = false;
            }
        }

        protected void UpdateKVBParameters()
        {
            KVBTrainLengthM = (float)Math.Ceiling((double)(TrainLengthM() / 100f)) * 100f;
            if (ElectroPneumaticBrake)
                KVBDelayBeforeBrakingEstablishedS = 2f;
            else if (HeavyFreightTrain)
                KVBDelayBeforeBrakingEstablishedS = 12f + KVBTrainLengthM / 200f;
            else
                KVBDelayBeforeBrakingEstablishedS = 2f + 2f * KVBTrainLengthM * KVBTrainLengthM * 0.00001f;
        }

        protected void UpdateKVBInit()
        {
            if (KVBInit)
            {
                if (!KVBInitOdometer.Started)
                {
                    KVBInitOdometer.Start();
                }

                if (KVBInitOdometer.Triggered)
                {
                    KVBInit = false;
                }
            }
        }

        protected void UpdateKVBTargets()
        {
            // Line speed limit
            KVBCurrentLineSpeedLimitMpS = CurrentPostSpeedLimitMpS();
            KVBNextLineSpeedLimitMpS = NextPostSpeedLimitMpS(0) > 0 ? NextPostSpeedLimitMpS(0) : float.PositiveInfinity;
            KVBNextLineSpeedDistanceM = NextPostDistanceM(0);

            // If train is about to cross a normal signal, get its information.
            float nextNormalSignalDistance = NextSignalDistanceM(0);
            Aspect nextNormalSignalAspect = Aspect.None;
            if (nextNormalSignalDistance <= 5f)
            {
                nextNormalSignalAspect = NextSignalAspect(0);
                KVBLastSignalSpeedLimitMpS = NextSignalSpeedLimitMpS(0) > 0f ? NextSignalSpeedLimitMpS(0) : float.PositiveInfinity;
            }

            // If train is about to cross a distant signal, get its information.
            float nextDistantSignalDistance = NextDistanceSignalDistanceM();
            Aspect nextDistantSignalAspect = Aspect.None;
            if (nextDistantSignalDistance <= 5f)
            {
                nextDistantSignalAspect = NextDistanceSignalAspect();
                // Hack for Swiss signals : only approach and clear aspects are allowed on distant signals
                if (nextDistantSignalAspect > Aspect.Approach_1)
                {
                    nextDistantSignalAspect = Aspect.Approach_1;
                }
            }

            if (nextNormalSignalAspect != Aspect.None || nextDistantSignalAspect != Aspect.None)
            {
                KVBLastSignalAspect = Max(nextNormalSignalAspect, nextDistantSignalAspect);
            }

            // If not on sight, current track node is longer than train length and no switch is in front of us, release the signal speed limit
            float trackNodeOFfset = Locomotive().Train.FrontTDBTraveller.TrackNodeOffset;
            float nextDivergingSwitchDistanceM = NextDivergingSwitchDistanceM(500f);
            float nextTrailingDivergingSwitchDistanceM = NextTrailingDivergingSwitchDistanceM(500f);
            if (!KVBOnSight
                && trackNodeOFfset > KVBTrainLengthM
                && nextDivergingSwitchDistanceM > nextNormalSignalDistance
                && nextTrailingDivergingSwitchDistanceM > nextNormalSignalDistance
                )
            {
                KVBLastSignalSpeedLimitMpS = float.PositiveInfinity;
            }

            if ((NormalSignalPassed || DistantSignalPassed) && SpeedMpS() > 0.1f)
            {
                // Signal passed at danger check
                if (KVBLastSignalAspect == Aspect.Stop)
                {
                    KVBSpadEmergency = true;
                    TriggerSoundPenalty2();
                    Message(ConfirmLevel.Warning, "SOS KVB");
                    Message(ConfirmLevel.Warning, "KVB : Franchissement carré / Signal passed at danger");
                }
                else if (KVBLastSignalAspect == Aspect.StopAndProceed)
                {
                    KVBOnSight = true;
                    KVBLastSignalSpeedLimitMpS = MpS.FromKpH(30);
                }
                // Search for a stop target
                else
                {
                    KVBOnSight = false;

                    int i;
                    Aspect aspect;

                    // Search for the next stop signal
                    for (i = 0; i < 5; i++)
                    {
                        aspect = NextSignalAspect(i);

                        if (aspect == Aspect.Stop
                            || aspect == Aspect.StopAndProceed)
                        {
                            break;
                        }
                    }

                    // If signal found
                    if (i < 5)
                    {
                        KVBStopTargetSignalNumber = i;
                        KVBStopTargetDistanceM = NextSignalDistanceM(i);
                    }
                    else
                    {
                        KVBStopTargetSignalNumber = -1;
                        KVBStopTargetDistanceM = float.PositiveInfinity;
                    }

                    // Reset release speed
                    KVBStopTargetReleaseSpeed = KVBReleaseSpeed.V30;
                }

                // Search for a speed restriction target
                {
                    int i;
                    float speed = 0f;

                    // Search for the next stop signal
                    for (i = 0; i < 5; i++)
                    {
                        speed = NextSignalSpeedLimitMpS(i);

                        if (speed > 0f && speed < KVBTrainSpeedLimitMpS)
                        {
                            break;
                        }
                    }

                    // If signal found
                    if (i < 5)
                    {
                        KVBSpeedRestrictionTargetSignalNumber = i;
                        KVBSpeedRestrictionTargetDistanceM = NextSignalDistanceM(i);
                        KVBSpeedRestrictionTargetSpeedMpS = speed;
                    }
                    else
                    {
                        KVBSpeedRestrictionTargetSignalNumber = -1;
                        KVBSpeedRestrictionTargetDistanceM = float.PositiveInfinity;
                        KVBSpeedRestrictionTargetSpeedMpS = float.PositiveInfinity;
                    }
                }
            }

            // Pre-announce aspect
            switch (KVBPreAnnounce)
            {
                case KVBPreAnnounceType.Deactivated:
                    if (!KVBInit
                        && KVBLastSignalSpeedLimitMpS > MpS.FromKpH(160f)
                        && (KVBSpeedRestrictionTargetSignalNumber != 0 || KVBSpeedRestrictionTargetSpeedMpS > MpS.FromKpH(160f))
                        && KVBCurrentLineSpeedLimitMpS > MpS.FromKpH(160f)
                        && (KVBNextLineSpeedLimitMpS > MpS.FromKpH(160f) || KVBNextLineSpeedDistanceM > 3000f))
                    {
                        KVBPreAnnounce = KVBPreAnnounceType.Armed;
                    }
                    break;

                case KVBPreAnnounceType.Armed:
                    if (KVBCurrentLineSpeedLimitMpS <= MpS.FromKpH(160f))
                    {
                        KVBPreAnnounce = KVBPreAnnounceType.Deactivated;
                    }

                    if (NormalSignalPassed
                        && KVBLastSignalSpeedLimitMpS > MpS.FromKpH(160f)
                        && KVBSpeedRestrictionTargetSignalNumber == 0
                        && KVBSpeedRestrictionTargetSpeedMpS <= MpS.FromKpH(160f))
                    {
                        KVBPreAnnounce = KVBPreAnnounceType.Triggered;
                        TriggerSoundInfo2();
                    }
                    // TODO : Use the P sign in order to locate the point where pre-announce must be deactivated (instead of 3000m before the start of the restriction)
                    else if (KVBNextLineSpeedLimitMpS <= MpS.FromKpH(160f)
                        && KVBNextLineSpeedDistanceM <= 3000f)
                    {
                        KVBPreAnnounce = KVBPreAnnounceType.Triggered;
                        TriggerSoundInfo2();
                    }
                    break;

                case KVBPreAnnounceType.Triggered:
                    if (KVBCurrentLineSpeedLimitMpS <= MpS.FromKpH(160f)
                        || KVBLastSignalSpeedLimitMpS <= MpS.FromKpH(160f))
                    {
                        KVBPreAnnounce = KVBPreAnnounceType.Deactivated;
                    }
                    break;
            }

            // Update distances
            if (KVBStopTargetSignalNumber >= 0)
            {
                KVBStopTargetDistanceM = NextSignalDistanceM(KVBStopTargetSignalNumber);

                // Proximity to a C aspect
                if (KVBStopTargetDistanceM <= 200f
                    && KVBStopTargetReleaseSpeed == KVBReleaseSpeed.V30
                    && NextSignalAspect(KVBStopTargetSignalNumber) == Aspect.Stop)
                {
                    KVBStopTargetReleaseSpeed = KVBReleaseSpeed.V10;
                }
            }

            if (KVBSpeedRestrictionTargetSignalNumber >= 0)
            {
                KVBSpeedRestrictionTargetDistanceM = NextSignalDistanceM(KVBSpeedRestrictionTargetSignalNumber);
            }
        }

        protected void UpdateKVBSpeedControl()
        {
            float KVBStopTargetAlertSpeedMpS = MpS.FromKpH(5f);
            float KVBStopTargetEBSpeedMpS = MpS.FromKpH(10f);
            float KVBStopTargetReleaseSpeedMpS = MpS.FromKpH(30f);
            if (KVBStopTargetReleaseSpeed == KVBReleaseSpeed.V10)
            {
                KVBStopTargetAlertSpeedMpS = MpS.FromKpH(2.5f);
                KVBStopTargetEBSpeedMpS = MpS.FromKpH(5f);
                KVBStopTargetReleaseSpeedMpS = MpS.FromKpH(10f);
            }

            bool alert = false;
            bool emergency = false;
            KVBSpeedTooHighLight = false;

            // Train speed limit
            alert |= SpeedMpS() > KVBTrainSpeedLimitMpS + MpS.FromKpH(5f);
            emergency |= SpeedMpS() > KVBTrainSpeedLimitMpS + MpS.FromKpH(10f);

            // Stop aspect
            if (KVBStopTargetSignalNumber >= 0)
            {
                alert |= CheckKVBSpeedCurve(
                    KVBStopTargetDistanceM,
                    0f,
                    KVBDeclivity,
                    KVBDelayBeforeBrakingEstablishedS + KVBDelayBeforeEmergencyBrakingS,
                    KVBStopTargetAlertSpeedMpS,
                    KVBStopTargetReleaseSpeedMpS);

                emergency |= CheckKVBSpeedCurve(
                    KVBStopTargetDistanceM,
                    0f,
                    KVBDeclivity,
                    KVBDelayBeforeBrakingEstablishedS,
                    KVBStopTargetEBSpeedMpS,
                    KVBStopTargetReleaseSpeedMpS);
            }

            // Speed restriction
            if (KVBSpeedRestrictionTargetSignalNumber >= 0)
            {
                alert |= CheckKVBSpeedCurve(
                    KVBSpeedRestrictionTargetDistanceM,
                    KVBSpeedRestrictionTargetSpeedMpS,
                    KVBDeclivity,
                    KVBDelayBeforeBrakingEstablishedS + KVBDelayBeforeEmergencyBrakingS,
                    MpS.FromKpH(5f),
                    KVBSpeedRestrictionTargetSpeedMpS);

                emergency |= CheckKVBSpeedCurve(
                    KVBSpeedRestrictionTargetDistanceM,
                    KVBSpeedRestrictionTargetSpeedMpS,
                    KVBDeclivity,
                    KVBDelayBeforeBrakingEstablishedS,
                    MpS.FromKpH(10f),
                    KVBSpeedRestrictionTargetSpeedMpS);
            }

            // Current speed restriction
            alert |= SpeedMpS() > KVBLastSignalSpeedLimitMpS + MpS.FromKpH(5f);
            KVBSpeedTooHighLight |= SpeedMpS() > KVBLastSignalSpeedLimitMpS + MpS.FromKpH(5f);
            emergency |= SpeedMpS() > KVBLastSignalSpeedLimitMpS + MpS.FromKpH(10f);

            // Current line speed
            if (KVBCurrentLineSpeedLimitMpS > MpS.FromKpH(160f) && KVBPreAnnounce == KVBPreAnnounceType.Deactivated)
            {
                alert |= SpeedMpS() > MpS.FromKpH(160f) + MpS.FromKpH(5f);
                KVBSpeedTooHighLight |= SpeedMpS() > MpS.FromKpH(160f) + MpS.FromKpH(5f);
                emergency |= SpeedMpS() > MpS.FromKpH(160f) + MpS.FromKpH(10f);
            }
            else
            {
                alert |= SpeedMpS() > KVBCurrentLineSpeedLimitMpS + MpS.FromKpH(5f);
                KVBSpeedTooHighLight |= SpeedMpS() > KVBCurrentLineSpeedLimitMpS + MpS.FromKpH(5f);
                emergency |= SpeedMpS() > KVBCurrentLineSpeedLimitMpS + MpS.FromKpH(10f);
            }

            // Next line speed
            if (KVBNextLineSpeedLimitMpS < KVBCurrentLineSpeedLimitMpS)
            {
                alert |= CheckKVBSpeedCurve(
                    KVBNextLineSpeedDistanceM,
                    KVBNextLineSpeedLimitMpS,
                    KVBDeclivity,
                    KVBDelayBeforeBrakingEstablishedS + KVBDelayBeforeEmergencyBrakingS,
                    MpS.FromKpH(5f),
                    KVBNextLineSpeedLimitMpS);

                emergency |= CheckKVBSpeedCurve(
                    KVBNextLineSpeedDistanceM,
                    KVBNextLineSpeedLimitMpS,
                    KVBDeclivity,
                    KVBDelayBeforeBrakingEstablishedS,
                    MpS.FromKpH(10f),
                    KVBNextLineSpeedLimitMpS);
            }

            switch (KVBState)
            {
                case KVBStateType.Normal:
                    if (alert)
                    {
                        TriggerSoundPenalty1();
                        KVBState = KVBStateType.Alert;
                        Message(ConfirmLevel.Warning, "KVB : Survitesse / Overspeed");
                    }
                    break;

                case KVBStateType.Alert:
                    if (!alert)
                    {
                        KVBState = KVBStateType.Normal;
                    }
                    else if (emergency)
                    {
                        TriggerSoundPenalty2();
                        KVBState = KVBStateType.Emergency;
                        Message(ConfirmLevel.Warning, "KVB : Survitesse / Overspeed");
                        Message(ConfirmLevel.Warning, "SOS KVB");
                    }
                    break;

                case KVBStateType.Emergency:
                    if (SpeedMpS() < 0.1f)
                    {
                        KVBState = KVBStateType.Normal;
                    }
                    break;
            }

            KVBOverspeedEmergency = KVBState == KVBStateType.Emergency;
        }

        protected void UpdateKVBEmergencyBraking()
        {
            if (KVBSpadEmergency && SpeedMpS() < 0.1f)
            {
                KVBSpadEmergency = false;
            }

            if (KVBOverspeedEmergency && SpeedMpS() < 0.1f)
            {
                KVBOverspeedEmergency = false;
            }


            if (!KVBEmergencyBraking)
            {
                if (KVBSpadEmergency || KVBOverspeedEmergency)
                {
                    KVBEmergencyBraking = true;
                }
            }
            else
            {
                if (!KVBSpadEmergency && !KVBOverspeedEmergency && RearmingButton)
                {
                    KVBEmergencyBraking = false;

                    // On sight till the end of the block section
                    KVBOnSight = true;
                    KVBLastSignalSpeedLimitMpS = MpS.FromKpH(30);
                    KVBStopTargetReleaseSpeed = KVBReleaseSpeed.V30;
                }
            }
        }

        protected void UpdateKVBDisplay()
        {
            SetOverspeedWarningDisplay(KVBState >= KVBStateType.Alert);

            // Legacy display
            if (KVBMode != KVBModeType.HighSpeedLine)
            {
                if (KVBPreAnnounce == KVBPreAnnounceType.Armed)
                {
                    SetNextSignalAspect(Aspect.Clear_2);
                }
                else if (KVBStopTargetReleaseSpeed == KVBReleaseSpeed.V10)
                {
                    SetNextSignalAspect(Aspect.Stop);
                }
                else
                {
                    SetNextSignalAspect(Aspect.Clear_1);
                }
            }

            // New display
            if (KVBMode == KVBModeType.HighSpeedLine)
            {
                SetCabDisplayControl(KVB_Principal1, 0);
                SetCabDisplayControl(KVB_Principal2, 0);
                SetCabDisplayControl(KVB_Auxiliary, 0);
            }
            else
            {
                if (KVBEmergencyBrakeLight)
                {
                    KVBPrincipalDisplayState = KVBPrincipalDisplayStateType.FU;
                    KVBPrincipalDisplayBlinking = false;
                    KVBAuxiliaryDisplayState = KVBAuxiliaryDisplayStateType.Empty;
                }
                else if (KVBInit)
                {
                    KVBPrincipalDisplayState = KVBPrincipalDisplayStateType.Empty;
                    KVBPrincipalDisplayBlinking = false;
                    KVBAuxiliaryDisplayState = KVBAuxiliaryDisplayStateType.Empty;
                }
                else if (KVBPreAnnounce == KVBPreAnnounceType.Armed)
                {
                    KVBPrincipalDisplayState = KVBPrincipalDisplayStateType.b;
                    KVBPrincipalDisplayBlinking = false;
                    KVBAuxiliaryDisplayState = KVBAuxiliaryDisplayStateType.Empty;
                }
                else if (KVBPreAnnounce == KVBPreAnnounceType.Triggered)
                {
                    KVBPrincipalDisplayState = KVBPrincipalDisplayStateType.Empty;
                    KVBPrincipalDisplayBlinking = false;
                    KVBAuxiliaryDisplayState = KVBAuxiliaryDisplayStateType.p;
                }
                else if (KVBOnSight)
                {
                    KVBPrincipalDisplayState = KVBPrincipalDisplayStateType.V00;
                    KVBPrincipalDisplayBlinking = KVBState == KVBStateType.Alert;
                    KVBAuxiliaryDisplayState = KVBAuxiliaryDisplayStateType.V00;
                }
                else if (KVBStopTargetSignalNumber == 0)
                {
                    if (KVBStopTargetReleaseSpeed == KVBReleaseSpeed.V10)
                    {
                        KVBPrincipalDisplayState = KVBPrincipalDisplayStateType.Empty;
                        KVBPrincipalDisplayBlinking = false;
                        KVBAuxiliaryDisplayState = KVBAuxiliaryDisplayStateType.V000;
                    }
                    else
                    {
                        if (KVBState == KVBStateType.Alert)
                        {
                            KVBPrincipalDisplayState = KVBPrincipalDisplayStateType.V00;
                            KVBPrincipalDisplayBlinking = true;
                        }
                        else
                        {
                            KVBPrincipalDisplayState = KVBPrincipalDisplayStateType.Empty;
                            KVBPrincipalDisplayBlinking = false;
                        }
                        KVBAuxiliaryDisplayState = KVBAuxiliaryDisplayStateType.V00;
                    }
                }
                else
                {
                    KVBPrincipalDisplayState = KVBPrincipalDisplayStateType.Dashes3;
                    KVBPrincipalDisplayBlinking = false;
                    KVBAuxiliaryDisplayState = KVBAuxiliaryDisplayStateType.Dashes3;
                }

                if (KVBPrincipalDisplayBlinking)
                {
                    if (!KVBPrincipalDisplayBlinker.Started)
                    {
                        KVBPrincipalDisplayBlinker.Start();
                    }
                }
                else
                {
                    if (KVBPrincipalDisplayBlinker.Started)
                    {
                        KVBPrincipalDisplayBlinker.Stop();
                    }
                }

                switch (KVBPrincipalDisplayState)
                {
                    case KVBPrincipalDisplayStateType.Empty:
                        SetCabDisplayControl(KVB_Principal1, 0);
                        SetCabDisplayControl(KVB_Principal2, 0);
                        break;

                    case KVBPrincipalDisplayStateType.FU:
                        SetCabDisplayControl(KVB_Principal1, 0);
                        SetCabDisplayControl(KVB_Principal2, 1);
                        break;

                    case KVBPrincipalDisplayStateType.V000:
                        if (KVBPrincipalDisplayBlinking)
                        {
                            SetCabDisplayControl(KVB_Principal1, KVBPrincipalDisplayBlinker.On ? 1 : 0);
                            SetCabDisplayControl(KVB_Principal2, 0);
                        }
                        else
                        {
                            SetCabDisplayControl(KVB_Principal1, 1);
                            SetCabDisplayControl(KVB_Principal2, 0);
                        }
                        break;

                    case KVBPrincipalDisplayStateType.V00:
                        if (KVBPrincipalDisplayBlinking)
                        {
                            SetCabDisplayControl(KVB_Principal1, KVBPrincipalDisplayBlinker.On ? 2 : 0);
                            SetCabDisplayControl(KVB_Principal2, 0);
                        }
                        else
                        {
                            SetCabDisplayControl(KVB_Principal1, 2);
                            SetCabDisplayControl(KVB_Principal2, 0);
                        }
                        break;

                    case KVBPrincipalDisplayStateType.L:
                        if (KVBPrincipalDisplayBlinking)
                        {
                            SetCabDisplayControl(KVB_Principal1, 0);
                            SetCabDisplayControl(KVB_Principal2, KVBPrincipalDisplayBlinker.On ? 4 : 0);
                        }
                        else
                        {
                            SetCabDisplayControl(KVB_Principal1, 0);
                            SetCabDisplayControl(KVB_Principal2, 4);
                        }
                        break;

                    case KVBPrincipalDisplayStateType.b:
                        if (KVBPrincipalDisplayBlinking)
                        {
                            SetCabDisplayControl(KVB_Principal1, KVBPrincipalDisplayBlinker.On ? 4 : 0);
                            SetCabDisplayControl(KVB_Principal2, 0);
                        }
                        else
                        {
                            SetCabDisplayControl(KVB_Principal1, 4);
                            SetCabDisplayControl(KVB_Principal2, 0);
                        }
                        break;

                    case KVBPrincipalDisplayStateType.p:
                        if (KVBPrincipalDisplayBlinking)
                        {
                            SetCabDisplayControl(KVB_Principal1, KVBPrincipalDisplayBlinker.On ? 5 : 0);
                            SetCabDisplayControl(KVB_Principal2, 0);
                        }
                        else
                        {
                            SetCabDisplayControl(KVB_Principal1, 5);
                            SetCabDisplayControl(KVB_Principal2, 0);
                        }
                        break;

                    case KVBPrincipalDisplayStateType.Dashes3:
                        if (KVBPrincipalDisplayBlinking)
                        {
                            SetCabDisplayControl(KVB_Principal1, KVBPrincipalDisplayBlinker.On ? 6 : 0);
                            SetCabDisplayControl(KVB_Principal2, 0);
                        }
                        else
                        {
                            SetCabDisplayControl(KVB_Principal1, 6);
                            SetCabDisplayControl(KVB_Principal2, 0);
                        }
                        break;

                    case KVBPrincipalDisplayStateType.Dashes9:
                        SetCabDisplayControl(KVB_Principal1, 7);
                        SetCabDisplayControl(KVB_Principal2, 0);
                        break;

                    case KVBPrincipalDisplayStateType.Test:
                        SetCabDisplayControl(KVB_Principal1, 0);
                        SetCabDisplayControl(KVB_Principal2, 7);
                        break;
                }


                switch (KVBAuxiliaryDisplayState)
                {
                    case KVBAuxiliaryDisplayStateType.Empty:
                        SetCabDisplayControl(KVB_Auxiliary, 0);
                        break;

                    case KVBAuxiliaryDisplayStateType.V000:
                        SetCabDisplayControl(KVB_Auxiliary, 1);
                        break;

                    case KVBAuxiliaryDisplayStateType.V00:
                        SetCabDisplayControl(KVB_Auxiliary, 2);
                        break;

                    case KVBAuxiliaryDisplayStateType.L:
                        SetCabDisplayControl(KVB_Auxiliary, 4);
                        break;

                    case KVBAuxiliaryDisplayStateType.p:
                        SetCabDisplayControl(KVB_Auxiliary, 5);
                        break;

                    case KVBAuxiliaryDisplayStateType.Dashes3:
                        SetCabDisplayControl(KVB_Auxiliary, 6);
                        break;

                    case KVBAuxiliaryDisplayStateType.Test:
                        SetCabDisplayControl(KVB_Auxiliary, 7);
                        break;
                }
            }

            // VY SOS KVB
            SetCabDisplayControl(VY_SOS_KVB, KVBEmergencyBraking ? 1 : 0);

            // VY VTE
            SetCabDisplayControl(VY_VTE, KVBSpeedTooHighLight ? 1 : 0);

            // VY FU
            KVBEmergencyBrakeLight = KVBSpadEmergency || KVBOverspeedEmergency;
            SetCabDisplayControl(VY_FU, KVBEmergencyBrakeLight ? 1 : 0);
        }

        protected bool CheckKVBSpeedCurve(float targetDistanceM, float targetSpeedMpS, float slope, float delayS, float marginMpS, float releaseSpeedMpS)
        {
            float speedCurveMpS =
                Math.Max(
                    SpeedCurve(
                        targetDistanceM,
                        targetSpeedMpS,
                        slope,
                        delayS,
                        SafeDecelerationMpS2
                    ),
                    releaseSpeedMpS + marginMpS
                );

            return SpeedMpS() > speedCurveMpS;
        }

        protected void ResetKVBTargets()
        {
            KVBPreAnnounce = KVBPreAnnounceType.Deactivated;

            KVBLastSignalAspect = Aspect.Clear_1;
            KVBLastSignalSpeedLimitMpS = float.PositiveInfinity;

            KVBStopTargetSignalNumber = -1;
            KVBStopTargetDistanceM = float.PositiveInfinity;
            KVBStopTargetReleaseSpeed = KVBReleaseSpeed.V30;
            KVBOnSight = false;

            KVBSpeedRestrictionTargetSignalNumber = -1;
            KVBSpeedRestrictionTargetDistanceM = float.PositiveInfinity;
            KVBSpeedRestrictionTargetSpeedMpS = float.PositiveInfinity;

            KVBCurrentLineSpeedLimitMpS = float.PositiveInfinity;
            KVBNextLineSpeedLimitMpS = float.PositiveInfinity;
            KVBNextLineSpeedDistanceM = float.PositiveInfinity;

            KVBState = KVBStateType.Normal;
        }

        protected void UpdateTVM()
        {
            if ((TVM300Present || TVM430Present) && IsSpeedControlEnabled())
            {
                // Automatic arming
                if (NextPostSpeedLimitMpS(0) > MpS.FromKpH(221f) && NextPostDistanceM(0) < 5f && PreviousLineSpeed <= MpS.FromKpH(221f) && SpeedMpS() > 0f && !TVMArmed)
                {
                    TVMArmed = true;
                    PreviousSectionAspect = NextSignalAspect(0);
                    PreviousSectionSpeed = Convert.ToInt32(MpS.ToKpH(NextSignalSpeedLimitMpS(0)));
                }

                // Automatic dearming
                if (CurrentPostSpeedLimitMpS() <= MpS.FromKpH(221f) && PreviousLineSpeed > MpS.FromKpH(221f) && SpeedMpS() > 0f && TVMArmed)
                {
                    TVMArmed = false;
                    TVMCOVITEmergencyBraking = false;
                    TVM430AspectChangeTimer.Stop();
                }

                if (TVMArmed)
                {
                    CalculateTvmSequence();
                    DetermineTvmAspect();
                    UpdateTvmCovit();
                    UpdateTvmDisplay();
                    UpdateTvmSounds();

                    TVMAspectPreviousCycle = TVMAspectCurrent;
                    TVMBlinkingPreviousCycle = TVMBlinkingCurrent;
                }
                else
                {
                    TVMCOVITEmergencyBraking = false;

                    TVMAspectCommand = TVMAspectType.None;
                    TVMAspectCurrent = TVMAspectType.None;
                    TVMAspectPreviousCycle = TVMAspectType.None;
                    TVMBlinkingCommand = false;
                    TVMBlinkingCurrent = false;
                    TVMBlinkingPreviousCycle = false;

                    TVMStartControlSpeedMpS = 0f;
                    TVMEndControlSpeedMpS = 0f;
                    TVMDecelerationMpS2 = 0f;

                    UpdateTvmDisplay();
                }
            }
        }

        protected void CalculateTvmSequence()
        {
            int i;

            if (NormalSignalPassed)
            {
                PreviousSectionSpeed = SpeedSequence[0];
                PreviousSectionAspect = AspectSequence[0];
                PreviousVcond = Vcond[0];
            }

            int ignoreCount = 0;

            // Get the 10 next signals (from 10th to 1st in order to be optimal)
            for (i = TVMNumberOfBlockSections - 1; i >= 0; i--)
            {
                SpeedSequence[i] = Convert.ToInt32(MpS.ToKpH(NextSignalSpeedLimitMpS(i)));
                AspectSequence[i] = NextSignalAspect(i);

                if (SpeedSequence[i] <= 0f && AspectSequence[i] > Aspect.Stop)
                {
                    // Ignore
                    ignoreCount++;
                    i++;
                }
            }

            // Calculate execution speeds (Vcond)
            for (i = TVMNumberOfBlockSections - 1; i >= 0; i--)
            {
                Aspect currentAspect;
                int currentSpeed;

                if (i == 0)
                {
                    currentAspect = PreviousSectionAspect;
                    currentSpeed = PreviousSectionSpeed;
                }
                else
                {
                    currentAspect = AspectSequence[i - 1];
                    currentSpeed = SpeedSequence[i - 1];
                }

                Aspect nextAspect = AspectSequence[i];
                int nextSpeed = SpeedSequence[i];

                if (currentAspect == Aspect.Stop || (currentAspect == Aspect.StopAndProceed && TVMModel > TVMModelType.TVM300))
                {
                    Vcond[i] = TVMSpeedType._RRR;
                }
                else if (nextSpeed == 30 && currentSpeed == 30)
                {
                    Vcond[i] = TVMSpeedType._RRR;
                }
                else if (nextSpeed == 60 && currentSpeed == 60 && TVMModel > TVMModelType.TVM300)
                {
                    Vcond[i] = TVMSpeedType._60E;
                }
                else if (nextAspect == Aspect.Stop && (TVMModel > TVMModelType.TVM300 || currentSpeed == 80))
                {
                    Vcond[i] = TVMSpeedType._80E;
                }
                else if (nextSpeed == 80 && currentSpeed == 80)
                {
                    Vcond[i] = TVMSpeedType._80E;
                }
                else if (nextSpeed == 130 && currentSpeed == 130 && TVMModel > TVMModelType.TVM300)
                {
                    Vcond[i] = TVMSpeedType._130E;
                }
                else if (nextSpeed == 130 && currentSpeed == 170) // 130 kph HSL exit
                {
                    Vcond[i] = TVMSpeedType._130E;
                }
                else if (nextSpeed == 160 && currentSpeed == 160)
                {
                    Vcond[i] = TVMSpeedType._160E;
                }
                else if (nextSpeed == 160 && currentSpeed == 170) // 160 kph HSL exit
                {
                    Vcond[i] = TVMSpeedType._160E;
                }
                else if (TVMModel == TVMModelType.TVM300 && nextAspect == Aspect.StopAndProceed) // MSTS TVM300 160E
                {
                    Vcond[i] = TVMSpeedType._160E;
                }
                else if (nextSpeed == 170 && currentSpeed == 170f && TVMModel > TVMModelType.TVM300)
                {
                    Vcond[i] = TVMSpeedType._170E;
                }
                else if (nextSpeed == 200 && currentSpeed == 200 && TVMModel > TVMModelType.TVM300)
                {
                    Vcond[i] = TVMSpeedType._200V;
                }
                else if (nextSpeed == 220 && currentSpeed == 220)
                {
                    // TODO : 220V not available currently
                    Vcond[i] = TVMSpeedType._220E;
                }
                else if (nextSpeed == 220 && currentSpeed == 230) // 220 kph HSL exit
                {
                    // TODO : 220V not available currently
                    Vcond[i] = TVMSpeedType._220E;
                }
                else if (TVMModel == TVMModelType.TVM300 && nextAspect == Aspect.Approach_1) // MSTS TVM300 220E
                {
                    Vcond[i] = TVMSpeedType._220E;
                }
                else if (nextSpeed == 230 && currentSpeed == 230 && TVMModel > TVMModelType.TVM300)
                {
                    // TODO : 230V not available currently
                    Vcond[i] = TVMSpeedType._230E;
                }
                else if (nextSpeed == 270 && currentSpeed == 270)
                {
                    Vcond[i] = TVMSpeedType._270V;
                }
                else if (nextSpeed == 300 && currentSpeed == 300 || TVMModel == TVMModelType.TVM300)
                {
                    Vcond[i] = TVMSpeedType._300V;
                }
                else
                {
                    Vcond[i] = TVMSpeedType._320V;
                }
            }

            for (i = 0; i < TVMNumberOfBlockSections - 1; i++)
            {
                TVMSpeedType previousVcond;

                if (i == 0)
                {
                    previousVcond = PreviousVcond;
                }
                else
                {
                    previousVcond = Vcond[i - 1];
                }

                TVMSpeedType currentVcond = Vcond[i];
                TVMSpeedType nextVcond = Vcond[i + 1];

                if (currentVcond == TVMSpeedType._300V && TVMModel == TVMModelType.TVM300 || currentVcond == TVMSpeedType._320V)
                {
                    if (previousVcond == TVMSpeedType._230E || previousVcond == TVMSpeedType._270V)
                    {
                        if (nextVcond == TVMSpeedType._270V)
                        {
                            Vcond[i] = TVMSpeedType._270V;
                        }
                        else if (nextVcond == TVMSpeedType._300V)
                        {
                            Vcond[i] = TVMSpeedType._300V;
                        }
                    }
                }
            }

            Dictionary<TVMSpeedType, TVMSpeedType> TAB1 = TVMModel == TVMModelType.TVM300 ? TVM300Tab1 : TVM430SncfTab1;
            Dictionary<TVMSpeedType, TVMSpeedType> TAB2 = TVMModel == TVMModelType.TVM300 ? TVM300Tab2 : TVM430SncfTab2;

            // Calculate Vc, Ve, Va

            // Initialize last block section
            i = TVMNumberOfBlockSections - 1;
            Vc[i] = Vcond[i];
            Ve[i] = Min(TAB2[Vcond[i]], TAB1[Vc[i]]);
            Va[i] = TAB2[Vcond[i]];

            // Calculate the sequence
            for (i = TVMNumberOfBlockSections - 1; i > 0; i--)
            {
                Vc[i-1] = Min(Vcond[i-1], Ve[i]);
                Ve[i-1] = Min(TAB2[Vcond[i-1]], TAB1[Vc[i-1]]);
                Va[i-1] = TAB2[Vc[i]];
            }
        }

        protected void DetermineTvmAspect()
        {
            switch (TVMModel)
            {
                case TVMModelType.TVM300:
                    DetermineTvm300Aspect();
                    break;

                case TVMModelType.TVM430_V300:
                case TVMModelType.TVM430_V320:
                    DetermineTvm430Aspect();
                    break;
            }
        }

        protected void DetermineTvm300Aspect()
        {
            Tuple<TVMAspectType, bool, float> onBoardValues;

            Tuple<TVMSpeedType, TVMSpeedType, TVMSpeedType> triplet = new Tuple<TVMSpeedType, TVMSpeedType, TVMSpeedType>(Ve[0], Vc[0], Va[0]);

            if (TVM300DecodingTable.ContainsKey(triplet))
            {
                onBoardValues = TVM300DecodingTable[triplet];
            }
            else
            {
                triplet = new Tuple<TVMSpeedType, TVMSpeedType, TVMSpeedType>(Ve[0], Vc[0], TVMSpeedType.Any);

                if (TVM300DecodingTable.ContainsKey(triplet))
                {
                    onBoardValues = TVM300DecodingTable[triplet];
                }
                else
                {
                    onBoardValues = new Tuple<TVMAspectType, bool, float>(TVMAspectType._RRR, false, 35f);
                }
            }

            TVMAspectCommand = onBoardValues.Item1;
            TVMBlinkingCommand = onBoardValues.Item2;
            TVMStartControlSpeedMpS = TVMEndControlSpeedMpS = MpS.FromKpH(onBoardValues.Item3);
            TVMDecelerationMpS2 = 0f;
        }

        protected void DetermineTvm430Aspect()
        {
            Tuple<TVMAspectType, bool, float, float, float> onBoardValues;

            Tuple<TVMSpeedType, TVMSpeedType, TVMSpeedType> triplet = new Tuple<TVMSpeedType, TVMSpeedType, TVMSpeedType>(Ve[0], Vc[0], Va[0]);

            if (TVM430DecodingTable.ContainsKey(triplet))
            {
                onBoardValues = TVM430DecodingTable[triplet];
            }
            else
            {
                triplet = new Tuple<TVMSpeedType, TVMSpeedType, TVMSpeedType>(Ve[0], Vc[0], TVMSpeedType.Any);

                if (TVM430DecodingTable.ContainsKey(triplet))
                {
                    onBoardValues = TVM430DecodingTable[triplet];
                }
                else
                {
                    onBoardValues = new Tuple<TVMAspectType, bool, float, float, float>(TVMAspectType._RRR, false, 35f, 35f, 0f);
                }
            }
            
            if ((TVMAspectCommand != onBoardValues.Item1 || TVMBlinkingCommand != onBoardValues.Item2) && !TVM430AspectChangeTimer.Started)
            {
                TVMAspectCommand = onBoardValues.Item1;
                TVMBlinkingCommand = onBoardValues.Item2;
                TVMStartControlSpeedMpS = MpS.FromKpH(onBoardValues.Item3);
                TVMEndControlSpeedMpS = MpS.FromKpH(onBoardValues.Item4);
                TVMDecelerationMpS2 = onBoardValues.Item5;
            }
        }

        protected void UpdateTvmCovit()
        {
            switch (TVMModel)
            {
                case TVMModelType.TVM300:
                    UpdateTvm300Covit();
                    break;

                case TVMModelType.TVM430_V300:
                case TVMModelType.TVM430_V320:
                    UpdateTvm430Covit();
                    break;
            }
        }

        protected void UpdateTvm300Covit()
        {
            if (TVMCOVITInhibited)
            {
                TVMCOVITEmergencyBraking = false;
            }
            else
            {
                SetCurrentSpeedLimitMpS(TVMStartControlSpeedMpS);
                SetNextSpeedLimitMpS(TVMEndControlSpeedMpS);

                TVMCOVITEmergencyBraking = SpeedMpS() > TVMStartControlSpeedMpS;
            }
        }

        protected void UpdateTvm430Covit()
        {
            if (TVMCOVITInhibited)
            {
                TVMCOVITEmergencyBraking = false;
            }
            else
            {
                SetCurrentSpeedLimitMpS(TVMStartControlSpeedMpS);
                SetNextSpeedLimitMpS(TVMEndControlSpeedMpS);

                float SpeedCurveMpS = Math.Min(
                    SpeedCurve(
                        NextSignalDistanceM(0),
                        TVMEndControlSpeedMpS,
                        0,
                        0,
                        TVMDecelerationMpS2
                    ),
                    TVMStartControlSpeedMpS
                );

                TVMCOVITEmergencyBraking = SpeedMpS() > SpeedCurveMpS;
            }
        }

        protected void UpdateTvmDisplay()
        {
            switch (TVMModel)
            {
                case TVMModelType.TVM300:
                    UpdateTvm300Display();
                    break;

                case TVMModelType.TVM430_V300:
                case TVMModelType.TVM430_V320:
                    UpdateTvm430Display();
                    break;
            }
        }

        protected void UpdateTvm300Display()
        {
            UpdateTvmCabSignal(TVMAspectCommand, TVMBlinkingCommand, TVMAspectCommand != TVMAspectPreviousCycle);

            SetCabDisplayControl(TVM_Mask, TVMAspectCommand == TVMAspectType.None ? 0 : 1);

            SetCabDisplayControl(VY_CV, TVMCOVITEmergencyBraking || KarmEmergencyBraking ? 1 : 0);
            SetCabDisplayControl(VY_SECT, TVMOpenCircuitBreaker ? 1 : 0);
            SetCabDisplayControl(VY_SECT_AU, 0);
            SetCabDisplayControl(VY_BPT, TVMLowerPantograph ? 1 : 0);

            // Legacy
            Aspect aspect = TVM300MstsTranslation[TVMAspectCommand];
            SetNextSignalAspect(aspect);
        }

        protected void UpdateTvm430Display()
        {
            UpdateTvmCabSignal(TVMAspectCurrent, TVMBlinkingCurrent, false);

            if (TVMAspectCommand != TVMAspectCurrent || TVMBlinkingCommand != TVMBlinkingCurrent)
            {
                if (!TVM430AspectChangeTimer.Started)
                {
                    TVM430AspectChangeTimer.Start();
                }
                else
                {
                    if (TVM430AspectChangeTimer.Triggered)
                    {
                        UpdateTvmCabSignal(TVMAspectCommand, TVMBlinkingCommand, TVMAspectCommand != TVMAspectCurrent);

                        TVM430AspectChangeTimer.Stop();
                    }
                }
            }

            SetCabDisplayControl(TVM_Mask, TVMAspectCommand == TVMAspectType.None ? 0 : 1);

            SetCabDisplayControl(VY_CV, TVMCOVITEmergencyBraking || KarmEmergencyBraking ? 1 : 0);
            SetCabDisplayControl(VY_SECT, TVMOpenCircuitBreaker ? 1 : 0);
            SetCabDisplayControl(VY_SECT_AU, TVMOpenCircuitBreakerAutomatic ? 1 : 0);
            SetCabDisplayControl(VY_BPT, TVMLowerPantograph ? 1 : 0);

            // Legacy
            Aspect aspect;
            if (TVM430TrainSpeedLimitMpS <= MpS.FromKpH(300f))
            {
                aspect = TVM430S300MstsTranslation[TVMAspectCommand];
            }
            else
            {
                aspect = TVM430S320MstsTranslation[TVMAspectCommand];
            }
            SetNextSignalAspect(aspect);
        }

        protected void UpdateTvmCabSignal(TVMAspectType aspect, bool blinking, bool resetBlinking)
        {
            TVMAspectCurrent = aspect;
            TVMBlinkingCurrent = blinking;

            bool on = true;

            if (blinking)
            {
                if (!TVMBlinker.Started)
                {
                    TVMBlinker.Start();
                }

                if (resetBlinking)
                {
                    TVMBlinker.Stop();
                    TVMBlinker.Start();
                }

                on = TVMBlinker.On;
            }
            else
            {
                TVMBlinker.Stop();
            }

            switch (aspect)
            {
                case TVMAspectType.None:
                    SetCabDisplayControl(TVM_VL, 0);
                    SetCabDisplayControl(TVM_Ex1, 0);
                    SetCabDisplayControl(TVM_Ex2, 0);
                    SetCabDisplayControl(TVM_An1, 0);
                    SetCabDisplayControl(TVM_An2, 0);
                    break;

                case TVMAspectType._RRR:
                    SetCabDisplayControl(TVM_VL, 0);
                    SetCabDisplayControl(TVM_Ex1, 1);
                    SetCabDisplayControl(TVM_Ex2, 0);
                    SetCabDisplayControl(TVM_An1, 0);
                    SetCabDisplayControl(TVM_An2, 0);
                    break;

                case TVMAspectType._000:
                    SetCabDisplayControl(TVM_VL, 0);
                    SetCabDisplayControl(TVM_Ex1, 0);
                    SetCabDisplayControl(TVM_Ex2, 0);
                    SetCabDisplayControl(TVM_An1, 1);
                    SetCabDisplayControl(TVM_An2, 0);
                    break;

                case TVMAspectType._30E:
                    SetCabDisplayControl(TVM_VL, 0);
                    SetCabDisplayControl(TVM_Ex1, on ? 2 : 0);
                    SetCabDisplayControl(TVM_Ex2, 0);
                    SetCabDisplayControl(TVM_An1, 0);
                    SetCabDisplayControl(TVM_An2, 0);
                    break;

                case TVMAspectType._30A:
                    SetCabDisplayControl(TVM_VL, 0);
                    SetCabDisplayControl(TVM_Ex1, 0);
                    SetCabDisplayControl(TVM_Ex2, 0);
                    SetCabDisplayControl(TVM_An1, on ? 2 : 0);
                    SetCabDisplayControl(TVM_An2, 0);
                    break;

                case TVMAspectType._60E:
                    SetCabDisplayControl(TVM_VL, 0);
                    SetCabDisplayControl(TVM_Ex1, on ? 3 : 0);
                    SetCabDisplayControl(TVM_Ex2, 0);
                    SetCabDisplayControl(TVM_An1, 0);
                    SetCabDisplayControl(TVM_An2, 0);
                    break;

                case TVMAspectType._60A:
                    SetCabDisplayControl(TVM_VL, 0);
                    SetCabDisplayControl(TVM_Ex1, 0);
                    SetCabDisplayControl(TVM_Ex2, 0);
                    SetCabDisplayControl(TVM_An1, on ? 3 : 0);
                    SetCabDisplayControl(TVM_An2, 0);
                    break;

                case TVMAspectType._80E:
                    SetCabDisplayControl(TVM_VL, 0);
                    SetCabDisplayControl(TVM_Ex1, on ? 4 : 0);
                    SetCabDisplayControl(TVM_Ex2, 0);
                    SetCabDisplayControl(TVM_An1, 0);
                    SetCabDisplayControl(TVM_An2, 0);
                    break;

                case TVMAspectType._80A:
                    SetCabDisplayControl(TVM_VL, 0);
                    SetCabDisplayControl(TVM_Ex1, 0);
                    SetCabDisplayControl(TVM_Ex2, 0);
                    SetCabDisplayControl(TVM_An1, on ? 4 : 0);
                    SetCabDisplayControl(TVM_An2, 0);
                    break;

                case TVMAspectType._100E:
                    SetCabDisplayControl(TVM_VL, 0);
                    SetCabDisplayControl(TVM_Ex1, on ? 5 : 0);
                    SetCabDisplayControl(TVM_Ex2, 0);
                    SetCabDisplayControl(TVM_An1, 0);
                    SetCabDisplayControl(TVM_An2, 0);
                    break;

                case TVMAspectType._100A:
                    SetCabDisplayControl(TVM_VL, 0);
                    SetCabDisplayControl(TVM_Ex1, 0);
                    SetCabDisplayControl(TVM_Ex2, 0);
                    SetCabDisplayControl(TVM_An1, on ? 5 : 0);
                    SetCabDisplayControl(TVM_An2, 0);
                    break;

                case TVMAspectType._130E:
                    SetCabDisplayControl(TVM_VL, 0);
                    SetCabDisplayControl(TVM_Ex1, on ? 6 : 0);
                    SetCabDisplayControl(TVM_Ex2, 0);
                    SetCabDisplayControl(TVM_An1, 0);
                    SetCabDisplayControl(TVM_An2, 0);
                    break;

                case TVMAspectType._130A:
                    SetCabDisplayControl(TVM_VL, 0);
                    SetCabDisplayControl(TVM_Ex1, 0);
                    SetCabDisplayControl(TVM_Ex2, 0);
                    SetCabDisplayControl(TVM_An1, on ? 6 : 0);
                    SetCabDisplayControl(TVM_An2, 0);
                    break;

                case TVMAspectType._160E:
                    SetCabDisplayControl(TVM_VL, 0);
                    SetCabDisplayControl(TVM_Ex1, on ? 7 : 0);
                    SetCabDisplayControl(TVM_Ex2, 0);
                    SetCabDisplayControl(TVM_An1, 0);
                    SetCabDisplayControl(TVM_An2, 0);
                    break;

                case TVMAspectType._160A:
                    SetCabDisplayControl(TVM_VL, 0);
                    SetCabDisplayControl(TVM_Ex1, 0);
                    SetCabDisplayControl(TVM_Ex2, 0);
                    SetCabDisplayControl(TVM_An1, on ? 7 : 0);
                    SetCabDisplayControl(TVM_An2, 0);
                    break;

                case TVMAspectType._170E:
                    SetCabDisplayControl(TVM_VL, 0);
                    SetCabDisplayControl(TVM_Ex1, 0);
                    SetCabDisplayControl(TVM_Ex2, on ? 1 : 0);
                    SetCabDisplayControl(TVM_An1, 0);
                    SetCabDisplayControl(TVM_An2, 0);
                    break;

                case TVMAspectType._170A:
                    SetCabDisplayControl(TVM_VL, 0);
                    SetCabDisplayControl(TVM_Ex1, 0);
                    SetCabDisplayControl(TVM_Ex2, 0);
                    SetCabDisplayControl(TVM_An1, 0);
                    SetCabDisplayControl(TVM_An2, on ? 1 : 0);
                    break;

                case TVMAspectType._200V:
                    SetCabDisplayControl(TVM_VL, on ? 2 : 0);
                    SetCabDisplayControl(TVM_Ex1, 0);
                    SetCabDisplayControl(TVM_Ex2, 0);
                    SetCabDisplayControl(TVM_An1, 0);
                    SetCabDisplayControl(TVM_An2, 0);
                    break;

                case TVMAspectType._200A:
                    SetCabDisplayControl(TVM_VL, 0);
                    SetCabDisplayControl(TVM_Ex1, 0);
                    SetCabDisplayControl(TVM_Ex2, 0);
                    SetCabDisplayControl(TVM_An1, 0);
                    SetCabDisplayControl(TVM_An2, on ? 2 : 0);
                    break;

                case TVMAspectType._220E:
                    SetCabDisplayControl(TVM_VL, 0);
                    SetCabDisplayControl(TVM_Ex1, 0);
                    SetCabDisplayControl(TVM_Ex2, on ? 3 : 0);
                    SetCabDisplayControl(TVM_An1, 0);
                    SetCabDisplayControl(TVM_An2, 0);
                    break;

                case TVMAspectType._220V:
                    SetCabDisplayControl(TVM_VL, on ? 3 : 0);
                    SetCabDisplayControl(TVM_Ex1, 0);
                    SetCabDisplayControl(TVM_Ex2, 0);
                    SetCabDisplayControl(TVM_An1, 0);
                    SetCabDisplayControl(TVM_An2, 0);
                    break;

                case TVMAspectType._220A:
                    SetCabDisplayControl(TVM_VL, 0);
                    SetCabDisplayControl(TVM_Ex1, 0);
                    SetCabDisplayControl(TVM_Ex2, 0);
                    SetCabDisplayControl(TVM_An1, 0);
                    SetCabDisplayControl(TVM_An2, on ? 3 : 0);
                    break;

                case TVMAspectType._230E:
                    SetCabDisplayControl(TVM_VL, 0);
                    SetCabDisplayControl(TVM_Ex1, 0);
                    SetCabDisplayControl(TVM_Ex2, on ? 4 : 0);
                    SetCabDisplayControl(TVM_An1, 0);
                    SetCabDisplayControl(TVM_An2, 0);
                    break;

                case TVMAspectType._230V:
                    SetCabDisplayControl(TVM_VL, on ? 4 : 0);
                    SetCabDisplayControl(TVM_Ex1, 0);
                    SetCabDisplayControl(TVM_Ex2, 0);
                    SetCabDisplayControl(TVM_An1, 0);
                    SetCabDisplayControl(TVM_An2, 0);
                    break;

                case TVMAspectType._230A:
                    SetCabDisplayControl(TVM_VL, 0);
                    SetCabDisplayControl(TVM_Ex1, 0);
                    SetCabDisplayControl(TVM_Ex2, 0);
                    SetCabDisplayControl(TVM_An1, 0);
                    SetCabDisplayControl(TVM_An2, on ? 4 : 0);
                    break;

                case TVMAspectType._270V:
                    SetCabDisplayControl(TVM_VL, on ? 5 : 0);
                    SetCabDisplayControl(TVM_Ex1, 0);
                    SetCabDisplayControl(TVM_Ex2, 0);
                    SetCabDisplayControl(TVM_An1, 0);
                    SetCabDisplayControl(TVM_An2, 0);
                    break;

                case TVMAspectType._270A:
                    SetCabDisplayControl(TVM_VL, 0);
                    SetCabDisplayControl(TVM_Ex1, 0);
                    SetCabDisplayControl(TVM_Ex2, 0);
                    SetCabDisplayControl(TVM_An1, 0);
                    SetCabDisplayControl(TVM_An2, on ? 5 : 0);
                    break;

                case TVMAspectType._300V:
                    SetCabDisplayControl(TVM_VL, on ? 6 : 0);
                    SetCabDisplayControl(TVM_Ex1, 0);
                    SetCabDisplayControl(TVM_Ex2, 0);
                    SetCabDisplayControl(TVM_An1, 0);
                    SetCabDisplayControl(TVM_An2, 0);
                    break;

                case TVMAspectType._300A:
                    SetCabDisplayControl(TVM_VL, 0);
                    SetCabDisplayControl(TVM_Ex1, 0);
                    SetCabDisplayControl(TVM_Ex2, 0);
                    SetCabDisplayControl(TVM_An1, 0);
                    SetCabDisplayControl(TVM_An2, on ? 6 : 0);
                    break;

                case TVMAspectType._320V:
                    SetCabDisplayControl(TVM_VL, on ? 7 : 0);
                    SetCabDisplayControl(TVM_Ex1, 0);
                    SetCabDisplayControl(TVM_Ex2, 0);
                    SetCabDisplayControl(TVM_An1, 0);
                    SetCabDisplayControl(TVM_An2, 0);
                    break;
            }
        }

        protected void UpdateTvmSounds()
        {
            if (TVMAspectCurrent != TVMAspectType.None && TVMAspectPreviousCycle != TVMAspectType.None)
            {
                TVMClosedSignal = (TVMAspectPreviousCycle > TVMAspectCurrent) || (TVMBlinkingCurrent && !TVMBlinkingPreviousCycle);
                TVMOpenedSignal = (TVMAspectPreviousCycle < TVMAspectCurrent) || (!TVMBlinkingCurrent && TVMBlinkingPreviousCycle);
            }
        }

        public override void HandleEvent(TCSEvent evt, string message)
        {
            switch (evt)
            {
                case TCSEvent.AlerterPressed:
                    VACMAPressed = true;
                    break;

                case TCSEvent.AlerterReleased:
                    VACMAPressed = false;
                    break;

                case TCSEvent.ThrottleChanged:
                case TCSEvent.DynamicBrakeChanged:
                case TCSEvent.HornActivated:
                    if (VACMAPressedAlertTimer.Started || VACMAPressedEmergencyTimer.Started)
                    {
                        VACMAPressedAlertTimer.Start();
                        VACMAPressedEmergencyTimer.Start();
                    }
                    break;

                case TCSEvent.GenericTCSButtonPressed:
                    {
                        int tcsButton = -1;
                        if (Int32.TryParse(message, out tcsButton))
                        {
                            SetCabDisplayControl(tcsButton, 1);

                            switch (tcsButton)
                            {
                                // BP (AC) SF
                                case BP_AC_SF:
                                    RSOPressed = true;
                                    break;

                                // BP (A) LS (SF)
                                case BP_A_LS_SF:
                                    RSOCancelPressed = true;
                                    break;
                            }
                        }
                    }
                    break;

                case TCSEvent.GenericTCSButtonReleased:
                    {
                        int tcsButton = -1;
                        if (Int32.TryParse(message, out tcsButton))
                        {
                            SetCabDisplayControl(tcsButton, 0);

                            switch (tcsButton)
                            {
                                // BP (AC) SF
                                case BP_AC_SF:
                                    RSOPressed = false;
                                    break;

                                // BP (A) LS (SF)
                                case BP_A_LS_SF:
                                    RSOCancelPressed = false;
                                    break;

                                // BP AM V1 and BP AM V2
                                case BP_AM_V1:
                                case BP_AM_V2:
                                    TVMArmed = true;
                                    break;

                                // BP DM
                                case BP_DM:
                                    TVMArmed = false;
                                    break;
                            }
                        }
                    }
                    break;

                case TCSEvent.GenericTCSSwitchOn:
                    {
                        int tcsButton = -1;
                        if (Int32.TryParse(message, out tcsButton))
                        {
                            SetCabDisplayControl(tcsButton, 1);

                            switch (tcsButton)
                            {
                                // Z (ES) VA
                                case Z_ES_VA:
                                    VACMATest = true;
                                    break;
                            }
                        }
                    }
                    break;

                case TCSEvent.GenericTCSSwitchOff:
                    {
                        int tcsButton = -1;
                        if (Int32.TryParse(message, out tcsButton))
                        {
                            SetCabDisplayControl(tcsButton, 0);

                            switch (tcsButton)
                            {
                                // Z (ES) VA
                                case Z_ES_VA:
                                    VACMATest = false;
                                    break;
                            }
                        }
                    }
                    break;
            }
        }

        public override void HandleEvent(PowerSupplyEvent evt, string message)
        {
            switch (evt)
            {
                case PowerSupplyEvent.CloseCircuitBreakerButtonPressed:
                case PowerSupplyEvent.CloseTractionCutOffRelayButtonPressed:
                    RearmingButton = true;
                    break;

                case PowerSupplyEvent.CloseCircuitBreakerButtonReleased:
                case PowerSupplyEvent.CloseTractionCutOffRelayButtonReleased:
                    RearmingButton = false;
                    break;
            }
        }

        protected void UpdateVACMA()
        {
            if (VACMAPresent && Activated && IsAlerterEnabled())
            {
                if (SpeedMpS() >= VACMAActivationSpeedMpS || VACMATest)
                {
                    if (VACMAPressed && (!VACMAPressedAlertTimer.Started || !VACMAPressedEmergencyTimer.Started))
                    {
                        VACMAReleasedAlertTimer.Stop();
                        VACMAReleasedEmergencyTimer.Stop();
                        VACMAPressedAlertTimer.Start();
                        VACMAPressedEmergencyTimer.Start();
                    }
                    if (!VACMAPressed && (!VACMAReleasedAlertTimer.Started || !VACMAReleasedEmergencyTimer.Started))
                    {
                        VACMAReleasedAlertTimer.Start();
                        VACMAReleasedEmergencyTimer.Start();
                        VACMAPressedAlertTimer.Stop();
                        VACMAPressedEmergencyTimer.Stop();
                    }
                }
                else
                {
                    VACMAReleasedAlertTimer.Stop();
                    VACMAReleasedEmergencyTimer.Stop();
                    VACMAPressedAlertTimer.Stop();
                    VACMAPressedEmergencyTimer.Stop();
                }

                if (VACMAReleasedAlertTimer.Started && VACMAReleasedAlertTimer.Triggered)
                    TriggerSoundWarning1();
                else
                    TriggerSoundWarning2();

                if (VACMAPressedAlertTimer.Started && VACMAPressedAlertTimer.Triggered)
                    TriggerSoundAlert1();
                else
                    TriggerSoundAlert2();

                if (!VACMAEmergencyBraking && (VACMAPressedEmergencyTimer.Triggered || VACMAReleasedEmergencyTimer.Triggered))
                {
                    VACMAEmergencyBraking = true;
                    SetVigilanceEmergencyDisplay(true);
                }

                if (VACMAEmergencyBraking && SpeedMpS() < VACMAActivationSpeedMpS && RearmingButton)
                {
                    VACMAEmergencyBraking = false;
                    SetVigilanceEmergencyDisplay(false);
                }
            }
            else
            {
                // Reset everything
                VACMAReleasedAlertTimer.Stop();
                VACMAReleasedEmergencyTimer.Stop();
                VACMAPressedAlertTimer.Stop();
                VACMAPressedEmergencyTimer.Stop();
                VACMAEmergencyBraking = false;
                SetVigilanceEmergencyDisplay(false);

                TriggerSoundWarning2();
                TriggerSoundAlert2();
                return;
            }

            // VY SOS VAC
            SetCabDisplayControl(VY_SOS_VAC, VACMAEmergencyBraking ? 1 : 0);

            // VY (ES) FU
            SetCabDisplayControl(VY_ES_FU, VACMATest && IsBrakeEmergency() && !TractionAuthorization() ? 1 : 0);
        }

        protected void UpdateSignalPassed()
        {
            NormalSignalPassed = NextSignalDistanceM(0) > PreviousNormalSignalDistanceM;

            PreviousNormalSignalDistanceM = NextSignalDistanceM(0);

            DistantSignalPassed = NextDistanceSignalDistanceM() > PreviousDistantSignalDistanceM;

            PreviousDistantSignalDistanceM = NextDistanceSignalDistanceM();
        }
    }
}