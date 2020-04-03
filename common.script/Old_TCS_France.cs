// COPYRIGHT 2014 by the Open Rails project.
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

using System;
using System.Collections.Generic;
using ORTS.Common;
using ORTS.Scripting.Api;

namespace ORTS.Scripting.Script
{
    public class Old_TCS_France : TrainControlSystem
    {
        enum CCS
        {
            RS,         // RS only
            DAAT,       // RS + DAAT
            TVM300      // RS partially inhibited + TVM300
        }

        CCS ActiveCCS = CCS.RS;
        CCS PreviousCCS = CCS.RS;

    // Train parameters
        bool VACMAPresent;                                  // VACMA
        bool RSPresent;                                     // RS
        bool DAATPresent;                                   // DAAT
        bool TVM300Present;                                 // TVM300

    // RS (Répétition des Signaux / Signal Repetition)
        // Parameters
        const float RSDelayBeforeEmergencyBrakingS = 4f;

        // Variables
        bool RSEmergencyBraking = false;
        bool RSType1Inhibition = false;                     // Inhibition 1 : Reverse
        bool RSType2Inhibition = false;                     // Inhibition 2 : Train on HSL
        bool RSType3Inhibition = false;                     // Inhibition 3 : TVM COVIT not inhibited
        bool RSClosedSignal = false;
        bool RSPreviousClosedSignal = false;
        bool RSOpenedSignal = false;
        bool RSPreviousOpenedSignal = false;
        Timer RSEmergencyTimer;

    // DAAT (Dispositif d'Arrêt Automatique des Trains / Automatic Train Stop System)
        // Not implemented

    // TVM COVIT common
        // Parameters
        bool TVMCOVITInhibited = false;

        // Variables
        bool TVMArmed = false;
        bool TVMCOVITEmergencyBraking = false;

        Aspect TVMAspect;
        Aspect TVMPreviousAspect;
        bool TVMClosedSignal;
        bool TVMPreviousClosedSignal;
        bool TVMOpenedSignal;
        bool TVMPreviousOpenedSignal;

    // TVM300 COVIT (Transmission Voie Machine 300 COntrôle de VITesse / Track Machine Transmission 300 Speed control)
        // Constants
        Dictionary<Aspect, float> TVM300CurrentSpeedLimitsKph = new Dictionary<Aspect, float>
        {
            {Aspect.None, 300f},
            {Aspect.Clear_2, 300f},
            {Aspect.Clear_1, 300f},
            {Aspect.Approach_3, 270f},
            {Aspect.Approach_2, 270f},
            {Aspect.Approach_1, 220f},
            {Aspect.Restricted, 220f},
            {Aspect.StopAndProceed, 160f},
            {Aspect.Stop, 160f},
            {Aspect.Permission, 30f}
        };
        Dictionary<Aspect, float> TVM300NextSpeedLimitsKph = new Dictionary<Aspect, float>
        {
            {Aspect.None, 300f},
            {Aspect.Clear_2, 300f},
            {Aspect.Clear_1, 270f},
            {Aspect.Approach_3, 270f},
            {Aspect.Approach_2, 220f},
            {Aspect.Approach_1, 220f},
            {Aspect.Restricted, 160f},
            {Aspect.StopAndProceed, 160f},
            {Aspect.Stop, 30f},
            {Aspect.Permission, 30f}
        };

        // Parameters
        float TVM300TrainSpeedLimitMpS;

        // Variables
        float TVM300CurrentSpeedLimitMpS;
        float TVM300NextSpeedLimitMpS;
        float TVM300EmergencySpeedMpS;

    // Vigilance monitoring (VACMA)
        // Parameters
        float VACMAActivationSpeedMpS;
        float VACMAReleasedAlertDelayS;
        float VACMAReleasedEmergencyDelayS;
        float VACMAPressedAlertDelayS;
        float VACMAPressedEmergencyDelayS;

        // Variables
        bool VACMAEmergencyBraking = false;
        bool VACMAPressed = false;
        Timer VACMAPressedAlertTimer;
        Timer VACMAPressedEmergencyTimer;
        Timer VACMAReleasedAlertTimer;
        Timer VACMAReleasedEmergencyTimer;

    // Other variables
        bool ExternalEmergencyBraking = false;
        float PreviousSignalDistanceM = 0f;
        bool SignalPassed = false;

        public Old_TCS_France() { }

        public override void Initialize()
        {
            // General section
            VACMAPresent = GetBoolParameter("General", "VACMAPresent", true);
            RSPresent = GetBoolParameter("General", "RSPresent", true);
            DAATPresent = GetBoolParameter("General", "DAATPresent", false);
            TVM300Present = GetBoolParameter("General", "TVM300Present", false);

            // RS section
            RSEmergencyTimer = new Timer(this);
            RSEmergencyTimer.Setup(RSDelayBeforeEmergencyBrakingS);

            // TVM common section
            TVMCOVITInhibited = GetBoolParameter("TVM", "CovitInhibited", false);

            // TVM300 section
            TVM300TrainSpeedLimitMpS = MpS.FromKpH(GetFloatParameter("TVM300", "TrainSpeedLimitKpH", 300f));

            // VACMA section
            VACMAActivationSpeedMpS = MpS.FromKpH(GetFloatParameter("VACMA", "ActivationSpeedKpH", 3f));
            VACMAReleasedAlertDelayS = GetFloatParameter("VACMA", "ReleasedAlertDelayS", 2.5f);
            VACMAReleasedEmergencyDelayS = GetFloatParameter("VACMA", "ReleasedEmergencyDelayS", 5f);
            VACMAPressedAlertDelayS = GetFloatParameter("VACMA", "PressedAlertDelayS", 55f);
            VACMAPressedEmergencyDelayS = GetFloatParameter("VACMA", "PressedEmergencyDelayS", 60f);

            VACMAPressedAlertTimer = new Timer(this);
            VACMAPressedAlertTimer.Setup(VACMAPressedAlertDelayS);
            VACMAPressedEmergencyTimer = new Timer(this);
            VACMAPressedEmergencyTimer.Setup(VACMAPressedEmergencyDelayS);
            VACMAReleasedAlertTimer = new Timer(this);
            VACMAReleasedAlertTimer.Setup(VACMAReleasedAlertDelayS);
            VACMAReleasedEmergencyTimer = new Timer(this);
            VACMAReleasedEmergencyTimer.Setup(VACMAReleasedEmergencyDelayS);

            Activated = true;
            PreviousSignalDistanceM = 0f;
        }

        public override void Update()
        {
            UpdateSignalPassed();

            if (IsAlerterEnabled() && VACMAPresent)
            {
                UpdateVACMA();
            }

            if (IsTrainControlEnabled())
            {
                if (RSPresent)
                {
                    UpdateRS();
                }

                if (CurrentPostSpeedLimitMpS() <= MpS.FromKpH(220f))
                {
                    if (!DAATPresent)
                    {
                        ActiveCCS = CCS.RS;
                    }
                    else
                    {
                        ActiveCCS = CCS.DAAT;
                    }

                    SetNextSignalAspect(Aspect.Clear_1);
                }
                else
                {
                    // High speed line = TVM active
                    if (TVM300Present)
                    {
                        ActiveCCS = CCS.TVM300;

                        UpdateTVM300Display();
                        UpdateTVM300COVIT();
                    }
                }

                SetEmergencyBrake(
                    RSEmergencyBraking
                    || TVMCOVITEmergencyBraking
                    || VACMAEmergencyBraking
                    || ExternalEmergencyBraking
                );

                SetPenaltyApplicationDisplay(IsBrakeEmergency());

                SetPowerAuthorization(!RSEmergencyBraking
                    && !TVMCOVITEmergencyBraking
                    && !VACMAEmergencyBraking
                );

                if (ActiveCCS != CCS.TVM300)
                {
                    TVMAspect = Aspect.None;
                    TVMPreviousAspect = Aspect.None;
                }

                RSType1Inhibition = IsDirectionReverse();
                RSType2Inhibition = TVM300Present && ActiveCCS == CCS.TVM300;
                RSType3Inhibition = TVM300Present && !TVMCOVITInhibited;

                PreviousCCS = ActiveCCS;
            }
        }

        public override void SetEmergency(bool emergency)
        {
            ExternalEmergencyBraking = emergency;
        }

        protected void UpdateRS()
        {
            if (NextSignalDistanceM(0) < 2f
                && (ActiveCCS == CCS.RS || ActiveCCS == CCS.DAAT)
                && SpeedMpS() > 0)
            {
                if (NextSignalAspect(0) == Aspect.Stop
                    || NextSignalAspect(0) == Aspect.StopAndProceed
                    || NextSignalAspect(0) == Aspect.Restricted)
                    RSClosedSignal = true;
                else if (NextSignalAspect(1) == Aspect.Stop
                    || NextSignalAspect(1) == Aspect.StopAndProceed)
                    RSClosedSignal = true;
                else if (NextSignalSpeedLimitMpS(1) >= 0f && NextSignalSpeedLimitMpS(1) < MpS.FromKpH(160f))
                    RSClosedSignal = true;
                else
                    RSOpenedSignal = true;
            }
            if (SignalPassed)
                RSClosedSignal = RSOpenedSignal = false;

            if (RSClosedSignal && !RSPreviousClosedSignal && !RSType1Inhibition
                || TVM300Present && TVMClosedSignal && !TVMPreviousClosedSignal)
            {
                RSEmergencyTimer.Start();
                TriggerSoundPenalty1();
            }

            if (RSEmergencyTimer.Started && RSEmergencyTimer.Triggered)
            {
                RSEmergencyTimer.Stop();
                TriggerSoundPenalty2();
            }

            if (RSOpenedSignal && !RSPreviousOpenedSignal && !RSType1Inhibition
                || TVM300Present && TVMOpenedSignal && !TVMPreviousOpenedSignal)
                TriggerSoundInfo1();

            RSPreviousClosedSignal = RSClosedSignal;
            RSPreviousOpenedSignal = RSOpenedSignal;
            TVMPreviousClosedSignal = TVMClosedSignal;
            TVMPreviousOpenedSignal = TVMOpenedSignal;
        }

        protected void UpdateTVM300Display()
        {
            UpdateTVMAspect(NextSignalAspect(0));
        }

        protected void UpdateTVMAspect(Aspect aspect)
        {
            TVMPreviousAspect = TVMAspect;
            TVMAspect = aspect;
            SetNextSignalAspect(aspect);

            if (TVMAspect != Aspect.None && TVMPreviousAspect != Aspect.None)
            {
                TVMClosedSignal = (TVMPreviousAspect < TVMAspect);
                TVMOpenedSignal = (TVMPreviousAspect > TVMAspect);
            }
        }

        protected void UpdateTVM300COVIT()
        {
            TVM300CurrentSpeedLimitMpS = MpS.FromKpH(TVM300CurrentSpeedLimitsKph[NextSignalAspect(0)]);
            TVM300NextSpeedLimitMpS = MpS.FromKpH(TVM300NextSpeedLimitsKph[NextSignalAspect(0)]);

            SetNextSpeedLimitMpS(TVM300NextSpeedLimitMpS);
            SetCurrentSpeedLimitMpS(TVM300CurrentSpeedLimitMpS);

            TVM300EmergencySpeedMpS = TVM300GetEmergencySpeed(TVM300CurrentSpeedLimitMpS);

            if (!TVMCOVITEmergencyBraking && SpeedMpS() > TVM300CurrentSpeedLimitMpS + TVM300EmergencySpeedMpS)
                TVMCOVITEmergencyBraking = true;

            if (TVMCOVITEmergencyBraking && SpeedMpS() <= TVM300CurrentSpeedLimitMpS)
                TVMCOVITEmergencyBraking = false;
        }

        private float TVM300GetEmergencySpeed(float speedLimit)
        {
            float emergencySpeed = 0f;

            if (speedLimit <= MpS.FromKpH(80f))
                emergencySpeed = MpS.FromKpH(5f);
            else if (speedLimit <= MpS.FromKpH(160f))
                emergencySpeed = MpS.FromKpH(10f);
            else
                emergencySpeed = MpS.FromKpH(15f);

            return emergencySpeed;
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
            }
        }

        protected void UpdateVACMA()
        {
            if (!Activated || !IsAlerterEnabled() || SpeedMpS() < VACMAActivationSpeedMpS)
            {
                VACMAReleasedAlertTimer.Stop();
                VACMAReleasedEmergencyTimer.Stop();
                VACMAPressedAlertTimer.Stop();
                VACMAPressedEmergencyTimer.Stop();
                VACMAEmergencyBraking = false;

                TriggerSoundWarning2();
                TriggerSoundAlert2();
                return;
            }

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

            if (VACMAEmergencyBraking && SpeedMpS() < 0.1f)
            {
                VACMAEmergencyBraking = false;
                SetVigilanceEmergencyDisplay(false);
            }
        }

        protected void UpdateSignalPassed()
        {
            SignalPassed = NextSignalDistanceM(0) > PreviousSignalDistanceM && SpeedMpS() > 0;

            PreviousSignalDistanceM = NextSignalDistanceM(0);
        }
    }
}