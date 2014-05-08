﻿// COPYRIGHT 2010, 2012 by the Open Rails project.
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
using System.IO;
using ORTS.Scripting.Api;

namespace ORTS.Scripting.Script
{
    public class PBL2BrakeController : BrakeController
    {
        enum State
        {
            Overload,
            QuickRelease,
            Release,
            Lap,
            Apply,
            Emergency
        }

        public float OverloadValue { get; private set; }
        public float QuickReleaseValue { get; private set; }
        public float ReleaseValue { get; private set; }
        public float LapValue { get; private set; }
        public float ApplyValue { get; private set; }
        public float EmergencyValue { get; private set; }

        // brake controller values
        private float OverloadPressureBar = 0.4f;
        private float FirstDepressureBar = 0.3f;
        private float BrakeReleasedDepressureBar = 0.2f;

        protected float CurrentPressure;

        private State CurrentState;
        private State PreviousState;

        private bool FirstDepressure = false;
        private bool Neutral = false;
        private bool Overload = false;
        private bool QuickRelease = false;
        private bool Release = false;
        private bool Apply = false;

        public PBL2BrakeController()
        {
        }

        public override void Initialize()
        {
            foreach (MSTSNotch notch in Notches())
            {
                switch (notch.Type)
                {
                    case MSTSNotchType.Release:
                        ReleaseValue = notch.Value;
                        break;
                    case MSTSNotchType.FullQuickRelease:
                        OverloadValue = notch.Value;
                        QuickReleaseValue = notch.Value;
                        break;
                    case MSTSNotchType.Lap:
                        LapValue = notch.Value;
                        break;
                    case MSTSNotchType.Apply:
                    case MSTSNotchType.GSelfLap:
                    case MSTSNotchType.GSelfLapH:
                        ApplyValue = notch.Value;
                        break;
                    case MSTSNotchType.Emergency:
                        EmergencyValue = notch.Value;
                        break;
                }
            }
        }

        public override float Update(float elapsedSeconds)
        {
            if (Apply)
                SetCurrentValue(ApplyValue);
            else if (Release)
                SetCurrentValue(ReleaseValue);
            else
                SetCurrentValue(LapValue);

            return CurrentValue();
        }

        public override void UpdatePressure(ref float pressureBar, float elapsedClockSeconds, ref float epPressureBar)
        {
            if (!FirstDepressure && Apply && pressureBar > MaxPressureBar() - FirstDepressureBar)
                FirstDepressure = true;
            else if (FirstDepressure && pressureBar <= MaxPressureBar() - FirstDepressureBar)
                FirstDepressure = false;

            if (Apply && Overload)
                Overload = false;
            if (Apply && QuickRelease)
                QuickRelease = false;

            if (EmergencyBrakingPushButton() || TCSEmergencyBraking())
                CurrentState = State.Emergency;
            else if (
                Apply && pressureBar > MaxPressureBar() - FullServReductionBar()
                || !Overload && pressureBar > MaxPressureBar()
                || FirstDepressure && !Release && !QuickRelease && pressureBar > MaxPressureBar() - FirstDepressureBar
                )
                CurrentState = State.Apply;
            else if (Overload && pressureBar < MaxPressureBar() + OverloadPressureBar)
                CurrentState = State.Overload;
            else if (QuickRelease && !Neutral && pressureBar < MaxPressureBar())
                CurrentState = State.QuickRelease;
            else if (
                !Neutral && (
                    Release && pressureBar < MaxPressureBar()
                    || !FirstDepressure && pressureBar > MaxPressureBar() - BrakeReleasedDepressureBar && pressureBar < MaxPressureBar()
                    || pressureBar < MaxPressureBar() - FullServReductionBar()
                    )
                )
                CurrentState = State.Release;
            else
                CurrentState = State.Lap;

            switch (CurrentState)
            {
                case State.Overload:
                    SetUpdateValue(-1);

                    pressureBar += QuickReleaseRateBarpS() * elapsedClockSeconds;
                    epPressureBar -= QuickReleaseRateBarpS() * elapsedClockSeconds;

                    if (pressureBar > MaxPressureBar() + OverloadPressureBar)
                        pressureBar = MaxPressureBar() + OverloadPressureBar;
                    break;

                case State.QuickRelease:
                    SetUpdateValue(-1);

                    pressureBar += QuickReleaseRateBarpS() * elapsedClockSeconds;
                    epPressureBar -= QuickReleaseRateBarpS() * elapsedClockSeconds;

                    if (pressureBar > MaxPressureBar())
                        pressureBar = MaxPressureBar();
                    break;

                case State.Release:
                    SetUpdateValue(-1);

                    pressureBar += ReleaseRateBarpS() * elapsedClockSeconds;
                    epPressureBar -= ReleaseRateBarpS() * elapsedClockSeconds;

                    if (pressureBar > MaxPressureBar())
                        pressureBar = MaxPressureBar();
                    break;

                case State.Lap:
                    SetUpdateValue(0);
                    break;

                case State.Apply:
                    SetUpdateValue(1);

                    pressureBar -= ApplyRateBarpS() * elapsedClockSeconds;
                    epPressureBar += ApplyRateBarpS() * elapsedClockSeconds;

                    if (pressureBar < MaxPressureBar() - FullServReductionBar())
                        pressureBar = MaxPressureBar() - FullServReductionBar();
                    break;

                case State.Emergency:
                    SetUpdateValue(1);

                    pressureBar -= EmergencyRateBarpS() * elapsedClockSeconds;

                    if (pressureBar < 0)
                        pressureBar = 0;
                    break;
            }

            if (epPressureBar > MaxPressureBar())
                epPressureBar = MaxPressureBar();
            if (epPressureBar < 0)
                epPressureBar = 0;

            if (QuickRelease && pressureBar == MaxPressureBar())
                QuickRelease = false;

            PreviousState = CurrentState;
        }

        public override void UpdateEngineBrakePressure(ref float pressureBar, float elapsedClockSeconds)
        {
            switch (CurrentState)
            {
                case State.Release:
                    SetCurrentValue(ReleaseValue);
                    pressureBar -= ReleaseRateBarpS() * elapsedClockSeconds;
                    break;
                
                case State.Apply:
                    SetCurrentValue(ApplyValue);
                    pressureBar += ApplyRateBarpS() * elapsedClockSeconds;
                    break;
                
                case State.Emergency:
                    SetCurrentValue(EmergencyValue);
                    pressureBar += EmergencyRateBarpS() * elapsedClockSeconds;
                    break;
            }

            if (CurrentState > PreviousState)
                SetUpdateValue(1);
            else if (CurrentState < PreviousState)
                SetUpdateValue(-1);
            else
                SetUpdateValue(0);


            if (pressureBar > MaxPressureBar())
                pressureBar = MaxPressureBar();
            if (pressureBar < 0)
                pressureBar = 0;

            PreviousState = CurrentState;
        }

        public override void HandleEvent(BrakeControllerEvent evt)
        {
            switch (evt)
            {
                case BrakeControllerEvent.StartIncrease:
                    Apply = true;
                    break;

                case BrakeControllerEvent.StopIncrease:
                    Apply = false;
                    break;

                case BrakeControllerEvent.StartDecrease:
                    Release = true;
                    break;

                case BrakeControllerEvent.StopDecrease:
                    Release = false;
                    break;
            }
        }

        public override void HandleEvent(BrakeControllerEvent evt, float? value)
        {
            switch (evt)
            {
                case BrakeControllerEvent.StartIncrease:
                    Apply = true;
                    break;

                case BrakeControllerEvent.StartDecrease:
                    Release = true;
                    break;

                case BrakeControllerEvent.SetRDPercent:
                    if (value != null)
                    {
                        float newValue = value ?? 0F;
                        SetRDPercent(newValue);
                    }
                    break;

                case BrakeControllerEvent.SetCurrentValue:
                    if (value != null)
                    {
                        float newValue = value ?? 0F;
                        SetValue(newValue);
                    }
                    break;
            }
        }

        public override bool IsValid()
        {
            return true;
        }

        public override string GetStatus()
        {
            switch (CurrentState)
            {
                case State.Overload:
                    return "Overload";

                case State.QuickRelease:
                    return "Quick Release";

                case State.Release:
                    return "Release";

                case State.Lap:
                    return "Lap";

                case State.Apply:
                    return "Apply";

                case State.Emergency:
                    if (EmergencyBrakingPushButton())
                        return "Emergency Braking Push Button";
                    else if (TCSEmergencyBraking())
                        return "TCS Emergency Braking";
                    else if (TCSFullServiceBraking())
                        return "TCS Full Service Braking";
                    else
                        return "Emergency";

                default:
                    return "";
            }
        }

        private float SetRDPercent(float percent)
        {
            if (percent < 40)
            {
                Apply = true;
                Release = false;
            }
            else if (percent > 60)
            {
                Apply = false;
                Release = true;
            }
            else
            {
                Apply = false;
                Release = false;
            }

            return CurrentValue() * 100;
        }

        private void SetValue(float v)
        {
            SetCurrentValue(v);
            
            if (CurrentValue() == EmergencyValue)
            {
                Apply = false;
                Release = false;
                QuickRelease = false;
            }
            else if (CurrentValue() == ApplyValue)
            {
                Apply = true;
                Release = false;
                QuickRelease = false;
            }
            else if (CurrentValue() == LapValue)
            {
                Apply = false;
                Release = false;
                QuickRelease = false;
            }
            else if (CurrentValue() == ReleaseValue)
            {
                Apply = false;
                Release = true;
                QuickRelease = false;
            }
            else if (CurrentValue() == QuickReleaseValue)
            {
                Apply = false;
                Release = false;
                QuickRelease = true;
            }
        }
    }
}