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

using Orts.Common;
using Orts.Simulation;
using ORTS.Common;
using ORTS.Scripting.Api;
using System.Globalization;

namespace ORTS.Scripting.Script
{

    public class SNCFElectricPowerSupply : ElectricPowerSupply
    {
        private IIRFilter PantographFilter;
        private IIRFilter VoltageFilter;
        private Timer PowerOnTimer;
        private Timer AuxPowerOnTimer;

        public override void Initialize()
        {
            PantographFilter = new IIRFilter(IIRFilter.FilterTypes.Butterworth, 1, IIRFilter.HzToRad(0.7f), 0.001f);
            VoltageFilter = new IIRFilter(IIRFilter.FilterTypes.Butterworth, 1, IIRFilter.HzToRad(0.7f), 0.001f);

            PowerOnTimer = new Timer(this);
            PowerOnTimer.Setup(PowerOnDelayS());

            AuxPowerOnTimer = new Timer(this);
            AuxPowerOnTimer.Setup(AuxPowerOnDelayS());
        }

        public override void Update(float elapsedClockSeconds)
        {
            SetCurrentBatteryState(BatterySwitchOn() ? PowerSupplyState.PowerOn : PowerSupplyState.PowerOff);
            SetCurrentLowVoltagePowerSupplyState(BatterySwitchOn() ? PowerSupplyState.PowerOn : PowerSupplyState.PowerOff);
            SetCurrentCabPowerSupplyState(BatterySwitchOn() && MasterKeyOn() ? PowerSupplyState.PowerOn : PowerSupplyState.PowerOff);

            switch (CurrentPantographState())
            {
                case PantographState.Down:
                case PantographState.Lowering:
                case PantographState.Raising:
                    if (PowerOnTimer.Started)
                        PowerOnTimer.Stop();
                    if (AuxPowerOnTimer.Started)
                        AuxPowerOnTimer.Stop();

                    if (CurrentMainPowerSupplyState() == PowerSupplyState.PowerOn)
                    {
                        SignalEvent(Event.EnginePowerOff);
                        SetCurrentMainPowerSupplyState(PowerSupplyState.PowerOff);
                    }
                    SetCurrentAuxiliaryPowerSupplyState(PowerSupplyState.PowerOff);
                    SetPantographVoltageV(PantographFilter.Filter(0.0f, elapsedClockSeconds));
                    SetFilterVoltageV(VoltageFilter.Filter(0.0f, elapsedClockSeconds));
                    break;

                case PantographState.Up:
                    SetPantographVoltageV(PantographFilter.Filter(LineVoltageV(), elapsedClockSeconds));

                    switch (CurrentCircuitBreakerState())
                    {
                        case CircuitBreakerState.Open:
                            if (PowerOnTimer.Started)
                                PowerOnTimer.Stop();
                            if (AuxPowerOnTimer.Started)
                                AuxPowerOnTimer.Stop();

                            if (CurrentMainPowerSupplyState() == PowerSupplyState.PowerOn)
                            {
                                SignalEvent(Event.EnginePowerOff);
                                SetCurrentMainPowerSupplyState(PowerSupplyState.PowerOff);
                            }
                            SetCurrentAuxiliaryPowerSupplyState(PowerSupplyState.PowerOff);
                            SetFilterVoltageV(VoltageFilter.Filter(0.0f, elapsedClockSeconds));
                            break;

                        case CircuitBreakerState.Closed:
                            if (!PowerOnTimer.Started)
                                PowerOnTimer.Start();
                            if (!AuxPowerOnTimer.Started)
                                AuxPowerOnTimer.Start();

                            if (PowerOnTimer.Triggered && CurrentMainPowerSupplyState() == PowerSupplyState.PowerOff)
                            {
                                SignalEvent(Event.EnginePowerOn);
                                SetCurrentMainPowerSupplyState(PowerSupplyState.PowerOn);
                            }
                            SetCurrentAuxiliaryPowerSupplyState(AuxPowerOnTimer.Triggered ? PowerSupplyState.PowerOn : PowerSupplyState.PowerOff);
                            SetFilterVoltageV(VoltageFilter.Filter(PantographVoltageV(), elapsedClockSeconds));
                            break;
                    }
                    break;
            }

            SetCurrentDynamicBrakeAvailability(true);

            SetCurrentElectricTrainSupplyState(CurrentMainPowerSupplyState());
        }

        public override void HandleEvent(PowerSupplyEvent evt)
        {
            switch (evt)
            {
                case PowerSupplyEvent.CloseCircuitBreakerButtonPressed:
                case PowerSupplyEvent.GiveCircuitBreakerClosingAuthorization:
                    if (MasterKeyOn())
                    {
                        SignalEventToCircuitBreaker(evt);
                        SignalEventToTcs(evt);
                        SignalEventToOtherTrainVehicles(evt);
                    }
                    else
                    {
                        if (CultureInfo.CurrentCulture.TwoLetterISOLanguageName == "fr")
                        {
                            Message(ConfirmLevel.Warning, "Ce bouton ne peut être utilisé que si la boîte à leviers est déverrouillée");
                        }
                        else
                        {
                            Message(ConfirmLevel.Warning, "This button can only be used when the master key is turned on");
                        }
                    }
                    break;

                case PowerSupplyEvent.CloseCircuitBreakerButtonReleased:
                case PowerSupplyEvent.RemoveCircuitBreakerClosingAuthorization:
                    SignalEventToCircuitBreaker(evt);
                    SignalEventToTcs(evt);
                    SignalEventToOtherTrainVehicles(evt);
                    break;

                case PowerSupplyEvent.SwitchOnElectricTrainSupply:
                    if (MasterKeyOn())
                    {
                        SignalEventToElectricTrainSupplySwitch(evt);
                        SignalEventToTcs(evt);
                        SignalEventToOtherTrainVehicles(evt);
                    }
                    else
                    {
                        if (CultureInfo.CurrentCulture.TwoLetterISOLanguageName == "fr")
                        {
                            Message(ConfirmLevel.Warning, "Ce bouton ne peut être utilisé que si la boîte à leviers est déverrouillée");
                        }
                        else
                        {
                            Message(ConfirmLevel.Warning, "This button can only be used when the master key is turned on");
                        }
                    }
                    break;

                case PowerSupplyEvent.SwitchOffElectricTrainSupply:
                    SignalEventToElectricTrainSupplySwitch(evt);
                    SignalEventToTcs(evt);
                    SignalEventToOtherTrainVehicles(evt);
                    break;

                case PowerSupplyEvent.CloseBatterySwitch:
                case PowerSupplyEvent.CloseBatterySwitchButtonPressed:
                case PowerSupplyEvent.CloseBatterySwitchButtonReleased:
                case PowerSupplyEvent.OpenBatterySwitch:
                case PowerSupplyEvent.OpenBatterySwitchButtonPressed:
                case PowerSupplyEvent.OpenBatterySwitchButtonReleased:
                    SignalEventToBatterySwitch(evt);
                    SignalEventToTcs(evt);
                    SignalEventToOtherTrainVehicles(evt);
                    break;

                case PowerSupplyEvent.TurnOnMasterKey:
                case PowerSupplyEvent.TurnOffMasterKey:
                    if (!CircuitBreakerDriverClosingAuthorization() && !CircuitBreakerDriverClosingOrder() && !CircuitBreakerDriverOpeningOrder())
                    {
                        SignalEventToMasterKey(evt);
                        SignalEventToTcs(evt);
                    }
                    else
                    {
                        if (CultureInfo.CurrentCulture.TwoLetterISOLanguageName == "fr")
                        {
                            Message(ConfirmLevel.Warning, "Tous les leviers de la rangée supérieure de la boîte à leviers doivent être bas (sauf le bouton de maintien de service) pour verrouiller la boîte à leviers");
                        }
                        else
                        {
                            Message(ConfirmLevel.Warning, "All upper levers must be down (except service hold button) in order to switch off the master key");
                        }
                    }
                    break;
            }
        }

        public override void HandleEventFromLeadLocomotive(PowerSupplyEvent evt)
        {
            switch (evt)
            {
                case PowerSupplyEvent.CloseCircuitBreakerButtonPressed:
                case PowerSupplyEvent.GiveCircuitBreakerClosingAuthorization:
                    SignalEventToCircuitBreaker(evt);
                    SignalEventToTcs(evt);
                    break;

                case PowerSupplyEvent.CloseCircuitBreakerButtonReleased:
                case PowerSupplyEvent.RemoveCircuitBreakerClosingAuthorization:
                    SignalEventToCircuitBreaker(evt);
                    SignalEventToTcs(evt);
                    break;

                case PowerSupplyEvent.SwitchOnElectricTrainSupply:
                case PowerSupplyEvent.SwitchOffElectricTrainSupply:
                    SignalEventToElectricTrainSupplySwitch(evt);
                    SignalEventToTcs(evt);
                    break;

                case PowerSupplyEvent.CloseBatterySwitch:
                case PowerSupplyEvent.CloseBatterySwitchButtonPressed:
                case PowerSupplyEvent.CloseBatterySwitchButtonReleased:
                case PowerSupplyEvent.OpenBatterySwitch:
                case PowerSupplyEvent.OpenBatterySwitchButtonPressed:
                case PowerSupplyEvent.OpenBatterySwitchButtonReleased:
                    SignalEventToBatterySwitch(evt);
                    SignalEventToTcs(evt);
                    break;
            }
        }
    }

}