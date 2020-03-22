using System;
using KSP.UI.Screens;
using UnityEngine;

/*
 * Adoption of GravityTurn Ascent with airbreathing engines like Rapiers.
 * 
 * Start in closed cycle to gain some speed, then turn to air breathing for ascent until around 1400 m/s or 26000m then thrust will drop and we need to switch back to closed cycle
 * first switch is controlled by input, second by looking at thrust 
 * 
 * further optimisation: in mixed configuration with classic rocket engines we throttle them if TWR is above choosen limit to save rocket fuel
 * 
 */

namespace MuMech
{
    public class MechJebModuleAscentBreathingGT : MechJebModuleAscentBase
    {
        public MechJebModuleAscentBreathingGT(MechJebCore core) : base(core) { }

        public Double turnStartAltitude = 500;

        [Persistent(pass = (int)(Pass.Type | Pass.Global))]
            public EditableDoubleMult turnStartVelocity = new EditableDoubleMult(50);
        [Persistent(pass = (int)(Pass.Type | Pass.Global))]
            public EditableDoubleMult turnStartPitch = new EditableDoubleMult(25);
        [Persistent(pass = (int)(Pass.Type | Pass.Global))]
            public EditableDoubleMult startBreathingSpeed = new EditableDoubleMult(220);
        [Persistent(pass = (int)(Pass.Type | Pass.Global))]
            public EditableDoubleMult minTWRthrottle = new EditableDoubleMult(2);
        [ValueInfoItem("Local TWR", InfoItem.Category.Vessel, format = "F2", showInEditor = false)]
        public double LocalTWR()
        {
            return vesselState.thrustAvailable / (vesselState.mass * vesselState.gravityForce.magnitude);
        }

        public override void OnModuleEnabled()
        {
            base.OnModuleEnabled();
            mode = AscentMode.VERTICAL_ASCENT;
        }

        public override void OnModuleDisabled()
        {
            base.OnModuleDisabled();
        }

        public bool IsVerticalAscent(double altitude, double velocity)
        {
            if (altitude < turnStartAltitude && velocity < turnStartVelocity)
            {
                return true;
            }
            return false;
        }

        bool ControlThrottle()
        {
            /* returns true if we did limit throttle */

            if (autopilot.autoThrottle)
            {
                core.thrust.targetThrottle = ThrottleToRaiseApoapsis(orbit.ApR, autopilot.desiredOrbitAltitude + mainBody.Radius);
                if (core.thrust.targetThrottle < 1.0F)
                {
                    status = "Fine tuning altitude";
                    return true;
                }
            }
            return false;
        }

        enum EngineMode { LiftOff, AirBreathing, ClosedCycle }
        EngineMode engineMode = EngineMode.LiftOff;
        ModuleEngines activeEngine; //random engine in air breathing mode to check thrust
        private float otherThrottle = 100.0f;

        void ControlAirBreathing()
        {
            if (engineMode == EngineMode.LiftOff && vesselState.speedSurface > startBreathingSpeed)
            {
                engineMode = EngineMode.AirBreathing;
                ApplyEngineMode();
                //Debug.Log("AscentBreathingGT: switching to air breathing mode");
            }
            else if (engineMode == EngineMode.AirBreathing && vesselState.speedSurface > 1.2* startBreathingSpeed && activeEngine.finalThrust < 0.15*activeEngine.maxThrust )
            {
                engineMode = EngineMode.ClosedCycle;
                ApplyEngineMode();
                //Debug.Log("AscentBreathingGT: switching to closed cylce mode");
            }
            else if (engineMode == EngineMode.AirBreathing && otherThrottle > 0f && LocalTWR() > minTWRthrottle )
            {
                otherThrottle -= 0.5f;
                ApplyOtherThrottle();
            }
            else if (engineMode == EngineMode.AirBreathing && otherThrottle < 100f && LocalTWR() < minTWRthrottle)
            {
                otherThrottle += 0.5f;
                ApplyOtherThrottle();
            }
        }

        void ApplyEngineMode()
        {
            foreach (MultiModeEngine engine in vessel.FindPartModulesImplementing<MultiModeEngine>())
            {
                if (engine.primaryEngineID == engineMode.ToString())
                {
                    engine.SetPrimary(true);
                    activeEngine = engine.PrimaryEngine;
                }
                else
                {
                    engine.SetSecondary(true);
                    activeEngine = engine.SecondaryEngine;
                }
            }
        }

        // Other = non multi mode engines, assumed to be rocket engines and we want to save fuel by not using them
        private void ApplyOtherThrottle()
        {
            foreach (ModuleEngines engine in vessel.FindPartModulesImplementing<ModuleEngines>())
            {
                if (engine.isOperational && engine.part.FindModuleImplementing<MultiModeEngine>() == null)
                {
                    engine.thrustPercentage = otherThrottle;
                }
            }
        }

        enum AscentMode { VERTICAL_ASCENT, INITIATE_TURN, GRAVITY_TURN, COAST_TO_APOAPSIS, EXIT };
        AscentMode mode;

        public override bool DriveAscent(FlightCtrlState s)
        {
            ControlAirBreathing();

            switch (mode)
            {
                case AscentMode.VERTICAL_ASCENT:
                    DriveVerticalAscent(s);
                    break;

                case AscentMode.INITIATE_TURN:
                    DriveInitiateTurn(s);
                    break;

                case AscentMode.GRAVITY_TURN:
                    DriveGravityTurn(s);
                    break;

                case AscentMode.COAST_TO_APOAPSIS:
                    DriveCoastToApoapsis(s);
                    break;

                case AscentMode.EXIT:
                    return false;
            }
            return true;
        }

        void DriveVerticalAscent(FlightCtrlState s)
        {
            if (!IsVerticalAscent(vesselState.altitudeTrue, vesselState.speedSurface)) mode = AscentMode.INITIATE_TURN;
            if (autopilot.autoThrottle && orbit.ApA > autopilot.desiredOrbitAltitude) mode = AscentMode.GRAVITY_TURN;

            //during the vertical ascent we just thrust straight up at max throttle
            attitudeTo(90);

            core.attitude.AxisControl(!vessel.Landed, !vessel.Landed, !vessel.Landed && vesselState.altitudeBottom > 50);

            if (autopilot.autoThrottle) core.thrust.targetThrottle = 1.0F;

            if (!vessel.LiftedOff() || vessel.Landed) status = "Awaiting liftoff for breathing GT";
            else status = "Vertical ascent for breathing GT";
        }

        void DriveInitiateTurn(FlightCtrlState s)
        {
            //stop the intermediate "burn" when our apoapsis reaches the desired altitude
            if (orbit.ApA > autopilot.desiredOrbitAltitude)
            {
                mode = AscentMode.COAST_TO_APOAPSIS;
                return;
            }

            if ((90 - turnStartPitch) >= srfvelPitch())
            {
                mode = AscentMode.GRAVITY_TURN;
                return;
            }

            //if we've fallen below the turn start altitude, go back to vertical ascent
            if (IsVerticalAscent(vesselState.altitudeTrue, vesselState.speedSurface))
            {
                mode = AscentMode.VERTICAL_ASCENT;
                return;
            }

            attitudeTo(90 - turnStartPitch);

            // do Throttle control and set status if we are not fine tuning
            if (!ControlThrottle())
                status = "Initiate breathing gravity turn";
        }

        void DriveGravityTurn(FlightCtrlState s)
        {
            //stop the intermediate "burn" when our apoapsis reaches the desired altitude
            if (orbit.ApA > autopilot.desiredOrbitAltitude)
            {
                mode = AscentMode.COAST_TO_APOAPSIS;
                return;
            }

            // srfvelPitch == zero AoA
            attitudeTo(srfvelPitch());

            // do Throttle control and set status if we are not fine tuning
            if (!ControlThrottle())
                status = "Breathing Gravity turn";
        }

        void DriveCoastToApoapsis(FlightCtrlState s)
        {
            core.thrust.targetThrottle = 0;

            double apoapsisSpeed = orbit.SwappedOrbitalVelocityAtUT(orbit.NextApoapsisTime(vesselState.time)).magnitude;

            if (vesselState.altitudeASL > mainBody.RealMaxAtmosphereAltitude())
            {
                mode = AscentMode.EXIT;
                core.warp.MinimumWarp();
                return;
            }

            //if our apoapsis has fallen too far, resume the gravity turn
            if (orbit.ApA < autopilot.desiredOrbitAltitude - 1000.0)
            {
                mode = AscentMode.GRAVITY_TURN;
                core.warp.MinimumWarp();
                return;
            }

            core.thrust.targetThrottle = 0;

            // follow surface velocity to reduce flipping
            attitudeTo(srfvelPitch());

            if (autopilot.autoThrottle && orbit.ApA < autopilot.desiredOrbitAltitude)
            {
                core.warp.WarpPhysicsAtRate(1);
                core.thrust.targetThrottle = ThrottleToRaiseApoapsis(orbit.ApR, autopilot.desiredOrbitAltitude + mainBody.Radius);
            }
            else
            {
                if (core.node.autowarp)
                {
                    //warp at x2 physical warp:
                    core.warp.WarpPhysicsAtRate(2);
                }
            }

            status = "Coasting to edge of atmosphere";
        }
    }
}
