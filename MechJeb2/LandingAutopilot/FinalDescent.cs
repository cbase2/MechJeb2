using System;
using UnityEngine;
using System.Linq;

namespace MuMech
{
    namespace Landing
    {
        public class FinalDescent : AutopilotStep
        {

            private IDescentSpeedPolicy aggressivePolicy;
            private bool waitForChute = false;
            private bool useChute = false;

            public FinalDescent(MechJebCore core) : base(core)
            {
                useChute = mainBody.atmosphere && core.landing.deployChutes && vesselState.parachutes.Count > 0;                
            }

            public override AutopilotStep OnFixedUpdate()
            {
                double minalt = Math.Min(vesselState.altitudeBottom, Math.Min(vesselState.altitudeASL, vesselState.altitudeTrue));

                if (core.node.autowarp && aggressivePolicy != null)
                {
                    double maxVel = 1.02 * aggressivePolicy.MaxAllowedSpeed(vesselState.CoM - mainBody.position, vesselState.surfaceVelocity);

                    double diffPercent = ((maxVel / vesselState.speedSurface) - 1) * 100;
                    
                    if (minalt > 200  && diffPercent > 0 && (Vector3d.Angle(vesselState.forward, -vesselState.surfaceVelocity) < 45))
                        core.warp.WarpRegularAtRate((float)(diffPercent * diffPercent * diffPercent));
                    else
                        core.warp.MinimumWarp(true);
                    
                }
                return this;
            }

            MovingAverage finalDifferenceTarget = new MovingAverage(120, 100);
            public override AutopilotStep Drive(FlightCtrlState s)
            {
                if (vessel.LandedOrSplashed)
                {
                    core.landing.StopLanding();
                    vessel.ActionGroups.SetGroup(KSPActionGroup.SAS, true);
                    vessel.Autopilot.Enable(VesselAutopilot.AutopilotMode.RadialIn);
                    return null;
                }

                if (useChute)
                    waitForChute = vesselState.parachutes.Any(p => p.Anim.isPlaying);

                double minalt = Math.Min(vesselState.altitudeBottom, Math.Min(vesselState.altitudeASL, vesselState.altitudeTrue));

                if (vesselState.limitedMaxThrustAccel < vesselState.gravityForce.magnitude)
                {
                    // if we have TWR < 1, just try as hard as we can to decelerate:
                    // (we need this special case because otherwise the calculations spit out NaN's)
                    core.thrust.tmode = MechJebModuleThrustController.TMode.KEEP_VERTICAL;
                    core.thrust.trans_kill_h = true;
                    core.thrust.trans_spd_act = 0;
                }
                else if (minalt > 200)
                {
                    if ((vesselState.surfaceVelocity.magnitude > 5) && (Vector3d.Angle(vesselState.surfaceVelocity, vesselState.up) < 80))
                    {
                        // if we have positive vertical velocity, point up and don't thrust:
                        core.attitude.attitudeTo(Vector3d.up, AttitudeReference.SURFACE_NORTH, null);
                        core.thrust.tmode = MechJebModuleThrustController.TMode.DIRECT;
                        core.thrust.trans_spd_act = 0;
                    }
                    else if ((vesselState.surfaceVelocity.magnitude > 5) && (Vector3d.Angle(vesselState.forward, -vesselState.surfaceVelocity) > 45))
                    {
                        // if we're not facing approximately retrograde, turn to point retrograde and don't thrust:
                        core.attitude.attitudeTo(Vector3d.back, AttitudeReference.SURFACE_VELOCITY, null);
                        core.thrust.tmode = MechJebModuleThrustController.TMode.DIRECT;
                        core.thrust.trans_spd_act = 0;
                    }
                    else if (useChute)
                    {
                        // rely on parachutes to get down to 200m
                        // always point up towards target, so any lift providing surface will get us closer
                        Vector3d diff = core.target.GetPositionTargetPosition()- core.landing.LandingSite;
                        core.attitude.attitudeTo(Quaternion.LookRotation(-vesselState.surfaceVelocity, diff), AttitudeReference.INERTIAL, null);
                        // for fine tuning we need to reduce deploy angles or we just spin around target
                        finalDifferenceTarget.value = diff.magnitude;
                        if (finalDifferenceTarget < 100)
                        {
                            foreach (var mcs in vessel.FindPartModulesImplementing<ModuleControlSurface>())
                            {
                                if (mcs.deploy)
                                {
                                    Vector2 scaledLimit = Vector2.Max((float)finalDifferenceTarget / 100f * mcs.deployAngleLimits,new Vector2(-2,2));
                                    mcs.deployAngle = Mathf.Clamp(mcs.deployAngle, scaledLimit.x, scaledLimit.y);
                                }
                            }
                        }

                        //Debug.Log(String.Format("Diff to target: {0:F2} ", (Vector3) diff));

                        //assume we fall straight due to parachutes
                        double maxSpeed = 0.9 * Math.Sqrt( 2d * vesselState.altitudeTrue * (vesselState.limitedMaxThrustAccel - mainBody.GeeASL * 9.81));
                        if (!waitForChute && (vesselState.TerminalVelocity() > maxSpeed) )
                        {
                            Debug.Log(String.Format("Emergency break in final descent alt:{0} accel:{1} maxSpeed:{2} terminal v:{3}", vesselState.altitudeTrue, vesselState.limitedMaxThrustAccel, maxSpeed, vesselState.TerminalVelocity()));
                            core.thrust.trans_spd_act = (float) maxSpeed;
                            core.thrust.tmode = MechJebModuleThrustController.TMode.KEEP_SURFACE;
                        }
                        else
                        {
                            core.thrust.tmode = MechJebModuleThrustController.TMode.DIRECT;
                            core.thrust.trans_spd_act = 0;
                        }

                    }
                    else
                    {
                        //if we're above 200m, point retrograde and control surface velocity:
                        core.attitude.attitudeTo(Vector3d.back, AttitudeReference.SURFACE_VELOCITY, null);

                        core.thrust.tmode = MechJebModuleThrustController.TMode.KEEP_SURFACE;

                        //core.thrust.trans_spd_act = (float)Math.Sqrt((vesselState.maxThrustAccel - vesselState.gravityForce.magnitude) * 2 * minalt) * 0.90F;
                        Vector3d estimatedLandingPosition = vesselState.CoM + vesselState.surfaceVelocity.sqrMagnitude / (2 * vesselState.limitedMaxThrustAccel) * vesselState.surfaceVelocity.normalized;
                        double terrainRadius = mainBody.Radius + mainBody.TerrainAltitude(estimatedLandingPosition);
                        aggressivePolicy = new GravityTurnDescentSpeedPolicy(terrainRadius, mainBody.GeeASL * 9.81, vesselState.limitedMaxThrustAccel);  // this constant policy creation is wastefull...
                        core.thrust.trans_spd_act = (float)(aggressivePolicy.MaxAllowedSpeed(vesselState.CoM - mainBody.position, vesselState.surfaceVelocity));
                    }
                }
                else
                {
                    // + 15% safety margin on break distance + distance for last three second with touchdown speed for MechJeb Thrust PID to catch up
                    double safety_alt = Math.Max(0.85D * (minalt - core.landing.touchdownSpeed * 3 /*s*/ ),0);
                    if (mainBody.atmosphere)
                    {
                        // calculate effective terminal velocity for powered landing
                        // it is smaller as reduced speed from slow down decreases drag
                        // equations are derived from F = k * v^2 + m* ( a -g ), meaning accelerating from ground with help of drag
                        // equation assumes that k is constant, which fails at bigger altitude, but within final touchdown it is safe assumption
                        double a_eff = vesselState.limitedMaxThrustAccel - mainBody.GeeASL;
                        double v_terminal = vesselState.TerminalVelocity() * Math.Sqrt(a_eff/mainBody.GeeASL);
                        double x_0 = - 0.5D * v_terminal / a_eff * Math.Log( 1+ core.landing.touchdownSpeed * core.landing.touchdownSpeed/ (v_terminal * v_terminal));
                        // v(t) = v_terminal * tan( a_eff * t / v_terminal) + arctan( touchdownSpeed / v_terminal) )
                        // x(t) =  x_o - v_terminal^2 / a_eff * ln(cos(a_eff / v_terminal * t + arctan(touchdownSpeed / v_terminal) ))
                        // note: equations are only for small t where assumption about drag holds -> parameter in cos stays in [0,pi/2]
                        // v(x) = v_terminal * tan( arccos( e ^(-a_eff/v_terminal^2 * ( x -x_0))))
                        // for survivable touchdownspeeds x_0 is few cm, so we neglect it
                        core.thrust.trans_spd_act = (float) (-v_terminal * Math.Tan(Math.Acos(Math.Exp(-a_eff / (v_terminal * v_terminal) * safety_alt))));
                        
                        Debug.Log(String.Format("FinalDescent: Limit speed for safety alt={0:F0} with v_term={1:F1} and x_0={2:F1} to v_target={3:F1}, v={4:F1}", safety_alt, v_terminal, x_0, core.thrust.trans_spd_act, vesselState.speedVertical));
                    }
                    else
                    {
                        // last 200 meters ramp down speed with limit of falling full throttle break 
                        core.thrust.trans_spd_act = -Mathf.Sqrt((float)((vesselState.limitedMaxThrustAccel - vesselState.localg) * safety_alt * 2D + core.landing.touchdownSpeed * core.landing.touchdownSpeed));
                    }
                    // take into account desired landing speed:
                    core.thrust.trans_spd_act = (float)Math.Min(-core.landing.touchdownSpeed, core.thrust.trans_spd_act);

                    //avoid hopping due to throttle PID oversteering on very last moment
                    if (vesselState.speedVertical < 3 * core.landing.touchdownSpeed)
                        core.thrust.pid.intAccum = 0;

                    //                    if (Math.Abs(Vector3d.Angle(-vessel.surfaceVelocity, vesselState.up)) < 10)
                    if (vesselState.speedSurfaceHorizontal < 5)
                    {
                        // if we're falling more or less straight down, control vertical speed and 
                        // kill horizontal velocity
                        core.thrust.tmode = MechJebModuleThrustController.TMode.KEEP_VERTICAL;
                        core.thrust.trans_kill_h = true;
                    }
                    else
                    {
                        // if we're falling at a significant angle from vertical, our vertical speed might be
                        // quite small but we might still need to decelerate. Control the total speed instead
                        // by thrusting directly retrograde
                        core.attitude.attitudeTo(Vector3d.back, AttitudeReference.SURFACE_VELOCITY, null);
                        core.thrust.tmode = MechJebModuleThrustController.TMode.KEEP_SURFACE;
                        core.thrust.trans_spd_act *= -1;
                    }
                }

                status = "Final descent: " + vesselState.altitudeBottom.ToString("F0") + "m above terrain";

                // ComputeCourseCorrection doesn't work close to the ground
                /* if (core.landing.landAtTarget)
                {
                    core.rcs.enabled = true;
                    Vector3d deltaV = core.landing.ComputeCourseCorrection(false);
                    core.rcs.SetWorldVelocityError(deltaV);

                    status += "\nDV: " + deltaV.magnitude.ToString("F3") + " m/s";
                } */

                return this;
            }
        }
    }
}
