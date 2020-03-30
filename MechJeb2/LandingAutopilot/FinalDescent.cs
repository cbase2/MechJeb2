using System;
using UnityEngine;

namespace MuMech
{
    namespace Landing
    {
        public class FinalDescent : AutopilotStep
        {

            private IDescentSpeedPolicy aggressivePolicy;
            private int waitForChute = 0;
            private bool useChute = false;

            public FinalDescent(MechJebCore core) : base(core)
            {
                useChute = mainBody.atmosphere && core.landing.deployChutes && vesselState.parachutes.Count > 0;
                if (useChute)
                    waitForChute = 30;
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

                if (waitForChute > 0) waitForChute--;

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
                        if (waitForChute == 0 && (vesselState.TerminalVelocity() > maxSpeed) )
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
                    if (useChute)
                    {
                        // ramp down from terminal velocity or current speed, but gravitation is already balanced by drag
                        float v_terminal = (float) Math.Max(vesselState.TerminalVelocity(),vesselState.speedSurface);
                        core.thrust.trans_spd_act = -Mathf.Lerp(0, v_terminal, 0.6666667f* (float) minalt / (v_terminal * v_terminal) * 2f * (float) vesselState.limitedMaxThrustAccel);
                    }
                    else
                    {
                        // last 200 meters ramp down free falling speed
                        core.thrust.trans_spd_act = -Mathf.Lerp(0, (float)Math.Sqrt((vesselState.limitedMaxThrustAccel - vesselState.localg) * 2 * 200) * 0.90F, (float)minalt / 200);
                    }

                    // take into account desired landing speed:
                    core.thrust.trans_spd_act = (float)Math.Min(-core.landing.touchdownSpeed, core.thrust.trans_spd_act);

//                    core.thrust.tmode = MechJebModuleThrustController.TMode.KEEP_VERTICAL;
//                    core.thrust.trans_kill_h = true;

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
