using KSP.UI.Screens;
using System;
using System.Collections.Generic;
using System.Linq;
using UnityEngine;


namespace MuMech
{
    namespace Landing
    {
        class AtmosphericCorrection : AutopilotStep
        {
            enum Phase { entry, measure, waitTransition, descend, parachutes };
            Phase phase = Phase.entry;

            
            TrajectoriesConnector.TargetInfo targetInfo;
            double breakingDistance;

            double origAirAngle, currentAngle, minAngle, maxAngle;
            double origDifference, minDifference, maxDifference;
            const double measureAngles = 8d; // +-° AoA change to check effect on impact

            double measureDistance = -1;

            class ParachuteInfo
            {
                public double maxSpeed;
                public double specificDeceleration;//multiply with mass to get break distance
                VesselState vesselState;

                class DragList { public double maxSpeed; public float totalDrag; };

                public ParachuteInfo(VesselState vesselState)
                {
                    this.vesselState = vesselState;
                    update(vesselState.speedSurface, vesselState.atmosphericDensity);
                }

                public void update(double curSpeed, double curDensity)
                {
                    maxSpeed = 0;
                    specificDeceleration = 0;
                    curDensity = Math.Max(curDensity, 0.2 * vesselState.mainBody.atmDensityASL);
                    vesselState.parachutes.ForEach(p => { p.refDensity = curDensity; p.CalcBaseStats(); });
                    // go over all parachutes by type and sum up drag by type and then 
                    var paraDragType = vesselState.parachutes.Where(p => p.deploymentState == ModuleParachute.deploymentStates.STOWED)
                        .GroupBy(p => p.maxSafeSpeedAtRef)
                        .Select(pt => new { maxSpeed = pt.Key, totalDrag = pt.Sum(p => p.fullyDeployedDrag) })
                        .OrderByDescending(pt => pt.maxSpeed) // higher speed para stays deployed, so sort desc for running sum
                        .Aggregate(new List<DragList>(), (list, pt) =>
                        {
                            list.Add(new DragList { maxSpeed = pt.maxSpeed, totalDrag = (list.Any() ? list.Last().totalDrag : 0) + pt.totalDrag });
                            return list;
                        })
                        .OrderBy(pt => pt.maxSpeed); // sort asc for force calculation

                    foreach (var pt in paraDragType)
                    {
                        if (curSpeed > maxSpeed)
                        {
                            double effSpeed = Math.Min(curSpeed, pt.maxSpeed);
                            // x / m = 0.5 * ( v_start^2 -v_end^2 ) / a = 0.5 * ( v_start^2 -v_end^2 ) * / F
                            specificDeceleration += 0.5 * (effSpeed * effSpeed - maxSpeed * maxSpeed) / (PhysicsGlobals.DragMultiplier * pt.totalDrag);
                            maxSpeed = pt.maxSpeed;
                        }
                    }
                    Debug.Log(String.Format("Parachute update speed={0:F0} maxSpeed={1:F0} break dist={2:F0}", curSpeed, maxSpeed, specificDeceleration*vesselState.mass));
                }
            }
            ParachuteInfo parachuteInfo;

            public AtmosphericCorrection(MechJebCore core) : base(core)
            {
                targetInfo = new TrajectoriesConnector.TargetInfo(core.target);
                parachuteInfo = new ParachuteInfo(vesselState);

                Debug.Log(String.Format("AtmoCorrection parachute breakDist={0:F0} maxSpeed={1:F0}", parachuteInfo.specificDeceleration* vesselState.mass , parachuteInfo.maxSpeed));
            }

            public override AutopilotStep Drive(FlightCtrlState s)
            {
                // primary directions come from trajectories, but we may change them in descend phase to stay on course
                targetInfo.update();
                bool isTransition = TrajectoriesConnector.API.isTransitionAlt();
                switch (phase)
                {
                    case Phase.entry:
                        status = "Holding entry attitude, no corrections";
                        //just hold attitude in this phase
                        core.attitude.attitudeTo(TrajectoriesConnector.API.PlannedOrientation().Value, AttitudeReference.SURFACE_VELOCITY, this);
                        if (TrajectoriesConnector.API.isBelowEntry())
                        {
                            phase = Phase.measure;
                            startMeasure();
                        }
                        break;
                    case Phase.measure:
                        core.thrust.targetThrottle = 0; // no thrust, no attitude change during measurement
                        status = "Analyzing impact with different angles";
                        if (targetInfo.isValid && measureAngle())
                        {
                            phase = Phase.descend;
                            measureDistance = targetInfo.distanceImpact.magnitude;
                        }
                        break;
                    case Phase.waitTransition:
                        if (!isTransition)
                        {
                            //transition finished, analyze new angles
                            phase = Phase.measure;
                            startMeasure();
                        }
                        goto case Phase.descend;
                    case Phase.descend:
                        descend();
                        if (isTransition)
                            phase = Phase.waitTransition;
                        if (vesselState.parachutes.Any(p => p.deploymentSafeState == ModuleParachute.deploymentSafeStates.SAFE
                                                        || p.deploymentState == ModuleParachute.deploymentStates.SEMIDEPLOYED
                                                        || p.deploymentState == ModuleParachute.deploymentStates.DEPLOYED))
                        {
                            phase = Phase.parachutes;
                            TrajectoriesConnector.API.AoA = origAirAngle;
                            core.attitude.attitudeTo(TrajectoriesConnector.API.PlannedOrientation().Value, AttitudeReference.SURFACE_VELOCITY, this);
                            core.thrust.targetThrottle = 0;
                        }
                        break;
                    case Phase.parachutes:
                        parachuteInfo.update(vesselState.speedSurface, vesselState.atmosphericDensity);
                        double dist = targetInfo.forwardDistance - parachuteInfo.specificDeceleration * vesselState.mass;
                        status = "Waiting "+MuUtils.ToSI(dist)+ "m with parachutes deploy to hit target";
                        // control parachute opening
                        if (dist <= 0)
                            deployParachutes();
                        break;
                }


                if (vesselState.altitudeTrue < 1000 || vesselState.speedSurface < 200)
                {
                    deployParachutes();
                    return new FinalDescent(core);
                }
                
                return this;
            }

            public override AutopilotStep OnFixedUpdate()
            {
                bool idleFlight = core.thrust.targetThrottle == 0 && (phase == Phase.entry || phase == Phase.descend)
                        && (core.attitude.attitudeAngleFromTarget() < 1 
                            || (core.attitude.attitudeAngleFromTarget() < 10 && !MuUtils.PhysicsRunning())
                            ); 
                //autowarp, but only if we're already aligned with the node
                if (core.node.autowarp)
                {
                    if (idleFlight)
                    {
                        if (vesselState.altitudeASL > mainBody.atmosphereDepth)
                            core.warp.WarpRegularAtRate(100);
                        else
                            core.warp.WarpPhysicsAtRate(4);
                    }
                    else
                    {
                        //realign, wait for action to happen
                        core.warp.MinimumWarp();
                    }
                }
                return this;
            }

            void deployParachutes()
            {
                foreach (var p in vesselState.parachutes.Where(p => p.deploymentState == ModuleParachute.deploymentStates.STOWED))
                {
                    p.Deploy();
                    p.deployAltitude = (float) vesselState.altitudeTrue;
                    Debug.Log(String.Format("Deploying Parachutes yeah ! Speed:{0:F0} dist:{1:F0} break:{2:F0} maxSpeed:{3:F0} ",vesselState.speedSurfaceHorizontal.value, targetInfo.forwardDistance, breakingDistance, parachuteInfo.maxSpeed));
                }
            }

            void descend()
            {
                Quaternion courseCorrection = Quaternion.identity;

                breakingDistance = parachuteInfo.specificDeceleration * vesselState.mass;

                // breaking dist with thrust to parachute speed 
                if (vesselState.limitedMaxThrustAccel > 0)
                    breakingDistance += 0.5 * (vesselState.speedSurfaceHorizontal * vesselState.speedSurfaceHorizontal - parachuteInfo.maxSpeed * parachuteInfo.maxSpeed) / vesselState.maxThrustAccel;
                double breakingCorrection = breakingDistance / targetInfo.forwardDistance;

                status = String.Format("Holding planned descent attitude, forward dist ={2:F1} targ backward={0:F1} break dist={1:F1}"
                    , targetInfo.backwardDifference, breakingDistance, targetInfo.forwardDistance);

                String logs = String.Format("Atmo Correction alt:{0:F0}, speed hor:{1:F0}, AoA: {2:F1}, dist:{3:F0}", vesselState.altitudeASL.value, vesselState.speedSurfaceHorizontal.value, vesselState.AoA.value, targetInfo.forwardDistance);

                double backwardCorrection = targetInfo.backwardDifference / targetInfo.forwardDistance;
                bool additionalCorrection;

                logs += String.Format(", target back:{0:F0} corr:{1:F4} break:{2:F0} corr:{3:F4}", targetInfo.backwardDifference, backwardCorrection, breakingDistance, breakingCorrection);

                // if we are still to fast, use thrust to slow down except we are already slow enough for parachutes
                if ( Math.Sign(backwardCorrection) > 0 && breakingCorrection > 0.9 && core.stageStats.atmoStats[core.stageStats.atmoStats.Length - 1].deltaV > 90)
                {

                    core.thrust.targetThrottle = Mathf.Clamp01((float)breakingCorrection);
                    status += ", breaking with thrust " + breakingCorrection.ToString("F1");
                    logs += " => breaking";
                }
                else
                {
                    if (backwardCorrection < 0.01 || targetInfo.backwardDifference < 500 || core.stageStats.atmoStats[core.stageStats.atmoStats.Length - 1].deltaV <= 90)
                    {
                        core.thrust.targetThrottle = 0;
                        logs += " => no thrust";
                    }

                    float maxBreakLimit = 0.3f + Mathf.Clamp( (1500-(float)vesselState.speedSurface.value)/(2*(1500-600)) , 0f, 0.5f);
                    if (breakingCorrection > maxBreakLimit)
                    {
                        // turn speed forward side up as much as can be sustained by steering
                        // clamp +-40° to stay stable
                        if (targetInfo.forwardDistance > 0) // retrograde ?
                            TrajectoriesConnector.API.AoA = MuUtils.Clamp(vesselState.AoA.value - 2d, 140, 220);
                        else
                            TrajectoriesConnector.API.AoA = MuUtils.Clamp(vesselState.AoA.value - 2d, -40, 40);
                        logs += " => max aero brake new Angle="+MuUtils.Clamp(vesselState.AoA - 2d, 140, 220).ToString("F1");
                    }
                    else if (Math.Abs(backwardCorrection) > 0.002)
                    {
                        double oldangle = TrajectoriesConnector.AoA;
                        // new angle should not run away for predictions if vessel can not hold AoA
                        double newangle = oldangle - 0.01 * core.landing.factor * backwardCorrection * measureDistance;
                        logs += " => change angle " + newangle.ToString("F1");
                        newangle = MuUtils.Clamp(newangle, vesselState.AoA.value - 1.2d, vesselState.AoA.value + 1.2d);
                        logs += ">" + newangle.ToString("F1");
                        newangle = MuUtils.Clamp(newangle, core.landing.minAngle, core.landing.maxAngle, out additionalCorrection);
                        logs += ">" + newangle.ToString("F1") + (additionalCorrection ? " is limit" : "");
                        status += ", correcting AirAngle " + newangle.ToString("F1");

                        TrajectoriesConnector.API.AoA = newangle; //avoid invalidate

                        // ToDo: include retro / prograde to check direction of thrust
                        // if aerodynamic breaking is at its limit, add thrust
                        if (additionalCorrection && backwardCorrection > 0.1
                                && core.stageStats.atmoStats[StageManager.CurrentStage].deltaV > 90)
                        {
                            core.thrust.targetThrottle = Mathf.Clamp01((float)backwardCorrection);
                            status += ", adding reverse thrust " + backwardCorrection.ToString("F1");
                            logs += " => adding thrust " + Mathf.Clamp01((float)backwardCorrection).ToString("F2");
                        }
                    }

                }
                // turn plannedOrientation towards Reference(=velocity), this should reduce drag and maybe we get more distance covered
                float courseDiffRate = (float)(30.0 * -targetInfo.normalDifference / targetInfo.distanceTarget.magnitude); //FIXME: hardcoded factor for rolling, maybe better scaling is somewhere in airplane code

                if (Mathf.Abs(courseDiffRate) < 0.0005) courseDiffRate = 0; //neglect very small differences
                if (Mathf.Abs(courseDiffRate) > 0)
                {
                    status += String.Format(", roll towards target with {0:F1}", Mathf.Clamp(courseDiffRate, -1, 1) * 30);
                    courseCorrection = Quaternion.AngleAxis(Mathf.Clamp(courseDiffRate, -1, 1) * 30, Vector3.forward);
                }
                Debug.Log(logs);

                Quaternion plannedOrientation = (Quaternion)TrajectoriesConnector.API.PlannedOrientation();
                core.attitude.attitudeTo(courseCorrection * plannedOrientation, AttitudeReference.SURFACE_VELOCITY, this);
            }

            void startMeasure()
            {
                minAngle = maxAngle = currentAngle = origAirAngle = TrajectoriesConnector.API.nextAoA.Value;
                minDifference = maxDifference = targetInfo.backwardDifference;
                TrajectoriesConnector.API.nextAoA = origAirAngle - measureAngles;
            }

            bool measureAngle()
            {
                Debug.Log(String.Format("measure next phase: nextAoA={0:F1} results diff={1:F0}",currentAngle, targetInfo.backwardDifference));
                if (targetInfo.backwardDifference < minDifference)
                {
                    minDifference = targetInfo.backwardDifference;
                    minAngle = currentAngle;
                }
                else if (targetInfo.backwardDifference > maxDifference)
                {
                    maxDifference = targetInfo.backwardDifference;
                    maxAngle = currentAngle;
                }

                if (currentAngle >= origAirAngle + measureAngles)
                {
                    //min/maxAngle are named here as they are associated to min and max difference
                    // next step needs them proper ordered for Clamp
                    // factor is negative if we need to decrease angle to increase backward difference
                    core.landing.minAngle = Math.Min(minAngle, maxAngle);
                    core.landing.maxAngle = Math.Max(minAngle, maxAngle);
                    core.landing.factor = (maxAngle - minAngle) / (maxDifference - minDifference);
                    TrajectoriesConnector.API.nextAoA = origAirAngle;
                    TrajectoriesConnector.API.invalidateCalculation();
                    Debug.Log(String.Format("Done analyzing vessel aerodynamic, approach angles =[{0:F1},{1:F1}] backward diff= [{3:F0},{4:F0}] difference gives factor = {2:G4}", minAngle, maxAngle, core.landing.factor, minDifference, maxDifference));
                    return true;
                }
                else
                {
                    TrajectoriesConnector.API.nextAoA = currentAngle += 1;
                }

                return false;
            }
        }
    }
}
