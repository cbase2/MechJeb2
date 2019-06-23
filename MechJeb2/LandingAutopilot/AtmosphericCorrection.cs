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
            double AoARescaleFactor = 1.0;
            double maxParachuteSpeed = 0;
            double parachuteBreakDistance = 0; // need to be multiplied with current mass to get real distance
            bool parachutePhase = false;

            class DragList { public double maxSpeed; public float totalDrag; };
            Vector3d currentImpactRadialVector = Vector3d.zero;
            double forwardDistance;
            double breakingDistance;

            public AtmosphericCorrection(MechJebCore core) : base(core)
            {
                parachuteBreakDistance = 0;
                maxParachuteSpeed = 0;
                

            // go over all parachutes by type and sum up drag by type and then 
            var paraDragType = vesselState.parachutes.Where(p => p.deploymentState == ModuleParachute.deploymentStates.STOWED)
                .GroupBy(p => p.maxSafeSpeedAtRef)
                .Select(pt => new { maxSpeed= pt.Key, totalDrag = pt.Sum(p => p.fullyDeployedDrag) })
                .OrderByDescending(pt => pt.maxSpeed) // higher speed para stays deployed, so sort desc for running sum
                .Aggregate(new List<DragList>(), (list, pt) =>
                {
                    list.Add(new DragList { maxSpeed = pt.maxSpeed, totalDrag = (list.Any() ? list.Last().totalDrag : 0) + pt.totalDrag });
                    return list;
                } )
                .OrderBy(pt => pt.maxSpeed); // sort asc for force calculation

                foreach ( var pt in paraDragType)
                {
                    // x / m = 0.5 * ( v_start^2 -v_end^2 ) / a = 0.5 * ( v_start^2 -v_end^2 ) * / F
                    parachuteBreakDistance += 0.5 * (pt.maxSpeed * pt.maxSpeed - maxParachuteSpeed * maxParachuteSpeed) / (PhysicsGlobals.DragMultiplier * pt.totalDrag);
                    maxParachuteSpeed = pt.maxSpeed;
                }
                
                Debug.Log(String.Format("AtmoCorrection parachute breakDist={0:F0} maxSpeed={1:F0}", parachuteBreakDistance*vesselState.mass , maxParachuteSpeed));
            }

            public override AutopilotStep Drive(FlightCtrlState s)
            {
                // primary directions come from trajectories, but we may change them to stay on course
                currentImpactRadialVector = Trajectories.API.GetImpactPosition() ?? currentImpactRadialVector; // relativ Vector to impact position on surface at current time
                Quaternion courseCorrection = Quaternion.identity;
                bool isbelowEntry = Trajectories.API.isBelowEntry();

                if (isbelowEntry) // no corrections in entry phase
                {
                    Vector3d currentTargetRadialVector = mainBody.GetWorldSurfacePosition(core.target.targetLatitude, core.target.targetLongitude, 0) - mainBody.position;

                    Vector3d differenceTarget = currentTargetRadialVector - currentImpactRadialVector;
                    Vector3d distanceTarget = currentTargetRadialVector - vesselState.orbitalPosition;
                    Vector3d distanceImpact = currentImpactRadialVector - vesselState.orbitalPosition;
                    //calculate sideway derivation at target location, which means we have to rescale if current impact is earlier or later
                    double normalDifference = (Vector3d.Dot(differenceTarget, vesselState.normalPlusSurface) * distanceTarget.magnitude / distanceImpact.magnitude);
                    double backwardDifference = -Vector3d.Dot(differenceTarget, vesselState.horizontalSurface); /* = overshoot >0, too short <0 */
                    forwardDistance = Vector3d.Dot(distanceTarget, vesselState.horizontalSurface);

                    breakingDistance = parachuteBreakDistance * vesselState.mass;
                    parachutePhase = vesselState.parachutes.Any(p => p.deploymentSafeState == ModuleParachute.deploymentSafeStates.SAFE
                                                           || p.deploymentState == ModuleParachute.deploymentStates.SEMIDEPLOYED
                                                           || p.deploymentState == ModuleParachute.deploymentStates.DEPLOYED);

                    // breaking dist with thrust to parachute speed 
                    if (vesselState.maxThrustAccel > 0)
                        breakingDistance += 0.5 * (vesselState.speedSurfaceHorizontal * vesselState.speedSurfaceHorizontal - maxParachuteSpeed * maxParachuteSpeed) / vesselState.maxThrustAccel;
                    double breakingCorrection = breakingDistance / forwardDistance;

                    // control parachute opening
                    if (forwardDistance <= 1.1 * parachuteBreakDistance * vesselState.mass)
                        deployParachutes();

                    status = String.Format("Holding planned descent attitude, dist to target={0:F0}"
                        + " diff from targ backward={1:F1} break dist={2:F1} normal={3:F1}"
                        , forwardDistance, backwardDifference, breakingDistance, normalDifference);

                    double forwardCorrection = backwardDifference / forwardDistance;


                    // do not thrust if we can use parachute or have less than 50 deltaV left for touch down in stage
                    if ((forwardCorrection > 0.5 || breakingCorrection > 0.9) && !parachutePhase && core.stageStats.atmoStats[StageManager.CurrentStage].deltaV > 50)
                    {
                        // needed correction can be applied by thrust 
                        core.thrust.targetThrottle = Mathf.Clamp01((float) Math.Max(forwardCorrection, breakingCorrection));
                        status += ", applying reverse thrust " + forwardCorrection.ToString("F1");
                        Debug.Log(status);
                    }
                    else
                    {
                        if (forwardCorrection < 0.02 || core.stageStats.atmoStats[StageManager.CurrentStage].deltaV <= 50) 
                            core.thrust.targetThrottle = 0;

                        if (Math.Abs(forwardCorrection) > 0.002)
                        {
                            double newangle = MuUtils.Clamp((Trajectories.API.AirAngle ?? 0) - 0.05* core.landing.factor * forwardCorrection, core.landing.minAngle, core.landing.maxAngle);
                            status += ", correcting AirAngle " + newangle.ToString("F1");
                            Debug.Log(status);
                            Trajectories.API.AirAngle = newangle;
                        }

                    }
                    // turn plannedOrientation towards Reference(=velocity), this should reduce drag and maybe we get more distance covered

                    
                    float courseDiffRate = (float)(30.0 * normalDifference / distanceTarget.magnitude); //FIXME: hardcoded factor for rolling, maybe better scaling is somewhere in airplane code

                    if (Mathf.Abs(courseDiffRate) < 0.0005) courseDiffRate = 0; //neglect very small differences
                    if (Mathf.Abs(courseDiffRate) > 0)
                    {
                        status += String.Format(", roll towards target with {0:F1}", Mathf.Clamp(courseDiffRate, -1, 1) * 30);
                        courseCorrection = Quaternion.AngleAxis(Mathf.Clamp(courseDiffRate, -1, 1) * 30, Vector3.forward);
                    }

                }

                Quaternion plannedOrientation = (Quaternion)Trajectories.API.PlannedOrientation();
                core.attitude.attitudeTo(courseCorrection * plannedOrientation, AttitudeReference.SURFACE_VELOCITY, this);

                if (vesselState.altitudeTrue < 1000 || vesselState.speedSurface < 250)
                    return new FinalDescent(core);
                
                return this;
            }

            public override AutopilotStep OnFixedUpdate()
            {
                bool idleFlight = core.thrust.targetThrottle == 0 && !parachutePhase 
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
                    Debug.Log("Deploying Parachutes yeah !");
                }
            }
        }
    }
}
