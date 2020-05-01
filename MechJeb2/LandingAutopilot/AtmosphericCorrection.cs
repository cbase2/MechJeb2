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
            enum Phase { waitForEntry, descend, parachutes };
            Phase phase = Phase.waitForEntry;

            
            TrajectoriesConnector.TargetInfo targetInfo;
            double breakDiff;
            double gee; //convience for mainbody gee ASL in m/s²

            class ParachuteInfo
            {
                public double maxSpeed;
                public double breakDistance;
                public double undeployedDistance;
                VesselState vesselState;

                public ParachuteInfo(VesselState vesselState)
                {
                    this.vesselState = vesselState;
                    update(vesselState.speedSurface, vesselState.atmosphericDensity);
                }

                public void update(double curSpeed, double curDensity)
                {
                    maxSpeed = 0;
                    breakDistance = 0;
                    curDensity = Math.Max(curDensity, 0.2 * vesselState.mainBody.atmDensityASL);
                    double curSpeedOfSound = Math.Max(vesselState.speedOfSound, vesselState.mainBody.GetSpeedOfSound( 0.2 * vesselState.mainBody.atmPressureASL , 0.2* vesselState.mainBody.atmDensityASL));
                    //create List by descending order of maxSpeed with drag and max chute reaction time
                    SortedList<double, (float, float)> parachuteTypes = new SortedList<double, (float, float)>(Comparer<double>.Create( (a, b) => Math.Sign(b-a) ) );
                    foreach (var p in vesselState.parachutes)
                    {
                        if (p.deploymentState == ModuleParachute.deploymentStates.STOWED)
                        {
                            p.refDensity = curDensity;
                            p.CalcBaseStats();
                            DragCubeList simParachute = new DragCubeList();
                            simParachute.LoadCube(p.part.DragCubes, "DEPLOYED");
                            simParachute.SetPart(p.part);
                            simParachute.SetOcclusionMultiplier(0f);
                            simParachute.ResetCubeWeights();
                            simParachute.SetDragWeights();
                            simParachute.SetPartOcclusion();                          
                            //Debug.Log(String.Format("ParachuteInfo.update Parchute {0} has up Cube area={1:F1} drag={2:F1} weight={3:F1} AreaOccluded={4:F1}",
                            //    p.part.name, simParachute.Cubes[0].Area[(int) DragCube.DragFace.YP], simParachute.Cubes[0].Drag[(int) DragCube.DragFace.YP],
                            //    simParachute.Cubes[0].Weight, simParachute.AreaOccluded[(int)DragCube.DragFace.YP]));
                            DragCubeList.CubeData simDrag = new DragCubeList.CubeData();
                            simParachute.AddSurfaceDragDirection(Vector3.up, (float)(p.maxSafeSpeedAtRef/ curSpeedOfSound), ref simDrag);
                            //Debug.Log(String.Format("ParachuteInfo.update Parchute {0} has areaDrag={1:F1}, crossSectionalArea={2:F1}, exposedArea={3:F1}",
                            //          p.part.name, simDrag.areaDrag, simDrag.crossSectionalArea, simDrag.exposedArea));
                            if (parachuteTypes.ContainsKey(p.maxSafeSpeedAtRef))
                                parachuteTypes[p.maxSafeSpeedAtRef] = (parachuteTypes[p.maxSafeSpeedAtRef].Item1 + simDrag.areaDrag,
                                                                       Mathf.Max(parachuteTypes[p.maxSafeSpeedAtRef].Item2, p.Anim[p.semiDeployedAnimation].length / p.semiDeploymentSpeed + p.Anim[p.fullyDeployedAnimation].length / p.deploymentSpeed));
                            else
                                parachuteTypes.Add(p.maxSafeSpeedAtRef, (simDrag.areaDrag, p.Anim[p.semiDeployedAnimation].length / p.semiDeploymentSpeed + p.Anim[p.fullyDeployedAnimation].length / p.deploymentSpeed));
                        }
                    }
                    float totalFriction = 0;
                    double lastSpeed = 0;
                    foreach (var pt in parachuteTypes)
                    {
                        double effSpeed = Math.Min(curSpeed, pt.Key);
                        // base equation deceleration term + chute reaction term with deceleration taken from moving equations with netwon friction
                        // during reaction time we hardly break, so assume 
                        float friction_const = pt.Value.Item1 * 0.0005f * (float)curDensity* PhysicsGlobals.DragMultiplier 
                            * PhysicsGlobals.DragCubeMultiplier * PhysicsGlobals.DragCurvePseudoReynolds.Evaluate((float) (effSpeed *curDensity)) ;
                        if (breakDistance == 0) // first term for highest speed
                        {                            
                            breakDistance = (float)vesselState.mass / friction_const * Math.Log(effSpeed) + effSpeed * pt.Value.Item2;
                            undeployedDistance = effSpeed * pt.Value.Item2;// - vesselState.mass / friction_const  ;
                            maxSpeed = lastSpeed = effSpeed;
                            totalFriction = friction_const;
                            //Debug.Log(String.Format("ParachuteInfo.update has first chute type with friction={0:F4} and effSpeed={1:F0}, delay={3:F1} results break dist={2:F0}", friction_const, effSpeed, breakDistance, pt.Value.Item2));
                        }
                        else
                        {                            
                            // calculate slowdown during parachute opening using old friction
                            effSpeed *= (float)vesselState.mass / (totalFriction * pt.Value.Item2 *effSpeed + vesselState.mass);
                            breakDistance -= (float)vesselState.mass / (totalFriction + friction_const) * friction_const / totalFriction * Math.Log(effSpeed);
                            undeployedDistance += pt.Value.Item2 * effSpeed;// + vesselState.mass * lastSpeed / (totalFriction * effSpeed) - vesselState.mass / (totalFriction + friction_const);
                            lastSpeed = effSpeed;
                            totalFriction += friction_const;
                            //Debug.Log(String.Format("ParachuteInfo.update has next chute type with friction={0:F4} and effSpeed={1:F0}, delay={3:F1} results break dist={2:F0}", friction_const, effSpeed, breakDistance, pt.Value.Item2));
                        }
                    }
                    if (totalFriction > 0)
                    {
                        double terminalSpeed = Math.Sqrt((float)vesselState.mass / totalFriction * vesselState.mainBody.GeeASL * PhysicsGlobals.GravitationalAcceleration);
                        //we are not getting slower than terminal velocity, so substract this as best guess for integration constant v
                        breakDistance -= (float)vesselState.mass / totalFriction * Math.Log(terminalSpeed);
                        //undeployedDistance += vesselState.mass * lastSpeed / (totalFriction * terminalSpeed);
                        //Debug.Log(String.Format("ParachuteInfo.update has terminal velocity term with friction={0:F4} and terminalSpeed={1:F0} results break dist={2:F0}", totalFriction, terminalSpeed, breakDistance));
                    }
                    Debug.Log(String.Format("ParachuteInfo.update for speed={0:F0} gives maxSpeed={1:F0} break dist={2:F0} undeployedDistance={3:F0}", curSpeed, maxSpeed, breakDistance, undeployedDistance));
                }
            }
            ParachuteInfo parachuteInfo;

            public AtmosphericCorrection(MechJebCore core) : base(core)
            {
                gee = mainBody.GeeASL * PhysicsGlobals.GravitationalAcceleration;
                targetInfo = new TrajectoriesConnector.TargetInfo(core.target); //plain target mode
                parachuteInfo = new ParachuteInfo(vesselState);
                
                Debug.Log(String.Format("AtmoCorrection parachute breakDist={0:F0} maxSpeed={1:F0}", parachuteInfo.breakDistance , parachuteInfo.maxSpeed));
            }

            public override AutopilotStep Drive(FlightCtrlState s)
            {
                // primary directions come from trajectories, but we may change them in descend phase to stay on course
                targetInfo.update();
                switch (phase)
                {
                    case Phase.waitForEntry:
                        status = "Holding entry attitude, no corrections";
                        //just hold attitude in this phase
                        core.attitude.attitudeTo(TrajectoriesConnector.API.PlannedOrientation().Value, AttitudeReference.SURFACE_VELOCITY, this);
                        if (vesselState.altitudeASL < mainBody.atmosphereDepth)
                        {                            
                            phase = Phase.descend;
                        }
                        break;
                    case Phase.descend:
                        descend();
                        if (core.landing.deployChutes &&
                            vesselState.parachutes.Any(p => p.deploymentSafeState == ModuleParachute.deploymentSafeStates.SAFE && p.deploymentState == ModuleParachute.deploymentStates.STOWED))
                        {
                            phase = Phase.parachutes;                            
                            core.attitude.attitudeTo(TrajectoriesConnector.API.PlannedOrientation().Value, AttitudeReference.SURFACE_VELOCITY, this);
                            core.thrust.targetThrottle = 0;
                        }
                        break;
                    case Phase.parachutes:
                        parachuteInfo.update(vesselState.speedSurface, vesselState.atmosphericDensity);
                        double dist = Vector3d.Dot(targetInfo.distanceTarget, vesselState.horizontalSurface) - parachuteInfo.breakDistance;
                        Debug.Log(String.Format("Waiting for parachutes to open Speed:{0:F0} dist:{1:F0} break:{2:F0} maxSpeed:{3:F0} ", vesselState.speedSurface, targetInfo.forwardDistance, dist, parachuteInfo.maxSpeed));
                        status = "Waiting "+MuUtils.ToSI(dist)+ "m with parachutes deploy to hit target";
                        // control parachute opening
                        if (dist <= 0)
                            deployParachutes();
                        if (vesselState.parachutes.All(p => p.deploymentState != ModuleParachute.deploymentStates.STOWED))
                            return new FinalDescent(core);
                        break;
                }


                if (vesselState.altitudeTrue < 1000 || vesselState.speedSurface < 200)
                {
                    if (core.landing.deployChutes)
                        deployParachutes();
                    return new FinalDescent(core);
                }
                
                return this;
            }

            public override AutopilotStep OnFixedUpdate()
            {
                bool idleFlight = core.thrust.targetThrottle == 0 && (phase == Phase.waitForEntry || phase == Phase.descend)
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
                foreach (var p in vesselState.parachutes.Where(p => p.deploymentState == ModuleParachute.deploymentStates.STOWED && p.deploymentSafeState == ModuleParachute.deploymentSafeStates.SAFE))
                {
                    p.Deploy();
                    p.deployAltitude = (float) vesselState.altitudeTrue;
                    Debug.Log(String.Format("Deploying Parachutes yeah ! Speed:{0:F0} dist:{1:F0} break:{2:F0} maxSpeed:{3:F0} ",vesselState.speedSurfaceHorizontal.value, targetInfo.forwardDistance, breakDiff, parachuteInfo.maxSpeed));
                }
            }

            bool aeroClamp = false;
            void descend()
            {
                Quaternion courseCorrection = Quaternion.identity;

                // we aim for parachute break point with half deploy time for opening, which we can anticipate
                breakDiff = targetInfo.backwardDifference - (core.landing.deployChutes ? 0.5D * parachuteInfo.undeployedDistance : 0);

                status = String.Format("Holding planned descent attitude, break diff={0:F1}", breakDiff);

                String logs = String.Format("Atmo Correction alt:{0:F0}, speed hor:{1:F0}, AoA: {2:F1}, dist:{3:F0}", vesselState.altitudeASL.value, vesselState.speedSurfaceHorizontal.value, vesselState.AoA.value, targetInfo.forwardDistance);

                
                double backwardCorrection = breakDiff / targetInfo.forwardDistance;

                logs += String.Format(", target back:{0:F0} corr:{1:F4} break:{2:F0}", targetInfo.backwardDifference, backwardCorrection, breakDiff);


                if (targetInfo.isValid)
                {
                    double AoA = TrajectoriesConnector.API.AoA.Value;
                    //if we are less than 0°-60° turned on entry or 0-30° otherwise, simply turn more or less to hit target
                    if (!TrajectoriesConnector.API.isBelowEntry() && Math.Abs(backwardCorrection) > 0.02 )
                    {
                        AoA = MuUtils.Clamp(AoA - Math.Sign(backwardCorrection) * 0.5, 120d, 180d, out aeroClamp, out _);
                        if (Math.Abs(TrajectoriesConnector.API.AoA.Value - AoA) > 0.1) 
                        {
                            TrajectoriesConnector.API.AoA = AoA;
                            logs += String.Format(", changed AoA=>{0:F1}", AoA);
                        }
                    }
                    else if (Math.Abs(backwardCorrection) > 0.02 )
                    {
                        AoA = MuUtils.Clamp(AoA - Math.Sign(backwardCorrection) * 0.5, 150d, 180d, out aeroClamp, out _);
                        if (TrajectoriesConnector.API.AoA != AoA)
                        {
                            TrajectoriesConnector.API.AoA = AoA;
                            logs += String.Format(", changed AoA=>{0:F1}", AoA);
                        }
                    }
                    if (aeroClamp && (core.thrust.targetThrottle != 0 || backwardCorrection > 0.03))
                    {
                        core.thrust.targetThrottle = Mathf.Clamp01((float)(gee / vesselState.limitedMaxThrustAccel * backwardCorrection)); // set thrust at 1G for 10% over target => early corrections, but slow enough for Trajectories
                        logs += String.Format(", reverse thrust=>{0:P0}", core.thrust.targetThrottle);
                        status += ", +THRUST";
                    }
                    else if (!aeroClamp)
                        core.thrust.targetThrottle = 0; // safeguard, actually backwardCorrection should be 0 before Angle gets reduced
                }

                float rollForTarget=0;
                // roll plannedOrientation towards target if we have signifikant lift
                if (vesselState.lift > 1)
                {
                    rollForTarget = -Mathf.Asin((float)(2.0 * targetInfo.normalDifference / targetInfo.distanceTarget.magnitude
                                 * vesselState.mass * vesselState.speedSurface / (targetInfo.TimeTillImpact * vesselState.lift))) * Mathf.Rad2Deg;
                    rollForTarget = Mathf.Clamp(rollForTarget, -30, 30);
                }

                logs += String.Format(", normalDiff: {0:F0}, time: {1:F0}s, rollAngle={2:F1}",targetInfo.normalDifference, targetInfo.TimeTillImpact, rollForTarget);
                if (Mathf.Abs(rollForTarget) < 0.0005 || targetInfo.normalDifference < 50 ) rollForTarget = 0; //neglect very small differences
                if (Mathf.Abs(rollForTarget) > 0 && TrajectoriesConnector.API.isBelowEntry())
                {
                    status += String.Format(", roll towards target with {0:F1}", rollForTarget );
                    courseCorrection = Quaternion.AngleAxis(rollForTarget, Vector3.forward);
                }
                Debug.Log(logs);

                Quaternion plannedOrientation = (Quaternion)TrajectoriesConnector.API.PlannedOrientation();
                core.attitude.attitudeTo(courseCorrection * plannedOrientation, AttitudeReference.SURFACE_VELOCITY, this);
            }

        }
    }
}
