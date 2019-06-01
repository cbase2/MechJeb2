using System;
using UnityEngine;

namespace MuMech
{
    namespace Landing
    {
        public class AtmosphericDeorbit : NodeExecution
        {
            public static float deorbitprecision = 0.04f;
            bool initialized = false;
            int waitCycles=10; // Trajectories may need some time to get precise new impact 

            public AtmosphericDeorbit(MechJebCore core) : base(core)
            {
                doAfterExecution = new AtmosphericCorrection(core);
            }

            // calculate deorbit node and if node is acceptable base class will excute it
            public override AutopilotStep OnFixedUpdate()
            {

                //if we don't want to deorbit but we're already on a reentry trajectory, we can't wait until the ideal point 
                //in the orbit to deorbt; we already have deorbited.
                if (orbit.ApA < mainBody.RealMaxAtmosphereAltitude())
                {
                    core.thrust.targetThrottle = 0;
                    return doAfterExecution;
                }

                if (node != null) // we are already in node execution 
                    return base.OnFixedUpdate();


                if (!initialized)
                {
                    vessel.RemoveAllManeuverNodes();
                    vessel.PlaceManeuverNode(vessel.orbit, OrbitalManeuverCalculator.DeltaVToChangePeriapsis(orbit, vesselState.time, mainBody.Radius + core.landing.reentryTargetHeight), vesselState.time);
                    Trajectories.API.SetTarget(core.target.targetLatitude, core.target.targetLongitude);
                    status = "Calculating deorbit trajectory";
                    initialized = true;
                }
                else
                {
                    waitCycles--;
                    Vector3d currentImpactRadialVector = (Vector3d) Trajectories.API.GetImpactPosition(); // relativ Vector to impact position on surface at current time
                    Vector3d currentTargetRadialVector = mainBody.GetWorldSurfacePosition(core.target.targetLatitude, core.target.targetLongitude, 0) - mainBody.position;

                    if (currentImpactRadialVector != null )
                    {
                        currentImpactRadialVector = currentImpactRadialVector.normalized;
                        
                        Vector3 orbitClosestToTarget = Vector3.ProjectOnPlane(currentTargetRadialVector, orbit.SwappedOrbitNormal()).normalized;
                        //Debug.Log("Autoland: currentImpactRadialVector="+currentImpactRadialVector+" Target on Orbit="+ core.target.GetPositionTargetPosition().normalized + " orbit normal="+orbit.SwappedOrbitNormal().normalized);

                        float targetAheadAngle = Vector3.SignedAngle(orbitClosestToTarget, currentImpactRadialVector, orbit.SwappedOrbitNormal()); //How far ahead the target is compared to impact, in degrees
                        
                        status = String.Format("Optimizing deorbit time based on trajectory prediction, shift by {0:F4} degree or {1:F0} seconds", targetAheadAngle, targetAheadAngle * orbit.period / 360f);
                        //Debug.Log(status);

                        if (Mathf.Abs(targetAheadAngle) < deorbitprecision)
                        {
                            return base.OnFixedUpdate(); // execute plannedNode
                        }
                        else if (waitCycles == 0)
                        {
                            //move Node
                            ManeuverNode plannedNode = vessel.patchedConicSolver.maneuverNodes[0];

                            double deorbitTime = plannedNode.UT - 0.5 * targetAheadAngle * orbit.period / 360f; // 50% adaption seems more stable to edge conditions
                            if (deorbitTime < vesselState.time) deorbitTime += orbit.period;
                            if (deorbitTime > vesselState.time + 1.5 * orbit.period) deorbitTime -= orbit.period;

                            plannedNode.RemoveSelf();
                            vessel.PlaceManeuverNode(vessel.orbit, OrbitalManeuverCalculator.DeltaVToChangePeriapsis(orbit, deorbitTime, mainBody.Radius + core.landing.reentryTargetHeight) , deorbitTime);

                            waitCycles = 10;
                        }
                    }
                }

                return this;
            }
        }
    }
}
