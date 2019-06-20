using System;
using UnityEngine;

namespace MuMech
{
    namespace Landing
    {
        public class AtmosphericDeorbit : NodeExecution
        {
            public static float deorbitprecision = 100f;
            bool initialized = false;
            Vector3d lastImpactRadialVector = Vector3d.zero; 

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
                    core.attitude.attitudeTo(Quaternion.identity, AttitudeReference.SURFACE_HORIZONTAL, this);
                    vessel.RemoveAllManeuverNodes();
                    vessel.PlaceManeuverNode(vessel.orbit, OrbitalManeuverCalculator.DeltaVToChangePeriapsis(orbit, vesselState.time, mainBody.Radius + core.landing.reentryTargetHeight), vesselState.time);
                    Trajectories.API.SetTarget(core.target.targetLatitude, core.target.targetLongitude);
                    status = "Calculating deorbit trajectory";
                    initialized = true;
                }
                else
                {

                    // primary calculations come from trajectories
                    Vector3d currentImpactRadialVector = Trajectories.API.GetImpactPosition() ?? Vector3d.zero; // relativ Vector to impact position on surface at current time

                    if (currentImpactRadialVector != Vector3d.zero &&
                        (currentImpactRadialVector - lastImpactRadialVector).magnitude < 100)
                    {
                            
                        // very rough estimate: add reentry overshoot to target as tangential vector at target
                        Vector3d currentTargetRadialVector = mainBody.GetWorldSurfacePosition(core.target.targetLatitude, core.target.targetLongitude, 0) - mainBody.position;
                        Vector3 targetForward = Vector3.Cross(currentTargetRadialVector, orbit.SwappedOrbitNormal()).normalized;
                        currentTargetRadialVector -= core.landing.reentryTargetAhead * targetForward;

                        Vector3d differenceTarget = currentTargetRadialVector - currentImpactRadialVector;
                        currentImpactRadialVector = currentImpactRadialVector.normalized;

                        Vector3 orbitClosestToTarget = Vector3.ProjectOnPlane(currentTargetRadialVector, orbit.SwappedOrbitNormal()).normalized;
                        Debug.Log(String.Format("Autoland: currentImpactRadialVector={0:F3} currentTargetRadialVector={1:F3} differenceTarget={2:F1}", currentImpactRadialVector, currentTargetRadialVector.normalized, differenceTarget));
                        targetForward = Vector3.Cross(orbitClosestToTarget, orbit.SwappedOrbitNormal()); //recalculate on orbital data for adjusted target
                        Debug.Log(String.Format("Autoland: orbitClosestToTarget={0:F3} orbitNormal={1:F3} targetForward={2:F3} ", orbitClosestToTarget, orbit.SwappedOrbitNormal(), targetForward));

                        //use angle between positions for rough adoption
                        float targetAheadAngle = Vector3.SignedAngle(orbitClosestToTarget, currentImpactRadialVector, orbit.SwappedOrbitNormal()); //How far ahead the target is compared to impact, in degrees

                        //calculate directional derivation at target location for fine tuning
                        double normalDifference = Vector3d.Dot(differenceTarget, orbit.SwappedOrbitNormal());
                        double backwardDifference = -Vector3d.Dot(differenceTarget, targetForward); /* = overshoot >0, too short <0 */
                        Debug.Log(String.Format("Autoland: normalDiff={0:F1}, backwardDiff={1:F1}", normalDifference, backwardDifference));

                        status = String.Format("Optimizing deorbit time based on trajectory prediction, shift by {0:F4} degree or {1:F0} seconds", targetAheadAngle, targetAheadAngle * orbit.period / 360f);
                        Debug.Log("Autoland: " + status);

                        if (Math.Abs(targetAheadAngle) < 0.5 && Math.Abs(backwardDifference) < deorbitprecision)
                        {
                            return base.OnFixedUpdate(); // execute plannedNode
                        }
                        else
                        {
                            //move Node
                            ManeuverNode plannedNode = vessel.patchedConicSolver.maneuverNodes[0];
                            double deorbitTime = plannedNode.UT;
                            double adaptionRate = 1; // 50% adaption rate 

                            if (Math.Abs(targetAheadAngle) < 0.5)
                            {
                                deorbitTime += adaptionRate * backwardDifference / vesselState.speedSurfaceHorizontal;
                            }
                            else
                            {
                                deorbitTime -= adaptionRate * targetAheadAngle * orbit.period / 360f;
                            }

                            if (deorbitTime < vesselState.time) deorbitTime += orbit.period;
                            if (deorbitTime > vesselState.time + 1.5 * orbit.period) deorbitTime -= orbit.period;

                            plannedNode.RemoveSelf();
                            vessel.PlaceManeuverNode(vessel.orbit, OrbitalManeuverCalculator.DeltaVToChangePeriapsis(orbit, deorbitTime, mainBody.Radius + core.landing.reentryTargetHeight), deorbitTime);
                            lastImpactRadialVector = Vector3d.zero;
                            Trajectories.API.invalidateCalculation();
                        }
                    }

                    if (currentImpactRadialVector != Vector3d.zero)
                        lastImpactRadialVector = currentImpactRadialVector;
                }

                return this;
            }
        }
    }
}
