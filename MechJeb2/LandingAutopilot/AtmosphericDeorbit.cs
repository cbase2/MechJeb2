using System;
using UnityEngine;

namespace MuMech
{
    namespace Landing
    {
        public class AtmosphericDeorbit : NodeExecution
        {
            public static float deorbitprecision = 500f;
            
            enum Phase { init, plan, measure, execute};
            Phase phase = Phase.init;

            Vector3d lastImpactRadialVector = Vector3d.zero;
            TrajectoriesConnector.TargetInfo targetInfo;

            double origAirAngle, minAngle, maxAngle;
            double origDifference, minDifference, maxDifference;

            public AtmosphericDeorbit(MechJebCore core) : base(core)
            {
                doAfterExecution = new AtmosphericCorrection(core);

                targetInfo = new TrajectoriesConnector.TargetInfo(core.target)
                                { targetOffset = core.landing.reentryTargetAhead };
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

                switch (phase)
                { 
                    case Phase.init:
                        //core.attitude.attitudeTo(Quaternion.identity, AttitudeReference.SURFACE_HORIZONTAL, this);
                        vessel.RemoveAllManeuverNodes();
                        vessel.PlaceManeuverNode(vessel.orbit, OrbitalManeuverCalculator.DeltaVToChangePeriapsis(orbit, vesselState.time, mainBody.Radius + core.landing.reentryTargetHeight), vesselState.time);
                        TrajectoriesConnector.API.SetTarget(core.target.targetLatitude, core.target.targetLongitude);
                        status = "Calculating deorbit trajectory";
                        phase = Phase.plan;
                        break;
                    case Phase.plan:
                        targetInfo.update();
                        if (targetInfo.isValid && optimizeNode())
                        {
                            //minDifference = maxDifference = -origDifference;
                            //minAngle = maxAngle = origAirAngle = TrajectoriesConnector.AirAngle; // we are only optimized with valid trajectory
                            //TrajectoriesConnector.AirAngle = origAirAngle - 15;
                            //phase = Phase.measure;
                            phase = Phase.execute; 
                        }
                        break;
                    case Phase.measure:
                        status = "Analyzing impact with different angles";
                        targetInfo.update();
                        // we have deorbit node and drive will turn to it, but meanwhile measure how changing AirAngle affects target difference
                        if (targetInfo.isValid && measureAngle())
                            phase = Phase.execute; //we are done measuring
                        break;
                    case Phase.execute:
                        return base.OnFixedUpdate();
                }

                return this;
            }

            bool optimizeNode()
            {

                //Debug.Log(String.Format("Autoland: currentImpactRadialVector={0} currentTargetRadialVector={1} differenceTarget={2}", targetInfo.currentImpactRadialVector, targetInfo.currentTargetRadialVector, targetInfo.differenceTarget));
                //Debug.Log(String.Format("Autoland: orbitClosestToTarget={0} orbitNormal={1} targetForward={2} ", targetInfo.orbitClosestToTarget, orbit.SwappedOrbitNormal(), targetForward.ToString("F3")));
                //Debug.Log(String.Format("Autoland: normalDiff={0:F1}, backwardDiff={1:F1}", targetInfo.normalDifference, targetInfo.backwardDifference));

                if (Math.Abs(targetInfo.targetAheadAngle) < 0.5 && Math.Abs(targetInfo.backwardDifference) < deorbitprecision)
                {
                    origDifference = targetInfo.backwardDifference;
                    return true; // execute plannedNode
                }
                else
                {
                    //move Node
                    ManeuverNode plannedNode = vessel.patchedConicSolver.maneuverNodes[0];
                    double deorbitTime = plannedNode.UT;
                    double timedelta;

                    if (Math.Abs(targetInfo.targetAheadAngle) < 0.5) // for small changes use tangential calculation, otherwise angluar
                    {
                        timedelta = targetInfo.backwardDifference / vesselState.speedSurfaceHorizontal;
                    }
                    else
                    {
                        timedelta = targetInfo.targetAheadAngle * orbit.period / 360f;
                    }

                    if (timedelta < 0) // asymetric to avoid jumping between two points without improvement. Observed with inclined trajectory.
                        deorbitTime -= timedelta;
                    else
                        deorbitTime -= 0.5 * timedelta;

                    if (deorbitTime < vesselState.time) deorbitTime += orbit.period;
                    if (deorbitTime > vesselState.time + 1.5 * orbit.period) deorbitTime -= orbit.period;

                    status = String.Format("Optimizing deorbit time based on trajectory prediction, shift by {0:F4} degree, {1:F0} m equals {2:F1} seconds", targetInfo.targetAheadAngle, targetInfo.backwardDifference, deorbitTime - plannedNode.UT);
                    Debug.Log("Autoland: " + status);

                    plannedNode.RemoveSelf();
                    vessel.PlaceManeuverNode(vessel.orbit, OrbitalManeuverCalculator.DeltaVToChangePeriapsis(orbit, deorbitTime, mainBody.Radius + core.landing.reentryTargetHeight), deorbitTime);
                    TrajectoriesConnector.API.invalidateCalculation();
                }

                return false;
            }

            bool measureAngle()
            {
                double currentAirAngle = TrajectoriesConnector.AoA;

                if (targetInfo.backwardDifference < minDifference)
                {
                    minDifference = targetInfo.backwardDifference;
                    minAngle = currentAirAngle;
                }
                else if (targetInfo.backwardDifference > maxDifference)
                {
                    maxDifference = targetInfo.backwardDifference;
                    maxAngle = currentAirAngle;
                }

                if (currentAirAngle >= origAirAngle + 15)
                {
                    //min/maxAngle are named here as they are associated to min and max difference
                    // next step needs them proper ordered for Clamp
                    // factor is negative if we need to decrease angle to increase backward difference
                    core.landing.minAngle = Math.Min(minAngle, maxAngle);
                    core.landing.maxAngle = Math.Max(minAngle, maxAngle);
                    core.landing.factor = (maxAngle-minAngle)/(maxDifference - minDifference);
                    TrajectoriesConnector.AoA = origAirAngle;
                    Debug.Log(String.Format("Done analyzing vessel aerodynamic, approach angles =[{0:F1},{1:F1}] backward diff= [{3:F0},{4:F0}] difference gives factor = {2:G4}", minAngle, maxAngle, core.landing.factor, minDifference, maxDifference));
                    return true;
                }
                else
                {
                    TrajectoriesConnector.AoA = currentAirAngle + 1;
                }

                return false;
            }
        }
    }
}
