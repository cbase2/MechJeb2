using System;
using UnityEngine;

namespace MuMech
{
    namespace Landing
    {
        public class AtmosphericDeorbit : NodeExecution
        {
            public static float deorbitprecision = 500f;
            public static int maxIterations = 50;
            int iteration = 0;
            double minDiff;
            double minUT;

            enum Phase { init, deorbit, minOrbit, planeChange, execute};
            Phase phase = Phase.init;

            Vector3d lastImpactRadialVector = Vector3d.zero;
            TrajectoriesConnector.TargetInfo targetInfo;

            public AtmosphericDeorbit(MechJebCore core) : base(core)
            {
                doAfterExecution = new AtmosphericCorrection(core);

                targetInfo = new TrajectoriesConnector.TargetInfo(core.target);
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
                        phase = Phase.deorbit;
                        iteration = 0;
                        break;
                    case Phase.deorbit:
                        targetInfo.update();
                        if (targetInfo.isValid && optimizeDeorbit())
                        {
                            iteration = 30;
                            phase = Phase.minOrbit;
                        }
                        else
                        {                            
                            if (iteration > maxIterations)
                            {
                                status = "Deorbit giving up in search for node, do manual deorbit";
                                core.landing.StopLanding();
                                return null;
                            }
                        }
                        break;
                    case Phase.minOrbit:
                        targetInfo.update();
                        if (targetInfo.isValid && minOrbit())
                        {
                            phase = Phase.planeChange;
                        }
                        break;
                    case Phase.planeChange:
                        targetInfo.update();
                        if (targetInfo.isValid && optimizePlaneChange())
                        {
                            phase = Phase.execute;
                        }
                        break;
                    case Phase.execute:
                        return base.OnFixedUpdate();
                }

                return this;
            }

            bool optimizeDeorbit()
            {

                //Debug.Log(String.Format("Autoland: currentImpactRadialVector={0} currentTargetRadialVector={1} differenceTarget={2}", targetInfo.currentImpactRadialVector, targetInfo.currentTargetRadialVector, targetInfo.differenceTarget));
                //Debug.Log(String.Format("Autoland: orbitClosestToTarget={0} orbitNormal={1} targetForward={2} ", targetInfo.orbitClosestToTarget, orbit.SwappedOrbitNormal(), targetForward.ToString("F3")));
                //Debug.Log(String.Format("Autoland: normalDiff={0:F1}, backwardDiff={1:F1}", targetInfo.normalDifference, targetInfo.backwardDifference));

                if (Math.Abs(targetInfo.targetAheadAngle) < 0.5 && Math.Abs(targetInfo.backwardDifference) < deorbitprecision)
                {
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

                    // deltaV needs to rotate
                    plannedNode.RemoveSelf();
                    vessel.PlaceManeuverNode(vessel.orbit, OrbitalManeuverCalculator.DeltaVToChangePeriapsis(orbit, deorbitTime, mainBody.Radius + core.landing.reentryTargetHeight), deorbitTime);
                    targetInfo.invalidateCalculation();
                    iteration++;
                }

                return false;
            }

            // find orbit round with min normalDiff to target, makes only sense if we are in inclined orbit
            bool minOrbit()
            {
                ManeuverNode plannedNode = vessel.patchedConicSolver.maneuverNodes[0];

                if (Math.Abs(orbit.inclination)<5)
                    return true;
                Debug.Log(String.Format("Current deorbit round at t={0:F0} has normalDiff={1:F1}", plannedNode.UT - Planetarium.GetUniversalTime(), targetInfo.normalDifference));
                if (iteration == 0 || Math.Abs(targetInfo.normalDifference) < minDiff)
                {
                    Debug.Log(String.Format("Found new optimum deorbit round at t={0:F0} for normalDiff={1:F1} ", plannedNode.UT-Planetarium.GetUniversalTime(), targetInfo.normalDifference));
                    minDiff = Math.Abs(targetInfo.normalDifference);
                    minUT = plannedNode.UT;
                }
                iteration++;
                if (iteration > maxIterations)
                {
                    plannedNode.RemoveSelf();
                    vessel.PlaceManeuverNode(vessel.orbit, OrbitalManeuverCalculator.DeltaVToChangePeriapsis(orbit, minUT, mainBody.Radius + core.landing.reentryTargetHeight), minUT);
                    TrajectoriesConnector.API.invalidateCalculation();
                    return true;
                }
                // factor accounts for ground movement during orbit round
                double nextRound = plannedNode.UT + orbit.period * vessel.mainBody.rotationPeriod / (vessel.mainBody.rotationPeriod - orbit.period);
                plannedNode.RemoveSelf();
                vessel.PlaceManeuverNode(vessel.orbit, OrbitalManeuverCalculator.DeltaVToChangePeriapsis(orbit, nextRound, mainBody.Radius + core.landing.reentryTargetHeight), nextRound);
                targetInfo.invalidateCalculation();
                return false;
            }

            bool optimizePlaneChange()
            {
                // modify Node to include required remaining planeChange
                double lastPlaneChange = -2.0 * vesselState.speedOrbital * Math.Sin(0.5 * Math.Asin(targetInfo.normalDifference / targetInfo.distanceTarget.magnitude));
                Debug.Log(String.Format("Autoland plane change {0:F1} for normalDiff={1:F1} ", (Vector3) (lastPlaneChange * orbit.GetOrbitNormal()), targetInfo.normalDifference));
                vessel.patchedConicSolver.maneuverNodes[0].DeltaV += lastPlaneChange * orbit.GetOrbitNormal();
                return true;
            }
        }
    }
}
