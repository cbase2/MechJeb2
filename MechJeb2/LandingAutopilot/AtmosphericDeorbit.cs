using System;
using UnityEngine;

namespace MuMech
{
    namespace Landing
    {
        public class AtmosphericDeorbit : NodeExecution
        {
            public static float deorbitprecision = 100f;
            
            enum Phase { init, plan, measure, execute};
            Phase phase = Phase.init;

            enum Measurement { calc, waitCalc, refine, waitRefine, done }

            class MeasureData
            {
                public Measurement state;
                public double angle;
                public double difference;
            }

            MeasureData lower = new MeasureData();
            MeasureData upper = new MeasureData();

            Vector3d lastImpactRadialVector = Vector3d.zero;
            double origAirAngle;
            double origDifference;

            bool inverseAngles = false;

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

                switch (phase)
                { 
                    case Phase.init:
                        core.attitude.attitudeTo(Quaternion.identity, AttitudeReference.SURFACE_HORIZONTAL, this);
                        vessel.RemoveAllManeuverNodes();
                        vessel.PlaceManeuverNode(vessel.orbit, OrbitalManeuverCalculator.DeltaVToChangePeriapsis(orbit, vesselState.time, mainBody.Radius + core.landing.reentryTargetHeight), vesselState.time);
                        Trajectories.API.SetTarget(core.target.targetLatitude, core.target.targetLongitude);
                        status = "Calculating deorbit trajectory";
                        phase = Phase.plan;
                        break;
                    case Phase.plan:
                        if (!optimizeNode())
                            break;
                        else
                        {
                            phase = Phase.measure;
                            core.landing.origAngle = origAirAngle = (double) Trajectories.API.AirAngle; // we are only optimized with valid trajectory
                            lower.state = Measurement.calc;
                            Trajectories.API.AirAngle = lower.angle = origAirAngle - 5;
                            upper.state = Measurement.waitCalc;
                            upper.angle = origAirAngle;
                            goto case Phase.measure;
                        } 
                    case Phase.measure:
                        // we have deorbit node and drive will turn to it, but meanwhile measure how changing AirAngle affects target difference
                        if (measureAngle())
                            phase = Phase.execute; //we are done measuring
                        break;
                    case Phase.execute:
                        return base.OnFixedUpdate();
                }

                return this;
            }

            bool optimizeNode()
            {
                // primary calculations come from trajectories
                Vector3d? impactPos = Trajectories.API.GetImpactPosition(); // relativ Vector to impact position on surface at current time

                if (impactPos.HasValue &&
                    ((impactPos ?? Vector3d.zero ) - lastImpactRadialVector).magnitude < 100)
                {
                    Vector3d currentImpactRadialVector = (Vector3d)impactPos;
                    // very rough estimate: add reentry overshoot to target as tangential vector at target
                    Vector3d currentTargetRadialVector = mainBody.GetWorldSurfacePosition(core.target.targetLatitude, core.target.targetLongitude, 0) - mainBody.position;
                    Vector3 orbitNormal = orbit.SwappedOrbitNormal();
                    
                    Vector3 orbitClosestToTarget = Vector3.ProjectOnPlane(currentTargetRadialVector, orbitNormal).normalized;
                    Vector3 targetForward = Vector3.Cross(orbitClosestToTarget, orbitNormal);

                    orbitClosestToTarget -= core.landing.reentryTargetAhead * targetForward;
                    Vector3d differenceTarget = orbitClosestToTarget - currentImpactRadialVector;
                    
                    //How far ahead the target is compared to impact, in degrees
                    float targetAheadAngle = Vector3.SignedAngle(orbitClosestToTarget, currentImpactRadialVector, orbitNormal); 

                    //calculate directional derivation at target location for fine tuning
                    double normalDifference = Vector3d.Dot(differenceTarget, orbitNormal);

                    double forwardDifference = Vector3d.Dot(differenceTarget, targetForward); /* = overshoot < 0, too short > 0 */

                    //Debug.Log(String.Format("Autoland: currentImpactRadialVector={0:F3} currentTargetRadialVector={1:F3} differenceTarget={2:F1}", currentImpactRadialVector, currentTargetRadialVector.normalized, differenceTarget));
                    //Debug.Log(String.Format("Autoland: orbitClosestToTarget={0:F3} orbitNormal={1:F3} targetForward={2:F3} ", orbitClosestToTarget, orbit.SwappedOrbitNormal(), targetForward));
                    //Debug.Log(String.Format("Autoland: normalDiff={0:F1}, backwardDiff={1:F1}", normalDifference, backwardDifference));

                    if (Math.Abs(targetAheadAngle) < 0.5 && Math.Abs(forwardDifference) < deorbitprecision)
                    {
                        origDifference = forwardDifference;
                        return true; // execute plannedNode
                    }
                    else
                    {
                        //move Node
                        ManeuverNode plannedNode = vessel.patchedConicSolver.maneuverNodes[0];
                        double deorbitTime = plannedNode.UT;
                        double adoptionRate = 1;

                        if (Math.Abs(targetAheadAngle) < 0.5) // for small changes use tangential calculation, otherwise angluar
                        {
                            deorbitTime -= adoptionRate * forwardDifference / vesselState.speedSurfaceHorizontal;
                        }
                        else
                        {
                            deorbitTime -= adoptionRate * targetAheadAngle * orbit.period / 360f;
                        }

                        if (deorbitTime < vesselState.time) deorbitTime += orbit.period;
                        if (deorbitTime > vesselState.time + 1.5 * orbit.period) deorbitTime -= orbit.period;

                        status = String.Format("Optimizing deorbit time based on trajectory prediction, shift by {0:F4} degree, {1:F0} m equals {2:F1} seconds", targetAheadAngle, forwardDifference, deorbitTime - plannedNode.UT);
                        Debug.Log("Autoland: " + status);

                        plannedNode.RemoveSelf();
                        vessel.PlaceManeuverNode(vessel.orbit, OrbitalManeuverCalculator.DeltaVToChangePeriapsis(orbit, deorbitTime, mainBody.Radius + core.landing.reentryTargetHeight), deorbitTime);
                        lastImpactRadialVector = Vector3d.zero;
                        Trajectories.API.invalidateCalculation();
                    }
                }
                else if (impactPos.HasValue)
                    lastImpactRadialVector = (Vector3d)impactPos;

                return false;
            }

            bool measureAngle()
            {

                // primary calculations come from trajectories
                Vector3d currentImpactRadialVector = Trajectories.API.GetImpactPosition() ?? Vector3d.zero; // relativ Vector to impact position on surface at current time

                if (currentImpactRadialVector != Vector3d.zero &&
                    (currentImpactRadialVector - lastImpactRadialVector).magnitude < 100)
                {
                    double currentAirAngle = (double) Trajectories.API.AirAngle;
                    Vector3d currentTargetRadialVector = mainBody.GetWorldSurfacePosition(core.target.targetLatitude, core.target.targetLongitude, 0) - mainBody.position;

                    Vector3 orbitNormal = orbit.SwappedOrbitNormal();
                     
                    Vector3 orbitClosestToTarget = Vector3.ProjectOnPlane(currentTargetRadialVector, orbitNormal).normalized;
                    Vector3 targetForward = Vector3.Cross(orbitClosestToTarget, orbitNormal);

                    orbitClosestToTarget -= core.landing.reentryTargetAhead * targetForward;

                    Vector3d differenceTarget = orbitClosestToTarget - currentImpactRadialVector;
                    double forwardDifference = Vector3d.Dot(differenceTarget, targetForward);

                    analyzeData(upper, forwardDifference, currentAirAngle, +1d);
                    analyzeData(lower, forwardDifference, currentAirAngle, -1d);

                    // if both are done, 
                    if (upper.state == Measurement.done && lower.state == Measurement.done)
                    {
                        core.landing.minAngle = lower.angle;
                        core.landing.maxAngle = upper.angle;
                        double distance = (currentTargetRadialVector- vesselState.orbitalPosition).magnitude;
                        core.landing.factor = distance * (upper.angle - lower.angle) / (upper.difference - lower.difference) ;
                        Trajectories.API.AirAngle = origAirAngle;
                        Debug.Log(String.Format("Done analyzing vessel aerodynamic, approach angles =[{0:F1},{1:F1}], factor = {2:F4}", lower.angle, upper.angle, core.landing.factor));
                        return true;
                    }
                        
                    lastImpactRadialVector = Vector3d.zero;
                    Trajectories.API.invalidateCalculation();
                }
                else if (currentImpactRadialVector != Vector3d.zero)
                    lastImpactRadialVector = currentImpactRadialVector;

                return false;
            }

            void analyzeData(MeasureData data, double newDiff, double curAngle, double sign)
            {
                Debug.Log(String.Format("Analyze Measurement: {0:F0}° yields {1:F0} diff in {2}, prev={3:F0}° with {4:F0}", curAngle, newDiff, data.state, data.angle, data.difference));
                switch (data.state)
                {
                    case Measurement.calc:

                        if (sign * (data.difference - newDiff) > 0)
                        {
                            data.difference = newDiff;
                            data.angle = curAngle;
                            data.state = Measurement.waitCalc;
                        }
                        else
                        {
                            data.state = Measurement.waitRefine;
                        }
                        break;
                    case Measurement.refine:
                        if (sign * (data.difference - newDiff) > 0)
                        {
                            data.difference = newDiff;
                            data.angle = curAngle;
                            data.state = Measurement.waitRefine;
                        }
                        else
                        {
                            data.state = Measurement.done;
                        }
                        break;
                    case Measurement.waitCalc:
                        if (data.angle <= origAirAngle + 20)
                        {
                            data.state = Measurement.calc;
                            Trajectories.API.AirAngle = data.angle += 5 * sign;
                        }
                        else
                        {
                            data.state = Measurement.done;
                        }
                        break;
                    case Measurement.waitRefine:
                        if (data.angle <= origAirAngle + 20)
                        {
                            data.state = Measurement.refine;
                            Trajectories.API.AirAngle = data.angle += sign;
                        }
                        else
                        {
                            data.state = Measurement.done;
                        }
                        break;
                }
            }

        }
    }
}
