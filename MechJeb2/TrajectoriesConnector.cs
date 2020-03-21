using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using UnityEngine;

namespace MuMech
{
    public class TrajectoriesConnector
    {
        //ToDo: instantiate API with Reflections
        public class API
        {
            public static Vector3? GetImpactPosition() { return Trajectories.API.GetImpactPosition(); }

            public static bool isBelowEntry() { return Trajectories.API.isBelowEntry(); }

            public static bool isTransitionAlt() { return Trajectories.API.isTransitionAlt(); }

            public static double? AoA {
                get { return Trajectories.API.AoA; }
                set { Trajectories.API.AoA=value; }
            }

            public static double? nextAoA
            {
                get { return Trajectories.API.nextAoA; }
                set { Trajectories.API.nextAoA = value; }
            }

            public static bool? RetrogradeEntry { get { return Trajectories.API.RetrogradeEntry; } }

            public static void SetTarget(double lat, double lon, double? alt = null)
                { Trajectories.API.SetTarget(lat, lon, alt); }

            public static bool HasTarget() { return Trajectories.API.HasTarget(); }

            public static Quaternion? PlannedOrientation() { return Trajectories.API.PlannedOrientation(); }

            public static void invalidateCalculation() { Trajectories.API.invalidateCalculation(); }
        }

        // this is what is actually used in atmospheric landing
        public class TargetInfo
        {
            CelestialBody mainBody;
            MechJebModuleTargetController target;
            VesselState vesselState;
            public int targetOffset = 0;

            public TargetInfo(MechJebModuleTargetController target)
            {
                this.mainBody = target.mainBody;
                this.target = target;
                this.vesselState = target.vesselState;
            }
            public bool isValid = false;
            protected double timestamp;

            Vector3d currentImpactRadialVector;
            Vector3d lastImpactRadialVector;
            Vector3d currentTargetRadialVector;

            public Vector3d CurrentTargetRadialVector { get { update(); return currentTargetRadialVector; } }
            public Vector3d CurrentImpactRadialVector { get { update(); return currentImpactRadialVector; } }

            Vector3 orbitNormal;
            Vector3 orbitClosestToTarget, orbitClosestToImpact;

            public Vector3d differenceTarget;
            public Vector3d distanceTarget;
            public Vector3d distanceImpact;

            public float targetAheadAngle;

            public double normalDifference;
            public double backwardDifference;
            public double forwardDistance;

            public void update()
            {
                if (Planetarium.GetUniversalTime() != timestamp)
                {
                    Vector3? impactPos = API.GetImpactPosition();

                    if (impactPos.HasValue)
                    {
                        lastImpactRadialVector = currentImpactRadialVector;
                        currentImpactRadialVector = impactPos.Value;
                        isValid = (lastImpactRadialVector - currentImpactRadialVector).magnitude < 100;

                        currentTargetRadialVector = mainBody.GetWorldSurfacePosition(target.targetLatitude, target.targetLongitude, 0) - mainBody.position;
                        orbitNormal = target.orbit.SwappedOrbitNormal();

                        orbitClosestToImpact = Vector3.ProjectOnPlane(currentImpactRadialVector, orbitNormal);
                        orbitClosestToTarget = Vector3.ProjectOnPlane(currentTargetRadialVector, orbitNormal);
                        Vector3 targetForward = -Vector3.Cross(orbitClosestToTarget.normalized, orbitNormal);

                        if (targetOffset != 0)
                        {
                            if (targetOffset < 0.5 * UtilMath.Deg2Rad * mainBody.Radius)
                            {
                                orbitClosestToTarget += (targetOffset * targetForward); // small distance
                            }
                            else
                            {
                                //long distance: rotate target by radiant distance
                                orbitClosestToTarget = Quaternion.AngleAxis((float)(UtilMath.Rad2Deg * targetOffset / mainBody.Radius), orbitNormal) * orbitClosestToTarget;
                            }
                        }
                        differenceTarget = orbitClosestToTarget - currentImpactRadialVector;

                        //How far ahead the target is compared to impact, in degrees
                        targetAheadAngle = Vector3.SignedAngle(orbitClosestToTarget, orbitClosestToImpact, orbitNormal);
                                                
                        distanceTarget = currentTargetRadialVector - vesselState.orbitalPosition;
                        distanceImpact = currentImpactRadialVector - vesselState.orbitalPosition;
                        //calculate sideway derivation at target location, which means we have to rescale if current impact is earlier or later
                        normalDifference = (Vector3d.Dot(differenceTarget, orbitNormal) * distanceTarget.magnitude / distanceImpact.magnitude);
                        backwardDifference = -Vector3d.Dot(differenceTarget, targetForward); /* = overshoot >0, too short <0 */
                        forwardDistance = Vector3d.Dot(distanceTarget, targetForward); 
                    }
                    else
                        isValid = false;
                    timestamp = Planetarium.GetUniversalTime();
                }
            }
        }

        //this is used to control descent for atmospheric landing
        static double airAngle = 0;
        public static double AoA
        {
            get {
                airAngle = API.AoA ?? airAngle;
                
                //use angles that are continuesly around reentry orientation
                if (API.RetrogradeEntry ?? false)
                    return MuUtils.ClampDegrees360(airAngle);
                else
                    return MuUtils.ClampDegrees180(airAngle);
            } 
            set
            {
                API.AoA = value;
                API.invalidateCalculation();
            }
        }

    }
}
