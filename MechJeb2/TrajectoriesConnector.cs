using System;
using System.Collections.Generic;
using System.Linq;
using System.Reflection;
using System.Text;
using UnityEngine;

namespace MuMech
{
    public class TrajectoriesConnector
    {
        const string assemblyname = "Trajectories";
        const string apiclass = "Trajectories.API";
        static public bool isLoadedTrajectories = ReflectionUtils.isAssemblyLoaded(assemblyname);

        public class API
        {
            public delegate Vector3? DGetTrueImpactPosition();
            public static DGetTrueImpactPosition GetTrueImpactPosition;

            public delegate double? DGetTimeTillImpact();
            public static DGetTimeTillImpact GetTimeTillImpact;

            public delegate bool DisBelowEntry();
            public static DisBelowEntry isBelowEntry;

            public delegate bool DHasTarget();
            public static DHasTarget HasTarget;

            public delegate Quaternion? DPlannedOrientation();
            public static DPlannedOrientation PlannedOrientation;

            public delegate void DinvalidateCalculation();
            public static DinvalidateCalculation invalidateCalculation;

            public delegate void DSetTarget(double lat, double lon);
            public static DSetTarget SetTarget;

            public delegate double? DAoAGetter();
            public static DAoAGetter AoAGetter;

            public delegate void DAoASetter(double? value);
            public static DAoASetter AoASetter;

            public delegate bool? DRetrogradeEntryGetter();
            public static DRetrogradeEntryGetter RetrogradeEntryGetter;

            // on class initialisation use reflections to resolve delegates
            static API()
            {
                if (isLoadedTrajectories)
                {
                    var m = ReflectionUtils.getMethodByReflection(assemblyname, apiclass, "GetTrueImpactPosition", BindingFlags.Public | BindingFlags.Static);
                    if (m != null)
                        GetTrueImpactPosition = (DGetTrueImpactPosition)Delegate.CreateDelegate(typeof(DGetTrueImpactPosition), m);
                    else
                        isLoadedTrajectories = false;

                    m = ReflectionUtils.getMethodByReflection(assemblyname, apiclass, "GetTimeTillImpact", BindingFlags.Public | BindingFlags.Static);
                    if (m != null)
                        GetTimeTillImpact = (DGetTimeTillImpact)Delegate.CreateDelegate(typeof(DGetTimeTillImpact), m);
                    else
                        isLoadedTrajectories = false;

                    m = ReflectionUtils.getMethodByReflection(assemblyname, apiclass, "isBelowEntry", BindingFlags.Public | BindingFlags.Static);
                    if (m != null)
                        isBelowEntry = (DisBelowEntry)Delegate.CreateDelegate(typeof(DisBelowEntry), m);
                    else
                        isLoadedTrajectories = false;

                    m = ReflectionUtils.getMethodByReflection(assemblyname, apiclass, "HasTarget", BindingFlags.Public | BindingFlags.Static);
                    if (m != null)
                        HasTarget = (DHasTarget)Delegate.CreateDelegate(typeof(DHasTarget), m);
                    else
                        isLoadedTrajectories = false;

                    m = ReflectionUtils.getMethodByReflection(assemblyname, apiclass, "PlannedOrientation", BindingFlags.Public | BindingFlags.Static);
                    if (m != null)
                        PlannedOrientation = (DPlannedOrientation)Delegate.CreateDelegate(typeof(DPlannedOrientation), m);
                    else
                        isLoadedTrajectories = false;

                    m = ReflectionUtils.getMethodByReflection(assemblyname, apiclass, "invalidateCalculation", BindingFlags.Public | BindingFlags.Static);
                    if (m != null)
                        invalidateCalculation = (DinvalidateCalculation)Delegate.CreateDelegate(typeof(DinvalidateCalculation), m);
                    else
                        isLoadedTrajectories = false;

                    m = ReflectionUtils.getMethodByReflection(assemblyname, apiclass, "SetTarget", BindingFlags.Public | BindingFlags.Static, new Type[] { typeof(double), typeof(double) });
                    if (m != null)
                        SetTarget = (DSetTarget)Delegate.CreateDelegate(typeof(DSetTarget), m);
                    else
                        isLoadedTrajectories = false;

                    var p = ReflectionUtils.getPropertyByReflection(assemblyname, apiclass, "AoA", BindingFlags.Public | BindingFlags.Static);
                    if (p != null)
                    {
                        AoAGetter = (DAoAGetter)Delegate.CreateDelegate(typeof(DAoAGetter), p.GetMethod);
                        AoASetter = (DAoASetter)Delegate.CreateDelegate(typeof(DAoASetter), p.SetMethod);
                    }
                    else
                        isLoadedTrajectories = false;

                    p = ReflectionUtils.getPropertyByReflection(assemblyname, apiclass, "RetrogradeEntry", BindingFlags.Public | BindingFlags.Static);
                    if (p != null)
                    {
                        RetrogradeEntryGetter = (DRetrogradeEntryGetter)Delegate.CreateDelegate(typeof(DRetrogradeEntryGetter), p.GetMethod);
                    }
                    else
                        isLoadedTrajectories = false;
                }
            }
            
            public static double? AoA {
                get { return AoAGetter(); }
                set { AoASetter(value); }
            }

            public static bool? RetrogradeEntry { get { return RetrogradeEntryGetter(); } }
        }

        // this is what is actually used in atmospheric landing
        public class TargetInfo
        {
            CelestialBody mainBody;
            MechJebModuleTargetController target;
            double targetAlt;
            VesselState vesselState;

            public TargetInfo(MechJebModuleTargetController target)
            {
                this.mainBody = target.mainBody;
                this.target = target;
                targetAlt = mainBody.TerrainAltitude(target.targetLatitude, target.targetLongitude, !mainBody.ocean);
                this.vesselState = target.vesselState;
            }
            public bool isValid = false;
            protected double timestamp;

            Vector3d trueImpactRadialVector;
            Vector3d lastImpactRadialVector;
            Vector3d trueTargetRadialVector;            
            double timeTillImpact;

            public Vector3d TrueTargetRadialVector { get { update(); return trueTargetRadialVector; } }
            public Vector3d TrueImpactRadialVector { get { update(); return trueImpactRadialVector; } }
            public double TimeTillImpact { get { update(); return timeTillImpact; } }

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
                    Vector3? impactPos = API.GetTrueImpactPosition();

                    if (impactPos.HasValue)
                    {
                        lastImpactRadialVector = trueImpactRadialVector;
                        trueImpactRadialVector = impactPos.Value;
                        timeTillImpact = API.GetTimeTillImpact().Value;

                        isValid = (lastImpactRadialVector - trueImpactRadialVector).magnitude < 100;

                        trueTargetRadialVector = mainBody.GetWorldSurfacePosition(target.targetLatitude, target.targetLongitude, targetAlt) - mainBody.position;
                        //float angle = (float)(timeTillImpact * mainBody.angularVelocity.magnitude / Math.PI * 180.0);
                        //trueTargetRadialVector = Quaternion.AngleAxis(angle, mainBody.angularVelocity.normalized) * trueTargetRadialVector;
                        trueTargetRadialVector = Quaternion.AngleAxis(-360f * (float) mainBody.rotPeriodRecip * Mathf.Lerp(0f, (float)timeTillImpact,(float) (vesselState.altitudeASL/mainBody.atmosphereDepth)), mainBody.RotationAxis) * trueTargetRadialVector;
                        orbitNormal = target.orbit.SwappedOrbitNormal(); 

                        orbitClosestToImpact = Vector3.ProjectOnPlane(trueImpactRadialVector, orbitNormal);
                        orbitClosestToTarget = Vector3.ProjectOnPlane(trueTargetRadialVector, orbitNormal);
                        Vector3 targetForward = -Vector3.Cross(orbitClosestToTarget.normalized, orbitNormal);

                        differenceTarget = orbitClosestToTarget - trueImpactRadialVector;

                        //How far ahead the target is compared to impact, in degrees
                        targetAheadAngle = Vector3.SignedAngle(orbitClosestToTarget, trueImpactRadialVector, orbitNormal);
                                                
                        distanceTarget = trueTargetRadialVector - vesselState.orbitalPosition;
                        distanceImpact = trueImpactRadialVector - vesselState.orbitalPosition;
                        //calculate sideway derivation at target location, which means we have to rescale if current impact is earlier or later
                        normalDifference = Vector3d.Dot(trueTargetRadialVector- trueImpactRadialVector, orbitNormal);
                        backwardDifference = -Vector3d.Dot(differenceTarget, targetForward); /* = overshoot >0, too short <0 */
                        forwardDistance = Vector3d.Dot(distanceTarget, targetForward); 
                    }
                    else
                        isValid = false;
                    timestamp = Planetarium.GetUniversalTime();
                }
            }
            public void invalidateCalculation()
            {
                timestamp = 0;
                isValid = false;
                API.invalidateCalculation();
            }
        }

        //convience wrapper around API AoA property
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
            }
        }
    }
}
