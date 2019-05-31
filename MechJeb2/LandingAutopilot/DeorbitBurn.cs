using System;
using System.Linq;
using UnityEngine;

// FIXME: use a maneuver node

namespace MuMech
{
    namespace Landing
    {
        public class DeorbitBurn : NodeExecution
        {

            public DeorbitBurn(MechJebCore core) : base(core)
            {
                doAfterExecution = new CourseCorrection(core);
            }


            public override AutopilotStep OnFixedUpdate()
            {
                  
                //if we don't want to deorbit but we're already on a reentry trajectory, we can't wait until the ideal point 
                //in the orbit to deorbt; we already have deorbited.
                if (orbit.ApA < mainBody.RealMaxAtmosphereAltitude())
                {
                    core.thrust.targetThrottle = 0;
                    return doAfterExecution;
                }

                if (node == null)
                {
                    //We aim for a trajectory that 
                    // a) has the same vertical speed as our current trajectory
                    // b) has a horizontal speed that will give it a periapsis of -10% of the body's radius
                    // c) has a heading that points toward where the target will be at the end of free-fall, accounting for planetary rotation
                    Vector3d horizontalDV = OrbitalManeuverCalculator.DeltaVToChangePeriapsis(orbit, vesselState.time, 0.9 * mainBody.Radius); //Imagine we are going to deorbit now. Find the burn that would lower our periapsis to -10% of the planet's radius
                    Orbit forwardDeorbitTrajectory = orbit.PerturbedOrbit(vesselState.time, horizontalDV);                                     //Compute the orbit that would put us on
                    double freefallTime = forwardDeorbitTrajectory.NextTimeOfRadius(vesselState.time, mainBody.Radius) - vesselState.time;     //Find how long that orbit would take to impact the ground
                    double planetRotationDuringFreefall = 360 * freefallTime / mainBody.rotationPeriod;                                        //Find how many degrees the planet will rotate during that time
                    Vector3d currentTargetRadialVector = mainBody.GetWorldSurfacePosition(core.target.targetLatitude, core.target.targetLongitude, 0) - mainBody.position; //Find the current vector from the planet center to the target landing site
                    Quaternion freefallPlanetRotation = Quaternion.AngleAxis((float)planetRotationDuringFreefall, mainBody.angularVelocity);   //Construct a quaternion representing the rotation of the planet found above
                    Vector3d freefallEndTargetRadialVector = freefallPlanetRotation * currentTargetRadialVector;                               //Use this quaternion to find what the vector from the planet center to the target will be when we hit the ground
                    Vector3d freefallEndTargetPosition = mainBody.position + freefallEndTargetRadialVector;                                    //Then find the actual position of the target at that time
                    Vector3d freefallEndHorizontalToTarget = Vector3d.Exclude(freefallEndTargetRadialVector, freefallEndTargetPosition - vesselState.CoM).normalized; //Find a horizontal unit vector that points toward where the target will be when we hit the ground
                    Vector3d currentHorizontalVelocity = Vector3d.Exclude(vesselState.up, vesselState.orbitalVelocity); //Find our current horizontal velocity
                    double finalHorizontalSpeed = (currentHorizontalVelocity + horizontalDV).magnitude;                     //Find the desired horizontal speed after the deorbit burn
                    Vector3d finalHorizontalVelocity = finalHorizontalSpeed * freefallEndHorizontalToTarget;                //Combine the desired speed and direction to get the desired velocity after the deorbi burn

                    //Compute the angle between the location of the target at the end of freefall and the normal to our orbit:
                    Vector3d currentRadialVector = vesselState.CoM - mainBody.position;
                    double targetAngleToOrbitNormal = Vector3d.Angle(orbit.SwappedOrbitNormal(), freefallEndTargetRadialVector);
                    targetAngleToOrbitNormal = Math.Min(targetAngleToOrbitNormal, 180 - targetAngleToOrbitNormal);

                    float targetAheadAngle = Vector3.SignedAngle(currentRadialVector, freefallEndTargetRadialVector, orbit.SwappedOrbitNormal()); //How far ahead the target is, in degrees
                    if (targetAheadAngle < 60) targetAheadAngle += 360;

                    float planeChangeAngle = Vector3.Angle(currentHorizontalVelocity, freefallEndHorizontalToTarget); //The plane change required to get onto the deorbit trajectory, in degrees

                    vessel.RemoveAllManeuverNodes();

                    //If the target is basically almost normal to our orbit, it doesn't matter when we deorbit; might as well do it now
                    //Otherwise, wait until the target is ahead
                    if (targetAngleToOrbitNormal < 10
                        || (targetAheadAngle < 90 && planeChangeAngle < 90))
                    {
                        vessel.PlaceManeuverNode(vessel.orbit, horizontalDV, vesselState.time);
                    }
                    else
                    {
                        double deorbitTime = vesselState.time + (targetAheadAngle - 90) * orbit.period / 360f;
                        vessel.PlaceManeuverNode(vessel.orbit, OrbitalManeuverCalculator.DeltaVToChangePeriapsis(orbit, deorbitTime, 0.9 * mainBody.Radius), deorbitTime );
                    }
                }

                return base.OnFixedUpdate();
            }
        }
    }
}
