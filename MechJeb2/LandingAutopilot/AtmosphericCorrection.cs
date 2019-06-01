using System;
using System.Linq;
using UnityEngine;


namespace MuMech
{
    namespace Landing
    {
        class AtmosphericCorrection : AutopilotStep
        {
            public AtmosphericCorrection(MechJebCore core) : base(core)
            {
            }

            public override AutopilotStep Drive(FlightCtrlState s)
            {
                // primary directions come from trajectories settings
                Quaternion plannedOrientation = (Quaternion) Trajectories.API.PlannedOrientation();

                status = "Holding planned descent attitude";

                Vector3d currentImpactRadialVector = (Vector3d)Trajectories.API.GetImpactPosition(); // relativ Vector to impact position on surface at current time
                Vector3d currentTargetRadialVector = mainBody.GetWorldSurfacePosition(core.target.targetLatitude, core.target.targetLongitude, 0) - mainBody.position;

                Vector3d differenceTarget = currentTargetRadialVector - currentImpactRadialVector;

                if (Vector3d.Dot(differenceTarget, vesselState.forward) > 0)
                {
                    // needed correction can be applied by thrust 
                    // 2500m over target is okay, we use parachutes to stop that
                    core.thrust.targetThrottle = Mathf.Clamp01(((float)differenceTarget.magnitude - 2500f) / 1000f);
                }
                else
                {
                    // turn plannedOrientation towards Reference(=velocity), this should reduce drag and maybe we get more distance covered
                    // with some lift it makes more sense to increase rotation and hope lift increases as well
                    core.thrust.targetThrottle = 0;
                    if (vesselState.liftUp > 0.1 * Mathf.Abs((float)vesselState.drag))
                    {
                        plannedOrientation = Quaternion.SlerpUnclamped(plannedOrientation, Quaternion.identity, -0.2f);
                    }
                    else
                    {
                        plannedOrientation = Quaternion.Slerp(plannedOrientation, Quaternion.identity, 0.2f);
                    }
                }

                core.attitude.attitudeTo(plannedOrientation, AttitudeReference.SURFACE_VELOCITY, this);

                if (vesselState.altitudeASL < 1000)
                    return new FinalDescent(core);

                return this;
            }
        }
    }
}
