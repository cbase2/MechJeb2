using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;

namespace MuMech.LandingAutopilot
{
    class AtmosphericBehaviour
    {
        // primary directions come from trajectories, but we may change them to stay on course
        Vector3d currentImpactRadialVector;
        Vector3d currentTargetRadialVector;

        Vector3d differenceTarget;
        Vector3d distanceTarget;
        Vector3d distanceImpact;
        //calculate sideway derivation at target location, which means we have to rescale if current impact is earlier or later
        double normalDifference;
        double backwardDifference;
        double forwardDistance;
        MechJebCore core;
        VesselState vesselState;
        CelestialBody mainBody;

        public AtmosphericBehaviour(MechJebCore core)
        {
            this.core = core;
            vesselState = core.vesselState;
            mainBody = core.part.vessel.mainBody;
        }

        public void getcurrentValues()
        {
            // primary directions come from trajectories, but we may change them to stay on course
            currentImpactRadialVector = (Vector3d)Trajectories.API.GetImpactPosition(); // relativ Vector to impact position on surface at current time
            currentTargetRadialVector = mainBody.GetWorldSurfacePosition(core.target.targetLatitude, core.target.targetLongitude, 0) - mainBody.position;

            differenceTarget = currentTargetRadialVector - currentImpactRadialVector;
            distanceTarget = currentTargetRadialVector - vesselState.orbitalPosition;
            distanceImpact = currentImpactRadialVector - vesselState.orbitalPosition;
            //calculate sideway derivation at target location, which means we have to rescale if current impact is earlier or later
            normalDifference = (Vector3d.Dot(differenceTarget, vesselState.normalPlusSurface) * distanceTarget.magnitude / distanceImpact.magnitude);
            backwardDifference = -Vector3d.Dot(differenceTarget, vesselState.horizontalSurface); /* = overshoot >0, too short <0 */
            forwardDistance = Vector3d.Dot(distanceTarget, vesselState.horizontalSurface);
        }


    }
}
