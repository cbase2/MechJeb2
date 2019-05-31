using System;
using UnityEngine;

// FIXME: use a maneuver node

namespace MuMech
{
    namespace Landing
    {
        public class NodeExecution : AutopilotStep
        {
            protected AutopilotStep doAfterExecution = null;
            protected bool burnTriggered = false;
            protected bool alignedForBurn = false;
            protected ManeuverNode node = null;

            public NodeExecution(MechJebCore core) : base(core)
            {
            }

            public override AutopilotStep Drive(FlightCtrlState s)
            {
                if (node != null)
                {
                    double dVLeft = node.GetBurnVector(orbit).magnitude;

                    core.thrust.targetThrottle = 0;


                    if (burnTriggered && alignedForBurn)
                    {
                        if (core.attitude.attitudeAngleFromTarget() < 90)
                        {
                            double timeConstant = (dVLeft > 10 || vesselState.minThrustAccel > 0.25 * vesselState.maxThrustAccel ? 0.5 : 2);
                            core.thrust.ThrustForDV(dVLeft + core.node.tolerance, timeConstant);
                        }
                        else
                        {
                            alignedForBurn = false;
                        }
                    }
                    else
                    {
                        //aim along the node
                        core.attitude.attitudeTo(Vector3d.forward, AttitudeReference.MANEUVER_NODE, this);
                        if (core.attitude.attitudeAngleFromTarget() < 2)
                        {
                            alignedForBurn = true;
                        }
                    }
                }

                return this;
            }

            public override AutopilotStep OnFixedUpdate()
            {
                if (!vessel.patchedConicsUnlocked() || vessel.patchedConicSolver.maneuverNodes.Count == 0)
                {
                   return doAfterExecution;
                }

                node = vessel.patchedConicSolver.maneuverNodes[0];
                double dVLeft = node.GetBurnVector(orbit).magnitude;

                if (dVLeft < core.node.tolerance && core.attitude.attitudeAngleFromTarget() > 5)
                {
                    burnTriggered = false;

                    node.RemoveSelf();

                    return this; // we are done for this frame, continue in next
                }

                double halfBurnTime;
                double burnTime = core.node.BurnTime(dVLeft, out halfBurnTime);

                double timeToNode = node.UT - vesselState.time;
                status = "Moving to node";

                if ((!double.IsInfinity(halfBurnTime) && halfBurnTime > 0 && timeToNode < halfBurnTime) || timeToNode < 0)
                {
                    burnTriggered = true;
                    status = "Executing node";
                    if (!MuUtils.PhysicsRunning()) core.warp.MinimumWarp();
                }
                //autowarp, but only if we're already aligned with the node
                if (core.node.autowarp && !burnTriggered)
                {
                    if ((core.attitude.attitudeAngleFromTarget() < 1 && core.vessel.angularVelocity.magnitude < 0.001) || (core.attitude.attitudeAngleFromTarget() < 10 && !MuUtils.PhysicsRunning()))
                    {
                        core.warp.WarpToUT(node.UT - halfBurnTime - core.node.leadTime);
                    }
                    else if (!MuUtils.PhysicsRunning() && core.attitude.attitudeAngleFromTarget() > 10 && timeToNode < 600)
                    {
                        //realign
                        core.warp.MinimumWarp();
                    }
                }

                return this;
            }
        }
    }
}