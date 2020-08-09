using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class PenaltyColliders : MonoBehaviour
{
    public RobotControllerAgent parentAgent;

    private void OnTriggerEnter(Collider other)
    {
        if (other.transform.CompareTag("RobotInternal"))
        {
            if(parentAgent != null)
                parentAgent.GroundHitPenalty();
        }
    }
}
