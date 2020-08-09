using System;
using UnityEngine;

public class Axis : MonoBehaviour
{
   public Vector3 rotationAxis;
   public Vector3 beginLocation;
   
   public float MinAngle;
   public float MaxAngle;
   private void Awake()
   {
      rotationAxis = new Vector3(0f,0f,1f);
      beginLocation = transform.position;
   }

   public float MinAngleRadians => Mathf.Deg2Rad * MinAngle;
   public float MaxAngleRandians => Mathf.Deg2Rad * MaxAngle;

   
}
