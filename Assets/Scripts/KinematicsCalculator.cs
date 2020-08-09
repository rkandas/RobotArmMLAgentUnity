using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class KinematicsCalculator
{
   private GameObject[] axes;

   public KinematicsCalculator(GameObject[] axes, float samplingDistance, float learningRate)
   {
      this.axes = axes;
      SamplingDistance =samplingDistance;
      LearningRate = learningRate;
      DistanceThreshold = 0.001f;
   }

   private Vector3 ForwardKinematics(float[] angles)
   {
      Vector3 joint = axes[0].transform.position;
      Quaternion rotation = Quaternion.identity;

      for (int i = 1; i < axes.Length; i++)
      {
         rotation *= Quaternion.AngleAxis(angles[i-1],axes[i-1].GetComponent<Axis>().rotationAxis);
         Vector3 nextJoint = joint + rotation * axes[i].GetComponent<Axis>().beginLocation;
         joint = nextJoint;
      }

      return joint;
   }

   private float DistanceFromTarget(Vector3 targetPosition, float[] angles)
   {
      Vector3 nextPosition = ForwardKinematics(angles);
      return Vector3.Distance(nextPosition,targetPosition);
   }

   public void InverseKinematics(Vector3 targetPosition, float[] angles)
   {
      float distance = DistanceFromTarget(targetPosition, angles);
      if ( distance < DistanceThreshold)
         return;
      
      for (int i = 0; i < axes.Length; i ++)
      {
         float gradient = PartialGradient(targetPosition, angles, i);
         angles[i] -= LearningRate * gradient;
         
         //angles[i] = Mathf.Clamp(angles[i], axes[i].GetComponent<Axis>().MinAngleRadians, axes[i].GetComponent<Axis>().MaxAngleRandians);
         distance = DistanceFromTarget(targetPosition, angles);
         Debug.Log(distance);
         if (distance < DistanceThreshold)
         {
            return;
         }
      }
   }

   public double DistanceThreshold { get; set; }

   public float LearningRate { get; set; }

   private float PartialGradient (Vector3 target, float[] angles, int jointIndex)
   {
      float angle = angles[jointIndex];

      float f_x = DistanceFromTarget(target, angles);
 
      angles[jointIndex] += SamplingDistance;
      float f_x_plus_d = DistanceFromTarget(target, angles);
 
      float gradient = (f_x_plus_d - f_x) / SamplingDistance;

      angles[jointIndex] = angle;
 
      return gradient;
   }

   public float SamplingDistance { get; set; }
}
