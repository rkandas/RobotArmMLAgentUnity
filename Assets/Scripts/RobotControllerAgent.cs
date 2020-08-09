using System;
using System.Data.Common;
using System.Linq;
using Unity.MLAgents;
using Unity.MLAgents.Sensors;
using UnityEngine;
using Random = UnityEngine.Random;

public class RobotControllerAgent : Agent
{
   [SerializeField]
   private GameObject[] armAxes;
   [SerializeField] 
   private GameObject endEffector;

   public bool trainingMode;
   private bool inFrontOfComponent = false;
   public GameObject nearestComponent;
   private Ray rayToTest = new Ray();
   private float[] angles = new float[5];
   private KinematicsCalculator calculator;
   private float beginDistance;
   private float prevBest;
   bool  inrange2 = false, inrange3 = false, inrange4 = false, inrange5 = false;
   private float baseAngle;
   private const float stepPenalty = -0.0001f;
   private void Start()
   {
      
   }

   public override void Initialize()
   {
      ResetAllAxis();
      MoveToSafeRandomPosition();
      if (!trainingMode) MaxStep = 0;
   }

   private void ResetAllAxis()
   {
      armAxes.All(c =>
      {
         c.transform.localRotation =  Quaternion.Euler(0f, 0f, 0f);
         return true;
      });
   }

   public override void OnEpisodeBegin()
   {
      if(trainingMode)
         ResetAllAxis();

      MoveToSafeRandomPosition();
      UpdateNearestComponent();
   }

   private void UpdateNearestComponent()
   {
      if (trainingMode)
      {
         inFrontOfComponent = UnityEngine.Random.value > 0.5f;
      }
      if(!inFrontOfComponent)
         nearestComponent.transform.position = transform.position + new Vector3(Random.Range(0.3f,0.6f),Random.Range(0.1f,0.3f), Random.Range(0.3f,0.6f));
      else
      {
         nearestComponent.transform.position = endEffector.transform.TransformPoint(Vector3.zero) + new Vector3(Random.Range(0.01f,0.15f),Random.Range(0.01f,0.15f), Random.Range(0.01f,0.15f));
      }
      beginDistance = Vector3.Distance(endEffector.transform.TransformPoint(Vector3.zero), nearestComponent.transform.position);
      prevBest = beginDistance;
      
      baseAngle = Mathf.Atan2( transform.position.x - nearestComponent.transform.position.x, transform.position.z - nearestComponent.transform.position.z) * Mathf.Rad2Deg;
      if (baseAngle < 0) baseAngle = baseAngle + 360f;
   }

   /// <summary>
   /// Markov Decision Process - Observes state for the current time step
   /// </summary>
   /// <param name="sensor"></param>
   public override void CollectObservations(VectorSensor sensor)
   {
      sensor.AddObservation(angles);
      sensor.AddObservation(transform.position.normalized);
      sensor.AddObservation(nearestComponent.transform.position.normalized);
      sensor.AddObservation(endEffector.transform.TransformPoint(Vector3.zero).normalized);
      Vector3 toComponent = (nearestComponent.transform.position - endEffector.transform.TransformPoint(Vector3.zero));
      sensor.AddObservation(toComponent.normalized);
      sensor.AddObservation(Vector3.Distance(nearestComponent.transform.position,endEffector.transform.TransformPoint(Vector3.zero)));
      sensor.AddObservation(StepCount / 5000);
   }

   public override void OnActionReceived(float[] vectorAction)
   {
      angles = vectorAction;
      if (trainingMode)
      {
         // Translate the floating point actions into Degrees of rotation for each axis
         armAxes[0].transform.localRotation =
            Quaternion.AngleAxis(angles[0] * 180f, armAxes[0].GetComponent<Axis>().rotationAxis);
         armAxes[1].transform.localRotation =
            Quaternion.AngleAxis(angles[1] * 90f, armAxes[1].GetComponent<Axis>().rotationAxis);
         armAxes[2].transform.localRotation =
            Quaternion.AngleAxis(angles[2] * 180f, armAxes[2].GetComponent<Axis>().rotationAxis);
         armAxes[3].transform.localRotation =
            Quaternion.AngleAxis(angles[3] * 90f, armAxes[3].GetComponent<Axis>().rotationAxis);
         armAxes[4].transform.localRotation =
            Quaternion.AngleAxis(angles[4] * 90f, armAxes[4].GetComponent<Axis>().rotationAxis);

         float distance = Vector3.Distance(endEffector.transform.TransformPoint(Vector3.zero),
            nearestComponent.transform.position);
         float diff = beginDistance - distance;
         
         if (distance > prevBest)
         {
            // Penalty if the arm moves away from the closest position to target
            AddReward(prevBest - distance);
         }
         else
         {
            // Reward if the arm moves closer to target
            AddReward(diff);
            prevBest = distance;
         }
         AddReward(stepPenalty);
      }
   }

   public void GroundHitPenalty()
   {
      AddReward(-1f);
      EndEpisode();
   }

   private void OnTriggerEnter(Collider other)
   {
      JackpotReward(other);
   }

   public void JackpotReward(Collider other)
   {
      if (other.transform.CompareTag("Components"))
      {
         float SuccessReward = 0.5f;
         float bonus = Mathf.Clamp01(Vector3.Dot(nearestComponent.transform.up.normalized,
                          endEffector.transform.up.normalized));
         float reward = SuccessReward + bonus;
         if (float.IsInfinity(reward) || float.IsNaN(reward)) return;
          Debug.LogWarning("Great! Component reached. Positive reward:" + reward );
         AddReward(reward);
         //EndEpisode();
         UpdateNearestComponent();
      }
   }
   

   // private float[] NormalizedAngles()
   // {
   //    float[] normalized = new float[6];
   //    for (int i = 0; i < 6; i++)
   //    {
   //       normalized[i] = angles[i] / 360f;
   //    }
   //
   //    return normalized;
   // }

   private void MoveToSafeRandomPosition()
   {
      int maxTries = 100;
      
      while (maxTries > 0)
      {
         armAxes.All(axis =>
            {
               Axis ax = axis.GetComponent<Axis>();
               Vector3 angle = ax.rotationAxis * Random.Range(ax.MinAngle, ax.MaxAngle);
               ax.transform.localRotation = Quaternion.Euler(angle.x, angle.y, angle.z);
               return true;
            }
         );
         Vector3 tipPosition = endEffector.transform.TransformPoint(Vector3.zero);
         Plane groundPlane = new Plane(Vector3.up, Vector3.zero);
         float distanceFromGround = groundPlane.GetDistanceToPoint(tipPosition);
         if (distanceFromGround > 0.1f && distanceFromGround <= 1f && tipPosition.y > 0.01f)
         {
            break;
         }
         maxTries--;
      }
   }
   private void Update()
   {
      if(nearestComponent != null)
         Debug.DrawLine(endEffector.transform.position,nearestComponent.transform.position, Color.green);
   }
}
