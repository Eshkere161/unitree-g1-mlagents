using System.Collections.Generic;
using UnityEngine;
using Unity.MLAgents;
using Unity.MLAgents.Actuators;
using Unity.MLAgents.Sensors;

public class G1SimpleAgent : Agent
{
    public Transform target;
    public Transform bodyRoot;

    public float jointRange = 8f;
    public float driveStiffness = 20f;
    public float driveDamping = 40f;
    public float driveForceLimit = 30f;

    private List<ArticulationBody> joints = new List<ArticulationBody>();

    private Vector3 startPosition;
    private Quaternion startRotation;
    private float previousDistance;

    public override void Initialize()
    {
        if (bodyRoot == null)
            bodyRoot = transform;

        startPosition = transform.position;
        startRotation = transform.rotation;

        joints.Clear();

        foreach (var ab in GetComponentsInChildren<ArticulationBody>())
        {
            if (!ab.isRoot)
            {
                joints.Add(ab);

                var drive = ab.xDrive;
                drive.stiffness = driveStiffness;
                drive.damping = driveDamping;
                drive.forceLimit = driveForceLimit;
                ab.xDrive = drive;
            }
        }

        Debug.Log("Joint agent found joints: " + joints.Count);
    }

    public override void OnEpisodeBegin()
    {
        transform.position = startPosition;
        transform.rotation = startRotation;

        foreach (var ab in GetComponentsInChildren<ArticulationBody>())
        {
            ab.linearVelocity = Vector3.zero;
            ab.angularVelocity = Vector3.zero;
        }

        target.position = startPosition + new Vector3(
            Random.Range(-2f, 2f),
            0.2f,
            Random.Range(2f, 4f)
        );

        previousDistance = Vector3.Distance(transform.position, target.position);
    }

    public override void CollectObservations(VectorSensor sensor)
    {
        Vector3 toTarget = target.position - transform.position;

        sensor.AddObservation(toTarget.x);
        sensor.AddObservation(toTarget.z);
        sensor.AddObservation(transform.position.y);
        sensor.AddObservation(transform.up.y);

        foreach (var joint in joints)
        {
            if (joint.jointPosition.dofCount > 0)
                sensor.AddObservation(joint.jointPosition[0]);
            else
                sensor.AddObservation(0f);

            if (joint.jointVelocity.dofCount > 0)
                sensor.AddObservation(joint.jointVelocity[0]);
            else
                sensor.AddObservation(0f);
        }
    }

    public override void OnActionReceived(ActionBuffers actions)
    {
        for (int i = 0; i < joints.Count && i < actions.ContinuousActions.Length; i++)
        {
            float action = Mathf.Clamp(actions.ContinuousActions[i], -1f, 1f);

            var drive = joints[i].xDrive;
            drive.target = action * jointRange;
            drive.stiffness = driveStiffness;
            drive.damping = driveDamping;
            drive.forceLimit = driveForceLimit;
            joints[i].xDrive = drive;
        }

        float upright = Mathf.Clamp01(transform.up.y);
        float height = transform.position.y;
        float distance = Vector3.Distance(transform.position, target.position);
        // награда за приближние к шару
        AddReward((previousDistance - distance) * 1.0f);

        previousDistance = distance;
        // награды за то что он не падает и штраф за педание 
        AddReward(upright * 0.02f);
        AddReward(height * 0.005f);
        AddReward(-0.001f);

        if (height < 0.3f || transform.up.y < 0.4f)
        {
            AddReward(-1.0f);
            EndEpisode();
        }
    }
}
    