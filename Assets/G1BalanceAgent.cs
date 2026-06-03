using System.Collections.Generic;
using UnityEngine;
using Unity.MLAgents;
using Unity.MLAgents.Actuators;
using Unity.MLAgents.Sensors;

public class G1BalanceAgent : Agent
{
    [Header("Robot")]
    public ArticulationBody rootBody;
    public Transform bodyRoot;

    [Header("Joint Control")]
    public float actionScale = 0.25f;
    public float stiffness = 80f;
    public float damping = 8f;
    public float forceLimit = 120f;

    [Header("Reset")]
    public float groundClearance = 0.02f;
    public float startY = 0.8f;

    private readonly List<ArticulationBody> joints = new List<ArticulationBody>();
    private Vector3 startPosition;
    private Quaternion startRotation;

    private readonly List<float> defaultTargets = new List<float>();

    public override void Initialize()
    {
        if (rootBody == null)
            rootBody = GetComponent<ArticulationBody>();

        if (bodyRoot == null)
            bodyRoot = transform;

        startPosition = transform.position;
        startPosition.y = startY;
        startRotation = transform.rotation;

        joints.Clear();
        defaultTargets.Clear();

        foreach (var ab in GetComponentsInChildren<ArticulationBody>())
        {
            if (ab == rootBody || ab.isRoot)
                continue;

            if (ab.jointType == ArticulationJointType.FixedJoint)
                continue;

            joints.Add(ab);

            var drive = ab.xDrive;
            drive.stiffness = stiffness;
            drive.damping = damping;
            drive.forceLimit = forceLimit;
            ab.xDrive = drive;

            defaultTargets.Add(drive.target);
        }

        Debug.Log("G1BalanceAgent joints: " + joints.Count);
    }

    public override void OnEpisodeBegin()
    {
        ResetRobot();
    }

    private void ResetRobot()
    {
        rootBody.TeleportRoot(startPosition, startRotation);

        foreach (var ab in GetComponentsInChildren<ArticulationBody>())
        {
            ab.linearVelocity = Vector3.zero;
            ab.angularVelocity = Vector3.zero;
        }

        for (int i = 0; i < joints.Count; i++)
        {
            var drive = joints[i].xDrive;
            drive.target = defaultTargets[i];
            drive.stiffness = stiffness;
            drive.damping = damping;
            drive.forceLimit = forceLimit;
            joints[i].xDrive = drive;
        }
    }

    public override void CollectObservations(VectorSensor sensor)
    {
        sensor.AddObservation(transform.up.y);
        sensor.AddObservation(transform.position.y);

        Vector3 localAngularVelocity = transform.InverseTransformDirection(rootBody.angularVelocity);
        Vector3 localVelocity = transform.InverseTransformDirection(rootBody.linearVelocity);

        sensor.AddObservation(localVelocity.x);
        sensor.AddObservation(localVelocity.y);
        sensor.AddObservation(localVelocity.z);

        sensor.AddObservation(localAngularVelocity.x);
        sensor.AddObservation(localAngularVelocity.y);
        sensor.AddObservation(localAngularVelocity.z);

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
            drive.target = defaultTargets[i] + action * actionScale;
            drive.stiffness = stiffness;
            drive.damping = damping;
            drive.forceLimit = forceLimit;
            joints[i].xDrive = drive;
        }

        Vector3 rootUp = rootBody.transform.up;
        float upright = Mathf.Clamp01(rootUp.y);
        float height = rootBody.transform.position.y;

        float heightReward = Mathf.Exp(-Mathf.Abs(height - startY) * 4f);

        Vector3 localAngularVelocity = rootBody.transform.InverseTransformDirection(rootBody.angularVelocity);
        Vector3 localVelocity = rootBody.transform.InverseTransformDirection(rootBody.linearVelocity);

        bool fallen =
            height < 0.45f ||
            upright < 0.50f;

        if (fallen)
        {
            AddReward(-1.0f);
            EndEpisode();
            return;
        }

        AddReward(upright * 0.01f);
        AddReward(heightReward * 0.01f);
        AddReward(0.001f);

        AddReward(-0.001f * localAngularVelocity.sqrMagnitude);
        AddReward(-0.0005f * localVelocity.sqrMagnitude);
    }
}