using System;
using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class TrajectoryCreater : MonoBehaviour
{
    [Header("Scene")]
    public RosSharp.RosBridgeClient.MavTrajectoryPublisher MavTrajectoryPublisher;

    [Header("Setup")]
    public int mav_id;
    public int cam_id;
    public int poses;
    public bool publishMessage;

    private Vector3[] positions;
    private Quaternion[] orientations;

    // Update is called once per frame
    void Update()
    {
        if (publishMessage)
        {
            PublishMessage();
            publishMessage = false;
        }

    }

    private void PublishMessage()
    {
        positions = RandomPositions(poses);
        orientations = RandomQuaternions(poses);
        MavTrajectoryPublisher.UpdateMessage(positions, orientations, mav_id, cam_id);
    }

    private Vector3[] RandomPositions(int poses)
    {
        positions = new Vector3[poses];

        for(int i=0; i<poses; i++)
        {
            positions[i] = new Vector3(UnityEngine.Random.Range(0, 10), UnityEngine.Random.Range(0, 10), 
                UnityEngine.Random.Range(0, 10));
        }

        return positions;
    }

    private Quaternion[] RandomQuaternions(int poses)
    {
        orientations = new Quaternion[poses];
        for(int i=0; i<poses; i++)
        {
            orientations[i] = new Quaternion();
            orientations[i] = Quaternion.Euler(new Vector3(UnityEngine.Random.Range(0, 2*Mathf.PI), 
                UnityEngine.Random.Range(0, 2 * Mathf.PI), UnityEngine.Random.Range(0, 2 * Mathf.PI)));
        }

        return orientations;
    }
}
