using RosSharp.RosBridgeClient;
using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class UnityHandlerTest : MonoBehaviour
{
    public PoseStampedPublisher RobotPose;
    public PoseStampedPublisher VRBPose;
    public TwistControlPublisher RobotTwist;
    public Vector3 speed;

    public bool publishRobotTwist;
    public bool publishRobotPose;
    public bool publishVRBPose;

    // Update is called once per frame
    void Update()
    {
        if (publishRobotTwist)
        {
            PublishRobotTwist();
        }
        if (publishRobotPose)
        {
            PublishRobotPose();
        }
        if (publishVRBPose)
        {
            PublishVRBPose();
        }
    }

    void PublishRobotTwist()
    {
        RobotTwist.UpdateMessage(speed, new Vector3(0, 0, 0));
    }

    void PublishRobotPose()
    {
        RobotPose.UpdateMessage();
    }

    void PublishVRBPose()
    {
        VRBPose.UpdateMessage();
    }
}
