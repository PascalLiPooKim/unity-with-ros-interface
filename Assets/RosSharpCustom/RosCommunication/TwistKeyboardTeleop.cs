using System.Collections;
using System.Collections.Generic;
using UnityEngine;

[RequireComponent(typeof(RosSharp.RosBridgeClient.TwistTeleopPublisher))]
public class TwistKeyboardTeleop : MonoBehaviour
{
    public float linearVelocity = 0.5f;
    public float angularVelocity = 0.5f;
    private RosSharp.RosBridgeClient.TwistTeleopPublisher twistPublisher;
    private Vector3 linearVelMsg = new Vector3();
    private Vector3 angularVelMsg = new Vector3();
    private Vector3 prevLinearMsg = new Vector3();
    private Vector3 prevAngularMsg = new Vector3();
    private bool publishLock = false;
    // Start is called before the first frame update
    void Start()
    {
        twistPublisher = GetComponent<RosSharp.RosBridgeClient.TwistTeleopPublisher>();
    }

    void Update()
    {
        if(Input.GetKeyDown(KeyCode.W)){
            linearVelMsg.z = linearVelocity;
        }
        if (Input.GetKeyUp(KeyCode.W))
        {
            linearVelMsg.z = 0;
        }
        if (Input.GetKeyDown(KeyCode.S))
        {
            linearVelMsg.z = -linearVelocity;
        }
        if (Input.GetKeyUp(KeyCode.S))
        {
            linearVelMsg.z = 0;
        }
        if (Input.GetKeyDown(KeyCode.D))
        {
            angularVelMsg.y = angularVelocity;
        }
        if (Input.GetKeyUp(KeyCode.D))
        {
            angularVelMsg.y = 0;
        }
        if (Input.GetKeyDown(KeyCode.A))
        {
            angularVelMsg.y = -angularVelocity;
        }
        if (Input.GetKeyUp(KeyCode.A))
        {
            angularVelMsg.y = 0;
        }

        if(!publishLock)
            twistPublisher.UpdateMessage(linearVelMsg, angularVelMsg);
        if(linearVelMsg == Vector3.zero && angularVelMsg == Vector3.zero)
            publishLock = true;
        else
            publishLock = false;
    }
}
