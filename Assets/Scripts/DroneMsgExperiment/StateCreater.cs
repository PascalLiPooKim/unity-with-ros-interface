using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class StateCreater : MonoBehaviour
{
    [Header("Scene")]
    public RosSharp.RosBridgeClient.MavStatePublisher mavStatePublisher;

    [Header("Setup")]
    public int mav_id;
    public int cam_id;
    public bool connected;
    public bool armed;
    public bool ready;
    public bool PublishMessage;

    // Update is called once per frame
    void Update()
    {
        if (PublishMessage)
            mavStatePublisher.UpdateMessage(mav_id, cam_id, connected, armed, ready);
    }
}
