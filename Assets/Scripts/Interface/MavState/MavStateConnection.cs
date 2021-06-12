using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using UnityEngine.Events;

public class MavStateConnection : MonoBehaviour
{
    [Header("Scene")]
    public RosSharp.RosBridgeClient.MavStateSubscriber mavStateSubscriber;

    [Header("Latency Check")]
    //Variables for latencyCheck
    private double? time = null;
    private double? prevTime = null;
    private bool prevLatencyCheck = true;

    [System.Serializable]
    public class StateTrigger : UnityEvent<bool> { };
    public StateTrigger stateTrigger;

    //Msg Variables
    private RosSharp.RosBridgeClient.MessageTypes.Std.Header header;
    private int update_rate;

    // Start is called before the first frame update
    void Start()
    {
        
    }

    private void Update()
    {
        if (mavStateSubscriber.isMessageReceived)
        {
            header = mavStateSubscriber.header;
            update_rate = mavStateSubscriber.update_rate;
            LatencyCheck();
        }
    }

    private void LatencyCheck()
    {
        if (prevTime == null)
        {
            HeaderTimeStampConvert(prevTime);
        }

        else
        {
            HeaderTimeStampConvert(time);
            bool latencyCheck = ((time - prevTime) > 2 / update_rate);
            if (latencyCheck && latencyCheck == prevLatencyCheck)
            {
                stateTrigger.Invoke(false);
                prevLatencyCheck = false;
            }

            else if (!latencyCheck & latencyCheck == prevLatencyCheck)
            {
                stateTrigger.Invoke(true);
                prevLatencyCheck = true;
            }
        }
    }

    private double? HeaderTimeStampConvert(double? time)
    {
        time = header.stamp.secs + (header.stamp.nsecs * 1e-9);
        return time;
    }


}
