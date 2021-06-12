using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using UnityEngine.Events;

public class MaveStateUI : MonoBehaviour
{
    [Header("Scene")]
    public RosSharp.RosBridgeClient.MavStateSubscriber mavStateSubscriber;
    public SpriteRenderer sprite1;
    public SpriteRenderer sprite2;
    public SpriteRenderer sprite3;
    public SpriteRenderer sprite4;



    [Header("Latency Check")]
    //Variables for latencyCheck
    private bool trackLatency = false;
    private double? time = null;
    private double? prevTime = null;
    private bool prevLatencyCheck = true;

    private bool RTSPTriggerBool = true;
    private bool StateTriggerBool = true;


    void Start()
    {
        sprite1.color = sprite2.color = sprite3.color = Color.red;
    }

    private void Update()
    {
        if (RTSPTriggerBool & StateTriggerBool)
            UpdateSprite(sprite4, true);
        else
            UpdateSprite(sprite4, false);
    }

    public void UpdateUI(bool connected, bool armed, bool ready) 
    {
        UpdateSprite(sprite1, connected);
        UpdateSprite(sprite2, armed);
        UpdateSprite(sprite3, ready);
    }

    private void UpdateSprite(SpriteRenderer sprite, bool data)
    {
        if (data)
            sprite.color = Color.green;
        else
            sprite.color = Color.red;
    }



    public void RTSPTrigger(bool data)
    {
        RTSPTriggerBool = data;
    }

    public void StateTrigger(bool data)
    {
        StateTriggerBool = data;
    }
}
