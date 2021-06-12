using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using Valve.VR;

public class TrajectoryInputManager : MonoBehaviour
{
    [Header("Actions")]
    public SteamVR_Input_Sources handType;
    public SteamVR_Action_Boolean press = null;
    public SteamVR_Action_Boolean trigger = null;

    [Header("Scene")]
    public TrajectoryControl trajectoryContol;

    // Start is called before the first frame update
    private void OnEnable()
    {
        trigger.onStateUp += TriggerRelease;
        press.onStateDown += PressDown;
        press.onState += Press;
    }

    private void OnDisable ()
    {
        trigger.onStateUp -= TriggerRelease;
        press.onStateDown -= PressDown;
        press.onState -= Press;
    }


    private void PressDown(SteamVR_Action_Boolean fromAction, SteamVR_Input_Sources fromSource)
    {

    }

    private void Press(SteamVR_Action_Boolean fromAction, SteamVR_Input_Sources fromSource)
    {

    }

    private void TriggerRelease(SteamVR_Action_Boolean fromAction, SteamVR_Input_Sources fromSource)
    {

    }
    // Update is called once per frame
    void Update()
    {
        
    }
}
