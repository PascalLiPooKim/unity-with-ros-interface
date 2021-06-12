using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using Valve.VR;

public class TwistInputManager : MonoBehaviour
{
    // Start is called before the first frame update
    [Header("Actions")]
    public SteamVR_Input_Sources handType;
    public SteamVR_Action_Boolean trigger;

    [Header("Scene")]
    public TwistControl twistControl;

    private void Start()
    {
        Show(false);
    }

    public void Show(bool state)
    {
        gameObject.SetActive(state);
    }

    private void OnEnable()
    {
        trigger.onStateDown += TriggerDown;
        trigger.onStateUp += TriggerUp;
        twistControl.Show(true);
    }

    private void OnDisable()
    {
        trigger.onStateDown -= TriggerDown;
        trigger.onStateUp -= TriggerUp;
        twistControl.Show(false);
    }

    private void TriggerDown(SteamVR_Action_Boolean fromAction, SteamVR_Input_Sources fromSource)
    {
        twistControl.BeginTracking();
    }

    private void TriggerUp(SteamVR_Action_Boolean fromAction, SteamVR_Input_Sources fromSource)
    {
        twistControl.EndTracking();
    }
}
