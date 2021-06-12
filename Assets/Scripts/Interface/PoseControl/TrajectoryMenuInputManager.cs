using System.Collections;
using System.Collections.Generic;
using System.Linq.Expressions;
using UnityEngine;
using Valve.VR;

public class TrajectoryMenuInputManager : MonoBehaviour
{
    [Header("Actions")]
    public SteamVR_Input_Sources handType;
    public SteamVR_Action_Boolean press = null;
    public SteamVR_Action_Vector2 touchPosition = null;
    public SteamVR_Action_Boolean touch = null;
    public SteamVR_Action_Boolean trigger = null;

    [Header("Scene")]
    public PoseMenu poseMenu = null;
    public TrajectoryControl trajectoryControl = null;

    private Vector2 touchAxis;
    private bool RTSPTriggerBool = true;
    private bool stateTriggerBool = true;

    private void OnEnable()
    {
        touch.onChange += Touch;
        press.onStateDown += PressDown;
        press.onState += Press;
        press.onStateUp += PressRelease;
        touchPosition.onAxis += Position;
        trigger.onStateUp += TriggerRelease;
    }

    private void OnDestroy()
    {
        touch.onChange -= Touch;
        press.onStateDown -= PressDown;
        press.onState -= Press;
        press.onStateUp -= PressRelease;
        touchPosition.onAxis -= Position;
        trigger.onStateUp -= TriggerRelease;
    }

    private void Touch(SteamVR_Action_Boolean fromAction, SteamVR_Input_Sources fromSource, bool newState)
    {
        poseMenu.ShowCursor(newState);
        poseMenu.ShowSelection(newState);
    }

    private void Position(SteamVR_Action_Vector2 fromAction, SteamVR_Input_Sources fromSource, Vector2 axis, Vector2 delta)
    {
        poseMenu.SetTouchPosition(axis);
        touchAxis = axis;
    }

    private void PressDown(SteamVR_Action_Boolean fromAction, SteamVR_Input_Sources fromSource)
    {
        if (touchAxis.y > 0)
            trajectoryControl.InitialiseTarget();
        else
            trajectoryControl.DeleteLastTarget();
    }

    private void Press(SteamVR_Action_Boolean fromAction, SteamVR_Input_Sources fromSource)
    {
        poseMenu.HighlightSection(true);
        if (touchAxis.y > 0)
        {
            trajectoryControl.TrackTarget();
        }
    }

    private void PressRelease(SteamVR_Action_Boolean fromAction, SteamVR_Input_Sources fromSource)
    {
        poseMenu.HighlightSection(false);
    }

    private void TriggerRelease(SteamVR_Action_Boolean fromAction, SteamVR_Input_Sources fromSource)
    {
        if(RTSPTriggerBool && stateTriggerBool)
            trajectoryControl.UpdateMessage();
    }

    private void Update()
    {
    }
    public void RTSPTrigger(bool data)
    {
        RTSPTriggerBool = data;
    }

    public void StateTrigger(bool data)
    {
        stateTriggerBool = data;
    }
}
