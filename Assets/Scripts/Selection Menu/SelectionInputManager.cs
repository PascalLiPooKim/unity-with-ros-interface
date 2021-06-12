using System;
using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using UnityEngine.Events;
using Valve.VR;

[System.Serializable]
public class BoolEvent : UnityEvent<bool> { }
[System.Serializable]
public class Vector2Event : UnityEvent<Vector2> { }
public class SelectionInputManager : MonoBehaviour
{
    [Header("Actions")]
    public SteamVR_Action_Boolean press = null;
    public SteamVR_Action_Vector2 touchPosition = null;
    public SteamVR_Action_Boolean touch = null;
    public SteamVR_Action_Boolean trigger = null;

    [Header("Events")]
    [SerializeField] public BoolEvent touchEvent;
    [SerializeField] public UnityEvent pressDownEvent;
    [SerializeField] public UnityEvent pressEvent;
    [SerializeField] public UnityEvent pressReleaseEvent;
    [SerializeField] public Vector2Event touchPositionEvent;
    [SerializeField] public UnityEvent triggerDownEvent;
    [SerializeField] public UnityEvent triggerUpEvent;
    public SelectionMenu selectionMenu;

    private void OnEnable()
    {
        press.onStateDown += PressDown;
        press.onState += Press;
        touch.onChange += Touch;
        press.onStateUp += PressRelease;
        touchPosition.onAxis += Position;
        trigger.onStateUp += TriggerRelease;
        trigger.onStateDown += TriggerDown;
    }

    private void OnDestroy()
    {
        press.onStateDown -= PressDown;
        press.onState -= Press;
        touch.onChange -= Touch;
        press.onStateUp -= PressRelease;
        touchPosition.onAxis -= Position;
        trigger.onStateUp -= TriggerRelease;
        trigger.onStateDown -= TriggerDown;
    }
    private void OnDisable()
    {
        press.onStateDown -= PressDown;
        press.onState -= Press;
        touch.onChange -= Touch;
        press.onStateUp -= PressRelease;
        touchPosition.onAxis -= Position;
        trigger.onStateUp -= TriggerRelease;
        trigger.onStateDown -= TriggerDown;
    }

    private void Touch(SteamVR_Action_Boolean fromAction, SteamVR_Input_Sources fromSource, bool newState)
    {
        //selectionMenu.showCursor(newState);
        //selectionMenu.showSelection(newState);
        Boolean state = newState;
        touchEvent.Invoke(state);
    }

    private void Position(SteamVR_Action_Vector2 fromAction, SteamVR_Input_Sources fromSource, Vector2 axis, Vector2 delta)
    {
        //selectionMenu.SetTouchPosition(axis);
        touchPositionEvent.Invoke(axis);
    }

    private void PressDown(SteamVR_Action_Boolean fromAction, SteamVR_Input_Sources fromSource)
    {
        //  selectionMenu.ActivateHighlightedSection(); 
        pressDownEvent.Invoke();
    }

    private void Press(SteamVR_Action_Boolean fromAction, SteamVR_Input_Sources fromSource)
    {
        pressEvent.Invoke();
    }

    private void PressRelease(SteamVR_Action_Boolean fromAction, SteamVR_Input_Sources fromSource)
    {
        //  selectionMenu.ActivateHighlightedSection();
        pressReleaseEvent.Invoke();
    }

    private void TriggerDown(SteamVR_Action_Boolean fromAction, SteamVR_Input_Sources fromSource)
    {
        //selectionMenu.ActivateHighlightedSection();
        triggerDownEvent.Invoke();
    }

    private void TriggerRelease(SteamVR_Action_Boolean fromAction, SteamVR_Input_Sources fromSource)
    {
        //selectionMenu.ActivateHighlightedSection();
        triggerUpEvent.Invoke();
    }
}
