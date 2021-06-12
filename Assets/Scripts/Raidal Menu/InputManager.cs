using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using Valve.VR;

public class InputManager : MonoBehaviour
{
    [Header("Actions")]
    public SteamVR_Input_Sources handType;
    public SteamVR_Action_Boolean touch = null;
    public SteamVR_Action_Boolean press = null;
    public SteamVR_Action_Vector2 touchPosition = null;

    [Header("Scene Objects")]
    public RadialMenu radialMenu= null;

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
        touch.onChange += Touch;
        press.onStateUp += PressRelease;
        press.onStateDown += PressDown;
        touchPosition.onAxis += Position;
        radialMenu.Show(true);
    }

    private void OnDisable()
    {
        touch.onChange -= Touch;
        press.onStateUp -= PressRelease;
        press.onStateDown -= PressDown;
        touchPosition.onAxis -= Position;
        radialMenu.Show(false);
    }

    //private void OnDestroy()
    //{
    //    touch.onChange -= Touch;
    //    press.onStateUp -= PressRelease;
    //    touchPosition.onAxis -= Position;
    //}

    private void Position(SteamVR_Action_Vector2 fromAction, SteamVR_Input_Sources fromSource, Vector2 axis, Vector2 delta)
    {
        radialMenu.SetTouchPosition(axis);
    }

    private void Touch(SteamVR_Action_Boolean fromAction, SteamVR_Input_Sources fromSource, bool newState)
    {
        radialMenu.ShowSelection(newState);
    }

    private void PressDown(SteamVR_Action_Boolean fromAction, SteamVR_Input_Sources fromSource)
    {
        radialMenu.HighlightSection(true);
    }

    private void PressRelease(SteamVR_Action_Boolean fromAction, SteamVR_Input_Sources fromSource)
    {
        radialMenu.ActivateHighlightedSection();
        radialMenu.HighlightSection(false);
    }


}
