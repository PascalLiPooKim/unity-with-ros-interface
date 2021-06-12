using System;
using UnityEngine;
using UnityEngine.Events;

[Serializable]
public class SelectionSection
{
    public string SectionName;
    public UnityEvent onPressDown = new UnityEvent();
    public UnityEvent onPress = new UnityEvent();
    public UnityEvent onPressRelease = new UnityEvent();
}