using System;
using UnityEngine;
using UnityEngine.Events;

[Serializable]
public class RadialSection
{
    public TextMesh text;
    public UnityEvent onPress = new UnityEvent();
}
