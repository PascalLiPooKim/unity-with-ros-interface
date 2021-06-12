using UnityEngine;
using System.Collections;
using TMPro;

public class TextOutline : MonoBehaviour
{
    public TextMeshPro textMeshPro;
    void Awake()
    {
        textMeshPro.outlineColor = new Color32(255, 255, 255, 255);
    }
}
