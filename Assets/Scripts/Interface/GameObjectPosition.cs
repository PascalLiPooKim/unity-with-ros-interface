using RosSharp;
using System;
using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using UnityEngine.UI;
using TMPro;

public class GameObjectPosition: MonoBehaviour
{
    [Header("Scene")]
    public GameObject gameObject;
    public TextMeshPro textMesh;

    [Header("String")]
    public string stringStart;

    private Vector3 gameObjectPosition;

    void Update()
    {
        gameObjectPosition = gameObject.transform.position;
        textMesh.text = String.Format(stringStart + "[x:{0:F2}, y:{1:F2}, z:{2:F2}]", gameObjectPosition.Unity2Ros().x, gameObjectPosition.Unity2Ros().y, gameObjectPosition.Unity2Ros().z);
    }
}
