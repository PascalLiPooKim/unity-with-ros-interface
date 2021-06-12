using RosSharp;
using System;
using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using UnityEngine.UI;

public class PoseText : MonoBehaviour
{
    [Header("Scene")]
    public GameObject gameObject;
    public Text text;

    private Vector3 gameObjectPosition;

    // Update is called once per frame
    void Update()
    {
        gameObjectPosition = gameObject.transform.position;
        text.text = String.Format("x:{0:F2}, y:{1:F2}, z:{2:F2}", gameObjectPosition.Unity2Ros().x, gameObjectPosition.Unity2Ros().y, gameObjectPosition.Unity2Ros().z);
    }
}
