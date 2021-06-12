using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using TMPro;
using System.Text;
using UnityEditor;
using RosSharp;
using System;

public class ListPositionText : MonoBehaviour
{
    [Header("Scene")]
    public TextMeshPro textMesh;

    [Header("String")]
    public string stringStart;

    private string text;
    private StringBuilder stringBuilder;
    private void Start()
    {
        textMesh.text = stringStart;
    }

    // Update is called once per frame
    public void UpdateString(List<GameObject> targetList)
    {
        stringBuilder = new StringBuilder();
        stringBuilder.Append(stringStart + "\n");
        for(int i = 0; i<targetList.Count; i++)
        {
            Vector3 targetPosition = targetList[i].transform.position;
            string targetString = String.Format("[x:{0:F2}, y:{1:F2}, z:{2:F2}]", targetPosition.Unity2Ros().x, targetPosition.Unity2Ros().y, targetPosition.Unity2Ros().z);
            stringBuilder.Append((i + 1).ToString() + ": " + targetString + "\n");
        }

        textMesh.text = stringBuilder.ToString();
        gameObject.transform.localPosition = new Vector3(0, gameObject.transform.localPosition.y, (float)(-0.4 - 0.0575*targetList.Count));
        //Debug.Log(gameObject.transform.localPosition.y.ToString());
    }
}
