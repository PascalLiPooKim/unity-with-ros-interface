using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class ColorOrganiser : MonoBehaviour
{
    public Color color;
    private Component[] components;
    // Start is called before the first frame update
    void Awake()
    {
    }

    void Update()
    {
        //UpdateColour(color);
    }

    public void UpdateColour(Color color)
    {
        for (int i = 0; i < gameObject.transform.childCount; i++)
            gameObject.transform.GetChild(i).gameObject.GetComponent<MeshRenderer>().material.color = color;
    }
}
