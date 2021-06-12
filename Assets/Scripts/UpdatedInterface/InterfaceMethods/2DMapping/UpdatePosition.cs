using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class UpdatePosition : MonoBehaviour
{
    public GameObject followObject;
    public bool xtrack;
    public bool ytrack;
    public bool ztrack;
    public bool rottrack;

    // Update is called once per frame
    void Update()
    {
        float x = 0.0f;
        float y = -0.9f;
        float z = 0.0f;
        if (xtrack)
            x = followObject.transform.position.x;
        if (ytrack)
            y = followObject.transform.position.y;
        if (ztrack)
            z = followObject.transform.position.z;
        if (rottrack)
        {
            Quaternion rotation = followObject.transform.rotation;
            gameObject.transform.rotation = rotation;
        }
           

        gameObject.transform.position = new Vector3(x, y, z);
    }
}
