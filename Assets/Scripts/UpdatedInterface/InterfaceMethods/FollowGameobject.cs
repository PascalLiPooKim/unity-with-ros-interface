using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class FollowGameobject : MonoBehaviour
{
    private GameObject followObject;
    private bool trackPos;
    private bool trackRot;

    private Quaternion orientation;
    private Vector3 position;
    private Vector3 offset;
    
    // Start is called before the first frame update
    void Start()
    {
        offset = new Vector3(0,0,0);
    }

    // Update is called once per frame
    void Update()
    {
        position = new Vector3(0, 0, 0);
        if(trackPos)
            position = followObject.transform.position;

        orientation = Quaternion.identity;
        if (trackRot)
            orientation = followObject.transform.rotation;

        gameObject.transform.position = position + offset;
        gameObject.transform.rotation = orientation;
    }

    public void ObjectTrack(bool trackPos, bool trackRot)
    {
        this.trackPos = trackPos;
        this.trackRot = trackRot;
    }

    public void SetOffset(Vector3 offset)
    {
        this.offset = offset;
    }

    public void UpdateObject(GameObject followObject)
    {
        if (followObject == null)
            ObjectTrack(false, false);

        this.followObject = followObject;
    }
}
