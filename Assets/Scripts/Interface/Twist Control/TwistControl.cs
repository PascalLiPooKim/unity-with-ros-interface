using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class TwistControl : MonoBehaviour
{
    [Header("Scene")]
    public GameObject controller;
    public RosSharp.RosBridgeClient.TwistControlPublisher twistControlPublisher;
    public GroupOrganiser groupOrganiser;
    public MavrosInfo mavrosInfo;

    [Header("Visualisation")]
    public GameObject directionArrow;
    public GameObject rotationArrow;
    public GameObject robot;
    public Material LineMaterial;

    private Vector3 startPos;
    private Quaternion startRot;

    private Vector3 publishPos;
    private Vector3 publishRot;
    private bool trackRot = true;
    private bool trackPos = true;
    private bool track = false;
    private LineRenderer line;

    [Header("Debug")]
    public float startRotation;
    public float controllerRotation;

    private void Start()
    {
        Debug.Log("twist control start");
        SetupLine();
        //Show(false);
    }

    public void SetupUI(GameObject newDirectionArrow, GameObject newRotationArrow)
    {
        directionArrow = newDirectionArrow;
        rotationArrow = newRotationArrow;
        directionArrow.SetActive(false);
        rotationArrow.SetActive(false);
    }

    public void SetupGameObjects(GameObject newRobot, GameObject newController)
    {
        controller = newController;
        robot = newRobot;
    }

    public void LockPosition()
    {
        trackPos = !trackPos;
    }

    public void LockRotation()
    {
        trackRot = !trackRot;
    }

    public void SetupLine()
    {
        line = gameObject.AddComponent<LineRenderer>();
        line.material = LineMaterial;
        line.sortingOrder = 5;
        line.positionCount = 2;
        line.SetPosition(0, new Vector3(0, 0, 0));
        line.SetPosition(1, new Vector3(0, 0, 0));
        line.startWidth = 0f;
        line.endWidth = 0f;
        line.useWorldSpace = true;
    }

    public void Show(bool state)
    {
        gameObject.SetActive(state);
    }

    void Update()
    {
        if (track)
        {
            rotationArrow.SetActive(trackRot);
            directionArrow.SetActive(trackPos);

            if (trackPos)
            {
                publishPos = trackPosition();
                UpdateDirectionArrow();
            }
            else
                publishPos = new Vector3(0, 0, 0);
            if (trackRot)
            {
                publishRot = trackRotation();
                UpdateRotationArrow();
            }
            else
                publishRot = new Vector3(0, 0, 0);

            Debug.Log(publishPos);
            twistControlPublisher.UpdateMessage(directionArrow.transform.position - robot.transform.position, publishRot);
            groupOrganiser.UpdateRobotVRBPosition(mavrosInfo.mavID);
        }
        //else
        //    twistControlPublisher.UpdateMessage(new Vector3(0,0,0), new Vector3(0,0,0));
    }

    public void BeginTracking()
    {
        SetStart();
        track = true;
        directionArrow.SetActive(true);
        rotationArrow.SetActive(true);
    }

    public void SetStart()
    {
        startPos = controller.transform.localPosition;
        startRot = controller.transform.localRotation;
    }

    public Vector3 trackPosition()
    {
        Vector3 position = controller.transform.localPosition - startPos;
        return position;
    }

    public Vector3 trackRotation()
    {
        float startAngle = startRot.eulerAngles.z;
        float controllerAngle = controller.transform.rotation.eulerAngles.z;
        float switchAngle = startAngle + 180;
        float normAngle = 0;
        if (switchAngle > 360)
            switchAngle -= 360;

        if(startAngle < 180)
        {
            if (controllerAngle > switchAngle)
                normAngle = (-360 + controllerAngle) - startAngle;
            else
                normAngle = controllerAngle - startAngle;    
        }
        else
        {   
            if(controllerAngle < switchAngle)
                normAngle = (360 + controllerAngle) - startAngle;         
            else
                normAngle = controllerAngle - startAngle;
        }

        return new Vector3(0, -normAngle, 0) * Mathf.Deg2Rad;
    }

    private void UpdateDirectionArrow()
    {
        directionArrow.transform.localPosition = publishPos;
        line.startWidth = .1f;
        line.endWidth = .1f;
        line.SetPosition(0, robot.transform.position);
        line.SetPosition(1, directionArrow.transform.position);
    }

    private void UpdateRotationArrow()
    {
        float ratio = publishRot.y / Mathf.PI;
        rotationArrow.transform.localScale = new Vector3(ratio, ratio, Mathf.Abs(ratio));
    }

    public void EndTracking()
    {
        track = false;
        rotationArrow.SetActive(false);
        directionArrow.SetActive(false);
        line.startWidth = 0f;
        line.endWidth = 0f;
        twistControlPublisher.UpdateMessage(new Vector3(0,0,0), new Vector3(0,0,0));

    }
}

