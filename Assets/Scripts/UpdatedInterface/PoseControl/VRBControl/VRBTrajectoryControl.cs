using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class VRBTrajectoryControl : MonoBehaviour
{
    [Header("Scene")]
    public GameObject targetPrefab;
    public GameObject controllerObject;
    public GameObject vrbObject;

    private GameObject currentTarget;
    private GameObject activeTarget;

    // Start is called before the first frame update
    void Start()
    {
        
    }

    public void SetupGameObjects(GameObject vrbObject, GameObject controllerObject)
    {
        this.vrbObject = vrbObject;
        this.controllerObject = controllerObject;
    }

    public void SetupUI(GameObject newTargetArrow)
    {
        targetPrefab = newTargetArrow;
    }

    public void InitialiseTarget()
    {
        DeleteTarget();
        //Spawn new target, add to gameobject list and set colour to red
        if(currentTarget==null)
        {
            currentTarget = Instantiate(targetPrefab, controllerObject.transform.position, controllerObject.transform.rotation);
            currentTarget.GetComponent<ColorOrganiser>().UpdateColour(Color.red);
        }
    }

    public void TrackTarget()
    {
        //Update current target position using controller position and update UI
        currentTarget.transform.position = controllerObject.transform.position;
        Vector3 eulerAngles = controllerObject.transform.rotation.eulerAngles;
        currentTarget.transform.rotation = UnityEngine.Quaternion.Euler(0, eulerAngles.y, 0);
    }

    public void DeleteTarget()
    {
        if (currentTarget != null)
        {
            Destroy(currentTarget);
        }
    }

    public void UpdatePosition()
    {
        currentTarget.GetComponent<ColorOrganiser>().UpdateColour(Color.green);
        vrbObject.transform.position = currentTarget.transform.position;
        vrbObject.transform.rotation = currentTarget.transform.rotation;
    }
}
