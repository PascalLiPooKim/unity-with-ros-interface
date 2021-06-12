using RosSharp;
using RosSharp.RosBridgeClient.MessageTypes.Geometry;
using System.Collections;
using System.Collections.Generic;
using System.Linq;
using UnityEngine;

public class MavTrajectoryContol : MonoBehaviour
{
    [Header("Scene")]
    public GameObject targetPrefab;
    public GameObject controllerObject;
    public GameObject droneObject;
    public ListPositionText targetListText;

    [Header("Drone Info")]
    public int mav_id;
    public int cam_id;

    private GameObject currentTarget;
    public List<GameObject> targetList;
    private UnityEngine.Vector3 currentTargetPos;
    

    public RosSharp.RosBridgeClient.MavTrajectoryPublisher mavTrajectoryPublisher;

    private void Start()
    {
        targetList = new List<GameObject>();
    }

    private void OnEnable()
    {
        Debug.Log("Enabled");
        targetList = new List<GameObject>();
    }

    public void SetupUI(GameObject newTargetArrow)
    {
        targetPrefab = newTargetArrow;
    }

    public void SetupGameObjects(GameObject newRobot, GameObject newController)
    {
        droneObject = newRobot;
        controllerObject = newController;
    }

    public void InitialiseTarget()
    {
        //Spawn new target, add to gameobject list and set colour to red
        currentTarget = Instantiate(targetPrefab, controllerObject.transform.position, controllerObject.transform.rotation);
        targetList.Add(currentTarget);
        currentTarget.GetComponent<ColorOrganiser>().UpdateColour(Color.red);
    }   

    public void TrackTarget()
    {
        //Update current target position using controller position and update UI
        currentTarget.transform.position = controllerObject.transform.position;
        UnityEngine.Vector3 eulerAngles = controllerObject.transform.rotation.eulerAngles;
        currentTarget.transform.rotation = UnityEngine.Quaternion.Euler(0, eulerAngles.y, 0);
        currentTargetPos = currentTarget.transform.position;
        //targetListText.UpdateString(targetList);
    }

    public void DeleteLastTarget()
    {
        //Delete current target and remove from UI
        if(targetList.Count>0)
        {
            currentTarget = targetList[targetList.Count - 1];
            targetList.RemoveAt(targetList.Count - 1);
            Destroy(currentTarget);
        }
        targetListText.UpdateString(targetList);
    }

    public void UpdateMessage()
    {
        //Formulate ROS message based upon target gameObjects and publish
        if (targetList.Count > 0) 
        {
            List<UnityEngine.Vector3> posePositions = new List<UnityEngine.Vector3>();
            List<UnityEngine.Quaternion> poseQuaternions = new List<UnityEngine.Quaternion>();

            for (int i = 0; i < targetList.Count; i++)
            {
                posePositions.Add(targetList[i].transform.position);
                poseQuaternions.Add(targetList[i].transform.rotation);
                targetList[i].GetComponent<ColorOrganiser>().UpdateColour(Color.green);
                Debug.Log(targetList[i].transform.position.Unity2Ros());
            }
    
            mavTrajectoryPublisher.UpdateMessage(posePositions.ToArray(), poseQuaternions.ToArray(), mav_id, cam_id);
        }
    }
}
