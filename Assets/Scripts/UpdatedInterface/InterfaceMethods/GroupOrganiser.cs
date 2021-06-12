using RosSharp.RosBridgeClient;
using System.Collections;
using System.Collections.Generic;
using System.Linq;
using UnityEngine;

public class Group
{
    public int ID;
    public GameObject VRBGameObject;
    public int robotNumber;
    public List<int> robotID;
    public List<GameObject> robotMarkers;
    public List<GameObject> robotGameObjects;
    public List<GameObject> robotVRBPosition;
    public Color32 groupColor;
    public LineRenderer line;

    public Group()
    {
        ID = 0;
        VRBGameObject = null;
        robotNumber = 0;
        robotID = new List<int>();
        robotMarkers = new List<GameObject>();
        robotGameObjects = new List<GameObject>();
        robotVRBPosition = new List<GameObject>();
        groupColor = new Color32();
    }
}

public class GroupDict
{
    public int ID;
    public GameObject VRBGameObject;
    public int robotNumber;
    public Dictionary<int, int> robotID;
    public Dictionary<int, GameObject> robotGameObjects;
    public Color32 groupColor;
    public LineRenderer line;

    public GroupDict()
    {
        ID = 0;
        VRBGameObject = null;
        robotNumber = 0;
        robotID = new Dictionary<int, int>();
        robotGameObjects = new Dictionary<int, GameObject>();
        groupColor = new Color32();
    }
}

public class GroupOrganiser : MonoBehaviour
{
    [Header("RosSharp")]
    public VRBGroupsPublisher VRBGroupsPublisher;

    [Header("Scene")]
    public GameObject cameraObject;
    public GameObject controllerObject;
    public RobotOrganisor robotOrganisor;

    [Header("Group UI")]
    public GameObject VRBMarker;
    public GameObject robotMarker;
    public GameObject robotVRBPosition;
    public Material lineMaterial;
    public GameObject highlightMarker;

    [Header("Input UI")]
    public GameObject twistRotationArrow;
    public GameObject twistDirectionArrow;
    public GameObject trajectoryArrow;
    public Material uiLineMaterial;

    private GameObject activeHighlight;
    private System.Random rnd = new System.Random();
    private int counter = 1;
    private List<Group> groups;
    private Dictionary<int, GroupDict> groupsDict;
    private List<int> activeGroups;
    private Group activeGroup;
    private GroupDict activeGroupDict;
    private int highlightedGroupID;
    private int highlightedRobotID;
    private VRBTrajectoryControl selectedTrajectoryControl;

    [Header("Debbuging")]
    //Debugging (find better way to do this)
    public MenuOrganiser menuOrganiser;

    // Start is called before the first frame update
    void Start()
    {
        groups = new List<Group>();
        groupsDict = new Dictionary<int, GroupDict>();
        activeGroups = new List<int>();     
    }

    // Update is called once per frame
    void Update()
    {
        UpdateLines();
        PublishGroups();
    }

    public void SetActiveGroup(int ID)
    {
        Debug.Log(string.Format("ActiveGroup = {0}", ID));
        activeGroup = groups.Find(x => x.ID == ID);
        selectedTrajectoryControl = activeGroup.VRBGameObject.GetComponent<VRBTrajectoryControl>();
        TrajectorySetup(); 
    }

    public void CreateGroupAtControllerPosition()
    {
        CreateNewGroup(controllerObject.transform.position);
    }

    public void UpdateHighlightedGroup(int groupID) 
    {
        highlightedGroupID = groupID;
    }

    public void UpdateHighlightedRobot(int robotID)
    {
        highlightedRobotID = robotID;
    }

    public void CreateNewGroup(Vector3 position)
    {
        //Creating New Group and group variables
        Group newGroup = new Group();
        newGroup.ID = counter;
        newGroup.VRBGameObject = Instantiate(VRBMarker, position, Quaternion.identity);
        newGroup.line = newGroup.VRBGameObject.AddComponent<LineRenderer>();
        newGroup.line.material = lineMaterial;
        newGroup.line.startWidth = .0f;
        newGroup.line.endWidth = .0f;

        newGroup.groupColor = new Color32((byte)rnd.Next(256), (byte)rnd.Next(256), (byte)rnd.Next(256), 255);

        newGroup.VRBGameObject.GetComponent<ColorOrganiser>().UpdateColour(newGroup.groupColor);
        newGroup.line.material.color = newGroup.groupColor;

        groups.Add(newGroup);
        activeGroup = newGroup;
        activeGroups.Add(counter);
        counter++;

        //DEBUG FOR MENU - REMOVE
        menuOrganiser.CreateGroupNode(newGroup.ID, newGroup.groupColor, newGroup.VRBGameObject);
    }

    public void AddRobotToGroup(int groupID, int robotID)
    {
        Debug.Log(robotID);
        GameObject robotObject = robotOrganisor.robotList.Find(x => x.GetComponent<MavrosInfo>().mavID == robotID);

        //If robot already part of group - return
        MavrosInfo robotInfo = robotObject.GetComponent<MavrosInfo>();
        if (robotInfo.groupID == groupID)
            return;

        //Remove robot from current group
        SetActiveGroup(groupID);
        if(robotInfo.groupID != 0)
            RemoveRobotFromGroup(robotID);
        

        //Add Robot to new group
        activeGroup.robotID.Add(robotInfo.mavID);
        activeGroup.robotGameObjects.Add(robotObject);

        //Add Marker and VRBPosition to Group
        GameObject markerObject = Instantiate(robotMarker, robotObject.transform);
        markerObject.transform.SetParent(robotObject.transform, true);
        markerObject.GetComponent<ColorOrganiser>().UpdateColour(activeGroup.groupColor);
        activeGroup.robotMarkers.Add(markerObject);

        GameObject VRBPositionObject = Instantiate(robotVRBPosition, robotObject.transform);
        VRBPositionObject.transform.SetParent(activeGroup.VRBGameObject.transform, true);
        activeGroup.robotVRBPosition.Add(VRBPositionObject);

        robotInfo.groupID = groupID;

        //Add robot number
        activeGroup.robotNumber++;

        //DEBUG FOR MENU - REMOVE
        menuOrganiser.AddRobotToGroup(groupID, robotInfo.mavID);
    }

    public void UpdateRobotVRBPosition(int robotID)
    {
        GameObject robotObject = robotOrganisor.robotList.Find(x => x.GetComponent<MavrosInfo>().mavID == robotID);
        MavrosInfo robotInfo = robotObject.GetComponent<MavrosInfo>();
        if (robotInfo.groupID == 0)
            return;

        Group tempGroup = groups.Find(x => x.ID == robotInfo.groupID);
        for (int i = 0; i < tempGroup.robotID.Count; i++)
        {
            if (tempGroup.robotID[i] == robotID)
            {
                tempGroup.robotVRBPosition[i].transform.position = tempGroup.robotGameObjects[i].transform.position;
            }
        }
    }

    public void RemoveRobotFromGroup(int robotID)
    {
        
        GameObject robotObject = robotOrganisor.robotList.Find(x => x.GetComponent<MavrosInfo>().mavID == robotID);
        MavrosInfo robotInfo = robotObject.GetComponent<MavrosInfo>();

        //If robot has no group - return
        if (robotInfo.groupID == 0)
            return;

        Group tempGroup = groups.Find(x => x.ID == robotInfo.groupID);

        //Set groupID to 0 and decrease robotNumber
        tempGroup.robotNumber--;

        //DEBUG FOR MENUORGANISER - REMOVE
        menuOrganiser.RemoveRobotFromGroup(robotInfo.groupID, robotInfo.mavID);

        //Remove robot from group data and destroy gameObjects
        for (int i=0; i<tempGroup.robotID.Count;i++)
        {
            if (tempGroup.robotID[i] == robotInfo.mavID)
            {
                tempGroup.robotGameObjects.RemoveAt(i); 
                tempGroup.robotID.RemoveAt(i);
                GameObject deletionObject = tempGroup.robotMarkers[i];
                tempGroup.robotMarkers.RemoveAt(i);
                Destroy(deletionObject);

                deletionObject = tempGroup.robotVRBPosition[i];
                tempGroup.robotVRBPosition.RemoveAt(i);
                Destroy(deletionObject);
            }
        }

        robotInfo.groupID = 0;
    }

    public void UpdateLines()
    {
        for(int i=0; i< groups.Count; i++)
        {
            if(groups[i].robotGameObjects.Count < 1)
            {
                groups[i].line.positionCount = 0;
                return;
            }

            groups[i].line.positionCount = groups[i].robotGameObjects.Count + 1;

            for (int j = 0; j < groups[i].robotGameObjects.Count; j++)
            {
                groups[i].line.SetPosition(j, groups[i].robotGameObjects[j].GetComponent<Transform>().position);
                groups[i].line.startWidth = .05f;
                groups[i].line.endWidth = .05f;
            }

            groups[i].line.SetPosition(groups[i].robotNumber, groups[i].robotGameObjects[0].GetComponent<Transform>().position);
            groups[i].line.startWidth = .05f;
            groups[i].line.endWidth = .05f;

        }
    }

    public void RemoveGroup(int groupID)
    {
        if (groupID == 0)
            return;

        //Obtain group
        Group tempGroup = groups.Find(x => x.ID == groupID);

        if (tempGroup == null)
            return;

        //Remove VRMobject
        Destroy(tempGroup.VRBGameObject);

        //Loop through active robots and remove gameobjects
        Debug.Log(tempGroup.robotID.Count);
        int[] robotsToRemove = tempGroup.robotID.ToArray();
        for (int i = 0; i < robotsToRemove.Count(); i++)
        {
            RemoveRobotFromGroup(robotsToRemove[i]);
            Debug.Log(string.Format("Removed robot {0} from group {1}", robotsToRemove[i], groupID));
        }

        menuOrganiser.RemoveGroup(groupID);

        groups.Remove(tempGroup);
    }

    public void PublishGroups()
    {
        if(groups.Count > 0)
            for(int i=0; i<groups.Count; i++)
            {
                for (int j = 0; j < groups[i].robotVRBPosition.Count; j++)
                {
                    PoseStampedPublisher posePublisher = groups[i].robotGameObjects[j].GetComponent<PoseStampedPublisher>();
                    posePublisher.PublishedTransform = groups[i].robotVRBPosition[j].transform;
                    posePublisher.UpdateMessage();
                }
            }
    }

    public void RemoveHighLightedGroup()
    {
        RemoveGroup(highlightedGroupID);
    }

    public void AddHighlightedRobotToGroup()
    {
        AddRobotToGroup(activeGroup.ID, highlightedRobotID);
    }

    public void RemoveHighlightedRobotFromGroup()
    {
        RemoveRobotFromGroup(highlightedRobotID);
    }

    //METHODS FOR TRAJECTORY CONTROL
    void TrajectorySetup()
    {
        selectedTrajectoryControl.SetupGameObjects(activeGroup.VRBGameObject, controllerObject);
        selectedTrajectoryControl.SetupUI(trajectoryArrow);
    }

    public void TrajectoryInitialiseTarget()
    {
        selectedTrajectoryControl.InitialiseTarget();
    }

    public void TrajectoryTrackTarget()
    {
        selectedTrajectoryControl.TrackTarget();
    }

    public void TrajectoryDeleteTarget()
    {
        selectedTrajectoryControl.DeleteTarget();
    }

    public void TrajectoryUpdateMessage()
    {
        selectedTrajectoryControl.UpdatePosition();
    }
}
