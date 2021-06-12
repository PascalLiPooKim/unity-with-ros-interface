using System.Collections;
using System.Collections.Generic;
using System.Linq;
using UnityEngine;
using Valve.VR.InteractionSystem.Sample;

public class MenuOrganiser : MonoBehaviour
{
    [Header("Settings")]
    public bool worldDistrubuted;

    [Header("Scene")]
    public RobotOrganisor robotOrganisor;
    public GroupOrganiser groupOrganiser;
    public GameObject cameraObject;

    [Header("Prefabs")]
    public MenuNode menuNode;
    public RobotMenuNode robotMenuNode;
    public GroupMenuNode groupMenuNode;
    public GameObject highlightArrow;

    public GameObject[] sourceMenu;
    public GameObject[] groupMenu;
    public GameObject[] robotMenu;

    private bool[] sourceMenuDefault;
    private bool[] groupMenuDefault;
    private bool[] robotMenuDefault;

    private MenuNode sourceNode;
    private MenuNode activeNode;
    private GameObject[] activeNodeMenu;
    private float buf = 1.5f;

    private List<GroupMenuNode> groupNodes;
    private List<RobotMenuNode> robotNodes;
    private double[] robotNodePositions;

    // Start is called before the first frame update
    void Start()
    {
        //Initialising default values
        sourceMenuDefault = new bool[] { true };
        groupMenuDefault = new bool[] { true, true, false };
        robotMenuDefault = new bool[] { true, false, false};
        SetGameObjectsActive(sourceMenu, sourceMenuDefault);
        activeNodeMenu = sourceMenu;
        SetGameObjectsActive(groupMenu, new bool[groupMenu.Count()]);
        SetGameObjectsActive(robotMenu, new bool[groupMenu.Count()]);

        groupNodes = new List<GroupMenuNode>();
        robotNodes = new List<RobotMenuNode>();
        SetParentNode();
        InitialiseRobotNodes();

        SetActiveNode(sourceNode);
    }

    // Update is called once per frame
    void Update()
    {
        if (worldDistrubuted)
        {
            UpdateWorldDistribution(robotNodes);
            UpdateWorldDistribution(groupNodes);
        }
    }

    void SetParentNode()
    {
        sourceNode = Instantiate(menuNode, gameObject.transform);
        sourceNode.SetText("Swarm");
    }

    public void SetActiveNode(MenuNode nodeToBeActive)
    {
        //Deactivating previous node
        DeactivateNode();

        //Setting ActiveNode
        activeNode = nodeToBeActive;
        activeNode.isActive = true;

        if(activeNode == sourceNode)
        {
            //cameraObject.transform.parent = null;
            //cameraObject.transform.position = new Vector3(0, -10, -10);
            cameraObject.GetComponent<FollowGameobject>().ObjectTrack(false, false);
            cameraObject.GetComponent<FollowGameobject>().UpdateObject(null);
            cameraObject.GetComponent<FollowGameobject>().SetOffset(new Vector3(0, -10, -10));
        }
        if(activeNode.nodeObject != null)
        {
            //cameraObject.transform.SetParent(activeNode.nodeObject.transform, false);
            //cameraObject.transform.localPosition = new Vector3(0, -10, -10);
            cameraObject.GetComponent<FollowGameobject>().ObjectTrack(true, false);
            cameraObject.GetComponent<FollowGameobject>().UpdateObject(activeNode.nodeObject);
            cameraObject.GetComponent<FollowGameobject>().SetOffset(new Vector3(0, -10, -10));

        }

        //Menu Logic
        if (nodeToBeActive == sourceNode)
        {
            SetGameObjectsActive(sourceMenu, sourceMenuDefault);
            activeNodeMenu = sourceMenu;
        }
        else if (nodeToBeActive is RobotMenuNode)
        {
            SetGameObjectsActive(robotMenu, robotMenuDefault);
            activeNodeMenu = robotMenu;
        }
        else if (nodeToBeActive is GroupMenuNode)
        {
            SetGameObjectsActive(groupMenu, groupMenuDefault);
            activeNodeMenu = groupMenu;
        }
    }

    void DeactivateNode()
    {
        Debug.Log("DeactivateNode");
        if (activeNode != null)
        {
            activeNode.isActive = false;
            activeNode = null;
            SetGameObjectsActive(activeNodeMenu, new bool[activeNodeMenu.Count()]);
        }
 
    }

    void SetGameObjectsActive(GameObject[] gameObjects, bool[] setting)
    {
        for(int i = 0; i<gameObjects.Count(); i++)
        {
            gameObjects[i].SetActive(setting[i]);
        }
    }

    void InitialiseRobotNodes()
    {
        for (int i = 0; i < robotOrganisor.robotList.Count; i++)
        {
            RobotMenuNode robotNode = Instantiate(robotMenuNode, sourceNode.transform);
            int mavID = robotOrganisor.robotList[i].GetComponent<MavrosInfo>().mavID;
            robotNode.SetText(string.Format("{0}:{1}", robotOrganisor.robotList[i].name, mavID));
            robotNode.robotID = mavID;
            robotNode.parentObject = sourceNode;
            robotNode.nodeObject = robotOrganisor.robotList[i];
            robotNodes.Add(robotNode);
            sourceNode.attatchedObjects.Add(robotNode);
        }

        SetPositions(sourceNode.attatchedObjects, sourceNode);
    }

    public void CreateGroupNode(int groupID, Color32 color, GameObject groupObject)
    {
        GroupMenuNode groupNode = Instantiate(groupMenuNode, sourceNode.transform);
        groupNode.parentObject = sourceNode;
        groupNode.groupID = groupID;
        groupNode.SetText(string.Format("Group{0}", groupID));
        groupNode.SetColour(color);
        groupNode.nodeObject = groupObject;
        sourceNode.attatchedObjects.Add(groupNode);
        groupNodes.Add(groupNode);

        SetPositions(sourceNode.attatchedObjects, sourceNode);
    }

    public void RemoveGroup(int groupID)
    {
        GroupMenuNode groupNodeToRemove = groupNodes.Find(x => x.groupID == groupID);
        groupNodes.Remove(groupNodeToRemove);
        RemoveNodeFromParent(groupNodeToRemove);
        Destroy(groupNodeToRemove.gameObject);
        SetPositions(sourceNode.attatchedObjects, sourceNode);
    }

    public void AddRobotToGroup(int groupID, int robotID)
    {
        GroupMenuNode groupNode = groupNodes.Find(x => x.groupID == groupID);
        RobotMenuNode robotNode = robotNodes.Find(x => x.robotID == robotID);
        RemoveNodeFromParent(robotNode);
        SetNodeParent(robotNode, groupNode);
        SetPositions(groupNode.attatchedObjects, groupNode);
        SetPositions(sourceNode.attatchedObjects, sourceNode);
    }

    public void RemoveRobotFromGroup(int groupID, int robotID)
    {
        GroupMenuNode groupNode = groupNodes.Find(x => x.groupID == groupID);
        RobotMenuNode robotNodeToRemove = robotNodes.Find(x => x.robotID == robotID);
        groupNode.attatchedObjects.Remove(robotNodeToRemove);
        SetPositions(groupNode.attatchedObjects, groupNode);

        SetNodeParent(robotNodeToRemove, sourceNode);
        SetPositions(sourceNode.attatchedObjects, sourceNode);
    }

    void RemoveNodeFromParent(MenuNode nodeToRemove) 
    {
        nodeToRemove.parentObject.attatchedObjects.Remove(nodeToRemove);

    }

    void SetNodeParent(MenuNode childNode, MenuNode parentNode)
    {
        childNode.parentObject = parentNode;
        childNode.parentObject.attatchedObjects.Add(childNode);

        if (!worldDistrubuted)
            childNode.transform.SetParent(parentNode.transform);
    }

    static double[] LINSPACE(float StartValue, float EndValue, int numberofpoints)
    {

        double[] parameterVals = new double[numberofpoints];
        double increment = Mathf.Abs(StartValue - EndValue) / System.Convert.ToDouble(numberofpoints - 1);
        int j = 0; //will keep a track of the numbers 
        double nextValue = StartValue;
        for (int i = 0; i < numberofpoints; i++)
        {


            parameterVals.SetValue(nextValue, j);
            j++;
            if (j > numberofpoints)
            {
                throw new System.IndexOutOfRangeException();
            }
            nextValue = nextValue + increment;
        }
        return parameterVals;   
    }

    void SetPositions(List<MenuNode> connectedObjects, MenuNode parentNode)
    {
        if (worldDistrubuted)
        {
            for (int i = 0; i < connectedObjects.Count; i++)
            {
                connectedObjects[i].transform.parent = connectedObjects[i].nodeObject.transform;
                connectedObjects[i].transform.localPosition = new Vector3(0, 1f, 0);
                connectedObjects[i].transform.localScale = new Vector3(1f, 1f, 1f);
            }
            return;
        }

        //Calculating spaces for UI
        float num = (connectedObjects.Count - 1) * 0.5f;
        double[] spaces = LINSPACE(-num, num, connectedObjects.Count);
        float extraBuf = 0;
        int counter = 0;
        bool track = false;

        for (int i = 0; i < connectedObjects.Count; i++)
        {
            if (connectedObjects[i].attatchedObjects.Count > 0)
            {
                extraBuf += (float)connectedObjects[i].attatchedObjects.Count/1.5f;
                if (!track)
                {
                    counter = i - 1;
                    track = true;
                }
            }
            connectedObjects[i].transform.localPosition = new Vector3((i-counter)*extraBuf + ((float)spaces[i] + (float)spaces[i] * (buf)), -1.5f);
        }
    }

    void UpdateWorldDistribution(List<GroupMenuNode> menuNodes)
    {
        for (int i = 0; i < menuNodes.Count; i++)
        {
            float distance = Mathf.Max(1, Vector3.Distance(menuNodes[i].transform.position, cameraObject.transform.GetChild(2).transform.position)/10);
            menuNodes[i].transform.rotation = Quaternion.LookRotation(cameraObject.transform.GetChild(2).transform.forward, cameraObject.transform.GetChild(2).transform.up);
            menuNodes[i].transform.localScale = new Vector3(distance, distance, distance);
        }
    }

    void UpdateWorldDistribution(List<RobotMenuNode> menuNodes)
    {
        for (int i = 0; i < menuNodes.Count; i++)
        {
            float distance = Mathf.Max(1, Vector3.Distance(menuNodes[i].transform.position, cameraObject.transform.GetChild(2).transform.position)/10);
            if(menuNodes[i].parentObject != sourceNode)
                distance = 1;
            menuNodes[i].transform.rotation = Quaternion.LookRotation(cameraObject.transform.GetChild(2).transform.forward, cameraObject.transform.GetChild(2).transform.up);
            menuNodes[i].transform.localScale = new Vector3(distance, distance, distance);
        }
    }

    //MUCH BETTER METHOD THEORETICALLY BUT NOT IMPORTANT TO COMPLETE RIGHT NOW
    //void SetPositionInProgress(List<MenuNode> connectedObjects, MenuNode parentNode)
    //{
    //    int nodesToAssign = connectedObjects.Count;
    //    List<int> nodesWithChildren = new List<int>();
    //    List<int[]> nodesToSpaceArray = new List<int[]>();
    //    for (int i = 0; i < connectedObjects.Count; i++)
    //    {
    //        if (connectedObjects[i].attatchedObjects.Count > 1)
    //        {
    //            nodesToAssign += connectedObjects[i].attatchedObjects.Count - 1;
    //            nodesWithChildren.Add(i);
    //        }
    //    }
    //    Debug.Log(nodesToAssign);
    //    //Debug.Log(nodesWithChildren.ToString());
    //    float linStart = (nodesToAssign - 1) * 0.5f;
    //    double[] spaces = LINSPACE(-linStart, linStart, nodesToAssign);
    //    float xSpace;
    //    for (int i = 0; i < connectedObjects.Count; i++)
    //    {
    //        xSpace = (float)spaces[i] + (float)spaces[i] * (buf);
    //        if (nodesWithChildren.Contains(i))
    //        {
    //            Debug.Log("NodewithChild");
    //            //Debug.Log("Current i={0}, Connected Objects={1}, new i={2}, max i={3}", i, connectedObjects[i])
    //            xSpace = (float)spaces[i] + (float)spaces[i] * (buf) + (float)spaces[i + connectedObjects[i].attatchedObjects.Count - 1] + (float)spaces[i + connectedObjects[i].attatchedObjects.Count - 1] * (buf) / 2;
    //            i += connectedObjects[i].attatchedObjects.Count;
    //        }
    //        connectedObjects[i].transform.localPosition = new Vector3(xSpace, -1.5f);
    //    }
    //}
}
