using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class GroupTest : MonoBehaviour
{
    [Header("Scene")]
    public List<GameObject> robotList;
    public int selectedRobot;
    public Vector3 VRMposition;
    public GroupOrganiser groupOrganiser;
    public MenuOrganiser menuOrganiser;

    [Header("Test Settings")]
    public bool createGroup;
    public bool addRobot;
    public bool removeRobot;
    public bool removeGroup;
    public int groupID;
    // Start is called before the first frame update
    void Start()
    {
        
    }

    // Update is called once per frame
    void Update()
    {
        if (createGroup)
        {
            groupOrganiser.CreateNewGroup(VRMposition);
            createGroup = false;
        }
        if (addRobot)
        {
            groupOrganiser.AddRobotToGroup(groupID, selectedRobot);
            addRobot = false;
        }
        if (removeRobot)
        {
            groupOrganiser.RemoveRobotFromGroup(selectedRobot);
            removeRobot = false;
        }
        if(removeGroup)
        {
            groupOrganiser.RemoveGroup(groupID);
            removeGroup = false;
        }
    }
}
