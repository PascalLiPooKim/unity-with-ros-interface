using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using UnityEngine.Events;

[System.Serializable]
public class intEvent : UnityEvent<int> { }
public class RobotMenuNode : MenuNode
{
    public int robotID;
    public new intEvent robotCallback;
    public RobotOrganisor robotOrganiser;

    public override void NodeSetup()
    {
        robotOrganiser = GameObject.Find("RobotOrganiser").GetComponent<RobotOrganisor>();
        robotCallback.AddListener(robotOrganiser.UpdateSelectedRobot);
    }

    public override void ButtonCallback()
    {
        buttonCallback.Invoke(this);
        robotCallback.Invoke(robotID);
        Debug.Log("RobotPress!");
    }
}
