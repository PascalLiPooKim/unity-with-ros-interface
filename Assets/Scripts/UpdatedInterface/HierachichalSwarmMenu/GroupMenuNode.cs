using System.Collections;
using System.Collections.Generic;
using UnityEngine;
public class GroupMenuNode : MenuNode
{
    public int groupID;
    public intEvent groupCallback;
    public GroupOrganiser groupOrganiser;

    public override void NodeSetup()
    {
        groupOrganiser = GameObject.Find("GroupOrganiser").GetComponent<GroupOrganiser>();
        groupCallback.AddListener(groupOrganiser.SetActiveGroup);
    }

    public override void ButtonCallback()
    {
        Debug.Log("GroupPress!");
        buttonCallback.Invoke(this);
        groupCallback.Invoke(groupID);
    }
}
