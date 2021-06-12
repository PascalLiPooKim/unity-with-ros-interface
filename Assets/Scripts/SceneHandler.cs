using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using UnityEngine.EventSystems;
using UnityEngine.UI;
using Valve.VR.Extras;

public class SceneHandler : MonoBehaviour
{
    public SteamVR_LaserPointer laserPointer;
    public GroupOrganiser groupOrganiser;
    void Awake()
    {
        laserPointer.PointerIn += PointerInside;
        laserPointer.PointerOut += PointerOutside;
        laserPointer.PointerClick += PointerClick;
    }

    public void PointerClick(object sender, PointerEventArgs e)
    {
        //Debug.Log(string.Format("Clicked on {0}", e.target.name));
        if(e.target.GetComponent<MenuNode>() != null)
            e.target.GetComponent<MenuNode>().ButtonCallback();
    }

    public void PointerInside(object sender, PointerEventArgs e)
    {
        if (e.target.GetComponent<RobotMenuNode>() != null)
            groupOrganiser.UpdateHighlightedRobot(e.target.GetComponent<RobotMenuNode>().robotID);
        if (e.target.GetComponent<GroupMenuNode>() != null)
            groupOrganiser.UpdateHighlightedGroup(e.target.GetComponent<GroupMenuNode>().groupID);
        if (e.target.GetComponent<MavrosInfo>() != null)
            groupOrganiser.UpdateHighlightedRobot(e.target.GetComponent<MavrosInfo>().mavID);
    }

    public void PointerOutside(object sender, PointerEventArgs e)
    {
        //Debug.Log(string.Format("Exited on {0}", e.target.name));
    }
}
