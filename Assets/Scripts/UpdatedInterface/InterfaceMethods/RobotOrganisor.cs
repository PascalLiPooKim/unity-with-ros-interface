using System.Collections;
using System.Collections.Generic;
using UnityEditor;
using UnityEngine;

public class RobotOrganisor : MonoBehaviour
{
    [Header("Scene")]
    [SerializeField] public List<GameObject> robotList;
    public GameObject cameraObject;
    public GameObject controllerObject;

    [Header("UI")]
    public GameObject twistRotationArrow;
    public GameObject twistDirectionArrow;
    public GameObject trajectoryArrow;

    private GameObject selectedRotationArrow;
    private GameObject selectedDirectionArrow;
    private GameObject selectedRobot;

    private TwistControl selectedTwistControl;
    private TrajectoryControl selectedTrajectoryControl;

    private List<GameObject> robotUI =  new List<GameObject>();

    //Grouping demo stuff
    public Material LineMaterial;
    private LineRenderer line;

    public void Update()
    {
    }

    public void Start()
    {
    }

    public void UpdateSelectedRobot(int index)
    {
        //Removing previous generated UI
        if(robotUI.Count!=0)
        {
            for(int i=0; i< robotUI.Count; i++)
            {
                Destroy(robotUI[i]);
            }
        }

        //Updating selected robot and snapping user to position
        selectedRobot = robotList[index];
        //cameraObject.transform.SetParent(selectedRobot.transform, false);

        //Generating new UI objects
        selectedRotationArrow = Instantiate(twistRotationArrow, new Vector3(0,0,0), Quaternion.identity);
        selectedRotationArrow.transform.SetParent(selectedRobot.transform, false);
        robotUI.Add(selectedRotationArrow);

        selectedDirectionArrow = Instantiate(twistDirectionArrow, new Vector3(0, 0, 0), Quaternion.identity);
        selectedDirectionArrow.transform.SetParent(selectedRobot.transform, false);
        robotUI.Add(selectedDirectionArrow);


        //Obtaining robots trajectory and twist publisherS
        selectedTwistControl = null;
        selectedTwistControl = selectedRobot.GetComponent<TwistControl>();
        selectedTrajectoryControl = null;
        selectedTrajectoryControl = selectedRobot.GetComponent<TrajectoryControl>();

        //Setting Up Twist and Trajectory Controllers
        TwistSetup();
        TrajectorySetup();
    }


    //TRAJECTORY METHODS - Links up to trajectory controls
    void TrajectorySetup()
    {
        selectedTrajectoryControl.SetupGameObjects(selectedRobot, controllerObject);
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
        selectedTrajectoryControl.DeleteLastTarget();
    }

    public void TrajectoryUpdateMessage()
    {
        selectedTrajectoryControl.UpdateMessage();
    }

    //TWIST METHODS - links up to twist controls
    void TwistSetup()
    {
        selectedTwistControl.SetupGameObjects(selectedRobot, controllerObject);
        selectedTwistControl.SetupUI(selectedDirectionArrow, selectedRotationArrow);
    }

    public void TwistLockRotation()
    {
        selectedTwistControl.LockRotation();
    }

    public void TwistLockPosition()
    {
        selectedTwistControl.LockPosition();
    }

    public void TwistBeginTracking()
    {
        selectedTwistControl.BeginTracking();
    }

    public void TwistEndTracking()
    {
        selectedTwistControl.EndTracking();
    }
}
