using System.Collections;
using System.Collections.Generic;
using UnityEngine;

[RequireComponent(typeof(RosSharp.RosBridgeClient.PosePublisher))]
public class MouseWaypoint : MonoBehaviour
{
    public GameObject marker;
    public Camera camera;
    private bool updatePos = true;
    private RosSharp.RosBridgeClient.PosePublisher posePublisher;
    // Start is called before the first frame update
    void Start()
    {
        posePublisher = GetComponent<RosSharp.RosBridgeClient.PosePublisher>();
        marker.SetActive(false);
    }

    // Update is called once per frame
    void Update()
    {
        var plane = new Plane(Vector3.up, transform.position);
        var ray = camera.ScreenPointToRay(Input.mousePosition);
        float distance;
        if (plane.Raycast(ray, out distance))
        {
            // some point of the plane was hit - get its coordinates
            var hitPoint = new Vector3();

            if (Input.GetMouseButton(0))
            {
                marker.SetActive(true);
                //Update marker position on first frame of mouse down
                if (updatePos == true)
                {
                    marker.transform.position = ray.GetPoint(distance);
                    updatePos = false;
                    marker.GetComponent<ColorOrganiser>().UpdateColour(Color.red);
                }

                //Update marker rotation on all frames
                marker.transform.LookAt(ray.GetPoint(distance));
            }
            else
                updatePos = true;

            //Publish waypoint on space down
            if (Input.GetKeyUp(KeyCode.Space))
            {
                Debug.Log("Publish");
                posePublisher.UpdateMessage(marker);
                marker.GetComponent<ColorOrganiser>().UpdateColour(Color.green);
            }
        }
    }
}
