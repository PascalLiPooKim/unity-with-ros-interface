using RosSharp.RosBridgeClient;
using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class RobotMappingOrganiser : MonoBehaviour
{
    public ImageSubscriber imageSubscriber;
    public MapOrganiser mapOrganiser;

    // Update is called once per frame
    void Update()
    {
        mapOrganiser.UpdateTile(gameObject.transform, imageSubscriber.imageData);
    }
}
