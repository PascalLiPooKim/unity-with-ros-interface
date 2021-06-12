using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using UnityEngine.Rendering;

public class MapOrganiser : MonoBehaviour
{
    [Header("Settings")]
    public int voxelX;
    public int voxelZ;
    public int mappingBound;
    public bool startMap;

    [Header("Scene")]
    public GameObject mapTile;

    private Dictionary<string, GameObject> tileObjects;
    private Texture2D cameraTexture;
    private Vector3 dronePos;

    // Start is called before the first frame update
    void Start()
    {
        cameraTexture = new Texture2D(320,240);
        tileObjects = new Dictionary<string, GameObject>();
    }

    public void UpdateTile(Transform gameObjectTransform, byte[] imageData)
    {
        Vector3 robotPosition = gameObjectTransform.position;
        if (startMap)
        {
            if(CheckPosition())
            {
                cameraTexture = new Texture2D(1, 1);
                cameraTexture.LoadImage(imageData);
                GameObject tileObject;
                if (!tileObjects.TryGetValue(string.Format("x:{0},y:{1}", ((int)Mathf.Round(robotPosition.x / (float)voxelX)) * voxelX, ((int)Mathf.Round(robotPosition.z / (float)voxelZ)) * voxelZ), out tileObject))
                {
                    tileObject = Instantiate(mapTile, new Vector3(((int)Mathf.Round(robotPosition.x / (float)voxelX)) * voxelX, -1.0f, ((int)Mathf.Round(robotPosition.z / (float)voxelZ)) * voxelZ), Quaternion.Euler(0, 180, 0));
                    tileObject.transform.localScale = new Vector3(voxelX / 10, 0.1f, voxelZ / 10);
                    tileObjects.Add(string.Format("x:{0},y:{1}", ((int)Mathf.Round(robotPosition.x / (float)voxelX)) * voxelX, ((int)Mathf.Round(robotPosition.z / (float)voxelZ)) * voxelZ), tileObject);
                }
                tileObject.GetComponent<Renderer>().material.mainTexture = cameraTexture;
            }
        }

        bool CheckPosition()
        {
            int nearestVoxelx = ((int)Mathf.Round(robotPosition.x / (float)voxelX)) *voxelX;
            int nearestVoxelz = ((int)Mathf.Round(robotPosition.z / (float)voxelZ)) *voxelZ;
            if (Mathf.Abs(robotPosition.x - nearestVoxelx) < mappingBound && Mathf.Abs(robotPosition.z - nearestVoxelz) < mappingBound)
                return true;
            return false;
        }
    }
}
