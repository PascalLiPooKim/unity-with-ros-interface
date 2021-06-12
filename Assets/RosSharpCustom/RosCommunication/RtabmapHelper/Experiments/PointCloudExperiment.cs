using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using System.IO;

public class PointCloudExperiment : MonoBehaviour
{
    private int frameCounter = 0;
    private int pointCounter = 0;
    private int avgFps = 0;
    

    public string fileName;
    private Pcx.PointCloudRenderer pointCloudRenderer;
    private Pcx.PointCloudData pointCloudData;

    public float waitTime;
    public float timer = 0.0f;
    public int pointCloudSize;
    
    private List<int> meanFPS = new List<int>();
    private List<int> points = new List<int>();
    private List<Vector3> positions = new List<Vector3>();
    private List<Color32> colors = new List<Color32>();

    private System.Random rnd;

    // Start is called before the first frame update
    void Start()
    {
        pointCloudRenderer = GetComponent<Pcx.PointCloudRenderer>();
        pointCloudData = ScriptableObject.CreateInstance<Pcx.PointCloudData>();
        rnd = new System.Random();
    }

    // Update is called once per frame
    void Update()
    {
        timer += Time.deltaTime;
        if (timer > waitTime)
        {
            avgFps = avgFps / frameCounter;
            Debug.Log("FPS: " + avgFps + "    Points: " + pointCounter);

            meanFPS.Add(avgFps);
            points.Add(pointCounter);

            frameCounter = 0;
            avgFps = 0;
            pointCounter += pointCloudSize;

            for (int i = 0; i < pointCloudSize; i++)
            {
                Vector3 pos = new Vector3((float)rnd.NextDouble()*10.0f, (float)rnd.NextDouble() * 10.0f, (float)rnd.NextDouble() * 10.0f); ;
                var bytes = new byte[3];
                rnd.NextBytes(bytes);
                positions.Add(pos);
                colors.Add(new Color32(bytes[0], bytes[1], bytes[2], (byte)200));
            }

            UnityEngine.Object.Destroy(pointCloudData);
            pointCloudData = ScriptableObject.CreateInstance<Pcx.PointCloudData>();
            pointCloudData.Initialize(positions, colors);
            //pointCloudRenderer.sourceData = null;
            pointCloudRenderer.sourceData = pointCloudData;
            timer -= waitTime;
        }

        avgFps += (int)(1f / Time.unscaledDeltaTime);
        frameCounter += 1;
    }

    private void OnApplicationQuit()
    {
        Debug.Log(Application.dataPath + "/Data/" + fileName + ".csv");

        string filePath = Application.dataPath + "/Data/" + fileName + ".csv";
        StreamWriter writer = new StreamWriter(filePath);
        writer.WriteLine("points,FPS");

        for (int i = 0; i < meanFPS.Count; i++)
        {
            writer.WriteLine(meanFPS[i] + "," + points[i]);
        }

        writer.Flush();
        writer.Close();

        Debug.Log(meanFPS.ToString());
        Debug.Log(points.ToString());
    }
}
