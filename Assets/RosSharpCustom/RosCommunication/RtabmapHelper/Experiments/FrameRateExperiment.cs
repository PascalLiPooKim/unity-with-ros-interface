using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using System.Text;
using System.IO;
using System;
public class FrameRateExperiment : MonoBehaviour
{
    private int frameCounter = 0;
    private int pointCounter = 0;
    private int avgFps = 0;
    private RosSharp.RosBridgeClient.PointCloudMapDataSubscriber pointCloudSubscriber;
    public string fileName;
    public bool saveData = false;

    private List<int> meanFPS = new List<int>();
    private List<int> points = new List<int>();
    void Start()
    {
        pointCloudSubscriber = GetComponent<RosSharp.RosBridgeClient.PointCloudMapDataSubscriber>();
    }

    void Update()
    {
        if (pointCloudSubscriber.totalPoints != pointCounter)
        {
            avgFps = avgFps / frameCounter;

            Debug.Log("FPS: " + avgFps + "    Points: " + pointCounter);

            meanFPS.Add(avgFps);
            points.Add(pointCounter);

            frameCounter = 0;
            avgFps = 0;
            pointCounter = pointCloudSubscriber.totalPoints;
        }

        avgFps += (int)(1f / Time.unscaledDeltaTime);
        frameCounter += 1;
    }

    private void OnApplicationQuit()
    {
        if (saveData)
        {
            Debug.Log(Application.dataPath + "/Data/" + fileName + ".csv");

            string filePath = Application.dataPath + "/Data/" + fileName + ".csv";
            StreamWriter writer = new StreamWriter(filePath);
            writer.WriteLine("points,FPS");

            for (int i = 0; i < meanFPS.Count; i++)
            {
                writer.WriteLine(points[i] + "," + meanFPS[i]);
            }

            writer.Flush();
            writer.Close();
        }
    }
}
