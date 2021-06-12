using System.Collections;
using System.Collections.Generic;
using System.Security.Policy;
using System.IO;
using UnityEngine;
using System;

[RequireComponent(typeof(RosSharp.RosBridgeClient.PointPublisher))]
[RequireComponent(typeof(RtspConnector))]
public class LatencyExperiment : MonoBehaviour
{
    private Color32 frameColour;
    private Color32 rtspFrameColour;
    private Vector3 colourMessage;
    private Texture2D texture;
    public Renderer renderer;
    private System.Diagnostics.Stopwatch sw = new System.Diagnostics.Stopwatch();
    private System.Diagnostics.Stopwatch bufferSW = new System.Diagnostics.Stopwatch();
    private RosSharp.RosBridgeClient.PointPublisher pointPublisher;
    private RtspConnector rtspConnector;
    public int iterations;
    public int i = 0;
    public bool shouldUpdateFrame;
    public bool shouldSaveData = false;
    public string fileName;
    private List<double> latency = new List<double>();
    private System.Random rand;

    // Start is called before the first frame update
    void Start()
    {
        rand = new System.Random();
        pointPublisher = GetComponent<RosSharp.RosBridgeClient.PointPublisher>();
        texture = (Texture2D)renderer.material.mainTexture;
        rtspConnector = GetComponent<RtspConnector>();
    }

    // Update is called once per frame
    void Update()
    {
        if (bufferSW.IsRunning && bufferSW.ElapsedMilliseconds > 500)
        {
            bufferSW.Stop();
            shouldUpdateFrame = true;
        }

        //If shouldUpdateFrame, create new colour and publish over ROS
        if (shouldUpdateFrame && i < iterations)
        {
            shouldUpdateFrame = false;

            sw.Stop();
            sw.Reset();
            PublishColour();
            sw.Start();
        }

        //Check texture colour, if same as frameColour stop timer and save to rtspFrameColour to lock until
        //next colour update
        if (IsSameColour() && rtspFrameColour.r != frameColour.r
            && rtspFrameColour.g != frameColour.g
            && rtspFrameColour.b != frameColour.b)
        {
            sw.Stop();
            latency.Add(sw.Elapsed.TotalMilliseconds);
            Debug.Log(sw.Elapsed.TotalMilliseconds);
            rtspFrameColour = frameColour;
            bufferSW.Reset();
            bufferSW.Start();
            i += 1;
        }
    }

    void PublishColour()
    {
        frameColour = UpdateColor(frameColour);
        colourMessage = ColorToVector(frameColour);
        pointPublisher.UpdateMessage(colourMessage);
    }

    Color32 UpdateColor(Color32 colour)
    {
        Color32 newColour = new Color32();
        newColour = colour;
        while(Math.Pow((Math.Sqrt((int)newColour.r - (int)colour.r)), 2) < 50 
            && Math.Pow((Math.Sqrt((int)newColour.b - (int)colour.b)), 2) < 50
            && Math.Pow((Math.Sqrt((int)newColour.g - (int)colour.g)), 2) < 50)
        {
            newColour.r = (byte)rand.Next(0, 255);
            newColour.g = (byte)rand.Next(0, 255);
            newColour.b = (byte)rand.Next(0, 255);
        }
        newColour.a = 255;

        return newColour;
    }

    Vector3 ColorToVector(Color32 colour)
    {
        Vector3 vector = new Vector3();
        vector.x = (int)colour.r;
        vector.y = (int)colour.b;
        vector.z = (int)colour.g;
        return vector;
    }

    bool IsSameColour()
    {
        bool isSameColour = false;
        texture = (Texture2D)renderer.material.mainTexture;
        if (texture == null)
        {
            return isSameColour;
        }
        var pixels = texture.GetPixels32();
        //Debug.Log("R: " + pixels[0].r + ";" + frameColour.r);
        //Debug.Log("G: " + pixels[0].g + ";" + frameColour.g);
        //Debug.Log("B: " + pixels[0].b + ";" + frameColour.b);
        //Debug.Log("Rfixed: " + pixels[0].b + ";" + frameColour.r);
        //Debug.Log("Gfixed: " + pixels[0].r + ";" + frameColour.g);
        //Debug.Log("Bfixed: " + pixels[0].g + ";" + frameColour.b);
        if ((int)pixels[0].b > (int)frameColour.r - 50 && (int)pixels[0].b < (int)frameColour.r + 50
            && (int)pixels[0].r > (int)frameColour.g - 50 && (int)pixels[0].r < (int)frameColour.g + 50
            && (int)pixels[0].g > (int)frameColour.b - 50 && (int)pixels[0].g < (int)frameColour.b + 50)
        {
            //Debug.Log("!!!!SAME!!!!");
            isSameColour = true;
        }

        return isSameColour;
    }
    private void OnApplicationQuit()
    {
        if (shouldSaveData)
        {
            Debug.Log(Application.dataPath + "/Data/" + fileName + ".csv");

            string filePath = Application.dataPath + "/Data/" + fileName + ".csv";
            StreamWriter writer = new StreamWriter(filePath);
            writer.WriteLine("Delay (ms)");

            for (int i = 0; i < latency.Count; i++)
            {
                writer.WriteLine(latency[i]);
            }

            writer.Flush();
            writer.Close();

            filePath = Application.dataPath + "/Data/" + fileName + "decode.csv";
            StreamWriter writer2 = new StreamWriter(filePath);
            writer2.WriteLine("Time (ms)");

            for (int i = 0; i < rtspConnector.frameDecodeTime.Count; i++)
            {
                writer2.WriteLine(rtspConnector.frameDecodeTime[i]);
            }

            writer2.Flush();
            writer2.Close();
        }
    }
}
