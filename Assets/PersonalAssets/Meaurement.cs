using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using UnityEngine.UI;
using System.Text;
using System.IO;
using System;

public class Meaurement : MonoBehaviour
{
    public Text timerText;
    public Text keystrokesCounterText;
    public int IDNumber;
    public int age;
    private bool targetReached = false;


    private float startTime = -1.0f;
    private int keystrokesCount;

    private string completionTimeText;
    private string numberOfKeyPressedText;
    


    
    // Start is called before the first frame update
    void Start()
    {
        //InitTimerAndCounter();
        
    }

    // Update is called once per frame
    void Update()
    {
        //if (Input.GetKeyDown(KeyCode.F))
        //{
        //    targetReached = true;
        //    SaveData(completionTimeText, numberOfKeyPressedText, "./CandidatesData.txt");
        //}

        if (targetReached)
		{
            return;
		}

        // https://www.youtube.com/watch?v=x-C95TuQtf0
        if (Input.GetKeyDown(KeyCode.Return))
		{
            InitTimerAndCounter();
        }
        
        UpdateTimerAndCounter();
        
    }

    private void InitTimerAndCounter()
	{
        startTime = Time.time;
        keystrokesCount = 0;
    }

    // https://www.youtube.com/watch?v=x-C95TuQtf0
    private void UpdateTimerAndCounter()
	{
        //float duration = Time.time - startTime;
        float duration = startTime >= 0 ? Time.time - startTime : 0;

        string minutes = ((int)duration / 60).ToString();
        string seconds = (duration % 60).ToString("f2");

        timerText.text = minutes + ':' + seconds;

        if (Input.GetKeyDown("w") || Input.GetKeyDown("a") || Input.GetKeyDown("s") || Input.GetKeyDown("d"))
        {
            keystrokesCount += 1;
        }

        keystrokesCounterText.text = keystrokesCount.ToString();

        completionTimeText = minutes + ':' + seconds;
        numberOfKeyPressedText = keystrokesCount.ToString();

    }

    

    public void Finish()
	{
        timerText.color = Color.green;
        keystrokesCounterText.color = Color.yellow;
        SaveData(completionTimeText, numberOfKeyPressedText, "./CandidatesData.txt");
        targetReached = true;
    }

    //   private void SaveData()
    //{
    //       string filePath = getPath();
    //       //string filePath = "ParticipantsData.csv";
    //       StreamWriter dataWriter = new StreamWriter(filePath);
    //       dataWriter.WriteLine("Completion Time, Number of KeyStrokes, Workload Rating");
    //       dataWriter.WriteLine("Anime is life");
    //       dataWriter.Flush();
    //       dataWriter.Close();
    //}

    // https://www.youtube.com/watch?v=vDpww7HsdnM
    private void SaveData(string completionTime, string numberOfKeystrokes, string filePath)
	{
		try
		{
            using (System.IO.StreamWriter file = new System.IO.StreamWriter(@filePath, true))
			{
                file.WriteLine(" ");
                file.WriteLine(IDNumber.ToString() + ", " + age.ToString() + ", " + completionTime + ", " + numberOfKeystrokes);
			}
		}
        catch(Exception e)
		{
            throw new ApplicationException("Lol", e);
            //print("Fail");
		}
	}


//	private string getPath()
//	{
//#if UNITY_EDITOR
//        return Application.dataPath  + "/ParticipantsData.csv";
//#elif UNITY_ANDROID
//        return Application.persistentDataPath+ "ParticipantsData.csv";
//#elif UNITY_IPHONE
//        return Application.persistentDataPath+ "/" + "ParticipantsData.csv";
//#else
//        return Application.dataPath + "/ParticipantsData.csv";
//#endif
//	}


}
