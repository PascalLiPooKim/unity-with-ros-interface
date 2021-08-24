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
    
    public bool NASA_TLX = true;

    
    // Start is called before the first frame update
    void Start()
    {
        
    }

    // Update is called once per frame
    void Update()
    {
        
        // Stop updating timer and keystrokes counter if goal has been reached
        if (targetReached)
		{
            return;
		}

        // https://www.youtube.com/watch?v=x-C95TuQtf0
        // Start timer and keystrokes counter when "Enter" key is pressed
        if (Input.GetKeyDown(KeyCode.Return))
		{
            InitTimerAndCounter();
        }
        

        UpdateTimerAndCounter();
        
    }

    // Initialise timer and keystrokes counter
    private void InitTimerAndCounter()
	{
        startTime = Time.time;
        keystrokesCount = 0;
    }

    // https://www.youtube.com/watch?v=x-C95TuQtf0
    // Increment timer and keystrokes counter, turn them to text and display them in Ynity canvas
    private void UpdateTimerAndCounter()
	{
        // Timer
        float duration = startTime >= 0 ? Time.time - startTime : 0;

        string minutes = ((int)duration / 60).ToString();
        string seconds = (duration % 60).ToString("f2");

        timerText.text = minutes + ':' + seconds;

        // Keystrokes counter
        if (Input.GetKeyDown("w") || Input.GetKeyDown("a") || Input.GetKeyDown("s") || Input.GetKeyDown("d"))
        {
            keystrokesCount += 1;
        }

        keystrokesCounterText.text = keystrokesCount.ToString();

        completionTimeText = minutes + ':' + seconds;
        numberOfKeyPressedText = keystrokesCount.ToString();

    }


    // Stop updating timer and keystrokes counter and save them in a .txt file depending on experiment
    public void Finish()
	{
        timerText.color = Color.green;
        keystrokesCounterText.color = Color.yellow;
        string path;
        if (NASA_TLX)
		{
            path = "./CandidatesDataWorkload.txt";
        }
		else
		{
            path = "./CandidatesDataDegradedVision.txt";

        }
        SaveData(completionTimeText, numberOfKeyPressedText, path);
        targetReached = true;
    }



    // https://www.youtube.com/watch?v=vDpww7HsdnM
    // Write data to .txt file
    private void SaveData(string completionTime, string numberOfKeystrokes, string filePath)
	{
		try
		{
            using (System.IO.StreamWriter file = new System.IO.StreamWriter(@filePath, true))
			{
                file.WriteLine("-----------------------------------------------------------------------------------------------");
                file.WriteLine(IDNumber.ToString() + ", " + age.ToString() + ", " + completionTime + ", " + numberOfKeystrokes);
			}
		}
        catch(Exception e)
		{
            throw new ApplicationException("Lol", e);
         
		}
	}


}
