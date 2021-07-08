using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using UnityEngine.UI;

public class Meaurement : MonoBehaviour
{
    public Text timerText;
    public Text keystrokesCounterText;
    private bool targetReached = false;


    private float startTime;
    private int keystrokesCount;
    // Start is called before the first frame update
    void Start()
    {
        InitTimerAndCounter();
    }

    // Update is called once per frame
    void Update()
    {
        if (targetReached)
		{
            return;
		}

        UpdateTimerAndCounter();
        
    }

    private void InitTimerAndCounter()
	{
        startTime = Time.time;
        keystrokesCount = 0;
    }

    private void UpdateTimerAndCounter()
	{
        float duration = Time.time - startTime;

        string minutes = ((int)duration / 60).ToString();
        string seconds = (duration % 60).ToString("f2");

        timerText.text = minutes + ':' + seconds;

        if (Input.GetKeyDown("w") || Input.GetKeyDown("a") || Input.GetKeyDown("s") || Input.GetKeyDown("d"))
        {
            keystrokesCount += 1;
        }

        keystrokesCounterText.text = keystrokesCount.ToString();
    }

    

    public void Finish()
	{
        timerText.color = Color.green;
        keystrokesCounterText.color = Color.yellow;
        targetReached = true;
    }
}
