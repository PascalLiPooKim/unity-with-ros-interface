using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using System.Text;
using System.IO;
using System;

//public class GoalBox : MonoBehaviour
//{
//    //public static Vector3 newPos;
//    // Start is called before the first frame update
//    void Start()
//    {
//        // Vector3 newPos = new Vector3(1.0f, 0.0f, 4.0f);
//        // gameObject.transform.TransformPoint(newPos);
//    }

//    // Update is called once per frame
//    void Update()
//    {

//    }

//    private void OnTriggerEnter(Collider other)
//	{
//        GameObject.Find("Husky_V2").SendMessage("Finish");
//	}
//}


namespace RosSharp.RosBridgeClient
{
    [RequireComponent(typeof(RosConnector))]

    public class GoalTextToSpeech : UnitySubscriber<MessageTypes.Std.Float32>
    {

        private bool isMessageReceived;
        private MessageTypes.Std.Float32 distToGoal;
        private float nextTTS = 0.0f;
        public float period = 1000.0f;
        int i = 0;

        protected override void Start()
        {
            base.Start();

            //StartCoroutine(WriteText)

        }

        void Update()
        {
            if (isMessageReceived)
            {
                TextToSpeech();
            }

            TextToSpeech();
            //print(Time.deltaTime);

        }


        protected override void ReceiveMessage(MessageTypes.Std.Float32 huskyToGoalDist)
        {
            isMessageReceived = true;
            distToGoal = huskyToGoalDist;
        }


        private void TextToSpeech()
        {
            //nextTTS += Time.deltaTime;
            //if (nextTTS > [Time Interval])
            //{
            //nextTTS = nextTTS - period;
            // execute block of code here
            //WriteText("./DistanceToGoalTTS.txt");
            //period = 0;

            //}

            //period  += Time.deltaTime;

            nextTTS += Time.deltaTime;
            if (nextTTS/100.0f > period)
			{
                WriteText("./DistanceToGoalTTS.txt");
                nextTTS = 0;
                print(i);
                i += 1;
            }

            
        }


        private void WriteText(string filePath)
        {
            try
            {
                using (System.IO.StreamWriter file = new System.IO.StreamWriter(@filePath, true))
                {
                    float val = Mathf.Round(distToGoal.data * 10f) / 10f;
                    file.WriteLine("The goal is " + val.ToString() + " metre away.");
                }

                //yield return new WaitForSeconds(5f);
            }
            catch (Exception e)
            {
                throw new ApplicationException("Lol", e);
                //print("Fail");
            }
        }
    }


}
