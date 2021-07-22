using System.Collections;
using System.Collections.Generic;
using UnityEngine;

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

    public class GoalBox : UnitySubscriber<MessageTypes.Std.Int32>
    {

        private bool isMessageReceived;
        private bool sentOnce = false;
        private MessageTypes.Std.Int32 measurementStatus;

        protected override void Start()
        {       
            base.Start();
        }

        void Update()
		{
            if (isMessageReceived && !sentOnce)
			{
                sentOnce = true;
                StopTimerAndCounter();
            }
                
        }


        protected override void ReceiveMessage(MessageTypes.Std.Int32 stopMeasuring)
        {
            isMessageReceived = true;
            measurementStatus = stopMeasuring;
        }


        private void StopTimerAndCounter()
		{
            if (measurementStatus.data == 1)
			{
                GameObject.Find("Husky_3_fixedAudioSources").SendMessage("Finish");
                GameObject.Find("Husky_5_fixedAudioSources").SendMessage("Finish");
                GameObject.Find("Husky_3_dynamicAudioSources").SendMessage("Finish");
            }
            else if (measurementStatus.data == 0)
            {
                print("Ok, it works");
			}
		}
    }


}
