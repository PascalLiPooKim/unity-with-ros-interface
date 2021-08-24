using System.Collections;
using System.Collections.Generic;
using UnityEngine;


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

        // Callback to receive message from topic
        protected override void ReceiveMessage(MessageTypes.Std.Int32 stopMeasuring)
        {
            isMessageReceived = true;
            measurementStatus = stopMeasuring;
        }

        // Call the "Finish" function from Measurement.cs when command is sent from ROS
        private void StopTimerAndCounter()
		{
            if (measurementStatus.data == 1)
			{
                if (GameObject.Find("Husky_3_fixedAudioSources")){
                    GameObject.Find("Husky_3_fixedAudioSources").SendMessage("Finish");
                }
                else if (GameObject.Find("Husky_5_fixedAudioSources")){
                    GameObject.Find("Husky_5_fixedAudioSources").SendMessage("Finish");
                }
                else if (GameObject.Find("Husky_3_dynamicAudioSources")){
                    GameObject.Find("Husky_3_dynamicAudioSources").SendMessage("Finish");
                }


            }
            else if (measurementStatus.data == 0)
            {
                print("Ok, it works");
			}
		}
    }


}
