using System;
using System.Collections.Generic;
using UnityEngine;

namespace RosSharp.RosBridgeClient
{
    [RequireComponent(typeof(RosConnector))]
    [RequireComponent(typeof(AudioSource))]

    public class PlayAudioBuzzer : UnitySubscriber<MessageTypes.Std.Float32>
    {

        // public AudioClip audioClip;
        private MessageTypes.Std.Float32 closestDistance;
        private bool isMessageReceived;
        AudioSource audioSource;


        // Start is called before the first frame update
        protected override void Start()
        {
            //Initialising ROS node
            base.Start();
            audioSource = GetComponent<AudioSource>();
            // audioSource.pitch = 0;
           
        }

        // Update is called once per frame
        void Update()
        {
            if (isMessageReceived)
                PlayBuzzer();
        }


        protected override void ReceiveMessage(MessageTypes.Std.Float32 closestPointDist)
        {
            closestDistance = closestPointDist;
            isMessageReceived = true;
        }

        private void PlayBuzzer()
        {
            if (closestDistance.data > 0.0f)
            {
                audioSource.pitch = Mathf.Min(1.0f/closestDistance.data, 3);
                audioSource.Play();
            }
            else
            {
                audioSource.Stop();
            }

            isMessageReceived = false;

        }
    }


}

