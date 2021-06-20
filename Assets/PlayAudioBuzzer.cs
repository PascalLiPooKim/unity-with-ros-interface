using System;
using System.Collections.Generic;
using UnityEngine;

namespace RosSharp.RosBridgeClient
{
    [RequireComponent(typeof(RosConnector))]
    [RequireComponent(typeof(AudioSource))]

    public class PlayAudioBuzzer : UnitySubscriber<Messages.Standard.Float32>
    {

        // public AudioClip audioClip;
        private Float32 closestDistance;


        // Start is called before the first frame update
        protected override void Start()
        {
            //Initialising ROS node
            base.Start();
            AudioSource audioSource = GetComponent<AudioSource>();
            // audioSource.pitch = 0;
           
        }

        // Update is called once per frame
        void Update()
        {
            if (isMessageReceived)
                PlayBuzzer();
        }


        protected override void ReceiveMessage(Messages.Standard.Float32 closestPointDist)
        {
            closestDistance = closestPointDist;
            isMessageReceived = true;
        }

        private void PlayBuzzer()
        {
            if (closestDistance > 0.0f)
            {
                audioSource.pitch = Mathf.Min(1.0f/closestPointDist, 3);
                audioSource.Play();
            }
            else
            {
                audioSource.Stop();
            }
        
        }
    }


}

