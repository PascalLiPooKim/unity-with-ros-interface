using System;
using System.Collections.Generic;
using UnityEngine;

namespace RosSharp.RosBridgeClient
{
    [RequireComponent(typeof(RosConnector))]
    [RequireComponent(typeof(AudioSource))]

    public class PlayAudioBuzzer : UnitySubscriber<MessageTypes.Std.Float32>
    {

        private MessageTypes.Std.Float32 closestDistance;
        private bool isMessageReceived;
        AudioSource audioSource;
        public AudioClip impact;
        public bool volumeChanged = false;


        // Start is called before the first frame update
        protected override void Start()
        {
            //Initialising ROS node
            base.Start();
            audioSource = GetComponent<AudioSource>();
           
        }

        // Update is called once per frame
        void Update()
        {
            if (isMessageReceived)
                PlayBuzzer();
        }

        // Callback to receive message from specified topic
        protected override void ReceiveMessage(MessageTypes.Std.Float32 closestPointDist)
        {
            closestDistance = closestPointDist;
            
            isMessageReceived = true;
        }

        private void PlayBuzzer()
        {
 
                if (Mathf.Pow(closestDistance.data, 1f / 2f) > 0.0f && Mathf.Pow(closestDistance.data, 1f / 2f) < 1.7f) // 1.5
                {

                    // Varying change in frequency
                    // audioSource.pitch = Mathf.Clamp(1.0f / Mathf.Pow(closestDistance.data, 1f / 2f) + 1.0f, 0.1f, 2.8f);

                    // Staircase change in frequency
                    if (volumeChanged)
				    {
                        audioSource.volume = Mathf.Clamp(1.0f / Mathf.Pow(closestDistance.data, 1f / 2f) - 0.5f, 0.0f, 1.0f);
                    }

                    float val = 1.0f / Mathf.Pow(closestDistance.data, 1f / 3.5f) + 0.5f;

                    if (val <= 1.3f)
                    {
                    audioSource.pitch = 1.2f;
                    }
                    else if (val > 1.3f && val <= 1.6f)
                    {
                        audioSource.pitch = 1.7f;
                    }
                    else if (val > 1.6f && val <= 1.9f)
                    {
                        audioSource.pitch = 2.1f;
                    }

                    else
                    {
                        audioSource.pitch = 2.5f;
                    }


                // Prevent interruption of playing sound after each update
                if (!audioSource.isPlaying)
                    {
                        audioSource.PlayOneShot(impact, 1f);
                    }

            }

                else
                {
                    audioSource.Stop();
                }

                
            isMessageReceived = false;

        }
    }


}

