using System.Collections;
using System.Collections.Generic;
using UnityEngine;

namespace RosSharp.RosBridgeClient
{
    [RequireComponent(typeof(RosConnector))]
    [RequireComponent(typeof(AudioSource))]

    public class ChangeAudioSourcePos : UnitySubscriber<MessageTypes.Geometry.Vector3>
    {

        
        private float closestDistance;
        private MessageTypes.Geometry.Vector3 closestPoint;
        private bool isMessageReceived;
        AudioSource audioSource;
        public AudioClip impact;
        private Vector3 newPos;


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

        // Callback to receive message from topic. ROS Vector3 from ROS are turned into Unity Vector3
        protected override void ReceiveMessage(MessageTypes.Geometry.Vector3 closestPointPos)
        {
            closestPoint = closestPointPos;
            
            Vector3 pos = new Vector3((float)closestPoint.x, (float)closestPoint.y, (float)closestPoint.z);
            newPos = pos;
            closestDistance = Mathf.Pow(pos.magnitude, 2);
            isMessageReceived = true;
        }

        // Check distance of closest object/point and play sound if object is within range
        private void PlayBuzzer()
        {


            if (Mathf.Pow(closestDistance, 1f / 2f) > 0.0f && Mathf.Pow(closestDistance, 1f / 2f) < 2.0f)
            {

                audioSource.pitch = Mathf.Clamp(1.0f / Mathf.Pow(closestDistance, 1f / 2f) - 0.1f, 0.1f, 2.0f);
                
                if (!audioSource.isPlaying)
                {
                    audioSource.PlayOneShot(impact, 1f);
                }
                print(closestDistance);

            }

            else
            {
                audioSource.Stop();
           
            }
        
            // Move audio source to closest object
            gameObject.transform.localPosition = newPos.Ros2Unity();
            
            isMessageReceived = false;

        }
    }


}