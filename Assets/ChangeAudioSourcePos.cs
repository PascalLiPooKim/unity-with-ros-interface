using System.Collections;
using System.Collections.Generic;
using UnityEngine;

namespace RosSharp.RosBridgeClient
{
    [RequireComponent(typeof(RosConnector))]
    [RequireComponent(typeof(AudioSource))]

    public class ChangeAudioSourcePos : UnitySubscriber<MessageTypes.Geometry.Vector3>
    {

        // public AudioClip audioClip;
        //private MessageTypes.Std.Float32 closestDistance;
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
            // audioSource.pitch = 0;

        }

        // Update is called once per frame
        void Update()
        {
            if (isMessageReceived)
                PlayBuzzer();
        }


        protected override void ReceiveMessage(MessageTypes.Geometry.Vector3 closestPointPos)
        {
            closestPoint = closestPointPos;
            //print(closestDistance.data);
            Vector3 pos = new Vector3((float)closestPoint.x, (float)closestPoint.y, (float)closestPoint.z);
            newPos = pos;
            closestDistance = Mathf.Pow(pos.magnitude, 2);
            isMessageReceived = true;
        }

        private void PlayBuzzer()
        {


            if (Mathf.Pow(closestDistance, 1f / 2f) > 0.0f && Mathf.Pow(closestDistance, 1f / 2f) < 2.0f)
            {
                //audioSource.pitch = Mathf.Min(1.0f/closestDistance.data, 3);

                audioSource.pitch = Mathf.Clamp(1.0f / Mathf.Pow(closestDistance, 1f / 2f) - 0.1f, 0.1f, 1.5f);
                // audioSource.Play();
                //audioSource.PlayOneShot(impact, 1.0F);
                //audioSource.PlayOneShot(impact, Mathf.Pow(closestDistance.data, 1f / 9f));
                if (!audioSource.isPlaying)
                {
                    audioSource.PlayOneShot(impact, 1f);
                }
                print(closestDistance);

            }

            else
            {
                audioSource.Stop();
                // audioSource.pitch = 1.0f;
                //audioSource.pitch = 0.0f;
            }


            //gameObject.transform.TransformPoint(newPos);
            gameObject.transform.position = newPos.Ros2Unity();
            


            // audioSource.Play();
            isMessageReceived = false;

        }
    }


}