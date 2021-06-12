/*
© Siemens AG, 2017-2018
Author: Dr. Martin Bischoff (martin.bischoff@siemens.com)
Licensed under the Apache License, Version 2.0 (the "License");
you may not use this file except in compliance with the License.
You may obtain a copy of the License at
<http://www.apache.org/licenses/LICENSE-2.0>.
Unless required by applicable law or agreed to in writing, software
distributed under the License is distributed on an "AS IS" BASIS,
WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
See the License for the specific language governing permissions and
limitations under the License.
*/

using System;
using System.Linq;
using UnityEngine;

namespace RosSharp.RosBridgeClient
{
    [RequireComponent(typeof(RosConnector))]
    public class PointCloudSubscriber : UnitySubscriber<MessageTypes.Sensor.PointCloud2>
    {
        
        private PointCloud pointCloud;
        private byte[] pointCloudData;
        private bool isMessageReceived;
        private int pointCloudSize;


        private ParticleSystem particleSystem;
        private ParticleSystem.Particle[] particles;

        private void Start()
        {
            //Initialising ROS node and saving the particle system
            base.Start();
            particleSystem = GetComponent<ParticleSystem>();
        }

        private void Update()
        {
            //Processing a new mesage when recieved
            if (isMessageReceived)
                ProcessMessage();
            //Debug.Log(1.0f / Time.deltaTime);
        }

        protected override void ReceiveMessage(MessageTypes.Sensor.PointCloud2 recPointCloud)
        {
            //Obtaining pointcloud data from message and number of points, setting bool to true
            pointCloud = new PointCloud(recPointCloud);
            isMessageReceived = true;
        }

        private bool checkVector3Valid(Vector3 vec)
        {
            if (checkFloatValid(vec.x) && checkFloatValid(vec.y) && checkFloatValid(vec.z))
            {
                return true;
            }
            return false;
        }

        private bool checkFloatValid(float num)
        {
            if (num > -100000.0f && num < 100000.0f)
            {
                return true;
            }
            return false;
        }

        private void ProcessMessage()
        {
            Debug.Log("Recieved Message");
            //Getting num of points for particle emmitter and creating particles of that size
            int pointCloudSize = pointCloud.Points.Length;
            particles = new ParticleSystem.Particle[pointCloudSize];
            Debug.Log(pointCloudSize);
            //Looping through every point in the pointcloud to generate equivalent particle
            for (int i = 0; i < pointCloudSize; i++)
            {
                //Saving the particle's position and generating an empty RGBpoint3
                RgbPoint3 point = pointCloud.Points[i];
                Vector3 pos = new Vector3(point.x, point.y, point.z);
                

                if (checkVector3Valid(pos))
                {
                    particles[i].position = pos.Ros2Unity();
                }
                else
                {
                    particles[i].position = Vector3.zero;
                }

                //Setting the partcle rotation, size and colour
                particles[i].rotation3D = Vector3.zero;
                particles[i].startSize = 0.05f;
                particles[i].startColor = new Color32((byte)point.rgb[2], (byte)point.rgb[1], (byte)point.rgb[0], (byte)200);
            }

            //Visualising the particles
            particleSystem.SetParticles(particles, particles.Length);
            isMessageReceived = false;
            Debug.Log("Message Rendered");
        }

    }
}
