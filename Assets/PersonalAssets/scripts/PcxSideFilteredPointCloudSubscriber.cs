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

using System.Collections.Generic;
using System;
using UnityEngine;

namespace RosSharp.RosBridgeClient
{
    [RequireComponent(typeof(RosConnector))]
    [RequireComponent(typeof(Pcx.PointCloudRenderer))]
    public class PcxSideFilteredPointCloudSubscriber : UnitySubscriber<MessageTypes.Sensor.PointCloud2>
    {

        private PointCloud pointCloud;
        private bool isMessageReceived;
        private int pointCloudSize;
        private string pcName;

        private Pcx.PointCloudRenderer pointCloudRenderer;
        private Pcx.PointCloudData pointCloudData;

        private List<Vector3> positions;
        private List<Color32> colors;

        [SerializeField] int factor = 1;

        [SerializeField] int LiDarRed;
        [SerializeField] int LiDarGreen;
        [SerializeField] int LiDarBlue;

        protected override void Start()
        {
            //Initialising ROS node and saving the particle system
            base.Start();
            pointCloudRenderer = GetComponent<Pcx.PointCloudRenderer>();
            pointCloudData = ScriptableObject.CreateInstance<Pcx.PointCloudData>();
        }

        void Update()
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
            pcName = recPointCloud.header.frame_id;
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
            //Getting num of points for particle emmitter and creating particles of that size
            int pointCloudSize = (int)pointCloud.Points.Length / factor;
            positions = new List<Vector3>(pointCloudSize);
            colors = new List<Color32>(pointCloudSize);

            //Debug.Log("Number of points: " + pointCloudSize);
            //Looping through every point in the pointcloud to generate equivalent particle
            if (pointCloudSize > 0)
			{
                for (int i = 0; i < pointCloudSize; i++)
                {
                    RgbPoint3 point = pointCloud.Points[i];
                    Vector3 pos = new Vector3(point.x, point.y, point.z);

                    if (checkVector3Valid(pos))
                    {
                        positions.Add(pos.Ros2Unity());
                    }
                    else
                    {
                        positions.Add(Vector3.zero);
                    }
                    if (point.rgb != null)
                    {
                        colors.Add(new Color32((byte)point.rgb[2], (byte)point.rgb[1], (byte)point.rgb[0], (byte)200));
                    }
                    else
                    {
                        colors.Add(new Color32((byte)LiDarRed, (byte)LiDarGreen, (byte)LiDarBlue, (byte)200));
                    }
                }

                //Setting data in PointCloudData
                pointCloudData = ScriptableObject.CreateInstance<Pcx.PointCloudData>();
                pointCloudData.Initialize(positions, colors);
                pointCloudRenderer.sourceData = pointCloudData;
                
            }
            else
			{
                pointCloudRenderer.sourceData = null;

            }

            isMessageReceived = false;
        }

    }
}
