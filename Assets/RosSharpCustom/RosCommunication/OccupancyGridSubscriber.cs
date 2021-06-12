using UnityEngine;
using System;

namespace RosSharp.RosBridgeClient
{
    [RequireComponent(typeof(RosConnector))]
    public class OccupancyGridSubscriber : UnitySubscriber<MessageTypes.Nav.OccupancyGrid>
    {
        public MeshRenderer meshRenderer;
        public GameObject mapObject;
        private Texture2D texture2D;
        private MessageTypes.Nav.OccupancyGrid recievedGrid;
        private Color32[] colourArray;
        private bool isMessageReceived = false;

        protected override void Start()
        {
            base.Start();
            texture2D = new Texture2D(1, 1);
            meshRenderer.material = new Material(Shader.Find("Standard"));
            Debug.Log("Initialized");
        }


        void Update()
        {
            if (isMessageReceived)
                ProcessMessage(); 
        }


        protected override void ReceiveMessage(MessageTypes.Nav.OccupancyGrid message)
        {
            
            recievedGrid = message;
            isMessageReceived = true;
        }


        void ProcessMessage()
        {
            //Loop through data and assign texture pixel colours
            colourArray = new Color32[recievedGrid.info.height * recievedGrid.info.width];
            for(int i=0; i< recievedGrid.info.height * recievedGrid.info.width; i++)
            {
                if ((int)recievedGrid.data[i] == -1)
                {
                    colourArray[i] = new Color32(171, 195, 205, 255);
                }
                else
                {
                    byte occupancyColour = (byte)(255 - Mathf.Round(recievedGrid.data[i] / 100.0f * 255));
                    colourArray[i] = new Color32(occupancyColour, occupancyColour, occupancyColour, 255);
                }
            }
            //Resize, apply new colours and set texture
            texture2D.Resize((int)recievedGrid.info.width, (int)recievedGrid.info.height);
            texture2D.SetPixels32(colourArray,0);
            texture2D.Apply();
            meshRenderer.material.SetTexture("_MainTex", texture2D);

            //Rescale gameObject to match map size and position correctly in the scene
            mapObject.transform.localScale = new Vector3((recievedGrid.info.width * recievedGrid.info.resolution), 0.001f, (recievedGrid.info.height * recievedGrid.info.resolution));
            gameObject.transform.rotation = Quaternion.Euler(new Vector3(0, 90, 0));
            gameObject.transform.position = GetPosition(recievedGrid.info.origin).Ros2Unity() + new Vector3(-recievedGrid.info.height * recievedGrid.info.resolution, 0, recievedGrid.info.width * recievedGrid.info.resolution) / 2;

            isMessageReceived = false;   
        }

        private Vector3 GetPosition(MessageTypes.Geometry.Pose message)
        {
            return new Vector3(
                (float)message.position.x,
                (float)message.position.y,
                (float)message.position.z);
        }

        //public Colour32[] GetColour()
        //{
        //    return texture2D;
        //}
    }
}