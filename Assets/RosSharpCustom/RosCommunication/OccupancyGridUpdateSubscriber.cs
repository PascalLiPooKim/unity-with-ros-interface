using System.Collections;
using System.Collections.Generic;
using UnityEngine;

namespace RosSharp.RosBridgeClient
{
    [RequireComponent(typeof(RosConnector))]
    [RequireComponent(typeof(OccupancyGridSubscriber))]
    public class OccupancyGridUpdateSubscriber : UnitySubscriber<MessageTypes.Nav.OccupancyGridUpdate>
    {
        public MeshRenderer meshRenderer;
        public OccupancyGridSubscriber occupancyGridSubscriber;

        private Color32[] colourArray;
        private MessageTypes.Nav.OccupancyGridUpdate message;
        private bool isMessageRecieved;

        // Start is called before the first frame update
        void Start()
        {

        }

        // Update is called once per frame
        void Update()
        {
            if (isMessageRecieved)
                ProcessMessage();
        }

        protected override void ReceiveMessage(MessageTypes.Nav.OccupancyGridUpdate recievedMessage)
        {

            message = recievedMessage;
            isMessageRecieved = true;
        }

        private void ProcessMessage()
        {
            //texture2D = occupancyGridSubscriber.GetTexture();

        }
    }
}

