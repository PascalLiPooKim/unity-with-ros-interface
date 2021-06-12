using System.Collections;
using System.Collections.Generic;
using UnityEngine;

namespace RosSharp.RosBridgeClient
{
    public class boolPublish : UnityPublisher<MessageTypes.Std.Bool>
    {
        private MessageTypes.Std.Bool message;
        public bool messageData;
        public bool publishMessage;
        protected override void Start()
        {
            base.Start();
            message = new MessageTypes.Std.Bool();
        }

        // Update is called once per frame
        void Update()
        {
            if(publishMessage)
            {
                message.data = messageData;
                Publish(message);
            }
        }
    }
}

