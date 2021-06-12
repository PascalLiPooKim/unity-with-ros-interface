using System.Collections;
using System.Collections.Generic;
using UnityEngine;

namespace RosSharp.RosBridgeClient
{
    public class BoolSubscriber : UnitySubscriber<MessageTypes.Std.Bool>
    {
        private MessageTypes.Std.Bool boolMessage;
        // Start is called before the first frame update
        void Start()
        {
            base.Start();
        }

        // Update is called once per frame
        void Update()
        {

        }

        protected override void ReceiveMessage(MessageTypes.Std.Bool Bool)
        {
            Debug.Log("RecMessage");
            boolMessage = Bool;
        }
    }
}

