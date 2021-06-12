using RosSharp.RosBridgeClient.MessageTypes.Std;
using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using UnityEngine.Events;

namespace RosSharp.RosBridgeClient
{
    public class MavStateSubscriber : UnitySubscriber<MessageTypes.Mav_interface.State>
    {
        public MessageTypes.Mav_interface.State state;
        public bool isMessageReceived = false;

        [Header("Message")]
        public Header header;
        public int mav_id;
        public int cam_id;
        public bool connected;
        public bool armed;
        public bool ready;
        public int update_rate;

        [System.Serializable]
        public class UpdateEvent : UnityEvent<bool, bool, bool> { };
        public UpdateEvent stateUpdate;


        // Start is called before the first frame update
        protected override void Start()
        {
            base.Start();
        }

        // Update is called once per frame
        void Update()
        {
            if (isMessageReceived)
                ProcessMessage();
        }

        private void ProcessMessage()
        {
            //Saving values
            header = state.header;
            cam_id = state.cam_id.data;
            mav_id = state.mav_id.data;
            connected = state.connected;
            armed = state.armed;
            ready = state.ready;
            update_rate = state.update_rate.data;
            
            //Enabling track latecy
            stateUpdate.Invoke(connected, armed, ready);
        }

        protected override void ReceiveMessage(MessageTypes.Mav_interface.State message)
        {
            state = message;
            isMessageReceived = true;
        }
    }
}

