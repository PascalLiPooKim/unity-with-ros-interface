using System.Collections;
using System.Collections.Generic;
using UnityEngine;

namespace RosSharp.RosBridgeClient
{
    public class MavStatePublisher : UnityPublisher<MessageTypes.Mav_interface.State>
    {
        private MessageTypes.Mav_interface.State message;
        // Start is called before the first frame update
        protected override void Start()
        {
            base.Start();
            message = new MessageTypes.Mav_interface.State();
        }

        // Update is called once per frame
        public void UpdateMessage(int mav_id, int cam_id, bool connected, bool armed, bool ready)
        {
            message.mav_id = new MessageTypes.Std.Int8((sbyte)mav_id);
            message.cam_id = new MessageTypes.Std.Int8((sbyte)cam_id);
            message.connected= connected;
            message.armed = armed;
            message.ready = ready;
            Publish(message);
        }
    }
}
