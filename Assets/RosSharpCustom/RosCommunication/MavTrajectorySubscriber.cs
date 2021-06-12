using UnityEngine;

namespace RosSharp.RosBridgeClient
{
    [RequireComponent(typeof(RosConnector))]
    public class MavTrajectorySubscriber : UnitySubscriber<MessageTypes.Mav_interface.Trajectory>
    {
        public MessageTypes.Mav_interface.Trajectory trajectory;
        public bool isMessageReceived;

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
        public void ProcessMessage()
        {
            Debug.Log(trajectory.pose_num.data.ToString());
        }
        
        protected override void ReceiveMessage(MessageTypes.Mav_interface.Trajectory message)
        {
            trajectory = message;
            isMessageReceived = true;
        }
    }
}

