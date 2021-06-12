using UnityEngine;

namespace RosSharp.RosBridgeClient
{
    public class TwistControlPublisher : UnityPublisher<MessageTypes.Geometry.TwistStamped>
    {
        private MessageTypes.Geometry.TwistStamped message;

        protected override void Start()
        {
            base.Start();
            InitializeMessage();
        }

        private void InitializeMessage()
        {
            message = new MessageTypes.Geometry.TwistStamped();
            message.header = new MessageTypes.Std.Header();
            message.twist.linear = new MessageTypes.Geometry.Vector3();
            message.twist.angular = new MessageTypes.Geometry.Vector3();
        }

        public void UpdateMessage(Vector3 linearVelocity, Vector3 angularVelocity)
        {
            message.header.Update();
            message.twist.linear = GetGeometryVector3(linearVelocity.Unity2Ros());
            message.twist.angular = GetGeometryVector3(-angularVelocity.Unity2Ros());
            Publish(message);
            //Debug.Log("Published");
        }

        private static MessageTypes.Geometry.Vector3 GetGeometryVector3(Vector3 vector3)
        {
            MessageTypes.Geometry.Vector3 geometryVector3 = new MessageTypes.Geometry.Vector3();
            geometryVector3.x = vector3.x;
            geometryVector3.y = vector3.y;
            geometryVector3.z = vector3.z;
            return geometryVector3;
        }
    }
}
