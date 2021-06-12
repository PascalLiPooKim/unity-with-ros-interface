using System.Collections;
using System.Collections.Generic;
using UnityEngine;

namespace RosSharp.RosBridgeClient
{
    [RequireComponent(typeof(RosConnector))]
    public class PointPublisher : UnityPublisher<MessageTypes.Geometry.Point>
    {
        private MessageTypes.Geometry.Point message;
        // Start is called before the first frame update
        void Start()
        {
            base.Start();
            InitializeMessage();
        }

        private void InitializeMessage()
        {
            message = new MessageTypes.Geometry.Point();

        }

        public void UpdateMessage(Vector3 data)
        {
            GetGeometryPoint(data, message);
            Publish(message);

        }

        private static void GetGeometryPoint(Vector3 position, MessageTypes.Geometry.Point geometryPoint)
        {
            geometryPoint.x = position.x;
            geometryPoint.y = position.y;
            geometryPoint.z = position.z;
        }
    }
}
