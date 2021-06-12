using System.Collections;
using System.Collections.Generic;
using UnityEngine;
namespace RosSharp.RosBridgeClient
{
public class PosePublisher : UnityPublisher<MessageTypes.Geometry.PoseStamped>
{
        public string FrameId = "Unity";

        private MessageTypes.Geometry.PoseStamped message;

        protected override void Start()
        {
            base.Start();
            InitializeMessage();
        }

        //private void FixedUpdate()
        //{
        //    UpdateMessage();
        //}

        private void InitializeMessage()
        {
            message = new MessageTypes.Geometry.PoseStamped
            {
                header = new MessageTypes.Std.Header()
                {
                    frame_id = FrameId
                }
            };
        }

        public void UpdateMessage(GameObject targetObject)
        {
            //message.header.Update();
            GetGeometryPoint(targetObject.GetComponent<Transform>().position.Unity2Ros(), message.pose.position);
            GetGeometryQuaternion(targetObject.GetComponent<Transform>().rotation.Unity2Ros(), message.pose.orientation);

            Publish(message);
        }

        private static void GetGeometryPoint(Vector3 position, MessageTypes.Geometry.Point geometryPoint)
        {
            geometryPoint.x = position.x;
            geometryPoint.y = position.y;
            geometryPoint.z = position.z;
        }

        private static void GetGeometryQuaternion(Quaternion quaternion, MessageTypes.Geometry.Quaternion geometryQuaternion)
        {
            geometryQuaternion.x = quaternion.x;
            geometryQuaternion.y = quaternion.y;
            geometryQuaternion.z = quaternion.z;
            geometryQuaternion.w = quaternion.w;
        }

    }
}


