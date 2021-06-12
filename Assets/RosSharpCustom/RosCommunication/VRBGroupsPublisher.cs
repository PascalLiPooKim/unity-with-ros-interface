using RosSharp.RosBridgeClient.MessageTypes.Mav_interface;
using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using System.Linq;

namespace RosSharp.RosBridgeClient
{
    public class VRBGroupsPublisher : UnityPublisher<MessageTypes.Mav_interface.VRBGroups>
    {
        private MessageTypes.Mav_interface.VRBGroups message;

        protected override void Start()
        {
            base.Start();
            InitializeMessage();
        }

        void InitializeMessage()
        {
            message = new MessageTypes.Mav_interface.VRBGroups();
        }

        public void UpdateMessage(List<Group> groups)
        {
            message = new VRBGroups();
            message.groups = new VRBGroup[groups.Count];

            for(int i =0; i<groups.Count; i++)
            {
                VRBGroup vRBGroup = new VRBGroup();
                vRBGroup.groupID = groups[i].ID;
                vRBGroup.robotNumber = groups[i].robotNumber;
                vRBGroup.robotID = groups[i].robotID.ToArray();

                //Setting VRBPose
                GetGeometryPoint(groups[i].VRBGameObject.transform.position.Unity2Ros(), vRBGroup.VRBPose.position);
                GetGeometryQuaternion(groups[i].VRBGameObject.transform.rotation.Unity2Ros(), vRBGroup.VRBPose.orientation);


                //Setting robot poses
                vRBGroup.robotPose = new MessageTypes.Geometry.Pose[groups[i].robotVRBPosition.Count];
                for(int j = 0; j<groups[i].robotVRBPosition.Count; j++)
                {
                    vRBGroup.robotPose[j] = new MessageTypes.Geometry.Pose();
                    GetGeometryPoint(groups[i].robotVRBPosition[j].transform.position.Unity2Ros(), vRBGroup.robotPose[j].position);
                    GetGeometryQuaternion(groups[i].robotVRBPosition[j].transform.rotation.Unity2Ros(), vRBGroup.robotPose[j].orientation);
                }

                message.groups[i] = vRBGroup;
            }

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

