using UnityEngine;

namespace RosSharp.RosBridgeClient
{
    public class MavTrajectoryPublisher : UnityPublisher<MessageTypes.Mav_interface.Trajectory>
    {
        public MessageTypes.Mav_interface.Trajectory message;
        private MessageTypes.Geometry.Pose messagePose;
        
        protected override void Start()
        {
            InitializeMessage();
            base.Start();
        }

        private void InitializeMessage()
        {
            message = new MessageTypes.Mav_interface.Trajectory();
        }   
        
        public void UpdateMessage(Vector3[] positions, Quaternion[] orientations, int mav_id, int cam_id)
        {
            if(positions.Length!=orientations.Length)
            {
                Debug.LogError(string.Format("Positions and Orientations not same length: {0} != {1}", positions.Length, orientations.Length));
            }
            message.mav_id = new MessageTypes.Std.Int8((sbyte)mav_id);
            message.cam_id = new MessageTypes.Std.Int8((sbyte)cam_id);
            int pose_num = positions.Length;
            message.pose_num = new MessageTypes.Std.Int8((sbyte)pose_num);
            UpdatePose(positions, orientations, pose_num);
            Publish(message);
        }

        private void UpdatePose(Vector3[] positions, Quaternion[] orientations, int pose_num)
        {   
            message.poses = new MessageTypes.Geometry.Pose[pose_num];
            for (int i=0; i< pose_num; i++)
            {
                messagePose = new MessageTypes.Geometry.Pose();
                messagePose.position = new MessageTypes.Geometry.Point();
                messagePose.orientation = new MessageTypes.Geometry.Quaternion();
                GetGeometryPoint(positions[i].Unity2Ros(), messagePose.position);
                GetGeometryQuaternion(orientations[i].Unity2Ros(), messagePose.orientation);
                message.poses[i] = messagePose;
            }

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

