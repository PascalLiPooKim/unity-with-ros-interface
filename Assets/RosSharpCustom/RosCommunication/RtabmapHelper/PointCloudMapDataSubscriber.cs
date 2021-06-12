using RosSharp.RosBridgeClient.MessageTypes.RtabmapRos;
using UnityEngine;
using System.Collections.Generic;
using System.Linq;
using System.Threading;
using System.Threading.Tasks;
using System;
using Pcx;
using UnityEditor.Experimental.GraphView;

namespace RosSharp.RosBridgeClient
{
    [RequireComponent(typeof(RosConnector))]

    public class PointCloudMapDataSubscriber : UnitySubscriber<MessageTypes.RtabmapHelper.PointCloudMapData>
    {
        struct NodeInfo
        {
            public GameObject node;
            public Vector3 localPosition;
            public Quaternion localRotation;
        }

        struct localTransform
        {
            public Vector3 position;
            public Quaternion rotation;
        }

        private MessageTypes.RtabmapHelper.PointCloudMapData pointCloudMapData;
        private MessageTypes.RtabmapHelper.PointCloudNodeData pointCloudNodeData;
        private MessageTypes.RtabmapRos.MapGraph mapGraph;

        private bool isMessageRecieved;

        [Header("GameObject Prefabs")]
        public GameObject mapNodePrefab;
        public GameObject mapNodeCullPrefab;
        
        [Header("Script Settings")]
        public bool isCullEnabled;

        [Header("Debug Variables")]
        public int totalPoints = 0;
        public float updateTime;

        private Dictionary<int, NodeInfo> nodeData;
        private Pcx.PointCloudData pointCloudData;
        private PointCloud pointCloud;
        private List<Vector3> positions;
        private List<Color32> colors;
        private Vector3 rightMax;
        private Vector3 leftMax;


        // Start is called before the first frame update
        private void Start()
        {
            base.Start();
            pointCloudMapData = new MessageTypes.RtabmapHelper.PointCloudMapData();
            pointCloudData = ScriptableObject.CreateInstance<Pcx.PointCloudData>();
            nodeData = new Dictionary<int, NodeInfo>();
        }

        void Update()
        {
            if (isMessageRecieved)
            {
                float timeBeforeUpdate = UnityEngine.Time.realtimeSinceStartup;
                ProcessMessage();
                updateTime = UnityEngine.Time.realtimeSinceStartup - timeBeforeUpdate;
            }
        }

        protected override void ReceiveMessage(MessageTypes.RtabmapHelper.PointCloudMapData pointCloudMapData)
        {
            this.pointCloudMapData = pointCloudMapData;
            isMessageRecieved = true;
        }

        private bool checkVector3Valid(Vector3 vec)
        {
            if (checkFloatValid(vec.x) && checkFloatValid(vec.y) && checkFloatValid(vec.z))
            {
                return true;
            }
            return false;
        }

        private bool checkFloatValid(float num)
        {
            if (num > -100000.0f && num < 100000.0f)
            {
                return true;
            }
            return false;
        }

        private Vector3 RosToUnityVector(MessageTypes.Geometry.Vector3 vector)
        {
            return new Vector3((float)vector.x, (float)vector.y, (float)vector.z).Ros2Unity();
        }

        private Vector3 RosToUnityVector(MessageTypes.Geometry.Point point)
        {
            return new Vector3((float)point.x, (float)point.y, (float)point.z).Ros2Unity();
        }

        private Quaternion RosToUnityQuaternion(MessageTypes.Geometry.Quaternion quaternion)
        {
            return new Quaternion((float)quaternion.x, (float)quaternion.y, (float)quaternion.z, (float)quaternion.w).Ros2Unity();
        }

        private Pcx.PointCloudData RenderPointcloud(MessageTypes.Sensor.PointCloud2 pointCloud2)
        {
            pointCloud = new PointCloud(pointCloud2);

            //Creating new lists for point data
            int pointCloudSize = pointCloud.Points.Length;
            totalPoints += pointCloudSize;
            positions = new List<Vector3>(pointCloudSize);
            colors = new List<Color32>(pointCloudSize);

            //Data for culling
            rightMax = new Vector3(0, 0, 0);
            leftMax = new Vector3(0, 0, 0);

            //Looping through every point in the pointcloud to generate equivalent point for renderer
            for (int i = 0; i < pointCloudSize; i++)
            {
                RgbPoint3 point = pointCloud.Points[i];
                Vector3 pos = new Vector3(point.x, point.y, point.z);

                if (checkVector3Valid(pos))
                {
                    pos = pos.Ros2Unity();
                    positions.Add(pos);
                    if (pos.z > rightMax.z)
                        rightMax = pos;
                    if (pos.z < leftMax.z)
                        leftMax = pos;
                }
                else
                {
                    positions.Add(Vector3.zero);
                }
                colors.Add(new Color32((byte)point.rgb[2], (byte)point.rgb[1], (byte)point.rgb[0], (byte)200));
            }

            //Initializing data in PointCloudData script.
            pointCloudData = ScriptableObject.CreateInstance<Pcx.PointCloudData>();
            pointCloudData.Initialize(positions, colors);

            return pointCloudData;
        }

        private void ProcessMessage()
        {
            pointCloudNodeData = pointCloudMapData.nodes[0];
            mapGraph = pointCloudMapData.graph;
            Debug.Log(pointCloudNodeData.localTransform.Length);
            //Insert data into pointCloudData.
            Pcx.PointCloudData newNodePointCloud = RenderPointcloud(pointCloudNodeData.nodeData);

            //If node already in map, update node data
            if (nodeData.ContainsKey(pointCloudNodeData.id))
            {
                Debug.Log("Updating node [ID]: " + pointCloudNodeData.id);
                NodeInfo prevNodeInfo = nodeData[pointCloudNodeData.id];
                prevNodeInfo.localPosition = RosToUnityVector(pointCloudNodeData.localTransform[0].translation);
                prevNodeInfo.localRotation = RosToUnityQuaternion(pointCloudNodeData.localTransform[0].rotation);
                if (isCullEnabled)
                {
                    prevNodeInfo.node.GetComponentInChildren<Pcx.PointCloudRenderer>().sourceData = newNodePointCloud;
                    prevNodeInfo.node.transform.GetChild(1).gameObject.transform.position = leftMax;
                    prevNodeInfo.node.transform.GetChild(2).gameObject.transform.position = rightMax;

                }
                else
                {
                    prevNodeInfo.node.GetComponent<Pcx.PointCloudRenderer>().sourceData = newNodePointCloud;
                }
            }

            ////If no new node is added remove previous node and add new data
            //else if (nodeData.Count == mapGraph.poses.Length -1 && !nodeData.ContainsKey(pointCloudNodeData.id))
            //{
            //    //Obtain old entry
            //    int prevKey = nodeData.Keys.Max();
            //    Debug.Log("Replacing node [ID]: " + prevKey + " with node [ID]: " + pointCloudNodeData.id);
            //    GameObject prevNodeObject = nodeData[prevKey].node;
            //    nodeData.Remove(prevKey);

            //    //Update node data and add new entry
            //    NodeInfo nodeInfoEntry = new NodeInfo();
            //    if (cull)
            //    {
            //        prevNodeObject.GetComponentInChildren<Pcx.PointCloudRenderer>().sourceData = newNodePointCloud;
            //        prevNodeObject.transform.GetChild(1).gameObject.transform.position = leftMax;
            //        prevNodeObject.transform.GetChild(2).gameObject.transform.position = leftMax;
            //    }
            //    else
            //    {
            //        prevNodeObject.GetComponent<Pcx.PointCloudRenderer>().sourceData = newNodePointCloud;
            //    }

            //    nodeInfoEntry.node = prevNodeObject;
            //    nodeInfoEntry.localPosition = RosToUnityVector(pointCloudNodeData.localTransform[0].translation);
            //    nodeInfoEntry.localRotation = RosToUnityQuaternion(pointCloudNodeData.localTransform[0].rotation);
            //    nodeData.Add(pointCloudNodeData.id, nodeInfoEntry);
            //}

            //Otherwise, add new node to inventory and render pointcloud
            else
            {
                if(pointCloudNodeData.localTransform.Length>0)
                {
                    Debug.Log("Adding new node [ID]: " + pointCloudNodeData.id);

                    NodeInfo nodeInfoEntry = new NodeInfo();
                    nodeInfoEntry.localPosition = RosToUnityVector(pointCloudNodeData.localTransform[0].translation);
                    nodeInfoEntry.localRotation = RosToUnityQuaternion(pointCloudNodeData.localTransform[0].rotation);

                    if (isCullEnabled)
                    {
                        GameObject nodeObject = Instantiate(mapNodeCullPrefab, new Vector3(), new Quaternion());
                        nodeObject.GetComponentInChildren<Pcx.PointCloudRenderer>().sourceData = newNodePointCloud;
                        nodeObject.transform.GetChild(1).gameObject.transform.position = leftMax;
                        nodeObject.transform.GetChild(2).gameObject.transform.position = rightMax;
                        nodeInfoEntry.node = nodeObject;
                        nodeData.Add(pointCloudNodeData.id, nodeInfoEntry);
                    }
                    else
                    {
                        GameObject nodeObject = Instantiate(mapNodePrefab, nodeInfoEntry.localPosition + RosToUnityVector(mapGraph.poses[mapGraph.poses.Count() - 1].position),
    nodeInfoEntry.localRotation * RosToUnityQuaternion(mapGraph.poses[mapGraph.poses.Count() - 1].orientation));
                        nodeObject.GetComponent<Pcx.PointCloudRenderer>().sourceData = newNodePointCloud;
                        nodeInfoEntry.node = nodeObject;
                        nodeData.Add(pointCloudNodeData.id, nodeInfoEntry);
                    }
                }
            }

            //Loop through mapGraph and update frames
            HashSet<int> keepFrames= new HashSet<int>();
            for ( int i = 0; i < mapGraph.posesId.Length; i++)
            {
                if (nodeData.ContainsKey(mapGraph.posesId[i]))
                {
                    //If node included in mapgraph save ID in keepFrames
                    keepFrames.Add(mapGraph.posesId[i]);
                    nodeData[mapGraph.posesId[i]].node.transform.position = RosToUnityVector(mapGraph.poses[i].position);
                    nodeData[mapGraph.posesId[i]].node.transform.rotation = RosToUnityQuaternion(mapGraph.poses[i].orientation);
                }
            }

            //Loop through nodeData and remove entries if not included in keepFrames
            //TODO: Find more efficient way of doing this (combine with previous loop somehow?)
            foreach(var node in nodeData.ToList())
            {
                if (!(keepFrames.Contains(node.Key)))
                {
                    Debug.Log("Remove node [ID]: " + node.Key);
                    GameObject nodeObjectDestroy = nodeData[node.Key].node;
                    nodeData.Remove(node.Key);
                    Destroy(nodeObjectDestroy);
                }
            }

            isMessageRecieved = false;
        }
    }
}

