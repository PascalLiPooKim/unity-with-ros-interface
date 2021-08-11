﻿using System.Collections;
using System.Collections.Generic;
using UnityEngine;

namespace RosSharp.RosBridgeClient
{
    [RequireComponent(typeof(RosConnector))]

    public class BlockCamera : UnitySubscriber<MessageTypes.Std.Bool>
    {

        private bool isMessageReceived;
        private bool spawn = false;
        public GameObject cube;

        // Start is called before the first frame update
        protected override void Start()
        {
            //Initialising ROS node
            base.Start();
            cube.SetActive(false);


        }

        // Update is called once per frame
        void Update()
        {
            if (isMessageReceived)
                SpawnCube();
        }


        protected override void ReceiveMessage(MessageTypes.Std.Bool pause)
        {
            print("lol");
            spawn = pause.data;
            print(spawn);
            isMessageReceived = true;
        }

        private void SpawnCube()
		{
            
            if (spawn)
			{
                cube.SetActive(true);
                print("Spawn");
            }
			else
			{
                cube.SetActive(false);
            }

            isMessageReceived = false;
        }

    }
}