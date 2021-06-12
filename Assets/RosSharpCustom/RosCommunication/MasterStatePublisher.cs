using RosSharp.RosBridgeClient.MessageTypes.Std;
using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using System.Diagnostics;
using System;
using System.Net.Sockets;

namespace RosSharp.RosBridgeClient
{
    public class MasterStatePublisher : UnityPublisher<MessageTypes.Mav_interface.MasterState>
    {
        private MessageTypes.Mav_interface.MasterState message;
        public Header header = new Header();
        public int update_rate = 50;
        public int id;
        public bool ready;

        //Header timestamp variables
        private DateTime epochStart = new DateTime(1970, 1, 1, 0, 0, 0, DateTimeKind.Utc);
        private DateTime startTime = DateTime.UtcNow;
        private int startTimeSeconds;
        private Stopwatch stopWatch = new Stopwatch();
        public uint seq = 0;

        protected override void Start()
        {
            base.Start();
            message = new MessageTypes.Mav_interface.MasterState();
            startTimeSeconds = (int)(startTime - epochStart).TotalSeconds;
            stopWatch.Start();
        }

        private void FixedUpdate()
        {
            UpdateHeader();
            UpdateMessage();
        }

        private void UpdateHeader()
        {
            //Incrementing seq value
            seq += 1;

            //Calculating secs and nsecs
            double stamp_nsecs = stopWatch.ElapsedTicks / (Stopwatch.Frequency / (1000d * 1000d * 1000d));
            double stamp_secs = Math.Floor(stamp_nsecs * 1e-9);

            //Updating variables
            header.seq = seq;
            header.stamp.secs = (uint)(stamp_secs + startTimeSeconds);
            header.stamp.nsecs = (uint)(stamp_nsecs - (stamp_secs / 1e-9));

        }
        private void UpdateMessage()
        {
            message.header = header;
            message.update_rate.data = (sbyte)update_rate;
            message.id.data = (sbyte)id;
            message.ready = ready;
            Publish(message);
        }

        public void changeID(int newID)
        {
            id = newID;
        }
    }
}
