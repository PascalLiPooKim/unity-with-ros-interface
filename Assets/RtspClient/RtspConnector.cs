using UnityEngine;
using System;
using System.Linq;
using System.Collections.Generic;
using System.Drawing;
using System.Threading;
using System.Threading.Tasks;
using RtspClientSharp;
using RtspClientSharp.RawFrames.Video;
using RtspClientSharp.RawFrames;
using RtspCapture.RawFramesDecoding;
using RtspCapture.RawFramesDecoding.DecodedFrames;
using RtspCapture.RawFramesDecoding.FFmpeg;
using UnityEngine.Events;
using RosSharp.RosBridgeClient.MessageTypes.Std;

public class RtspConnector : MonoBehaviour
{
    [Header("RTSP Settings")]
    public string Uri;
    public bool isUDP;
    public bool isForceGC;

    private bool latencyTest;
    private int latencyThreshold;
    [System.Serializable]
    private class UpdateEvent : UnityEvent<bool> { };
    private UpdateEvent RTSPTrigger;

    //Variables needed for logic of latency check
    private int? prevFrameTimeStamp = null;
    private DateTime epochStart = new DateTime(1970, 1, 1, 0, 0, 0, DateTimeKind.Utc);
    private bool prevLatencyCheck = true;

    //RTSP Settings
    private CancellationTokenSource cancellationTokenSource;
    private Task makeSnapshotsTask;

    private const int streamWidth = 1;
    private const int streamHeight = 1;

    private static int targetWidth;
    private static int targetHeight;

    private static byte[] _decodedFrameBuffer = new byte[0];

    private static PostVideoDecodingParameters _postVideoDecodingParameters = new PostVideoDecodingParameters(RectangleF.Empty,
        new Size(0, 0), ScalingPolicy.Stretch, RtspCapture.RawFramesDecoding.PixelFormat.Bgr24, ScalingQuality.Nearest);

    private static readonly Dictionary<FFmpegVideoCodecId, FFmpegVideoDecoder> _videoDecodersMap =
        new Dictionary<FFmpegVideoCodecId, FFmpegVideoDecoder>();

    [Header("Unity Settings")]
    public MeshRenderer Renderer;
    public Texture2D texture;
    private static Texture2D textureTEST;
    private byte[] frame_bytes = new byte[0];
    private bool loadtex = false;

    [Header("Debug Settings")]
    public bool checkDecodeTime;
    public List<double> frameDecodeTime = new List<double>();


    private System.Diagnostics.Stopwatch sw = new System.Diagnostics.Stopwatch();

    void Start()
    {
        //Setting up variables required for RTSPClientSharp and starting the aync task to obtain RTSP stream
        var serverUri = new Uri(Uri);
        cancellationTokenSource = new CancellationTokenSource();
        texture = new Texture2D(streamWidth, streamHeight, TextureFormat.RGB24, false);
        Renderer.material.SetTextureScale("_MainTex", new Vector2(1, -1));
        Task makeSnapshotsTask = Task.Run(() => MakeSnapshotsAsync(serverUri, cancellationTokenSource.Token));
    }


    void Update()
    {
        //Check Latency - if greater than threshold, invoke event
        if(prevFrameTimeStamp != null)
        {
            int currentTime = (int)(DateTime.UtcNow - epochStart).TotalMilliseconds;
            float timeDiff = Mathf.Abs(currentTime - (int)prevFrameTimeStamp);

            //
            //if ((timeDiff > latencyThreshold) & ((timeDiff > latencyThreshold) != prevLatencyCheck))
            bool latencyTest = (timeDiff > latencyThreshold);
            if (latencyTest && latencyTest == prevLatencyCheck)
            {
                RTSPTrigger.Invoke(false);
                prevLatencyCheck = false;
            }
            else if(!latencyTest && (latencyTest == prevLatencyCheck))
            {
                RTSPTrigger.Invoke(true);
                prevLatencyCheck = true;
            }

        }

        //Loads new textures recieved from RTSPClientSharp
        if (loadtex == true)
        {
            if (streamWidth != targetWidth || streamHeight != targetHeight)
                texture = new Texture2D(targetWidth, targetHeight, TextureFormat.RGB24, false);

            texture.LoadRawTextureData(frame_bytes);
            Renderer.material.mainTexture = texture;
            texture.Apply();
            if (sw.IsRunning)
            {
                sw.Stop();
                //Debug.Log(sw.Elapsed.TotalMilliseconds);
                frameDecodeTime.Add(sw.Elapsed.TotalMilliseconds);
            }
            Resources.UnloadUnusedAssets();
            loadtex = false;
        }
    }


    private void OnApplicationQuit()
    {
        //Cancel the task when the application is closed
        cancellationTokenSource.Cancel();

    }


    public async Task MakeSnapshotsAsync(Uri serverUri, CancellationToken token)
    {
        //Recieves frames from RTSP stream, decodes using FFMPEG and saves raw bytes
        var connectionParameters = new ConnectionParameters(serverUri);

        //Set required transport protocal
        if (isUDP)
            connectionParameters.RtpTransport = RtpTransportProtocol.UDP;
        else
            connectionParameters.RtpTransport = RtpTransportProtocol.TCP;

        connectionParameters.ReceiveTimeout = TimeSpan.FromSeconds(60);
        connectionParameters.UserAgent = serverUri.ToString();

        using (var rtspClient = new RtspClient(connectionParameters))
        {
            rtspClient.FrameReceived += (sender, frame) =>
            {
                if ((frame is RawH264PFrame) | (frame is RawH264IFrame))
                {
                    if (!sw.IsRunning)
                    {
                        sw.Reset();
                        sw.Start();
                    }
                        
                    //Tracking time
                    prevFrameTimeStamp = (int)(frame.Timestamp.ToUniversalTime() - epochStart).TotalMilliseconds;

                    FFmpegVideoDecoder decoder = GetDecoderForFrame(frame);

                    if (!(bool)decoder.TryDecode(frame, out DecodedVideoFrameParameters decodedFrameParameters))
                        return;

                    targetWidth = decodedFrameParameters.Width;
                    targetHeight = decodedFrameParameters.Height;

                    int bufferSize = targetHeight * ImageUtils.GetStride(targetWidth, RtspCapture.RawFramesDecoding.PixelFormat.Bgr24);

                    if (_decodedFrameBuffer.Length != bufferSize)
                        _decodedFrameBuffer = new byte[bufferSize];

                    var bufferSegment = new ArraySegment<byte>(_decodedFrameBuffer);

                    if (_postVideoDecodingParameters.TargetFrameSize.Width != targetWidth ||
                        _postVideoDecodingParameters.TargetFrameSize.Height != targetHeight)
                    {
                        _postVideoDecodingParameters = new PostVideoDecodingParameters(RectangleF.Empty,
                        new Size(targetWidth, targetHeight),
                        ScalingPolicy.Stretch, RtspCapture.RawFramesDecoding.PixelFormat.Bgr24, ScalingQuality.Nearest);
                    }

                    IDecodedVideoFrame decodedFrame = decoder.GetDecodedFrame(bufferSegment, _postVideoDecodingParameters);
                    var frame_segarray = decodedFrame.DecodedBytes;
                    frame_bytes = frame_segarray.Array;
                    loadtex = true;
                    if (isForceGC)
                        GC.Collect();
                    return;
                }
            };

            Debug.Log("Connecting...");
            await rtspClient.ConnectAsync(token);
            Debug.Log("Receiving...");
            await rtspClient.ReceiveAsync(token);
        }
    }


    private static FFmpegVideoDecoder GetDecoderForFrame(RawFrame videoFrame)
    {
        //Gets the correct decoder for frame
        FFmpegVideoCodecId codecId = DetectCodecId(videoFrame);
        if (!_videoDecodersMap.TryGetValue(codecId, out FFmpegVideoDecoder decoder))
        {
            decoder = FFmpegVideoDecoder.CreateDecoder(codecId);
            _videoDecodersMap.Add(codecId, decoder);
        }

        return decoder;
    }


    private static FFmpegVideoCodecId DetectCodecId(RawFrame videoFrame)
    {
        //Gets the correct codec ID for frame
        if (videoFrame is RawJpegFrame)
            return FFmpegVideoCodecId.MJPEG;
        if (videoFrame is RawH264Frame)
        {
            return FFmpegVideoCodecId.H264;

        }

        throw new ArgumentOutOfRangeException(nameof(videoFrame));
    }
}
