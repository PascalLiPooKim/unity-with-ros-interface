using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class MuteOneChannel : MonoBehaviour
{
    public bool right;
    // Start is called before the first frame update
    void Start()
    {
        
    }

    // Update is called once per frame
    void Update()
    {
        
    }

    void OnAudioFilterRead(float[] data, int channels)
    {
        if (right)
		{
            for (int i = 0; i < data.Length; i += channels)
            {
                data[i] = 0; // mute left channel
            }
            //print("yes");
        }
		else
		{
            for (int i = 0; i < data.Length; i += channels)
            {
                data[i + 1] = 0; // mute right channel
            }
        }
        
    }

}
