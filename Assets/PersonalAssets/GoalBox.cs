using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class GoalBox : MonoBehaviour
{
    //public static Vector3 newPos;
    // Start is called before the first frame update
    void Start()
    {
        // Vector3 newPos = new Vector3(1.0f, 0.0f, 4.0f);
        // gameObject.transform.TransformPoint(newPos);
    }

    // Update is called once per frame
    void Update()
    {
        
    }

    private void OnTriggerEnter(Collider other)
	{
        GameObject.Find("Husky_V2").SendMessage("Finish");
	}
}
