using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class CullCheck : MonoBehaviour
{
    // Start is called before the first frame update
    public GameObject leftMax;
    public GameObject rightMax;
    public GameObject pclRenderer;

    private VisibilityCheck leftCullCheck;
    private VisibilityCheck rightCullCheck;
    private VisibilityCheck gameObjectCullCheck;

    private void Start()
    {
        leftCullCheck = leftMax.GetComponent<VisibilityCheck>();
        rightCullCheck = rightMax.GetComponent<VisibilityCheck>();
        gameObjectCullCheck = GetComponent<VisibilityCheck>();
    }

    // Update is called once per frame
    void Update()
    {
        if(leftCullCheck.visible == false && rightCullCheck.visible == false && gameObjectCullCheck.visible == false)
        {
            pclRenderer.SetActive(false);
        }
        else
        {
            pclRenderer.SetActive(true);
            //Debug.Log("Unculled one!");
        }
    }
}
