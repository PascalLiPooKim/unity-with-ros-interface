using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class VisibilityCheck: MonoBehaviour
{
    public bool visible;

    void OnBecameVisible()
    {
        visible = true;
    }

    void OnBecameInvisible()
    {
        visible = false;
    }
}
