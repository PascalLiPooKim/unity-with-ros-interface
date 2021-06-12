using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using UnityEditor;


[CustomEditor(typeof(SelectionSection))]
public class SelectionSectionEditor : Editor
{
    public override void OnInspectorGUI()
    {
        DrawDefaultInspector();

        var selectionMenu = (SelectionMenu)target;
    }
}
