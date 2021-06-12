using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using UnityEditor;

[CustomEditor(typeof(SelectionMenu))]
public class SelectionMenuEditor : Editor
{
    public override void OnInspectorGUI()
    {
        DrawDefaultInspector();

        var selectionMenu = (SelectionMenu)target;
        //Setting text of selection menu to match the string
        //for (int i=0; i<selectionMenu.selectionSections.Count; i++)
        //{
        //    selectionMenu.selectionSections[i].testMesh = new TextMesh();
        //    selectionMenu.selectionSections[i].testMesh.text = selectionMenu.selectionSections[i].SectionName;
        //    selectionMenu.selectionSections[i].testMesh.font = (Font)Resources.Load("Arial");
        //}
    }
}
