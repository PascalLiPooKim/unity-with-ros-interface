// Pcx - Point cloud importer & renderer for Unity
// https://github.com/keijiro/Pcx

using UnityEngine;
using UnityEditor;

namespace Pcx
{
    [CanEditMultipleObjects]
    [CustomEditor(typeof(PointCloudRenderer))]
    public class PointCloudRendererInspector : Editor
    {
        SerializedProperty _sourceData;
        SerializedProperty _pointTint;
        SerializedProperty _pointSize;
        SerializedProperty _pointShader;
        SerializedProperty _diskShader;
        SerializedProperty _pointMaterial;
        SerializedProperty _diskMaterial;

        void OnEnable()
        {
            _sourceData = serializedObject.FindProperty("_sourceData");
            _pointTint = serializedObject.FindProperty("_pointTint");
            _pointSize = serializedObject.FindProperty("_pointSize");
            _pointShader = serializedObject.FindProperty("_pointShader");
            _diskShader = serializedObject.FindProperty("_diskShader");
            _pointMaterial = serializedObject.FindProperty("_pointMaterial");
            _diskMaterial = serializedObject.FindProperty("_diskMaterial");
        }

        public override void OnInspectorGUI()
        {
            serializedObject.Update();

            EditorGUILayout.PropertyField(_sourceData);
            EditorGUILayout.PropertyField(_pointTint);
            EditorGUILayout.PropertyField(_pointSize);
            EditorGUILayout.PropertyField(_pointShader);
            EditorGUILayout.PropertyField(_diskShader);
            EditorGUILayout.PropertyField(_pointMaterial);
            EditorGUILayout.PropertyField(_diskMaterial);
            serializedObject.ApplyModifiedProperties();
        }
    }
}
