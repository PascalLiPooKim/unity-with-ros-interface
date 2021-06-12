using System.Collections;
using System.Collections.Generic;
using TMPro;
using UnityEngine;
using UnityEngine.Events;
using UnityEngine.UI;

public class MenuNodeEvent : UnityEvent<MenuNode> { }
public class MenuNode : MonoBehaviour
{
    public string nodeName;
    public Color color;
    public TextMeshPro textMesh;
    public MenuNodeEvent buttonCallback;
    public List<MenuNode> attatchedObjects;
    public MenuNode parentObject;
    public GameObject nodeObject;
    public bool isActive;
    private GameObject activeLight;
    public MenuOrganiser menuOrganiser;

    void Start()
    {
        NodeSetup();
        menuOrganiser = GameObject.Find("MenuOrganiser").GetComponent<MenuOrganiser>();
        buttonCallback = new MenuNodeEvent();
        buttonCallback.AddListener(menuOrganiser.SetActiveNode);
        SetColour(color);
        SetText(nodeName);

    }

    public virtual void NodeSetup()
    {

    }

    private void OnMouseDown()
    {
        ButtonCallback();
    }

    public void SetColour(Color32 color)
    {
        this.color = color;
        gameObject.transform.GetChild(1).GetComponent<MeshRenderer>().material.color = color;
    }

    public void SetText(string newNodeName)
    {
        nodeName = newNodeName;
        textMesh.text = nodeName;
    }

    public virtual void ButtonCallback()
    {
        Debug.Log("Button Press!");
        buttonCallback.Invoke(this);
    }

    private void Update()
    {
        gameObject.transform.GetChild(2).gameObject.SetActive(isActive);
    }
}
