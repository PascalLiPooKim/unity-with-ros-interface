using System.Collections;
using System.Collections.Generic;
using UnityEngine;

[RequireComponent(typeof(SelectionInputManager))]

public class SelectionMenu : MonoBehaviour
{
    
    [Header("Scene")]
    private int sections;
    public SelectionTextManager selectionTextManager;
    public GameObject cursor = null;
    public GameObject selection = null;
    public GameObject activeSelection = null;
    public Transform cursorTransform = null;
    public Transform selectionTransform = null;
    public Transform activeTransform = null;

    [Header("Events")]
    [SerializeField] public List<SelectionSection> selectionSections = null;

    private Vector2 touchPosition = Vector2.zero;
    
    private SelectionSection highlightedSection = null;

    // Start is called before the first frame update
    void Start()
    {
        sections = selectionSections.Count;
        scaleSections();
        showCursor(false);
        showSelection(false);
        activeSelection.SetActive(false);
        selectionTextManager.InitiateText(selectionSections);
    }

    void scaleSections()
    {
        selection.transform.localScale = new Vector3(2, (float)2 / sections, 1);
        activeSelection.transform.localScale = new Vector3(2, (float)2 / sections, 1);
    }

    void Update()
    {
        SetCursorPosition();
        int selectionIndex = SetSelection();
        SetSelectedEvent(selectionIndex);
    }

    public void showCursor(bool state)
    {
        cursor.SetActive(state);
    }

    public void showSelection(bool state)
    {
        selection.SetActive(state);
    }

    private void SetCursorPosition()
    {
        cursorTransform.localPosition = touchPosition;
    }

    private int SetSelection()  
    {
        int selectionIndex = Mathf.RoundToInt(((touchPosition.y + 1) / 2) * (sections-1));
        float sectionLength = 2 / (float)sections;
        double yTransform   = -(sections-1)/(float)sections + (sectionLength * selectionIndex);
        selectionTransform.localPosition = new Vector3(0, (float)yTransform, 0);
        return selectionIndex;
    }

    private void SetSelectedEvent(int selectionIndex)
    {
        highlightedSection = selectionSections[selectionIndex];
    }

    public void SetTouchPosition(Vector2 newValue)
    {
        touchPosition = newValue;
    }

    public void OnPressDown()
    {
        highlightedSection.onPressDown.Invoke();
        activeSelection.SetActive(true);
        activeTransform.localPosition = selectionTransform.localPosition;
    }

    public void OnPress()
    {   
        if(highlightedSection != null)
            highlightedSection.onPress.Invoke();
    }

    public void OnPressRelease()
    {
        if (highlightedSection != null)
            highlightedSection.onPressRelease.Invoke();
    }
}
