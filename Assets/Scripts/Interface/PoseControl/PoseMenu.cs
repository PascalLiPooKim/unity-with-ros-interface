using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class PoseMenu : MonoBehaviour
{
    [Header("Scene")]
    public Transform selectionTransform = null;
    public Transform cursorTransform = null;
    public GameObject selection = null;
    public GameObject activeSelection = null;
    public GameObject cursor = null;

    [Header("Events")]
    public RadialSection top = null;
    public RadialSection bottom = null;

    private Vector2 touchPosition = Vector2.zero;
    private List<RadialSection> radialSections = null;
    private RadialSection highlightedSection = null;

    private readonly float degreeIncrement = 180.0f;
    private void CreateandSetupSections()
    {
        radialSections = new List<RadialSection>()
        {
            top,
            bottom
        };
    }

    private void Start()
    {
        CreateandSetupSections();
        //Show(false);
        ShowSelection(false);
        HighlightSection(false);
    }

    public void ShowCursor(bool state)
    {
        cursor.SetActive(state);
    }

    public void Show(bool value)
    {
        gameObject.SetActive(value);
        ShowSelection(false);
    }

    public void ShowSelection(bool value)
    {
        cursor.SetActive(value);
        selection.SetActive(value);
    }

    private void Update()
    {
        Vector2 direction = touchPosition;
        float rotation = GetDegree(direction);

        SetCursorPosition();
        SetSelectionRotation(rotation);
        SetSelectedEvent(rotation);
    }

    private float GetDegree(Vector2 direction)
    {
        float value = Mathf.Atan2(direction.x, direction.y);
        value *= Mathf.Rad2Deg;

        if (value < 0)
            value += 360.0f;

        return value;
    }

    private void SetCursorPosition()
    {
        cursorTransform.localPosition = touchPosition;
    }
    private void SetSelectionRotation(float newRotation)
    {
        float snappedRotation = SnapRotation(newRotation);
        selectionTransform.localEulerAngles = new Vector3(0, 0, -snappedRotation);
        activeSelection.transform.localEulerAngles = selectionTransform.localEulerAngles;
    }

    private float SnapRotation(float rotation)
    {
        return GetNearestIncrement(rotation) * degreeIncrement;
    }

    private int GetNearestIncrement(float rotation)
    {
        return Mathf.RoundToInt(rotation / degreeIncrement);
    }

    private void SetSelectedEvent(float currentRotation)
    {
        int index = GetNearestIncrement(currentRotation);

        if (index == 2)
            index = 0;

        highlightedSection = radialSections[index];
    }

    public void SetTouchPosition(Vector2 newValue)
    {
        touchPosition = newValue;
    }

    public void HighlightSection(bool value)
    {
        activeSelection.SetActive(value);
    }

    public void ActivateHighlightedSection()
    {
        highlightedSection.onPress.Invoke();
    }
}
