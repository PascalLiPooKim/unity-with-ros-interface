using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class SelectionTextManager : MonoBehaviour
{
    public GameObject segmentText;
    // Update is called once per frame
    public void InitiateText(List<SelectionSection> selectionSections)
    {
        Debug.Log("Sorting text");
        for(int i=0; i<selectionSections.Count; i++)
        {
            GameObject text = Instantiate(segmentText, gameObject.transform);
            float yPosition = ((float)i / (selectionSections.Count-1) - 0.5f) * 2;
            text.transform.localPosition = new Vector2(0, yPosition);

            text.GetComponent<TextMesh>().text = selectionSections[i].SectionName;
            text.GetComponent<TextMesh>().fontSize = 125;
        }
    }
}
