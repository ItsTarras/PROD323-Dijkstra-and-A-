using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class ScrollingWater : MonoBehaviour
{
    // Start is called before the first frame update
    public float scrollSpeedX = 0.1f;
    public float scrollSpeedY = 0.1f;
    private MeshRenderer meshRenderer;

    void Start()
    {
        meshRenderer = GetComponent<MeshRenderer>();    
    }

    // Update is called once per frame
    void Update()
    {
        //Updates the texutre based on the time.
        meshRenderer.material.mainTextureOffset = new Vector2(Time.realtimeSinceStartup * scrollSpeedX, Time.realtimeSinceStartup * scrollSpeedY);
    }
}
