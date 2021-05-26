using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class screeen : MonoBehaviour
{
    // Start is called before the first frame update
    public ScreenShot snapCam;

    // Update is called once per frame
    void Update()
    {
        // if (Input.GetKeyDown(KeyCode.Space))
        // {
            snapCam.CallTakeScreenShot();
        // }

       
        
    }
}
