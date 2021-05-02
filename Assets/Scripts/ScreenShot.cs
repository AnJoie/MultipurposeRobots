﻿using System;
using System.Collections;
using System.Collections.Generic;
using Isaac;
using rtaNetworking.Streaming;
using UnityEngine;
[RequireComponent(typeof(Camera))]
public class ScreenShot : MonoBehaviour
{
    private Camera screenCam;
    private int resWidth = 1024;
    private int resHeight = 1024;

    private static object _locker=new object();

    private byte[] _currentScreenshot;
    public byte[] CurrentScreenshot
    {
        get
        {
            lock (_locker)
            {
                return _currentScreenshot;
            }
        }
        set
        {
            lock (_locker)
            {
                _currentScreenshot = value;
            }
        }
    }

    void Awake()
    {
        screenCam = GetComponent<Camera>();
        if (screenCam.targetTexture == null)
        {
            screenCam.targetTexture = new RenderTexture(resWidth, resHeight, 24);
            
        }
        else
        {
            resWidth = screenCam.targetTexture.width;
            resHeight = screenCam.targetTexture.height;
        }
        screenCam.gameObject.SetActive(false);
    }

    // Start is called before the first frame update
   private  Texture2D snapshot;
    void Start()
    {   
        var server = new Server(8080,this);
        server.Start();
        Debug.Log(server.IsRunning);
        snapshot = new Texture2D(resWidth, resHeight, TextureFormat.RGB24, false);
    }
    

    public void CallTakeScreenShot()
    {
       screenCam.gameObject.SetActive(true);
       
    }


    void LateUpdate()
    {
        if (screenCam.gameObject.activeInHierarchy)
        {
           
            screenCam.Render();
            RenderTexture.active = screenCam.targetTexture;
            snapshot.ReadPixels(new Rect(0,0,resWidth,resHeight),0,0);

            CurrentScreenshot = snapshot.EncodeToJPG();
            screenCam.gameObject.SetActive(false);
            Debug.Log($"{RenderTexture.currentTextureMemory}");
            
            


        }
        
    }

    // string SnapShotName()
    // {
    //     return string.Format("{0}/Snapshots/snap/snap_{1}x{2}_{3}.jpg", Application.dataPath, 256, 256,
    //         System.DateTime.Now.ToString("yy-MM-dd_HH-mm-ss"));
    // }
    
}