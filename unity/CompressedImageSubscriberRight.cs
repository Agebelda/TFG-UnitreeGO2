using UnityEngine;
using Unity.Robotics.ROSTCPConnector;
using RosMessageTypes.Sensor;
using UnityEngine.UI;
using System;

public class CompressedImageSubscriberRight : MonoBehaviour
{
    public RawImage targetRawImage;

    private ROSConnection ros;

    void Start()
    {
        ros = ROSConnection.GetOrCreateInstance();
        ros.Subscribe<CompressedImageMsg>("/stereo_virtual/right/out/compressed", ReceiveImage);
    }

    void ReceiveImage(CompressedImageMsg msg)
    {
        try
        {
            byte[] imageData = msg.data;
            Texture2D tex = new Texture2D(2, 2);
            tex.LoadImage(imageData);
            tex.Apply();

            if (targetRawImage != null)
            {
                targetRawImage.texture = tex;
            }
        }
        catch (Exception e)
        {
            Debug.LogError("Error decoding image: " + e.Message);
        }
    }
}
