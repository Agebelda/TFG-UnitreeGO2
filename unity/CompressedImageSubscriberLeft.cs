using UnityEngine;
using Unity.Robotics.ROSTCPConnector;
using RosMessageTypes.Sensor;
using UnityEngine.UI;
using System;
using System.Collections.Concurrent;
using System.Threading;

public class CompressedImageSubscriberLeft : MonoBehaviour
{
    public RawImage targetRawImage;

    private ROSConnection ros;

    // Cola de imágenes en bruto (byte[])
    private ConcurrentQueue<byte[]> imageDataQueue = new ConcurrentQueue<byte[]>();

    void Start()
    {
        ros = ROSConnection.GetOrCreateInstance();
        ros.Subscribe<CompressedImageMsg>("/stereo_virtual/left/out/compressed", ReceiveImage);
    }

    void Update()
    {
        // Aplicar textura si hay imagen pendiente
        if (imageDataQueue.TryDequeue(out byte[] rawData))
        {
            try
            {
                Texture2D tex = new Texture2D(2, 2, TextureFormat.RGB24, false);
                tex.LoadImage(rawData);
                tex.Apply();

                if (targetRawImage != null)
                {
                    targetRawImage.texture = tex;
                }
            }
            catch (Exception e)
            {
                Debug.LogError("Error al aplicar imagen en Update: " + e.Message);
            }
        }
    }

    void ReceiveImage(CompressedImageMsg msg)
    {
        // Decodificar en segundo plano (pero solo el buffer)
        ThreadPool.QueueUserWorkItem(_ =>
        {
            try
            {
                // Verificamos que no haya muchas imágenes acumuladas
                if (imageDataQueue.Count < 2)
                {
                    imageDataQueue.Enqueue(msg.data);
                }
                else
                {
                    // Descarta imagen si se está saturando
                    Debug.LogWarning("Descartando imagen comprimida por sobrecarga");
                }
            }
            catch (Exception e)
            {
                Debug.LogError("Error en ReceiveImage: " + e.Message);
            }
        });
    }
}