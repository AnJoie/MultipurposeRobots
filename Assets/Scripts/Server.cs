using System;
using System.Net;
using System.Net.Sockets;
using System.Text;
using System.Threading;
using UnityEngine;

namespace Isaac
{
    class Server
    {
        TcpListener Listener; // Объект, принимающий TCP-клиентов

        private int _port;
        
        private Thread _Thread;

        private ScreenShot _imgSource; 
 
        // Запуск сервера
        public Server(int Port, ScreenShot imgSource)
        {
            _port = Port;
            _Thread = null;
            _imgSource = imgSource;
        }
        
        public bool IsRunning { get { return (_Thread != null && _Thread.IsAlive); } }

        public void Start()
        {
            lock (this)
            {
                _Thread = new Thread(ServerThread);
                _Thread.IsBackground = true;
                _Thread.Start();
            }
        }
        

        private void ServerThread()
        {
            Listener = new TcpListener(IPAddress.Any, _port);
            Listener.Start(); // Запускаем его
 
            // В бесконечном цикле
            while (true)
            {
                // Принимаем новых клиентов
                try
                {
                    new Client(Listener.AcceptTcpClient(),_imgSource);
                }
                catch (Exception e)
                {
                    Debug.Log(e);
                }
                
            }
        }
        
        public void Stop()
        {

            if (this.IsRunning)
            {
                try
                {
                    _Thread.Join();
                    _Thread.Abort();
                }
                finally
                {
                    if (Listener != null)
                    {
                        // Остановим его
                        Listener.Stop();
                    }
                    _Thread = null;
                }
            }
        }
 
        // Остановка сервера
        ~Server()
        {
            // Если "слушатель" был создан
            if (Listener != null)
            {
                // Остановим его
                Listener.Stop();
            }
        }
 
        
    }
    
    class Client
    {
        // Конструктор класса. Ему нужно передавать принятого клиента от TcpListener
        public Client(TcpClient Client,ScreenShot imageSource)
        {
            // Код простой HTML-странички
            string Html = "<html><body><h1>Я пушишка для убийств</h1></body></html>";
            // Необходимые заголовки: ответ сервера, тип и длина содержимого. После двух пустых строк - само содержимое

            string header = "HTTP/1.1 200 OK\r\n" +
                            "Content-Type: multipart/x-mixed-replace; boundary=--boundary" +
                            "\r\n";
            // string Str = "HTTP/1.1 200 OK\nContent-type: text/html\nContent-Length:" + Html.Length.ToString() + "\n\n" + Html;
            // Приведем строку к виду массива байт
            byte[] Buffer = Encoding.ASCII.GetBytes(header);
            int offset = 0;
            // Отправим его клиенту
            Client.GetStream().Write(Buffer,  offset, Buffer.Length);
            Client.GetStream().Flush();
            // offset = Buffer.Length;
            try
            {
                while (true)
                {
                    StringBuilder sb = new StringBuilder();
                    var img = imageSource.CurrentScreenshot;

                    sb.AppendLine();
                    sb.AppendLine("--boundary");
                    sb.AppendLine("Content-Type: image/jpeg");
                    sb.AppendLine("Content-Length: " + img.Length);
                    sb.AppendLine();
                    byte[] data = Encoding.ASCII.GetBytes(sb.ToString());
                    Client.GetStream().Write(data, 0, data.Length);
                    Client.GetStream().Flush();
                    Client.GetStream().Write(img, 0, img.Length);

                    data = Encoding.ASCII.GetBytes("\r\n");
                    Client.GetStream().Write(data, 0, data.Length);
                    Client.GetStream().Flush();
                    Thread.Sleep(30);
                }
            }

            catch(SocketException ex)
            {
                 Client.Close();
            }
            finally
            {
                Client.Close();    
            }
            
        }
    }
}