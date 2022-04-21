using System;
using System.IO;
using System.IO.MemoryMappedFiles;
using System.Text;
using System.Threading;

namespace ConsoleAppCore
{
    class Program
    {
        static void Main(string[] args)
        {
            if(!File.Exists("sharedfile"))
            {
                using (FileStream fs = File.Create("sharedfile"))
                {
                    long offset = fs.Seek(12289, SeekOrigin.Begin);
                    fs.WriteByte(new byte());
                    fs.Close();
                }
            }

            Thread readTh = new Thread(new ThreadStart(Read));
            readTh.IsBackground = true;
            readTh.Start();

            Thread writeTh = new Thread(new ThreadStart(Write));
            writeTh.IsBackground = true;
            writeTh.Start();

            Console.ReadKey();
        }

        public static void Read()
        {
            using (var mmf = MemoryMappedFile.CreateFromFile("sharedfile", FileMode.OpenOrCreate, null))
            {
                using (var stream = mmf.CreateViewStream())
                {
                    while (true)
                    {
                        stream.Position = 0;
                        var data = stream.ReadByte();
                        while (data != -1)
                        {
                            Console.Write((char)data);
                            data = stream.ReadByte();
                        }
                        Thread.Sleep(1000);
                    }
                }
            }
        }

        public static void Write()
        {
            string caseToWhere = "";
            while(true)
            {
                caseToWhere += "002";
                if (caseToWhere.Length == 237)
                {
                    caseToWhere += "777";
                    break;
                }
            }
            using (var mmf = MemoryMappedFile.CreateFromFile("sharedfile", FileMode.OpenOrCreate, null))
            {
                using (var stream = mmf.CreateViewStream())
                {
                    while (true)
                    {
                        stream.Position = 0;
                        byte[] buffer = Encoding.UTF8.GetBytes(caseToWhere);
                        stream.Write(buffer, 0, buffer.Length);
                        Thread.Sleep(1000);
                    }
                }
            }
        }
    }
}
