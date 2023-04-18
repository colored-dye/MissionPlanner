using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;

namespace SimpleExample
{
    public class MyStream : System.IO.Stream
    {
        System.Collections.Concurrent.ConcurrentQueue<byte> queue;

        public MyStream()
        {
            queue = new System.Collections.Concurrent.ConcurrentQueue<byte>();
            CanSeek = true;
            Position = 0;
        }

        public override long Position { get; set; }
        public override bool CanRead { get; }
        public override bool CanSeek { get; }
        public override bool CanWrite { get; }
        private long _Length = 0;
        public override long Length
        {
            get { return queue.Count; }
        }

        public override void Write(byte[] buffer, int offset, int count)
        {
            for (int i = 0; i < count; i++)
            {
                queue.Enqueue(buffer[offset + i]);
            }
        }
        public override int ReadByte()
        {
            byte b;
            if (queue.TryDequeue(out b))
            {
                SetLength(Length - 1);
                return (int)b;
            }
            else
            {
                return -1;
            }
        }
        public override int Read(byte[] b, int off, int cnt)
        {
            int counter = 0;
            while (queue.TryDequeue(out byte c) && counter < cnt)
            {
                b[off + counter++] = c;
            }
            return counter;
        }
        public override long Seek(long l, System.IO.SeekOrigin so)
        {
            switch (so)
            {
                case System.IO.SeekOrigin.Begin:
                    Position = l;
                    break;
                case System.IO.SeekOrigin.Current:
                    Position += l;
                    break;
                case System.IO.SeekOrigin.End:
                    Position = _Length + l;
                    break;
            }
            return l;
        }
        public override void SetLength(long l)
        {
            _Length = l;
        }
        public override void Flush()
        {
            Console.WriteLine("Not implemented");
        }
    }
}
