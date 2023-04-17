using System;
using System.Collections.Generic;
using System.ComponentModel;
using System.Data;
using System.Drawing;
using System.IO.Ports;
using System.Linq;
using System.Text;
using System.Threading.Tasks;
using System.Windows.Forms;


namespace SimpleExample
{
    public partial class simpleexample : Form
    {
        MAVLink.MavlinkParse mavlink = new MAVLink.MavlinkParse();
        bool armed = false;
        // locking to prevent multiple reads on serial port
        object readlock = new object();
        // our target sysid
        byte sysid;
        // our target compid
        byte compid;

        GEC gec = new GEC();
        Deque.Deque<byte> receive_deque = new Deque.Deque<byte>(4096);
        //System.Collections.Concurrent.ConcurrentQueue<byte> receive_queue = new System.Collections.Concurrent.ConcurrentQueue<byte>();
        //System.Collections.Concurrent.ConcurrentQueue<byte> plaintext_queue = new System.Collections.Concurrent.ConcurrentQueue<byte>();

        //System.IO.MemoryStream plaintext_stream = new System.IO.MemoryStream();
        //System.IO.MemoryStream ciphertext_stream = new System.IO.MemoryStream();

        public simpleexample()
        {
            InitializeComponent();
        }

        private void but_connect_Click(object sender, EventArgs e)
        {
            // if the port is open close it
            if (serialPort1.IsOpen)
            {
                serialPort1.Close();
                return;
            }

            // set the comport options
            serialPort1.PortName = CMB_comport.Text;
            serialPort1.BaudRate = int.Parse(cmb_baudrate.Text);

            // open the comport
            serialPort1.Open();

            // set timeout to 2 seconds
            serialPort1.ReadTimeout = 2000;
            //serialPort1.DataReceived += DataReceived;

            BackgroundWorker bgw = new BackgroundWorker();

            bgw.DoWork += bgw_DoWork;
            //bgw.DoWork += bgw_Decrypt;

            bgw.RunWorkerAsync();
        }

        //void DataReceived(object sender, SerialDataReceivedEventArgs e)
        //{
        //    try
        //    {
        //        lock (readlock)
        //        {
        //            var serial_port = (SerialPort)sender;
        //            var bytes_to_read = serialPort1.BytesToRead;
        //            var buffer = new byte[bytes_to_read];
        //            serialPort1.Read(buffer, 0, bytes_to_read);
        //            //ciphertext_stream.Write(buffer, 0, buffer.Length);
        //            foreach (var b in buffer)
        //            {
        //                receive_queue.Enqueue(b);
        //            }
        //        }
        //    }
        //    catch (TimeoutException)
        //    {
        //        Console.WriteLine("Timeout");
        //    }
        //    catch (InvalidOperationException)
        //    {
        //        Console.WriteLine("Invalid Operation Exception");
        //    }
        //}

        //void bgw_Decrypt(object sender, DoWorkEventArgs e)
        //{
        //    while (true)
        //    {
        //        var ct = new GEC.Gec_ciphertext();
        //        var pt = new GEC.Gec_plaintext();
        //        int cnt = 0;
        //        while (true)
        //        {
        //            //if (ciphertext_stream.Length > 0)
        //            //{
        //            //    byte c = (byte)ciphertext_stream.ReadByte();
        //            //    if (c == 0x7e)
        //            //    {
        //            //        while (ciphertext_stream.Length == 0)
        //            //        {

        //            //        }
        //            //        c = (byte)ciphertext_stream.ReadByte();
        //            //        if (c == 0)
        //            //        {
        //            //            break;
        //            //        }
        //            //    }
        //            ////}
        //            byte c;
        //            if (receive_queue.TryDequeue(out c))
        //            {
        //                if (c == 0x7e)
        //                {
        //                    while (!receive_queue.TryDequeue(out c))
        //                    {
        //                    }
        //                    if (c == 0)
        //                    {
        //                        break;
        //                    }
        //                }
        //            }
        //        }
        //        while (cnt < GEC.GEC_CT_LEN)
        //        {
        //            //int len = ciphertext_stream.Read(ct.byte_array, cnt, GEC.GEC_CT_LEN - cnt);
        //            //cnt += len;
        //            byte c;
        //            if (receive_queue.TryDequeue(out c))
        //            {
        //                ct.byte_array[cnt++] = c;
        //            }
        //        }
        //        if (!gec.decrypt(2, ct, pt))
        //        {
        //            Console.WriteLine("Decrypt failed");
        //        }
        //        else
        //        {
        //            lock (readlock)
        //            {
        //                long position = plaintext_stream.Position;
        //                plaintext_stream.Write(pt.byte_array, 0, pt.byte_array.Length);
        //                plaintext_stream.Position = position;
        //            }
        //        }
        //    }
        //}

        void bgw_DoWork(object sender, DoWorkEventArgs e)
        {
            while (serialPort1.IsOpen)
            {
                try
                {
                    MAVLink.MAVLinkMessage packet;
                    var ct = new GEC.Gec_ciphertext();
                    var pt = new GEC.Gec_plaintext();
                    lock (readlock)
                    {
                        int cnt = 0;
                        while (true)
                        {
                            int c;
                            c = serialPort1.ReadByte();
                            if (c != -1)
                            {
                                if ((byte)c == 0x7e)
                                {
                                    while (true)
                                    {
                                        c = serialPort1.ReadByte();
                                        if (c != -1)
                                        {
                                            break;
                                        }
                                    }
                                    if ((byte)c == 0)
                                    {
                                        break;
                                    }
                                }
                            }
                        }
                        while (cnt < GEC.GEC_CT_LEN)
                        {
                            int len = serialPort1.Read(ct.byte_array, cnt, GEC.GEC_CT_LEN - cnt);
                            cnt += len;
                        }
                        if (!gec.decrypt(2, ct, pt))
                        {
                            Console.WriteLine("Decrypt failed");
                            continue;
                        }
                    }

                    foreach (var b in pt.byte_array)
                    {
                        receive_deque.AddToBack(b);
                    }

                    while (true)
                    {
                        try
                        {
                            packet = mavlink.ReadPacketFromQueue(receive_deque);

                            // check its valid
                            if (packet == null || packet.data == null)
                            {
                                Console.WriteLine("========Packet invalid");
                                continue;
                            }

                            // check to see if its a hb packet from the comport
                            if (packet.data.GetType() == typeof(MAVLink.mavlink_heartbeat_t))
                            {
                                Console.WriteLine("HEARTBEAT: ({0}:{1})", packet.sysid, packet.seq);

                                var hb = (MAVLink.mavlink_heartbeat_t)packet.data;

                                // save the sysid and compid of the seen MAV
                                sysid = packet.sysid;
                                compid = packet.compid;
                            }

                            // from here we should check the the message is addressed to us
                            if (sysid != packet.sysid || compid != packet.compid)
                                continue;

                            //Console.WriteLine(packet.msgtypename);

                            if (packet.msgid == (byte)MAVLink.MAVLINK_MSG_ID.ATTITUDE)
                            //or
                            //if (packet.data.GetType() == typeof(MAVLink.mavlink_attitude_t))
                            {
                                var att = (MAVLink.mavlink_attitude_t)packet.data;

                                Console.WriteLine(att.pitch * 57.2958 + " " + att.roll * 57.2958);
                            }
                        }catch
                        {
                            break;
                        }
                    }
                }
                catch
                {
                }

                System.Threading.Thread.Sleep(1);
            }
        }

        T readsomedata<T>(byte sysid, byte compid, int timeout = 2000)
        {
            DateTime deadline = DateTime.Now.AddMilliseconds(timeout);

            lock (readlock)
            {
                // read the current buffered bytes
                while (DateTime.Now < deadline)
                {
                    var packet = mavlink.ReadPacket(serialPort1.BaseStream);

                    // check its not null, and its addressed to us
                    if (packet == null || sysid != packet.sysid || compid != packet.compid)
                        continue;

                    Console.WriteLine(packet);

                    if (packet.data.GetType() == typeof(T))
                    {
                        return (T)packet.data;
                    }
                }
            }

            throw new Exception("No packet match found");
        }

        private void but_armdisarm_Click(object sender, EventArgs e)
        {
            MAVLink.mavlink_command_long_t req = new MAVLink.mavlink_command_long_t();

            req.target_system = 1;
            req.target_component = 1;

            req.command = (ushort)MAVLink.MAV_CMD.COMPONENT_ARM_DISARM;

            req.param1 = armed ? 0 : 1;
            armed = !armed;
            /*
            req.param2 = p2;
            req.param3 = p3;
            req.param4 = p4;
            req.param5 = p5;
            req.param6 = p6;
            req.param7 = p7;
            */

            byte[] packet = mavlink.GenerateMAVLinkPacket10(MAVLink.MAVLINK_MSG_ID.COMMAND_LONG, req);

            serialPort1.Write(packet, 0, packet.Length);

            try
            {
                var ack = readsomedata<MAVLink.mavlink_command_ack_t>(sysid, compid);
                if (ack.result == (byte)MAVLink.MAV_RESULT.ACCEPTED)
                {

                }
            }
            catch
            {
            }
        }

        private void CMB_comport_Click(object sender, EventArgs e)
        {
            CMB_comport.DataSource = SerialPort.GetPortNames();
        }

        private void but_mission_Click(object sender, EventArgs e)
        {
            MAVLink.mavlink_mission_count_t req = new MAVLink.mavlink_mission_count_t();

            req.target_system = 1;
            req.target_component = 1;

            // set wp count
            req.count = 1;

            byte[] packet = mavlink.GenerateMAVLinkPacket10(MAVLink.MAVLINK_MSG_ID.MISSION_COUNT, req);
            Console.WriteLine("MISSION_COUNT send");
            serialPort1.Write(packet, 0, packet.Length);

            var ack = readsomedata<MAVLink.mavlink_mission_request_t>(sysid, compid);
            if (ack.seq == 0)
            {
                MAVLink.mavlink_mission_item_int_t req2 = new MAVLink.mavlink_mission_item_int_t();

                req2.target_system = sysid;
                req2.target_component = compid;

                req2.command = (byte)MAVLink.MAV_CMD.WAYPOINT;

                req2.current = 1;
                req2.autocontinue = 0;

                req2.frame = (byte)MAVLink.MAV_FRAME.GLOBAL_RELATIVE_ALT;

                req2.y = (int)(115 * 1.0e7);
                req2.x = (int)(-35 * 1.0e7);

                req2.z = (float)(2.34);

                req2.param1 = 0;
                req2.param2 = 0;
                req2.param3 = 0;
                req2.param4 = 0;

                req2.seq = 0;

                packet = mavlink.GenerateMAVLinkPacket10(MAVLink.MAVLINK_MSG_ID.MISSION_ITEM_INT, req2);
                Console.WriteLine("MISSION_ITEM_INT send");
                lock (readlock)
                {
                    serialPort1.Write(packet, 0, packet.Length);

                    var ack2 = readsomedata<MAVLink.mavlink_mission_ack_t>(sysid, compid);
                    if ((MAVLink.MAV_MISSION_RESULT)ack2.type != MAVLink.MAV_MISSION_RESULT.MAV_MISSION_ACCEPTED)
                    {

                    }
                }


                MAVLink.mavlink_mission_ack_t req3 = new MAVLink.mavlink_mission_ack_t();
                req3.target_system = 1;
                req3.target_component = 1;
                req3.type = 0;

                packet = mavlink.GenerateMAVLinkPacket10(MAVLink.MAVLINK_MSG_ID.MISSION_ACK, req3);
                Console.WriteLine("MISSION_ACK send");
                serialPort1.Write(packet, 0, packet.Length);
            }
        }
    }
}
