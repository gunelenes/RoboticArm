using dynamixel_sdk;
using System.ComponentModel;
using System.Numerics;
using System.Security.Cryptography.Xml;

namespace Robotic_Arm
{
    public partial class Form1 : Form
    {
        public bool controller = true;
        public int index = 0;
        // Control table address
        public const int ADDR_MX_TORQUE_ENABLE = 24;                  // Control table address is different in Dynamixel model
        public const int ADDR_MX_GOAL_POSITION = 30;
        public const int ADDR_MX_PRESENT_POSITION = 36;

        // Protocol version
        public const int PROTOCOL_VERSION = 1;                   // See which protocol version is used in the Dynamixel


        // Data Byte Length
        public const int LEN_MX_GOAL_POSITION = 2;
        public const int LEN_MX_PRESENT_POSITION = 2;

        // Default setting
        public const int DXL_ID = 1;                   // Dynamixel ID: 1
        public const int BAUDRATE = 1000000;
        public const string DEVICENAME = "COM5";              // Check which port is being used on your controller
                                                              // ex) Windows: "COM1"   Linux: "/dev/ttyUSB0" Mac: "/dev/tty.usbserial-*"
        public const int MOVING_SPEED = 32;
        public const int TORQUE_LIMITE = 34;
        public const int TORQUE_ENABLE = 1;                   // Value for enabling the torque
        public const int TORQUE_DISABLE = 0;                   // Value for disabling the torque
        public const int DXL_MINIMUM_POSITION_VALUE = 100;                 // Dynamixel will rotate between this value
        public const int DXL_MAXIMUM_POSITION_VALUE = 400;                // and this value (note that the Dynamixel would not move when the position value is out of movable range. Check e-manual about the range of the Dynamixel you use.)
        public const int DXL_MOVING_STATUS_THRESHOLD = 10;                  // Dynamixel moving status threshold

        public const byte ESC_ASCII_VALUE = 0x1b;

        public const int COMM_SUCCESS = 0;                   // Communication Success result value
        public const int COMM_TX_FAIL = -1001;               // Communication Tx Failed

        int port_num = dynamixel.portHandler(DEVICENAME);
        public Form1()
        {
            InitializeComponent();
        }

        private void hScrollBar1_Scroll(object sender, ScrollEventArgs e)
        {
            label3.Text = hScrollBar1.Value.ToString();
        }

        private void hScrollBar2_Scroll(object sender, ScrollEventArgs e)
        {
            label4.Text = hScrollBar2.Value.ToString();
        }

        private void hScrollBar3_Scroll(object sender, ScrollEventArgs e)
        {
            label5.Text = hScrollBar3.Value.ToString();

        }

        private void hScrollBar4_Scroll(object sender, ScrollEventArgs e)
        {

            label6.Text = hScrollBar4.Value.ToString();
        }

        private void hScrollBar8_Scroll(object sender, ScrollEventArgs e)
        {

            label7.Text = hScrollBar8.Value.ToString();
        }

        private void hScrollBar7_Scroll(object sender, ScrollEventArgs e)
        {

            label8.Text = hScrollBar7.Value.ToString();
        }

        private void hScrollBar6_Scroll(object sender, ScrollEventArgs e)
        {

            label9.Text = hScrollBar6.Value.ToString();
        }

        private void hScrollBar5_Scroll(object sender, ScrollEventArgs e)
        {

            label10.Text = hScrollBar5.Value.ToString();
        }

        public void Torque_and_Speed()
        {
            dynamixel.write2ByteTxRx(port_num, PROTOCOL_VERSION, 1, MOVING_SPEED, ushort.Parse(label7.Text));
            dynamixel.write2ByteTxRx(port_num, PROTOCOL_VERSION, 2, MOVING_SPEED, ushort.Parse(label8.Text));
            dynamixel.write2ByteTxRx(port_num, PROTOCOL_VERSION, 3, MOVING_SPEED, ushort.Parse(label9.Text));
            dynamixel.write2ByteTxRx(port_num, PROTOCOL_VERSION, 4, MOVING_SPEED, ushort.Parse(label10.Text));


            dynamixel.write2ByteTxRx(port_num, PROTOCOL_VERSION, 1, TORQUE_LIMITE, ushort.Parse(label3.Text));
            dynamixel.write2ByteTxRx(port_num, PROTOCOL_VERSION, 2, TORQUE_LIMITE, ushort.Parse(label4.Text));
            dynamixel.write2ByteTxRx(port_num, PROTOCOL_VERSION, 3, TORQUE_LIMITE, ushort.Parse(label5.Text));
            dynamixel.write2ByteTxRx(port_num, PROTOCOL_VERSION, 4, TORQUE_LIMITE, ushort.Parse(label6.Text));
        }
        private void button1_Click(object sender, EventArgs e) // start position
        {


            // Initialize PacketHandler Structs
            dynamixel.packetHandler();

            index = (index == 0) ? 1 : 0;
            int dxl_comm_result = COMM_TX_FAIL;                                   // Communication result
            UInt16[] dxl_goal_position = new UInt16[2] { DXL_MINIMUM_POSITION_VALUE, DXL_MAXIMUM_POSITION_VALUE };         // Goal position

            byte dxl_error = 0;                                                   // Dynamixel error
            UInt16 dxl_present_position = 0;                                      // Present position
            dynamixel.openPort(port_num);
            dynamixel.setBaudRate(port_num, BAUDRATE);




            // Write goal position

            Torque_and_Speed();

            drawToPoint(508, 522, 480, 497);


            Thread.Sleep(2000);


            // Close port
            dynamixel.closePort(port_num);

            return;
            //MessageBox.Show("Ýþlem baþladý. Ýþlemi durdurmak için 'a' tuþuna basýn.");

        }

        private void button2_Click(object sender, EventArgs e) // square
        {

            dynamixel.packetHandler();

            index = (index == 0) ? 1 : 0;
            int dxl_comm_result = COMM_TX_FAIL;                                   // Communication result
            UInt16[] dxl_goal_position = new UInt16[2] { DXL_MINIMUM_POSITION_VALUE, DXL_MAXIMUM_POSITION_VALUE };         // Goal position
            int group_num = dynamixel.groupSyncWrite(port_num, PROTOCOL_VERSION, ADDR_MX_GOAL_POSITION, LEN_MX_GOAL_POSITION);
            byte dxl_error = 0;                                                   // Dynamixel error
            UInt16 dxl_present_position = 0;                                      // Present position
            dynamixel.openPort(port_num);
            dynamixel.setBaudRate(port_num, BAUDRATE);


            Torque_and_Speed();

            CreateAnglePointMap();

            drawQueue = new Queue<anglePoint>();

            for (int i = 0; i < 10; i++)
            {
                drawQueue.Enqueue(anglePointMap[0, i]);
            }
            for (int i = 0; i < 10; i++)
            {
                drawQueue.Enqueue(anglePointMap[i, 9]);
            }
            for (int i = 0; i < 10; i++)
            {
                drawQueue.Enqueue(anglePointMap[9, 9 - i]);
            }
            for (int i = 0; i < 10; i++)
            {
                drawQueue.Enqueue(anglePointMap[9 - i, 0]);
            }

            drawToPointSt(drawQueue);

            dynamixel.closePort(port_num);

            return;

        }

        private void button3_Click(object sender, EventArgs e) // rectangle
        {
            dynamixel.packetHandler();

            index = (index == 0) ? 1 : 0;
            int dxl_comm_result = COMM_TX_FAIL;                                   // Communication result
            UInt16[] dxl_goal_position = new UInt16[2] { DXL_MINIMUM_POSITION_VALUE, DXL_MAXIMUM_POSITION_VALUE };         // Goal position
            int group_num = dynamixel.groupSyncWrite(port_num, PROTOCOL_VERSION, ADDR_MX_GOAL_POSITION, LEN_MX_GOAL_POSITION);
            byte dxl_error = 0;                                                   // Dynamixel error
            UInt16 dxl_present_position = 0;                                      // Present position
            dynamixel.openPort(port_num);
            dynamixel.setBaudRate(port_num, BAUDRATE);


            Torque_and_Speed();

            /*

            //1
            drawToPoint(478, 388, 541, 360);

            //2            
            drawToPoint(514, 396, 561, 375);

            //3            
            drawToPoint(551, 395, 556, 375);

            //9            
            drawToPoint(576, 388, 540, 637);

            //10
            drawToPoint(588, 431, 637, 337);

            //11
            drawToPoint(606, 451, 717, 276);

            //5
            drawToPoint(566, 440, 720, 218);

            //6
            drawToPoint(510, 440, 735, 211);

            //7
            drawToPoint(450, 439, 718, 234);

            //8
            drawToPoint(469, 424, 637, 310);

            //1
            drawToPoint(478, 388, 541, 360);
            */

            CreateAnglePointMap();

            drawQueue = new Queue<anglePoint>();

            for (int i = 0; i < 10; i++)
            {
                drawQueue.Enqueue(anglePointMap[0,i]);
            }
            for (int i = 0; i < 5; i++)
            {
                drawQueue.Enqueue(anglePointMap[i, 9]);
            }
            for (int i = 0; i < 10; i++)
            {
                drawQueue.Enqueue(anglePointMap[5, 9-i]);
            }
            for (int i = 0; i < 5; i++)
            {
                drawQueue.Enqueue(anglePointMap[5-i, 0]);
            }


            drawToPointSt(drawQueue);

            dynamixel.closePort(port_num);
            return;

        }

        private void button4_Click(object sender, EventArgs e) // star
        {

            dynamixel.packetHandler();

            index = (index == 0) ? 1 : 0;
            int dxl_comm_result = COMM_TX_FAIL;                                   // Communication result
            UInt16[] dxl_goal_position = new UInt16[2] { DXL_MINIMUM_POSITION_VALUE, DXL_MAXIMUM_POSITION_VALUE };         // Goal position
            int group_num = dynamixel.groupSyncWrite(port_num, PROTOCOL_VERSION, ADDR_MX_GOAL_POSITION, LEN_MX_GOAL_POSITION);
            byte dxl_error = 0;                                                   // Dynamixel error
            UInt16 dxl_present_position = 0;                                      // Present position
            dynamixel.openPort(port_num);
            dynamixel.setBaudRate(port_num, BAUDRATE);


            Torque_and_Speed();

            CreateAnglePointMap();

            drawQueue = new Queue<anglePoint>();

            drawQueue.Enqueue(anglePointMap[4,1]);
            drawQueue.Enqueue(anglePointMap[4, 3]);
            drawQueue.Enqueue(anglePointMap[2, 4]);
            drawQueue.Enqueue(anglePointMap[4, 5]);
            drawQueue.Enqueue(anglePointMap[4, 6]);
            drawQueue.Enqueue(anglePointMap[4, 7]);
            drawQueue.Enqueue(anglePointMap[5, 6]);
            drawQueue.Enqueue(anglePointMap[6, 5]);
            drawQueue.Enqueue(anglePointMap[8, 6]);
            drawQueue.Enqueue(anglePointMap[7, 4]);
            drawQueue.Enqueue(anglePointMap[8, 2]);
            drawQueue.Enqueue(anglePointMap[6, 3]);
            drawQueue.Enqueue(anglePointMap[5, 2]);
            drawQueue.Enqueue(anglePointMap[4, 1]);

            drawToPointSt(drawQueue);

            dynamixel.closePort(port_num);
        }

        private void button5_Click(object sender, EventArgs e)
        { // Initialize PacketHandler Structs

        }

        private void drawToPoint(uint m1, uint m2, uint m3, uint m4)
        {
            int group_num = dynamixel.groupSyncWrite(port_num, PROTOCOL_VERSION, ADDR_MX_GOAL_POSITION, LEN_MX_GOAL_POSITION);
            dynamixel.groupSyncWriteAddParam(group_num, 1, m1, LEN_MX_GOAL_POSITION);
            dynamixel.groupSyncWriteAddParam(group_num, 2, m2, LEN_MX_GOAL_POSITION);
            dynamixel.groupSyncWriteAddParam(group_num, 3, m3, LEN_MX_GOAL_POSITION);
            dynamixel.groupSyncWriteAddParam(group_num, 4, m4, LEN_MX_GOAL_POSITION);
            dynamixel.groupSyncWriteTxPacket(group_num);
            Thread.Sleep(1000);
        }

        private void drawToPointSt(Queue<anglePoint> inp )
        {
            while (inp.Count>0)
            {
                anglePoint pt = inp.Dequeue();
                int group_num = dynamixel.groupSyncWrite(port_num, PROTOCOL_VERSION, ADDR_MX_GOAL_POSITION, LEN_MX_GOAL_POSITION);
                dynamixel.groupSyncWriteAddParam(group_num, 1, pt.m1, LEN_MX_GOAL_POSITION);
                dynamixel.groupSyncWriteAddParam(group_num, 2, pt.m2, LEN_MX_GOAL_POSITION);
                dynamixel.groupSyncWriteAddParam(group_num, 3, pt.m3, LEN_MX_GOAL_POSITION);
                dynamixel.groupSyncWriteAddParam(group_num, 4, pt.m4, LEN_MX_GOAL_POSITION);
                dynamixel.groupSyncWriteTxPacket(group_num);
                Thread.Sleep(1000);
            }
        }

        private struct anglePoint
        {
            public uint m1;
            public uint m2;
            public uint m3;
            public uint m4;
        }

        private anglePoint[,] anglePointMap;

        private Queue<anglePoint> drawQueue;

        private void CreateAnglePointMap()
        {
            anglePointMap = new anglePoint[10, 10] {
               { new anglePoint {m1=466,m2=441,m3=695,m4=282 },new anglePoint {m1=478,m2=443,m3=695,m4=282 },new anglePoint {m1=489,m2=445,m3=697,m4=282 },new anglePoint {m1=500,m2=448,m3=702,m4=282 },new anglePoint {m1=511,m2=449,m3=703,m4=282 },new anglePoint {m1=523,m2=449,m3=705,m4=282 },new anglePoint {m1=533,m2=449,m3=705,m4=282 },new anglePoint {m1=546,m2=449,m3=705,m4=282 },new anglePoint {m1=556,m2=449,m3=705,m4=282 },new anglePoint {m1=567,m2=445,m3=702,m4=282 } },
               { new anglePoint {m1=470,m2=440,m3=675,m4=296 },new anglePoint {m1=479,m2=441,m3=678,m4=296 },new anglePoint {m1=491,m2=442,m3=682,m4=296 },new anglePoint {m1=501,m2=443,m3=683,m4=296 },new anglePoint {m1=512,m2=445,m3=685,m4=296 },new anglePoint {m1=523,m2=445,m3=687,m4=296 },new anglePoint {m1=533,m2=445,m3=687,m4=296 },new anglePoint {m1=545,m2=443,m3=686,m4=296 },new anglePoint {m1=554,m2=443,m3=686,m4=296 },new anglePoint {m1=566,m2=442,m3=685,m4=296 } },
               { new anglePoint {m1=474,m2=432,m3=656,m4=318 },new anglePoint {m1=483,m2=433,m3=658,m4=318 },new anglePoint {m1=494,m2=434,m3=659,m4=311 },new anglePoint {m1=503,m2=435,m3=662,m4=309 },new anglePoint {m1=513,m2=435,m3=664,m4=306 },new anglePoint {m1=524,m2=435,m3=666,m4=304 },new anglePoint {m1=534,m2=434,m3=666,m4=302 },new anglePoint {m1=544,m2=434,m3=667,m4=304 },new anglePoint {m1=553,m2=434,m3=666,m4=304 },new anglePoint {m1=564,m2=433,m3=666,m4=304 } },
               { new anglePoint {m1=478,m2=420,m3=639,m4=299 },new anglePoint {m1=485,m2=423,m3=641,m4=299 },new anglePoint {m1=496,m2=424,m3=645,m4=299 },new anglePoint {m1=504,m2=425,m3=646,m4=299 },new anglePoint {m1=515,m2=425,m3=651,m4=299 },new anglePoint {m1=524,m2=425,m3=650,m4=299 },new anglePoint {m1=534,m2=425,m3=651,m4=299 },new anglePoint {m1=544,m2=425,m3=651,m4=299 },new anglePoint {m1=553,m2=425,m3=651,m4=299 },new anglePoint {m1=563,m2=424,m3=649,m4=299 } },
               { new anglePoint {m1=478,m2=419,m3=621,m4=326 },new anglePoint {m1=488,m2=422,m3=621,m4=326 },new anglePoint {m1=497,m2=424,m3=625,m4=326 },new anglePoint {m1=507,m2=424,m3=626,m4=326 },new anglePoint {m1=516,m2=425,m3=628,m4=326 },new anglePoint {m1=525,m2=424,m3=629,m4=326 },new anglePoint {m1=533,m2=425,m3=631,m4=326 },new anglePoint {m1=543,m2=424,m3=631,m4=326 },new anglePoint {m1=550,m2=424,m3=630,m4=326 },new anglePoint {m1=560,m2=424,m3=630,m4=326 } },
               { new anglePoint {m1=481,m2=411,m3=601,m4=335 },new anglePoint {m1=489,m2=415,m3=602,m4=337 },new anglePoint {m1=498,m2=415,m3=602,m4=337 },new anglePoint {m1=506,m2=416,m3=605,m4=337 },new anglePoint {m1=516,m2=416,m3=606,m4=337 },new anglePoint {m1=525,m2=416,m3=606,m4=337 },new anglePoint {m1=533,m2=417,m3=607,m4=337 },new anglePoint {m1=543,m2=416,m3=607,m4=337 },new anglePoint {m1=550,m2=416,m3=607,m4=337 },new anglePoint {m1=560,m2=416,m3=607,m4=337 } },
               { new anglePoint {m1=483,m2=401,m3=584,m4=337 },new anglePoint {m1=491,m2=403,m3=584,m4=337 },new anglePoint {m1=499,m2=406,m3=589,m4=337 },new anglePoint {m1=508,m2=409,m3=590,m4=337 },new anglePoint {m1=516,m2=409,m3=591,m4=337 },new anglePoint {m1=525,m2=410,m3=591,m4=337 },new anglePoint {m1=533,m2=410,m3=591,m4=337 },new anglePoint {m1=542,m2=410,m3=591,m4=337 },new anglePoint {m1=549,m2=410,m3=591,m4=337 },new anglePoint {m1=557,m2=405,m3=591,m4=337 } },
               { new anglePoint {m1=485,m2=392,m3=569,m4=337 },new anglePoint {m1=492,m2=393,m3=569,m4=337 },new anglePoint {m1=500,m2=394,m3=570,m4=337 },new anglePoint {m1=509,m2=394,m3=571,m4=337 },new anglePoint {m1=516,m2=395,m3=572,m4=337 },new anglePoint {m1=525,m2=395,m3=573,m4=337 },new anglePoint {m1=532,m2=395,m3=576,m4=337 },new anglePoint {m1=541,m2=395,m3=576,m4=337 },new anglePoint {m1=548,m2=395,m3=576,m4=337 },new anglePoint {m1=556,m2=394,m3=576, m4=337 } },
               { new anglePoint {m1=486,m2=383,m3=541,m4=352 },new anglePoint {m1=494,m2=383,m3=543,m4=352 },new anglePoint {m1=501,m2=385,m3=546,m4=352 },new anglePoint {m1=509,m2=388,m3=550,m4=352 },new anglePoint {m1=517,m2=388,m3=553,m4=352 },new anglePoint {m1=526,m2=388,m3=553,m4=352 },new anglePoint {m1=532,m2=388,m3=554,m4=352 },new anglePoint {m1=539,m2=388,m3=554,m4=352 },new anglePoint {m1=548,m2=387,m3=554,m4=352 },new anglePoint {m1=555,m2=384,m3=554,m4=352 } },
               { new anglePoint {m1=489,m2=373,m3=516,m4=372 },new anglePoint {m1=496,m2=373,m3=516,m4=368 },new anglePoint {m1=503,m2=375,m3=521,m4=364 },new anglePoint {m1=511,m2=377,m3=527,m4=358 },new anglePoint {m1=517,m2=377,m3=528,m4=358 },new anglePoint {m1=526,m2=377,m3=528,m4=356 },new anglePoint {m1=532,m2=377,m3=530,m4=355 },new anglePoint {m1=539,m2=377,m3=530,m4=355 },new anglePoint {m1=547,m2=377,m3=530,m4=355 },new anglePoint {m1=554,m2=376,m3=530,m4=355 } } };
        }

    }
}