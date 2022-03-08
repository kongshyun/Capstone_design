using System;
using System.Collections.Generic;
using System.ComponentModel;
using System.Data;
using System.Drawing;
using System.Linq;
using System.Text;
using System.Threading.Tasks;
using System.Windows.Forms;

namespace Ball
{
    public partial class Form1 : Form
    {
        int home_width = 975;
        int home_height = 745;
        int ball_width = 27;
        int ball_height = 32;
        public Form1()
        {
            InitializeComponent();
        }
        int speed_left = 4, speed_top = 2;

        private void timer1_Tick(object sender, EventArgs e)
        {
            picBall.Left += speed_left;//공 움직이기
            picBall.Top += speed_top;
            if (picBall.Left > 935)
            {
                speed_left = -speed_left;
            }
            if (picBall.Left < 0)
            {
                speed_left = -speed_left;
            }
            if (picBall.Top > 680)
            {
                speed_top = -speed_top;
            }
            if (picBall.Top < 0)
            {
                speed_top = -speed_top;
            }
            if (((picBall.Left > 300) && (picBall.Left < 700)) && ((picBall.Top < 600) && (picBall.Top > 300)))
            {
                Color col = Color.Red;
                picBall.BackColor = col;
            }
            else 
            {Color col = Color.Blue; }
        }
        private void radFast_CheckedChanged(object sender, EventArgs e)
        {
            if (radFast.Checked == true)
            {
                speed_left = speed_left*2;
                speed_top = speed_top*2;
            }
        }
        private void radSlow_CheckedChanged(object sender, EventArgs e)
        {
            if(radSlow.Checked == true)
            {
                speed_left = speed_left/2;
                speed_top = speed_top/2;
            }
        }

    }
}
