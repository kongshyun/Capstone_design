namespace Ball
{
    partial class Form1
    {
        /// <summary>
        /// 필수 디자이너 변수입니다.
        /// </summary>
        private System.ComponentModel.IContainer components = null;

        /// <summary>
        /// 사용 중인 모든 리소스를 정리합니다.
        /// </summary>
        /// <param name="disposing">관리되는 리소스를 삭제해야 하면 true이고, 그렇지 않으면 false입니다.</param>
        protected override void Dispose(bool disposing)
        {
            if (disposing && (components != null))
            {
                components.Dispose();
            }
            base.Dispose(disposing);
        }

        #region Windows Form 디자이너에서 생성한 코드

        /// <summary>
        /// 디자이너 지원에 필요한 메서드입니다. 
        /// 이 메서드의 내용을 코드 편집기로 수정하지 마세요.
        /// </summary>
        private void InitializeComponent()
        {
            this.components = new System.ComponentModel.Container();
            this.picBall = new System.Windows.Forms.PictureBox();
            this.timer1 = new System.Windows.Forms.Timer(this.components);
            this.radFast = new System.Windows.Forms.RadioButton();
            this.radSlow = new System.Windows.Forms.RadioButton();
            ((System.ComponentModel.ISupportInitialize)(this.picBall)).BeginInit();
            this.SuspendLayout();
            // 
            // picBall
            // 
            this.picBall.BackColor = System.Drawing.Color.Blue;
            this.picBall.Location = new System.Drawing.Point(112, 87);
            this.picBall.Name = "picBall";
            this.picBall.Size = new System.Drawing.Size(27, 32);
            this.picBall.TabIndex = 0;
            this.picBall.TabStop = false;
            // 
            // timer1
            // 
            this.timer1.Enabled = true;
            this.timer1.Interval = 10;
            this.timer1.Tick += new System.EventHandler(this.timer1_Tick);
            // 
            // radFast
            // 
            this.radFast.AutoSize = true;
            this.radFast.Checked = true;
            this.radFast.Location = new System.Drawing.Point(857, 53);
            this.radFast.Name = "radFast";
            this.radFast.Size = new System.Drawing.Size(47, 16);
            this.radFast.TabIndex = 1;
            this.radFast.TabStop = true;
            this.radFast.Text = "Fast";
            this.radFast.UseVisualStyleBackColor = true;
            this.radFast.CheckedChanged += new System.EventHandler(this.radFast_CheckedChanged);
            // 
            // radSlow
            // 
            this.radSlow.AutoSize = true;
            this.radSlow.Location = new System.Drawing.Point(857, 87);
            this.radSlow.Name = "radSlow";
            this.radSlow.Size = new System.Drawing.Size(51, 16);
            this.radSlow.TabIndex = 2;
            this.radSlow.Text = "Slow";
            this.radSlow.UseVisualStyleBackColor = true;
            this.radSlow.CheckedChanged += new System.EventHandler(this.radSlow_CheckedChanged);
            // 
            // Form1
            // 
            this.AutoScaleDimensions = new System.Drawing.SizeF(7F, 12F);
            this.AutoScaleMode = System.Windows.Forms.AutoScaleMode.Font;
            this.ClientSize = new System.Drawing.Size(959, 706);
            this.Controls.Add(this.radSlow);
            this.Controls.Add(this.radFast);
            this.Controls.Add(this.picBall);
            this.Name = "Form1";
            this.Text = "Form1";
            ((System.ComponentModel.ISupportInitialize)(this.picBall)).EndInit();
            this.ResumeLayout(false);
            this.PerformLayout();

        }

        #endregion

        private System.Windows.Forms.PictureBox picBall;
        private System.Windows.Forms.Timer timer1;
        private System.Windows.Forms.RadioButton radFast;
        private System.Windows.Forms.RadioButton radSlow;
    }
}

