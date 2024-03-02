using Godot;
using System;

public partial class simulation_gui : Node2D
{
	private GridBLAS grid;
	private Image level_image;
	private int sampleScale = 5;
	public override void _Ready()
	{
		
		grid = new GridBLAS(200,200,0.01f);
		level_image = Image.Create(200,200,false,Image.Format.Rgb8);
		Step();
		// ImageFromSet(grid.pressure,grid.u_vel_hold,grid.v_vel_hold);
		// UpdateImage();
	}

	public void OnClicked(Godot.Vector2 pos) {
		grid.PrintData(pos.X,pos.Y);
	}

	public void Step() {
		ulong start_time = Time.GetTicksUsec();
		grid.Step();
		QueueRedraw();
		ulong end_time = Time.GetTicksUsec();
		GD.Print($"Time Spent: {(end_time - start_time)} micro-seconds");
		// ImageFromSet(grid.pressure,grid.u_vel,grid.v_vel);
		// UpdateImage();	
	}

	public void StepTen() {
		for (int i = 0; i < 10; i++) {
			grid.Step();
		}
		QueueRedraw();
		// ImageFromSet(grid.pressure,grid.u_vel,grid.v_vel);
		// UpdateImage();	
	}

	public void ImageFromSet(double[,] red,float[,] green, float[,] blue)
	{
		for (int i = 0; i < red.GetLength(0); i++)
		{
			for (int j = 0; j < red.GetLength(1); j++)
			{
				level_image.SetPixel(i, j, new Color(0,Mathf.Pow(green[i,j],0.3f),Mathf.Pow(blue[i,j],0.3f)));
			}
		}
	}

	public override void _Draw()
    {
		ulong time = Time.GetTicksUsec();
		DrawRect(new Rect2(0,0,grid.width,grid.height),Colors.White);
		for (int i = 0; i < grid.width; i++)
		{
			for (int j = 0; j < grid.height; j++)
			{
				if (grid.GetMaterialType(i,j) == global::Material.SOLID) {
					DrawRect(new Rect2(i-0.5f,j-0.5f,1,1), Colors.SlateGray);
				}
			}
		}

        for (int i = 0; i < grid.width; i+=sampleScale)
		{
			for (int j = 0; j < grid.height; j+=sampleScale)
			{
				Godot.Vector2 from = new Godot.Vector2(i,j);
				Godot.Vector2 to   = from + new Godot.Vector2(grid.u_vel[i,j],grid.v_vel[i,j]) * sampleScale;
				Color color = new Color(Mathf.Abs(grid.u_vel[i,j]),0,Mathf.Abs(grid.v_vel[i,j]));

				DrawCircle(from, sampleScale / 9f, Colors.Green);
				DrawCircle(to,sampleScale / 8f,color);
				DrawLine(from,to,color,sampleScale / 4f);
			}
		}
		ulong timen = Time.GetTicksUsec();
		GD.Print($"Draw Time: {timen - time} micro-seconds");
    }
	public void UpdateImage()
	{
		(GetNode("Sprite2D") as Sprite2D).Texture = ImageTexture.CreateFromImage(level_image);
	}

	public void UpdateScale(float _scale) {
		sampleScale = (int)_scale;
		QueueRedraw();
	}
}
