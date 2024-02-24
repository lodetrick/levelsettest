using Godot;
using System;

public partial class simulation_gui : Node2D
{
	private Grid grid;
	private Image level_image;
	public override void _Ready()
	{
		grid = new Grid(200,200,0.01f);
		level_image = Image.Create(200,200,false,Image.Format.Rgb8);
		ImageFromSet(grid.pressure,grid.u_vel_hold,grid.v_vel_hold);
		UpdateImage();
	}

	public void OnClicked(Vector2 pos) {
		grid.PrintData(pos.X,pos.Y);
	}

	public void Step() {
		grid.Step();
		ImageFromSet(grid.pressure,grid.u_vel,grid.v_vel);
		UpdateImage();	
	}

	public void StepTen() {
		for (int i = 0; i < 10; i++) {
			grid.Step();
		}
		ImageFromSet(grid.pressure,grid.u_vel,grid.v_vel);
		UpdateImage();	
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
	public void UpdateImage()
	{
		(GetNode("Sprite2D") as Sprite2D).Texture = ImageTexture.CreateFromImage(level_image);
	}
}
