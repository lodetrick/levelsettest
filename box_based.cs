using Godot;
using System;

public partial class box_based : Node2D
{	
	private Vector2 first_point = Vector2.Up;
	private LevelSet ls;
	private Image level_image;

	private bool circ_ready = false;
	private Vector2 circ_position;
	private float circ_dist0,circ_dist1;
	public override void _Ready()
	{	
		ls = new LevelSet(1024,1024);
		level_image = Image.Create(1024,1024,false,0);
		BlankImage();
		UpdateImage();
	}

	public void ClearPoints()
	{
		first_point = Vector2.Up;
		ls.Clear();
		BlankImage();
		UpdateImage();
	}

	public void ComputeClosest(Vector2 position)
	{	
		//GD.Print("Start:");
		//GD.Print(position);
		if (0 < position.X && position.X < 1024 && 0 < position.Y && position.Y < 1024)
		{
			VectorFuncs.Vector2 closest = ls.FindClosest(position.X,position.Y);
			//GD.Print(new Vector2(closest.x,closest.y));
			(GetNode("Target") as Sprite2D).Show();
			(GetNode("Target") as Sprite2D).Position = new Vector2(closest.x,closest.y);

			circ_position = position;
			circ_dist0 = position.DistanceTo(new Vector2(closest.x,closest.y));
			circ_dist1 = ls.GetDistance(position.X,position.Y);
			circ_ready = true;

			QueueRedraw();
		}
		else {
			(GetNode("Target") as Sprite2D).Hide();
		}
	}

	public override void _Draw() {
		if (circ_ready) {
			DrawArc(circ_position,circ_dist0,0,10,20,Colors.Red  ,5);
			DrawArc(circ_position,circ_dist1,0,10,20,Colors.White,5);
		}
	}
	public void BlankImage()
	{
		level_image.Fill(Colors.White);
	}

	public void ImageFromSet(float[,] dataset)
	{
		for (int i = 0; i < dataset.GetLength(0); i++)
		{
			for (int j = 0; j < dataset.GetLength(1); j++)
			{
				level_image.SetPixel(i, j, new Color((dataset[i,j] + 100f) / 600f,0,0));
				if (dataset[i,j] <= 1 && dataset[i,j] >= -1)
				{
					level_image.SetPixel(i,j,new Color(1,1,1));
				}
			}
		}
	}
	
	public void UpdateImage()
	{
		(GetNode("Sprite2D") as Sprite2D).Texture = ImageTexture.CreateFromImage(level_image);
	}

	public void AddPoint(Vector2 position)
	{
		if (0 < position.X && position.X < 1024 && 0 < position.Y && position.Y < 1024)
		{
			if (first_point.Y < 0) {
				first_point = position;
			}
		}	
	}

	public void AddSecondPoint(Vector2 position) {
		if (0 < position.X && position.X < 1024 && 0 < position.Y && position.Y < 1024)
		{
			if (first_point.Y >= 0) {
				ls.AddBox(first_point.X,first_point.Y,position.X, position.Y);
				first_point = Vector2.Up;
				ImageFromSet(ls.GetField());
				UpdateImage();
			}
		}	
	}
}
