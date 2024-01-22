using Godot;
using System;
using System.Collections.Generic;

public partial class main : Node2D
{
	private HashSet<Vector2I> inputPoints;
	private float[,] gridDistance;
	private Vector2I[,] closestPointIndex;
	private Image level_image;

	public override void _Ready()
	{
		inputPoints = new HashSet<Vector2I>();
		level_image = Image.Create(1024,1024,false,0);
		UpdateImage();
	}

	public override void _Process(double delta)
	{

	}

	public void OnComputeButtonPressed()
	{

	}

	public void ClearPoints()
	{
		inputPoints.Clear();
		BlankImage();
		UpdateImage();
	}

	public void ComputeClosest()
	{	
		GD.Print("Working?");
		int gridHeight = 1024;
		int gridWidth = 1024;
		//instantiate arrays
		gridDistance = new float[gridHeight,gridWidth];
		closestPointIndex = new Vector2I[gridHeight,gridWidth];
		for (int i = 0; i < gridHeight; i++)
		{
			for (int j = 0; j < gridWidth; j++)
			{
				gridDistance[i,j] = 10_000_000;
				closestPointIndex[i,j] = new Vector2I(-1,-1);
			}
		}
		//fill in known values
		foreach (Vector2I point in inputPoints)
		{
			gridDistance[point.X,point.Y] = 0;
			closestPointIndex[point.X,point.Y] = point;
		}

		//fast sweeping method
		FastSweeping(gridHeight,gridWidth);
		
		GD.Print("Computed");
		ImageFromSet(gridDistance);
		UpdateImage();
	}

	private void FastSweeping(int gridHeight, int gridWidth) {
		//+x
		for (int iter = 0; iter < 2; iter++)
		{
			for (int i = 1; i < gridHeight; i++)
			{
				for (int j = 0; j < gridWidth; j++)
				{
					if (closestPointIndex[i-1,j].X != -1)
					{
						float distance = (closestPointIndex[i-1,j] - new Vector2I(i,j)).Length();
						if (distance < gridDistance[i,j])
						{
							gridDistance[i,j] = distance;
							closestPointIndex[i,j] = closestPointIndex[i-1,j];
						}
					}
				}
			}
			
			//+y
			for (int j = 1; j < gridHeight; j++)
			{
				for (int i = 0; i < gridWidth; i++)
				{
					if (closestPointIndex[i,j-1].X != -1)
					{
						float distance = (closestPointIndex[i,j-1] - new Vector2I(i,j)).Length();
						if (distance < gridDistance[i,j])
						{
							gridDistance[i,j] = distance;
							closestPointIndex[i,j] = closestPointIndex[i,j-1];
						}
					}
				}
			}

			//-x
			for (int i = gridHeight - 2; i >= 0; i--)
			{
				for (int j = 0; j < gridWidth; j++)
				{
					if (closestPointIndex[i+1,j].X != -1)
					{
						float distance = (closestPointIndex[i+1,j] - new Vector2I(i,j)).Length();
						if (distance < gridDistance[i,j])
						{
							gridDistance[i,j] = distance;
							closestPointIndex[i,j] = closestPointIndex[i+1,j];
						}
					}
				}
			}

			//-y
			for (int j = gridHeight - 2; j >= 0; j--)
			{
				for (int i = 0; i < gridWidth; i++)
				{
					if (closestPointIndex[i,j+1].X != -1)
					{
						float distance = (closestPointIndex[i,j+1] - new Vector2I(i,j)).Length();
						if (distance < gridDistance[i,j])
						{
							gridDistance[i,j] = distance;
							closestPointIndex[i,j] = closestPointIndex[i,j+1];
							//GD.Print(distance);
						}
					}
				}
			}
		}
	}

	private void CheckNeighbors(int x, int y)
	{
		if (closestPointIndex[x+1,y].X != -1)
		{
			float distance = (closestPointIndex[x+1,y] - new Vector2I(x,y)).Length();
			if (distance < gridDistance[x,y])
			{
				gridDistance[x,y] = distance;
				closestPointIndex[x,y] = closestPointIndex[x+1,y];
			}
		}
	}
	public void BlankImage()
	{
		level_image.Fill(Colors.White);
		foreach (Vector2I point in inputPoints)
		{
			level_image.SetPixel(point.X,point.Y,Colors.Black);
		}
	}

	public void ImageFromSet(float[,] dataset)
	{
		for (int i = 0; i < dataset.GetLength(0); i++)
		{
			for (int j = 0; j < dataset.GetLength(1); j++)
			{
				level_image.SetPixel(i, j, new Color(dataset[i,j] / 600f,0,0));
				if (dataset[i,j] <= 0.001 && dataset[i,j] >= -0.001)
				{
					level_image.SetPixel(i,j,new Color(1,1,1));
				}
			}
		}
	}
	
	public void UpdateImage()
	{
		GD.Print("Updating Image");
		(GetNode("Sprite2D") as Sprite2D).Texture = ImageTexture.CreateFromImage(level_image);
	}

	public void DrawPixel(Vector2I position)
	{
		if (0 < position.X && position.X < 1024 && 0 < position.Y && position.Y < 1024)
		{
			inputPoints.Add(position);
			BlankImage();
			UpdateImage();
		}	
	}
}
