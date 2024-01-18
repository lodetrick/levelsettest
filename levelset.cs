using System;
using System.Linq;

public class LevelSet {
    private float[,] distance_field;
    private int width, height;

    public LevelSet(int width, int height) {
        this.width = width;
        this.height = height;
        distance_field = new float[width,height];
        Clear();
    }

    public void Clear() {
        for (int i = 0; i < width; i++) {
            for (int j = 0; j < height; j++) {
                distance_field[i,j] = int.MaxValue;
            }
        }
    }

    public void AddBox(float x0, float y0, float x1, float y1) {
        for (int i = 0; i < width; i++) {
            for (int j = 0; j < height; j++) {
                // if i,j is in box
                if (x0 < i && i < x1 && y0 < j && j < y1) {
                    if (distance_field[i,j] < 0) {
                        distance_field[i,j] = System.Math.Max(distance_field[i,j],new float[] { x0-i, i-x1, y0-j, j-y1 }.Max());
                    }
                    else {
                        distance_field[i,j] = new float[] { x0-i, i-x1, y0-j, j-y1 }.Max();
                    }
                }
                else {
                    float p,q;
                    if (i < x0) {
                        p = x0;
                    } else if (i > x1) {
                        p = x1;
                    }
                    else {
                        p = (float)(i);
                    }
                    if (j < y0) {
                        q = y0;
                    } else if (i > y1) {
                        q = y1;
                    }
                    else {
                        q = (float)(j);
                    }
                    distance_field[i,j] = (float)Math.Min(Math.Sqrt((i-p)*(i-p) + (j-q)*(j-q)),distance_field[i,j]);
                }
            }
        }
    }

    public Grid.Vector2 FindClosest(Grid.Vector2 position) {
        return new Grid.Vector2(0,0);
    }

    public Grid.Vector2 FindClosest(float x, float y) {
        return new Grid.Vector2(0,0);
    }
}