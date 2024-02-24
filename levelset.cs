using System;
using System.Linq;
using static Godot.GD;
using static VectorFuncs;

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
                distance_field[i,j] = float.MaxValue;
            }
        }
    }

    public float[,] GetField() {
        return distance_field;
    }

    public float GetDistance(Vector2 position) {
        return VectorFuncs.LerpFloatField(position,distance_field);
    }

    public float GetDistance(float x, float y) {
        return VectorFuncs.LerpFloatField(x,y,distance_field);
    }

    public float GetDistanceOnGrid(int x, int y) {
        if (x < 0 || y < 0 || x >= width || y >= height) {
            return -1; // in solid
        }
        return distance_field[x,y];
    }

    public void AddBox(float x0, float y0, float x1, float y1, float sign = 1f) {
        UnionBox(Math.Min(x0,x1),Math.Min(y0,y1),Math.Max(x0,x1),Math.Max(y0,y1),sign);
    }

    public void UnionBox(float x0, float y0, float x1, float y1, float sign = 1f) {
        for (int i = 0; i < width; i++) {
            for (int j = 0; j < height; j++) {
                // if i,j is in box
                if (x0 < i && i < x1 && y0 < j && j < y1) {
                    distance_field[i,j] = sign * new float[] { x0-i, i-x1, y0-j, j-y1, -Math.Abs(distance_field[i,j])}.Max();
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
                    } else if (j > y1) {
                        q = y1;
                    }
                    else {
                        q = (float)(j);
                    }
                    //make negative if already negative
                    if (distance_field[i,j] < 0 || sign < 0) {
                        distance_field[i,j] = (float)Math.Max(-1.0f * Math.Sqrt((i-p)*(i-p) + (j-q)*(j-q)),-Math.Abs(distance_field[i,j]));
                    }
                    else {
                        distance_field[i,j] = (float)Math.Min(Math.Sqrt((i-p)*(i-p) + (j-q)*(j-q)),distance_field[i,j]);
                    }
                }
            }
        }
    }

    public Vector2 FindClosest(Vector2 position) {
        int N = 3;
        int M = 3;
        float epsilon = 0.00001f;

        Vector2 p = position;
        p.Constrain(0,0,width-1,height-1,1);
        float distance = VectorFuncs.LerpFloatField(p,distance_field);
        Vector2 dir = VectorFuncs.GradientFloatField(p,distance_field,1);

        if (Math.Abs(distance) < epsilon) {
            return p;
        }

        for (int i = 0; i < N; i++) {
            float alpha = 1;
            for (int j = 0; j < M; j++) {
                Vector2 q = p - alpha * distance * dir;
                float q_distance = VectorFuncs.LerpFloatField(q, distance_field);
                if (Math.Abs(q_distance) < Math.Abs(distance)) {
                    p = q;
                    distance = q_distance;
                    dir = VectorFuncs.GradientFloatField(q,distance_field,1);
                    if (Math.Abs(q_distance) < epsilon) {
                        return p;
                    }
                }
                else {
                    alpha *= 0.7f;
                }
            }
        }
        return p;
    }

    public Vector2 FindClosest(float x, float y) {
        int N = 3;
        int M = 3;
        float epsilon = 0.001f;

        Vector2 p = new Vector2(x,y);
        float distance = VectorFuncs.LerpFloatField(p, distance_field);
        Vector2 dir = VectorFuncs.GradientFloatField(p,distance_field,1);

        if (Math.Abs(distance) < epsilon) {
            return p;
        }

        for (int i = 0; i < N; i++) {
            float alpha = 1;
            for (int j = 0; j < M; j++) {
                Vector2 q = p - alpha * distance * dir;
                float q_distance = VectorFuncs.LerpFloatField(q,distance_field);
                if (Math.Abs(q_distance) < Math.Abs(distance)) {
                    p = q;
                    distance = q_distance;
                    dir = VectorFuncs.GradientFloatField(q,distance_field,1);
                    if (Math.Abs(q_distance) < epsilon) {
                        return p;
                    }
                }
                else {
                    alpha *= 0.7f;
                }
            }
        }
        return p;
    }
}