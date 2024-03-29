using NumFlat;
using static System.MathF;
using static Godot.GD;

public class VectorFuncs {
    public struct Vector2 {
        public float x {get;}
        public float y {get;}

        public Vector2(float x, float y) {
            this.x = x;
            this.y = y;
        }

        public static Vector2 operator +(Vector2 a) => a;
        public static Vector2 operator -(Vector2 a) => new Vector2(-a.x,-a.y);

        public static Vector2 operator +(Vector2 a, Vector2 b) => new Vector2(a.x + b.x, a.y + b.y);
        public static Vector2 operator -(Vector2 a, Vector2 b) => new Vector2(a.x - b.x, a.y - b.y);
        public static Vector2 operator *(float a, Vector2 b) => new Vector2(a * b.x, a * b.y);

        public float Length() {return Sqrt(x*x + y*y);}

        public void Print() {
            Godot.GD.Print($"({x}, {y})");
        }

        public Vector2 Constrain(int x0, int y0, int x1, int y1, int padding = 0) {
            float xn = x,yn = y;
            if (x <= x0 + padding) {
                xn = x0 + padding;
            }
            if (y <= y0 + padding) {
                yn = y0 + padding;
            }
            if (x >= x1 - padding) {
                xn = x1 - padding;
            }
            if (y >= y1 - padding) {
                yn = y1 - padding;
            }
            return new Vector2(xn,yn);
        }
    }
    public struct Vector3 {
        public float x {get;}
        public float y {get;}
        public float z {get;}

        public Vector3(float x, float y, float z) {
            this.x = x;
            this.y = y;
            this.z = z;
        }

        public static Vector3 operator +(Vector3 a) => a;
        public static Vector3 operator -(Vector3 a) => new Vector3(-a.x,-a.y,-a.z);

        public static Vector3 operator +(Vector3 a, Vector3 b) => new Vector3(a.x + b.x, a.y + b.y, a.z + b.z);
        public static Vector3 operator -(Vector3 a, Vector3 b) => new Vector3(a.x - b.x, a.y - b.y, a.z - b.z);
        public static Vector3 operator *(float a, Vector3 b) => new Vector3(a * b.x, a * b.y, a * b.z);

        public float Length() {return Sqrt(x*x + y*y + z*z);}
    }
    public static float Lerp(float a, float b, float s) {
        return (1 - s) * a + s * b;
    }

    public static float Cubic(float x0, float x1, float x2, float x3, float s) {
        return  (0.5f * s*s - s / 3.0f - s*s*s / 6.0f) * x0 +
                (1.0f - s*s + 0.5f * (s*s*s - s))      * x1 + 
                (s + 0.5f * (s*s - s*s*s))             * x2 + 
                ((s*s*s - s) / 6.0f)                   * x3;
    }

    public static float CubicFloatField(Vector2 position, float[,] float_field) {
        int i = (int)(position.x);
        int j = (int)(position.y);
        float s_x = position.x - i;
        return Cubic(
            Cubic(float_field[i-1,j-1],float_field[i,j-1],float_field[i+1,j-1],float_field[i+2,j-1],s_x),
            Cubic(float_field[i-1,j  ],float_field[i,j  ],float_field[i+1,j  ],float_field[i+2,j  ],s_x),
            Cubic(float_field[i-1,j+1],float_field[i,j+1],float_field[i+1,j+1],float_field[i+2,j+1],s_x),
            Cubic(float_field[i-1,j+2],float_field[i,j+2],float_field[i+1,j+2],float_field[i+2,j+2],s_x),
            position.y - j
        );
    }

    public static float CubicFloatField(float x, float y, float[,] float_field) {
        int i = (int)(x);
        int j = (int)(y);
        float s_x = x - i;
        return Cubic(
            Cubic(float_field[i-1,j-1],float_field[i,j-1],float_field[i+1,j-1],float_field[i+2,j-1],s_x),
            Cubic(float_field[i-1,j  ],float_field[i,j  ],float_field[i+1,j  ],float_field[i+2,j  ],s_x),
            Cubic(float_field[i-1,j+1],float_field[i,j+1],float_field[i+1,j+1],float_field[i+2,j+1],s_x),
            Cubic(float_field[i-1,j+2],float_field[i,j+2],float_field[i+1,j+2],float_field[i+2,j+2],s_x),
            y - j
        );
    }

    public static float LerpFloatField(Vector2 position,  float[,] float_field) {
        int i = (int)(position.x);
        int j = (int)(position.y);
        float s_x = position.x - i;
        return Lerp(Lerp(float_field[i,j  ],float_field[i+1,j  ],s_x),
                    Lerp(float_field[i,j+1],float_field[i+1,j+1],s_x),
                    position.y - j);
    }

    public static float LerpFloatField(Vector2 position,  Vec<float> float_field, int width, int height) {
        int i = (int)(position.x);
        int j = (int)(position.y);
        float s_x = position.x - i;
        return Lerp(Lerp(float_field[i*width+j  ],float_field[i*width+j+width  ],s_x),
                    Lerp(float_field[i*width+j+1],float_field[i*width+j+1+width],s_x),
                    position.y - j);
    }

    public static float LerpFloatField(float x, float y,  float[,] float_field) {
        int i = (int)(x);
        int j = (int)(y);
        float s_x = x - i;
        return Lerp(Lerp(float_field[i,j  ],float_field[i+1,j  ],s_x),
                    Lerp(float_field[i,j+1],float_field[i+1,j+1],s_x),
                    y - j);
    }

    public static float LerpFloatField(float x, float y,  Vec<float> float_field, int width, int height) {
        int i = (int)(x);
        int j = (int)(y);
        float s_x = x - i;
        return Lerp(Lerp(float_field[i*width+j  ],float_field[i*width+j+width  ],s_x),
                    Lerp(float_field[i*width+j+1],float_field[i*width+width+j+1],s_x),
                    y - j);
    }
    public static Vector2 LerpVelocityField(Vector2 position,  float[,] u_vel,  float[,] v_vel) {
        int i = (int)(position.x + 0.5f);
        int j = (int)(position.y + 0.5f);
        return new Vector2(Lerp(u_vel[i,j],u_vel[i+1,j],position.x - i + 0.5f),
                           Lerp(v_vel[i,j],v_vel[i,j+1],position.y - j + 0.5f));
    }

    public static Vector2 CubicVectorField(Vector2 position, float[,] u_vel, float[,] v_vel) {
        return new Vector2(LerpFloatField(position,u_vel),
                           LerpFloatField(position,v_vel));
    }

    public static float BigCentralDifferenceX(Vector2 position,  float[,] float_field, float dx) {
        return (float_field[(int)position.x + 1,(int)position.y] - float_field[(int)position.x - 1,(int)position.y]) / (2.0f * dx);
    }
    public static float BigCentralDifferenceY(Vector2 position,  float[,] float_field, float dx) {
        return (float_field[(int)position.x,(int)position.y + 1] - float_field[(int)position.x,(int)position.y - 1]) / (2.0f * dx);
    }
    public static float BigCentralDifferenceX(Vector2 position,  Vec<float> float_field, float dx,int width,int height) {
        return (float_field[(int)position.x*width+(int)position.y+width] - float_field[(int)position.x*width+(int)position.y-width]) / (2.0f * dx);
    }
    public static float BigCentralDifferenceY(Vector2 position,  Vec<float> float_field, float dx,int width, int height) {
        return (float_field[(int)position.x*width+(int)position.y + 1] - float_field[(int)position.x*width+(int)position.y - 1]) / (2.0f * dx);
    }

    public static Vector2 GradientFloatField(Vector2 position, float[,] float_field, float dx) {
        return new Vector2(BigCentralDifferenceX(position,float_field,dx),
                           BigCentralDifferenceY(position,float_field,dx));
    }

    public static Vector2 GradientFloatField(Vector2 position, Vec<float> float_field, float dx,int width, int height) {
        return new Vector2(BigCentralDifferenceX(position,float_field,dx,width,height),
                           BigCentralDifferenceY(position,float_field,dx,width,height));
    }
}