using static System.Math;

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
    }
    public static float Lerp(float a, float b, float s) {
        return (1 - s) * a + s * b;
    }

    public static float LerpFloatField(Vector2 position,  float[,] float_field) {
        int i = (int)(position.x);
        int j = (int)(position.y);
        float s_x = position.x - i;
        return Lerp(Lerp(float_field[i,j  ],float_field[i+1,j  ],s_x),
                    Lerp(float_field[i,j+1],float_field[i+1,j+1],s_x),
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
    public static Vector2 LerpVelocityField(Vector2 position,  float[,] u_vel,  float[,] v_vel) {
        int i = (int)(position.x+0.5f);
        int j = (int)(position.y+0.5f);
        return new Vector2(Lerp(u_vel[i,j],u_vel[i+1,j],position.x - i + 0.5f),
                           Lerp(v_vel[i,j],v_vel[i,j+1],position.y - j + 0.5f));
    }

    public static float CentralDifferenceX(Vector2 position,  float[,] float_field) {
        return (float_field[(int)position.x + 1,(int)position.y] - float_field[(int)position.x - 1,(int)position.y]) / (2.0f * Grid.grid_length);
    }
    public static float CentralDifferenceY(Vector2 position,  float[,] float_field) {
        return (float_field[(int)position.x,(int)position.y + 1] - float_field[(int)position.x,(int)position.y - 1]) / (2.0f * Grid.grid_length);
    }
}