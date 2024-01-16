using static System.Math;

public class Grid {
    public struct Vector2 {
        float x;
        float y;

        public Vector2(float x, float y) {
            this.x = x;
            this.y = y;
        }

        public static Vector2 operator +(Vector2 a) => a;
        public static Vector2 operator -(Vector2 a) => new Vector2(-a.x,-a.y);

        public static Vector2 operator +(Vector2 a, Vector2 b) => new Vector2(a.x + b.x, a.y + b.y);
        public static Vector2 operator -(Vector2 a, Vector2 b) => new Vector2(a.x - b.x, a.y - b.y);
        public static Vector2 operator *(float a, Vector2 b) => new Vector2(a * b.x, a * b.y);

        public Vector2 slice() => new Vector2(System.Math.Abs(this.x % 1),System.Math.Abs(this.y % 1));
    }
    public struct Vector3 {
        float x;
        float y;
        float z;

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

        public Vector3 slice() => new Vector3(System.Math.Abs(this.x % 1),System.Math.Abs(this.y % 1),System.Math.Abs(this.z % 1));
    }
    //MAC Grid
    private float[,] pressure;
    private float[,] u_vel;
    private float[,] v_vel;
    private float timestep, grid_length;
    private int width,height;

    public Grid(int width, int height,float grid_length) {
        //instantiates MAC grid, creates base values
        this.grid_length = grid_length;
        this.width = width;
        this.height = height;
        pressure = new float[width,height];         //p(i,j,k) = p_i,j,k
        u_vel = new float[width+1,height];          //u(i,j,k) = u_i-.5,j,k
        v_vel = new float[width,height+1];          //v(i,j,k) = v_i,j-.5,k
//      w_vel = new float[width,height,depth+1];    //w(i,j,k) = w_i,j,k-.5
    }

    //gets velocity at that point
    private Vector2 VelocityOnGrid(int i, int j) {
        return new Vector2((u_vel[i,j] + u_vel[i+1,j]) / 2.0f,(v_vel[i,j]+v_vel[i,j+1]) / 2.0f);
    }
    private Vector2 LerpVelocity(Vector2 position) {
        int i = System.Math.Floor(position.x);
        int j = System.Math.Floor(position.y);
        Vector2 s = position - new Vector2(i,j);
        return new Vector2((1-s.x)*u_vel[i,j]+s.x*u_vel[i+1][j],(1-s.y)*v_vel[i,j] + s.y*v_vel[i,j+1]);
    }
    private float LerpPressure(Vector2 position) {
        return 0;
    }
    private Vector2 CubicVelocity(Vector2 position) {
        return new Vector2(42,42);
    }
    private float CubicPressure(Vector2 position) {
        return 0;
    }

    private void SemiLagrangian() {
        //loop over every position in grid
            //Vector2 x_P = RungeKutta2(position)
            //set new values at the position to the old values at x_P
    }

    //2nd Order Runge-Kutta Method
    private Vector2 RungeKutta2(Vector2 position) {
        Vector2 x_mid = position - 0.5f * timestep * LerpVelocity(position);
        return position - timestep * LerpVelocity(x_mid);
    }

    //Time step for Semi-Langrangian
    private float MaxAdvectTimestep() {
        //vel_max = max(vel.length) + sqrt(5*grid_length*gravity)
        //timestep = (5 * grid_length) / (vel_max)
        // that is the max for the advect step
        return 0;
    }
}