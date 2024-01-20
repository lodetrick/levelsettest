using static VectorFuncs;

public class Grid {
    //MAC Grid
    private float[,] pressure;
    private float[,] u_vel;
    private float[,] v_vel;
    private float timestep;
    public static float grid_length {get; set;}
    private int width,height;

    public Grid(int width, int height,float _grid_length) {
        //instantiates MAC grid, creates base values
        grid_length = _grid_length;
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
            //use cubic interpolation
    }

    //2nd Order Runge-Kutta Method
    private Vector2 RungeKutta2(Vector2 position) {
        Vector2 x_mid = position - 0.5f * timestep * VectorFuncs.LerpVelocityField(position, u_vel, v_vel);
        return position - timestep * VectorFuncs.LerpVelocityField(x_mid, u_vel, v_vel);
    }

    //Time step for Semi-Langrangian
    private float MaxAdvectTimestep() {
        //vel_max = max(vel.length) + sqrt(5*grid_length*gravity)
        //timestep = (5 * grid_length) / (vel_max)
        // that is the max for the advect step
        return 0;
    }
}