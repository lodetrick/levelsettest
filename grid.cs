using static VectorFuncs;
using static System.MathF;

public class Grid {
    //MAC Grid
    private float[,] pressure;
    private float[,] u_vel;
    private float[,] v_vel;
    private Vector2 bodyforce;
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

    private void SemiLagrangian() {
        for (int i = 0; i < width; i++) {
            for (int j = 0; j < height; j++) {
                Vector2 x_Pu = RungeKutta2(new Vector2(i - 0.5f,j));
                u_vel[i,j] = VectorFuncs.CubicFloatField(x_Pu,u_vel); ////////

                Vector2 x_Pv = RungeKutta2(new Vector2(i,j - 0.5f));
                v_vel[i,j] = VectorFuncs.CubicFloatField(x_Pv,v_vel); ////////
            }
        }

        for (int i = 0; i < height; i++) {
            Vector2 x_Pul = RungeKutta2(new Vector2(width + 0.5f,i));
            u_vel[width,i] = VectorFuncs.CubicFloatField(x_Pul,u_vel); ////////
        }

        for (int i = 0; i < width; i++) {
            Vector2 x_Pvl = RungeKutta2(new Vector2(i, height + 0.5f));
            v_vel[i,height] = VectorFuncs.CubicFloatField(x_Pvl,v_vel); ////////
        }
    }

    //Forward Euler
    private void BodyForces() {
        for (int i = 0; i < width; i++) {
            for (int j = 0; j < height; j++) {
                u_vel[i,j] += bodyforce.x * timestep; ////////////////////////
                v_vel[i,j] += bodyforce.y * timestep; ////////////////////////
            }
        }

        for (int i = 0; i < height; i++) {
            u_vel[width,i]  += bodyforce.x * timestep; ///////////////////////
        }

        for (int i = 0; i < width; i++) {
            v_vel[i,height] += bodyforce.y * timestep; ///////////////////////
        }
    }

    //2nd Order Runge-Kutta Method
    private Vector2 RungeKutta2(Vector2 position) {
        Vector2 x_mid = position - 0.5f * timestep * VectorFuncs.LerpVelocityField(position, u_vel, v_vel);
        return position - timestep * VectorFuncs.LerpVelocityField(x_mid, u_vel, v_vel);
    }

    private float MaxSpeed() {
        return 0;
    }

    //Time step for Semi-Langrangian
    private float MaxAdvectTimestep() {
        float vel_max = Sqrt(MaxSpeed()) + Sqrt(5*grid_length*bodyforce.Length());
        return (5 * grid_length) / (vel_max);
        // that is the max for the advect step
    }
}