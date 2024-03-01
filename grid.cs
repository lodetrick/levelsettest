using static VectorFuncs;
using static System.MathF;
using System.Text.Json.Serialization;
using System;
using System.Collections.Generic;
using System.Linq;
using static Godot.GD;
using static Godot.Time;

public enum Material {
    SOLID,
    LIQUID,
    EMPTY
}

public class Grid {

    private bool ready = false;

    //MAC Grid
    public double[,] pressure;
    public float[,] u_vel, u_vel_hold, v_vel, v_vel_hold;
    // Variables for the Smoke Simulation
    public float[,] smoke_density, smoke_density_hold;
    public bool using_hold = false;
    // Variables for the Project Step
    private double[,] Adiag,Ax,Ay,precon,z,rhs,s,r;
    private LevelSet levelSet;
    private Vector2 bodyforce, outsideforce;
    private static float density;
    private float timestep;
    public static float dx {get; set;}
    public int width,height;

    static Grid() {
        dx = 0.01f;        // s
        density = 1000; // kg / m^3
    }

    public Grid(int width, int height,float _grid_length) {
        //instantiates MAC grid, creates base values
        dx = _grid_length;
        this.width = width;
        this.height = height;
        pressure = new double[width,height];         //p(i,j,k) = p_i,j,k
        u_vel = new float[width+1,height];          //u(i,j,k) = u_i-.5,j,k
        v_vel = new float[width,height+1];          //v(i,j,k) = v_i,j-.5,k
//      w_vel = new float[width,height,depth+1];    //w(i,j,k) = w_i,j,k-.5
        u_vel_hold = new float[width+1,height];          //u(i,j,k) = u_i-.5,j,k
        v_vel_hold = new float[width,height+1];          //v(i,j,k) = v_i,j-.5,k

        // Smoke Simulation Parameters
        smoke_density = new float[width,height];
        smoke_density_hold = new float[width, height];

        // Project step
        rhs = new double[width,height];
        Adiag = new double[width,height];
        Ax = new double[width,height];
        Ay = new double[width,height];
        precon = new double[width,height];
        z = new double[width,height];
        s = new double[width,height];
        r = new double[width,height];

        levelSet = new LevelSet(width,height);
        bodyforce = new Vector2(0,0);
        outsideforce = new Vector2(0.5f,0.3f);


        InitialConditions();
    }

    private void InitialConditions() {
        levelSet.AddBox(3,5,197,195,-1); // inverse box
        levelSet.AddBox(90,90,110,110);
        levelSet.AddBox(105,105,130,120);
        levelSet.AddBox(120,80,140,95);
        levelSet.AddBox(40,67.5f,80,80);
        levelSet.AddBox(60,70,70,95);
        levelSet.AddBox(50,50,60,60);
        levelSet.AddBox(45,180,150,165);
        levelSet.AddBox(80,185,130,175);
        levelSet.AddBox(140,60,150,65);
        levelSet.AddBox(135,55,120,20);
        levelSet.AddBox(155,8,165,30);

        for (int i = 0; i < width; i++) {
            for (int j = 0; j < height; j++) {
                if (GetMaterialType(i+1,j+1) == Material.LIQUID && GetMaterialType(i-1,j-1) == Material.LIQUID) {
                    u_vel_hold[i,j] = 0.5f;
                    v_vel_hold[i,j] = 0f;
                }
                else {
                    u_vel_hold[i,j] = 0f;
                    v_vel_hold[i,j] = 0f;
                }
                if (i > width/2 && i < width/2 + 20 && j > height * 0.8 && GetMaterialType(i,j) != Material.SOLID) {
                    smoke_density[i,j] = 1;
                }
            }
        }

        float vel_max = Sqrt(MaxSpeedHold()) + Sqrt(5*dx*bodyforce.Length());
        timestep = (5 * dx) / (vel_max);
        Print($"Timestep: {timestep}");
        Project(300); // make sure initial velocities are divergence-free
    }

    public void PrintData(float x, float y) {
        Vector2 position = new Vector2(x,y).Constrain(0,0,width-1,height-1,1);
        Print($"Data At: ({position.x},{position.y})");
       //Print($"Pressure: {pressure[(int)x,(int)y]}");
        Print("Velocity: ");
        VectorFuncs.CubicVectorField(position,u_vel,v_vel).Print();
        Print($"Density: {VectorFuncs.LerpFloatField(position,using_hold ? smoke_density_hold : smoke_density)}");
        string type = "";
        switch (GetMaterialType((int)x,(int)y)) {
            case Material.SOLID: type = "Solid"; break;
            case Material.LIQUID: type = "Liquid"; break;
            case Material.EMPTY: type = "Empty"; break;
        }
        Print($"State: {type}");

    }

    //gets velocity at that point
    private Vector2 VelocityOnGrid(int i, int j) {
        return new Vector2((u_vel[i,j] + u_vel[i+1,j]) / 2.0f,(v_vel[i,j]+v_vel[i,j+1]) / 2.0f);
    }

    public void Step() {
        timestep = CalculateTimestep();
        Print($"Timestep: {timestep}");
        //SemiLagrangian(using_hold ? smoke_density_hold : smoke_density, using_hold ? smoke_density : smoke_density_hold);
        //using_hold = !using_hold;
        //Print("Smoke Advect");

        ready = true;
        ulong time = GetTicksUsec();
        SemiLagrangian(); // u_vel -> u_vel_hold
        ulong timen = GetTicksUsec();
        Print($"Semi-Lagrangian Took {timen - time} micro-seconds");
        //Print("Velocity Advect");
        BodyForces();     // u_vel_hold -> u_vel_hold
        time = GetTicksUsec();
        Print($"Body Forces Took {time - timen} Micro-Seconds");
        //Print("Body Forces");
        Project();        // u_vel_hold -> u_vel  - Technically could be hold -> hold, but wanted to end on u_vel
        timen = GetTicksUsec();
        Print($"Project takes {timen - time} Micro-Seconds");
        //Print("Project");
    }
    private void SemiLagrangian(float[,] get_field, float[,] store_field) {
        for (int i = 0; i < width; i++) {
            for (int j = 0; j < height; j++) {
                Vector2 x_P = RungeKutta2(new Vector2(i,j));
                x_P = x_P.Constrain(0,0,width-1,height-1,1);
                if (levelSet.GetDistance(x_P) < 0) {
                    x_P = levelSet.FindClosest(x_P);
                }
                x_P = x_P.Constrain(0,0,width-1,height-1,5);
                store_field[i,j] = VectorFuncs.CubicFloatField(x_P,get_field); ////////
            }
        }
    }

    private void SemiLagrangian() {
        for (int i = 0; i < width; i++) {
            for (int j = 0; j < height; j++) {
                Vector2 x_Pu = RungeKutta2(new Vector2(i-0.5f,j));
                x_Pu = x_Pu.Constrain(0,0,width-1,height-1,1);
                if (levelSet.GetDistance(x_Pu) < 0) {
                    x_Pu = levelSet.FindClosest(x_Pu);
                }
                x_Pu = x_Pu.Constrain(0,0,width-1,height-1,5);
                u_vel_hold[i,j] = VectorFuncs.CubicFloatField(x_Pu,u_vel); ////////

                Vector2 x_Pv = RungeKutta2(new Vector2(i,j-0.5f));
                x_Pv = x_Pv.Constrain(0,0,width-1,height-1,1);
                if (levelSet.GetDistance(x_Pv) < 0) {
                    x_Pv = levelSet.FindClosest(x_Pv);
                }
                x_Pv = x_Pv.Constrain(0,0,width-1,height-1,5);
                v_vel_hold[i,j] = VectorFuncs.CubicFloatField(x_Pv,v_vel); ////////
            }
        }

        for (int i = 0; i < height; i++) {
            Vector2 x_Pul = RungeKutta2(new Vector2(width - 0.5f,i));
            x_Pul = x_Pul.Constrain(0,0,width-1,height-1,1);
            if (levelSet.GetDistance(x_Pul) < 0) {
                x_Pul = levelSet.FindClosest(x_Pul);
            }
            x_Pul = x_Pul.Constrain(0,0,width-1,height-1,5);
            u_vel_hold[width,i] = VectorFuncs.CubicFloatField(x_Pul,u_vel); ////////
        }

        for (int i = 0; i < width; i++) {
            Vector2 x_Pvl = RungeKutta2(new Vector2(i, height - 0.5f));
            x_Pvl = x_Pvl.Constrain(0,0,width-1,height-1,1);
            if (levelSet.GetDistance(x_Pvl) < 0) {
                x_Pvl = levelSet.FindClosest(x_Pvl);
            }
            x_Pvl = x_Pvl.Constrain(0,0,width-1,height-1,5);
            v_vel_hold[i,height] = VectorFuncs.CubicFloatField(x_Pvl,v_vel); ////////
        }
    }

    //Forward Euler
    private void BodyForces() {
        for (int i = 0; i < width; i++) {
            for (int j = 0; j < height; j++) {
                u_vel_hold[i,j] += bodyforce.x * timestep;
                v_vel_hold[i,j] += bodyforce.y * timestep;
            }
        }

        for (int i = 0; i < height; i++) {
            u_vel_hold[width,i] += bodyforce.x * timestep;
        }

        for (int i = 0; i < width; i++) {
            v_vel_hold[i,height] += bodyforce.y * timestep;
        }
    }

    private void Project(int max_iter = 200) {
        // 1. Calculate negative divergence with modifications at solid wall boundaries
        ulong time = GetTicksUsec();
        CalculateRHS();
        ulong timen = GetTicksUsec();
        Print($"Calc RHS Takes {timen - time} micro-seconds");
       // Print("RHS");
        // Note: Matrix Solves should use Doubles, not Floats
        // 2. Set the entries of A (Adiag, Ax, Ay)
        CalculateLHS();
        time = GetTicksUsec();
        Print($"Calc LHS Takes {time - timen} micro-seconds");
       // Print("LHS");
        // 3. Construct the Preconditioner (MIC(0))
        CalculatePreconditioner();
        timen = GetTicksUsec();
        Print($"Calc Precon Takes {timen - time} micro-seconds");
       // Print("Preconditioner");

        for (int i = 0; i < width; i++) {
            for (int j = 0; j < height; j++) {
                pressure[i,j] = 0;
                r[i,j] = rhs[i,j];
            }
        }
        time = GetTicksUsec();
        Print($"Clear Takes {time - timen} micro-seconds");
        // 4. Solve Ap = b with PCG
        PCGAlgo(max_iter);
        timen = GetTicksUsec();
        Print($"PCG Algo Takes {timen - time} micro-seconds");
       // Print("PCG");
        // 5. Compute new velocities according to pressure-gradient update to u. - Mark certain positions as unknown
        PressureGradientUpdate();
        time = GetTicksUsec();
        Print($"Pressure Gradient Update Takes {time - timen} micro-seconds");
        //Print("PressureGradientUpdate");
        // 6. Use Breadth-First Search to extrapolate velocity values
       // ExtrapolateField(u_vel);
       // Print("Extrapolate U");
       // ExtrapolateField(v_vel);
       // Print("Extrapolate V");
    }

    // Calculate Negative Divergence (Right Hand Side of Equation)
    private void CalculateRHS() {
        double scale = 1 / dx;
        for (int i = 0; i < width; i++) {
            for (int j = 0; j < height; j++) {
                if (GetMaterialType(i,j) == Material.LIQUID) {
                    rhs[i,j] = -scale * (u_vel_hold[i+1,j] - u_vel_hold[i,j]
                                        +v_vel_hold[i,j+1] - v_vel_hold[i,j]);
                    
                    if (GetMaterialType(i-1,j) == Material.SOLID) {rhs[i,j] -= scale * (u_vel_hold[i,j] - GetUSolid(i,j));}
                    if (GetMaterialType(i+1,j) == Material.SOLID) {rhs[i,j] += scale * (u_vel_hold[i+1,j] - GetUSolid(i+1,j));}

                    if (GetMaterialType(i,j-1) == Material.SOLID) {rhs[i,j] -= scale * (v_vel_hold[i,j] - GetVSolid(i,j));}
                    if (GetMaterialType(i,j+1) == Material.SOLID) {rhs[i,j] += scale * (v_vel_hold[i,j+1] - GetVSolid(i,j+1));}
                }
                else {
                    rhs[i,j] = 0;
                }
            }
        }

        //Compatibility Condition:
        //quick solution, not good
        double sum = 0;
        for (int i = 10; i < 190; i++) {
            for (int j = 10; j < 190; j++) {
                sum += rhs[i,j];
            }
        }

        for (int i = 0; i < width; i++) {
            for (int j = 0; j < height; j++) {
                sum -= sum / (180*180);
            }
        }
    }

    // Calculate the varieties of A
    private void CalculateLHS() {
        // Notes: Adiag is storing the coefficient for the relationship with ITSELF,
        // Ax and Ay are storing the coefficients for the relationships between two grid locations
        double scale = timestep / (density * dx * dx);
        for (int i = 0; i < width; i++) {
            for (int j = 0; j < height; j++) {
                Adiag[i,j] = 0;
                Ax[i,j] = 0;
                Ay[i,j] = 0;
                if (GetMaterialType(i,j) == Material.LIQUID) {
                    // x-axis
                    if (GetMaterialType(i-1,j) == Material.LIQUID) { Adiag[i,j] += scale; }
                    if (GetMaterialType(i+1,j) == Material.LIQUID) { Adiag[i,j] += scale; Ax[i,j] = -scale; }
               else if (GetMaterialType(i+1,j) == Material.EMPTY ) { Adiag[i,j] += scale; }
                    // y-axis
                    if (GetMaterialType(i,j-1) == Material.LIQUID) { Adiag[i,j] += scale; }
                    if (GetMaterialType(i,j+1) == Material.LIQUID) { Adiag[i,j] += scale; Ay[i,j] = -scale; }
               else if (GetMaterialType(i,j+1) == Material.EMPTY ) { Adiag[i,j] += scale; }
                    if (Adiag[i,j] == 0) {
                        throw new ArithmeticException($"Calculating A: \n{Adiag[i,j]} {Ax[i,j]} {Ay[i,j]}: {GetMaterialType(i-1,j)}, {GetMaterialType(i+1,j)}, {GetMaterialType(i,j-1)}, {GetMaterialType(i,j+1)}");
                    }
                }
            }
        }
    }

    private void CalculatePreconditioner() {
        // precon starts filled with zeros

        // then update it
        double tuning = 0.97;
        double safety = 0.25;
        double holder = 0;
        for (int i = 0; i < width; i++) {
            for (int j = 0; j < height; j++) {
                if (GetMaterialType(i,j) == Material.LIQUID) {
                    holder = Adiag[i,j] - Math.Pow(Ax[i-1,j] * precon[i-1,j],2)
                                        - Math.Pow(Ay[i,j-1] * precon[i,j-1],2)
                                        - tuning *(Ax[i-1,j] * (Ay[i-1,j]) * precon[i-1,j] * precon[i-1,j]
                                                 + Ay[i,j-1] * (Ax[i,j-1]) * precon[i,j-1] * precon[i,j-1]);
                    if (holder < safety * Adiag[i,j]) { holder = Adiag[i,j]; }
                    precon[i,j] = 1 / Math.Sqrt(holder);
                    if (double.IsNaN(precon[i,j])) {
                        throw new ArithmeticException($"Precon is NaN: {holder} =  {Adiag[i,j]} - {Ax[i-1,j]} - {Ay[i,j-1]}");
                    }
                }
                else { precon[i,j] = 0; } // more precisely, what should the default value be? 1 / 0?
            }
        }
    }

    // Apply Precon
    private void ApplyPreconditioner(double[,] r) {
        double holder = 0;
        // solve Lq = r
        // I am tempted to replace every mention of q with z. Safe?
        for (int i = 0; i < width; i++) {
            for (int j = 0; j < height; j++) {
                if (GetMaterialType(i,j) == Material.LIQUID) {
                    holder = r[i,j] - Ax[i-1,j] * precon[i-1,j] * z[i-1,j]
                                    - Ay[i,j-1] * precon[i,j-1] * z[i,j-1];
                    z[i,j] = holder * precon[i,j];
                }
                else { z[i,j] = 0; } // what should the default value be? r * precon? 0? 
            }
        }
        // solve Ltranspose z = q
        for (int i = width - 1; i >= 0; i--) {
            for (int j = height - 1; j >= 0; j--) {
                if (GetMaterialType(i,j) == Material.LIQUID) {
                    holder = z[i,j] - Ax[i,j] * precon[i,j] * z[i+1,j]
                                    - Ay[i,j] * precon[i,j] * z[i,j+1];
                    z[i,j] = holder * precon[i,j];

                    if (double.IsNaN(z[i,j])) {
                        throw new ArithmeticException($"Applying Precon is NaN: {holder} * {precon[i,j]}");
                    }
                }
                else { z[i,j] = 0; }
            }
        }
    }

    // Apply A : p = Ar, return p
    private double[,] ApplyA(double[,] r) {
        double[,] applied = new double[r.GetLength(0),r.GetLength(1)];
        // we have Adiag, Ax, Ay
        // A is symmetric
        // multiply matrix A by vector r
        // for each cell, e = Adiag[i,j] * r[i,j] + Ax[i,j] * r[i+1,j] + Ax[i-1,j] * r[i-1,j] + Ay[i,j] * r[i,j+1] + Ay[i,j-1] * r[i,j-1]
        for (int i = 1; i < width-1; i++) {
            for (int j = 1; j < height-1; j++) {
                applied[i,j] = Adiag[i  ,j  ] * r[i  ,j  ] 
                             + Ax   [i  ,j  ] * r[i+1,j  ] 
                             + Ax   [i-1,j  ] * r[i-1,j  ] 
                             + Ay   [i  ,j  ] * r[i  ,j+1] 
                             + Ay   [i  ,j-1] * r[i  ,j-1];
            }
        }
        for (int i = 0; i < width; i++) {
            applied[i,0] = 0;
            applied[i,height-1] = 0;
        }
        for (int i = 0; i < height; i++) {
            applied[0,i] = 0;
            applied[width-1,i] = 0;
        }
        return applied;
    }

    private double dotproduct(double[,] lhs, double[,] rhs) {
        double sum = 0;
        for (int i = 0; i < width; i++) {
            for (int j = 0; j < height; j++) {
                sum += lhs[i,j] * rhs[i,j];
            }
        }
        return sum;
    }

    private void PCGAlgo(int max_iter = 200, double prev_max_r = 0) {
        // SETUP
        // tolerance tol
        double tol = 0.000001; // 10^-6
        // initial guess for pressure, residual r

        ApplyPreconditioner(r); // z = precon * r
        for (int i = 0; i < width; i++) {
            for (int j = 0; j < height; j++) {
                s[i,j] = z[i,j];
                if (s[i,j] != 0) {
                    //Print($"z is not zero: {s[i,j]}");
                }
            }
        }

        double sigma = dotproduct(z,r);
        // ITERATIONS
        for (int iter = 0; iter < max_iter; iter++) {
            ulong time = GetTicksUsec();

            z = ApplyA(s);

            ulong timea = GetTicksUsec();

            double alpha = sigma / (dotproduct(z,s));

            ulong timep = GetTicksUsec();

            double max_r = r[0,0];
            for (int i = 0; i < width; i++) {
                for (int j = 0; j < height; j++) {
                    pressure[i,j] += alpha * s[i,j];
                    r[i,j] -= alpha * z[i,j];
                    if (max_r < Math.Abs(r[i,j])) {
                        max_r = Math.Abs(r[i,j]);
                    }
                }
            }
            if (max_r <= tol) {
                Print($"Iterations: {iter}, (Residual {max_r})");
                return;
            }
            ulong timen = GetTicksUsec();/*
            else if (iter > 150){
                Print($"MAX Residual: {max_r} with a {alpha} at ({x}, {y})\n z= {z[x,y]}, s={s[x,y]}, Adiag={Adiag[x,y]}, Ax={Ax[x,y]}, Ay={Ay[x,y]}, rhs={rhs[x,y]}");
            }
            else if (max_r == prev_max_r) {
                Print($"Same Residual: {max_r} with a {alpha} at ({x}, {y})\n z= {z[x,y]}, s={s[x,y]}, Adiag={Adiag[x,y]}, Ax={Ax[x,y]}, Ay={Ay[x,y]}, rhs={rhs[x,y]}");
            }
            if (iter > 20 && max_r >= 1.5 * prev_max_r) {
                // reverse pressure and r
                for (int i = 0; i < width; i++) {
                    for (int j = 0; j < height; j++) {
                        pressure[i,j] -= alpha * s[i,j];
                        r[i,j] += alpha * z[i,j];
                    }
                }

                Print($"Restart at {iter}. {max_r} vs. {prev_max_r}");
                PCGAlgo(max_iter - iter,prev_max_r);
                return;
            }*/

            prev_max_r = max_r;
            ulong timeq = GetTicksUsec();
            ApplyPreconditioner(r);
            ulong timeb = GetTicksUsec();
            double _sigma = dotproduct(z,r);
            ulong times = GetTicksUsec();
            alpha = _sigma / sigma;

            for (int i = 0; i < width; i++) {
                for (int j = 0; j < height; j++) {
                    s[i,j] = z[i,j] + alpha * s[i,j];
                }
            }
            sigma = _sigma;
            ulong timer = GetTicksUsec();
            Print($"AA: {timea - time}, al: {timep - timea}, p+r: {timen - timep}, max_r: {timeq-timen}, APre: {timeb - timeq}, dot: {times - timeb}, s: {timer - times} Total: {timer - time}");
        }

        Print($"Max Iterations Exceeded (Residual {prev_max_r})"); // report iteration limit exceeded
    }

    // Pressure Gradient Update : Given Pressure, Apply it
    private void PressureGradientUpdate() {
        float scale = timestep / (density * dx);
        for (int i = 0; i < width; i++) {
            for (int j = 0; j < height; j++) {
                // x-axis
                if (GetMaterialType(i,j) == Material.LIQUID || GetMaterialType(i-1,j) == Material.LIQUID) {
                    if (GetMaterialType(i,j) == Material.SOLID || GetMaterialType(i-1,j) == Material.SOLID) { u_vel[i,j] = GetUSolid(i,j); }
                    else { u_vel[i,j] = u_vel_hold[i,j] - scale * (float)(pressure[i,j] - pressure[i-1,j]); }
                }
                else { u_vel[i,j] = 0; }

                // y-axis
                if (GetMaterialType(i,j) == Material.LIQUID || GetMaterialType(i,j-1) == Material.LIQUID) {
                    if (GetMaterialType(i,j) == Material.SOLID || GetMaterialType(i,j-1) == Material.SOLID) { v_vel[i,j] = GetVSolid(i,j); }
                    else { v_vel[i,j] = v_vel_hold[i,j] - scale * (float)(pressure[i,j] - pressure[i,j-1]); }
                }
                else { v_vel[i,j] = 0; }
            }
        }
    }

    //MaterialType: Return 0 for Solid, 1 for Fluid, 2 for Empty (Don't know how to check if it is empty yet)
    public Material GetMaterialType(int x, int y) {
        if (levelSet.GetDistanceOnGrid(x,y) <= 0.1) {
            return Material.SOLID;
        }/*
        else if ((using_hold ? smoke_density_hold[x,y] : smoke_density[x,y]) < 0.1) {
            return Material.EMPTY;
        }*/
        return Material.LIQUID;
       // return levelSet.GetDistanceOnGrid(x,y) < 0 ? Material.SOLID : Material.LIQUID;
    }

    // outside: levelSet.AddBox(3,5,197,195,-1);

    private float GetUSolid(int x, int y) {
        if ((x <= 5 || x >= 195) && y >= 5 && y <= 195) {
            return outsideforce.x; // moving right
        }
        return 0;
    }
    private float GetVSolid(int x, int y) {
         if ((x <= 4 || x >= 196) || (y <= 6 || y >= 194)) {
            return outsideforce.y; // moving down
        }
        return 0;
    }

    // Breadth-First Search To Extrapolate Values
    private void ExtrapolateField(float[,] float_field) {
        int[,] markers = new int[float_field.GetLength(0),float_field.GetLength(1)];
        List<ValueTuple<int,int> > Wave = new List<ValueTuple<int,int>>();
        int i, j;

        for (i = 1; i < width-1; i++) {
            for (j = 1; j < height-1; j++) {
                if (double.IsNaN(float_field[i,j])) {
                    if (double.IsNaN(float_field[i-1,j]) && double.IsNaN(float_field[i+1,j]) && double.IsNaN(float_field[i,j-1]) && double.IsNaN(float_field[i,j+1])) {
                        markers[i,j] = int.MaxValue;
                        continue;
                    }
                    Wave.Add((i,j));
                    markers[i,j] = 1;
                }
                else {
                    markers[i,j] = 0;
                }
            }
        }
        for (i = 0; i < width; i++) {
            markers[i,0] = int.MaxValue;
            markers[i,height-1] = int.MaxValue;
        }
        for (i = 0; i < height; i++) {
            markers[0,i] = int.MaxValue;
            markers[width-1,i] = int.MaxValue;
        }
        // to prevent the compiler from compiling this away
        int t = 0;
        while (t < Wave.Count) {
            i = Wave[t].Item1;
            j = Wave[t].Item2;
            float_field[i,j] = 0;
            int total = 0;
            if (i   != 0      && markers[i-1,j] < markers[i,j]) { float_field[i,j] += float_field[i-1,j]; total++; }
            if (i+1 != width  && markers[i+1,j] < markers[i,j]) { float_field[i,j] += float_field[i+1,j]; total++; }
            if (j   != 0      && markers[i,j-1] < markers[i,j]) { float_field[i,j] += float_field[i,j-1]; total++; }
            if (j+1 != height && markers[i,j+1] < markers[i,j]) { float_field[i,j] += float_field[i,j+1]; total++; }
            float_field[i,j] /= total;
            if (total == 0) {
                Print("Total is Zero, add case");
            }

            if (i   != 0      && markers[i-1,j] == int.MaxValue) { Wave.Add((i-1,j)); markers[i-1,j] = markers[i,j] + 1; }
            if (i+1 != width  && markers[i+1,j] == int.MaxValue) { Wave.Add((i+1,j)); markers[i+1,j] = markers[i,j] + 1; }
            if (j   != 0      && markers[i,j-1] == int.MaxValue) { Wave.Add((i,j-1)); markers[i,j-1] = markers[i,j] + 1; }
            if (j+1 != height && markers[i,j+1] == int.MaxValue) { Wave.Add((i,j+1)); markers[i,j+1] = markers[i,j] + 1; }

            t++;
        }
    }

    private float CalculateTimestep() {
        return MaxAdvectTimestep();
    }

    //2nd Order Runge-Kutta Method
    private Vector2 RungeKutta2(Vector2 position) {
        Vector2 x_mid = position - 0.5f * timestep * VectorFuncs.LerpVelocityField(position.Constrain(0,0,width-1,height-1,1),
                                                                                   u_vel, v_vel);
        return position - timestep * VectorFuncs.LerpVelocityField(x_mid.Constrain(0,0,width-1,height-1,1), u_vel, v_vel);
    }

    private float MaxSpeed() {
        float maxu = u_vel.Cast<float>().Max();
        float maxv = v_vel.Cast<float>().Max();
        Print($"Max Speed: {Sqrt(maxu * maxu + maxv * maxv)}");
        return maxu * maxu + maxv * maxv;
    }

    private float MaxSpeedHold() {
        float maxu = u_vel_hold.Cast<float>().Max();
        float maxv = v_vel_hold.Cast<float>().Max();
        Print($"Max Speed: {Sqrt(maxu * maxu + maxv * maxv)}");
        return maxu * maxu + maxv * maxv;
    }

    //Time step for Semi-Langrangian
    private float MaxAdvectTimestep() {
        float vel_max = Sqrt(MaxSpeed()) + Sqrt(5*dx*bodyforce.Length());
        return (5 * dx) / (vel_max);
        // that is the max for the advect step
    }
}