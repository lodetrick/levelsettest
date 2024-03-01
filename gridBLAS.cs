using static System.MathF;
using NumFlat;
using OpenBlasSharp;
using static VectorFuncs;
using System.Text.Json.Serialization;
using System;
using System.Collections.Generic;
using System.Linq;
using static Godot.GD;
using static Godot.Time;

public class GridBLAS {

    private bool ready = false;

    //MAC Grid
    public Vec<double> pressure;
    public float[,] u_vel, u_vel_hold, v_vel, v_vel_hold;
    // Variables for the Project Step
    private Vec<double> z,s,r,holdervec;
    private Vec<double> Adiag,Ax,Ay,precon,rhs;
    private LevelSet levelSet;
    private float u_bodyforce,v_bodyforce;
    private static float density;
    private float timestep;
    public static float dx {get; set;}
    public int width,height;

    static GridBLAS() {
        dx = 0.01f;        // s
        density = 1000; // kg / m^3
    }

    public GridBLAS(int width, int height,float _grid_length) {
        //instantiates MAC grid, creates base values
        dx = _grid_length;
        this.width = width;
        this.height = height;
        pressure = new Vec<double>(width*height);         //p(i,j,k) = p_i,j,k
        u_vel = new float[(width+1),height];          //u(i,j,k) = u_i-.5,j,k
        v_vel = new float[(width),height+1];          //v(i,j,k) = v_i,j-.5,k
//      w_vel = new float[width,height,depth+1];    //w(i,j,k) = w_i,j,k-.5
        u_vel_hold = new float[(width+1),height];          //u(i,j,k) = u_i-.5,j,k
        v_vel_hold = new float[(width),height+1];          //v(i,j,k) = v_i,j-.5,k

        // Project step
        rhs = new Vec<double>(width*height);
        Adiag = new Vec<double>(width*height);
        Ax = new Vec<double>(width*height);
        Ay = new Vec<double>(width*height);
        precon = new Vec<double>(width*height);
        z = new Vec<double>(width*height);
        s = new Vec<double>(width*height);
        r = new Vec<double>(width*height);
        holdervec = new Vec<double>(width*height);

        levelSet = new LevelSet(width,height);
        u_bodyforce = 0;
        v_bodyforce = 0;

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
            }
        }

        float vel_max = Sqrt(MaxSpeedHold()) + Sqrt(5*dx*(Sqrt(u_bodyforce*u_bodyforce + v_bodyforce*v_bodyforce)));
        timestep = (5 * dx) / (vel_max);
        Print($"Timestep: {timestep}");
        Project(1000); // make sure initial velocities are divergence-free
        Project(1000);
    }

    public void PrintData(float x, float y) {
        Vector2 position = new Vector2(x,y).Constrain(0,0,width-1,height-1,1);
        Print($"Data At: ({position.x},{position.y})");
       //Print($"Pressure: {pressure[(int)x,(int)y]}");
        Print("Velocity: ");
        VectorFuncs.CubicVectorField(position,u_vel,v_vel).Print();
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
        SemiLagrangian(); // u_vel -> u_vel_hold
        //Print("Velocity Advect");
        BodyForces();     // u_vel_hold -> u_vel_hold
        //Print("Body Forces");
        Project();        // u_vel_hold -> u_vel  - Technically could be hold -> hold, but wanted to end on u_vel
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
                u_vel_hold[i,j] += u_bodyforce * timestep;
                v_vel_hold[i,j] += v_bodyforce * timestep;
            }
        }

        for (int i = 0; i < height; i++) {
            u_vel_hold[width,i] += u_bodyforce * timestep;
        }

        for (int i = 0; i < width; i++) {
            v_vel_hold[i,height] += v_bodyforce * timestep;
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


        pressure.Clear();
        rhs.CopyTo(r);

        time = GetTicksUsec();
        Print($"Clear Takes {time - timen} micro-seconds");
        // 4. Solve Ap = b with PCG
        PCGAlgo(max_iter);
        timen = GetTicksUsec();
        Print($"PCGAlgo Takes {timen - time} micro-seconds");
       // Print("PCG");
        // 5. Compute new velocities according to pressure-gradient update to u. - Mark certain positions as unknown
        PressureGradientUpdate();
        time = GetTicksUsec();
        Print($"Pressure Takes {time - timen} micro-seconds");
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
        int position;
        for (int i = 0; i < width; i++) {
            for (int j = 0; j < height; j++) {
                position = i * width + j;
                if (GetMaterialType(i,j) == Material.LIQUID) {
                    rhs[position] = -scale * (u_vel_hold[i+1,j] - u_vel_hold[i,j]
                                        +v_vel_hold[i,j+1] - v_vel_hold[i,j]);
                    
                    if (GetMaterialType(i-1,j) == Material.SOLID) {rhs[position] -= scale * (u_vel_hold[i,j] - GetUSolid(i,j));}
                    if (GetMaterialType(i+1,j) == Material.SOLID) {rhs[position] += scale * (u_vel_hold[i+1,j] - GetUSolid(i+1,j));}

                    if (GetMaterialType(i,j-1) == Material.SOLID) {rhs[position] -= scale * (v_vel_hold[i,j] - GetVSolid(i,j));}
                    if (GetMaterialType(i,j+1) == Material.SOLID) {rhs[position] += scale * (v_vel_hold[i,j+1] - GetVSolid(i,j+1));}
                }
                else {
                    rhs[position] = 0;
                }
            }
        }

        //Compatibility Condition:
        //quick solution, not good
        // double sum = 0;
        // for (int i = 10; i < 190; i++) {
        //     for (int j = 10; j < 190; j++) {
        //         sum += rhs[i,j];
        //     }
        // }

        // for (int i = 0; i < width; i++) {
        //     for (int j = 0; j < height; j++) {
        //         sum -= sum / (180*180);
        //     }
        // }
    }

    // Calculate the varieties of A
    private void CalculateLHS() {
        // Notes: Adiag is storing the coefficient for the relationship with ITSELF,
        // Ax and Ay are storing the coefficients for the relationships between two grid locations
        double scale = timestep / (density * dx * dx);
        int position;
        for (int i = 0; i < width; i++) {
            for (int j = 0; j < height; j++) {
                position = i * width + j;
                Adiag[position] = 0;
                Ax[position] = 0;
                Ay[position] = 0;
                if (GetMaterialType(i,j) == Material.LIQUID) {
                    // x-axis
                    if (GetMaterialType(i-1,j) == Material.LIQUID) { Adiag[position] += scale; }
                    if (GetMaterialType(i+1,j) == Material.LIQUID) { Adiag[position] += scale; Ax[position] = -scale; }
               else if (GetMaterialType(i+1,j) == Material.EMPTY ) { Adiag[position] += scale; }
                    // y-axis
                    if (GetMaterialType(i,j-1) == Material.LIQUID) { Adiag[position] += scale; }
                    if (GetMaterialType(i,j+1) == Material.LIQUID) { Adiag[position] += scale; Ay[position] = -scale; }
               else if (GetMaterialType(i,j+1) == Material.EMPTY ) { Adiag[position] += scale; }
                    if (Adiag[position] == 0) {
                        throw new ArithmeticException($"Calculating A: \n{Adiag[position]} {Ax[position]} {Ay[position]}: {GetMaterialType(i-1,j)}, {GetMaterialType(i+1,j)}, {GetMaterialType(i,j-1)}, {GetMaterialType(i,j+1)}");
                    }
                }
            }
        }
    }

    private void CalculatePreconditioner() {
        // precon starts filled with zeros
        ulong time = GetTicksUsec();

        // then update it
        double tuning = 0.97;
        double safety = 0.25;
        double holder = 0;
        int position;
        for (int i = 0; i < width; i++) {
            for (int j = 0; j < height; j++) {
                position = i * width + j;
                if (GetMaterialType(i,j) == Material.LIQUID) {
                    holder = Adiag[position] - Math.Pow(Ax[position-width] * precon[position-width],2)
                                        - Math.Pow(Ay[position-1] * precon[position-1],2)
                                        - tuning *(Ax[position-width] * (Ay[position-width]) * precon[position-width] * precon[position-width]
                                                 + Ay[position-1    ] * (Ax[position-1    ]) * precon[position-1    ] * precon[position-1    ]);
                    if (holder < safety * Adiag[position]) { holder = Adiag[position]; }
                    precon[position] = 1 / Math.Sqrt(holder);
                    if (double.IsNaN(precon[position])) {
                        throw new ArithmeticException($"Precon is NaN: {holder} =  {Adiag[position]} - {Ax[position-width]} - {Ay[position-1]}");
                    }
                }
                else { precon[position] = 0; } // more precisely, what should the default value be? 1 / 0?
            }
        }
        ulong timen  = GetTicksUsec();
        Print($"CalcPrecon: {timen-time}");
    }

    // Apply Precon
    private void ApplyPreconditioner(Vec<double> r) {
        // ulong time = GetTicksUsec();
        double holder = 0;
        // solve Lq = r
        // I am tempted to replace every mention of q with z. Safe?
        int position;
        for (int i = 0; i < width; i++) {
            for (int j = 0; j < height; j++) {
                position = i * width + j;
                if (GetMaterialType(i,j) == Material.LIQUID) {
                    holder = r[position] - Ax[position-width] * precon[position-width] * z[position - width]
                                         - Ay[position-1] * precon[position-1] * z[position - 1];
                    z[position] = holder * precon[position];
                }
                else { z[position] = 0; } // what should the default value be? r * precon? 0? 
            }
        }
        // solve Ltranspose z = q
        for (int i = width - 1; i >= 0; i--) {
            for (int j = height - 1; j >= 0; j--) {
                position = i * width + j;
                if (GetMaterialType(i,j) == Material.LIQUID) {
                    holder = z[position] - Ax[position] * precon[position] * z[position + width]
                                         - Ay[position] * precon[position] * z[position + 1];
                    z[position] = holder * precon[position];
                }
                else { z[position] = 0; }
            }
        }
        // ulong timen  = GetTicksUsec();
        // Print($"ApplyPrecon: {timen-time}");
    }

    private void ApplyPreconditioner() { // z = precon * r
        var sr = r.Memory.Span;
        var spre = precon.Memory.Span;
        var sax = Ax.Memory.Span;
        var say = Ay.Memory.Span;
        var sz = z.Memory.Span;
        var sls = levelSet.distance_field.Memory.Span;
        var pr = 0;
        var holder = 0.0;

        while (pr < sz.Length) {
            if (sls[pr] > 0) { // material is liquid
                holder = sr[pr] - sax[pr - width] * spre[pr - width] * sz[pr - width]
                                - say[pr - 1    ] * spre[pr - 1    ] * sz[pr - 1    ];
                sz[pr] = holder * spre[pr]; 
            }
            else {
                sz[pr] = 0;
            }
            pr++;
        }

        pr = height * width - 1;

        while (pr >= 0) {
            if (sls[pr] > 0) {
                holder = sz[pr] - sax[pr] * spre[pr] * sz[pr + width]
                                - say[pr] * spre[pr] * sz[pr + 1    ];
                sz[pr] = holder * spre[pr];
            }
            else {
                sz[pr] = 0;
            }

            pr--;
        }
    }

    // Apply A : p = Ar, return p
    private void ApplyA(Vec<double> r) {
        // ulong time = GetTicksUsec();
        //Vec<double> applied = new Vec<double>(r.Count);
        // we have Adiag, Ax, Ay
        // A is symmetric
        // multiply matrix A by vector r
        // for each cell, e = Adiag[i,j] * r[i,j] + Ax[i,j] * r[i+1,j] + Ax[i-1,j] * r[i-1,j] + Ay[i,j] * r[i,j+1] + Ay[i,j-1] * r[i,j-1]
        

        // This is 900 usec, but the while loop is 560 usec. Why?
        // int position;
        // for (int i = 1; i < width-1; i++) {
        //     for (int j = 1; j < height-1; j++) {
        //         position = i * width + j;
        //         z[position] = Adiag[i  ,j  ] * r[position]          // i    , j
        //                     + Ax   [i  ,j  ] * r[position + width]  // i + 1, j
        //                     + Ax   [i-1,j  ] * r[position - width]  // i - 1, j
        //                     + Ay   [i  ,j  ] * r[position + 1]      // i    , j + 1
        //                     + Ay   [i  ,j-1] * r[position - 1];     // i    , j - 1
        //     }
        // }

        var sr = r.Memory.Span;
        var sad = Adiag.Memory.Span;
        var sax = Ax.Memory.Span;
        var say = Ay.Memory.Span;
        var sz = z.Memory.Span;
        var pr = width;

        while (pr < sz.Length - width)
        {
            sz[pr] = sr[pr      ] * sad[pr      ]
                   + sr[pr+width] * sax[pr      ]
                   + sr[pr-width] * sax[pr-width]
                   + sr[pr+1    ] * say[pr      ]
                   + sr[pr-1    ] * say[pr-1    ];
            pr++;
        }
        // ulong timen  = GetTicksUsec();
        // Print($"ApplyA: {timen-time}");
    }

    private double dotproduct(Vec<double> rhs,Vec<double> lhs) {
        //double sum = 0.0;
        // sum = rhs * lhs;

        // obviously slow, but extremely consistant (which I value more)
        // the ddot does 8-10, but with common jumps 100-1000X
        Vec.PointwiseMul(rhs,lhs,holdervec);
        return holdervec.Sum();
        // return rhs.Dot(lhs);
        // foreach(var value in holdervec.GetUnsafeFastIndexer()) {
        //     sum += value;
        // }
        // for (int position = 0; position < width * height * rhs.Stride; position += rhs.Stride) {
        //     sum += rhs.Memory.Span[position] * lhs.Memory.Span[position];
        // }
        // for (int i = 0; i < height*width; i+= 10) {
        //         sum += lhs[i] * rhs[i] 
        //              + lhs[i+1] * rhs[i+1]
        //              + lhs[i+2] * rhs[i+2]
        //              + lhs[i+3] * rhs[i+3]
        //              + lhs[i+4] * rhs[i+4]
        //              + lhs[i+5] * rhs[i+5]
        //              + lhs[i+6] * rhs[i+6]
        //              + lhs[i+7] * rhs[i+7]
        //              + lhs[i+8] * rhs[i+8]
        //              + lhs[i+9] * rhs[i+9];
        // }
        // return sum;
    }

    private unsafe void PCGAlgo(int max_iter = 200, double prev_max_r = 0) {
        // SETUP
        // tolerance tol
        double tol = 0.000001; // 10^-6
        // initial guess for pressure, residual r

        ApplyPreconditioner(); // z = precon * r
        z.CopyTo(s);
        
        double sigma = dotproduct(z,r);
        
        // ITERATIONS
        for (int iter = 0; iter < max_iter; iter++) {
            ulong time = GetTicksUsec();

            ApplyA(s);
            ulong timea = GetTicksUsec();

            Vec.PointwiseMul(z,s,holdervec);
            double alpha = sigma / holdervec.Sum();
            // double alpha = sigma / z.Dot(s);

            ulong timep = GetTicksUsec();

            // Regular Loop Takes 645 usec. This takes 120 usec (WOW)
            Vec.Mul(s,alpha,holdervec);
            pressure.AddInplace(holdervec);
            Vec.Mul(z,alpha,holdervec);
            r.SubInplace(holdervec);
            // for (int i = 0; i < width * height; i+=5) {
            //     pressure[i  ] = pressure[i  ] + alpha * s[i  ];
            //     pressure[i+1] = pressure[i+1] + alpha * s[i+1];
            //     pressure[i+2] = pressure[i+2] + alpha * s[i+2];
            //     pressure[i+3] = pressure[i+3] + alpha * s[i+3];
            //     pressure[i+4] = pressure[i+4] + alpha * s[i+4];
            //     r[i  ] = r[i  ] - alpha * z[i  ];
            //     r[i+1] = r[i+1] - alpha * z[i+1];
            //     r[i+2] = r[i+2] - alpha * z[i+2];
            //     r[i+3] = r[i+3] - alpha * z[i+3];
            //     r[i+4] = r[i+4] - alpha * z[i+4];
            // }
            // These take 180 min, but with common jumps to 8000
            // pressure.AddInplace(alpha * s);
            // r.SubInplace(alpha * z);
            // pressure = pressure + alpha * s;
            // r = r - alpha * z;

            ulong timen = GetTicksUsec();

            // 20 usec vs 90 usec for linq .Max() vs. 250 usec for simple loop
            double max_r = r.InfinityNorm();
            // double max_r = r.Max();
            // double max_r = r[0];
            // for (int i = 0; i < height * width; i++) {
            //     if (max_r < r[i]) {
            //         max_r = r[i];
            //     }
            // }


            if (max_r <= tol) {
                Print($"Iterations: {iter}, (Residual {max_r})");
                return;
            }

            //prev_max_r = max_r;
            ulong timeq = GetTicksUsec();

            ApplyPreconditioner();

            ulong timeb = GetTicksUsec();

            // double _sigma;
            // fixed (double* px = z.Memory.Span)
            // fixed (double* py = r.Memory.Span)
            // {
            //     _sigma =  Blas.Ddot(z.Count, px, z.Stride, py, r.Stride);
            // }

            Vec.PointwiseMul(z,r,holdervec);
            double _sigma = holdervec.Sum();
            // double _sigma = z.Dot(r);
            // double _sigma = z * r;

            ulong times = GetTicksUsec();

            alpha = _sigma / sigma;
            
            // Made it a lot more consistant (idk how, maybe not allocating more memory than needed?)
            // A bit slower (66-150 around vs 66 more frequent but with common spikes up to 10 msec)
            // 66 around a third of the time vs. 66 around a half of the time (with huge spikes)
            s.MulInplace(alpha);
            s.AddInplace(z);
            // s = z + (alpha * s);

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
                    else { u_vel[i,j] = u_vel_hold[i,j] - scale * (float)(pressure[i*width + j] - pressure[(i-1) * width + j]); }
                }
                else { u_vel[i,j] = 0; }

                // y-axis
                if (GetMaterialType(i,j) == Material.LIQUID || GetMaterialType(i,j-1) == Material.LIQUID) {
                    if (GetMaterialType(i,j) == Material.SOLID || GetMaterialType(i,j-1) == Material.SOLID) { v_vel[i,j] = GetVSolid(i,j); }
                    else { v_vel[i,j] = v_vel_hold[i,j] - scale * (float)(pressure[i*width + j] - pressure[i * width + j-1]); }
                }
                else { v_vel[i,j] = 0; }
            }
        }
    }

    //MaterialType: Return 0 for Solid, 1 for Fluid, 2 for Empty (Don't know how to check if it is empty yet)
    public Material GetMaterialType(int x, int y) {
        if (levelSet.GetDistanceOnGrid(x,y) <= 0.1) {
            return Material.SOLID;
        }
        return Material.LIQUID;
       // return levelSet.GetDistanceOnGrid(x,y) < 0 ? Material.SOLID : Material.LIQUID;
    }

    private float GetUSolid(int x, int y) {
        if ((x <= 5 || x >= 195) && y >= 5 && y <= 195) {
            return 0.5f; // moving right
        }
        return 0;
    }
    private float GetVSolid(int x, int y) {
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
        float vel_max = Sqrt(MaxSpeed()) + Sqrt(5*dx*Sqrt(u_bodyforce*u_bodyforce + v_bodyforce*v_bodyforce));
        return (5 * dx) / (vel_max);
        // that is the max for the advect step
    }
}
