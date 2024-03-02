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
    //MAC Grid
    private Vec<double> _pressure;
    public float[,] UVel, UVelHold, VVel, VVelHold;
    // Variables for the Project Step
    private Vec<double> _z,_s,_r,_holdervec;
    private Vec<double> _adiag,_ax,_ay,_precon,_rhs;
    private LevelSet _levelSet;
    private float _uBodyforce,_vBodyforce;
    private static float _density;
    private float _timestep;
    public static float Dx {get; set;}
    public int Width,Height;

    static GridBLAS() {
        Dx = 0.01f;        // s
        _density = 1000; // kg / m^3
    }

    public GridBLAS(int width, int height,float gridLength) {
        //instantiates MAC grid, creates base values
        Dx = gridLength;
        this.Width = width;
        this.Height = height;
        _pressure = new Vec<double>(width*height);         //p(i,j,k) = p_i,j,k
        UVel = new float[(width+1),height];          //u(i,j,k) = u_i-.5,j,k
        VVel = new float[(width),height+1];          //v(i,j,k) = v_i,j-.5,k
//      w_vel = new float[width,height,depth+1];    //w(i,j,k) = w_i,j,k-.5
        UVelHold = new float[(width+1),height];          //u(i,j,k) = u_i-.5,j,k
        VVelHold = new float[(width),height+1];          //v(i,j,k) = v_i,j-.5,k

        // Project step
        _rhs = new Vec<double>(width*height);
        _adiag = new Vec<double>(width*height);
        _ax = new Vec<double>(width*height);
        _ay = new Vec<double>(width*height);
        _precon = new Vec<double>(width*height);
        _z = new Vec<double>(width*height);
        _s = new Vec<double>(width*height);
        _r = new Vec<double>(width*height);
        _holdervec = new Vec<double>(width*height);

        _levelSet = new LevelSet(width,height);
        _uBodyforce = 0;
        _vBodyforce = 0;

        InitialConditions();
    }

    private void InitialConditions() {
        _levelSet.AddBox(3,5,197,195,-1); // inverse box
        _levelSet.AddBox(90,90,110,110);
        _levelSet.AddBox(105,105,130,120);
        _levelSet.AddBox(120,80,140,95);
        _levelSet.AddBox(40,67.5f,80,80);
        _levelSet.AddBox(60,70,70,95);
        _levelSet.AddBox(50,50,60,60);
        _levelSet.AddBox(45,180,150,165);
        _levelSet.AddBox(80,185,130,175);
        _levelSet.AddBox(140,60,150,65);
        _levelSet.AddBox(135,55,120,20);
        _levelSet.AddBox(155,8,165,30);

        for (int i = 0; i < Width; i++) {
            for (int j = 0; j < Height; j++) {
                if (GetMaterialType(i+1,j+1) == Material.LIQUID && GetMaterialType(i-1,j-1) == Material.LIQUID) {
                    UVelHold[i,j] = 0.5f;
                    VVelHold[i,j] = 0f;
                }
                else {
                    UVelHold[i,j] = 0f;
                    VVelHold[i,j] = 0f;
                }
            }
        }

        float velMax = Sqrt(MaxSpeedHold()) + Sqrt(5*Dx*(Sqrt(_uBodyforce*_uBodyforce + _vBodyforce*_vBodyforce)));
        _timestep = (5 * Dx) / (velMax);
        Print($"Timestep: {_timestep}");
        Project(1000); // make sure initial velocities are divergence-free
    }

    public void PrintData(float x, float y) {
        Vector2 position = new Vector2(x,y).Constrain(0,0,Width-1,Height-1,1);
        Print($"Data At: ({position.x},{position.y})");
       //Print($"Pressure: {pressure[(int)x,(int)y]}");
        Print("Velocity: ");
        VectorFuncs.CubicVectorField(position,UVel,VVel).Print();
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
        return new Vector2((UVel[i,j] + UVel[i+1,j]) / 2.0f,(VVel[i,j]+VVel[i,j+1]) / 2.0f);
    }

    public void Step() {
        _timestep = CalculateTimestep();
        Print($"Timestep: {_timestep}");
        //SemiLagrangian(using_hold ? smoke_density_hold : smoke_density, using_hold ? smoke_density : smoke_density_hold);
        //using_hold = !using_hold;
        //Print("Smoke Advect");

        SemiLagrangian(); // u_vel -> u_vel_hold
        //Print("Velocity Advect");
        BodyForces();     // u_vel_hold -> u_vel_hold
        //Print("Body Forces");
        Project();        // u_vel_hold -> u_vel  - Technically could be hold -> hold, but wanted to end on u_vel
        //Print("Project");
    }
    private void SemiLagrangian(float[,] getField, float[,] storeField) {
        for (int i = 0; i < Width; i++) {
            for (int j = 0; j < Height; j++) {
                Vector2 xP = RungeKutta2(new Vector2(i,j));
                xP = xP.Constrain(0,0,Width-1,Height-1,1);
                if (_levelSet.GetDistance(xP) < 0) {
                    xP = _levelSet.FindClosest(xP);
                }
                xP = xP.Constrain(0,0,Width-1,Height-1,5);
                storeField[i,j] = VectorFuncs.CubicFloatField(xP,getField); ////////
            }
        }
    }

    private void SemiLagrangian() {
        for (int i = 0; i < Width; i++) {
            for (int j = 0; j < Height; j++) {
                Vector2 xPu = RungeKutta2(new Vector2(i-0.5f,j));
                xPu = xPu.Constrain(0,0,Width-1,Height-1,1);
                if (_levelSet.GetDistance(xPu) < 0) {
                    xPu = _levelSet.FindClosest(xPu);
                }
                xPu = xPu.Constrain(0,0,Width-1,Height-1,5);
                UVelHold[i,j] = VectorFuncs.CubicFloatField(xPu,UVel); ////////

                Vector2 xPv = RungeKutta2(new Vector2(i,j-0.5f));
                xPv = xPv.Constrain(0,0,Width-1,Height-1,1);
                if (_levelSet.GetDistance(xPv) < 0) {
                    xPv = _levelSet.FindClosest(xPv);
                }
                xPv = xPv.Constrain(0,0,Width-1,Height-1,5);
                VVelHold[i,j] = VectorFuncs.CubicFloatField(xPv,VVel); ////////
            }
        }

        for (int i = 0; i < Height; i++) {
            Vector2 xPul = RungeKutta2(new Vector2(Width - 0.5f,i));
            xPul = xPul.Constrain(0,0,Width-1,Height-1,1);
            if (_levelSet.GetDistance(xPul) < 0) {
                xPul = _levelSet.FindClosest(xPul);
            }
            xPul = xPul.Constrain(0,0,Width-1,Height-1,5);
            UVelHold[Width,i] = VectorFuncs.CubicFloatField(xPul,UVel); ////////
        }

        for (int i = 0; i < Width; i++) {
            Vector2 xPvl = RungeKutta2(new Vector2(i, Height - 0.5f));
            xPvl = xPvl.Constrain(0,0,Width-1,Height-1,1);
            if (_levelSet.GetDistance(xPvl) < 0) {
                xPvl = _levelSet.FindClosest(xPvl);
            }
            xPvl = xPvl.Constrain(0,0,Width-1,Height-1,5);
            VVelHold[i,Height] = VectorFuncs.CubicFloatField(xPvl,VVel); ////////
        }
    }

    //Forward Euler
    private void BodyForces() {
        for (int i = 0; i < Width; i++) {
            for (int j = 0; j < Height; j++) {
                UVelHold[i,j] += _uBodyforce * _timestep;
                VVelHold[i,j] += _vBodyforce * _timestep;
            }
        }

        for (int i = 0; i < Height; i++) {
            UVelHold[Width,i] += _uBodyforce * _timestep;
        }

        for (int i = 0; i < Width; i++) {
            VVelHold[i,Height] += _vBodyforce * _timestep;
        }
    }

    private void Project(int maxIter = 200) {
        // 1. Calculate negative divergence with modifications at solid wall boundaries
        ulong time = GetTicksUsec();
        CalculateRhs();
        ulong timen = GetTicksUsec();
        Print($"Calc RHS Takes {timen - time} micro-seconds");
       // Print("RHS");
        // Note: Matrix Solves should use Doubles, not Floats
        // 2. Set the entries of A (Adiag, Ax, Ay)
        CalculateLhs();
        time = GetTicksUsec();
        Print($"Calc LHS Takes {time - timen} micro-seconds");
       // Print("LHS");
        // 3. Construct the Preconditioner (MIC(0))
        CalculatePreconditioner();
        timen = GetTicksUsec();
        Print($"Calc Precon Takes {timen - time} micro-seconds");
       // Print("Preconditioner");


        _pressure.Clear();
        _rhs.CopyTo(_r);

        time = GetTicksUsec();
        Print($"Clear Takes {time - timen} micro-seconds");
        // 4. Solve Ap = b with PCG
        PcgAlgo(maxIter);
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
    private void CalculateRhs() {
        double scale = 1 / Dx;
        int position;
        for (int i = 0; i < Width; i++) {
            for (int j = 0; j < Height; j++) {
                position = i * Width + j;
                if (GetMaterialType(i,j) == Material.LIQUID) {
                    _rhs[position] = -scale * (UVelHold[i+1,j] - UVelHold[i,j]
                                        +VVelHold[i,j+1] - VVelHold[i,j]);
                    
                    if (GetMaterialType(i-1,j) == Material.SOLID) {_rhs[position] -= scale * (UVelHold[i,j] - GetUSolid(i,j));}
                    if (GetMaterialType(i+1,j) == Material.SOLID) {_rhs[position] += scale * (UVelHold[i+1,j] - GetUSolid(i+1,j));}

                    if (GetMaterialType(i,j-1) == Material.SOLID) {_rhs[position] -= scale * (VVelHold[i,j] - GetVSolid(i,j));}
                    if (GetMaterialType(i,j+1) == Material.SOLID) {_rhs[position] += scale * (VVelHold[i,j+1] - GetVSolid(i,j+1));}
                }
                else {
                    _rhs[position] = 0;
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
    private void CalculateLhs() {
        // Notes: Adiag is storing the coefficient for the relationship with ITSELF,
        // Ax and Ay are storing the coefficients for the relationships between two grid locations
        double scale = _timestep / (_density * Dx * Dx);
        int position;
        for (int i = 0; i < Width; i++) {
            for (int j = 0; j < Height; j++) {
                position = i * Width + j;
                _adiag[position] = 0;
                _ax[position] = 0;
                _ay[position] = 0;
                if (GetMaterialType(i,j) == Material.LIQUID) {
                    // x-axis
                    if (GetMaterialType(i-1,j) == Material.LIQUID) { _adiag[position] += scale; }
                    if (GetMaterialType(i+1,j) == Material.LIQUID) { _adiag[position] += scale; _ax[position] = -scale; }
               else if (GetMaterialType(i+1,j) == Material.EMPTY ) { _adiag[position] += scale; }
                    // y-axis
                    if (GetMaterialType(i,j-1) == Material.LIQUID) { _adiag[position] += scale; }
                    if (GetMaterialType(i,j+1) == Material.LIQUID) { _adiag[position] += scale; _ay[position] = -scale; }
               else if (GetMaterialType(i,j+1) == Material.EMPTY ) { _adiag[position] += scale; }
                    if (_adiag[position] == 0) {
                        throw new ArithmeticException($"Calculating A: \n{_adiag[position]} {_ax[position]} {_ay[position]}: {GetMaterialType(i-1,j)}, {GetMaterialType(i+1,j)}, {GetMaterialType(i,j-1)}, {GetMaterialType(i,j+1)}");
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
        for (int i = 0; i < Width; i++) {
            for (int j = 0; j < Height; j++) {
                position = i * Width + j;
                if (GetMaterialType(i,j) == Material.LIQUID) {
                    holder = _adiag[position] - Math.Pow(_ax[position-Width] * _precon[position-Width],2)
                                        - Math.Pow(_ay[position-1] * _precon[position-1],2)
                                        - tuning *(_ax[position-Width] * (_ay[position-Width]) * _precon[position-Width] * _precon[position-Width]
                                                 + _ay[position-1    ] * (_ax[position-1    ]) * _precon[position-1    ] * _precon[position-1    ]);
                    if (holder < safety * _adiag[position]) { holder = _adiag[position]; }
                    _precon[position] = 1 / Math.Sqrt(holder);
                    if (double.IsNaN(_precon[position])) {
                        throw new ArithmeticException($"Precon is NaN: {holder} =  {_adiag[position]} - {_ax[position-Width]} - {_ay[position-1]}");
                    }
                }
                else { _precon[position] = 0; } // more precisely, what should the default value be? 1 / 0?
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
        for (int i = 0; i < Width; i++) {
            for (int j = 0; j < Height; j++) {
                position = i * Width + j;
                if (GetMaterialType(i,j) == Material.LIQUID) {
                    holder = r[position] - _ax[position-Width] * _precon[position-Width] * _z[position - Width]
                                         - _ay[position-1] * _precon[position-1] * _z[position - 1];
                    _z[position] = holder * _precon[position];
                }
                else { _z[position] = 0; } // what should the default value be? r * precon? 0? 
            }
        }
        // solve Ltranspose z = q
        for (int i = Width - 1; i >= 0; i--) {
            for (int j = Height - 1; j >= 0; j--) {
                position = i * Width + j;
                if (GetMaterialType(i,j) == Material.LIQUID) {
                    holder = _z[position] - _ax[position] * _precon[position] * _z[position + Width]
                                         - _ay[position] * _precon[position] * _z[position + 1];
                    _z[position] = holder * _precon[position];
                }
                else { _z[position] = 0; }
            }
        }
        // ulong timen  = GetTicksUsec();
        // Print($"ApplyPrecon: {timen-time}");
    }

    private void ApplyPreconditioner() { // z = precon * r
        var sr = _r.Memory.Span;
        var spre = _precon.Memory.Span;
        var sax = _ax.Memory.Span;
        var say = _ay.Memory.Span;
        var sz = _z.Memory.Span;
        var sls = _levelSet.distance_field.Memory.Span;
        var pr = 0;

        while (pr < Height * Width) {
            if (sls[pr] > 0) { // material is liquid (needs revising)
                sz[pr] = spre[pr] * (sr[pr] - sax[pr - Width] * spre[pr - Width] * sz[pr - Width]
                                            - say[pr - 1    ] * spre[pr - 1    ] * sz[pr - 1    ]); 
            }
            else {
                sz[pr] = 0;
            }
            pr++;
        }

        pr = Height * Width - 1;

        while (pr >= 0) {
            if (sls[pr] > 0) {
                sz[pr] = spre[pr] * (sz[pr] - sax[pr] * spre[pr] * sz[pr + Width]
                                            - say[pr] * spre[pr] * sz[pr + 1    ]);
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

        // Vec.PointwiseMul(r,Adiag,z);
        // Vec.PointwiseMul(r,Ay,holdervec);

        var sr = r.Memory.Span;
        // var shv = holdervec.Memory.Span;
        var sad = _adiag.Memory.Span;
        var sax = _ax.Memory.Span;
        var say = _ay.Memory.Span;
        var sz = _z.Memory.Span;

        var pr = Width;
        var prip = Width << 1;
        var prin = 0;
        var prjp = Width + 1;
        var prjn = Width - 1;
        // var pad  = width;
        // var pax  = width;
        // var paxin = 0;
        // var pay = width;
        // var payjn = width - 1;

        while (prip < Width * Height)
        {
            sz[pr] = sr[pr  ] * sad[pr  ]
                   + sr[prip] * sax[pr  ]
                   + sr[prin] * sax[prin]
                   + sr[prjp] * say[pr  ]
                   + sr[prjn] * say[prjn];
        /*
            sz[pr] = sr[pr      ] * sad[pr      ]
                   + sr[pr+width] * sax[pr      ]
                   + sr[pr-width] * sax[pr-width]
                   + sr[pr+1    ] * say[pr      ]
                   + sr[pr-1    ] * say[pr-1    ];
        */
            
            pr++;
            prip++;
            prin++;
            prjp++;
            prjn++;
            // pad++;
            // pax++;
            // paxin++;
            // pay++;
            // payjn++;
        }
        // ulong timen  = GetTicksUsec();
        // Print($"ApplyA: {timen-time}");
    }

    private double Dotproduct(Vec<double> rhs,Vec<double> lhs) {
        // double sum = 0.0;
        // sum = rhs * lhs;

        // Span<double> span = lhs.Memory.Span;
        // Span<double> span2 = rhs.Memory.Span;
        // for (int pr = 0; pr < span.Length; pr++)
        // {
        //     sum += span[pr] * span2[pr];
        // }
        // return sum;

        // obviously slow, but extremely consistant (which I value more)
        // the ddot does 8-10, but with common jumps 100-1000X
        Vec.PointwiseMul(rhs,lhs,_holdervec);
        return _holdervec.Sum();

        // var sl = lhs.Memory.Span;
        // var sr = rhs.Memory.Span;
        // var pr = 0;
        // double sum = 0.0;

        // while (pr < sl.Length)
        // {
        //     sum += sl[pr] * sr[pr];
        //     pr++;
        // }
        // return sum;
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

    private unsafe void PcgAlgo(int maxIter = 200, double prevMaxR = 0) {
        // SETUP
        // tolerance tol
        double tol = 0.000001; // 10^-6
        // initial guess for pressure, residual r

        ApplyPreconditioner(); // z = precon * r
        _z.CopyTo(_s);
        
        double sigma = Dotproduct(_z,_r);
        
        // ITERATIONS
        for (int iter = 0; iter < maxIter; iter++) {
            ulong time = GetTicksUsec();

            ApplyA(_s);

            ulong timea = GetTicksUsec();

            // Vec.PointwiseMul(z,s,holdervec);
            // double alpha = sigma / holdervec.Sum();
            // double alpha = sigma / z.Dot(s);
            double alpha = sigma / Dotproduct(_z,_s);

            ulong timep = GetTicksUsec();

            // Regular Loop Takes 645 usec. This takes 120 usec (WOW)
            Vec.Mul(_s,alpha,_holdervec);
            _pressure.AddInplace(_holdervec);
            Vec.Mul(_z,alpha,_holdervec);
            _r.SubInplace(_holdervec);
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
            double maxR = _r.InfinityNorm();
            // double max_r = r.Max();
            // double max_r = r[0];
            // for (int i = 0; i < height * width; i++) {
            //     if (max_r < r[i]) {
            //         max_r = r[i];
            //     }
            // }


            if (maxR <= tol) {
                Print($"Iterations: {iter}, (Residual {maxR})");
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

            double sigmaNew = Dotproduct(_z,_r);
            // Vec.PointwiseMul(z,r,holdervec);
            // double _sigma = holdervec.Sum();
            // double _sigma = z.Dot(r);
            // double _sigma = z * r;

            ulong times = GetTicksUsec();

            alpha = sigmaNew / sigma;
            
            // Made it a lot more consistant (idk how, maybe not allocating more memory than needed?)
            // A bit slower (66-150 around vs 66 more frequent but with common spikes up to 10 msec)
            // 66 around a third of the time vs. 66 around a half of the time (with huge spikes)
            _s.MulInplace(alpha);
            _s.AddInplace(_z);
            // s = z + (alpha * s);

            sigma = sigmaNew;
            ulong timer = GetTicksUsec();
            Print($"AA: {timea - time}, al: {timep - timea}, p+r: {timen - timep}, max_r: {timeq-timen}, APre: {timeb - timeq}, dot: {times - timeb}, s: {timer - times} Total: {timer - time}");
        }

        Print($"Max Iterations Exceeded (Residual {prevMaxR})"); // report iteration limit exceeded
    }

    // Pressure Gradient Update : Given Pressure, Apply it
    private void PressureGradientUpdate() {
        float scale = _timestep / (_density * Dx);
        for (int i = 0; i < Width; i++) {
            for (int j = 0; j < Height; j++) {
                // x-axis
                if (GetMaterialType(i,j) == Material.LIQUID || GetMaterialType(i-1,j) == Material.LIQUID) {
                    if (GetMaterialType(i,j) == Material.SOLID || GetMaterialType(i-1,j) == Material.SOLID) { UVel[i,j] = GetUSolid(i,j); }
                    else { UVel[i,j] = UVelHold[i,j] - scale * (float)(_pressure[i*Width + j] - _pressure[(i-1) * Width + j]); }
                }
                else { UVel[i,j] = 0; }

                // y-axis
                if (GetMaterialType(i,j) == Material.LIQUID || GetMaterialType(i,j-1) == Material.LIQUID) {
                    if (GetMaterialType(i,j) == Material.SOLID || GetMaterialType(i,j-1) == Material.SOLID) { VVel[i,j] = GetVSolid(i,j); }
                    else { VVel[i,j] = VVelHold[i,j] - scale * (float)(_pressure[i*Width + j] - _pressure[i * Width + j-1]); }
                }
                else { VVel[i,j] = 0; }
            }
        }
    }

    //MaterialType: Return 0 for Solid, 1 for Fluid, 2 for Empty (Don't know how to check if it is empty yet)
    public Material GetMaterialType(int x, int y) {
        if (_levelSet.GetDistanceOnGrid(x,y) <= 0.1) {
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
    private void ExtrapolateField(float[,] floatField) {
        int[,] markers = new int[floatField.GetLength(0),floatField.GetLength(1)];
        List<ValueTuple<int,int> > wave = new List<ValueTuple<int,int>>();
        int i, j;

        for (i = 1; i < Width-1; i++) {
            for (j = 1; j < Height-1; j++) {
                if (double.IsNaN(floatField[i,j])) {
                    if (double.IsNaN(floatField[i-1,j]) && double.IsNaN(floatField[i+1,j]) && double.IsNaN(floatField[i,j-1]) && double.IsNaN(floatField[i,j+1])) {
                        markers[i,j] = int.MaxValue;
                        continue;
                    }
                    wave.Add((i,j));
                    markers[i,j] = 1;
                }
                else {
                    markers[i,j] = 0;
                }
            }
        }
        for (i = 0; i < Width; i++) {
            markers[i,0] = int.MaxValue;
            markers[i,Height-1] = int.MaxValue;
        }
        for (i = 0; i < Height; i++) {
            markers[0,i] = int.MaxValue;
            markers[Width-1,i] = int.MaxValue;
        }
        // to prevent the compiler from compiling this away
        int t = 0;
        while (t < wave.Count) {
            i = wave[t].Item1;
            j = wave[t].Item2;
            floatField[i,j] = 0;
            int total = 0;
            if (i   != 0      && markers[i-1,j] < markers[i,j]) { floatField[i,j] += floatField[i-1,j]; total++; }
            if (i+1 != Width  && markers[i+1,j] < markers[i,j]) { floatField[i,j] += floatField[i+1,j]; total++; }
            if (j   != 0      && markers[i,j-1] < markers[i,j]) { floatField[i,j] += floatField[i,j-1]; total++; }
            if (j+1 != Height && markers[i,j+1] < markers[i,j]) { floatField[i,j] += floatField[i,j+1]; total++; }
            floatField[i,j] /= total;
            if (total == 0) {
                Print("Total is Zero, add case");
            }

            if (i   != 0      && markers[i-1,j] == int.MaxValue) { wave.Add((i-1,j)); markers[i-1,j] = markers[i,j] + 1; }
            if (i+1 != Width  && markers[i+1,j] == int.MaxValue) { wave.Add((i+1,j)); markers[i+1,j] = markers[i,j] + 1; }
            if (j   != 0      && markers[i,j-1] == int.MaxValue) { wave.Add((i,j-1)); markers[i,j-1] = markers[i,j] + 1; }
            if (j+1 != Height && markers[i,j+1] == int.MaxValue) { wave.Add((i,j+1)); markers[i,j+1] = markers[i,j] + 1; }

            t++;
        }
    }

    private float CalculateTimestep() {
        return MaxAdvectTimestep();
    }

    //2nd Order Runge-Kutta Method
    private Vector2 RungeKutta2(Vector2 position) {
        Vector2 xMid = position - 0.5f * _timestep * VectorFuncs.LerpVelocityField(position.Constrain(0,0,Width-1,Height-1,1),
                                                                                   UVel, VVel);
        return position - _timestep * VectorFuncs.LerpVelocityField(xMid.Constrain(0,0,Width-1,Height-1,1), UVel, VVel);
    }

    private float MaxSpeed() {
        float maxu = UVel.Cast<float>().Max();
        float maxv = VVel.Cast<float>().Max();
        Print($"Max Speed: {Sqrt(maxu * maxu + maxv * maxv)}");
        return maxu * maxu + maxv * maxv;
    }

    private float MaxSpeedHold() {
        float maxu = UVelHold.Cast<float>().Max();
        float maxv = VVelHold.Cast<float>().Max();
        Print($"Max Speed: {Sqrt(maxu * maxu + maxv * maxv)}");
        return maxu * maxu + maxv * maxv;
    }

    //Time step for Semi-Langrangian
    private float MaxAdvectTimestep() {
        float velMax = Sqrt(MaxSpeed()) + Sqrt(5*Dx*Sqrt(_uBodyforce*_uBodyforce + _vBodyforce*_vBodyforce));
        return (5 * Dx) / (velMax);
        // that is the max for the advect step
    }
}
