package org.firstinspires.ftc.teamcode.odometry;

public class tracking {
    //constants
    public static final double TICKS_PER_REV = 8192;
    public static final double MAX_RPM = 300;
    public static double WHEEL_RADIUS = 0.688976; // in
    public static double GEAR_RATIO = 1; // output (wheel) speed / input (motor) speed
    public static double TRACK_WIDTH = 13; // in



    final double s_L; //left enc offset
    final double s_R; //right enc offset
    final double s_S; //center enc offset
    final double theta_R; //initial orientation

    int l_0;
    int r_0;
    int s_0;
    int l_1;
    int r_1;
    int s_1;
    double deltaS = 0; //inches traveled since last cycle
    double deltaR = 0;//inches traveled since last cycle
    double deltaL = 0;//inches traveled since last cycle
    double deltaL_R = 0; //inches traveled since last reset
    double deltaR_R = 0;//inches traveled since last reset
    double deltaS_R = 0;
    double theta_1 = 0; //current cycle orientation
    double theta_0 = 0; //prev cycle orientation
    double deltaTheta = 0;
    double[] deltaD_L = new double[2]; //local offset
    double theta_m = 0; //avg orientation
    double[] d_1 = new double[2]; // current global offset
    double[] d_0 = new double[2]; // previous global offset
    double[] deltaD = new double[2];

    public tracking(double s_L, double s_R, double s_S, double theta_R)
    {
        this.s_L = s_L;
        this.s_R = s_R;
        this.s_S = s_S;
        this.theta_R = theta_R;
    }


    public  double encoderTicksToInches(int ticks) {
        return WHEEL_RADIUS * 2 * Math.PI * GEAR_RATIO * ticks / TICKS_PER_REV;
    }

    public  double currentOrientation(double delta_L, double delta_R, double theta_R)
    {
        return theta_R + ((delta_L - delta_R)/(s_L + s_R));
    }
    public double deltaTheta(double theta_1, double theta_0){
        return theta_1 - theta_0;
    }
    public double[] deltaD_L(double theta_1, double deltaS, double deltaTheta, double deltaR){
        double[] deltaD_L = new double[]{deltaS, deltaR};
        if(deltaTheta != 0){
//            deltaD_L[0] = 2*Math.sin(theta_1/2) *( (deltaS/deltaTheta) + s_S);
//            deltaD_L[1] = 2*Math.sin(theta_1/2) *( (deltaR/deltaTheta) + s_R);

            deltaD_L[0] = 2*Math.sin(deltaTheta/2) *( (deltaS/deltaTheta) + s_S);
            deltaD_L[1] = 2*Math.sin(deltaTheta/2) *( (deltaR/deltaTheta) + s_R);
        }

        return deltaD_L;
    }
    public double theta_m(double theta_0, double delta_theta){
        return theta_0 + 0.5*delta_theta;
    }
    public double[] deltaD_1(double[] d_0, double[] deltaD){
        double[] deltaD_1 = new double[]{d_0[0],d_0[1]};

        deltaD_1[0] += deltaD[0];
        deltaD_1[1] += deltaD[1];

        return deltaD_1;
    }

    public double[] deltaD(double[] deltaD_L, double theta_m)
    {
        double[] deltaD = new double[2];

        double x1 = deltaD_L[0];
        double y1 = deltaD_L[1];

        double theta = -theta_m;

        deltaD[0] = Math.cos(theta) * x1 - Math.sin(theta) * y1;
        deltaD[1] = Math.sin(theta) * x1 + Math.cos(theta) * y1;

        return deltaD;
    }

    public void calculateValues(int leftEncTicks, int rightEncTicks, int centerEncTicks, int lEInitial, int rEInitial, int cEInitial)
    {
        // set current values to previous values
        theta_0 = theta_1;
        d_0[0] = d_1[0];
        d_0[1] = d_1[1];
        l_0 = l_1;
        r_0 = r_1;
        s_0 = s_1;


        // find distances traveled by each encoder
        l_1 = leftEncTicks;
        r_1 = rightEncTicks;
        s_1 = centerEncTicks;


        deltaS = encoderTicksToInches(s_1-s_0);
        deltaL = encoderTicksToInches(l_1-l_0);
        deltaR = encoderTicksToInches(r_1-r_0);

        // calculating change in enc values since reset
        deltaL_R = encoderTicksToInches(l_1-lEInitial);
        deltaR_R = encoderTicksToInches(r_1-rEInitial);
        deltaS_R = encoderTicksToInches(s_1-cEInitial);

        // set current orientation
        theta_1 = currentOrientation(deltaL_R, deltaR_R, theta_R);

        // calculate change in orientation
        deltaTheta = deltaTheta(theta_1, theta_0);

        // calculate change in local coordinates
        deltaD_L(theta_1, deltaS, deltaTheta, deltaR);

        // theta_m
        theta_m = theta_m(theta_0, deltaTheta);

        // calculate delta d
        deltaD_L = deltaD_L(theta_1, deltaS, deltaTheta, deltaR);
        deltaD = deltaD(deltaD_L, theta_m);

        // calculate new position
        d_1[0] = d_0[0] + deltaD[0];
        d_1[1] = d_0[1] + deltaD[1];
    }


}
