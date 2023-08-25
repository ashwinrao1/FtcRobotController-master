package org.firstinspires.ftc.teamcode.odometry;

public class drive {
     double FRBLPOWER;
     double FLBRPOWER;

    public drive(double angle, double distance){
        FRBL(angle);
        FLBR(angle);

    }
    public void FRBL(double angle){
        double FRBLPOWER = Math.sin(angle - 0.25*Math.PI);

    }
    public void FLBR(double angle){
        double FLBRPOWER = Math.sin(angle + 0.25*Math.PI);

    }
    public static void main(String[] args){
        drive x = new drive(90, 90);
        System.out.println(x.FLBRPOWER);
        System.out.println(x.FRBLPOWER);
    }
}
