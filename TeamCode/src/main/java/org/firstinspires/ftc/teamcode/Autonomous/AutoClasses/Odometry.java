package org.firstinspires.ftc.teamcode.Autonomous.AutoClasses;

public class Odometry {
    //declares varibles and sets needed numbers
    double deltaE1, deltaE2, deltaE3;
    double e1Previous, e2Previous, e3Previous;
    final double COUNTS_PER_INCH = 236.5;
    double thetaChange, thetaInRadians = 0;
    double yCoordinatePosition, xCoordinatePosition;
    double e1Current, e2Current, e3Current;
    double deltaX;
    double deltaY;
    double deltaMiddlePos;
    double deltaPerpPos;
    double e2CenterOffSet = 4.25 * COUNTS_PER_INCH;//r
    double e2VertOffSet = 1;//rb
    double vertHeadingPivotPoint;
    double HorisontalHeadingPivotPoint;
    public void RadiusOdometry(double e1current, double e2current, double e3current){
        //writes parameters to variables for easy use in the method

        //finds the difference in the encoders from the last loop cycle
        deltaE1 = e1current - e1Previous;//ΔL
        deltaE2 = e2current - e2Previous;//ΔB
        deltaE3 = -e3current - e3Previous;//ΔR
        //finds the angle the robot has changed in radians
        thetaChange = (deltaE1 - deltaE3) / (2 * e2CenterOffSet);//Δ0
        //using the calculated change in angle we calculate the angle the robot is facing in radians
        thetaInRadians = thetaInRadians + thetaChange;

        deltaMiddlePos =(deltaE1 + deltaE3)/2;
        deltaPerpPos = deltaE2 - thetaChange;

        deltaX = deltaMiddlePos * Math.cos(thetaInRadians) - deltaPerpPos * Math.sin(thetaInRadians);
        deltaY = deltaMiddlePos * Math.sin(thetaInRadians) + deltaPerpPos * Math.cos(thetaInRadians);

        xCoordinatePosition = xCoordinatePosition + deltaX;
        yCoordinatePosition = yCoordinatePosition + deltaY;

        //set the encoder to a varible to use in the next loop cycle to calculate the change in position
        e1Previous = e1current;
        e2Previous = e2current;
        e3Previous = -e3current;
    }
    //returns the values to use in other classes
    public double odoXReturn(){return (xCoordinatePosition/COUNTS_PER_INCH);}
    public double odoYReturn(){return yCoordinatePosition/COUNTS_PER_INCH;}
    public double thetaINRadiansReturn(){return thetaInRadians;}
    public double thetaInDegreesReturn(){return Math.toDegrees(thetaInRadians);}//returns our angle in degrees with a value from 1-360
}
