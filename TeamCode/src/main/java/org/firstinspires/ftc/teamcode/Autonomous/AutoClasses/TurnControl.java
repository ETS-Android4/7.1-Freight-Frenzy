package org.firstinspires.ftc.teamcode.Autonomous.AutoClasses;

public class TurnControl {
    //Declares Varibles
    double thetaError;
    double thetaProportionalMultiplier = .35;
    double thetaProportional;
    public double theta;
    double thetaSetPoint = 0;
    double thetaLastError = 0;
    double thetaDerivativeMultiplier = .24;
    double thetaDerivative;
    public double turnControl(double thetaendsetpoint, double thetaindegrees){
        //Turns the robots in incriments instead of going straight to the end setpoint
        //If our endsepoint is greater then where we want to go plus our turn incriment
        thetaSetPoint = thetaendsetpoint;
        //Theta PD
        thetaError = thetaSetPoint - thetaindegrees;
        thetaProportional = thetaError * thetaProportionalMultiplier;
        thetaDerivative = (thetaError - thetaLastError)* thetaDerivativeMultiplier;
        thetaLastError = thetaError;
        theta = thetaProportional + thetaDerivative;
        return theta;
    }
}