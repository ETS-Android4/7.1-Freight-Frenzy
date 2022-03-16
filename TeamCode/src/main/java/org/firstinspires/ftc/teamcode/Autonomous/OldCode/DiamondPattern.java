package org.firstinspires.ftc.teamcode.Autonomous.OldCode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Autonomous.AutoClasses.DirectionCalcClass;
import org.firstinspires.ftc.teamcode.Autonomous.AutoClasses.Odometry;
import org.firstinspires.ftc.teamcode.Autonomous.AutoClasses.SpeedClass;
import org.firstinspires.ftc.teamcode.Autonomous.AutoClasses.TestCode.Just_DrivetrainHardware;
import org.firstinspires.ftc.teamcode.Autonomous.AutoClasses.TurnControl;

//@Autonomous
//@Config
public class DiamondPattern extends LinearOpMode {
    Just_DrivetrainHardware robot = new Just_DrivetrainHardware();
    SpeedClass SpeedCalc = new SpeedClass();
    DirectionCalcClass DirectionClass = new DirectionCalcClass();
    TurnControl TurnControl = new TurnControl();
    Odometry OdoClass = new Odometry();
    //Uses Vuforia Developer Code
    //Declares Varibles
    double breakout;
    double Detected;
    double startPointX;
    double startPointY;
    double lastEndPointY;
    double justTurn;
    double timepassed;
    double lastEndPointX;
    double xSetpoint;
    double ySetpoint;
    double turnIncriments;
    double onlyDriveMotors;
    double thetaSetpoint;
    double loopcount;
    double accelerationDistance;
    double slowMoveSpeed;
    double decelerationDistance;
    double slowMovedDistance;
    double thetaTargetSpeed;
    double thetaDeccelerationDegree;
    double targetSpeed;
    double stopProgram;
    double timepassed2;

    double action;
    public static double SpeedDM = .0035;
    public static double SpeedPM = .0035;
    public static double ThetaPM = .003;
    public static double ThetaDM = 0;
    public static double yPM;
    public static double yDM;
    public static double xPM;
    public static double xDM;

    FtcDashboard dashboard = FtcDashboard.getInstance();
    Telemetry dashboardTelemetry = dashboard.getTelemetry();
    @Override
    public void runOpMode() {
        robot.init(hardwareMap);

        waitForStart();


        //DirectionClass.xDM = xDM;
        //DirectionClass.xPM = xPM;
        //DirectionClass.yDM = yDM;
        //DirectionClass.yPM = yPM;
        //Shuts down Tensor Flow
        //Sets our intial varible setpoints
        action = 1;
        startPointX = 0;
        startPointY = 0;
        stopProgram = 0;
        //Depending on the ring stack we change our intake to diffrent heights to be able to reach the top of the stack
        //Enters our 1 loop system, will exit once all actions are done
        while (opModeIsActive() && stopProgram == 0) {
            //Moves to first power shot shooting position
            SpeedCalc.thetaDM = ThetaDM;
            SpeedCalc.thetaPM = ThetaPM;
            SpeedCalc.speedDM = SpeedDM;
            SpeedCalc.speedPM = SpeedPM;

            if (action == 1) {
                xSetpoint = 30;
                ySetpoint = 0;
                thetaSetpoint = 0;
                targetSpeed = 40;
                accelerationDistance = 0.025;
                decelerationDistance = 1.5;
                slowMoveSpeed = 4;
                slowMovedDistance = 1.5;
                thetaDeccelerationDegree = 1;
                thetaTargetSpeed = 10;
                //Exits once the robot is a certain distance and angle away
                if (DirectionClass.distanceFromReturn() <= .25 && breakout != 0 && (OdoClass.thetaInDegreesReturn() < 1 && OdoClass.thetaInDegreesReturn() > -1)) {
                    StopMotors();
                    action = 2;
                    startPointX = OdoClass.odoXReturn();
                    startPointY = OdoClass.odoYReturn();
                    breakout = 0;
                } else {
                    breakout = 1;
                }
            }
            else if (action == 2) {
                xSetpoint = 0;
                ySetpoint = 0;
                thetaSetpoint = 0;
                targetSpeed = 20;
                //Exits once the robot is a certain distance and angle away
                if (DirectionClass.distanceFromReturn() <= .375 && breakout != 0 && (OdoClass.thetaInDegreesReturn() < 1 && OdoClass.thetaInDegreesReturn() > -1)) {
                    StopMotors();
                    action = 3;
                    startPointX = OdoClass.odoXReturn();
                    startPointY = OdoClass.odoYReturn();
                    breakout = 0;
                } else {
                    breakout = 1;
                }
            }
            else if (action == 3) {
                xSetpoint = 0;
                ySetpoint = 20;
                thetaSetpoint = 0;
                targetSpeed = 40;
                //Exits once the robot is a certain distance and angle away
                if (DirectionClass.distanceFromReturn() <= .25 && breakout != 0 && (OdoClass.thetaInDegreesReturn() < 1 && OdoClass.thetaInDegreesReturn() > -1)) {
                    StopMotors();
                    action = 4;
                    startPointX = OdoClass.odoXReturn();
                    startPointY = OdoClass.odoYReturn();
                    breakout = 0;
                } else {
                    breakout = 1;
                }
            }
            else if (action == 4) {
                xSetpoint = 0;
                ySetpoint = 0;
                thetaSetpoint = 0;
                targetSpeed = 20;
                //Exits once the robot is a certain distance and angle away
                if (DirectionClass.distanceFromReturn() <= .25 && breakout != 0 && (OdoClass.thetaInDegreesReturn() < 1 && OdoClass.thetaInDegreesReturn() > -1)) {
                    StopMotors();
                    action = 1;
                    startPointX = OdoClass.odoXReturn();
                    startPointY = OdoClass.odoYReturn();
                    breakout = 0;
                } else {
                    breakout = 1;
                }
            }



            //If nothing else to do, stop the program
            else {
                stopProgram = 1;
            }
            //Runs all of our equations each loop cycle
            Movement(xSetpoint, ySetpoint, thetaSetpoint, targetSpeed, thetaTargetSpeed, thetaDeccelerationDegree, slowMoveSpeed, accelerationDistance, decelerationDistance, slowMovedDistance);
            PowerSetting();
            Telemetry();
        }
        StopMotors();

    }

    public void Telemetry() {
        //Displays telemetry

        dashboardTelemetry.addData("Speed", SpeedCalc.speed);
        dashboardTelemetry.addData("SpeedDM", SpeedDM);
        dashboardTelemetry.addData("SpeedPM", SpeedPM);
        dashboardTelemetry.addData("ThetaSpeed", SpeedCalc.ThetaSpeed());
        dashboardTelemetry.addData("Distance_From", DirectionClass.distanceFromReturn());
        dashboardTelemetry.addData("Actual Speed", SpeedCalc.CurrentSpeed());

        telemetry.addData("Odo X", OdoClass.odoXReturn());
        telemetry.addData("Odo Y", OdoClass.odoYReturn());
        telemetry.addData("Theta Angle", OdoClass.thetaInDegreesReturn());
        telemetry.addData("X", DirectionClass.XReturn());
        telemetry.addData("Y", DirectionClass.YReturn());
        telemetry.addData("Theta", TurnControl.theta);
        telemetry.addData("SlowMoveSpeed", slowMoveSpeed);
        telemetry.addData("slowMovedDistance", slowMovedDistance);
        telemetry.addData("Distance", DirectionClass.distanceReturn());
        telemetry.addData("Distance From", DirectionClass.distanceFromReturn());
        telemetry.addData("Speed Setpoint", SpeedCalc.speedSetpoint());
        telemetry.addData("Speed", SpeedCalc.SpeedReturn());
        telemetry.addData("time", getRuntime());
        telemetry.addData("Distance Delta", SpeedCalc.DistanceDelta());
        telemetry.addData("XSetpoint", DirectionClass.XSetpointReturn());
        telemetry.addData("YSetpoint", DirectionClass.YSetpointReturn());
        telemetry.addData("LF_Power", robot.LF_M.getPower());
        telemetry.addData("RF_Power", robot.RF_M.getPower());
        telemetry.addData("LF_Direction", DirectionClass.LF_M_DirectionReturn());
        telemetry.addData("Motor Power Ratio", DirectionClass.motorPowerRatioReturn());
        telemetry.addData("Action", action);
        telemetry.update();
        dashboardTelemetry.update();
    }

    public void Movement(double endpointx, double endpointy, double thetasetpoint, double targetspeed, double thetaTargetSpeed, double thetaDeccelerationDegree, double slowMoveSpeed, double accelerationdistance, double deccelerationdistance, double slowMovedDistance) {
        OdoClass.RadiusOdometry(robot.LF_M.getCurrentPosition(), robot.LB_M.getCurrentPosition(), robot.RF_M.getCurrentPosition());
        TurnControl.turnControl(thetaSetpoint, OdoClass.thetaInDegreesReturn());
        DirectionClass.DirectionCalc(startPointX, startPointY, endpointx, endpointy, OdoClass.odoXReturn(), OdoClass.odoYReturn(), TurnControl.theta);
        SpeedCalc.MotionProfile(targetspeed, accelerationdistance, deccelerationdistance, slowMovedDistance, DirectionClass.distanceReturn(), DirectionClass.distanceFromReturn(), slowMoveSpeed, thetaDeccelerationDegree, thetasetpoint, thetaTargetSpeed, OdoClass.thetaInDegreesReturn());
        SpeedCalc.SpeedCalc(OdoClass.odoXReturn(), OdoClass.odoYReturn(), OdoClass.thetaInDegreesReturn(), getRuntime(), SpeedCalc.speedSetpoint(), SpeedCalc.thetaSpeedSetpoint());
    }

    public void PowerSetting() {


        robot.LF_M.setPower(DirectionClass.LF_M_DirectionReturn() * (SpeedCalc.SpeedReturn()));
        robot.LB_M.setPower(DirectionClass.LB_M_DirectionReturn() * (SpeedCalc.SpeedReturn()));
        robot.RF_M.setPower(DirectionClass.RF_M_DirectionReturn() * (SpeedCalc.SpeedReturn()));
        robot.RB_M.setPower(DirectionClass.RB_M_DirectionReturn() * (SpeedCalc.SpeedReturn()));

    }

    public void StopMotors() {
        robot.LF_M.setPower(0);
        robot.LB_M.setPower(0);
        robot.RF_M.setPower(0);
        robot.RB_M.setPower(0);
    }
}