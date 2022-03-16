package org.firstinspires.ftc.teamcode.Autonomous.AutoClasses.TestCode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Autonomous.AutoClasses.Odometry;
import org.firstinspires.ftc.teamcode.Autonomous.AutoClasses.SpeedClass;
import org.firstinspires.ftc.teamcode.Autonomous.AutoClasses.TurnControl;
import org.firstinspires.ftc.teamcode.GeneralRobotCode.FreightFrenzyHardwareMap;

//@Autonomous

public class RadianAutoTest extends LinearOpMode {
    FreightFrenzyHardwareMap robot = new FreightFrenzyHardwareMap();
    SpeedClass SpeedClass = new SpeedClass();
    RadianDirectionCalcClass RadDirectionClass = new RadianDirectionCalcClass();
    TurnControl TurnControl = new TurnControl();
    Odometry OdoClass = new Odometry();
    /*ExtendClass ExtendClass = new ExtendClass();
    VPivotClass VPivotClass = new VPivotClass();
    RotateClass RotateClass = new RotateClass();*/

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
    double rotateSetpoint;
    double rotateSpeed;
    double extendSetpoint;
    double extendSpeed;
    double VPivotSetpoint;
    double VPivotSpeed;
    double timepassed2;
    double initPOsitionOrder = 1;
    double action;

    @Override

    public void runOpMode() {
        robot.init(hardwareMap);
        while (!opModeIsActive()) {


        }
        waitForStart();
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

                //Exits once the robot is a certain distance and angle away
                if (action == 1) {
                    thetaSetpoint = 0;
                    accelerationDistance = .2;
                    decelerationDistance = 8;
                    slowMoveSpeed = 3.85;
                    slowMovedDistance = 1;
                    thetaDeccelerationDegree = 2;
                    thetaTargetSpeed = 1;
                    xSetpoint = 40;
                    ySetpoint = 40;
                    thetaSetpoint = 0;
                    targetSpeed = 15;
                    //Exits once the robot is a certain distance and angle away
                    if (RadDirectionClass.distanceFromReturn() <= 1 && breakout != 0 && (OdoClass.thetaInDegreesReturn() < 1 && OdoClass.thetaInDegreesReturn() > -1)) {
                        StopMotors();
                        action =  1;
                        startPointX = OdoClass.odoXReturn();
                        startPointY = OdoClass.odoYReturn();
                        breakout = 0;
                    } else {
                        breakout = 1;
                    }
                }

                else {
                    stopProgram = 1;
                }
                //Runs all of our equations each loop cycle
                Movement(xSetpoint, ySetpoint, thetaSetpoint, targetSpeed, thetaTargetSpeed, thetaDeccelerationDegree, slowMoveSpeed, accelerationDistance, decelerationDistance, slowMovedDistance);
             //   RotateClass.RotateAutoMethod(rotateSetpoint, rotateSpeed, robot.TR_M.getCurrentPosition(), robot.TR_G.getState());
               // ExtendClass.ExtendAutoMethod(extendSetpoint, extendSpeed, robot.TE_M.getCurrentPosition(), robot.TE_G.getState());
                //VPivotClass.VPivotAutoMethod(VPivotSetpoint, VPivotSpeed, robot.TP_P.getVoltage());
                PowerSetting();
                Telemetry();
            }
            StopMotors();

        }


    public void Telemetry() {
        //Displays telemetry
        telemetry.addData("Odo X", OdoClass.odoXReturn());
        telemetry.addData("Odo Y", OdoClass.odoYReturn());
        telemetry.addData("Theta Angle", OdoClass.thetaInDegreesReturn());
        telemetry.addData("X", RadDirectionClass.XReturn());
        telemetry.addData("Y", RadDirectionClass.YReturn());
        telemetry.addData("Theta", TurnControl.theta);
        telemetry.addData("ThetaSpeedSetpoint", SpeedClass.thetaSpeedSetpoint());
        telemetry.addData("SlowMoveSpeed", slowMoveSpeed);
        telemetry.addData("slowMovedDistance", slowMovedDistance);
        telemetry.addData("Distance", RadDirectionClass.distanceReturn());
        telemetry.addData("Distance From", RadDirectionClass.distanceFromReturn());
        telemetry.addData("Speed Setpoint", SpeedClass.speedSetpoint());
        telemetry.addData("Speed", SpeedClass.SpeedReturn());
        telemetry.addData("time", getRuntime());
        telemetry.addData("Distance Delta", SpeedClass.DistanceDelta());
        telemetry.addData("XSetpoint", RadDirectionClass.XSetpointReturn());
        telemetry.addData("YSetpoint", RadDirectionClass.YSetpointReturn());
        telemetry.addData("LF_Power", robot.LF_M.getPower());
        telemetry.addData("RF_Power", robot.RF_M.getPower());
        telemetry.addData("LF_Direction", RadDirectionClass.LF_M_DirectionReturn());
        telemetry.addData("Motor Power Ratio", RadDirectionClass.motorPowerRatioReturn());
        telemetry.addData("Action", action);
        telemetry.addData("E1", robot.LF_M.getCurrentPosition());
        telemetry.addData("E2", robot.LB_M.getCurrentPosition());
        telemetry.addData("E3", robot.RF_M.getCurrentPosition());
        //telemetry.addData("PT", robot.TP_P.getVoltage());
        telemetry.update();
    }

    public void Movement(double endpointx, double endpointy, double thetasetpoint, double targetspeed, double thetaTargetSpeed, double thetaDeccelerationDegree, double slowMoveSpeed, double accelerationdistance, double deccelerationdistance, double slowMovedDistance) {
        OdoClass.RadiusOdometry(robot.LF_M.getCurrentPosition(), robot.LB_M.getCurrentPosition(), robot.RF_M.getCurrentPosition());
        TurnControl.turnControl(thetaSetpoint, OdoClass.thetaInDegreesReturn());
        RadDirectionClass.DirectionCalc(startPointX, startPointY, endpointx, endpointy, OdoClass.odoXReturn(), OdoClass.odoYReturn(), TurnControl.theta, OdoClass.thetaINRadiansReturn());
        SpeedClass.MotionProfile(targetspeed, accelerationdistance, deccelerationdistance, slowMovedDistance, RadDirectionClass.distanceReturn(), RadDirectionClass.distanceFromReturn(), slowMoveSpeed, thetaDeccelerationDegree, thetasetpoint, thetaTargetSpeed, OdoClass.thetaInDegreesReturn());
        SpeedClass.SpeedCalc(OdoClass.odoXReturn(), OdoClass.odoYReturn(), OdoClass.thetaInDegreesReturn(), getRuntime(), SpeedClass.speedSetpoint(), SpeedClass.thetaSpeedSetpoint());
    }

    public void PowerSetting() {

      //  robot.TR_M.setPower(RotateClass.rotateMotorPower);
       // robot.TE_M.setPower(ExtendClass.ExtendMotorPower);
      // robot.TP_M.setPower(VPivotClass.FinalMotorPower);
        robot.LF_M.setPower(RadDirectionClass.LF_M_DirectionReturn() * (SpeedClass.SpeedReturn()));
        robot.LB_M.setPower(RadDirectionClass.LB_M_DirectionReturn() * (SpeedClass.SpeedReturn() ));
       robot.RF_M.setPower(RadDirectionClass.RF_M_DirectionReturn() * (SpeedClass.SpeedReturn()));
      robot.RB_M.setPower(RadDirectionClass.RB_M_DirectionReturn() * (SpeedClass.SpeedReturn() ));

    }

    public void StopMotors() {
        robot.LF_M.setPower(0);
        robot.LB_M.setPower(0);
        robot.RF_M.setPower(0);
        robot.RB_M.setPower(0);
    }
}