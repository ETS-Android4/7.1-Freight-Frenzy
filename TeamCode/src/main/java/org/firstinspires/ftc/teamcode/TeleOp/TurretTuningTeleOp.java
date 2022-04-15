package org.firstinspires.ftc.teamcode.TeleOp;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.GeneralRobotCode.FreightFrenzyHardwareMap;
import org.firstinspires.ftc.teamcode.GeneralRobotCode.Smoothing;
import org.firstinspires.ftc.teamcode.TurretClasses.TurretCombined;


//@Config
//@TeleOp

public class TurretTuningTeleOp extends LinearOpMode{

    public static double UPARMPM = .028;
    public static double UPARMDM = .02;
    public static double DNPM = .018;
    public static double DNDM = .015;
    public static double EXTENDSPEED = 20;
    public static double EXTENDP = .015;
    public static double EXTENDD = 0.01;
    public static double ROTATEP = 0.0001;
    public static double ROTATED = 0.0001;
    public static double EXTENDSET = 300;
    public static double VPIVOTSPEEDSET = 16;
    public static double VPIVOTSETPOINT = 1500;
    public static double ROTATESPEED = 500;
    public static double ROTATESET = 10;


    public double vPivotSetPoint = 1500, extendSetPoint = 0;
    public double x, y, z, initPOsitionOrder = 1, xOriginal;


    FreightFrenzyHardwareMap robot = new FreightFrenzyHardwareMap();
    TurretCombined CombinedTurret = new TurretCombined();
    Smoothing Smoothing = new Smoothing();
    FtcDashboard dashboard = FtcDashboard.getInstance();
    Telemetry dashboardTelemetry = dashboard.getTelemetry();



    @Override
    public void runOpMode(){
        robot.init(hardwareMap);
        CombinedTurret.turretHomingTrigger = true;


        waitForStart();

        while (opModeIsActive()){

            CombinedTurret.vPivotUpPm = UPARMPM;
            CombinedTurret.vPivotUpDM = UPARMDM;
            CombinedTurret.vPivotDnPM = DNPM;
            CombinedTurret.vPivotDnDM = DNDM;

            CombinedTurret.extendPM = EXTENDP;
            CombinedTurret.extendDM = EXTENDD;

            CombinedTurret.rotatePM = ROTATEP;
            CombinedTurret.extendDM = ROTATED;

            CombinedTurret.TurretCombinedMethod(EXTENDSET,EXTENDSPEED,ROTATESET,ROTATESPEED, VPIVOTSETPOINT,VPIVOTSPEEDSET, robot.TE_M.getCurrentPosition(), robot.TE_G.getState(), robot.TR_M.getCurrentPosition(), robot.TR_G.getState(), robot.TP_M.getCurrentPosition(), robot.TP_G.getState());

            robot.TE_M.setPower(CombinedTurret.extendFinalMotorPower);
            robot.TR_M.setPower(CombinedTurret.rotateFinalMotorPower);
            robot.TP_M.setPower(CombinedTurret.vPivotFinalMotorPower);

            x = Smoothing.SmoothDriveX(-Math.copySign(gamepad1.left_stick_x, gamepad1.left_stick_x * gamepad1.left_stick_x * gamepad1.left_stick_x));
            y = Smoothing.SmoothDriveY( -(Math.copySign(gamepad1.left_stick_y, gamepad1.left_stick_y * gamepad1.left_stick_y * gamepad1.left_stick_y)));
            z = Smoothing.SmoothDriveZ(  Math.copySign(gamepad1.right_stick_x, gamepad1.right_stick_x * gamepad1.right_stick_x * gamepad1.right_stick_x));




            //setting the possiblity of a slow speed on the drivetrain
            if(gamepad1.right_bumper){
                robot.LF_M.setPower(.4*((y)-x+(z)));//LF
                robot.LB_M.setPower(.4*((y)+x+(z)));//LB
                robot.RF_M.setPower(.4*(-((y)+x-(z))));//RF
                robot.RB_M.setPower(.4*(-((y)-x-(z))));//RB
            }else{
                robot.LF_M.setPower(.8*((y)-x+(.8*z)));//LF
                robot.LB_M.setPower(.8*((y)+x+(.8*z)));//LB
                robot.RF_M.setPower(.8*(-((y)+x-(.8*z))));//RF
                robot.RB_M.setPower(.8*(-((y)-x-(.8*z))));//RB
            }

            if(gamepad1.a || gamepad2.a){
                robot.RI_S.setPower(-1);
                robot.LI_S.setPower(1);
            }else if(gamepad1.b || gamepad2.b){
                robot.RI_S.setPower(.5);
                robot.LI_S.setPower(-.5);
            }else{
                robot.RI_S.setPower(0);
                robot.LI_S.setPower(0);
            }
            if(gamepad2.left_bumper){
                if(gamepad2.b){
                    robot.TC_M.setPower(0);
                }else if(gamepad2.a){
                    robot.TC_M.setPower(1);
                }else if(gamepad2.x){
                    robot.TC_M.setPower(-1);
                }else{
                    robot.TC_M.setPower(robot.TC_M.getPower() + gamepad2.right_stick_y);
                }
            }

            Telemetry();
        }

    }
    public void Telemetry(){

        dashboardTelemetry.addData("vPivotMotor Power", CombinedTurret.vPivotFinalMotorPower);
        dashboardTelemetry.addData("vPivot Modified Encoder", CombinedTurret.vPivotModifiedEncoder);
        dashboardTelemetry.addData("vPivot RAW Encoder", robot.TP_M.getCurrentPosition());

        dashboardTelemetry.addData("extend Motor Power", CombinedTurret.extendFinalMotorPower);
        dashboardTelemetry.addData("extend RAW Encoder", robot.TE_M.getCurrentPosition());
        dashboardTelemetry.addData("vPivot Degree", CombinedTurret.currentDegree);
        dashboardTelemetry.addData("extend speed", CombinedTurret.extendSpeed);
        dashboardTelemetry.addData("extend Modified Encoder", CombinedTurret.extendModifiedEncoder);
        dashboardTelemetry.addData("extend Set", CombinedTurret.extendSet);
        dashboardTelemetry.addData("turret Homing trigger", CombinedTurret.turretHomingTrigger);

        dashboardTelemetry.addData("rotate Speed", CombinedTurret.rotateSpeed);
        dashboardTelemetry.addData("roate Motor Power", CombinedTurret.rotateFinalMotorPower);
        dashboardTelemetry.addData("vPivot Speed", CombinedTurret.vPivotSpeed);
        dashboardTelemetry.addData("rotate Mod Encoder", CombinedTurret.rotateModifiedEncoder);
        dashboardTelemetry.addData("rotate SPeed Set", CombinedTurret.rotateSpeedSet);


        dashboardTelemetry.update();
        telemetry.update();
    }

}
