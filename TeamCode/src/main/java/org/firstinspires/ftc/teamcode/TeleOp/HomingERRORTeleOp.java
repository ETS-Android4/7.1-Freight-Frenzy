package org.firstinspires.ftc.teamcode.TeleOp;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.GeneralRobotCode.FreightFrenzyHardwareMap;
import org.firstinspires.ftc.teamcode.GeneralRobotCode.Smoothing;
import org.firstinspires.ftc.teamcode.TurretClasses.TurretCombined;


@Config
@TeleOp
public class HomingERRORTeleOp extends LinearOpMode{



    public double x, y, z;

    double teleOpExtendSet = 0, teleOpRotateSet = 0, teleOpVPivotSet = 0;
    double intakeVPivotSet = 480, intakeRotateSet = 0, intakeExtendSet = 275;
    double teleOpExtendSpeedSet = 30, teleOpRotateSpeedSet = 2000, teleOpVPivotSpeedSet = 18;
    double lastDS = 5, timeStart = 0, CarouselMotor = 0;
    boolean oneLoop = false;


    FreightFrenzyHardwareMap robot = new FreightFrenzyHardwareMap();
    TurretCombined CombinedTurret = new TurretCombined();
    Smoothing Smoothing = new Smoothing();
  //  FtcDashboard dashboard = FtcDashboard.getInstance();
  //  Telemetry dashboardTelemetry = dashboard.getTelemetry();



    @Override
    public void runOpMode(){
        robot.init(hardwareMap);

    CombinedTurret.turretHomingTrigger = false;

        waitForStart();

        while (opModeIsActive()){

            CombinedTurret.turretHomingTrigger = false;




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
                robot.LF_M.setPower(((y)-x+(.8*z)));//LF
                robot.LB_M.setPower(((y)+x+(.8*z)));//LB
                robot.RF_M.setPower((-((y)+x-(.8*z))));//RF
                robot.RB_M.setPower((-((y)-x-(.8*z))));//RB
            }



                if(gamepad1.a || gamepad2.a){//intake
                    robot.RI_S.setPower(-.5);
                    robot.LI_S.setPower(.5);
                }else if(gamepad1.b || gamepad2.b){//outtake
                    robot.RI_S.setPower(.3);
                    robot.LI_S.setPower(-.3);
                }else{//servos off
                    robot.RI_S.setPower(0);
                    robot.LI_S.setPower(0);
                }



                    teleOpExtendSet = teleOpExtendSet - (gamepad2.right_stick_y * 50);
                    if(gamepad2.right_trigger > .05){
                        teleOpRotateSet = teleOpRotateSet + (gamepad2.right_trigger * 40);
                    }else if(gamepad2.left_trigger > .05){
                        teleOpRotateSet = teleOpRotateSet - (gamepad2.left_trigger * 40);
                    }

                    teleOpVPivotSet = teleOpVPivotSet + (gamepad2.left_stick_y * -30);


            if(gamepad2.right_bumper || gamepad2.left_bumper) {
                teleOpVPivotSet = 2550;
                if(CombinedTurret.vPivotModifiedEncoder > 800){
                    teleOpExtendSet = 0;
                }
                if (oneLoop == false) {
                    timeStart = getRuntime();
                    oneLoop = true;
                }
                if (getRuntime() - timeStart < 1) {
                    CarouselMotor = ((getRuntime() - timeStart) / .5) * -.4;

                } else {
                    CarouselMotor = -.8;
                }
            }else{
                CarouselMotor = 0;
                oneLoop = false;
            }


            CombinedTurret.TurretCombinedMethod(teleOpExtendSet,teleOpExtendSpeedSet,teleOpRotateSet,teleOpRotateSpeedSet, teleOpVPivotSet,teleOpVPivotSpeedSet, robot.TE_M.getCurrentPosition(), robot.TE_G.getState(), robot.TR_M.getCurrentPosition(), robot.TR_G.getState(), robot.TP_M.getCurrentPosition(), robot.TP_G.getState());

            robot.TE_M.setPower(CombinedTurret.extendFinalMotorPower);
            robot.TR_M.setPower(CombinedTurret.rotateFinalMotorPower);
            robot.TP_M.setPower(CombinedTurret.vPivotFinalMotorPower);
            robot.TC_M.setPower(CarouselMotor);

            Telemetry();
            lastDS = robot.I_DS.getDistance(DistanceUnit.INCH);
        }

    }
    public void Telemetry(){

      //  dashboardTelemetry.addData("vPivotMotor Power", CombinedTurret.vPivotFinalMotorPower);
      //  dashboardTelemetry.addData("vPivot Modified Encoder", CombinedTurret.vPivotModifiedEncoder);

      //  dashboardTelemetry.addData("extend Motor Power", CombinedTurret.extendFinalMotorPower);
      //  dashboardTelemetry.addData("extend speed", CombinedTurret.extendSpeed);
      //  dashboardTelemetry.addData("extend Modified Encoder", CombinedTurret.extendModifiedEncoder);
      //  dashboardTelemetry.addData("extend Set", CombinedTurret.extendSet);
      //  dashboardTelemetry.addData("turret Homing trigger", CombinedTurret.turretHomingTrigger);

        telemetry.addData("intake Dist", robot.I_DS.getDistance(DistanceUnit.INCH));
        telemetry.addData("rotate set", CombinedTurret.rotateSet);
        telemetry.addData("VPivot Mod Encoder", CombinedTurret.vPivotModifiedEncoder);
        telemetry.addData("extend Mod Encoder", CombinedTurret.extendModifiedEncoder);
        telemetry.addData("rotate mod encoder", CombinedTurret.rotateModifiedEncoder);
        telemetry.addData("rotate RAW Encoder", robot.TR_M.getCurrentPosition());
        telemetry.addData("carousel timer", getRuntime() - timeStart);


     //   dashboardTelemetry.update();
        telemetry.update();
    }

}
