package org.firstinspires.ftc.teamcode.TeleOp;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.GeneralRobotCode.FreightFrenzyHardwareMap;
import org.firstinspires.ftc.teamcode.GeneralRobotCode.Smoothing;
import org.firstinspires.ftc.teamcode.TurretClasses.TurretCombined;


//@Config
//@TeleOp

public class Ryan_Is_Amazing extends LinearOpMode{

    public static double yint = 720;
    public static double slope = 2;
    public static double rotateP = .00012;
    public static double rotateD = .0001;


    public double x, y, z;

    double teleOpExtendSet = 200, teleOpRotateSet = 0, teleOpVPivotSet = 1000;
    double intakeVPivotSet = 480, intakeRotateSet = 0, intakeExtendSet = 275;
    double teleOpExtendSpeedSet = 38, teleOpRotateSpeedSet = 2500, teleOpVPivotSpeedSet = 18;
    double lastDS = 5, timeStart = 0, CarouselMotor = 0, carouselP = .00012, lastTime = 0, lastCarouselE = 0;
    boolean oneLoop = false;
    boolean controllerY = false;
    double rotateChange = 0, rotateChangeLeft = 0;
    double startCEncoder = 0;
    public static double CarouselTopSpeed = 1, CaroselSlowSpeed = .38, CaroselSlowDist = 500, CaroselRampUpDist = 1000;




    FreightFrenzyHardwareMap robot = new FreightFrenzyHardwareMap();
    TurretCombined CombinedTurret = new TurretCombined();
    Smoothing Smoothing = new Smoothing();
    FtcDashboard dashboard = FtcDashboard.getInstance();
    Telemetry dashboardTelemetry = dashboard.getTelemetry();



    @Override
    public void runOpMode(){
        robot.init(hardwareMap);



        waitForStart();

        while (opModeIsActive()){

            CombinedTurret.rotatePM = rotateP;
            CombinedTurret.rotateDM = rotateD;
            //Drivetrain setpoints with smoothing and expontential code

            z = Smoothing.SmoothDriveX(Math.copySign(gamepad1.left_stick_x, gamepad1.left_stick_x * gamepad1.left_stick_x * gamepad1.left_stick_x));
            y = Smoothing.SmoothDriveY( -(Math.copySign(gamepad1.left_stick_y, gamepad1.left_stick_y * gamepad1.left_stick_y * gamepad1.left_stick_y)));
            x= Smoothing.SmoothDriveZ( - Math.copySign(gamepad1.right_stick_x, gamepad1.right_stick_x));


            //slow speed on the drivetrain
            if(gamepad1.right_bumper){
                robot.LF_M.setPower(.4*((y)-x+(z)));//LF
                robot.LB_M.setPower(.4*((y)+x+(z)));//LB
                robot.RF_M.setPower(.4*(-((y)+x-(z))));//RF
                robot.RB_M.setPower(.4*(-((y)-x-(z))));//RB
            }else{
                robot.LF_M.setPower(((y)-x+(.6*z)));//LF
                robot.LB_M.setPower(((y)+x+(.6*z)));//LB
                robot.RF_M.setPower((-((y)+x-(.6*z))));//RF
                robot.RB_M.setPower((-((y)-x-(.6*z))));//RB
            }

            //intaking/outtaking/off

            if(gamepad1.a || gamepad2.a || gamepad1.left_trigger > .05){//intake
                robot.RI_S.setPower(-.5);
                robot.LI_S.setPower(.5);
            }else if(gamepad1.b || gamepad2.b || gamepad1.left_bumper){//outtake
                robot.RI_S.setPower(.4);
                robot.LI_S.setPower(-.4);
            }else{//servos off
                robot.RI_S.setPower(0);
                robot.LI_S.setPower(0);
            }

                                //turret Presets
                if(gamepad2.y || gamepad1.y){//Resetting our intake position in case of any encoder drift
                    intakeVPivotSet = CombinedTurret.vPivotModifiedEncoder;
                    intakeExtendSet = CombinedTurret.extendModifiedEncoder;
                    intakeRotateSet = CombinedTurret.rotateModifiedEncoder;

                }else if(gamepad2.dpad_right || gamepad1.dpad_right) {//Alliance hub dropping preset
                    teleOpVPivotSet = 1550;
                    if (CombinedTurret.vPivotModifiedEncoder > 1000) {
                        teleOpRotateSet = intakeRotateSet - 1450;
                        if(CombinedTurret.rotateModifiedEncoder < -1000) {
                            teleOpExtendSet = 820;
                        }
                    }
                }else if(gamepad2.dpad_down || gamepad1.dpad_down) {//Intake position
                    if (Math.abs(intakeRotateSet - CombinedTurret.rotateModifiedEncoder) < 150 && Math.abs(intakeExtendSet - CombinedTurret.extendModifiedEncoder) < 100) {
                        teleOpVPivotSet = intakeVPivotSet;
                    }else{
                        if (CombinedTurret.vPivotModifiedEncoder > 800) {
                            if(Math.abs(intakeExtendSet - CombinedTurret.extendModifiedEncoder) < 800){
                                teleOpRotateSet = intakeRotateSet;
                            }
                            teleOpExtendSet = intakeExtendSet;

                        }else{
                            teleOpVPivotSet = 900;
                        }
                    }

                }else if(gamepad2.dpad_left || gamepad1.dpad_left){//Shared shipping hub intake position
                    teleOpVPivotSet = 800;
                    if (CombinedTurret.vPivotModifiedEncoder > 700) {
                        teleOpRotateSet = intakeRotateSet + 1540;
                        teleOpExtendSet = 0;
                    }

                }else if(gamepad2.dpad_up || gamepad1.dpad_up){//Mid alliance hub scoring position
                    teleOpVPivotSet = 1260;
                    if (CombinedTurret.vPivotModifiedEncoder > 1000) {
                        teleOpRotateSet = intakeRotateSet - 1530;
                        if(CombinedTurret.rotateModifiedEncoder < -800) {
                            teleOpExtendSet = 800;
                        }
                    }

                } else{//manual turret position setting
                    teleOpExtendSet = teleOpExtendSet - Smoothing.SmoothExtend(gamepad2.right_stick_y * 45);

                    rotateChange = Smoothing.SmoothRotate(gamepad2.right_trigger * 40);
                    //rotateChange = gamepad2.right_trigger * 40;
                    //rotateChangeLeft = gamepad2.left_trigger * -40;

                    rotateChangeLeft = Smoothing.SmoothRotateLeft(gamepad2.left_trigger * -40);

                    teleOpRotateSet = teleOpRotateSet + rotateChange + rotateChangeLeft;

                    teleOpVPivotSet = teleOpVPivotSet + (gamepad2.left_stick_y * -30);

                }
            if(gamepad2.right_bumper || gamepad2.left_bumper) {//carousel stuff
                teleOpVPivotSet = 2550;
                if(CombinedTurret.vPivotModifiedEncoder > 800){
                    teleOpExtendSet = 0;
                    teleOpRotateSet = 0;
                }

                if (oneLoop == false) {
                    timeStart = getRuntime();
                    oneLoop = true;
                    startCEncoder = robot.TC_M.getCurrentPosition();
                }
                if(robot.TC_M.getCurrentPosition() - startCEncoder < CaroselSlowDist){
                    CarouselMotor = CaroselSlowSpeed;
                }else{
                    CarouselMotor = CaroselSlowSpeed + (CarouselTopSpeed * (((robot.TC_M.getCurrentPosition() - startCEncoder) - CaroselSlowDist)/CaroselRampUpDist));

                }
                if(CarouselMotor > CarouselTopSpeed){
                    CarouselMotor = CarouselTopSpeed;
                }



            }else{
                CarouselMotor = 0;
                oneLoop = false;
            }


            if(gamepad1.y){
                controllerY = true;
            }else if(gamepad2.y){
                controllerY = true;
            }else{
                controllerY = false;
            }



            if(gamepad1.x || gamepad2.x){ //bypassing limits
                teleOpVPivotSet = CombinedTurret.VPivotLimits(teleOpExtendSet, teleOpVPivotSet, teleOpRotateSet, controllerY, true);
            }else{//normal mode
                teleOpVPivotSet = CombinedTurret.VPivotLimits(teleOpExtendSet, teleOpVPivotSet, teleOpRotateSet, controllerY, false);

                //setpoint limits
                if(teleOpVPivotSet > 2600){
                    teleOpVPivotSet = 2600;
                }else if(teleOpVPivotSet < 100){
                    teleOpVPivotSet = 100;
                }

                if(teleOpExtendSet > 920){
                    teleOpExtendSet = 920;
                }if(teleOpExtendSet < -40){
                    teleOpExtendSet = -40;
                }

                if(teleOpRotateSet > 5000){
                    teleOpRotateSet = 5000;
                }else if(teleOpRotateSet < -5000){
                    teleOpRotateSet = -5000;
                }
            }


            CombinedTurret.TurretCombinedMethod(teleOpExtendSet,teleOpExtendSpeedSet,teleOpRotateSet,teleOpRotateSpeedSet, teleOpVPivotSet,teleOpVPivotSpeedSet, robot.TE_M.getCurrentPosition(), robot.TE_G.getState(), robot.TR_M.getCurrentPosition(), robot.TR_G.getState(), robot.TP_M.getCurrentPosition(), robot.TP_G.getState());

            robot.TE_M.setPower(CombinedTurret.extendFinalMotorPower);
            robot.TR_M.setPower(CombinedTurret.rotateFinalMotorPower);
            robot.TP_M.setPower(CombinedTurret.vPivotFinalMotorPower);
            robot.TC_M.setPower(CarouselMotor);

            Telemetry();
            lastDS = robot.I_DS.getDistance(DistanceUnit.INCH);
            lastTime = getRuntime();
            lastCarouselE = robot.TC_M.getCurrentPosition();
        }

    }
    public void Telemetry(){

      //  dashboardTelemetry.addData("vPivotMotor Power", CombinedTurret.vPivotFinalMotorPower);
        dashboardTelemetry.addData("vPivot Modified Encoder", CombinedTurret.vPivotModifiedEncoder);
        dashboardTelemetry.addData("Rotate Modified Encoder", CombinedTurret.rotateModifiedEncoder);
      //  dashboardTelemetry.addData("extend Motor Power", CombinedTurret.extendFinalMotorPower);
      //  dashboardTelemetry.addData("extend speed", CombinedTurret.extendSpeed);
        dashboardTelemetry.addData("extend Modified Encoder", CombinedTurret.extendModifiedEncoder);
        dashboardTelemetry.addData("rotate Motor Power", robot.TR_M.getPower());
        dashboardTelemetry.addData("rotate Set", teleOpRotateSet);
        dashboardTelemetry.addData("rotate Speed", CombinedTurret.rotateSpeed);
        dashboardTelemetry.addData("carouselSpeed", CarouselMotor);
      //  dashboardTelemetry.addData("extend Set", CombinedTurret.extendSet);
      //  dashboardTelemetry.addData("turret Homing trigger", CombinedTurret.turretHomingTrigger);

        //telemetry.addData("intake Dist", robot.I_DS.getDistance(DistanceUnit.INCH));
        //telemetry.addData("rotate set", CombinedTurret.rotateSet);
        telemetry.addData("VPivot Mod Encoder", CombinedTurret.vPivotModifiedEncoder);
        telemetry.addData("extend Mod Encoder", CombinedTurret.extendModifiedEncoder);
        telemetry.addData("rotate mod encoder", CombinedTurret.rotateModifiedEncoder);
        telemetry.addData("Extend Set", teleOpExtendSet);
        telemetry.addData("VPivot Set", teleOpVPivotSet);
        telemetry.addData("rotateLeftChange", rotateChangeLeft);
        telemetry.addData("rotateChange right", rotateChange);
        //telemetry.addData("rotate RAW Encoder", robot.TR_M.getCurrentPosition());
        //telemetry.addData("carousel timer", getRuntime() - timeStart);
        //dashboardTelemetry.addData("carousel Speed", ((robot.TC_M.getCurrentPosition() - lastCarouselE)/(getRuntime() - lastTime)));
        //dashboardTelemetry.addData("carosel motor", CarouselMotor);


        dashboardTelemetry.update();
        telemetry.update();
    }

}
