package org.firstinspires.ftc.teamcode.TeleOp;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.Autonomous.AutoClasses.Odometry;
import org.firstinspires.ftc.teamcode.GeneralRobotCode.FreightFrenzyHardwareMap;
import org.firstinspires.ftc.teamcode.GeneralRobotCode.Smoothing;
import org.firstinspires.ftc.teamcode.TurretClasses.TurretCombined;


@Config
@TeleOp

public class FieldCentricBlueTeleOp extends LinearOpMode{



    public double x, y, z;

    double teleOpExtendSet = 200, teleOpRotateSet = 0, teleOpVPivotSet = 1000;
    double intakeVPivotSet = 480, intakeRotateSet = 0, intakeExtendSet = 275;
    double teleOpExtendSpeedSet = 30, teleOpRotateSpeedSet = 2000, teleOpVPivotSpeedSet = 18;
    double lastDS = 5, timeStart = 0, CarouselMotor = 0, carouselP = .00012, lastTime = 0, lastCarouselE = 0;
    boolean oneLoop = false;


    FreightFrenzyHardwareMap robot = new FreightFrenzyHardwareMap();
    TurretCombined CombinedTurret = new TurretCombined();
    Smoothing Smoothing = new Smoothing();
    Odometry OdoClass = new Odometry();
    FtcDashboard dashboard = FtcDashboard.getInstance();
    Telemetry dashboardTelemetry = dashboard.getTelemetry();

    double setpointAngle = 0, setpointAngleDeg = 0, finalAngle = 0, finalX = 0, finalY = 0, LF_M_DIR = 0, LB_M_DIR = 0, RF_M_DIR = 0,RB_M_DIR = 0;
    double motorPowerRatio = 0, speed = 0;

    @Override
    public void runOpMode(){
        robot.init(hardwareMap);



        waitForStart();

        while (opModeIsActive()){

            //getting exponential joystick control for the drivetrain

            OdoClass.RadiusOdometry(robot.LF_M.getCurrentPosition(), robot.LB_M.getCurrentPosition(), robot.RF_M.getCurrentPosition());
            x = Smoothing.SmoothDriveX(-Math.copySign(gamepad1.left_stick_x, gamepad1.left_stick_x * gamepad1.left_stick_x * gamepad1.left_stick_x));
            y = Smoothing.SmoothDriveY( -(Math.copySign(gamepad1.left_stick_y, gamepad1.left_stick_y * gamepad1.left_stick_y * gamepad1.left_stick_y)));
            z = Smoothing.SmoothDriveZ(  Math.copySign(gamepad1.right_stick_x, gamepad1.right_stick_x * gamepad1.right_stick_x * gamepad1.right_stick_x));


            setpointAngle = Math.atan2(x, y);
            setpointAngleDeg = Math.toDegrees(setpointAngle);

            if(x == 0){
                if(y < 0){
                    setpointAngleDeg = 180;
                }else{
                    setpointAngleDeg = 0;
                }
            }
            finalAngle = setpointAngle + OdoClass.thetaINRadiansReturn();
            finalX = Math.sin(finalAngle) * 1;
            finalY = Math.cos(finalAngle) * 1;

        if(Math.abs(gamepad1.left_stick_x) <= 0.05 && Math.abs(gamepad1.left_stick_y) <= .05) {
            finalX = 0;
            finalY = 0;
        }

                LF_M_DIR = (.4*((finalY)-finalX+(z)));//LF
                LB_M_DIR = (.4*((finalY)+finalX+(z)));//LB
                RF_M_DIR = (.4*(-((finalY)+finalX-(z))));//RF
                RB_M_DIR = (.4*(-((finalY)-finalX-(z))));//RB
            motorPowerRatio = Math.max(Math.max(Math.abs(RF_M_DIR), Math.abs(RB_M_DIR)), Math.max(Math.abs(LF_M_DIR), Math.abs(LB_M_DIR)));

            LF_M_DIR = LF_M_DIR/motorPowerRatio;
            LB_M_DIR = LB_M_DIR/motorPowerRatio;
            RF_M_DIR = RF_M_DIR/motorPowerRatio;
            RB_M_DIR = RB_M_DIR/motorPowerRatio;
            speed = Math.hypot(x, y) + Math.abs(z);
            if(speed >= 1){
                speed = 1;
            }
            if(gamepad1.right_bumper){
                speed = .6;
            }
            robot.LF_M.setPower(LF_M_DIR*speed);
            robot.LB_M.setPower(LB_M_DIR*speed);
            robot.RF_M.setPower(RF_M_DIR*speed);
            robot.RB_M.setPower(RB_M_DIR*speed);




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

                                //turret Presets
                if(gamepad2.y){//Resetting our intake position in case of any encoder drift
                    intakeVPivotSet = CombinedTurret.vPivotModifiedEncoder;
                    intakeExtendSet = CombinedTurret.extendModifiedEncoder;
                    intakeRotateSet = CombinedTurret.rotateModifiedEncoder;

                }else if(gamepad2.dpad_right) {//Alliance hub dropping preset
                    teleOpVPivotSet = 1600;
                    if (CombinedTurret.vPivotModifiedEncoder > 1000) {
                        teleOpRotateSet = intakeRotateSet + 1300;
                        if(CombinedTurret.rotateModifiedEncoder > 700) {
                            teleOpExtendSet = 1200;
                        }
                    }
                }else if(gamepad2.dpad_down) {//Intake position


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

                }else if(gamepad2.dpad_left){//Shared shipping hub intake position
                    teleOpVPivotSet = 800;
                    if (CombinedTurret.vPivotModifiedEncoder > 700) {
                        teleOpRotateSet = intakeRotateSet -1200;
                        teleOpExtendSet = 0;
                    }

                }else if(gamepad2.dpad_up){//Mid alliance hub scoring position
                    teleOpVPivotSet = 1260;
                    if (CombinedTurret.vPivotModifiedEncoder > 1000) {
                        teleOpRotateSet = intakeRotateSet + 1300;
                        if(CombinedTurret.rotateModifiedEncoder > 700) {
                            teleOpExtendSet = 900;
                        }
                    }

                } else{//manual turret position setting
                    teleOpExtendSet = teleOpExtendSet - (gamepad2.right_stick_y * 50);
                    if(gamepad2.right_trigger > .05){
                        teleOpRotateSet = teleOpRotateSet + (gamepad2.right_trigger * 40);
                    }else if(gamepad2.left_trigger > .05){
                        teleOpRotateSet = teleOpRotateSet - (gamepad2.left_trigger * 40);
                    }

                    teleOpVPivotSet = teleOpVPivotSet + (gamepad2.left_stick_y * -30);

                }
            if(gamepad2.right_bumper || gamepad2.left_bumper) {
                teleOpVPivotSet = 2550;
                if(CombinedTurret.vPivotModifiedEncoder > 800){
                    teleOpExtendSet = 0;
                }

                if (oneLoop == false) {
                    timeStart = getRuntime();
                    oneLoop = true;
                }
                if(getRuntime() - timeStart < 1){
                    CarouselMotor = CarouselMotor + ((((getRuntime() - timeStart) * -950) - ((robot.TC_M.getCurrentPosition() - lastCarouselE)/(getRuntime() - lastTime))) * carouselP);

                }else {
                    CarouselMotor = CarouselMotor + ((-950 - ((robot.TC_M.getCurrentPosition() - lastCarouselE) / (getRuntime() - lastTime))) * carouselP);
                }


            }else{
                CarouselMotor = 0;
                oneLoop = false;
            }



                //if we have a freight in our intake we raise up our arm
            if(robot.I_DS.getDistance(DistanceUnit.INCH) < 1 && teleOpVPivotSet < 550 && Math.abs(intakeRotateSet - CombinedTurret.rotateModifiedEncoder) < 100){
                teleOpVPivotSet = 550;
            }
            //setpoint limits
            if(teleOpVPivotSet > 2600){
                teleOpVPivotSet = 2600;
            }else if(teleOpVPivotSet < 100){
                teleOpVPivotSet = 100;
            }

            if(teleOpExtendSet > 1250){
                teleOpExtendSet = 1250;
            }if(teleOpExtendSet < 0){
                teleOpExtendSet = 0;
            }

            if(teleOpRotateSet > 5000){
                teleOpRotateSet = 5000;
            }else if(teleOpRotateSet < -5000){
                teleOpRotateSet = -5000;
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
        dashboardTelemetry.addData("carousel Speed", ((robot.TC_M.getCurrentPosition() - lastCarouselE)/(getRuntime() - lastTime)));
        dashboardTelemetry.addData("carosel motor", CarouselMotor);


        dashboardTelemetry.update();
        telemetry.update();
    }

}
