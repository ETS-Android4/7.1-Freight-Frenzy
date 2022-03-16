package org.firstinspires.ftc.teamcode.TestHub;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.GeneralRobotCode.FreightFrenzyHardwareMap;
import org.firstinspires.ftc.teamcode.GeneralRobotCode.Smoothing;
import org.firstinspires.ftc.teamcode.TurretClasses.TurretCombined;


//@Config
//@TeleOp
public class CarouselTesting extends LinearOpMode{



    public double x, y, z;
    double timeWait = 1, slowSpeed = .3, fastSpeed = 1, timeStart = 0;
    boolean oneLoop = false;

    double teleOpExtendSet = 200, teleOpRotateSet = 0, teleOpVPivotSet = 1000;
    double intakeVPivotSet = 480, intakeRotateSet = 0, intakeExtendSet = 275;
    double teleOpExtendSpeedSet = 30, teleOpRotateSpeedSet = 2000, teleOpVPivotSpeedSet = 18;
    double lastDS = 5;



    FreightFrenzyHardwareMap robot = new FreightFrenzyHardwareMap();
    TurretCombined CombinedTurret = new TurretCombined();
    Smoothing Smoothing = new Smoothing();
  //  FtcDashboard dashboard = FtcDashboard.getInstance();
  //  Telemetry dashboardTelemetry = dashboard.getTelemetry();



    @Override
    public void runOpMode(){
        robot.init(hardwareMap);



        waitForStart();

        while (opModeIsActive()){

        if(gamepad1.dpad_up){
            timeWait = timeWait + .00001;
        }else if(gamepad1.dpad_down){
            timeWait = timeWait - .00001;
        }

        if(gamepad1.y){
            fastSpeed = fastSpeed + .00001;
        }else if(gamepad1.a){
            fastSpeed = fastSpeed - .00001;
        }

        if(gamepad1.dpad_right){
            slowSpeed = slowSpeed + .00001;
        }else if(gamepad1.dpad_left){
            slowSpeed = slowSpeed - .00001;
        }
        if(gamepad1.left_trigger >= .2){
            slowSpeed = 0.678;
            fastSpeed = 1;
            timeWait = 0.895;
        }


        if(gamepad1.right_bumper || gamepad1.left_bumper){
            if(oneLoop == false){
                timeStart = getRuntime();
                oneLoop = true;
            }
            if(getRuntime() - timeStart < timeWait){
                robot.TC_M.setPower(-slowSpeed);
            }else{
                robot.TC_M.setPower(-fastSpeed);
            }

        }else{
            oneLoop = false;
            robot.TC_M.setPower(0);
        }

        if(timeWait < 0){
            timeWait = 0;
        }

        if(slowSpeed > 1){
            slowSpeed = 1;
        }else if(slowSpeed < 0){
            slowSpeed = 0;
        }

        if(fastSpeed > 1){
            fastSpeed = 1;
        }else if(fastSpeed < 0){
            fastSpeed = 0;
        }

        telemetry.addData("slow speed", slowSpeed);
        telemetry.addData("fast Speed", fastSpeed);
        telemetry.addData("wait timer", timeWait);
        telemetry.update();



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
        telemetry.addData("rotate mod encoder", CombinedTurret.rotateModifiedEncoder);
        telemetry.addData("rotate RAW Encoder", robot.TR_M.getCurrentPosition());


     //   dashboardTelemetry.update();
        telemetry.update();
    }

}
