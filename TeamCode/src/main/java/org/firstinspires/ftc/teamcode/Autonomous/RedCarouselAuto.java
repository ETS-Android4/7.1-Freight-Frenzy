package org.firstinspires.ftc.teamcode.Autonomous;

import com.acmerobotics.dashboard.FtcDashboard;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.Autonomous.AutoClasses.DirectionCalcClass;
import org.firstinspires.ftc.teamcode.Autonomous.AutoClasses.Odometry;
import org.firstinspires.ftc.teamcode.Autonomous.AutoClasses.SpeedClass;
import org.firstinspires.ftc.teamcode.Autonomous.AutoClasses.TurnControl;
import org.firstinspires.ftc.teamcode.GeneralRobotCode.FreightFrenzyHardwareMap;
import org.firstinspires.ftc.teamcode.TurretClasses.TurretCombined;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvPipeline;

@Autonomous
public class RedCarouselAuto extends LinearOpMode {
    FreightFrenzyHardwareMap robot = new FreightFrenzyHardwareMap();
    SpeedClass SpeedClass = new SpeedClass();
    DirectionCalcClass DirectionClass = new DirectionCalcClass();
    TurnControl TurnControl = new TurnControl();
    Odometry OdoClass = new Odometry();
    TurretCombined CombinedTurret = new TurretCombined();
    FtcDashboard dashboard = FtcDashboard.getInstance();
    Telemetry dashboardTelemetry = dashboard.getTelemetry();
    //Uses Vuforia Developer Code
    //Declares Varibles
    double breakout;
    double lastAction = 0;
    double intakeXSetMod = 1;
    double Detected;
    double startPointX;
    double startPointY;
    boolean STOPMOTORS;
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
    double TSEPos;
    double nextMove;
    double leftIntakeSet = 0, rightIntakeSet = 0;
    double timeRemaining = 30, startTime;
    double timepassed2;
    public static double UPARMPM = .015;
    public static double UPARMDM = .009;
    public static double DNPM = .004;
    public static double DNDM = .012;
    public  static double SPEEDSET = 16;
    public static double MINSPEED = .2;
    public static double SETPOINT = 1500;
    double TSERegionThreshold = 100;
    double IntakeXSetpoint = 44;
    double YChangingSet = 1;

    double action;
    double initPOsitionOrder = 1;
    OpenCvCamera webcam;
    static OpenCVPipeline pipeline;
    @Override

    public void runOpMode() {
        robot.init(hardwareMap);
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        webcam = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);

        //allows to call pipline
        pipeline = new OpenCVPipeline();
        //sets the webcam to the pipeline
        webcam.setPipeline(pipeline);
        webcam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            //starts the webcam and defines the pixels
            public void onOpened() {
                webcam.startStreaming(640, 480, OpenCvCameraRotation.UPRIGHT);
            }

            @Override
            public void onError(int errorCode) {
                /*
                 * This will be called if the camera could not be opened
`               */
            }
        });
        telemetry.update();
        //Depending on the ring stack we change our intake to diffrent heights to be able to reach the top of the stack
        //Enters our 1 loop system, will exit once all actions are done
        while (!opModeIsActive()) {

            VPivotSetpoint = 900;
            VPivotSpeed = 10;
            if(Math.abs(0 - CombinedTurret.extendModifiedEncoder) < 50 && Math.abs(-600 - CombinedTurret.rotateModifiedEncoder) < 50){
                VPivotSetpoint = 570;
            }else if(VPivotSetpoint > 850){
                extendSetpoint = -30;
                extendSpeed = 15;
                rotateSetpoint = -620;
                rotateSpeed = 1000;
            }

            CombinedTurret.TurretCombinedMethod(extendSetpoint,extendSpeed,rotateSetpoint,rotateSpeed, VPivotSetpoint,VPivotSpeed, robot.TE_M.getCurrentPosition(), robot.TE_G.getState(), robot.TR_M.getCurrentPosition(), robot.TR_G.getState(), robot.TP_M.getCurrentPosition(), robot.TP_G.getState());
            robot.TR_M.setPower(CombinedTurret.rotateFinalMotorPower);
            robot.TE_M.setPower(CombinedTurret.extendFinalMotorPower);
            robot.TP_M.setPower(CombinedTurret.vPivotFinalMotorPower);
            if(gamepad1.dpad_up){
                TSERegionThreshold = TSERegionThreshold + 1;
            }else if(gamepad1.dpad_down){
                TSERegionThreshold = TSERegionThreshold - 1;
            }
            if (pipeline.region1Avg() <= TSERegionThreshold) {
                TSEPos = 1;
                telemetry.addData("TSE", 1);
            } else if (pipeline.region2Avg() <= TSERegionThreshold) {
                TSEPos = 2;
                telemetry.addData("TSE", 2);
            } else {
                TSEPos = 3;
                telemetry.addData("TSE", 3);
            }
            telemetry.addData("region2", pipeline.region1Avg());
            telemetry.addData("region3", pipeline.region2Avg());
            telemetry.addData("TSE Threshold", TSERegionThreshold);
            telemetry.update();
        }



        waitForStart();
        robot.LF_M.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.LF_M.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        robot.LB_M.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.LB_M.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        robot.RF_M.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.RF_M.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        //Shuts down Tensor Flow
        //Sets our intial varible setpoints
        action = 1;
        startTime = getRuntime();


        loopcount = 0;
        VPivotSpeed = 23;
        timepassed = 0;
        rotateSpeed = 2500;
        VPivotSetpoint = 2600;
        extendSpeed = 25;
        VPivotSpeed = 12;
        while (opModeIsActive() && stopProgram == 0) {
            if(action == 1){ //Move to carousel position

                accelerationDistance = 0;
                decelerationDistance = 4;
                slowMoveSpeed = 1;
                slowMovedDistance = 2;
                thetaDeccelerationDegree = .5;
                thetaTargetSpeed = 3;
                xSetpoint = -14;
                ySetpoint = -6;
                thetaSetpoint = 0;
                targetSpeed = 12.5;
                if(CombinedTurret.vPivotModifiedEncoder >= 2200){
                    rotateSetpoint = 460;
                }
                if(DirectionClass.distanceFromReturn() <= .5 && breakout == 1){
                    action = 2;
                    StopMotors();
                    startPointX = OdoClass.odoXReturn();
                    startPointY = OdoClass.odoYReturn();
                    breakout = 0;
                    loopcount = 1;
                    nextMove = 0;
                }else{
                    breakout = 1;
                }
            }
            else if(action ==2){
               if(loopcount == 1){
                   timepassed = getRuntime() + 4.5;
                   loopcount = 0;
               }
               if(timepassed >= getRuntime()){
                   robot.TC_M.setPower(.3);
               }
               else{
                   nextMove = 1;
               }
                if(nextMove == 1 && breakout == 1){
                    action = 3;
                    StopMotors();
                    startPointX = OdoClass.odoXReturn();
                    startPointY = OdoClass.odoYReturn();
                    breakout = 0;
                    nextMove = 0;
                    robot.TC_M.setPower(0);
                }else{
                    breakout = 1;
                }
            }
            else if(action == 3){ //Move to carousel position
                rotateSetpoint = -550;
                if(TSEPos == 1){
                    VPivotSetpoint = 1000;
                }
                else if(TSEPos == 2){
                    VPivotSetpoint = 1200;
                }
                else if (TSEPos == 3){
                    VPivotSetpoint = 1450;
                }
                targetSpeed = 20;
               xSetpoint = 0;
               ySetpoint = -15;

                if(DirectionClass.distanceFromReturn() <= .5 && breakout == 1){
                    action = 4;
                    StopMotors();
                    startPointX = OdoClass.odoXReturn();
                    startPointY = OdoClass.odoYReturn();
                    breakout = 0;
                    loopcount = 0;

                    nextMove = 0;
                }else{
                    breakout = 1;
                }
            }
            else if(action == 4){

                if(TSEPos == 1) {
                    if (CombinedTurret.vPivotModifiedEncoder >= 950 && CombinedTurret.vPivotModifiedEncoder <= 1050){
                        extendSetpoint = 1250;
                    }
                }

                else if(TSEPos == 2) {
                if (CombinedTurret.vPivotModifiedEncoder >= 1150 && CombinedTurret.vPivotModifiedEncoder <= 1250) {
                    extendSetpoint = 1250;
                }
                }
                else if (TSEPos == 3){
                if (CombinedTurret.vPivotModifiedEncoder >= 1400 && CombinedTurret.vPivotModifiedEncoder <= 1500){
                    extendSetpoint = 1250;
                }
                }
                if(CombinedTurret.extendModifiedEncoder >= 1150){
                    leftIntakeSet = -.5;
                    rightIntakeSet = .5;
                    if(loopcount == 0){
                        timepassed = getRuntime() + 3;
                        loopcount = 1;
                    }
                    if(timepassed <= getRuntime()){
                        nextMove = 1;
                    }
                }
                if(robot.I_DS.getDistance(DistanceUnit.INCH) >= 1 || nextMove == 1){
                    action = 5;
                    StopMotors();
                    startPointX = OdoClass.odoXReturn();
                    startPointY = OdoClass.odoYReturn();
                    breakout = 0;
                    loopcount = 0;
                    nextMove = 0;
                    leftIntakeSet = 0;
                    rightIntakeSet = 0;

                }
            }
            else if(action == 5){

            targetSpeed = 15;
            xSetpoint = -18.5;
            rotateSetpoint = 400;
            if(loopcount == 0){
                timepassed = getRuntime() + 4;
                loopcount = 1;
            }
            ySetpoint = -32;
            if(DirectionClass.distanceFromReturn() <= 1){
                VPivotSetpoint = 700;
                extendSetpoint = 0;
                rotateSetpoint = 420;
            }


            if((DirectionClass.distanceFromReturn() <= .5 && breakout == 1) && timepassed <= getRuntime()){
                action = 6;
                StopMotors();
                startPointX = OdoClass.odoXReturn();
                startPointY = OdoClass.odoYReturn();
                breakout = 0;
                loopcount = 0;
                nextMove = 0;
            }else{
                breakout = 1;
            }
        }



            //If nothing else to do, stop the program
            else {
                stopProgram = 1;
                StopMotors();
            }
            /*
            if(action == 2 && lastAction != 2){
                IntakeXSetpoint = 42 + intakeXSetMod;
                intakeXSetMod = intakeXSetMod + 1.5;
                YChangingSet = YChangingSet - 0;
            }



            lastAction = action;



             */
            if(DirectionClass.distanceFromReturn() <= 1){
                STOPMOTORS = true;
            }
            else{
                STOPMOTORS = false;
            }
            //Runs all of our equations each loop cycle
            Movement(xSetpoint, ySetpoint, thetaSetpoint, targetSpeed, thetaTargetSpeed, thetaDeccelerationDegree, slowMoveSpeed, accelerationDistance, decelerationDistance, slowMovedDistance);
            CombinedTurret.TurretCombinedMethod(extendSetpoint,extendSpeed,rotateSetpoint,rotateSpeed, VPivotSetpoint,VPivotSpeed, robot.TE_M.getCurrentPosition(), robot.TE_G.getState(), robot.TR_M.getCurrentPosition(), robot.TR_G.getState(), robot.TP_M.getCurrentPosition(), robot.TP_G.getState());

            PowerSetting();
            Telemetry();



        }
    }


    public void Telemetry() {
        //Displays telemetry
        telemetry.addData("Action", action);
        telemetry.addData("time", getRuntime());
        telemetry.addData("start Time", startTime);
        telemetry.addData("time left", getRuntime() - startTime);
        telemetry.addData("Odo X", OdoClass.odoXReturn());
        telemetry.addData("Odo Y", OdoClass.odoYReturn());
        telemetry.addData("Theta Angle", OdoClass.thetaInDegreesReturn());
        telemetry.addData("X", DirectionClass.XReturn());
        telemetry.addData("Y", DirectionClass.YReturn());
        telemetry.addData("Theta", TurnControl.theta);
        telemetry.addData("ThetaSpeedSetpoint", SpeedClass.thetaSpeedSetpoint());
        telemetry.addData("SlowMoveSpeed", slowMoveSpeed);
        telemetry.addData("slowMovedDistance", slowMovedDistance);
        telemetry.addData("Distance", DirectionClass.distanceReturn());
        telemetry.addData("Distance From", DirectionClass.distanceFromReturn());
        telemetry.addData("Speed Setpoint", SpeedClass.speedSetpoint());
        telemetry.addData("VPivot", CombinedTurret.vPivotModifiedEncoder);
        telemetry.addData("Speed", SpeedClass.SpeedReturn());

        telemetry.addData("Distance Delta", SpeedClass.DistanceDelta());
        telemetry.addData("XSetpoint", DirectionClass.XSetpointReturn());
        telemetry.addData("YSetpoint", DirectionClass.YSetpointReturn());
        telemetry.addData("LF_Power", robot.LF_M.getPower());
        telemetry.addData("RF_Power", robot.RF_M.getPower());
        telemetry.addData("LF_Direction", DirectionClass.LF_M_DirectionReturn());
        telemetry.addData("Motor Power Ratio", DirectionClass.motorPowerRatioReturn());

       // telemetry.addData("PT", robot.TP_P.getVoltage());

        telemetry.addData("Distance Sensor", robot.I_DS.getDistance(DistanceUnit.INCH));
        telemetry.update();
    }

    public void Movement(double endpointx, double endpointy, double thetasetpoint, double targetspeed, double thetaTargetSpeed, double thetaDeccelerationDegree, double slowMoveSpeed, double accelerationdistance, double deccelerationdistance, double slowMovedDistance) {
        OdoClass.RadiusOdometry(robot.LF_M.getCurrentPosition(), robot.LB_M.getCurrentPosition(), robot.RF_M.getCurrentPosition());
        TurnControl.turnControl(thetaSetpoint, OdoClass.thetaInDegreesReturn());
        DirectionClass.DirectionCalc(startPointX, startPointY, endpointx, endpointy, OdoClass.odoXReturn(), OdoClass.odoYReturn(), TurnControl.theta);
        SpeedClass.MotionProfile(targetspeed, accelerationdistance, deccelerationdistance, slowMovedDistance, DirectionClass.distanceReturn(), DirectionClass.distanceFromReturn(), slowMoveSpeed, thetaDeccelerationDegree, thetasetpoint, thetaTargetSpeed, OdoClass.thetaInDegreesReturn());
        SpeedClass.SpeedCalc(OdoClass.odoXReturn(), OdoClass.odoYReturn(), OdoClass.thetaInDegreesReturn(), getRuntime(), SpeedClass.speedSetpoint(), SpeedClass.thetaSpeedSetpoint());
    }

    public void PowerSetting() {
        if(STOPMOTORS == false) {


            robot.TR_M.setPower(CombinedTurret.rotateFinalMotorPower);
            robot.TE_M.setPower(CombinedTurret.extendFinalMotorPower);
            robot.TP_M.setPower(CombinedTurret.vPivotFinalMotorPower);
            robot.LF_M.setPower(DirectionClass.LF_M_DirectionReturn() * (SpeedClass.SpeedReturn()));
            robot.LB_M.setPower(DirectionClass.LB_M_DirectionReturn() * (SpeedClass.SpeedReturn()));
            robot.RF_M.setPower(DirectionClass.RF_M_DirectionReturn() * (SpeedClass.SpeedReturn()));
            robot.RB_M.setPower(DirectionClass.RB_M_DirectionReturn() * (SpeedClass.SpeedReturn()));
            robot.LI_S.setPower(leftIntakeSet);
            robot.RI_S.setPower(rightIntakeSet);
        }else{
            robot.LF_M.setPower(0);
            robot.LB_M.setPower(0);
            robot.RF_M.setPower(0);
            robot.RB_M.setPower(0);
            robot.TR_M.setPower(CombinedTurret.rotateFinalMotorPower);
            robot.TE_M.setPower(CombinedTurret.extendFinalMotorPower);
            robot.TP_M.setPower(CombinedTurret.vPivotFinalMotorPower);
            robot.LI_S.setPower(leftIntakeSet);
            robot.RI_S.setPower(rightIntakeSet);
        }


    }

    public void StopMotors() {
        robot.LF_M.setPower(0);
        robot.LB_M.setPower(0);
        robot.RF_M.setPower(0);
        robot.RB_M.setPower(0);
    }
   public static class OpenCVPipeline extends OpenCvPipeline {

        //sets colors for the boxes that we will look in
       static final Scalar CRIMSON = new Scalar(220, 20, 60);
       static final Scalar AQUA = new Scalar(79, 195, 247);
      // static final Scalar PARAKEET = new Scalar(3, 192, 74);

       //sets the boxes where we will look
       static final Point REGION1_TOPLEFT_ANCHOR_POINT = new Point(285, 218);
       static int REGION1_WIDTH = 30;
       static int REGION1_HEIGHT = 80;
       static final Point REGION2_TOPLEFT_ANCHOR_POINT = new Point(425, 218);
       static final int REGION2_WIDTH = 30;
       static final int REGION2_HEIGHT = 80;
        //static final Point REGION3_TOPLEFT_ANCHOR_POINT = new Point(430, 260);
        //static final int REGION3_WIDTH = 60;
        //static final int REGION3_HEIGHT = 85;
        //static final Point REGION1_TOPLEFT_ANCHOR_POINT = new Point(155, 260);
       //static int REGION1_WIDTH = 60;
       //static int REGION1_HEIGHT = 85;

        //uses the boxes setpoints and makes the actual box
        Point region1_pointA = new Point(REGION1_TOPLEFT_ANCHOR_POINT.x, REGION1_TOPLEFT_ANCHOR_POINT.y);
        Point region1_pointB = new Point(REGION1_TOPLEFT_ANCHOR_POINT.x + REGION1_WIDTH, REGION1_TOPLEFT_ANCHOR_POINT.y + REGION1_HEIGHT);

        Point region2_pointA = new Point(REGION2_TOPLEFT_ANCHOR_POINT.x, REGION2_TOPLEFT_ANCHOR_POINT.y);
        Point region2_pointB = new Point(REGION2_TOPLEFT_ANCHOR_POINT.x + REGION2_WIDTH, REGION2_TOPLEFT_ANCHOR_POINT.y + REGION2_HEIGHT);

       // Point region3_pointA = new Point(REGION3_TOPLEFT_ANCHOR_POINT.x, REGION3_TOPLEFT_ANCHOR_POINT.y);
        //Point region3_pointB = new Point(REGION3_TOPLEFT_ANCHOR_POINT.x + REGION3_WIDTH, REGION2_TOPLEFT_ANCHOR_POINT.y + REGION3_HEIGHT);


        Mat region1_G, region2_G, region3_G, region4_G;
        Mat GRAY = new Mat();
        int avg1, avg2, avg3, avg4;


        //actual image processing
        void inputToG(Mat input) {
            Imgproc.cvtColor(input, GRAY, Imgproc.COLOR_RGB2GRAY);
            Core.extractChannel(GRAY, GRAY, 0);
        }

        @Override
        public void init(Mat firstFrame) {
            inputToG(firstFrame);
            //sets region to look in for color
            region1_G = GRAY.submat(new Rect(region1_pointA, region1_pointB));
            region2_G = GRAY.submat(new Rect(region2_pointA, region2_pointB));
            //region3_G = A.submat(new Rect(region3_pointA, region3_pointB));
            //region4_G = L.submat(new Rect(region4_pointA, region4_pointB));
        }

        @Override
        public Mat processFrame(Mat input) {

            inputToG(input);

            avg1 = (int) Core.mean(region1_G).val[0];
            avg2 = (int) Core.mean(region2_G).val[0];
            //avg3 = (int) Core.mean(region3_G).val[0];
            //avg4 = (int) Core.mean(region4_G).val[0];

            Imgproc.rectangle(input, region1_pointA, region1_pointB, CRIMSON, 2);
            Imgproc.rectangle(input, region2_pointA, region2_pointB, AQUA, 2);
           // Imgproc.rectangle(input, region3_pointA, region3_pointB, PARAKEET, 2);
            //Imgproc.rectangle(input, region4_pointA, region4_pointB, GOLD, 2);

            return input;
        }

        public int region1Avg() {
            return avg1;
        }

        public int region2Avg() {
            return avg2;
        }

        //public int region3Avg() { return avg3; }



    }
}