package org.firstinspires.ftc.teamcode.Autonomous;

import static org.opencv.core.Core.inRange;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
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
import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvPipeline;

import java.util.ArrayList;
import java.util.List;

@Config
@Autonomous
public class BlueCarouselWithDUCK extends LinearOpMode {
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
    public static double Hmin = 15, Hmax = 50, Smin = 150, Smax = 255, Vmin = 150, Vmax = 255 ;
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
    double cubeLocationRotateEncoder = 0;

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
    boolean TurretCamOpened = false, TSECamOpened = false;

    double action;
    double initPOsitionOrder = 1;
    boolean hasAction5 = false;
    boolean Oneloop = false;
    double startOPTime = 0;

    static CubeTracking_Pipeline CubePipline;

    static OpenCV_Pipeline pipeline;
    OpenCvCamera RightCam1;
    OpenCvCamera TurretCam2;

    @Override

    public void runOpMode() {

        FtcDashboard dashboard = FtcDashboard.getInstance();
        telemetry = new MultipleTelemetry(telemetry, dashboard.getTelemetry());

        robot.init(hardwareMap);

        //sets up the camera for openCV piplines
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());

        int[] viewportContainerIds = OpenCvCameraFactory.getInstance().splitLayoutForMultipleViewports(cameraMonitorViewId, 2,OpenCvCameraFactory.ViewportSplitMethod.VERTICALLY);
        // SwitchableWebcam = OpenCvCameraFactory.getInstance().createSwitchableWebcam(cameraMonitorViewId, RightCam, TurretCam1);
        TurretCam2 = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "TurretCam1"), viewportContainerIds[0]);
        RightCam1 = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "RightCam"), viewportContainerIds[1]);


        //allows to call pipline
        pipeline = new OpenCV_Pipeline();
        CubePipline = new CubeTracking_Pipeline();

        /*RightCam1.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            //starts the webcam and defines the pixels
            public void onOpened() {

                RightCam1.setPipeline(pipeline);
                TSECamOpened = true;


                RightCam1.startStreaming(640, 480, OpenCvCameraRotation.UPRIGHT);
                //gives FTC dashboard acess to the camera
                FtcDashboard.getInstance().startCameraStream(RightCam1, 10);
                telemetry.addData("TSECameraOpened", "");
                telemetry.update();




            }
            //if the camera errors this happens
            @Override
            public void onError(int errorCode) {
                telemetry.addData("TSEcameraNotOpened", 000000000000000000000000000000000000000000000000000000);
                telemetry.update();
                /*
                 * This will be called if the camera could not be opened
`               */
       //     }
       // });

       TurretCam2.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            //starts the webcam and defines the pixels
            public void onOpened() {

                TurretCam2.setPipeline(CubePipline);
                TurretCamOpened = true;


                TurretCam2.startStreaming(640, 480, OpenCvCameraRotation.UPRIGHT);
                //gives FTC dashboard acess to the camera
                FtcDashboard.getInstance().startCameraStream(TurretCam2, 10);
                telemetry.addData("TURRETCameraOpened", "");
                telemetry.update();




            }

            //if the camera errors this happens
            @Override
            public void onError(int errorCode) {
                telemetry.addData("TURRETcameraNotOpened", 000000000000000000000000000000000000000000000000000000);
                telemetry.update();
                /*
                 * This will be called if the camera could not be opened
`               */
            }
        });

        while (!opModeIsActive()) {

            VPivotSetpoint = 900;
            VPivotSpeed = 10;
            if(Math.abs(-50 - CombinedTurret.extendModifiedEncoder) < 50 && Math.abs(465 - CombinedTurret.rotateModifiedEncoder) < 50){
                VPivotSetpoint = 602;

            }else if(VPivotSetpoint > 850){
                extendSetpoint = -50;

                extendSpeed = 15;

                rotateSetpoint = 465;
                rotateSpeed = 1000;

            }

            CombinedTurret.TurretCombinedMethod(extendSetpoint,extendSpeed,rotateSetpoint,rotateSpeed, VPivotSetpoint,VPivotSpeed, robot.TE_M.getCurrentPosition(), robot.TE_G.getState(), robot.TR_M.getCurrentPosition(), robot.TR_G.getState(), robot.TP_M.getCurrentPosition(), robot.TP_G.getState());
            robot.TR_M.setPower(CombinedTurret.rotateFinalMotorPower);
            robot.TE_M.setPower(CombinedTurret.extendFinalMotorPower);
            robot.TP_M.setPower(CombinedTurret.vPivotFinalMotorPower);

            telemetry.addData("TSEcamOpened?", TSECamOpened);
            telemetry.addData("TurretCamOpened?", TurretCamOpened);
            telemetry.addData("TSELocation", pipeline.TSELocation);
            telemetry.update();
            dashboardTelemetry.update();
        }



        waitForStart();
        if(pipeline.TSELocation != 0){
            TSEPos = pipeline.TSELocation;
        }else{
            TSEPos = 3;
        }
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
                xSetpoint = -13;
                ySetpoint = 6;
                thetaSetpoint = 0;
                targetSpeed = 12.5;
                if(CombinedTurret.vPivotModifiedEncoder >= 2200){
                    rotateSetpoint = 2000;
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
            else if(action ==2){//spinning the carousel
               if(loopcount == 1){
                   timepassed = getRuntime() + 4.5;
                   loopcount = 0;
               }
               if(timepassed >= getRuntime()){
                   robot.TC_M.setPower(-.3);
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
            else if(action == 3){ //Move to Aliiance Hub dropping
                if(CombinedTurret.vPivotModifiedEncoder > 800){
                    rotateSetpoint = 450;
                }

                if(TSEPos == 1){
                    VPivotSetpoint = 1000;
                }
                else if(TSEPos == 2){
                    VPivotSetpoint = 1200;
                }
                else if (TSEPos == 3){
                    VPivotSetpoint = 1550;
                }
                targetSpeed = 20;
               xSetpoint = 0;
               ySetpoint = 19;

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
            else if(action == 4){//extending to drop
                if(CombinedTurret.vPivotModifiedEncoder > 800){
                    rotateSetpoint = 450;
                }
                if(CombinedTurret.rotateModifiedEncoder > 100) {

                    if (TSEPos == 1) {
                        if (CombinedTurret.vPivotModifiedEncoder >= 950 && CombinedTurret.vPivotModifiedEncoder <= 1050) {
                            extendSetpoint = 800;
                        }
                    } else if (TSEPos == 2) {
                        if (CombinedTurret.vPivotModifiedEncoder >= 1150 && CombinedTurret.vPivotModifiedEncoder <= 1250) {
                            extendSetpoint = 800;
                        }
                    } else if (TSEPos == 3) {
                        if (CombinedTurret.vPivotModifiedEncoder >= 1500 && CombinedTurret.vPivotModifiedEncoder <= 1600) {
                            extendSetpoint = 800;
                        }
                    }
                }
                if(CombinedTurret.extendModifiedEncoder >= 760){
                    leftIntakeSet = -.4;
                    rightIntakeSet = .4;
                    if(loopcount == 0){
                        timepassed = getRuntime() + 2.1;
                        loopcount = 1;
                    }
                    if(timepassed <= getRuntime()){
                        nextMove = 1;
                    }
                }
                if(hasAction5){
                    if(nextMove == 1){
                        if(hasAction5){
                            action = 6;
                        }else{
                            action = 5;
                        }
                        //action = 6;

                        StopMotors();
                        startPointX = OdoClass.odoXReturn();
                        startPointY = OdoClass.odoYReturn();
                        breakout = 0;
                        loopcount = 0;
                        nextMove = 0;
                        leftIntakeSet = 0;
                        rightIntakeSet = 0;

                    }
                }else {
                    if (robot.I_DS.getDistance(DistanceUnit.INCH) >= 1.5 || nextMove == 1) {
                        if (hasAction5) {
                            action = 6;
                        } else {
                            action = 5;
                        }
                        //action = 6;

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
            }else if(action == 5){
                if(Oneloop == false){
                    Oneloop = true;
                    extendSetpoint = 0;
                    CombinedTurret.trackingRotate = true;
                    rotateSpeed = 600;
                }
                hasAction5 = true;

                rightIntakeSet = -0.5;
                leftIntakeSet = 0.5;

                if(CombinedTurret.rotateModifiedEncoder > -400) {

                    VPivotSetpoint = 900;
                    if (CombinedTurret.vPivotModifiedEncoder > 800) {
                        rotateSetpoint = -800;
                    }

                }else {


                    if (CombinedTurret.rotateModifiedEncoder < -700) {
                        VPivotSetpoint = 450;


                        if (Math.abs(rotateSetpoint-CombinedTurret.rotateModifiedEncoder) < 15 || CubePipline.targetX + (CubePipline.targetWidth / 2) > 0 && CombinedTurret.rotateModifiedEncoder < -800) {
                            cubeLocationRotateEncoder = (.6555 * (CubePipline.targetX + (CubePipline.targetWidth / 2))) - 212;

                        } else {
                            if(rotateSetpoint > -2300){
                                cubeLocationRotateEncoder = -7;

                            }else{
                                rotateSetpoint = -500;
                            }

                        }
                        rotateSetpoint = rotateSetpoint + cubeLocationRotateEncoder;

                    } else {
                        VPivotSetpoint = 900;
                    }


                    if (rotateSetpoint > 0) {
                        rotateSetpoint = 0;
                    }


                    if (Math.abs(320 - (CubePipline.targetX + (CubePipline.targetWidth / 2))) < 40) {
                        extendSetpoint = extendSetpoint + 20;
                    }
                    if (extendSetpoint > 800) {
                        extendSetpoint = 0;
                        VPivotSetpoint = 500;
                        rotateSetpoint = -1900;
                    }
                    if (robot.I_DS.getDistance(DistanceUnit.INCH) < 1) {
                        action = 3;
                        rotateSpeed = 2500;
                        extendSetpoint = 100;
                        CombinedTurret.trackingRotate = false;

                    }
                }


            } else if(action == 6){//going to park in the STORAGEUNIT
                //setpoints for Shipping unit
                xSetpoint = -18.5;
                ySetpoint = 27;
            targetSpeed = 15;

                VPivotSetpoint = 900;
                if(VPivotSetpoint > 800){
                    rotateSetpoint = 300;
                    extendSetpoint = 0;
                }




            if((DirectionClass.distanceFromReturn() <= .5 && breakout == 1) && CombinedTurret.rotateModifiedEncoder > 100){
                //action = 6.5;
                action = 8;
                StopMotors();
                startPointX = OdoClass.odoXReturn();
                startPointY = OdoClass.odoYReturn();
                breakout = 0;
                loopcount = 0;
                nextMove = 0;
            }else{
                breakout = 1;
            }
        }else if(action == 6.5) {//Parking in the WAREHOUSE

                if(getRuntime() - startTime > 20){

                    //setpoints for Warehouse
                    xSetpoint = 40;
                    ySetpoint = .5;
                    targetSpeed = 30;
                }


                VPivotSetpoint = 900;
                if(VPivotSetpoint > 800){
                    rotateSetpoint = 300;
                    extendSetpoint = 0;
                }



                if((DirectionClass.distanceFromReturn() <= .5 && breakout == 1) && CombinedTurret.rotateModifiedEncoder > 100){
                    action = 6.75;
                    StopMotors();
                    startPointX = OdoClass.odoXReturn();
                    startPointY = OdoClass.odoYReturn();
                    breakout = 0;
                    loopcount = 0;
                    nextMove = 0;
                    Oneloop = true;
                }
                if(xSetpoint > 20){
                    breakout = 1;
                }

            }else if(action == 6.75){
                if(Oneloop == true){
                    Oneloop = false;
                    breakout = 0;
                }
                xSetpoint = 81;
                ySetpoint = .5;
                targetSpeed = 20;

                VPivotSetpoint = 900;
                if(VPivotSetpoint > 800){
                    rotateSetpoint = 300;
                    extendSetpoint = 0;
                }

                if((DirectionClass.distanceFromReturn() <= .5 && breakout == 1) && CombinedTurret.rotateModifiedEncoder > 100){
                    action = 8;
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
            else{
                stopProgram = 1;
            }

            if(DirectionClass.distanceFromReturn() <= .75){
                STOPMOTORS = true;
            }
            else{
                STOPMOTORS = false;
            }
            //Runs all of our equations each loop cycle
            VPivotSetpoint = CombinedTurret.VPivotLimits(extendSetpoint, VPivotSetpoint, rotateSetpoint, false, false);
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
    //OpenCV pipline for TSE Detection
    public static class OpenCV_Pipeline extends OpenCvPipeline {

        /** Most important section of the code: Colors **/
        static final Scalar GOLD = new Scalar(255, 215, 0);
        static final Scalar CRIMSON = new Scalar(220, 20, 60);
        static final Scalar AQUA = new Scalar(79, 195, 247);
        static final Scalar PARAKEET = new Scalar(3, 192, 74);
        static final Scalar CYAN = new Scalar(0, 139, 139);
        static final Scalar WHITE = new Scalar(255, 255, 255);

        int indexLowest; double yLowest = -10;

        double TSELocation = 0;

        // Create a Mat object that will hold the color data


        Rect redMask;






        List<MatOfPoint> redContours;

        // Make a Constructor
        public OpenCV_Pipeline() {

            redContours = new ArrayList<MatOfPoint>();

        }

        public boolean filterContours(MatOfPoint contour) {
            return Imgproc.contourArea(contour) > 400;
        }

        @Override
        public Mat processFrame(Mat input) {

            Mat YCrCb = new Mat();
            Mat HSV = new Mat();
            Mat RGBA = new Mat();





            Imgproc.cvtColor(input, HSV, Imgproc.COLOR_RGB2HSV);

            //Scalar scalarLowerYCrCb = new Scalar(TSEHmin, TSESmin, TSEVmin);//for adjusting
            //Scalar scalarUpperYCrCb = new Scalar(TSEHmax, TSESmax, TSEVmax);//for adjusting
            Scalar scalarLowerYCrCb = new Scalar(30.0, 120.0, 75.0);//GREEN
            Scalar scalarUpperYCrCb = new Scalar(78.0, 255.0, 255.0);//GREEN
            //Scalar scalarLowerYCrCb = new Scalar(130.0, 0.0, 50.0);//Purple
            //Scalar scalarUpperYCrCb = new Scalar(180.0, 255.0, 255.0);//Purple
            // min 0,0,0
            // Max 180, 255,255
            Mat maskRed = new Mat();
            //BLUE DO NOT REMOVE
            //Scalar scalarLowerYCrCb = new Scalar(80.0, 70.0, 100.0);
            //Scalar scalarUpperYCrCb = new Scalar(180.0, 255.0, 255.0);

            //inRange(HSV, lowYellow, highYellow, maskYellow);
            //inRange(HSV, lowWhite, highWhite, maskWhite);
            inRange(HSV, scalarLowerYCrCb, scalarUpperYCrCb, maskRed);


            redContours.clear();


            Imgproc.findContours(maskRed, redContours, new Mat(), Imgproc.RETR_TREE, Imgproc.CHAIN_APPROX_SIMPLE);
            Imgproc.drawContours(input, redContours, -1, AQUA); //input


            yLowest = -640;
            indexLowest = 0;
            Imgproc.rectangle(input, new Point(100, 225), new Point(210,375), AQUA);
            Imgproc.rectangle(input, new Point(210, 225), new Point(340, 375), PARAKEET);
            Imgproc.rectangle(input, new Point(340, 225), new Point(480, 375), GOLD);
            TSELocation = 0;
            if (redContours.size() > 0) {
                for (int i = 0; i < redContours.size(); i++) {
                    if (filterContours(redContours.get(i))) {
                        redMask = Imgproc.boundingRect(redContours.get(i));
                        Imgproc.rectangle(input, redMask, AQUA, 10);
                        if(TSELocation == 0 || TSELocation == 1) {
                            if (redMask.y + redMask.height > 200 && redMask.y + redMask.height < 400) {
                                if (redMask.x + (redMask.width / 2) > 340 && redMask.x + (redMask.width / 2) < 480) {
                                    TSELocation = 3;
                                } else if (redMask.x + (redMask.width / 2) > 210 && redMask.x + (redMask.width / 2) < 340) {
                                    TSELocation = 2;
                                } else if (redMask.x + (redMask.width / 2) > 100 && redMask.x + (redMask.width / 2) < 21) {
                                    TSELocation = 1;
                                }
                            }
                        }
                    }
                }
                redMask = Imgproc.boundingRect(redContours.get(indexLowest));
                Imgproc.rectangle(input, redMask, PARAKEET, -5);


            } else {
                TSELocation = -1;

            }

            maskRed.release();



            YCrCb.release();
            RGBA.release();
            HSV.release();




            return input;
        }
    }

    public static class CubeTracking_Pipeline extends OpenCvPipeline {

        /** Most important section of the code: Colors **/
        static final Scalar GOLD = new Scalar(255, 215, 0);
        static final Scalar CRIMSON = new Scalar(220, 20, 60);
        static final Scalar AQUA = new Scalar(79, 195, 247);
        static final Scalar PARAKEET = new Scalar(3, 192, 74);
        static final Scalar CYAN = new Scalar(0, 139, 139);
        static final Scalar WHITE = new Scalar(255, 255, 255);

        int indexLowest; double yLowest = -10;
        double targetX; double targetY; double targetWidth; double targetArea; double yLeft = -10;
        double cubeCenter = 320;
        // Create a Mat object that will hold the color data


        Rect CubeMask;


        List<MatOfPoint> CubeContours;

        // Make a Constructor
        public CubeTracking_Pipeline() {
            CubeContours = new ArrayList<MatOfPoint>();
        }

        public boolean filterContours(MatOfPoint contour) {
            return Imgproc.contourArea(contour) > 400;
        }

        @Override
        public Mat processFrame(Mat input) {

            Mat YCrCb = new Mat();
            Mat HSV = new Mat();
            Mat RGBA = new Mat();

            Imgproc.cvtColor(input, HSV,Imgproc.COLOR_RGB2HSV);

            Scalar scalarLowerYCrCb = new Scalar(Hmin, Smin, Vmin);
            Scalar scalarUpperYCrCb = new Scalar(Hmax, Smax, Vmax);
            //Scalar scalarLowerYCrCb = new Scalar(15.0, 100.0, 120.0);
            //Scalar scalarUpperYCrCb = new Scalar(45.0, 255.0, 255.0);
            Mat maskCube = new Mat();


            inRange(HSV, scalarLowerYCrCb, scalarUpperYCrCb, maskCube);


            CubeContours.clear();


            Imgproc.findContours(maskCube, CubeContours, new Mat(), Imgproc.RETR_TREE, Imgproc.CHAIN_APPROX_SIMPLE);
            Imgproc.drawContours(input, CubeContours, -1, AQUA); //input
      /*      Imgproc.rectangle(input, new Point(0,0), new Point(50,480), AQUA, 1);
            Imgproc.rectangle(input, new Point(50,0), new Point(100,480), AQUA, 1);
            Imgproc.rectangle(input, new Point(100,0), new Point(150,480), AQUA, 1);
            Imgproc.rectangle(input, new Point(150,0), new Point(200,480), AQUA, 1);
            Imgproc.rectangle(input, new Point(200,0), new Point(250,480), AQUA, 1);
            Imgproc.rectangle(input, new Point(250,0), new Point(300,480), AQUA, 1);
            Imgproc.rectangle(input, new Point(300,0), new Point(350,480), AQUA, 1);
            Imgproc.rectangle(input, new Point(350,0), new Point(400,480), AQUA, 1);
            Imgproc.rectangle(input, new Point(400,0), new Point(450,480), AQUA, 1);
            Imgproc.rectangle(input, new Point(450,0), new Point(500,480), AQUA, 1);
            Imgproc.rectangle(input, new Point(500,0), new Point(550,480), AQUA, 1);
            Imgproc.rectangle(input, new Point(550,0), new Point(600,480), AQUA, 1);
            Imgproc.rectangle(input, new Point(600,0), new Point(640,480), GOLD, -1);*/


            yLowest = -640;
            indexLowest = 0;

            if(CubeContours.size() > 0){
                for (int i = 0; i < CubeContours.size(); i++){
                    if (filterContours(CubeContours.get(i))){
                        CubeMask = Imgproc.boundingRect(CubeContours.get(i));
                        Imgproc.rectangle(input, CubeMask, AQUA, 2);

                        if(Math.abs((CubeMask.y + CubeMask.height) - yLowest) < 20){
                            if(CubeMask.x + CubeMask.width > yLeft){
                                indexLowest = i;
                                yLowest = CubeMask.y + CubeMask.height;
                                yLeft = CubeMask.x + CubeMask.width;
                            }
                        }else if(CubeMask.y + CubeMask.height > yLowest){
                            indexLowest = i;
                            yLowest = CubeMask.y + CubeMask.height;
                            yLeft = CubeMask.x + CubeMask.width;
                        }
                    }
                }
                CubeMask = Imgproc.boundingRect(CubeContours.get(indexLowest));
                Imgproc.rectangle(input, CubeMask, PARAKEET, -5);
                targetX = CubeMask.x;
                targetY = CubeMask.y;
                targetWidth = CubeMask.width;
                targetArea = CubeMask.height * CubeMask.width;
                cubeCenter = targetX + (targetWidth/2);

            }else{
                targetX = -1;
                targetY = -1;
                targetWidth = -1;
                targetArea = -1;
                cubeCenter = 320;

            }


            YCrCb.release();
            RGBA.release();
            HSV.release();
            maskCube.release();


            return input;
        }
    }
}