package org.firstinspires.ftc.teamcode.Autonomous.OldCode;

//@Autonomous
/*public class BlueWarehouseAuto extends LinearOpMode {
    FreightFrenzyHardwareMap robot = new FreightFrenzyHardwareMap();
    SpeedClass SpeedClass = new SpeedClass();
    DirectionCalcClass DirectionClass = new DirectionCalcClass();
    TurnControl TurnControl = new TurnControl();
    Odometry OdoClass = new Odometry();
    ExtendClass ExtendClass = new ExtendClass();
    VPivotClass VPivotClass = new VPivotClass();
    RotateClass RotateClass = new RotateClass();
    FtcDashboard dashboard = FtcDashboard.getInstance();
    Telemetry dashboardTelemetry = dashboard.getTelemetry();
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
    double TSEPos;
    double timepassed2;
    public static double UPARMPM = .015;
    public static double UPARMDM = .009;
    public static double DNPM = .004;
    public static double DNDM = .012;
    public  static double SPEEDSET = 16;
    public static double MINSPEED = .2;
    public static double SETPOINT = 1500;

    double action;
    double initPOsitionOrder = 1;
    OpenCvCamera webcam;
    static OpenCVPipeline pipeline;
    @Override

    public void runOpMode() {
        robot.init(hardwareMap);
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        webcam = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 2"), cameraMonitorViewId);

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

            }
        });
        telemetry.update();
        //Depending on the ring stack we change our intake to diffrent heights to be able to reach the top of the stack
        //Enters our 1 loop system, will exit once all actions are done
        while (!opModeIsActive()) {
            if (RotateClass.isHomedRotateReturn() == false) {
                if (VPivotClass.has1stloop == true) {
                   // robot.TP_M.setPower(VPivotClass.NEWVPivot(2000, 10, robot.TP_P.getVoltage(), -robot.TP_M.getCurrentPosition(), robot.TP_G.getState(), getRuntime(), 16, UPARMPM, UPARMDM, DNPM, DNDM, MINSPEED));
                    if (VPivotClass.encoderWithOffset > 1900 && VPivotClass.encoderWithOffset < 2100) {
                        robot.TE_M.setPower(ExtendClass.ExtendHoming(robot.TE_G.getState(), robot.TE_M.getCurrentPosition()));
                        if (ExtendClass.isHomedExtendReturn() == true) {
                            robot.TR_M.setPower(RotateClass.RotateHoming(robot.TR_G.getState(), robot.TR_M.getCurrentPosition()));
                        }
                    }
                } else {
                  //  robot.TP_M.setPower(VPivotClass.NEWVPivot(3000, 10, robot.TP_P.getVoltage(), -robot.TP_M.getCurrentPosition(), robot.TP_G.getState(), getRuntime(), 16, UPARMPM, UPARMDM, DNPM, DNDM, MINSPEED));
                }

            } else {
                telemetry.addData("homed", 0);
                robot.TE_M.setPower(ExtendClass.ExtendAutoMethod(10, .8, robot.TE_M.getCurrentPosition(), robot.TE_G.getState()));
                if (initPOsitionOrder == 1) {
                  //  robot.TP_M.setPower(VPivotClass.NEWVPivot(2000, 10, robot.TP_P.getVoltage(), -robot.TP_M.getCurrentPosition(), robot.TP_G.getState(), getRuntime(), 16, UPARMPM, UPARMDM, DNPM, DNDM, MINSPEED));
                    robot.TR_M.setPower(RotateClass.RotateAutoMethod(800, .5, robot.TR_M.getCurrentPosition(), robot.TR_G.getState()));
                    if (RotateClass.modifiedRotateCurrent() > 750 && RotateClass.modifiedRotateCurrent() < 850) {
                        initPOsitionOrder = 2;
                    }
                } else if (initPOsitionOrder == 2) {
                    robot.TR_M.setPower(RotateClass.RotateAutoMethod(800, .5, robot.TR_M.getCurrentPosition(), robot.TR_G.getState()));
                   // robot.TP_M.setPower(VPivotClass.NEWVPivot(500, 10, robot.TP_P.getVoltage(), -robot.TP_M.getCurrentPosition(), robot.TP_G.getState(), getRuntime(), 16, UPARMPM, UPARMDM, DNPM, DNDM, MINSPEED));
                    if (VPivotClass.encoderWithOffset < 550 && VPivotClass.encoderWithOffset > 450) {
                        initPOsitionOrder = 3;
                    }
                } else if (initPOsitionOrder == 3) {
                    robot.TR_M.setPower(RotateClass.RotateAutoMethod(600, .4, robot.TR_M.getCurrentPosition(), robot.TR_G.getState()));
                   // robot.TP_M.setPower(VPivotClass.NEWVPivot(475, 5, robot.TP_P.getVoltage(), -robot.TP_M.getCurrentPosition(), robot.TP_G.getState(), getRuntime(), 16, UPARMPM, UPARMDM, DNPM, DNDM, MINSPEED));

                }


            }

            if (pipeline.region1Avg() <= 160) {
                TSEPos = 2;
                telemetry.addData("TSE", 2);
            } else if (pipeline.region2Avg() <= 160) {
                TSEPos = 3;
                telemetry.addData("TSE", 3);
            } else {
                TSEPos = 1;
                telemetry.addData("TSE", 1);
            }
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
        startPointX = 0;
        startPointY = 0;
        stopProgram = 0;
        xSetpoint = 0;
        ySetpoint = 0;
        thetaSetpoint = 0;
        targetSpeed = .25;
        accelerationDistance = 0;
        decelerationDistance = 1;
        slowMoveSpeed = .25;
        slowMovedDistance = 1;
        thetaDeccelerationDegree = 1;
        thetaTargetSpeed = .4;
        rotateSpeed = .6;
        extendSpeed = .8;
        VPivotSpeed = 12;
        loopcount = 0;
        timepassed = 0;
        while (opModeIsActive() && stopProgram == 0) {
            if(action == 1) {
                if (TSEPos == 1 && action == 1) {
                    rotateSetpoint = 1800;
                    extendSetpoint = 0;
                    VPivotSetpoint = 900;
                    if (DirectionClass.distanceFromReturn() <= .7) {
                        StopMotors();
                    }
                    if ((VPivotClass.encoderWithOffset >= 875) && (VPivotClass.encoderWithOffset <= 950)) {
                        extendSetpoint = 1240;
                        if (ExtendClass.extendModifiedEncoder <= 1400 && ExtendClass.extendModifiedEncoder >= 1220) {
                            if (loopcount == 0) {
                                loopcount = 1;
                                timepassed = getRuntime() + 4;
                            }
                            robot.LI_S.setPower(-1);
                            robot.RI_S.setPower(1);
                            if (robot.I_DS.getDistance(DistanceUnit.INCH) >= 2.5 || timepassed <= getRuntime()) {
                                StopMotors();
                                action = 2;
                                startPointY = OdoClass.odoYReturn();
                                startPointX = OdoClass.odoXReturn();
                                robot.LI_S.setPower(0);
                                robot.RI_S.setPower(0);
                            }
                        }
                    }
                }
                if (TSEPos == 2 && action == 1) {
                    rotateSetpoint = 1800;
                    extendSetpoint = 0;
                    VPivotSetpoint = 1190;
                    if (DirectionClass.distanceFromReturn() <= .7) {
                        StopMotors();
                    }
                    if (VPivotClass.encoderWithOffset >= 875) {
                        extendSetpoint = 1300;
                        if (ExtendClass.extendModifiedEncoder <= 1450 && ExtendClass.extendModifiedEncoder >= 1280) {
                            if(loopcount == 0){
                                loopcount = 1;
                                timepassed = getRuntime() + 4;
                            }
                            robot.LI_S.setPower(-1);
                            robot.RI_S.setPower(1);
                            if (robot.I_DS.getDistance(DistanceUnit.INCH) >= 2.5 || timepassed <= getRuntime()) {
                                StopMotors();
                                action = 2;
                                startPointY = OdoClass.odoYReturn();
                                robot.LI_S.setPower(0);
                                robot.RI_S.setPower(0);
                            }
                        }
                    }
                }
                if (TSEPos == 3 && action == 1) {
                    rotateSetpoint = 1800;
                    extendSetpoint = 0;
                    VPivotSetpoint = 1500;
                    if (DirectionClass.distanceFromReturn() <= .7) {
                        StopMotors();
                    }
                    if (VPivotClass.encoderWithOffset >= 875) {
                        extendSetpoint = 1455;
                        if (ExtendClass.extendModifiedEncoder <= 1480 && ExtendClass.extendModifiedEncoder >= 1375) {
                            if(loopcount == 0){
                                loopcount = 1;
                                timepassed = getRuntime() + 4;
                            }
                            robot.LI_S.setPower(-1);
                            robot.RI_S.setPower(1);
                            if (robot.I_DS.getDistance(DistanceUnit.INCH) >= 2.5 || timepassed <= getRuntime()) {
                                StopMotors();
                                action = 2;
                                startPointY = OdoClass.odoYReturn();
                                robot.LI_S.setPower(0);
                                robot.RI_S.setPower(0);
                                breakout = 0;
                            }
                        }
                    }
                }
            }
            else if(action == 2){
                extendSetpoint = 100;
                extendSpeed = .8;
                thetaSetpoint = 0;
                accelerationDistance = .25;
                decelerationDistance = 7.5;
                slowMoveSpeed = 1;
                slowMovedDistance = 2;
                thetaDeccelerationDegree = 2;
                thetaTargetSpeed = 1;
                VPivotSetpoint = .9;
                VPivotSpeed = 5;
                xSetpoint = 41;
                ySetpoint = 1;
                thetaSetpoint = 0;
                targetSpeed = 18;
                rotateSpeed = .65;
                rotateSetpoint = 0;
                VPivotSetpoint = 1500;
                //Exits once the robot is a certain distance and angle away
                if (DirectionClass.distanceFromReturn() <= 1.3 && breakout != 0 && (OdoClass.thetaInDegreesReturn() < 7 && OdoClass.thetaInDegreesReturn() > -7)) {
                    StopMotors();
                    breakout = 0;
                    action = 3;
                    startPointX = OdoClass.odoXReturn();
                    startPointY = OdoClass.odoYReturn();
                }else {
                    breakout = 1;
                }
            }





            //If nothing else to do, stop the program
            else {
                stopProgram = 1;
                StopMotors();
            }
            //Runs all of our equations each loop cycle
            Movement(xSetpoint, ySetpoint, thetaSetpoint, targetSpeed, thetaTargetSpeed, thetaDeccelerationDegree, slowMoveSpeed, accelerationDistance, decelerationDistance, slowMovedDistance);
            RotateClass.RotateAutoMethod(rotateSetpoint, rotateSpeed, robot.TR_M.getCurrentPosition(), robot.TR_G.getState());
            ExtendClass.ExtendAutoMethod(extendSetpoint, extendSpeed, robot.TE_M.getCurrentPosition(), robot.TE_G.getState());
          //  VPivotClass.NEWVPivot(VPivotSetpoint, VPivotSpeed, robot.TP_P.getVoltage(), -robot.TP_M.getCurrentPosition(), robot.TP_G.getState(), getRuntime(), 16, UPARMPM, UPARMDM, DNPM,DNDM,MINSPEED);
            PowerSetting();
            Telemetry();
        }
    }


    public void Telemetry() {
        //Displays telemetry
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
        telemetry.addData("Speed", SpeedClass.SpeedReturn());
        telemetry.addData("time", getRuntime());
        telemetry.addData("Distance Delta", SpeedClass.DistanceDelta());
        telemetry.addData("XSetpoint", DirectionClass.XSetpointReturn());
        telemetry.addData("YSetpoint", DirectionClass.YSetpointReturn());
        telemetry.addData("LF_Power", robot.LF_M.getPower());
        telemetry.addData("RF_Power", robot.RF_M.getPower());
        telemetry.addData("LF_Direction", DirectionClass.LF_M_DirectionReturn());
        telemetry.addData("Motor Power Ratio", DirectionClass.motorPowerRatioReturn());
        telemetry.addData("Action", action);
       // telemetry.addData("PT", robot.TP_P.getVoltage());
        telemetry.addData("ExtendE", ExtendClass.extendModifiedEncoder);
        telemetry.addData("RotateE", RotateClass.modifiedRotateCurrent());
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

        robot.TR_M.setPower(RotateClass.rotateMotorPower);
        robot.TE_M.setPower(ExtendClass.ExtendMotorPower);
       robot.TP_M.setPower(VPivotClass.FinalMotorPower);
        robot.LF_M.setPower(DirectionClass.LF_M_DirectionReturn() * (SpeedClass.SpeedReturn()));
        robot.LB_M.setPower(DirectionClass.LB_M_DirectionReturn() * (SpeedClass.SpeedReturn() ));
       robot.RF_M.setPower(DirectionClass.RF_M_DirectionReturn() * (SpeedClass.SpeedReturn()));
       robot.RB_M.setPower(DirectionClass.RB_M_DirectionReturn() * (SpeedClass.SpeedReturn() ));


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
        static final Point REGION1_TOPLEFT_ANCHOR_POINT = new Point(115, 300);
        static int REGION1_WIDTH = 80;
        static int REGION1_HEIGHT = 40;
        static final Point REGION2_TOPLEFT_ANCHOR_POINT = new Point(253, 305);
        static final int REGION2_WIDTH = 80;
        static final int REGION2_HEIGHT = 40;
        //static final Point REGION3_TOPLEFT_ANCHOR_POINT = new Point(430, 260);
        //static final int REGION3_WIDTH = 60;
        //static final int REGION3_HEIGHT = 85;
        //static final Point REGION1_TOPLEFT_ANCHOR_POINT = new Point(155, 260);
       //static int REGION1_WIDTH = 60;
       //static int REGION1_HEIGHT = 85;

        //uses the boxes setpoints and makes the ctual box
        Point region1_pointA = new Point(REGION1_TOPLEFT_ANCHOR_POINT.x, REGION1_TOPLEFT_ANCHOR_POINT.y);
        Point region1_pointB = new Point(REGION1_TOPLEFT_ANCHOR_POINT.x + REGION1_WIDTH, REGION1_TOPLEFT_ANCHOR_POINT.y + REGION1_HEIGHT);

        Point region2_pointA = new Point(REGION2_TOPLEFT_ANCHOR_POINT.x, REGION2_TOPLEFT_ANCHOR_POINT.y);
        Point region2_pointB = new Point(REGION2_TOPLEFT_ANCHOR_POINT.x + REGION2_WIDTH, REGION2_TOPLEFT_ANCHOR_POINT.y + REGION2_HEIGHT);

       // Point region3_pointA = new Point(REGION3_TOPLEFT_ANCHOR_POINT.x, REGION3_TOPLEFT_ANCHOR_POINT.y);
        //Point region3_pointB = new Point(REGION3_TOPLEFT_ANCHOR_POINT.x + REGION3_WIDTH, REGION2_TOPLEFT_ANCHOR_POINT.y + REGION3_HEIGHT);


        Mat region1_G, region2_G, region3_G, region4_G;
        Mat RGBA = new Mat();
        Mat HLS = new Mat();
        Mat A = new Mat();
        Mat H = new Mat();
        int avg1, avg2, avg3, avg4;


        //actual image processing
        void inputToG(Mat input) {
            Imgproc.cvtColor(input, RGBA, Imgproc.COLOR_RGB2RGBA);
            Core.extractChannel(RGBA, A, 1);
        }

        @Override
        public void init(Mat firstFrame) {
            inputToG(firstFrame);
            //sets region to look in for color
            region1_G = A.submat(new Rect(region1_pointA, region1_pointB));
            region2_G = A.submat(new Rect(region2_pointA, region2_pointB));
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
}*/