package org.firstinspires.ftc.teamcode.TeleOp;

import static org.opencv.core.Core.inRange;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.GeneralRobotCode.FreightFrenzyHardwareMap;
import org.firstinspires.ftc.teamcode.TurretClasses.TurretCombined;
import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvPipeline;

import java.util.ArrayList;
import java.util.List;

@Autonomous
@Config
public class Cube_Tracking extends LinearOpMode {

    public static double followP = .05;

    double teleOpExtendSpeedSet = 10, teleOpRotateSet = 0, teleOpVPivotSet = 500, teleOpExtendSet = 220, teleOpRotateSpeedSet = 1000, teleOpVPivotSpeedSet = 10;

    double cubeDifference = 0, cubeChange = 0;

    boolean controllerY = false;
    public static double Hmin = 15, Hmax = 50, Smin = 150, Smax = 255, Vmin = 150, Vmax = 255 ;

    TurretCombined CombinedTurret = new TurretCombined();
    FreightFrenzyHardwareMap robot = new FreightFrenzyHardwareMap();

    // Define Webcam
    OpenCvCamera webcam;

    // Create Pipeline
    static OpenCV_Pipeline pipeline;

    @Override
    public void runOpMode() {


        robot.init(hardwareMap);


        // Set up webcam
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        webcam = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "TurretCam1"), cameraMonitorViewId);

        // Set up pipeline
        pipeline = new OpenCV_Pipeline();
        webcam.setPipeline(pipeline);



        // Start camera streaming
        webcam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened() {
                webcam.startStreaming(640, 480, OpenCvCameraRotation.UPRIGHT);
                FtcDashboard dashboard = FtcDashboard.getInstance();
                telemetry = new MultipleTelemetry(telemetry, dashboard.getTelemetry());
                FtcDashboard.getInstance().startCameraStream(webcam, 10);
            }

            @Override
            public void onError(int errorCode)
            {
                telemetry.addData("cameraNotOpened", 0);
                telemetry.update();
                /*
                 * This will be called if the camera could not be opened
                 */
            }
        });

        telemetry.addLine("Waiting for start");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()){
            // Telemetry readings for the HSV values in each region
//            telemetry.addData("Region 1", "%7d, %7d, %7d", pipeline.RGB_Value[0], pipeline.RGB_Value[1], pipeline.RGB_Value[2]);

            telemetry.addData("targetX", pipeline.targetX);
            telemetry.addData("targetY", pipeline.targetY);
            telemetry.addData("targetWidth", pipeline.targetWidth);
            telemetry.addData("target Area", pipeline.targetArea);

            cubeDifference = 320 - pipeline.cubeCenter;
            if(Math.abs(cubeDifference) < 10){
                cubeChange = 0;
            }else{
                cubeChange = cubeDifference * followP;
            }

            teleOpRotateSet = teleOpRotateSet - cubeChange;


            if(gamepad1.left_bumper && robot.I_DS.getDistance(DistanceUnit.INCH) > 2){
                if(pipeline.cubeCenter > 300 && pipeline.cubeCenter < 340){
                    teleOpExtendSet = teleOpExtendSet + 50;
                }
                robot.RI_S.setPower(-.5);
                robot.LI_S.setPower(.5);
            }else{
                teleOpExtendSet = 275;
                teleOpVPivotSet = 500;
                robot.RI_S.setPower(0);
                robot.LI_S.setPower(0);
            }
            if(pipeline.targetX < 0){
                teleOpRotateSet = 0;
            }

            if(gamepad1.left_trigger < .1){
                teleOpRotateSet = 0;
            }



            if(gamepad1.y){
                controllerY = true;
            }else if(gamepad2.y){
                controllerY = true;
            }else{
                controllerY = false;
            }

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



        CombinedTurret.TurretCombinedMethod(teleOpExtendSet,teleOpExtendSpeedSet,teleOpRotateSet,teleOpRotateSpeedSet, teleOpVPivotSet,teleOpVPivotSpeedSet, robot.TE_M.getCurrentPosition(), robot.TE_G.getState(), robot.TR_M.getCurrentPosition(), robot.TR_G.getState(), robot.TP_M.getCurrentPosition(), robot.TP_G.getState());

        robot.TE_M.setPower(CombinedTurret.extendFinalMotorPower);
        robot.TR_M.setPower(CombinedTurret.rotateFinalMotorPower);
        robot.TP_M.setPower(CombinedTurret.vPivotFinalMotorPower);

            telemetry.update();
        }
        webcam.stopStreaming();
        webcam.closeCameraDevice();


    }


    public static class OpenCV_Pipeline extends OpenCvPipeline {

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

        Rect yellowMask;
        Rect whiteMask;
        Rect redMask;

        List<MatOfPoint> yellowContours;
        List<MatOfPoint> whiteContours;
        List<MatOfPoint> redContours;

        // Make a Constructor
        public OpenCV_Pipeline() {
            yellowContours = new ArrayList<MatOfPoint>();
            whiteContours = new ArrayList<MatOfPoint>();
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

            Imgproc.cvtColor(input, HSV,Imgproc.COLOR_RGB2HSV);

            Scalar scalarLowerYCrCb = new Scalar(Hmin, Smin, Vmin);
            Scalar scalarUpperYCrCb = new Scalar(Hmax, Smax, Vmax);
            //Scalar scalarLowerYCrCb = new Scalar(15.0, 100.0, 120.0);
            //Scalar scalarUpperYCrCb = new Scalar(45.0, 255.0, 255.0);
            Mat maskRed = new Mat();
            //BLUE DO NOT REMOVE
            //Scalar scalarLowerYCrCb = new Scalar(80.0, 70.0, 100.0);
            //Scalar scalarUpperYCrCb = new Scalar(180.0, 255.0, 255.0);

            //inRange(HSV, lowYellow, highYellow, maskYellow);
            //inRange(HSV, lowWhite, highWhite, maskWhite);
            inRange(HSV, scalarLowerYCrCb, scalarUpperYCrCb, maskRed);

//            Core.inRange();

            yellowContours.clear();
            whiteContours.clear();
            redContours.clear();


            Imgproc.findContours(maskRed, redContours, new Mat(), Imgproc.RETR_TREE, Imgproc.CHAIN_APPROX_SIMPLE);
            Imgproc.drawContours(input, redContours, -1, AQUA); //input


            yLowest = -640;
            indexLowest = 0;

            if(redContours.size() > 0){
                for (int i = 0; i < redContours.size(); i++){
                    if (filterContours(redContours.get(i))){
                        redMask = Imgproc.boundingRect(redContours.get(i));
                        Imgproc.rectangle(input, redMask, AQUA, 2);

                        if(Math.abs((redMask.y + redMask.height) - yLowest) < 80){
                            if(redMask.x + redMask.width > yLeft){
                                indexLowest = i;
                                yLowest = redMask.y + redMask.height;
                                yLeft = redMask.x + redMask.width;
                            }
                        }else if(redMask.y + redMask.height > yLowest){
                            indexLowest = i;
                            yLowest = redMask.y + redMask.height;
                            yLeft = redMask.x + redMask.width;
                        }
                    }
                }
                redMask = Imgproc.boundingRect(redContours.get(indexLowest));
                Imgproc.rectangle(input, redMask, PARAKEET, -5);
                targetX = redMask.x;
                targetY = redMask.y;
                targetWidth = redMask.width;
                targetArea = redMask.height * redMask.width;
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
            maskRed.release();


            return input;
        }
    }
}
