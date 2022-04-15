package org.firstinspires.ftc.teamcode.GeneralRobotCode.OpenCVTestFolder;

import static org.opencv.core.Core.inRange;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
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

//@Autonomous
public class Cube_Detection extends LinearOpMode {

    // Define Webcam
    OpenCvCamera webcam;

    // Create Pipeline
    static OpenCV_Pipeline pipeline;

    @Override
    public void runOpMode() {

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
        double targetX; double targetY; double targetWidth; double targetArea;
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
            return Imgproc.contourArea(contour) > 150;
        }

        @Override
        public Mat processFrame(Mat input) {

            Mat YCrCb = new Mat();
            Mat HSV = new Mat();

            Imgproc.cvtColor(input, HSV, Imgproc.COLOR_RGB2HSV);

            Scalar scalarLowerYCrCb = new Scalar(  10.0, 160.0, 160);
            Scalar scalarUpperYCrCb = new Scalar(25.0, 255.0, 255.0);
            Mat maskRed = new Mat();


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

                        if(redMask.y > yLowest){
                            indexLowest = i;
                            yLowest = redMask.y;
                        }
                    }
                }
                redMask = Imgproc.boundingRect(redContours.get(indexLowest));
                Imgproc.rectangle(input, redMask, GOLD, -5);
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

            }





            YCrCb.release();
            HSV.release();
            maskRed.release();


            return input;
        }
    }
}
