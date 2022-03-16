package org.firstinspires.ftc.teamcode.TestHub;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

//@TeleOp
public class ProximityColorSensorTestRobot extends LinearOpMode {

    TestHubHardware robot = new TestHubHardware();

    public void runOpMode(){
        robot.init(hardwareMap);
        waitForStart();
        while (opModeIsActive()) {
            //telemetry.addData("IN", robot.PC.getDistance(DistanceUnit.INCH));
            telemetry.update();

        }
    }

}

