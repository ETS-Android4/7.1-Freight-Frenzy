package org.firstinspires.ftc.teamcode.TestHub;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

//@TeleOp

public class LEDTest extends LinearOpMode {

    TestHubHardware robot = new TestHubHardware();

    @Override
    public void runOpMode() {
        robot.init(hardwareMap);
        waitForStart();
        while (opModeIsActive()) {
                robot.greenLED.setState(true);
                robot.redLED.setState(false);
            }
        }
    }

