package org.firstinspires.ftc.teamcode.TestHub;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

//@TeleOp
public class ColorSensorTest extends LinearOpMode {

    TestHubHardware robot = new TestHubHardware();

    public void runOpMode(){
        robot.init(hardwareMap);
        waitForStart();
        while (opModeIsActive()) {
            //NormalizedRGBA ColorSensor = robot.ColorSensor.getNormalizedColors();

           // telemetry.addData("color1", ColorSensor.alpha);
           // telemetry.addData("color1", ColorSensor.red);
           // telemetry.addData("color1", ColorSensor.blue);
           // telemetry.addData("color1", ColorSensor.green);

            if(gamepad1.a){
                //robot.LED1.setState(true);
            }else{
                //robot.LED1.setState(false);
            }





            telemetry.update();

        }
    }

}

