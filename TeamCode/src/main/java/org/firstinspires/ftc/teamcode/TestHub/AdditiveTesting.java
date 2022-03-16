package org.firstinspires.ftc.teamcode.TestHub;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp
public class AdditiveTesting extends LinearOpMode {

    TestHubHardware robot = new TestHubHardware();
    double motor1current = 0;
    double motor2current = 0;
    double motor3current = 0;
    double motor4current = 0;
    double crservo1current = 0;
    double crservo2current = 0;
    public void runOpMode(){
        robot.init(hardwareMap);
        waitForStart();
        while (opModeIsActive()) {
            //motor1current = motor1current + gamepad1.left_stick_y;
            if(gamepad1.dpad_up){
                motor1current = motor1current + .005;
                motor2current = motor2current + .005;
                motor3current = motor3current + .005;
                motor4current = motor4current + .005;
                crservo1current = crservo1current + .005;
                crservo2current = crservo2current + .005;
            }else if(gamepad1.dpad_down){
                motor1current = motor1current - .005;
                motor3current = motor3current - 005;
                motor4current = motor4current - .005;
                motor2current = motor2current - .005;
                crservo1current = crservo1current - .005;
                crservo2current = crservo2current - .005;
            }
            if(gamepad1.b){
                motor1current = 0;
                motor2current = 0;
                motor3current = 0;
                motor4current = 0;
                crservo1current = 0;
                crservo2current = 0;
            }else if (gamepad1.a){
                motor1current = 1;
                motor2current = 1;
                motor3current = 1;
                motor4current = 1;
                crservo1current = .5;
                crservo2current = .5;
            }else if(gamepad1.x){
                motor1current = -1;
                motor2current = -1;
                motor3current = -1;
                motor4current = -1;
                crservo1current = -.5;
                crservo2current = -.5;
            }
            robot.Motor1.setPower(-motor1current);
            robot.Motor2.setPower(motor2current);
            robot.Motor3.setPower(motor3current);
            robot.Motor4.setPower(motor4current);
            robot.CRServo1.setPower(-crservo1current);
            robot.CRServo2.setPower(crservo2current);
            telemetry.addData("motor1", robot.Motor1.getPower());
            telemetry.addData("motor2", robot.Motor2.getPower());

            telemetry.update();

        }
    }

}

