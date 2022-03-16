package org.firstinspires.ftc.teamcode.TestHub;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

//@TeleOp
public class Mecanum extends  LinearOpMode {



        TestHubHardware robot = new TestHubHardware();

        public void runOpMode() {
            double x, y, z;
            robot.init(hardwareMap);
            waitForStart();
            while (opModeIsActive()) {
                if(gamepad1.left_stick_x < 0){
                    x = (gamepad1.left_stick_x * gamepad1.left_stick_x);
                }else{
                    x = -(gamepad1.left_stick_x * gamepad1.left_stick_x);
                }
                if(gamepad1.left_stick_y < 0){
                    y = (gamepad1.left_stick_y * gamepad1.left_stick_y);
                }else{
                    y = -(gamepad1.left_stick_y * gamepad1.left_stick_y);
                }
                if(gamepad1.right_stick_x < 0){
                    z = -(gamepad1.right_stick_x * gamepad1.right_stick_x);
                }else{
                    z = gamepad1.right_stick_x * gamepad1.right_stick_x;
                }

                //LFM = (-y) - ((-x) + (-z));
                //LBM = (-y) + ((-x) - (-z));
                //RFM = (-y) + ((-x) + (-z));
                //RBM = (-y) - ((-x) - (-z));
                //robot.Motor1.setPower((gamepad1.left_stick_y)-gamepad1.left_stick_x+(gamepad1.right_stick_x));//LF
                //robot.Motor2.setPower((gamepad1.left_stick_y)+gamepad1.left_stick_x+(gamepad1.right_stick_x));//LB
                //robot.Motor3.setPower(-((gamepad1.left_stick_y)+gamepad1.left_stick_x-(gamepad1.right_stick_x)));//RF
                //robot.Motor4.setPower(-((gamepad1.left_stick_y)-gamepad1.left_stick_x-(gamepad1.right_stick_x)));//RB

                if(gamepad1.right_bumper){
                    robot.Motor1.setPower(.4*((y)-x+(z)));//LF
                    robot.Motor2.setPower(.4*((y)+x+(z)));//LB
                    robot.Motor3.setPower(.4*(-((y)+x-(z))));//RF
                    robot.Motor4.setPower(.4*(-((y)-x-(z))));//RB
                }else{
                    robot.Motor1.setPower((y)-x+(z));//LF
                    robot.Motor2.setPower((y)+x+(z));//LB
                    robot.Motor3.setPower(-((y)+x-(z)));//RF
                    robot.Motor4.setPower(-((y)-x-(z)));//RB
                }

                telemetry.addData("motor1", robot.Motor1.getPower());
                telemetry.addData("motor2", robot.Motor2.getPower());
                telemetry.addData("motor3", robot.Motor3.getPower());
                telemetry.addData("motor4", robot.Motor4.getPower());


            }
        }


}
