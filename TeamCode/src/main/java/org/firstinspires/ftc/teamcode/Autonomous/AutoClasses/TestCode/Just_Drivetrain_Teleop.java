package org.firstinspires.ftc.teamcode.Autonomous.AutoClasses.TestCode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

//@TeleOp
public class Just_Drivetrain_Teleop extends LinearOpMode {



        Just_DrivetrainHardware robot = new Just_DrivetrainHardware();

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
                    robot.LF_M.setPower(.4*((y)-x+(z)));//LF
                    robot.LB_M.setPower(.4*((y)+x+(z)));//LB
                    robot.RF_M.setPower(.4*(-((y)+x-(z))));//RF
                    robot.RB_M.setPower(.4*(-((y)-x-(z))));//RB
                }else{
                    robot.LF_M.setPower((y)-x+(z));//LF
                    robot.LB_M.setPower((y)+x+(z));//LB
                    robot.RF_M.setPower(-((y)+x-(z)));//RF
                    robot.RB_M.setPower(-((y)-x-(z)));//RB
                }

                telemetry.addData("LF", robot.LF_M.getPower());
                telemetry.addData("LB", robot.LB_M.getPower());
                telemetry.addData("RF", robot.RF_M.getPower());
                telemetry.addData("RB", robot.RB_M.getPower());


            }
        }


}
