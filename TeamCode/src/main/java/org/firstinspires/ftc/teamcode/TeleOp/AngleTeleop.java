package org.firstinspires.ftc.teamcode.TeleOp;

//@TeleOp
//public class AngleTeleop extends LinearOpMode {
  //  FreightFrenzyHardwareMap robot = new FreightFrenzyHardwareMap();
  /*  org.firstinspires.ftc.teamcode.TurretClasses.ExtendClass ExtendClass = new ExtendClass();
    org.firstinspires.ftc.teamcode.TurretClasses.VPivotClass VPivotClass = new VPivotClass();
    RotateClass RotateClass = new RotateClass();
    Odometry OdoClass = new Odometry();
    double x, y, z;
    double Vpivotcontroller = 1.15;
    double initPOsitionOrder = 1;
    double setpointAngle;
    double finalX, finalY;
    double LF_M_DIR, LB_M_DIR, RF_M_DIR, RB_M_DIR;
    double motorPowerRatio;
    double setpointAngleDeg;
    double speed;
    double finalAngle;
    public void runOpMode() {
        robot.init(hardwareMap);
       /* while (!opModeIsActive()) {
            if (RotateClass.isHomedRotateReturn() == false) {
                robot.TP_M.setPower(VPivotClass.VPivotAutoMethod(1.15, .5, robot.TP_P.getVoltage()));
                if (robot.TP_P.getVoltage() > 1.1 && robot.TP_P.getVoltage() < 1.25) {
                    robot.TE_M.setPower(ExtendClass.ExtendHoming(robot.TE_G.getState(), robot.TE_M.getCurrentPosition()));
                    if (ExtendClass.isHomedExtendReturn() == true) {
                        robot.TR_M.setPower(RotateClass.RotateHoming(robot.TR_G.getState(), robot.TR_M.getCurrentPosition()));
                    }
                }
            } else {
                telemetry.addData("homed", 0);
                robot.TE_M.setPower(ExtendClass.ExtendAutoMethod(10, .8, robot.TE_M.getCurrentPosition(), robot.TE_G.getState()));
                if (initPOsitionOrder == 1) {
                    robot.TR_M.setPower(RotateClass.RotateAutoMethod(0, .4, robot.TR_M.getCurrentPosition(), robot.TR_G.getState()));
                    if (RotateClass.modifiedRotateCurrent() > -50 && RotateClass.modifiedRotateCurrent() < 50) {
                        initPOsitionOrder = 2;
                    }
                } else if (initPOsitionOrder == 2) {
                    robot.TP_M.setPower(VPivotClass.VPivotAutoMethod(1.15, .5, robot.TP_P.getVoltage()));
                    if (robot.TP_P.getVoltage() < 1.25 && robot.TP_P.getVoltage() > 1) {
                        initPOsitionOrder = 3;
                    }
                } else if (initPOsitionOrder == 3) {
                    robot.TR_M.setPower(RotateClass.RotateAutoMethod(0, .4, robot.TR_M.getCurrentPosition(), robot.TR_G.getState()));
                    robot.TP_M.setPower(VPivotClass.VPivotAutoMethod(1.15, .5, robot.TP_P.getVoltage()));

                }


            }
            telemetry.addData("Rotate homed boolean", RotateClass.isHomedRotateReturn());
            telemetry.addData("initPosition order", initPOsitionOrder);
            telemetry.addData("Vpiovot PT", robot.TP_P.getVoltage());
            telemetry.addData("rotate modified", RotateClass.modifiedRotateCurrent());
            telemetry.update();
        }


        robot.LF_M.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.LF_M.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        robot.LB_M.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.LB_M.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        robot.RF_M.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.RF_M.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        waitForStart();
        while (opModeIsActive()) {
            z = gamepad1.right_stick_x;

                //getting exponential joystick control for the drivetrain
                OdoClass.RadiusOdometry(robot.LF_M.getCurrentPosition(), robot.LB_M.getCurrentPosition(), robot.RF_M.getCurrentPosition());
                x = -gamepad1.left_stick_x;
                y = -gamepad1.left_stick_y;

                setpointAngle = Math.atan2(x, y);
                setpointAngleDeg = Math.toDegrees(setpointAngle);
            /*
                setpointAngle = Math.atan2(y, x);
            if(x != 0){
                setpointAngle = Math.toDegrees(setpointAngle);
            }

            else if(y < 0){ //Joy stick up
                setpointAngle = 180;
            }
            else{// Joy stick down
                setpointAngle = 0;
            }


                finalAngle = setpointAngle + OdoClass.thetaINRadiansReturn();
                finalX = Math.sin(finalAngle) * 1;
                finalY = Math.cos(finalAngle) * 1;

        if(Math.abs(gamepad1.left_stick_x) <= 0.05 && Math.abs(gamepad1.left_stick_y) <= .05) {
            finalX = 0;
            finalY = 0;
        }
                    //setting the possiblity of a slow speed on the drivetrain
                LF_M_DIR = (.4*((finalY)-finalX+(z)));//LF
                LB_M_DIR = (.4*((finalY)+finalX+(z)));//LB
                RF_M_DIR = (.4*(-((finalY)+finalX-(z))));//RF
                RB_M_DIR = (.4*(-((finalY)-finalX-(z))));//RB
            motorPowerRatio = Math.max(Math.max(Math.abs(RF_M_DIR), Math.abs(RB_M_DIR)), Math.max(Math.abs(LF_M_DIR), Math.abs(LB_M_DIR)));

            LF_M_DIR = LF_M_DIR/motorPowerRatio;
            LB_M_DIR = LB_M_DIR/motorPowerRatio;
            RF_M_DIR = RF_M_DIR/motorPowerRatio;
            RB_M_DIR = RB_M_DIR/motorPowerRatio;
            speed = Math.hypot(x, y) + Math.abs(z);
            if(speed >= 1){
                speed = 1;
            }
            robot.LF_M.setPower(LF_M_DIR*speed);
            robot.LB_M.setPower(LB_M_DIR*speed);
            robot.RF_M.setPower(RF_M_DIR*speed);
            robot.RB_M.setPower(RB_M_DIR*speed);


            /*
            if(gamepad1.a || gamepad2.a){
                robot.RI_S.setPower(-1);
                robot.LI_S.setPower(1);
            }else if(gamepad1.b || gamepad2.b){
                robot.RI_S.setPower(.5);
                robot.LI_S.setPower(-.5);
            }else{
                robot.RI_S.setPower(0);
                robot.LI_S.setPower(0);
            }

            if(gamepad2.dpad_down){
                    //if arm is between deadzone of setpoint
                if(robot.TP_P.getVoltage() > 1.05 && robot.TP_P.getVoltage() < 1.25){
                        //rotate turret to "home"
                    robot.TR_M.setPower(RotateClass.RotateAutoMethod(0,.8, robot.TR_M.getCurrentPosition(),robot.TR_G.getState()));
                        //extend to 300 encoder ticks
                    robot.TE_M.setPower(ExtendClass.ExtendAutoMethod(400, .5, robot.TE_M.getCurrentPosition(), robot.TE_G.getState()));
                }
                //look to see if previous task is completed
                if(ExtendClass.extendModifiedEncoder > 380 && ExtendClass.extendModifiedEncoder < 420 && RotateClass.modifiedRotateCurrent() < 50 && RotateClass.modifiedRotateCurrent() > -50){
                        //pivot arm down to intaking position
                    robot.TP_M.setPower(VPivotClass.VPivotAutoMethod(1.75,.6,robot.TP_P.getVoltage()));
                        //hold rotate position at "home"
                    robot.TR_M.setPower(RotateClass.RotateAutoMethod(0,.8, robot.TR_M.getCurrentPosition(),robot.TR_G.getState()));
                        //Hold extend at 300 encoder ticks
                    robot.TE_M.setPower(ExtendClass.ExtendAutoMethod(400, .5, robot.TE_M.getCurrentPosition(), robot.TE_G.getState()));
                    Vpivotcontroller = VPivotClass.PivotSetReturn();
                }else{
                    //pivot to cleared height
                    robot.TP_M.setPower(VPivotClass.VPivotAutoMethod(1.15,.8,robot.TP_P.getVoltage()));
                }

            }else if(gamepad2.dpad_up){
                robot.TP_M.setPower(VPivotClass.VPivotAutoMethod(1.32,.7,robot.TP_P.getVoltage()));
                robot.TR_M.setPower(RotateClass.RotateMethod(gamepad2.right_trigger, gamepad2.left_trigger, robot.TR_M.getCurrentPosition(), robot.TR_G.getState()));
                robot.TE_M.setPower(ExtendClass.ExtendMethod((-gamepad2.left_stick_y), robot.TE_M.getCurrentPosition(), robot.TE_G.getState()));
                Vpivotcontroller = VPivotClass.PivotSetReturn();

            }else if(gamepad2.dpad_right){
                robot.TP_M.setPower(VPivotClass.VPivotAutoMethod(1.15,.9,robot.TP_P.getVoltage()));
                Vpivotcontroller = VPivotClass.PivotSetReturn();
                robot.TR_M.setPower(RotateClass.RotateAutoMethod(1400,.8, robot.TR_M.getCurrentPosition(),robot.TR_G.getState()));
                robot.TE_M.setPower(ExtendClass.ExtendMethod((-gamepad2.left_stick_y), robot.TE_M.getCurrentPosition(), robot.TE_G.getState()));
            }else if(gamepad2.dpad_left){

                if(RotateClass.modifiedRotateCurrent() > -1450 && RotateClass.modifiedRotateCurrent() < -1350){
                    robot.TP_M.setPower(VPivotClass.VPivotAutoMethod(1.6,.9,robot.TP_P.getVoltage()));
                    Vpivotcontroller = VPivotClass.PivotSetReturn();
                }else{
                    robot.TP_M.setPower(VPivotClass.VPivotAutoMethod(1.35,.9,robot.TP_P.getVoltage()));
                    Vpivotcontroller = VPivotClass.PivotSetReturn();
                }
                robot.TR_M.setPower(RotateClass.RotateAutoMethod(-1400,.8, robot.TR_M.getCurrentPosition(),robot.TR_G.getState()));
                robot.TE_M.setPower(ExtendClass.ExtendAutoMethod(0, .5, robot.TE_M.getCurrentPosition(), robot.TE_G.getState()));

            }else{


                    //else run TeleOp programs
                Vpivotcontroller = Vpivotcontroller +(.03 * gamepad2.right_stick_y);
                robot.TP_M.setPower(VPivotClass.VPivotAutoMethod(Vpivotcontroller ,1,robot.TP_P.getVoltage()));
                // robot.TP_M.setPower(VPivotClass.VPivotMethod(gamepad2.right_stick_y, robot.TP_P.getVoltage()));
                robot.TR_M.setPower(RotateClass.RotateMethod(gamepad2.right_trigger, gamepad2.left_trigger, robot.TR_M.getCurrentPosition(), robot.TR_G.getState()));

                robot.TE_M.setPower(ExtendClass.ExtendMethod((-gamepad2.left_stick_y), robot.TE_M.getCurrentPosition(), robot.TE_G.getState()));


            }
                */
            /*
            telemetry.addData("rotate motor power",robot.TR_M.getPower());
            telemetry.addData("rotate motor power", RotateClass.rotatemotorPowerReturn());
            telemetry.addData("extend motor power",robot.TE_M.getPower());
            telemetry.addData("pivot motor power", robot.TP_M.getPower());
            telemetry.addData("vpivotset", VPivotClass.PivotSetReturn());
            telemetry.addData("pivot POT", robot.TP_P.getVoltage());


            telemetry.addData("setpointAngle", setpointAngle);
            telemetry.addData("speed", speed);
            telemetry.addData("LF_M",robot.LF_M.getPower());
            telemetry.addData("finalX", finalX);
            telemetry.addData("finalY", finalY);
            telemetry.addData("x", x);
            telemetry.addData("y", y);
            telemetry.addData("theata in degrees", OdoClass.thetaInDegreesReturn());
            telemetry.update();
        }
    }
}*/