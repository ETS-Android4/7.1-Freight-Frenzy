package org.firstinspires.ftc.teamcode.Autonomous.AutoClasses.TestCode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class Just_DrivetrainHardware {
    // Motors and Servos
    public DcMotor LF_M;
    public DcMotor LB_M;
    public DcMotor RF_M;
    public DcMotor RB_M;

    HardwareMap TestHubHardware;

    public void init(HardwareMap TestHubHardware){

        // Define motors and servos
        LF_M = TestHubHardware.get(DcMotor.class, "LF_M");
        LB_M = TestHubHardware.get(DcMotor.class, "LB_M");
        RF_M = TestHubHardware.get(DcMotor.class, "RF_M");
        RB_M = TestHubHardware.get(DcMotor.class, "RB_M");


        // ColorSensor1 = JustmotorhardwareMap.get(NormalizedColorSensor.class, "ColorSensor1");

        //servo = hardwareMap.get(Servo.class, "servo");

        // Set all motors to zero power

        // Set all motors to run without encoders.
        // May want to use RUN_USING_ENCODERS if encoders are installed.
        LF_M.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        LB_M.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        RF_M.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        RB_M.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        LF_M.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        LB_M.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        RF_M.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        RB_M.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        LF_M.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        LB_M.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        RF_M.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        RB_M.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

    }

}
