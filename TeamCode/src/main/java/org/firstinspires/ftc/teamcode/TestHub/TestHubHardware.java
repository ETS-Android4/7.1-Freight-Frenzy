package org.firstinspires.ftc.teamcode.TestHub;

import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class TestHubHardware {
    // Motors and Servos
    public DcMotor Motor1;
    public DcMotor Motor2;
    public DcMotor Motor3;
    public DcMotor Motor4;
    public CRServo CRServo1;
    public CRServo CRServo2;
    public AnalogInput PivotPT;
    public DigitalChannel redLED;
    public DigitalChannel greenLED;
    //public NormalizedColorSensor ColorSensor;
   // DigitalChannel MagneticExtend;
   // DigitalChannel MagneticRotate;


    HardwareMap TestHubHardware;

    public void init(HardwareMap TestHubHardware){

        // Define motors and servos
        Motor1 = TestHubHardware.get(DcMotor.class, "Motor1");
        Motor2 = TestHubHardware.get(DcMotor.class, "Motor2");
        Motor3 = TestHubHardware.get(DcMotor.class, "Motor3");
        Motor4 = TestHubHardware.get(DcMotor.class, "Motor4");
        CRServo1 = TestHubHardware.get(CRServo.class, "CRServo1");
        CRServo2 = TestHubHardware.get(CRServo.class, "CRServo2");
        PivotPT = TestHubHardware.get(AnalogInput.class, "PivotPT");
        redLED= TestHubHardware.get(DigitalChannel.class, "redLED");
        greenLED = TestHubHardware.get(DigitalChannel.class, "greenLED");

     //   MagneticExtend = TestHubHardware.get(DigitalChannel.class, "MagneticTest");
     //   MagneticRotate = TestHubHardware.get(DigitalChannel.class, "MagneticRotate");

       // ColorSensor = TestHubHardware.get(NormalizedColorSensor.class, "ColorSensor");
        // ColorSensor1 = JustmotorhardwareMap.get(NormalizedColorSensor.class, "ColorSensor1");

        //servo = hardwareMap.get(Servo.class, "servo");

        // Set all motors to zero power

        // Set all motors to run without encoders.
        // May want to use RUN_USING_ENCODERS if encoders are installed.
        Motor1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        Motor2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        Motor3.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        Motor4.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        Motor1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        Motor2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        Motor3.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        Motor4.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        Motor1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        Motor4.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        Motor3.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        Motor2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        redLED.setMode(DigitalChannel.Mode.OUTPUT);
        greenLED.setMode(DigitalChannel.Mode.OUTPUT);




     //   MagneticRotate.setMode(DigitalChannel.Mode.INPUT);
       // MagneticExtend.setMode(DigitalChannel.Mode.INPUT);
    }

}
