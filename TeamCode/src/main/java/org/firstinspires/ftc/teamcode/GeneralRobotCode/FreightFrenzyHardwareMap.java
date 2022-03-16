package org.firstinspires.ftc.teamcode.GeneralRobotCode;

import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

public class FreightFrenzyHardwareMap {
    // Motors and Servos
    public DcMotor LF_M;
    public DcMotor LB_M;
    public DcMotor RF_M;
    public DcMotor RB_M;
    public DcMotor TR_M;
    public DcMotor TP_M;
    public DcMotor TE_M;
    public DcMotor TC_M;
    public CRServo RI_S;
    public CRServo LI_S;

    public DigitalChannel TR_G;
    public DigitalChannel TE_G;
    public DigitalChannel TP_G;
    public RevColorSensorV3 I_DS;
    public ColorSensor LF_C;
    public ColorSensor  LB_C;
    public ColorSensor  RF_C;
    public ColorSensor  RB_C;



    HardwareMap TestHubHardware;

    public void init(HardwareMap TestHubHardware){

        // Define motors and servos
        LF_M = TestHubHardware.get(DcMotor.class, "LF_M");
        LB_M = TestHubHardware.get(DcMotor.class, "LB_M");
        RF_M = TestHubHardware.get(DcMotor.class, "RF_M");
        RB_M = TestHubHardware.get(DcMotor.class, "RB_M");
        TR_M = TestHubHardware.get(DcMotor.class, "TR_M");
        TP_M = TestHubHardware.get(DcMotor.class, "TP_M");
        TE_M = TestHubHardware.get(DcMotor.class, "TE_M");
        RI_S = TestHubHardware.get(CRServo.class, "RI_S");
        LI_S = TestHubHardware.get(CRServo.class, "LI_S");
        TC_M = TestHubHardware.get(DcMotor.class, "TC_M");

        TR_G = TestHubHardware.get(DigitalChannel.class, "TR_G");
        TE_G = TestHubHardware.get(DigitalChannel.class, "TE_G");
        TP_G = TestHubHardware.get(DigitalChannel.class, "TP_G");
        I_DS = TestHubHardware.get(RevColorSensorV3.class, "I_DS");


        LF_C = TestHubHardware.get(ColorSensor .class, "LF_C");
        LB_C = TestHubHardware.get(ColorSensor.class, "LB_C");
        RF_C = TestHubHardware.get(ColorSensor .class, "RF_C");
        RB_C = TestHubHardware.get(ColorSensor .class, "RB_C");
        // ColorSensor1 = JustmotorhardwareMap.get(NormalizedColorSensor.class, "ColorSensor1");

        //servo = hardwareMap.get(Servo.class, "servo");

        // Set all motors to zero power

        // Set all motors to run without encoders.
        // May want to use RUN_USING_ENCODERS if encoders are installed.
        LF_M.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        LB_M.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        RF_M.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        RB_M.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        TR_M.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        TP_M.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        TE_M.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        TC_M.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        TC_M.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        LF_M.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        LB_M.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        RF_M.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        RB_M.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        TR_M.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        TP_M.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        TE_M.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        LF_M.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        LB_M.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        RF_M.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        RB_M.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        TR_M.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        TP_M.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        TE_M.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        TC_M.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);


        TR_G.setMode(DigitalChannel.Mode.INPUT);
        TE_G.setMode(DigitalChannel.Mode.INPUT);
        TP_G.setMode(DigitalChannel.Mode.INPUT);

        RobotInitTimer.startTime();
    }
    ElapsedTime RobotInitTimer = new ElapsedTime();

    public double TimerCustom(){

        return RobotInitTimer.seconds();
    }

}
