package org.firstinspires.ftc.teamcode.TeleOp;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.GeneralRobotCode.FreightFrenzyHardwareMap;
import org.firstinspires.ftc.teamcode.GeneralRobotCode.Smoothing;
import org.firstinspires.ftc.teamcode.TurretClasses.TurretCombined;


//@TeleOp

public class RyanTeleOpStuff extends LinearOpMode{


    double tape_Setpoint = 1000;
    double tape_Speed_Set = 100;//in/second
    double tape_direction = 1;
    double tape_Servo_Power = 0;
    double tape_Current_Speed = 0;
    double tape_lat_Time = 0;
    double tape_last_Encoder = 0;




    FreightFrenzyHardwareMap robot = new FreightFrenzyHardwareMap();



    @Override
    public void runOpMode(){
        robot.init(hardwareMap);


        waitForStart();

        while (opModeIsActive()){

            if(gamepad1.a){
                tape_Setpoint = tape_Setpoint + 10;
            }else if(gamepad1.b){
                tape_Setpoint = tape_Setpoint - 10;
            }

            if(Math.abs(tape_Setpoint - robot.TC_M.getCurrentPosition()) < 50){
                tape_Speed_Set = tape_Speed_Set * ((Math.abs(tape_Setpoint - robot.TC_M.getCurrentPosition()))/50);
            }else{
                tape_Speed_Set = 100;
            }

            if(robot.TC_M.getCurrentPosition() < tape_Setpoint){
                tape_direction = 1;
            }else{
                tape_direction = -1;
            }

            tape_Speed_Set = Math.copySign(tape_Speed_Set, tape_direction);

            tape_Current_Speed = (tape_last_Encoder - robot.TC_M.getCurrentPosition())/(tape_lat_Time - getRuntime());

            tape_Servo_Power = tape_Servo_Power + (tape_Speed_Set - tape_Current_Speed) * .001;



            robot.TC_M.setPower(tape_Servo_Power);

            tape_last_Encoder = robot.TC_M.getCurrentPosition();
            tape_lat_Time = getRuntime();

        }

    }


}
