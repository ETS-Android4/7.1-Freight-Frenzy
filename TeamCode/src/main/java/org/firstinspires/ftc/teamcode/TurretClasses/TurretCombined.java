package org.firstinspires.ftc.teamcode.TurretClasses;

import org.firstinspires.ftc.teamcode.GeneralRobotCode.FreightFrenzyHardwareMap;

public class TurretCombined {
    FreightFrenzyHardwareMap robot = new FreightFrenzyHardwareMap();


    //setting varibles for use in calculating corrections for 3 axes of our turret
    public double extendSet, rotateSet, vPivotSet, extendEncoder, rotateEncoder, vPivotEncoder;
    double timeInSec = robot.TimerCustom(), lastTime = 0;
    public double extendDeltaEncoder, extendModifiedEncoder, rotateDeltaEncoder, rotateModifiedEncoder, vPivotDeltaEncoder, vPivotModifiedEncoder;
    double extendLastEncoder, rotateLastEncoder, vPivotLastEncoder;
    double extendDirection, rotateDirection, vPivotDirection;
    double extendSpeedSet, rotateSpeedSet, vPivotSpeedSet;
    public double extendSpeed, rotateSpeed, vPivotSpeed;
    double extendEncoderTickPerIn = 53,vPivotEncoder1Degree = 23 ;
    public double vPivotPM, vPivotDM, vPivotUpPm = .028, vPivotDnPM = .018, vPivotUpDM = .02, vPivotDnDM = .015;
    public double extendPM = .015, extendDM = 0.012;
    public double rotatePM = .0003, rotateDM = 0.0003;
    double extendSpeedDifference, rotateSpeedDifference, vPivotSpeedDifference;
    double extendLastSpeedDifference = 0, rotateLastSpeedDifference = 0, vPivotLastSpeedDifference = 0;
    public double extendFinalMotorPower = 0, rotateFinalMotorPower = 0, vPivotFinalMotorPower = 0;
    public double cosineMult = 1, currentDegree;
    public boolean turretHomingTrigger = true, vPivotIsHomed = false, lastVPivotMag = false, homing1loop = false, extendStart = false;
    public boolean extendHomingHasExtended = false, extendIsHomed = false, rotateMagStart, rotateIsHomed = false, rotateHomingHasMoved = false;

    //the main turret method and inputting parameters to run the calculations
    public void TurretCombinedMethod(double ExtendSet, double ExtendSpeedSet, double RotateSet, double RotateSpeedSet,
                                     double VPivotSet, double VPivotSpeedSet, double ExtendEncoder, boolean ExtendMag,
                                     double RotateEncoder, boolean RotateMag, double VPivotEncoder, boolean VPivotMag){
        //setting a time variable for use to calculate speed
        timeInSec = robot.TimerCustom();



        //setting our encoder parameters to variables to reverse some encoders
        extendEncoder = ExtendEncoder;
        rotateEncoder = RotateEncoder;
        vPivotEncoder = VPivotEncoder;



        //homing sequence to home to turret
        if(turretHomingTrigger == true){
            if(homing1loop = false){
                extendStart = ExtendMag;
                homing1loop = true;
            }
            //vPivot Homing
            if(vPivotIsHomed == false){
                vPivotSet = 3000;
                vPivotSpeedSet = 10;
            }
            if(VPivotMag == false && lastVPivotMag == true){
                vPivotModifiedEncoder = 1600;
                vPivotSet = 1500;
                vPivotIsHomed = true;
            }
            if(vPivotIsHomed == true){
                //extend homing
                if(extendStart == false && ExtendMag == false && extendIsHomed == false){
                    extendSet = extendSet + 20;
                    extendSpeedSet = 10;
                }else if(extendIsHomed == false){
                    extendSet = extendSet - 20;
                    extendSpeedSet = 10;
                    extendHomingHasExtended = true;
                }
                if(extendHomingHasExtended == true && ExtendMag == false && extendIsHomed == false){
                    extendModifiedEncoder = 0;
                    extendSet = 0;
                    extendIsHomed = true;
                }
                //rotate Homing
                if(rotateMagStart == false && RotateMag == false && rotateIsHomed == false){
                    rotateSet = rotateSet + 15;
                    rotateSpeedSet = 1000;
                }else if(rotateIsHomed == false){
                    rotateSet = rotateSet - 15;
                    rotateSpeedSet = 1000;
                    rotateHomingHasMoved = true;
                }

                if(rotateHomingHasMoved == true && RotateMag == false && rotateIsHomed == false){
                    rotateSet = 0;
                    rotateModifiedEncoder = 20;
                    rotateIsHomed = true;

                }
                if(rotateIsHomed == true && extendIsHomed == true && vPivotIsHomed == true){
                    turretHomingTrigger = false;
                }
            }

        }else {

            //setpoint limits
            if (ExtendSet > 1290) {
                extendSet = 1290;
            } else {
                extendSet = ExtendSet;
            }

            if (RotateSet > 5000) {
                rotateSet = 5000;
            } else if (RotateSet < -5000) {
                rotateSet = -5000;
            } else {
                rotateSet = RotateSet;
            }

            if (VPivotSet > 2600) {
                vPivotSet = 2600;
            } else if (VPivotSet < 200) {
                vPivotSet = 200;
            } else {
                vPivotSet = VPivotSet;
            }

        }

        //setting the current encoder to a variable to reset positions using sensors
        extendDeltaEncoder = extendEncoder - extendLastEncoder;
        extendModifiedEncoder = extendModifiedEncoder + extendDeltaEncoder;

        rotateDeltaEncoder = rotateEncoder - rotateLastEncoder;
        rotateModifiedEncoder = rotateModifiedEncoder + rotateDeltaEncoder;

        vPivotDeltaEncoder = vPivotEncoder - vPivotLastEncoder;
        vPivotModifiedEncoder = vPivotModifiedEncoder + vPivotDeltaEncoder;



        //setting which direction each axis has to move to get to it's setpoint
        if(extendSet < extendModifiedEncoder){
            extendDirection = -1;
        }else if(extendSet > extendModifiedEncoder){
            extendDirection = 1;
        }

        if(rotateSet < rotateModifiedEncoder){
            rotateDirection = -1;
        }else if(rotateSet > rotateModifiedEncoder){
            rotateDirection = 1;
        }

        if(vPivotSet < vPivotModifiedEncoder){
            vPivotDirection = 1;
        }else if(vPivotSet > vPivotModifiedEncoder){
            vPivotDirection = -1;
        }


        //setting our speed setpoint with the correct direction
        extendSpeedSet = Math.copySign(ExtendSpeedSet, extendDirection);
        rotateSpeedSet = Math.copySign(RotateSpeedSet, rotateDirection);
        vPivotSpeedSet = Math.copySign(VPivotSpeedSet, vPivotDirection);



        //Setting motion profiles and deadzones
        if(Math.abs(extendSet - extendModifiedEncoder) < Math.abs(ExtendSpeedSet * 10)){
            extendSpeedSet = extendSpeedSet * (Math.abs(extendSet - extendModifiedEncoder)/Math.abs(ExtendSpeedSet * 10));
        }

        if(Math.abs(rotateSet - rotateModifiedEncoder) < Math.abs(rotateSpeedSet/5)){
            rotateSpeedSet = rotateSpeedSet * Math.abs((rotateSet - rotateModifiedEncoder)/Math.abs(rotateSpeedSet/5));
        }else if( Math.abs(rotateSet - rotateModifiedEncoder) < 10){
            rotateSpeedSet = 0;
        }

        if(Math.abs(vPivotSet - vPivotModifiedEncoder) < 5){
            vPivotSpeedSet = 0;
        }else if(Math.abs(vPivotSet - vPivotModifiedEncoder) < 300){
            vPivotSpeedSet = vPivotSpeedSet * ((Math.abs(vPivotSet - vPivotModifiedEncoder))/300);
        }



        //calculating speed on all 3 axis
        extendSpeed = (extendDeltaEncoder/extendEncoderTickPerIn)/(timeInSec - lastTime);
        rotateSpeed = rotateDeltaEncoder/(timeInSec - lastTime);
        vPivotSpeed = -((2 * 16) * Math.PI)*((vPivotDeltaEncoder/vPivotEncoder1Degree) / 360) / (timeInSec - lastTime);

        //calculating the cosine multiplier, this is to let the arm correct with less power and the easier
        //to move position like lower or higher angles
        currentDegree = (vPivotModifiedEncoder - 1250)/vPivotEncoder1Degree;
        cosineMult = Math.cos(Math.toRadians(currentDegree));



        //setting which PD multiplier for the vertical pivot
        if(vPivotSet < vPivotModifiedEncoder){
            vPivotPM = vPivotDnPM;
            vPivotDM = vPivotDnDM;
        }else if(vPivotSet > vPivotModifiedEncoder){
            vPivotPM = vPivotUpPm;
            vPivotDM = vPivotUpDM;
        }



        //calculating the speed difference from speed set to actual speed
        extendSpeedDifference = extendSpeedSet - extendSpeed;
        rotateSpeedDifference = rotateSpeedSet - rotateSpeed;
        vPivotSpeedDifference = vPivotSpeedSet - vPivotSpeed;



        //calculating the correction motor power for each axis
        extendFinalMotorPower = extendFinalMotorPower + (extendSpeedDifference * extendPM) + ((extendSpeedDifference - extendLastSpeedDifference) * extendDM);
        rotateFinalMotorPower = rotateFinalMotorPower + (rotateSpeedDifference * rotatePM) + ((rotateSpeedDifference - rotateLastSpeedDifference) * rotateDM);
        vPivotFinalMotorPower = vPivotFinalMotorPower + (((vPivotSpeedDifference * vPivotPM) + ((vPivotSpeedDifference - vPivotLastSpeedDifference) * vPivotDM))* cosineMult);



        //limiting our motor powers to prevent motor overload and our correction loops from having a correction delay
        if(extendFinalMotorPower > 1){
            extendFinalMotorPower = 1;
        }else if(extendFinalMotorPower < -1){
            extendFinalMotorPower = -1;
        }

        if(rotateFinalMotorPower > 1){
            rotateFinalMotorPower = 1;
        }else if(rotateFinalMotorPower < -1){
            rotateFinalMotorPower = -1;
        }

        if(vPivotFinalMotorPower > 1){
            vPivotFinalMotorPower = 1;
        }else if(vPivotFinalMotorPower < -1){
            vPivotFinalMotorPower = -1;
        }



        lastTime = timeInSec;
        extendLastEncoder = extendEncoder;
        rotateLastEncoder = rotateEncoder;
        vPivotLastEncoder = vPivotEncoder;
        extendLastSpeedDifference = extendSpeedDifference;
        rotateLastSpeedDifference = rotateSpeedDifference;
        vPivotLastSpeedDifference = vPivotSpeedDifference;
        lastVPivotMag = VPivotMag;



    }
}
