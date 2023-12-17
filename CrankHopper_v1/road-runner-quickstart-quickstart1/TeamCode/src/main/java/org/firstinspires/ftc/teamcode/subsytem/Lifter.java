package org.firstinspires.ftc.teamcode.subsytem;



import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.control.PIDCoefficients;
import com.acmerobotics.roadrunner.control.PIDFController;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.ftccommon.internal.manualcontrol.responses.PidCoefficients;
import org.firstinspires.ftc.ftccommon.internal.manualcontrol.responses.PidfCoefficients;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
@Config
public class Lifter {

    public DcMotorEx leftElevator, rightElevator;

    public final int HOME_POSITION = 0, LOW = 1, MID = 2, HIGH = 3, low_mid= 4;


    public final int[] POSITIONS = {-402,120, 410, 650,-150};

    //
    public static double Kp = 0.01;
    public static double Ki = 0;
    public static double Kd = 0;
    public static double Kf = 0;

    private PIDFController LifterController;

    public static double targetLift = 0;




    public Lifter(HardwareMap hardwareMap, Telemetry telemetry){
        leftElevator = hardwareMap.get(DcMotorEx.class, "ArmL");
        rightElevator = hardwareMap.get(DcMotorEx.class, "ArmR");

        leftElevator.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightElevator.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftElevator.setDirection(DcMotorSimple.Direction.REVERSE);

        leftElevator.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightElevator.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);



    }


    public void extendTo(int position, double power){

        leftElevator.setTargetPosition(position);
        leftElevator.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        leftElevator.setPower(power);


        rightElevator.setTargetPosition(position);
        rightElevator.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightElevator.setPower(power);
    }

    public void extendToCustom(int positionLeft,int positionRight, double power){

        leftElevator.setTargetPosition(positionLeft);
        leftElevator.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        leftElevator.setPower(power);

        rightElevator.setTargetPosition(positionRight);
        rightElevator.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightElevator.setPower(power);


    }

    public void extendToInit(){
        extendTo(POSITIONS[0], 0.7);
    }


    public void extendToLow(){
        extendTo(POSITIONS[LOW],0.7);
    }

    public void extendToMedium(){
        extendTo(POSITIONS[MID],0.7);
    }

    public void extendToHigh(){extendTo(POSITIONS[HIGH],0.7);}

    public void extendTolow_mid(){extendTo(POSITIONS[low_mid],0.7);}

    public void extendToHomePos(){


        LifterController.setTargetPosition(0);


//


    }

    public void lifterINC(){
        extendTo(leftElevator.getCurrentPosition()+10,1);
    }
    public void lifterDEC(){extendTo(leftElevator.getCurrentPosition()-10,1);}




    public double[] getPosition(){
        return new double[]{leftElevator.getCurrentPosition(), rightElevator.getCurrentPosition()};
    }

    public void reset(){
        leftElevator.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightElevator.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }

    public double[] getCurrent(){
        return new double[]{leftElevator.getCurrent(CurrentUnit.AMPS), rightElevator.getCurrent(CurrentUnit.AMPS)};
    }
}
