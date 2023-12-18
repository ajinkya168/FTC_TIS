package org.firstinspires.ftc.teamcode.subsytem;

import static android.os.SystemClock.sleep;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;


@Config
public class Intake {

    public DcMotorEx IntakeMotor=null;
    public static Servo InR;
    public static Servo InL;

    public String intakeState="INIT";

    public static double leftServoGrippingPos = 0.48;//left servo 0.1594 for intake of  pixels

    public static double rightServoGrippingPos = (1.0 - leftServoGrippingPos);

    public static double leftServoInitPos = 0.2; //0.15 // left servo 0.829 for init
    public static double rightServoInitPos = (1.0 - leftServoInitPos);
    public static double LeftAutoStartIntakePose = 0.3;
    public static double RightAutoStartIntakePose = (1.0 - LeftAutoStartIntakePose);
    public static double CycleIntakrLeftServo = 0.1;
    public static double CycleIntakrRightServo = (1.0 - CycleIntakrLeftServo);

    public static double rightServoReverseIntakePos = 0.4;
    public static double leftServoReverseIntakePos = (1.0 - rightServoReverseIntakePos);

    public static int i = 0;


    //distance of chassis from the wall = 149mm

    //left servo for second pixel = 0.609444444444
    //left servo for third pixel = 0.5794444
    //left servo for fourth pixel = 0.549444
    //left servo for first pixel = 0.63944444
    public static double pickFirstPixel = 0.3;//0.63944444;
    public static double pickSecondPixel = pickFirstPixel+0.03;//0.609444444
    public static double pickThirdPixel = pickSecondPixel+0.03;//0.579444444
    public static double pickFourthPixel = pickThirdPixel+0.03;//0.54944444



    //INTAKE MOTOR
    public static double intakeOnFor=1;
    public static double intakeREV=0.5;

    public Intake(HardwareMap hardwareMap, Telemetry telemetry) {
        InR = hardwareMap.get(Servo.class, "InR");
        InL = hardwareMap.get(Servo.class, "InL");
        IntakeMotor = hardwareMap.get(DcMotorEx.class, "In");

    }

    //TODO INTAKE SERVOS

    public void intakeInit(){
        intakeState="INIT";
        InR.setPosition(leftServoInitPos);
        InL.setPosition(rightServoInitPos);
    }
    public void intakeGrip(){
        intakeState="GRIP";
        InR.setPosition(leftServoGrippingPos);
        InL.setPosition(rightServoGrippingPos);
    }
    public void intakeAutoStart(){
        intakeState="GRIP";
        InR.setPosition(LeftAutoStartIntakePose);
        InL.setPosition(RightAutoStartIntakePose);
    }
    public void intakeCycle(){
        InR.setPosition(CycleIntakrRightServo);
        InL.setPosition(CycleIntakrLeftServo);
    }

    public void intakeStack(double PF){
        intakeState="GRIP";
        leftServoGrippingPos =PF;
        rightServoGrippingPos = (1.0 - leftServoGrippingPos);
        InR.setPosition(leftServoGrippingPos);
        InL.setPosition(rightServoGrippingPos);
    }
    public void ToUseWhenPixelGetsStuck(){
        if(IntakeMotor.getCurrent(CurrentUnit.AMPS)>0.8){
            intakeOnRev();
            sleep(500);
        }
    }


    public void intakeServoINC(){
        double leftpos=InL.getPosition()+0.01;
        double rigpos = 1 - leftpos;
        InL.setPosition(leftpos);
        InR.setPosition(rigpos);
    }
    public void intakeServoDEC(){
        double leftpos=InL.getPosition()-0.01;
        double rigpos = 1 - leftpos;
        InL.setPosition(leftpos);
        InR.setPosition(rigpos);
    }

    //TODO INTAKE MOTOR
    public void intakeMotor(double power){
        IntakeMotor.setPower(power);
    }

    public void intakeOnFor(){intakeMotor(-intakeOnFor);}
    public void intakeOnRev(){
        //intakeRevInit();
        intakeMotor(intakeREV);
    }
    public void intakeOFF(){intakeMotor(0);}

    public void intakeRevInit(){
        InL.setPosition(leftServoReverseIntakePos);
        InR.setPosition(rightServoReverseIntakePos);
    }
    public void setIntakeServo(double lefpos) {
        double rigpos = 1 - lefpos;
        InL.setPosition(lefpos);
        InR.setPosition(rigpos);
    }

    public void IntakeTwoPixel(double topPos, double bottomPos){

        //intakeState="GRIP";
            leftServoGrippingPos = topPos;
            rightServoGrippingPos = (1.0 - leftServoGrippingPos);
            InR.setPosition(leftServoGrippingPos);
            InL.setPosition(rightServoGrippingPos);
            sleep(1000);
            leftServoGrippingPos = bottomPos;
            rightServoGrippingPos = (1.0 - leftServoGrippingPos);
            InR.setPosition(leftServoGrippingPos);
            InL.setPosition(rightServoGrippingPos);
            sleep(500);


    }
//    public void IntakeTwoPixel(double Pos){
//
//        //intakeState="GRIP";
//            leftServoGrippingPos = Pos;
//            rightServoGrippingPos = (1.0 - leftServoGrippingPos);
//            InR.setPosition(leftServoGrippingPos);
//            InL.setPosition(rightServoGrippingPos);
//            sleep(500);
//            intakeInit();
//
//
//    }







}
