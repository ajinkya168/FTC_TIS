package org.firstinspires.ftc.teamcode.teleop;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.subsytem.HangerAndDrone;
import org.firstinspires.ftc.teamcode.subsytem.Intake;
import org.firstinspires.ftc.teamcode.subsytem.Lifter;
import org.firstinspires.ftc.teamcode.subsytem.Outake;

@TeleOp
//@Config
public class CrankHopper extends LinearOpMode {

    SampleMecanumDrive drive = null;
    Lifter lift =null;
    Intake intake=null;
    Outake outake=null;
    HangerAndDrone endgame = null;

    public static double turn = 0.6;


    public static double Yvalue = 1;
    public static double Xvalue = 1;
    public static double TurnX  = 1;
    //

    Servo hanger_servo,plane;
    //
    public static double Kp = 0.01;
    public static double Ki = 0;
    public static double Kd = 0;

    private PIDController elevatorcontroller;
    public static int elevatortarget = 0;

    public static int value,variablevalue = 0;
    public  static double elevatorff=0;

    public static int val;
    boolean gmup = false;//for Dropping mech
    boolean gmpx = false;//for Dropping mech
    boolean gmpy = false;//for Dropping mech
    boolean gmpb = false;//for  Intake to start
    boolean movelift1 = false;//for medium
    boolean movelift2 = false;//for low
    boolean movelift3 = false;//for high
    boolean moveIntake = false; //for intake on and off

    boolean gmp1a = false;

    public void runOpMode() throws InterruptedException {
        drive = new SampleMecanumDrive(hardwareMap);
        lift= new Lifter(hardwareMap,telemetry);
        intake=new Intake(hardwareMap,telemetry);
        outake=new Outake(hardwareMap,telemetry);
        endgame = new HangerAndDrone(hardwareMap, telemetry);
        hanger_servo = hardwareMap.get(Servo.class, "hang");
        plane = hardwareMap.get(Servo.class, "plane");

        //LIFT PID SETUP
        elevatorcontroller = new PIDController(Kp,Ki,Kd);
        telemetry= new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());


        while (opModeInInit()){

            InitFunction();

            telemetry.addData("Lifter Position: ", "left:"+lift.getPosition()[0] +", "+"right:"+ lift.getPosition()[1]);
            telemetry.addData("Lifter Current: ","left:"+lift.getCurrent()[0]+", "+"right:"+lift.getCurrent()[1]);
            telemetry.addData("variable value",variablevalue);
            telemetry.update();
        }

        waitForStart();
        while (opModeIsActive()) {
            elevatorcontroller.setPID(Kp,Ki,Kd);
            int elevatorFinalPos=lift.leftElevator.getCurrentPosition();
            double elevatorpower = Range.clip(((elevatorcontroller.calculate(elevatorFinalPos, elevatortarget)+ elevatorff)) , -1   , 1);



            //drive end
            drive.setWeightedDrivePower(
                    new Pose2d(
                            -gamepad1.left_stick_y * Yvalue, //Yvalue
                            -gamepad1.left_stick_x * Xvalue, //Xvalue
                            -gamepad1.right_stick_x * TurnX //TurnX
                    )
            );
            drive.update();

//
//            //TODO CONTROLS
            boolean UP = gamepad1.dpad_up;
            boolean RIGHT = gamepad1.dpad_right;
            boolean DOWN = gamepad1.dpad_down;
            boolean LEFT = gamepad1.dpad_left;
            boolean RB = gamepad1.right_bumper;
            boolean LB = gamepad1.left_bumper;
//            boolean RS = gamepad1.right_stick_button;
//            boolean LS = gamepad1.left_stick_button;
            boolean LT = gamepad1.left_trigger>0.8;
            boolean RT = gamepad1.right_trigger>0.8;




            //TODO INTAKE
            //TODO: Intake Mech
            if (!gmp1a && gamepad1.a) {
                gmp1a = true;
                if (moveIntake == true) {
                    IntakeMechOff();
                    outake.closeGripper();
                    moveIntake = false;
                } else {

                    IntakeMechReverse();
                    moveIntake = true;
                }
            }
            if (!gamepad1.a) {
                gmp1a = false;
            }

            if (!gmpb && gamepad1.b) {
                gmpb = true;
                if (moveIntake == true) {
                    IntakeMechOff();
                    outake.closeGripper();
                    moveIntake = false;
                } else {
                    IntakeMechOn();
                    moveIntake = true;
                }
            }
            if (!gamepad1.b) {
                gmpb = false;
            }

//            if (UP){
//                IntakeMechReverse();
//            }

            //TODO OUTAKE
            //TODO: Toggle for Dropping Mechanisms
            /*For Medium Position*/

            if (!gmup && LB) {
                gmup = true;
                if (movelift1 == true) {
                    DropMechInactive(0.7);

                    Yvalue = 0.95;
                    Xvalue = 0.95;
                    TurnX = 0.9;
                    movelift1 = false;
                } else {
                    DropMechActiveMid();
                    Yvalue = 0.5;
                    Xvalue = 0.5;
                    TurnX = 0.5;

                    movelift1 = true;
                }
            }

            if (!LB) {
                gmup = false;
            }

            /*For Low Position*/

            if (!gmpx && gamepad1.x) {
                gmpx = true;
                if (movelift2 == true) {
                    DropMechInactive(0.7);
//                    DropMechActiveLow();
//                    outake.outakeArmInit();


                    Yvalue = 0.95;
                    Xvalue = 0.95;
                    TurnX = 0.9;
                    movelift2 = false;
                } else {
                    DropMechActiveLow();
//                    lift.extendToLow();
//                    outake.outakeArmPlace();
                    Yvalue = 0.5;
                    Xvalue = 0.5;
                    TurnX = 0.5;

                    movelift2 = true;
                }
            }

            if (!gamepad1.x) {
                gmpx = false;
            }


            /*For High Position*/

            if (!gmpy && gamepad1.y) {
                gmpy = true;
                if (movelift3 == true) {
                    DropMechInactive(0.7);
                   // lift.extendToInit();
                    Yvalue = 0.95;
                    Xvalue = 0.95;
                    TurnX = 0.9;
                    movelift3 = false;
                } else {
                    DropMechActiveHigh();
                    Yvalue = 0.5;
                    Xvalue = 0.5;
                    TurnX = 0.5;
                    movelift3 = true;
                }
            }

            if (!gamepad1.y) {
                gmpy = false;
            }

            //TODO: Outake Grip Mech


            if (RT) {
                outake.PixelDrop();
            }

            //TODO: Hanger and drone
            if(LT){
                endgame.HangerInc(endgame.HangerMotor.getCurrentPosition());
            }
            if(gamepad1.dpad_left){

                endgame.ShootDrone();

            }

            if(gamepad1.dpad_right){
                endgame.HangerOn();

            }
            if (gamepad1.dpad_up){
                endgame.HangerUp();
            }


            if(gamepad1.dpad_down){
                endgame.HangerDown();
            }

            if(gamepad1.back){
                outake.gripclose();
            }

            telemetry.addData("Intake Left Servo",intake.InL.getPosition());
            telemetry.addData("Intake Right Servo",intake.InR.getPosition());
            telemetry.addData("Intake Motor Current",intake.IntakeMotor.getCurrent(CurrentUnit.AMPS));
            telemetry.addData("Outake Arm",outake.outakeArm.getPosition());
            telemetry.addData("Outake Grip",outake.outakeGrip.getPosition());


            //TODO LIFTER TELEMETRY
            telemetry.addData("Lifter Position: ", "left:"+lift.getPosition()[0] +", "+"right:"+ lift.getPosition()[1]);
            telemetry.addData("Lifter Current: ","left:"+lift.getCurrent()[0]+", "+"right:"+lift.getCurrent()[1]);
            telemetry.addData("leftFront current:", drive.leftFront.getCurrent(CurrentUnit.AMPS));
            telemetry.addData("rightFront current:", drive.rightFront.getCurrent(CurrentUnit.AMPS));
            telemetry.addData("leftRear current:", drive.leftRear.getCurrent(CurrentUnit.AMPS));
            telemetry.addData("rightReear current:", drive.rightRear.getCurrent(CurrentUnit.AMPS));
            telemetry.update();
        }

    }
    public void InitFunction(){

        lift.leftElevator.setPower(-0.2);
        lift.rightElevator.setPower(-0.2);
        if(lift.leftElevator.getCurrent(CurrentUnit.AMPS)>=1 ){
            lift.extendTo(lift.leftElevator.getCurrentPosition(), 0.2);
            val = lift.leftElevator.getCurrentPosition();
        }

        intake.intakeInit();
        outake.outakeArmInit();
        endgame.DroneInit();
        endgame.HangerInit();
        outake.closeGripper();
        outake.outakepushi();
        outake.gripclose();
    }
    public void IntakeMechOn(){
        intake.intakeOnFor();
        intake.intakeGrip();

    }
    public void IntakeMechReverse(){
        intake.intakeRevInit();
        intake.intakeOnRev();

    }
    public void IntakeMechOff(){
        intake.intakeOFF();
        intake.intakeInit();
    }

    public void DropMechActiveMid(){
        lift.extendToLow();
        outake.outakeArmPlace();
    }
    public void DropMechActiveLow(){
        lift.extendToMedium();
       // lift.extendTolow_mid();
        outake.outakeArmPlace();
    }
    public void DropMechActiveHigh(){
        lift.extendToHigh();
        outake.outakeArmPlace();
    }
    public  void DropMechInactive(double elevatorpower){
        lift.extendTo(val,elevatorpower);
      //  Outake.outakeArm.setPosition(0.7);
        //sleep(500);
        //Outake.outakeArm.setPosition(0.8);
        outake.outakeArmGripPos();
        outake.closeGripper();
    }

    public void newout() {

        outake.gripopen();
        outake.outakepusho();
        outake.gripclose();
        outake.outakepushi();
        outake.gripopen();
        outake.outakepusho();

    }

}
