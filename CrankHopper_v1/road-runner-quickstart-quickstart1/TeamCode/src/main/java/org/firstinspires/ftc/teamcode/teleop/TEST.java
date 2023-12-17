package org.firstinspires.ftc.teamcode.teleop;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.control.PIDFController;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.subsytem.HangerAndDrone;
import org.firstinspires.ftc.teamcode.subsytem.Intake;
import org.firstinspires.ftc.teamcode.subsytem.Lifter;
import org.firstinspires.ftc.teamcode.subsytem.Outake;

@Config
@TeleOp(name="KCenterstage")
public class TEST extends LinearOpMode {


    SampleMecanumDrive drive = null;
    Lifter lift =null;
    Intake intake=null;
    Outake outake=null;
    HangerAndDrone endgame = null;
   // Servo s1,s2,s3,s4;


    //drive variables
    public double THROTTLE;
    public double TURN;
    public double HEADING;

    public static double throttle = 0.8;
    public static double strafe = 0.8;
    public static double turn = 0.6;


    //

    Servo hanger_servo,plane;
    //
    public static double Kp = 0.01;
    public static double Ki = 0;
    public static double Kd = 0;
    public static double Kf = 0;
    ElapsedTime lifTimer;

    private PIDController elevatorcontroller;
    public static int elevatortarget = 0;

    public  static int rightLifter=0;

    public static int value,variablevalue = 0;
    public  static double elevatorff=0;

    public boolean booleanflag = true;

    public static int val;
    boolean gmup = false;//for Dropping mech
    boolean gmpx = false;//for Dropping mech
    boolean gmpy = false;//for Dropping mech
    boolean gmrb = false;//for Dropping an dropping pixel
    boolean gmpb = false;//for  Intake to start
    boolean gmlb = false;//for  Intake to Reverse
    boolean movelift1 = false;//for medium
    boolean movelift2 = false;//for low
    boolean movelift3 = false;//for high
    boolean movegrip = false;//for gripping ans dropping
    boolean moveIntake = false; //for intake on and off











    PIDFController lifterPID;
    @Override
    public void runOpMode() throws InterruptedException {
        drive = new SampleMecanumDrive(hardwareMap);
        lift= new Lifter(hardwareMap,telemetry);
        intake=new Intake(hardwareMap,telemetry);
        outake=new Outake(hardwareMap,telemetry);
        endgame = new HangerAndDrone(hardwareMap, telemetry);
//s1=hardwareMap.get(Servo.class,"s1");
//        s2=hardwareMap.get(Servo.class,"s2");
//        s3=hardwareMap.get(Servo.class,"s3");
//        s4=hardwareMap.get(Servo.class,"s4");
//        s1.setPosition(0.5);
//        s2.setPosition(0.5);
//        s3.setPosition(0.5);
//        s4.setPosition(0.5);

        //
         hanger_servo = hardwareMap.get(Servo.class, "hang");
        plane = hardwareMap.get(Servo.class, "plane");

        //LIFT PID SETUP
        elevatorcontroller = new PIDController(Kp,Ki,Kd);
        telemetry= new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());





        intake.intakeInit();
        outake.outakeArmInit();
//        lift.extendToHomePos();


        while (opModeInInit()){

//            variablevalue = lift.leftElevator.getCurrentPosition();
            InitFunction();
            endgame.DroneInit();
            endgame.HangerInit();
            telemetry.addData("Lifter Position: ", "left:"+lift.getPosition()[0] +", "+"right:"+ lift.getPosition()[1]);
            telemetry.addData("Lifter Current: ","left:"+lift.getCurrent()[0]+", "+"right:"+lift.getCurrent()[1]);
            telemetry.addData("variable value",variablevalue);
            telemetry.update();
        }

        waitForStart();
        while (opModeIsActive()) {
//            booleanflag = false;

            elevatorcontroller.setPID(Kp,Ki,Kd);
            int elevatorFinalPos=lift.leftElevator.getCurrentPosition();
            double elevatorpower = Range.clip(((elevatorcontroller.calculate(elevatorFinalPos, elevatortarget)+ elevatorff)) , -1   , 1);

            //drive begin

//            Pose2d poseEstimate = drive.getPoseEstimate();
//            Vector2d input = new Vector2d(Math.pow(Range.clip(gamepad1.left_stick_y, -1, 1), 3), Math.pow(Range.clip(gamepad1.left_stick_x, -1, 1), 3)).rotated(-poseEstimate.getHeading());
//
//            drive.setWeightedDrivePower(
//                    new Pose2d(input.getX() * THROTTLE, input.getY() * TURN, -gamepad1.right_stick_x * HEADING));
//
////            drive.update();
//            Pose2d poseEstimate = drive.getPoseEstimate();
//            Vector2d input = new Vector2d(-gamepad1.left_stick_y * throttle, -gamepad1.left_stick_x * strafe);//.rotated(-poseEstimate.getHeading());
//
//
//            drive.setWeightedDrivePower(
//                    new Pose2d(input.getX(), input.getY(), -gamepad1.right_stick_x * turn
//                    ));
//            drive.update();
//
//            telemetry.addData("heading", poseEstimate.getHeading());

            //drive end
            drive.setWeightedDrivePower(
                    new Pose2d(
                            -gamepad1.left_stick_y * 0.95,
                            -gamepad1.left_stick_x * 0.95,
                            -gamepad1.right_stick_x * 0.9
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
            boolean R3 = gamepad1.right_stick_button;
            boolean L3 = gamepad1.left_stick_button;
            boolean TL = gamepad1.left_trigger>0.8;
            boolean TR = gamepad1.right_trigger>0.8;



            //TODO INTAKE
            //TODO: Intake Mech
            if (!gmlb && UP) {
                gmlb = true;
                if (moveIntake == true) {
                    IntakeMechOff();
                    outake.closeGripper();
                    moveIntake = false;
                } else {
                    IntakeMechReverse();
                    moveIntake = true;
                }
            }
            if (!UP) {
                gmlb = false;
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

            if (UP){
                IntakeMechReverse();
            }
            //TODO ARM INCREMENT
//            if(gamepad2.dpad_up){
//                outake.outakeArm.setPosition(outake.outakeArm.getPosition() + 0.01);
//            }
//            if(gamepad2.dpad_down){
//                outake.outakeArm.setPosition(outake.outakeArm.getPosition() - 0.01);
//            }

            //TODO OUTAKE
            //TODO: Toggle for Dropping Mechanisms
            /*For Medium Position*/

            if (!gmup && LB) {
                gmup = true;
                if (movelift1 == true) {
                    DropMechInactive(0.7);
                    movelift1 = false;
                } else {
                    DropMechActiveMid();
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
                    movelift2 = false;
                } else {
                    DropMechActiveLow();
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
                    movelift3 = false;
                } else {
                    DropMechActiveHigh();
                    movelift3 = true;
                }
            }

            if (!gamepad1.y) {
                gmpy = false;
            }

            //TODO: Outake Grip Mech

            if (!gmrb && RB) {
                gmrb = true;
                if (movegrip == true) {
                    outake.dropPixel2();
                    movegrip = false;
                } else {
                    outake.dropPixel1();
                    movegrip = true;
                }
            }

            if (!RB) {
                gmrb = false;
            }
            ////
//            if (TR) {
//                outake.closeGripper();
//            } else if (TL) {
//                outake.openGripper();
//            }
            if (TR){
                PixelDrop();
            }
            if (TL){
                PixelDrop2();
            }
//            if (gamepad2.x) {
//                outake.dropPixel1();
//            } else if (gamepad2.y) {
//                outake.dropPixel2();
//            }
//
//            }
            //TODO: Hanger and drone
            if(gamepad2.left_bumper){
               endgame.ShootDrone();

            }
            if(gamepad2.right_bumper){
                endgame.HangerOn();
            }

            if (gamepad2.dpad_up){
                endgame.HangerUp();
            }

            if(gamepad2.dpad_down){
                endgame.HangerDown();
            }

//
//            if (LEFT){
//                hanger_servo.setPosition(hanger_servo.getPosition()+0.01);
//            }
//            if (RIGHT){
//                plane.setPosition(plane.getPosition()+0.01);
//            }



            telemetry.addData("Intake Left Servo",intake.InL.getPosition());
            telemetry.addData("Intake Right Servo",intake.InR.getPosition());

            telemetry.addData("Intake Motor Current",intake.IntakeMotor.getCurrent(CurrentUnit.AMPS));


            //
            telemetry.addData("Outake Arm",outake.outakeArm.getPosition());
            telemetry.addData("Outake Grip",outake.outakeGrip.getPosition());

            //
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
        hanger_servo.setPosition(0.5);
        plane.setPosition(0.5);
        outake.closeGripper();
    }
    public void IntakeMechOn(){
        intake.intakeOnFor();
        intake.intakeGrip();

    }
    public void IntakeMechReverse(){
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
        outake.outakeArmPlace();
    }
    public void DropMechActiveHigh(){
        lift.extendToHigh();
        outake.outakeArmPlace();
    }
    public void DropMechInactive(double elevatorpower){
        lift.extendTo(val,elevatorpower);
        outake.outakeArmGripPos();
        outake.closeGripper();
    }
    public void PixelDrop(){
        //0.369 drop
        //0.6494 grip
        outake.outakeGrip.setPosition(0.369);
        sleep(100);
        outake.outakeGrip.setPosition(0.369);
        sleep(200);
        outake.outakeGrip.setPosition(0.57);//0.6494
    }
    public void PixelDrop2(){
        outake.outakeGrip.setPosition(0.6494);
    }

}
