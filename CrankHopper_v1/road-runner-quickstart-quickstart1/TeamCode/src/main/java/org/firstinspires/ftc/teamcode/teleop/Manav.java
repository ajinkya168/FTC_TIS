package org.firstinspires.ftc.teamcode.teleop;




import static java.lang.Math.abs;
import static java.lang.Math.atan2;
import static java.lang.Math.cos;
import static java.lang.Math.hypot;
import static java.lang.Math.max;
import static java.lang.Math.sin;
import static java.lang.Math.toRadians;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;

import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.Servo;
import org.firstinspires.ftc.teamcode.util.LynxModuleUtil;
import com.qualcomm.robotcore.hardware.VoltageSensor;


import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;

@Config
@TeleOp

public class Manav extends LinearOpMode {
    public static double Pos2 = 0.1;
    public static double throttle = 0.8;
    public static double strafe = 0.8;
    public static double turn = 0.6;
    public static double zeroPos = 0.2; //0.5;
    public static double zeroPos1 = 0.8; //0.5;
    public static double Pos = 0.55;
    public static double Pos1 = 0.45;
    public static double hangerpos = 0.5;

    String gripper_condition = "";
    /* Declare OpMode members. */

    DcMotorEx In;
    DcMotorEx ArmL;
    DcMotorEx ArmR;
    Servo plane;

    public static int ArmL_high_pos = -750 ;
    public static int ArmR_high_pos  = 750;
    public static int ArmL_med_pos = -600 ;
    public static int ArmR_med_pos = 600 ;
    public static int ArmL_low_pos = -375;
    public static int ArmR_low_pos = 375;
    public static int ArmL_init = 198;
    public static int ArmR_init = -203;
    public static double motorPower = 0.5;
    public static int hanger_pos_lift = -11500;
    public static int hanger_pos_down = -300;



    //public DcMotor lifterL  = null;
    //public DcMotor  lifterR  = null;
    //public DcMotor  Arm  = null;9
    Servo InR;
    Servo InL;
    Servo out;
    DcMotorEx leftFront;
    DcMotorEx rightFront;
    DcMotorEx rightRear;
    DcMotorEx leftRear;
    DcMotorEx hangerMotor;
    Servo out_arm;
    Servo hanger_servo;
    public static int In_pos= 0;
    @Override
    public void runOpMode() throws InterruptedException {
        InR = hardwareMap.get(Servo.class, "InR");
        InL = hardwareMap.get(Servo.class, "InL");
        out = hardwareMap.get(Servo.class, "out");
        out_arm = hardwareMap.get(Servo.class, "outarm");
        hanger_servo = hardwareMap.get(Servo.class, "hang");
        plane = hardwareMap.get(Servo.class, "plane");

        //Lifters
        ArmL = hardwareMap.get(DcMotorEx.class, "ArmL");
        ArmR = hardwareMap.get(DcMotorEx.class, "ArmR");
        //Intake
        In = hardwareMap.get(DcMotorEx.class, "In");

        hangerMotor = hardwareMap.get(DcMotorEx.class, "hangerMotor");

        leftFront = hardwareMap.get(DcMotorEx.class, "leftFront");
        leftRear = hardwareMap.get(DcMotorEx.class, "leftRear");
        rightRear = hardwareMap.get(DcMotorEx.class, "rightRear");
        rightFront = hardwareMap.get(DcMotorEx.class, "rightFront");


        //ArmL.setDirection(DcMotorEx.Direction.REVERSE);
        //ArmR.setDirection(DcMotorEx.Direction.FORWARD);
        hangerMotor.setDirection(DcMotor.Direction.REVERSE);

        ArmL.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        ArmR.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
//        In.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        hangerMotor.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);

        ArmL.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        ArmR.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        In.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        hangerMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        ArmL.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        ArmR.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        In.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        hangerMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        // Send telemetry message to signify robot waiting;
        telemetry.addData(">", "Robot Ready.  Press Play.");


        telemetry.addData("ArmL Current (A)", ArmL.getCurrent(CurrentUnit.AMPS));
        telemetry.addData("ArmR Current (A)", ArmR.getCurrent(CurrentUnit.AMPS));
        telemetry.addData("aRM l", ArmL.getCurrentPosition());
        telemetry.addData("aRM R", ArmR.getCurrentPosition());
        telemetry.update();
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        // lifter_init();.0
        hanger_servo.setPosition(0.35);



        waitForStart();
        while (opModeIsActive()) {

            Pose2d poseEstimate = drive.getPoseEstimate();
            Vector2d input = new Vector2d(-gamepad1.left_stick_y * throttle, -gamepad1.left_stick_x * strafe);//.rotated(-poseEstimate.getHeading());


            drive.setWeightedDrivePower(
                    new Pose2d(input.getX(), input.getY(), -gamepad1.right_stick_x * turn
                    ));
            drive.update();



            if (gamepad1.start) {
                In.setPower(-1);
            }
            if (gamepad1.back) {
                In.setPower(0);
            }
//            if (gamepad1.a)
//            {
//                In.setPower(1);
//            }

            //init/up
            if (gamepad1.left_trigger > 0) {
                In.setPower(0);
                InR.setPosition(zeroPos);
                InL.setPosition(zeroPos1);

//                    out_arm.setPosition(0);
            }

            //pick
            if (gamepad1.right_trigger > 0) {
                InR.setPosition(Pos);
                InL.setPosition(Pos1);
            }

            if (gamepad1.y) {
                out.setPosition(0.64); //close
            }
            if (gamepad1.a) {
                out.setPosition(0.46); //open
            }
            if (gamepad1.x) {
                out_arm.setPosition(0.87); //back
            }
            if (gamepad1.b) {
                out_arm.setPosition(0.35);
            }

            if (gamepad1.dpad_up)
            {
                // lifter_high();
                plane.setPosition(Pos2);
            }
            if (gamepad1.dpad_right)
            {
                lifter_med();
                plane.setPosition(0.53);
            }
//
            if (gamepad1.dpad_down)
            {
                //lifter_low();
//                hanger_servo.setPosition(0.35);
                hanger_servo.setPosition(hangerpos);
            }

//            if (gamepad1.dpad_right) {
//                hanger_servo.setPosition(0.35);
//            }
            if (gamepad1.dpad_left) {
                lifter_init();
                //hanger_servo.setPosition(hangerpos);
            }
//                if (gamepad1.dpad_down)
//                {
//                    lifter_init();
//                }

            if (gamepad1.right_bumper) {
                hanger_lift();
            }
            if (gamepad1.left_bumper) {

                hanger_down(); //check rachet
            }
//            if(gamepad1.right_trigger>0){
//                hangerMotor.setTargetPosition(hangerMotor.getCurrentPosition() - 100);
//                hangerMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//                hangerMotor.setPower(0.6);
//
//            }
//            if(gamepad1.left_trigger>0){
//
//                hangerMotor.setTargetPosition(hangerMotor.getCurrentPosition() + 100);
//                hangerMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//                hangerMotor.setPower(0.6);
//
//            }

            // double ArmLCurrent = ArmL.getCurrent(CurrentUnit.AMPS);
            //double ArmRCurrent = ArmR.getCurrent(CurrentUnit.AMPS);
            double InCurrent = In.getCurrent(CurrentUnit.AMPS);

//
//                telemetry.addData("ArmL Current (A)", ArmLCurrent);
//                telemetry.addData("ArmR Current (A)", ArmRCurrent);
            telemetry.addData("ArmL Current (A)", ArmL.getCurrent(CurrentUnit.AMPS));
            telemetry.addData("ArmR Current (A)", ArmR.getCurrent(CurrentUnit.AMPS));
            telemetry.addData("Intake Current (A)", InCurrent);
//            // Output the current readings in telemetry

            telemetry.addData("left front", leftFront.getCurrent(CurrentUnit.AMPS));
            telemetry.addData("right front", rightFront.getCurrent(CurrentUnit.AMPS));
            telemetry.addData("left back", leftRear.getCurrent(CurrentUnit.AMPS));
            telemetry.addData("right back", rightRear.getCurrent(CurrentUnit.AMPS));

            telemetry.addData("leftF", leftFront.getCurrentPosition());
            telemetry.addData("leftR", leftRear.getCurrentPosition());
            telemetry.addData("rightF", rightFront.getCurrentPosition());
            telemetry.addData("rightR", rightRear.getCurrentPosition());
            telemetry.addData("leftFVel", leftFront.getVelocity());
            telemetry.addData("leftRVel", leftRear.getVelocity());
            telemetry.addData("rightFVel", rightFront.getVelocity());
            telemetry.addData("rightRVel", rightRear.getVelocity());
            telemetry.addData("aRM l", ArmL.getCurrentPosition());
            telemetry.addData("aRM R", ArmR.getCurrentPosition());
            telemetry.addData("hanger current", hangerMotor.getCurrent(CurrentUnit.AMPS));
            telemetry.addData("hang", hangerMotor.getCurrentPosition());
            // telemetry.addData("hang",hangerMotor.analogInput.getVoltage() );
            //double currentVoltage = getVoltage();

            telemetry.update();
        }
    }

    public void lifter_high()
    {
        ArmL.setTargetPosition(ArmL_high_pos);
        ArmL.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
        ArmL.setPower(motorPower);
        ArmR.setTargetPosition(ArmR_high_pos);
        ArmR.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
        ArmR.setPower(motorPower);

    }
    public void lifter_med()
    {
        ArmL.setTargetPosition(ArmL_med_pos);
        ArmL.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
        ArmL.setPower(motorPower);
        ArmR.setTargetPosition(ArmR_med_pos);
        ArmR.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
        ArmR.setPower(motorPower);
    }
    public void lifter_low()
    {
        ArmL.setTargetPosition(ArmL_low_pos);
        ArmL.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
        ArmL.setPower(motorPower);
        ArmR.setTargetPosition(ArmR_low_pos);
        ArmR.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
        ArmR.setPower(motorPower);

    }
    public void lifter_init()
    {
        ArmL.setTargetPosition(ArmL_init);
        ArmL.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
        ArmL.setPower(motorPower);
        ArmR.setTargetPosition(ArmR_init);
        ArmR.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
        ArmR.setPower(motorPower);

    }


    public void hanger_lift(){
        //hangerMotor.setTargetPosition(hangerMotor.getCurrentPosition() + 100);
        hangerMotor.setTargetPosition((hanger_pos_lift));
        hangerMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        hangerMotor.setPower(0.6);

    }
    public void hanger_down(){
        //hangerMotor.setTargetPosition(hangerMotor.getCurrentPosition() - 100);
        hangerMotor.setTargetPosition((hanger_pos_down));
        hangerMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        hangerMotor.setPower(0.4);
    }
//    public void extendArm(int targetPosition,double power){
//        ArmL.setTargetPosition(targetPosition);
//        ArmL.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
//        ArmL.setPower(power);
//        ArmR.setTargetPosition(targetPosition);
//        ArmR.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
//        ArmR.setPower(power);
//
//    }

}