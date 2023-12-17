package org.firstinspires.ftc.teamcode.auto;



import static org.firstinspires.ftc.teamcode.subsytem.Intake.pickFirstPixel;
import static org.firstinspires.ftc.teamcode.subsytem.Intake.pickSecondPixel;
import static org.firstinspires.ftc.teamcode.subsytem.Outake.OuttakePixelDrop;
import static org.firstinspires.ftc.teamcode.subsytem.Outake.OuttakePixelGrip;
import static org.firstinspires.ftc.teamcode.subsytem.Outake.armGripPos;
import static org.firstinspires.ftc.teamcode.subsytem.Outake.armPlacePos;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.subsytem.Intake;
import org.firstinspires.ftc.teamcode.subsytem.Lifter;
import org.firstinspires.ftc.teamcode.subsytem.Outake;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;

@Autonomous
@Config
public class AutoV2 extends LinearOpMode {

    SampleMecanumDrive drive = null;
    Lifter lift =null;
    Intake intake=null;
    Outake outake=null;

    //drive variables
    public double THROTTLE;
    public double TURN;
    public double HEADING;

    public static double throttle = 0.8;
    public static double strafe = 0.8;
    public static double turn = 0.6;
    public static double bottomPos1 = 0.414, topPos1 = 0.385;
    public static double bottomPos2 = 0.452, topPos2 = 0.431;

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

    public static int val=0;



    @Override
    public void runOpMode() throws InterruptedException {

        drive = new SampleMecanumDrive(hardwareMap);
        lift= new Lifter(hardwareMap,telemetry);
        intake=new Intake(hardwareMap,telemetry);
        outake=new Outake(hardwareMap,telemetry);
        InitFunction();

        Pose2d startPose = new Pose2d(12,-62,Math.toRadians(0));
        drive.setPoseEstimate(startPose);

//        TrajectorySequence trajectoryseq
//  -------------------------------------- FIRST CYCLE ----------------------------------------------
//        ------------------------ Start point to Backdrop ------------------------------

        TrajectorySequence trajectoryseq = drive.trajectorySequenceBuilder(startPose)
                .addTemporalMarker(()->intake.intakeAutoStart())
                .UNSTABLE_addTemporalMarkerOffset(0.4, () -> {

                    outake.outakeArm.setPosition(armPlacePos);
                })
                .UNSTABLE_addTemporalMarkerOffset(1.45, () -> {
                    intake.IntakeMotor.setPower(0.8);

                })


                .lineToConstantHeading(new Vector2d(26 , -36))

                .splineToConstantHeading(new Vector2d(53,-40),0)
                .splineToConstantHeading(new Vector2d(56.5,-40),0)

                .UNSTABLE_addTemporalMarkerOffset(0.0000001, () ->
                {
                    outake.outakeGrip.setPosition(OuttakePixelDrop);
                    intake.IntakeMotor.setPower(0);
                })
                .waitSeconds(0.6)

                .setReversed(true)


//  ------------------ Backdrop to First Two intake -----------------------------
                .UNSTABLE_addTemporalMarkerOffset(0.4, () -> {
                    outake.outakeGrip.setPosition(OuttakePixelGrip);
                    outake.outakeArm.setPosition(armGripPos);

                })
                .splineToConstantHeading(new Vector2d(15,-8),-Math.PI)

                .splineToConstantHeading(new Vector2d(-52.8,-7.8),Math.PI)

                .UNSTABLE_addTemporalMarkerOffset(0.01, () -> {
                    intake.IntakeMotor.setPower(-1);
                    //intake.intakeStack(0.38);

                    intake.IntakeTwoPixel(topPos1, bottomPos1);

                    //.waitSeconds(0.01)
                })

                .waitSeconds(1.5)
                .setReversed(false)
                .lineToConstantHeading(new Vector2d(-49,-10))
                .UNSTABLE_addTemporalMarkerOffset(0.0001, ()->{
                    intake.intakeStack(bottomPos1 + 0.12);
                    })
                .waitSeconds(1.0)
                .UNSTABLE_addTemporalMarkerOffset(0.4, ()->{
                    intake.intakeStack(topPos1);
                })

                .splineToConstantHeading(new Vector2d(17,-10),0)
                .UNSTABLE_addTemporalMarkerOffset(0.001, () -> {

                    outake.outakeArm.setPosition(armPlacePos);
                    //sleep(1000);

                })
                .splineToConstantHeading(new Vector2d(53,-31),0)
                .splineToConstantHeading(new Vector2d(57,-31),0)

                .UNSTABLE_addTemporalMarkerOffset(0.0000001, () -> {
                    intake.IntakeMotor.setPower(0);
                    outake.outakeGrip.setPosition(OuttakePixelDrop);
                    sleep(200);
                    outake.outakeGrip.setPosition(OuttakePixelGrip);
                    sleep(200);
                    outake.outakeGrip.setPosition(OuttakePixelDrop);

                    lift.extendToLow();
                    outake.outakeGrip.setPosition(OuttakePixelDrop);
                    sleep(200);
                    outake.outakeGrip.setPosition(OuttakePixelGrip);
                    sleep(200);
                    outake.outakeGrip.setPosition(OuttakePixelDrop);
                    intake.IntakeMotor.setPower(0);



                })
                .waitSeconds(1)
                .setReversed(true)


//----------------------------------------------------------------------------------------


//  -------------------------------- SECOND CYCLE -------------------------------------
                .UNSTABLE_addTemporalMarkerOffset(0.5, () -> {
                    outake.outakeGrip.setPosition(OuttakePixelGrip);
                    outake.outakeArm.setPosition(armGripPos);
                    lift.extendToInit();

                })
                .lineToConstantHeading(new Vector2d(15,-8))

                .splineToConstantHeading(new Vector2d(-52.5,-7.8),Math.PI)

                .UNSTABLE_addTemporalMarkerOffset(0.01, () -> {
                    intake.IntakeMotor.setPower(-1);
                    //intake.intakeStack(0.38);

                    intake.IntakeTwoPixel(topPos2, bottomPos2);

                    //.waitSeconds(0.01)
                })

                .waitSeconds(1.5)
                .setReversed(false)
                .lineToConstantHeading(new Vector2d(-49,-10))
                .UNSTABLE_addTemporalMarkerOffset(0.0001, ()->{
                    intake.intakeStack(bottomPos2 + 0.12);
                })
                .waitSeconds(1.0)
                .UNSTABLE_addTemporalMarkerOffset(0.4, ()->{
                    intake.intakeStack(topPos1);
                })

                .lineToConstantHeading(new Vector2d(17,-10))
                .UNSTABLE_addTemporalMarkerOffset(0.001, () -> {

                    outake.outakeArm.setPosition(armPlacePos);
                    //sleep(1000);

                })
                .splineToConstantHeading(new Vector2d(53,-31), 0)
                .splineToConstantHeading(new Vector2d(57,-31),0)

                .UNSTABLE_addTemporalMarkerOffset(0.0000001, () -> {
                    intake.IntakeMotor.setPower(0);
                    outake.outakeGrip.setPosition(OuttakePixelDrop);
                    sleep(200);
                    outake.outakeGrip.setPosition(OuttakePixelGrip);
                    sleep(200);
                    outake.outakeGrip.setPosition(OuttakePixelDrop);

                    lift.extendToLow();
                    outake.outakeGrip.setPosition(OuttakePixelDrop);
                    sleep(200);
                    outake.outakeGrip.setPosition(OuttakePixelGrip);
                    sleep(200);
                    outake.outakeGrip.setPosition(OuttakePixelDrop);
                    intake.IntakeMotor.setPower(0);



                })
                 .waitSeconds(5)
                .setReversed(true)
//------------------------------------------------------------------------------------------------
                .build();

        //TODO SETUP -INIT
        waitForStart();
        //    if (!isStopRequested()) return;
        drive.followTrajectorySequence(trajectoryseq);
        intake.ToUseWhenPixelGetsStuck();

    }
    public void InitFunction(){

        lift.leftElevator.setPower(-0.2);
        lift.rightElevator.setPower(-0.2);
        if(lift.leftElevator.getCurrent(CurrentUnit.AMPS)>=1 ) {
            lift.extendTo(lift.leftElevator.getCurrentPosition(), 0.2);
            val = lift.leftElevator.getCurrentPosition();
        }


        intake.intakeInit();
        outake.outakeGrip.setPosition(0.51);
        outake.outakeArmInit();
        //  hanger_servo.setPosition(0.5);
        //. plane.setPosition(0.5);


    }
}
