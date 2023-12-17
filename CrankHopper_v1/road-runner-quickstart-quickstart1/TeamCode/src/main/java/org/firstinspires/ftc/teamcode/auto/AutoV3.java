package org.firstinspires.ftc.teamcode.auto;



import static org.firstinspires.ftc.teamcode.subsytem.Intake.leftServoGrippingPos;
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
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.drive.DriveConstants;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.subsytem.HangerAndDrone;
import org.firstinspires.ftc.teamcode.subsytem.Intake;
import org.firstinspires.ftc.teamcode.subsytem.Lifter;
import org.firstinspires.ftc.teamcode.subsytem.Outake;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequenceBuilder;

@Autonomous
@Config
public class AutoV3 extends LinearOpMode {

    SampleMecanumDrive drive = null;
    Lifter lift =null;
    Intake intake=null;
    Outake outake=null;
    HangerAndDrone endgame = null;

    public static double turn = 0.6;
    public static double bottomPos1 = 0.416, topPos1 = 0.385;
    public static double bottomPos2 = 0.454, topPos2 = 0.431;

    DistanceSensor dsFront = null, dsBack = null;
    //

    public static int value = 0;
    public static int val=0;



    @Override
    public void runOpMode() throws InterruptedException {

        drive = new SampleMecanumDrive(hardwareMap);
        lift= new Lifter(hardwareMap,telemetry);
        intake=new Intake(hardwareMap,telemetry);
        outake=new Outake(hardwareMap,telemetry);
        endgame = new HangerAndDrone(hardwareMap, telemetry);
        dsFront = hardwareMap.get(DistanceSensor.class, "dsFront");
        dsBack = hardwareMap.get(DistanceSensor.class, "dsBack");
        InitFunction();

        Pose2d startPose = new Pose2d(12,-62,Math.toRadians(0));
        drive.setPoseEstimate(startPose);

//        TrajectorySequence trajectoryseq
//  -------------------------------------- FIRST CYCLE ----------------------------------------------
//        ------------------------ Start point to Backdrop ------------------------------

        TrajectorySequence trajectoryseq = drive.trajectorySequenceBuilder(startPose)
                .addTemporalMarker(()->{intake.intakeAutoStart();
                    InitFunction();
                })
                .UNSTABLE_addTemporalMarkerOffset(0.4, () -> {
                    lift.extendToLow();
                    outake.outakeArm.setPosition(armPlacePos);
                })
                .UNSTABLE_addTemporalMarkerOffset(1.45, () -> {
                    intake.IntakeMotor.setPower(0.5);

                })


                .lineToConstantHeading(new Vector2d(26 , -36))
                .splineToConstantHeading(new Vector2d(53,-40),0)
                .splineTo(new Vector2d(57.3, -40), 0,   SampleMecanumDrive.getVelocityConstraint(25, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL/2))
                //.splineToConstantHeading(new Vector2d(56.5,-40),0)
                .UNSTABLE_addTemporalMarkerOffset(0.0000001, () ->
                {
                    outake.outakeGrip.setPosition(OuttakePixelDrop);

                    intake.IntakeMotor.setPower(0);

                })
                .waitSeconds(0.6)

                .setReversed(true)


////  ------------------ Backdrop to First Two intake -----------------------------
                .UNSTABLE_addTemporalMarkerOffset(0.4, () -> {
                    outake.outakeGrip.setPosition(OuttakePixelGrip);
                    outake.outakeArm.setPosition(armGripPos);
                    lift.extendToInit();


                })
                .splineToConstantHeading(new Vector2d(15,-8),-Math.PI)

                .splineToConstantHeading(new Vector2d(-52.5,-8.5),Math.PI)

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
                    intake.intakeStack(leftServoGrippingPos);
                })
                .waitSeconds(1.0)
                .UNSTABLE_addTemporalMarkerOffset(0.4, ()->{
                    intake.intakeStack(topPos1);
                })

                .splineToConstantHeading(new Vector2d(17,-10), 0)
                .UNSTABLE_addTemporalMarkerOffset(0.000001, () -> {

                    outake.outakeArm.setPosition(armPlacePos);
                    lift.extendToLow();
                    //sleep(1000);

                })
                .splineToConstantHeading(new Vector2d(53,-31),0)
                .splineTo(new Vector2d(57.3,-31),0, SampleMecanumDrive.getVelocityConstraint(25, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL/2))



                .UNSTABLE_addTemporalMarkerOffset(0.0000001, () -> {
                    intake.IntakeMotor.setPower(0);
                    outake.outakeGrip.setPosition(OuttakePixelDrop);
                    sleep(200);
                    outake.outakeGrip.setPosition(OuttakePixelGrip);
                    sleep(200);
                    outake.outakeGrip.setPosition(OuttakePixelDrop);

                    lift.extendToMedium();

                    outake.outakeGrip.setPosition(OuttakePixelDrop);
                    sleep(200);
                    outake.outakeGrip.setPosition(OuttakePixelGrip);
                    sleep(200);
                    outake.outakeGrip.setPosition(OuttakePixelDrop);
                    intake.IntakeMotor.setPower(0);



                })
                .waitSeconds(1)
                .setReversed(true)


////----------------------------------------------------------------------------------------

////  -------------------------------- SECOND CYCLE -------------------------------------
                .UNSTABLE_addTemporalMarkerOffset(0.5, () -> {
                    outake.outakeGrip.setPosition(OuttakePixelGrip);
                    outake.outakeArm.setPosition(armGripPos);
                    lift.extendToInit();

                })
                .lineToConstantHeading(new Vector2d(15,-8))

                .splineToConstantHeading(new Vector2d(-52.5,-8.5),Math.PI)

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
                    intake.intakeStack(leftServoGrippingPos);
                })
                .waitSeconds(1.0)
                .UNSTABLE_addTemporalMarkerOffset(0.4, ()->{
                    intake.intakeStack(topPos1);
                })

                .lineToConstantHeading(new Vector2d(17,-10))
                .UNSTABLE_addTemporalMarkerOffset(0.001, () -> {
                    lift.extendToLow();
                    outake.outakeArm.setPosition(armPlacePos);
                    //sleep(1000);

                })
                .splineToConstantHeading(new Vector2d(53,-31), 0)
                .splineTo(new Vector2d(57.5,-31),0, SampleMecanumDrive.getVelocityConstraint(25, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL/2))

                .UNSTABLE_addTemporalMarkerOffset(0.0000001, () -> {
                    intake.IntakeMotor.setPower(0);
                    outake.outakeGrip.setPosition(OuttakePixelDrop);
                    sleep(200);
                    outake.outakeGrip.setPosition(OuttakePixelGrip);
                    sleep(200);
                    outake.outakeGrip.setPosition(OuttakePixelDrop);

                    lift.extendToMedium();
                    outake.outakeGrip.setPosition(OuttakePixelDrop);
                    sleep(200);
                    outake.outakeGrip.setPosition(OuttakePixelGrip);
                    sleep(200);
                    outake.outakeGrip.setPosition(OuttakePixelDrop);
                    intake.IntakeMotor.setPower(0);



                })
                .waitSeconds(5)
                .setReversed(true)
////------------------------------------------------------------------------------------------------
                .build();


        TrajectorySequence dsTrajectory = drive.trajectorySequenceBuilder(new Pose2d(56.5,-40))
                .splineToConstantHeading(new Vector2d(56.5, -4), 0)
                .build();
        //TODO SETUP -INIT
        waitForStart();
        if (!isStopRequested()) {
            drive.followTrajectorySequence(trajectoryseq);

            if(dsFront.getDistance(DistanceUnit.CM) <11 && dsFront.getDistance(DistanceUnit.CM)>9){
                drive.followTrajectorySequence(dsTrajectory);
                telemetry.addLine("Distance Detected");
                //drive.setMotorPowers(0,0,0,0);
            }
            intake.ToUseWhenPixelGetsStuck();
            telemetry.update();
        }
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
        endgame.HangerInit();
        endgame.DroneInit();
        //  hanger_servo.setPosition(0.5);
        //. plane.setPosition(0.5);


    }
}

