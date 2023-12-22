//package org.firstinspires.ftc.teamcode.auto;
//
//import static org.firstinspires.ftc.teamcode.subsytem.Intake.leftServoGrippingPos;
//import static org.firstinspires.ftc.teamcode.subsytem.Outake.OuttakePixelDrop;
//import static org.firstinspires.ftc.teamcode.subsytem.Outake.OuttakePixelGrip;
//import static org.firstinspires.ftc.teamcode.subsytem.Outake.armGripPos;
//import static org.firstinspires.ftc.teamcode.subsytem.Outake.armPlacePos;
//
//import com.acmerobotics.roadrunner.geometry.Vector2d;
//import com.qualcomm.robotcore.hardware.DistanceSensor;
//import com.qualcomm.robotcore.hardware.HardwareMap;
//
//import org.firstinspires.ftc.robotcore.external.Telemetry;
//import org.firstinspires.ftc.teamcode.drive.DriveConstants;
//import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
//import org.firstinspires.ftc.teamcode.subsytem.HangerAndDrone;
//import org.firstinspires.ftc.teamcode.subsytem.Intake;
//import org.firstinspires.ftc.teamcode.subsytem.Lifter;
//import org.firstinspires.ftc.teamcode.subsytem.Outake;
//import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;
//
//public class SecondCycleTrajectories() {
//    SampleMecanumDrive drive = null;
//    Lifter lift =null;
//    Intake intake=null;
//    Outake outake=null;
//    HangerAndDrone endgame = null;
//
//    public static double turn = 0.6;
//    public static double bottomPos1 = 0.416, topPos1 = 0.385;
//    public static double bottomPos2 = 0.454, topPos2 = 0.431;
//
//    DistanceSensor dsFront = null, dsBack = null;
//    //
//
//    public static int value = 0;
//    public static int val=0;
//
//    public SecondCycleTrajectories(HardwareMap hardwareMap, Telemetry telemetry) {
//        drive = new SampleMecanumDrive(hardwareMap);
//        lift = new Lifter(hardwareMap, telemetry);
//        intake = new Intake(hardwareMap, telemetry);
//        outake = new Outake(hardwareMap, telemetry);
//        endgame = new HangerAndDrone(hardwareMap, telemetry);
//        dsFront = hardwareMap.get(DistanceSensor.class, "dsFront");
//        dsBack = hardwareMap.get(DistanceSensor.class, "dsBack");
//    }
//
//    public void NormalPath(){
//        TrajectorySequence trajectoryseq = drive.trajectorySequenceBuilder(startPose)
//                .addTemporalMarker(()->{intake.intakeAutoStart();
//                    InitFunction();
//                })
//                .UNSTABLE_addTemporalMarkerOffset(0.4, () -> {
//                    lift.extendToLow();
//                    outake.outakeArm.setPosition(armPlacePos);
//                })
//                .UNSTABLE_addTemporalMarkerOffset(1.4, () -> {
//                    intake.IntakeMotor.setPower(0.5);
//
//                })
//
//
//                .lineToConstantHeading(new Vector2d(26 , -36))
//                .splineToConstantHeading(new Vector2d(53,-40),0)
//                .splineTo(new Vector2d(57.3, -40), 0, SampleMecanumDrive.getVelocityConstraint(25, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
//                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL/2))
//                //.splineToConstantHeading(new Vector2d(56.5,-40),0)
//                .resetConstraints()
//                .UNSTABLE_addTemporalMarkerOffset(0.0000001, () ->
//                {
//                    outake.outakeGrip.setPosition(OuttakePixelDrop);
//
//                    intake.IntakeMotor.setPower(0);
//
//                })
//                .waitSeconds(0.6)
//
//                .setReversed(true)
//
//
//////  ------------------ Backdrop to First Two intake -----------------------------
//                .UNSTABLE_addTemporalMarkerOffset(0.4, () -> {
//                    outake.outakeGrip.setPosition(OuttakePixelGrip);
//                    outake.outakeArm.setPosition(armGripPos);
//                    //lift.extendToInit();
//                    DropMechInactive(0.7);
//
//                })
//                .splineToConstantHeading(new Vector2d(15,-8),-Math.PI)
//
//                .splineToConstantHeading(new Vector2d(-52.5,-9),Math.PI)
//
//                .UNSTABLE_addTemporalMarkerOffset(0.01, () -> {
//                    intake.IntakeMotor.setPower(-1);
//                    //intake.intakeStack(0.38);
//
//                    intake.IntakeTwoPixel(topPos1, bottomPos1);
//
//                    //.waitSeconds(0.01)
//                })
//
//                .waitSeconds(1.5)
//                .setReversed(false)
//                .lineToConstantHeading(new Vector2d(-49,-10))
//                .UNSTABLE_addTemporalMarkerOffset(0.0001, ()->{
//                    intake.intakeStack(leftServoGrippingPos);
//                })
//                .waitSeconds(0.3)
//                .UNSTABLE_addTemporalMarkerOffset(0.001, ()->{
//                    intake.intakeStack(topPos1);
//                })
//
//                .splineToConstantHeading(new Vector2d(17,-10), 0)
//                .UNSTABLE_addTemporalMarkerOffset(0.000001, () -> {
//
//                    outake.outakeArm.setPosition(armPlacePos);
//                    lift.extendToLow();
//                    //sleep(1000);
//
//                })
//                .splineToConstantHeading(new Vector2d(53,-31),0)
//                .splineTo(new Vector2d(57.3,-31),0, SampleMecanumDrive.getVelocityConstraint(25, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
//                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL/2))
//
//                .resetConstraints()
//
//                .UNSTABLE_addTemporalMarkerOffset(0.0000001, () -> {
//                    intake.IntakeMotor.setPower(0);
//                    outake.outakeGrip.setPosition(OuttakePixelDrop);
//                    sleep(300);
//                    outake.outakeGrip.setPosition(OuttakePixelGrip);
//                    sleep(200);
//                    outake.outakeGrip.setPosition(OuttakePixelDrop);
//
//                    outake.outakeGrip.setPosition(OuttakePixelDrop);
//                    sleep(300);
//                    outake.outakeGrip.setPosition(OuttakePixelGrip);
//                    sleep(200);
//                    outake.outakeGrip.setPosition(OuttakePixelDrop);
//                    intake.IntakeMotor.setPower(0);
//
//
//
//                })
//                .waitSeconds(1)
//                .setReversed(true)
//////----------------------------------------------------------------------------------------
//
//////  -------------------------------- SECOND CYCLE -------------------------------------
//                .UNSTABLE_addTemporalMarkerOffset(0.5, () -> {
//                    outake.outakeGrip.setPosition(OuttakePixelGrip);
//                    outake.outakeArm.setPosition(armGripPos);
//                    DropMechInactive(0.7);
//
//                })
//                .lineToConstantHeading(new Vector2d(15,-8))
//
//                .splineToConstantHeading(new Vector2d(-52.5,-8.5),Math.PI)
//
//                .UNSTABLE_addTemporalMarkerOffset(0.01, () -> {
//                    intake.IntakeMotor.setPower(-1);
//                    intake.IntakeTwoPixel(topPos2, bottomPos2);
//                })
//
//                .waitSeconds(1.5)
//                .setReversed(false)
//                .lineToConstantHeading(new Vector2d(-49,-10))
//                .UNSTABLE_addTemporalMarkerOffset(0.0001, ()->{
//                    intake.intakeStack(leftServoGrippingPos);
//                })
//                .waitSeconds(0.3)
//                .UNSTABLE_addTemporalMarkerOffset(0.4, ()->{
//                    intake.intakeStack(topPos1);
//                })
//
//                .lineToConstantHeading(new Vector2d(17,-10))
//                .UNSTABLE_addTemporalMarkerOffset(0.001, () -> {
//                    lift.extendToMedium();
//                    outake.outakeArm.setPosition(armPlacePos);
//
//                })
//                .splineToConstantHeading(new Vector2d(53,-31), 0)
//                .splineTo(new Vector2d(57.5,-31),0, SampleMecanumDrive.getVelocityConstraint(25, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
//                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL/2))
//                .resetConstraints()
//
//                .UNSTABLE_addTemporalMarkerOffset(0.0000001, () -> {
//                    intake.IntakeMotor.setPower(0);
//                    outake.outakeGrip.setPosition(OuttakePixelDrop);
//                    sleep(300);
//                    outake.outakeGrip.setPosition(OuttakePixelGrip);
//                    sleep(200);
//                    outake.outakeGrip.setPosition(OuttakePixelDrop);
//
//                    outake.outakeGrip.setPosition(OuttakePixelDrop);
//                    sleep(300);
//                    outake.outakeGrip.setPosition(OuttakePixelGrip);
//                    sleep(200);
//                    outake.outakeGrip.setPosition(OuttakePixelDrop);
//
//                })
//                .waitSeconds(2)
//                .UNSTABLE_addTemporalMarkerOffset(0.0001, ()->DropMechInactive(0.7))
//                .splineToConstantHeading(new Vector2d(53,-31), 0)
//                .waitSeconds(1)
//                .setReversed(true)
//////------------------------------------------------------------------------------------------------
//                .build();
//    }
//}
