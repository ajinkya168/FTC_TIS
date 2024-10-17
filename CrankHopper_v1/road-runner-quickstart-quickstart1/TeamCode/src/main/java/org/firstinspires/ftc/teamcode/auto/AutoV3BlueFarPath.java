package org.firstinspires.ftc.teamcode.auto;



import static org.firstinspires.ftc.teamcode.subsytem.Intake.leftServoGrippingPos;
import static org.firstinspires.ftc.teamcode.subsytem.Outake.OuttakePixelDrop;
import static org.firstinspires.ftc.teamcode.subsytem.Outake.OuttakePixelGrip;
import static org.firstinspires.ftc.teamcode.subsytem.Outake.armGripPos;
import static org.firstinspires.ftc.teamcode.subsytem.Outake.armPlacePos;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DistanceSensor;

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
//@Config
public class AutoV3BlueFarPath extends LinearOpMode {


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



    public void runOpMode() throws InterruptedException {

        drive = new SampleMecanumDrive(hardwareMap);
        lift= new Lifter(hardwareMap,telemetry);
        intake=new Intake(hardwareMap,telemetry);
        outake=new Outake(hardwareMap,telemetry);
        endgame = new HangerAndDrone(hardwareMap, telemetry);
        dsFront = hardwareMap.get(DistanceSensor.class, "dsFront");
        dsBack = hardwareMap.get(DistanceSensor.class, "dsBack");
        InitFunction();
        Pose2d newStartPose = null;
        Pose2d startPose = new Pose2d(-33,69,Math.toRadians(180));
        drive.setPoseEstimate(startPose);

//        TrajectorySequence trajectoryseq
//  -------------------------------------- LEFT CYCLE ----------------------------------------------
//        ------------------------ Start point to intake ------------------------------

        TrajectorySequence trajectoryseqLeft = drive.trajectorySequenceBuilder(startPose)
                .addTemporalMarker(()->{intake.intakeAutoStart();
                    InitFunction();
                })
                .UNSTABLE_addTemporalMarkerOffset(1.4, () -> {
                    intake.IntakeMotor.setPower(0.7);
                })

                .lineToConstantHeading(new Vector2d(-32, 37))
                .waitSeconds(0.5)
               // .build();
//---------------------------------------------------------------------------------------------

            //   TrajectorySequenceBuilder NormalTrajectory = drive.trajectorySequenceBuilder(newStartPose)
                .splineToLinearHeading(new Pose2d(-50,38, Math.toRadians(0)),0)
                .lineToConstantHeading(new Vector2d(-54.5, 39))
                .UNSTABLE_addTemporalMarkerOffset(0.0000001, ()->{
                    intake.IntakeMotor.setPower(-1);
                    intake.IntakeOnePixel(topPos1);
                })
                .waitSeconds(1.5)
                .setReversed(true)
                .lineToConstantHeading(new Vector2d(-48,38))
                .UNSTABLE_addTemporalMarkerOffset(0.000001, ()->{
                    intake.InR.setPosition(0.53);
                    intake.InL.setPosition(0.55);
                })
// ---------------------------------- intake to Backrop -----------------------------
                .waitSeconds(0.3)
                .lineToConstantHeading(new Vector2d(-36, 14)) //close to start
                .splineToConstantHeading(new Vector2d(15, 14), 0)
                .UNSTABLE_addTemporalMarkerOffset(0.000001, () -> {

                    outake.outakeArm.setPosition(0.42);
                    lift.extendToLow();
                    //sleep(1000);

                })
                .splineToConstantHeading(new Vector2d(45,46.5),0) //backdrop
                .splineTo(new Vector2d(55.2, 46.5), 0, SampleMecanumDrive.getVelocityConstraint(25, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL/2))
                .resetConstraints()
                .UNSTABLE_addTemporalMarkerOffset(0.0000001, () -> {
                    intake.IntakeMotor.setPower(0);
                    outake.outakeGrip.setPosition(OuttakePixelDrop);
                    sleep(300);
                    outake.outakeGrip.setPosition(OuttakePixelGrip);
                    sleep(200);
                    outake.outakeGrip.setPosition(OuttakePixelDrop);

                    outake.outakeGrip.setPosition(OuttakePixelDrop);
                    sleep(300);
                    outake.outakeGrip.setPosition(OuttakePixelGrip);
                    sleep(200);
                    outake.outakeGrip.setPosition(OuttakePixelDrop);
                    intake.IntakeMotor.setPower(0);

                })
                .waitSeconds(2)
                .UNSTABLE_addTemporalMarkerOffset(0.0001, ()->DropMechInactive(0.7))
                .splineToConstantHeading(new Vector2d(50,46.5), 0)
                .waitSeconds(1)
                .setReversed(true)
                .build();

//_--------------------------------------------------------------------------------------------//



// -------------------------------- with Detection ------------------------------------//
        TrajectorySequence mltrajectoryseq = drive.trajectorySequenceBuilder( startPose)
                .addTemporalMarker(()->{intake.intakeAutoStart();
                    InitFunction();
                })
                .UNSTABLE_addTemporalMarkerOffset(0.4, () -> {
                    lift.extendToLow();
                    outake.outakeArm.setPosition(armPlacePos);
                })
                .UNSTABLE_addTemporalMarkerOffset(1.4, () -> {
                    intake.IntakeMotor.setPower(0.5);

                })


                .lineToConstantHeading(new Vector2d(26 , 36))
                .splineToConstantHeading(new Vector2d(53,40),0)
                .splineTo(new Vector2d(57.3, 40), 0, SampleMecanumDrive.getVelocityConstraint(25, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL/2))
                //.splineToConstantHeading(new Vector2d(56.5,-40),0)
                .resetConstraints()
                .UNSTABLE_addTemporalMarkerOffset(0.0000001, () ->
                {
                    outake.outakeGrip.setPosition(OuttakePixelDrop);

                    intake.IntakeMotor.setPower(0);

                })
                .waitSeconds(0.6)

                .setReversed(true)
                .build();

//--------------------------------------------------------------------------------------//

        waitForStart();
        //----------------------------- MAIN DRIVING CODE ---------------------------------------//
        if (!isStopRequested()) {
            drive.followTrajectorySequence(trajectoryseqLeft);
            intake.ToUseWhenPixelGetsStuck();
            telemetry.update();
        }
    }
    // -----------------------------------------------------------------------------------------
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



    }

    public  void DropMechInactive(double elevatorpower){
        lift.extendTo(val,elevatorpower);
        Outake.outakeArm.setPosition(0.7);
        Outake.outakeArm.setPosition(0.8);
        outake.outakeArmGripPos();
        outake.closeGripper();
    }

    // public TrajectorySequence Path1(){
//        drive.followTrajectorySequence(Auto );
//    }
}

