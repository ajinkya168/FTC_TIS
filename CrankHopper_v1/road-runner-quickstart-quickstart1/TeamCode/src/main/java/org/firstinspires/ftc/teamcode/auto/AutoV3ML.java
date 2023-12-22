package org.firstinspires.ftc.teamcode.auto;//package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.teamcode.subsytem.Intake.leftServoGrippingPos;
import static org.firstinspires.ftc.teamcode.subsytem.Outake.OuttakePixelDrop;
import static org.firstinspires.ftc.teamcode.subsytem.Outake.OuttakePixelGrip;
import static org.firstinspires.ftc.teamcode.subsytem.Outake.armGripPos;
import static org.firstinspires.ftc.teamcode.subsytem.Outake.armPlacePos;

import android.util.Size;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DistanceSensor;

import org.firstinspires.ftc.robotcontroller.external.samples.ConceptTensorFlowObjectDetection;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.teamcode.drive.DriveConstants;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.subsytem.HangerAndDrone;
import org.firstinspires.ftc.teamcode.subsytem.Intake;
import org.firstinspires.ftc.teamcode.subsytem.Lifter;
import org.firstinspires.ftc.teamcode.subsytem.Outake;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.tfod.TfodProcessor;

import java.util.List;

@Autonomous
public class AutoV3ML extends LinearOpMode {
    private static final boolean USE_WEBCAM = true;  // true for webcam, false for phone camera

    // TFOD_MODEL_ASSET points to a model file stored in the project Asset location,
    private static final String TFOD_MODEL_ASSET = "blackbox.tflite";
    private static final String[] LABELS = {
            "beacon"
    };
    private TfodProcessor tfod;
    private VisionPortal visionPortal;
    public static int val=0;
    double x;
    double y;
    String propPosition = " ";
    AutoV3 traj = null;
    SampleMecanumDrive drive = null;
    Lifter lift =null;
    Intake intake=null;
    Outake outake=null;
    HangerAndDrone endgame = null;
    DistanceSensor dsFront = null, dsBack = null;
    public static double bottomPos1 = 0.416, topPos1 = 0.385;
    public static double bottomPos2 = 0.454, topPos2 = 0.431;
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
        initTfod();
        Pose2d startPose = new Pose2d(12,-62,Math.toRadians(0));
        drive.setPoseEstimate(startPose);
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
                .UNSTABLE_addTemporalMarkerOffset(1.4, () -> {
                    intake.IntakeMotor.setPower(0.5);

                })


                .lineToConstantHeading(new Vector2d(26 , -36))
                .splineToConstantHeading(new Vector2d(53,-40),0)
                .splineTo(new Vector2d(57.3, -40), 0, SampleMecanumDrive.getVelocityConstraint(25, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
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


////  ------------------ Backdrop to First Two intake -----------------------------
                .UNSTABLE_addTemporalMarkerOffset(0.4, () -> {
                    outake.outakeGrip.setPosition(OuttakePixelGrip);
                    outake.outakeArm.setPosition(armGripPos);
                    //lift.extendToInit();
                    DropMechInactive(0.7);

                })
                .splineToConstantHeading(new Vector2d(15,-8),-Math.PI)

                .splineToConstantHeading(new Vector2d(-52.5,-9),Math.PI)

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
                .waitSeconds(0.3)
                .UNSTABLE_addTemporalMarkerOffset(0.001, ()->{
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
                .waitSeconds(1)
                .setReversed(true)
////----------------------------------------------------------------------------------------

////  -------------------------------- SECOND CYCLE -------------------------------------
                .UNSTABLE_addTemporalMarkerOffset(0.5, () -> {
                    outake.outakeGrip.setPosition(OuttakePixelGrip);
                    outake.outakeArm.setPosition(armGripPos);
                    DropMechInactive(0.7);

                })
                .lineToConstantHeading(new Vector2d(15,-8))

                .splineToConstantHeading(new Vector2d(-52.5,-8.5),Math.PI)

                .UNSTABLE_addTemporalMarkerOffset(0.01, () -> {
                    intake.IntakeMotor.setPower(-1);
                    intake.IntakeTwoPixel(topPos2, bottomPos2);
                })

                .waitSeconds(1.5)
                .setReversed(false)
                .lineToConstantHeading(new Vector2d(-49,-10))
                .UNSTABLE_addTemporalMarkerOffset(0.0001, ()->{
                    intake.intakeStack(leftServoGrippingPos);
                })
                .waitSeconds(0.3)
                .UNSTABLE_addTemporalMarkerOffset(0.4, ()->{
                    intake.intakeStack(topPos1);
                })

                .lineToConstantHeading(new Vector2d(17,-10))
                .UNSTABLE_addTemporalMarkerOffset(0.001, () -> {
                    lift.extendToMedium();
                    outake.outakeArm.setPosition(armPlacePos);

                })
                .splineToConstantHeading(new Vector2d(53,-31), 0)
                .splineTo(new Vector2d(57.5,-31),0, SampleMecanumDrive.getVelocityConstraint(25, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
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

                })
                .waitSeconds(2)
                .UNSTABLE_addTemporalMarkerOffset(0.0001, ()->DropMechInactive(0.7))
                .splineToConstantHeading(new Vector2d(53,-31), 0)
                .waitSeconds(1)
                .setReversed(true)
////------------------------------------------------------------------------------------------------
                .build();


// -------------------------------- With Detection ------------------------------------//
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


                .lineToConstantHeading(new Vector2d(26 , -36))
                .splineToConstantHeading(new Vector2d(53,-40),0)
                .splineTo(new Vector2d(57.3, -40), 0, SampleMecanumDrive.getVelocityConstraint(25, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
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


        while (isStopRequested() == false && isStarted() == false )   //&& ParkingZone == "None"
        {
            List<Recognition> currentRecognitions = tfod.getRecognitions();
            telemetry.addData("# Objects Detected", currentRecognitions.size());

            if (currentRecognitions.size() != 0) {

                boolean objectFound = false;

                for (Recognition recognition : currentRecognitions) {
                    x = (recognition.getLeft() + recognition.getRight()) / 2;
                    y = (recognition.getTop() + recognition.getBottom()) / 2;

                    objectFound = true;

                    telemetry.addLine("Beacon");
                    telemetry.addData("", " ");
                    telemetry.addData("Image", "%s (%.0f %% Conf.)", recognition.getLabel(), recognition.getConfidence() * 100);
                    telemetry.addData("- Position", "%.0f / %.0f", x, y);
                    telemetry.addData("- Size", "%.0f x %.0f", recognition.getWidth(), recognition.getHeight());
                    telemetry.update();

                    break;
                }

                if(objectFound){

//                    Adjust values according to your bot and camera position
                    if( x>400 && x<=500){
                        propPosition  = "left";
                    }
                    else if(x>=600 && x<=700){
                        propPosition = "center";
                    }
                    else if(x>=1000) {
                        propPosition = "right";
                    }


                }
                else{
                    telemetry.addLine("Don't see the beacon :(");
                }
            }
            else{
                telemetry.addLine("Don't see the beacon :(");
            }
        }


        waitForStart();
        visionPortal.close();
        telemetry.addData("position",propPosition);
        telemetry.update();
        sleep(500);



        //            drive.followTrajectorSequence(positionLeft);

            if (propPosition.equals("left")) {
                telemetry.addLine("left");
                telemetry.update();
                drive.followTrajectorySequence(mltrajectoryseq);
            } else if (propPosition.equals("right")) {
                drive.followTrajectorySequence(trajectoryseq);
            }



}
    private void initTfod() {

        // Create the TensorFlow processor by using a builder.
        tfod = new TfodProcessor.Builder()

                // With the following lines commented out, the default TfodProcessor Builder
                // will load the default model for the season. To define a custom model to load,
                // choose one of the following:
                .setModelAssetName(TFOD_MODEL_ASSET)
                .setModelLabels(LABELS)
                //.setIsModelTensorFlow2(true)
                //.setIsModelQuantized(true)
                //.setModelInputSize(300)
                //.setModelAspectRatio(16.0 / 9.0)

                .build();

        // C reate the vision portal by using a builder.


        VisionPortal.Builder builder = new VisionPortal.Builder();
        builder.setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"));


        // Choose a camera resolution. Not all cameras support all resolutions.
        builder.setCameraResolution(new Size(1280, 720));

        // Enable the RC preview (LiveView).  Set "false" to omit camera monitoring.
        builder.enableCameraMonitoring(true);

        // Set the stream format; MJPEG uses less bandwidth than default YUY2.
        builder.setStreamFormat(VisionPortal.StreamFormat.YUY2);

        // Choose whether or not LiveView stops if no processors are enabled.
        // If set "true", monitor shows solid orange screen if no processors enabled.
        // If set "false", monitor shows camera view without annotations.
        builder.setAutoStopLiveView(false);

        // Set and enable the processor.
        builder.addProcessor(tfod);

        // Build the Vision Portal, using the above settings.
        visionPortal = builder.build();
        telemetry.addLine("init done");
        telemetry.update();

        // Set confidence threshold for TFOD recognitions, at any time.
        //tfod.setMinResultConfidence(0.75f);

        // Disable or re-enable the TFOD processor at any time.
        //visionPortal.setProcessorEnabled(tfod, true);

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



        }

        public  void DropMechInactive(double elevatorpower){
            lift.extendTo(val,elevatorpower);
            Outake.outakeArm.setPosition(0.7);
            Outake.outakeArm.setPosition(0.8);
            outake.outakeArmGripPos();
            outake.closeGripper();
        }


}



