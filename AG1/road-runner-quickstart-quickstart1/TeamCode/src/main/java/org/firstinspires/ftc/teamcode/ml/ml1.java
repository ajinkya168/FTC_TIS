package org.firstinspires.ftc.teamcode.ml;

import android.util.Size;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.teamcode.drive.SampleTankDrive;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.tfod.TfodProcessor;

import java.util.List;

@Autonomous
public class ml1 extends LinearOpMode {
    private static final boolean USE_WEBCAM = true;  // true for webcam, false for phone camera

    // TFOD_MODEL_ASSET points to a model file stored in the project Asset location,
    private static final String TFOD_MODEL_ASSET = "blackbox.tflite";
    private static final String[] LABELS = {
            "beacon"
    };
    private TfodProcessor tfod;
    private VisionPortal visionPortal;
    SampleTankDrive drive =null;

    double x;
    double y;
    String propPosition = " ";
    public DcMotor lifter = null;

    public Servo GripIn = null;
    public Servo GripOut = null;
    public Servo Arm = null;


    @Override
    public void runOpMode() throws InterruptedException {

        initTfod();


        lifter = hardwareMap.get(DcMotor.class, "lifter");
        lifter.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        lifter.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        lifter.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        GripIn = hardwareMap.get(Servo.class, "GripL");
        GripOut = hardwareMap.get(Servo.class, "GripR");
        GripIn.setPosition(0.3);
        GripOut.setPosition(0.15);
        Arm = hardwareMap.get(Servo.class, "Arm");
        Arm.setPosition(0.5);
        lifter.setTargetPosition(100);
        lifter.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        lifter.setPower(0.8);
        drive = new SampleTankDrive(hardwareMap);
        Pose2d startPose = new Pose2d(11.0, 65.0,Math.toRadians(180));
        drive.setPoseEstimate(startPose);


        //left
        TrajectorySequence trajectoryseqLeft = drive.trajectorySequenceBuilder(startPose)
                .addTemporalMarker(()-> {lift();})
                .waitSeconds(0.1)

                .forward(21)
                .turn(Math.toRadians(63))
//                .addTemporalMarker(()-> {drop1();})
                .waitSeconds(0.5)
                .turn(Math.toRadians(65))
                .forward(38)
//                .addTemporalMarker(()-> {drop2();})
                .build();

        //right
        TrajectorySequence trajectoryseqRight = drive.trajectorySequenceBuilder(startPose)
                .addTemporalMarker(()-> {lift();})
                .waitSeconds(0.1)

                .forward(24)
                .turn(Math.toRadians(-90))
//                .addTemporalMarker(()-> {drop1();}
                .waitSeconds(0.5)
                .turn(Math.toRadians(145))
                .forward(46.2)
//                .addTemporalMarker(()-> {drop2();})
                .build();

        //center
        TrajectorySequence trajectoryseqCenter = drive.trajectorySequenceBuilder(startPose)
                .addTemporalMarker(()-> {lift();})
                .waitSeconds(0.1)

                .forward(29)
//                .addTemporalMarker(()-> {drop1();})
                .waitSeconds(0.5)
                .turn(Math.toRadians(105))
                .forward(44)
//                .addTemporalMarker(()-> {drop2();})
                .build();



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
                    if( x>50 && x<=190){
                        propPosition  = "left";
                    }
                    else if(x>=200 && x<=350){
                        propPosition = "center";

                    }
                    else if(x>=400 && x<=550) {
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
            telemetry.update();
        }

        waitForStart();
        visionPortal.close();
        telemetry.addData("position",propPosition);
        telemetry.update();

        if (propPosition.equals("left")) {
            telemetry.addLine("left");
            telemetry.update();
            drive.followTrajectorySequence(trajectoryseqLeft);

        }
        else if (propPosition.equals("right")) {
            telemetry.addLine("right");
            telemetry.update();
            drive.followTrajectorySequence(trajectoryseqRight);

        }
        else if(propPosition.equals("center")) {
            telemetry.addLine("center");
            telemetry.update();
            drive.followTrajectorySequence(trajectoryseqCenter);
        }
        else{
            telemetry.addLine("not following traj");
            telemetry.update();
        }

//        Then follow your cycle trajectories

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

        // Create the vision portal by using a builder.
        VisionPortal.Builder builder = new VisionPortal.Builder();
        builder.setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"));


        // Choose a camera resolution. Not all cameras support all resolutions.
        builder.setCameraResolution(new Size(640, 480));

        // Enable the RC preview (LiveView).  Set "false" to omit camera monitoring.
        builder.enableLiveView(true);

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
    public void drop1()
    {
        Arm.setPosition(0.5);
        GripOut.setPosition(0.6);
    }
    public void drop2()
    {
        lifter.setTargetPosition(70);
        lifter.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        lifter.setPower(0.8);
        Arm.setPosition(0.5);
        GripIn.setPosition(0.6);
    }
    public void lift()
    {
        lifter.setTargetPosition(230);
        lifter.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        lifter.setPower(0.8);
    }
}


