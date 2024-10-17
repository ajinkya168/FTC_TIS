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
import org.firstinspires.ftc.teamcode.hardware.Globals;
import org.firstinspires.ftc.teamcode.hardware.Location;
import org.firstinspires.ftc.teamcode.hardware.PropPipeline;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.vision.VisionPortal;

import java.util.List;

@Autonomous
public class RedDropSafe extends LinearOpMode {
//    private satic final boolean USE_WEBCAM = true;  // true for webcam, false for phone camera

    // TFOD_MODEL_ASSET points to a model file stored in the project Asset location,
//    private static final String TFOD_MODEL_ASSET = "blackbox.tflite";
//    private static final String[] LABELS = {
//            "beacon"
//    };
//    private TfodProcessor tfod;
    private VisionPortal visionPortal;
    private PropPipeline propPipeline;
    private VisionPortal portal;
    private Location randomization;

    public static String marker = "CENTER";;
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

        //initTfod();
        Globals.IS_AUTO = true;
        Globals.ALLIANCE = Location.RED;
        Globals.SIDE = Location.CLOSE;

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
        Pose2d startPose = new Pose2d(11.0, -58,Math.toRadians(90));
        drive.setPoseEstimate(startPose);
        propPipeline = new PropPipeline();
        portal = new VisionPortal.Builder()
                .setCamera(hardwareMap.get(WebcamName.class, "Webcam"))
                .setCameraResolution(new Size(1280, 720))
                .addProcessor(propPipeline)
                .setStreamFormat(VisionPortal.StreamFormat.MJPEG)
                .enableLiveView(true)
                .setAutoStopLiveView(true)
                .build();

        //left
        TrajectorySequence trajectoryseqLeft = drive.trajectorySequenceBuilder(startPose)
                .addTemporalMarker(()-> {lift();})
                .waitSeconds(0.1)

                .forward(28)
                .turn(Math.toRadians(76))
//                .forward(2)
                .addTemporalMarker(()-> {drop1();})
                .waitSeconds(0.5)
                .turn(Math.toRadians(-210))
                .forward(48)
                .addTemporalMarker(()-> {drop2();})
                .build();

        //right
        TrajectorySequence trajectoryseqRight = drive.trajectorySequenceBuilder(startPose)
                .addTemporalMarker(()-> {lift();})
                .waitSeconds(0.1)
                .forward(21)
                .turn(Math.toRadians(-86))
                .addTemporalMarker(()-> {drop1();})
                .waitSeconds(0.5)
                .turn(Math.toRadians(-94))
                .forward(38)
                .addTemporalMarker(()-> {drop2();})
                .build();

        //center
        TrajectorySequence trajectoryseqCenter = drive.trajectorySequenceBuilder(startPose)
                .addTemporalMarker(()-> {lift();})
                .waitSeconds(0.1)

                .forward(32.5)
//                .addTemporalMarker(()-> {drop1();})
                .waitSeconds(0.5)
                .turn(Math.toRadians(-134))
                .forward(40)
//                .addTemporalMarker(()-> {drop2();})
                .build();

        while (getCameraState() != VisionPortal.CameraState.STREAMING && portal.getCameraState() != VisionPortal.CameraState.STREAMING) {
            telemetry.addLine("initializing... please wait");
            telemetry.update();
        }

        while(opModeInInit()){
            try {
                marker = propPipeline.getLocation().toString();
            }
            catch (Exception e){
                marker = "CENTER";
            }
            telemetry.addLine("ready");
            if(gamepad1.x){
                marker = "LEFT";
            } else if(gamepad1.y){
                marker = "CENTER";
            } else  if(gamepad1.b){
                marker = "RIGHT";
            }
            telemetry.addData("position", propPipeline.getLocation());
            telemetry.addData("marker", marker);
            telemetry.update();
        }


        portal.close();
        waitForStart();

        if(marker == "CENTER"){
            drive.followTrajectorySequence(trajectoryseqCenter);
        }
        else if(marker == "LEFT"){
            drive.followTrajectorySequence(trajectoryseqLeft);
        }
        else if( marker == "RIGHT"){
            drive.followTrajectorySequence(trajectoryseqRight);
        }


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

    public VisionPortal.CameraState getCameraState(){
        if(portal != null) return portal.getCameraState();
        return null;
    }
}


