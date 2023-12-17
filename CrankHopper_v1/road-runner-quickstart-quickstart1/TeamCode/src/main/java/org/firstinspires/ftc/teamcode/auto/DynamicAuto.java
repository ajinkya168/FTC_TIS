package org.firstinspires.ftc.teamcode.auto;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DistanceSensor;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.subsytem.Intake;
import org.firstinspires.ftc.teamcode.subsytem.Lifter;
import org.firstinspires.ftc.teamcode.subsytem.Outake;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequenceBuilder;
@Autonomous
public class DynamicAuto extends LinearOpMode {

    SampleMecanumDrive drive = null;
    Lifter lift =null;
    Intake intake=null;
    Outake outake=null;
    DistanceSensor ds = null;

    public String state = null;

    public void runOpMode(){
        drive = new SampleMecanumDrive(hardwareMap);
        lift= new Lifter(hardwareMap,telemetry);
        intake=new Intake(hardwareMap,telemetry);
        outake=new Outake(hardwareMap,telemetry);
        ds = hardwareMap.get(DistanceSensor.class, "ds");

        Pose2d startPose = new Pose2d(12,-62,Math.toRadians(0));
        drive.setPoseEstimate(startPose);

//------------------------- Dynamic Trajectory ------------------------------//

        TrajectorySequence ts1 = drive.trajectorySequenceBuilder(startPose)
                //.addTemporalMarker(()->intake.intakeAutoStart())
                .lineToConstantHeading(new Vector2d(26 , -36))

                .splineToConstantHeading(new Vector2d(53,-40),0)
                .splineToConstantHeading(new Vector2d(57,-40),0)
                .waitSeconds(2)
                .splineToConstantHeading(new Vector2d(12,-62),0)
                .UNSTABLE_addTemporalMarkerOffset(0.1,()->{
                    autoTele();
                    if(ds.getDistance(DistanceUnit.CM) > 57 && ds.getDistance(DistanceUnit.CM) < 117){
                        //telemetry.addLine("Object Detected");
                        state = "detected";
                        autoTele();
                    }
                    else{
                       // telemetry.addLine("Object Not detected");
                        state = "notDetected";
                        autoTele();
                    }

                })
                .waitSeconds(2)
                .build();

        TrajectorySequence ts2 = drive.trajectorySequenceBuilder(startPose)
                .splineToConstantHeading(new Vector2d(12,-62.001),0)
                .addTemporalMarker(()->autoTele())
                        .build();
        waitForStart();

        if(!isStopRequested()){
            drive.followTrajectorySequence(ts1);
            switch (state){
                case "detected" : drive.followTrajectorySequence(ts2);
                    break;
                case "notDetected": drive.followTrajectorySequence(ts1);
                    break;
            }
        }

    }
    public void autoTele(){
        telemetry.addData("distance",ds.getDistance(DistanceUnit.CM));
    }
}
