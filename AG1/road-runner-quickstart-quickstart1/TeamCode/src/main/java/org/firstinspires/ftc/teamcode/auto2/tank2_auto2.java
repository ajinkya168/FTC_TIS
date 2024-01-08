package org.firstinspires.ftc.teamcode.auto2;//package org.firstinspires.ftc.teamcode.tank2;


import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.drive.SampleTankDrive;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;

@Autonomous
public class tank2_auto2 extends LinearOpMode {
    int lifter_pos;
    int Arm_pos;
    public DcMotor lifter = null;

    public Servo GripIn = null;
    public Servo GripOut = null;
    public Servo Arm = null;


    @Override
    public void runOpMode() throws InterruptedException {
        lifter = hardwareMap.get(DcMotor.class, "lifter");
        lifter.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        lifter.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        lifter.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        GripIn = hardwareMap.get(Servo.class, "GripL");
        GripOut = hardwareMap.get(Servo.class, "GripR");
        GripIn.setPosition(0.3);
        GripOut.setPosition(0);
        Arm = hardwareMap.get(Servo.class, "Arm");
        Arm.setPosition(0.5);


//right
        SampleTankDrive drive = new SampleTankDrive(hardwareMap);
        Pose2d startPose = new Pose2d(11.0, -58,Math.toRadians(90));
        drive.setPoseEstimate(startPose);
        TrajectorySequence trajSeq = drive.trajectorySequenceBuilder(startPose)
//                .addTemporalMarker(()-> {lift();})
                .waitSeconds(0.1)
                .forward(21)
                .turn(Math.toRadians(-86))
                .addTemporalMarker(()-> {drop1();})
                .waitSeconds(0.5)
                .turn(Math.toRadians(-94))
                .forward(38)
                .addTemporalMarker(()-> {drop2();})
                .build();
        waitForStart();
        if (!isStopRequested())
            drive.followTrajectorySequence(trajSeq);
    }
    public void drop1()
    {
        Arm.setPosition(0.5);
        GripOut.setPosition(0.6);
    }
    public void drop2()
    {
//        lifter.setTargetPosition(70);
//        lifter.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//        lifter.setPower(0.8);
        Arm.setPosition(0.5);
        GripIn.setPosition(0.6);
    }
    public void lift()
    {
        lifter.setTargetPosition(200);
        lifter.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        lifter.setPower(0.8);
    }
}



