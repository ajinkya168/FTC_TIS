package org.firstinspires.ftc.teamcode.auto1;

import static org.firstinspires.ftc.robotcontroller.external.samples.RobotTeleopPOV_Linear.MID_SERVO;
//import static org.firstinspires.ftc.teamcode.tank1.tank_1.MID_SERVO;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.drive.SampleTankDrive;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;

@Autonomous
public class tank1_auto2 extends LinearOpMode {
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
        GripIn.setPosition(MID_SERVO);
        GripOut.setPosition(MID_SERVO);
        Arm = hardwareMap.get(Servo.class, "Arm");
        Arm.setPosition(0.5);
//        lifter.setTargetPosition(500);
//        lifter.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//        lifter.setPower(-0.8);
////left
        SampleTankDrive drive = new SampleTankDrive(hardwareMap);
        Pose2d startPose = new Pose2d(11.0, 65.0,Math.toRadians(180));
        drive.setPoseEstimate(startPose);
        TrajectorySequence trajSeq = drive.trajectorySequenceBuilder(startPose)
                .forward(21)
                .turn(Math.toRadians(63))
//                .addTemporalMarker(()-> {drop1();})
                .waitSeconds(0.5)
                .turn(Math.toRadians(65))
                .forward(38)
//                .addTemporalMarker(()-> {drop2();})
                .build();
        waitForStart();
        if (!isStopRequested())
            drive.followTrajectorySequence(trajSeq);
    }
    public void drop1()
    {
        GripOut.setPosition(0);
    }
    public void drop2()
    {
        GripIn.setPosition(0);
    }
}


