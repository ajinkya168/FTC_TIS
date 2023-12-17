package org.firstinspires.ftc.teamcode.drive;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.teamcode.auto.Auto;
import org.firstinspires.ftc.teamcode.subsytem.Intake;
import org.firstinspires.ftc.teamcode.subsytem.Lifter;
import org.firstinspires.ftc.teamcode.subsytem.Outake;

@Autonomous
public class DropTwoPixel extends LinearOpMode {
    Outake outtake = null;
    Intake intake = null;
    Lifter lifter = null;
    public static int val = 0;
    public void runOpMode(){
        outtake = new Outake(hardwareMap, telemetry);
        lifter = new Lifter(hardwareMap, telemetry);
        intake = new Intake(hardwareMap, telemetry);
        InitFunction();

        waitForStart();
        while(opModeIsActive()) {
            lifter.extendToLow();
            outtake.outakeArm.setPosition(0.22);
            sleep(500);
            outtake.outakeGrip.setPosition(0.4);
        }
    }


    public void InitFunction(){

        lifter.leftElevator.setPower(-0.2);
        lifter.rightElevator.setPower(-0.2);
        if(lifter.leftElevator.getCurrent(CurrentUnit.AMPS)>=1 ) {
            lifter.extendTo(lifter.leftElevator.getCurrentPosition(), 0.2);
            val = lifter.leftElevator.getCurrentPosition();
        }


        intake.intakeInit();
        outtake.outakeGrip.setPosition(0.51);
        outtake.outakeArmInit();
        //  hanger_servo.setPosition(0.5);
        //. plane.setPosition(0.5);


    }
}
