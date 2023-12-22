package org.firstinspires.ftc.teamcode.drive;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.teamcode.subsytem.Intake;
import org.firstinspires.ftc.teamcode.subsytem.Lifter;

@Autonomous
public class InitFuncTest extends LinearOpMode {
    Lifter lift = null;
    Intake intake = null;
    double val = 0;
    public void runOpMode(){
        lift = new Lifter(hardwareMap, telemetry);
        intake = new Intake(hardwareMap, telemetry);
        InitFunction();
        intake.intakeInit();
        waitForStart();

        while(opModeIsActive()){
            telemetry.addData("Left Elevator :", lift.leftElevator.getCurrentPosition());
            telemetry.addData("Right Elevator :", lift.rightElevator.getCurrentPosition());
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


        //  hanger_servo.setPosition(0.5);
        //. plane.setPosition(0.5);


    }
}
