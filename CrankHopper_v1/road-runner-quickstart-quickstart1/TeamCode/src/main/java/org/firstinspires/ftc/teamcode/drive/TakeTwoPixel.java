package org.firstinspires.ftc.teamcode.drive;

import static org.firstinspires.ftc.teamcode.subsytem.Intake.InL;
import static org.firstinspires.ftc.teamcode.subsytem.Intake.InR;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.subsytem.Intake;

@Autonomous
@Config
public class TakeTwoPixel extends LinearOpMode {

    Intake intake = null;
    public static double bottomPos = 0.414, topPos = 0.385;

    public void runOpMode(){
        intake = new Intake(hardwareMap, telemetry);

        waitForStart();
        while(opModeIsActive()) {
            intake.IntakeMotor.setPower(-1);
            intake.IntakeTwoPixel(topPos, bottomPos);
        }



        //}

    }
}
