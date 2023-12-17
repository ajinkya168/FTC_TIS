package org.firstinspires.ftc.teamcode.teleop;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DistanceSensor;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

@TeleOp
public class revDistance extends LinearOpMode {
    DistanceSensor dsFront = null, dsBack = null;
    public void runOpMode(){
        dsFront = hardwareMap.get(DistanceSensor.class, "dsFront");
        dsBack = hardwareMap.get(DistanceSensor.class, "dsBack");
        waitForStart();


        while(opModeIsActive()){
            telemetry.addData("Distance Front:", dsFront.getDistance(DistanceUnit.CM));
            telemetry.addData("Distance Back:", dsBack.getDistance(DistanceUnit.CM));
            telemetry.update();
        }
    }
}
