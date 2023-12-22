package org.firstinspires.ftc.teamcode.teleop;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.subsytem.HangerAndDrone;

@TeleOp
public class DroneTest extends LinearOpMode {

    HangerAndDrone endgame = null;

    public void runOpMode(){
        endgame = new HangerAndDrone(hardwareMap, telemetry);

        waitForStart();

        while(opModeIsActive()){
            if(gamepad1.x){
                endgame.drone.setPosition(0);
            }
            if(gamepad1.b){
                endgame.drone.setPosition(1);
            }
            if(gamepad1.y){
                endgame.drone.setPosition(0.5);
            }
        }
    }

}
