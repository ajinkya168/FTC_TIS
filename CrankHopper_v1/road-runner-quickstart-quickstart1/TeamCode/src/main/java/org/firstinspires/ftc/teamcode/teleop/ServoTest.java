package org.firstinspires.ftc.teamcode.teleop;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;
@TeleOp
public class ServoTest extends LinearOpMode {
 Servo s1 = null;

 public void runOpMode(){
     s1 = hardwareMap.get(Servo.class, "s1");
     waitForStart();
     while(opModeIsActive()){
         if(gamepad1.x){

             s1.setPosition(0.970);
         }
         if(gamepad1.y){
             s1.setPosition(0.1);
         }
     }
 }
}

