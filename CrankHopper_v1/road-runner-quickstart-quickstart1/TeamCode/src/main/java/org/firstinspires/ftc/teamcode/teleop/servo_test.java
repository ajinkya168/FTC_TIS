package org.firstinspires.ftc.teamcode.teleop;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp
@Config
public class servo_test extends LinearOpMode {

    Servo servo1;
    Servo servo2;

    public static double s1pos1 = 0.505; //close
    public static double s1pos2 = 0.482; //open
    public static double s2pos1 = 0.51;//back
    public static double s2pos2 = 0.465; //push


    @Override
    public void runOpMode() throws InterruptedException {

        servo1 = hardwareMap.get(Servo.class, "s1");
     //   servo2 = hardwareMap.get(Servo.class, "s2");

        waitForStart();
        while(opModeIsActive()){


            if(gamepad1.x){
                servo1.setPosition(s1pos1);

            }
            if(gamepad1.y){
                servo1.setPosition(s1pos2);
            }
//            if(gamepad1.a){
//                servo2.setPosition(s2pos1);
//            }
//            if(gamepad1.b){
//                servo2.setPosition(s2pos2);
//            }
//            if(gamepad1.dpad_up){
//                servo2.setPosition(0.5);
            }
        }
    }
//}
