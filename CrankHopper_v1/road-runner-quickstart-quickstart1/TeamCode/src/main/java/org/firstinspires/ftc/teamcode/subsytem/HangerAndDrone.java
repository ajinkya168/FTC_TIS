package org.firstinspires.ftc.teamcode.subsytem;

import  com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;

//@TeleOp
@Config
public class HangerAndDrone{
    public Servo hanger, drone;
    public DcMotor HangerMotor;
    public static double hangerInit = 0.38, droneInit = 0.9, droneShoot = 0.2, hangShoot = 0.8, i = 0;

    public boolean done = false;

    public HangerAndDrone(HardwareMap hardwareMap, Telemetry telemetry){
        hanger = hardwareMap.get(Servo.class, "hang");
        drone = hardwareMap.get(Servo.class, "plane");
        HangerMotor = hardwareMap.get(DcMotor.class, "hangerMotor");

        HangerMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        HangerMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        HangerMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }

    public void HangerInit(){
        hanger.setPosition(hangerInit);
    }

    public void DroneInit(){
        drone.setPosition(droneInit);
    }
    public void ShootDrone(){
        drone.setPosition(droneShoot);
    }

    public void HangerOn(){
        hanger.setPosition(hangShoot);
    }

    public  void HangerUp(){
        HangerMotor.setTargetPosition(HangerMotor.getCurrentPosition() + 10800);
        HangerMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        HangerMotor.setPower(1);
        done = true;

    }


    public void HangerDown(){
        HangerMotor.setTargetPosition(HangerMotor.getCurrentPosition() - 10800);
        HangerMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        HangerMotor.setPower(1);
    }

    public void HangerInc(int pos){
        HangerMotor.setTargetPosition(pos + 100);
        HangerMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        HangerMotor.setPower(1);
    }


//    public void runOpMode(){
//        hanger = hardwareMap.get(Servo.class, "hang");
//        drone = hardwareMap.get(Servo.class, "plane");
//        HangerMotor = hardwareMap.get(DcMotor.class, "hangerMotor");
//
//        HangerMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
//        HangerMotor.setDirection(DcMotorSimple.Direction.REVERSE);
//        HangerMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//
//        drone.setPosition(droneInit);
//        hanger.setPosition(hangerInit);
//        waitForStart();
//
//        while(opModeIsActive()){
//            if(gamepad1.dpad_up){
//                hanger.setPosition(hanger.getPosition() + 0.001);
//            }
//            if(gamepad1.dpad_down){
//                hanger.setPosition(hanger.getPosition() - 0.001);
//            }
//
//            if(gamepad1.dpad_left){
//                drone.setPosition(drone.getPosition() + 0.001);
//            }
//            if(gamepad1.dpad_right){
//                drone.setPosition(drone.getPosition() - 0.001);
//            }
//
//            if(gamepad1.x){
//                drone.setPosition(droneShoot);
//            }
//            if(gamepad1.b){
//                while(i < hangShoot){
//                    hanger.setPosition(hanger.getPosition() + 0.01);
//                }
//                //hanger.setPosition(hangShoot);
//            }
//
//            if(gamepad1.y){
//                //int pos = HangerMotor.getCurrentPosition() + 20;
//                HangerMotor.setTargetPosition(HangerMotor.getCurrentPosition() - 11488);
//                HangerMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//                HangerMotor.setPower(1);
//            }
//            if(gamepad1.a){
//              //  int pos = HangerMotor.getCurrentPosition() - 20;
//                HangerMotor.setTargetPosition(HangerMotor.getCurrentPosition() +  11488);
//                HangerMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//                HangerMotor.setPower(1);
//            }
//            telemetry.addData("Hanger Position:", hanger.getPosition());
//            telemetry.addData("Drone Position:", drone.getPosition());
//            telemetry.addData("HangerMotor pos: ", HangerMotor.getCurrentPosition());
//            telemetry.update();
//        }
//    }

}
