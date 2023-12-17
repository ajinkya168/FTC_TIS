package org.firstinspires.ftc.teamcode.teleop;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name="spider")
@Config
public class SPIDER extends LinearOpMode {

    Servo FL,FR,RL,RR;
    Servo FLR,FRR,RLR,RRR;

    public static double fl=0.5;
    public static double fr=0.5;
    public static double rr=0.5;
    public static double rl=0.5;
    //
    public static double flr=0.8; //FLR
    public static double frr=0.1;  //FRR
    public static double rrr=0;
    public static double rlr=0.7;






    @Override
    public void runOpMode() throws InterruptedException {

        FL= hardwareMap.get(Servo.class, "FL");
        FR= hardwareMap.get(Servo.class, "FR");
        RR= hardwareMap.get(Servo.class, "RR");
        RL= hardwareMap.get(Servo.class, "RL");


        FLR= hardwareMap.get(Servo.class, "FLR");
        FRR= hardwareMap.get(Servo.class, "FRR");
        RRR= hardwareMap.get(Servo.class, "RRR");
        RLR= hardwareMap.get(Servo.class, "RLR");

        servoPos(0.5);

        waitForStart();

        while(opModeIsActive()){

            if(gamepad1.left_bumper){
                FLR.setPosition(flr);
                FRR.setPosition(frr);
                RRR.setPosition(rrr);
                RLR.setPosition(rlr);
            }

            if(gamepad1.right_bumper){

            }
            if(gamepad1.b){

            }
            if(gamepad1.x){

            }
//            if(gamepad1.a){
//                RR.setPosition(FL.getPosition()+0.01);
//            }if(gamepad1.y){
//                RL.setPosition(FL.getPosition()+0.01);
//            }

            if(gamepad1.dpad_up){
                FLR.setPosition(flr);    //fr
            }
            if(gamepad1.dpad_down){
                FRR.setPosition(frr);   //
            }
            if(gamepad1.dpad_right){
                RRR.setPosition(rrr);
            }if(gamepad1.dpad_left){
                RLR.setPosition(rlr);  //
            }


            if(gamepad1.start){
                FL.setPosition(0.5);//take leg up
                sleep(200);
                FLR.setPosition(0.);//leg forward
                sleep(400);
                FL.setPosition(0.2); //leg down
                sleep(600);
//
                RR.setPosition(0.5);
                sleep(200);
                RRR.setPosition(0);
                sleep(300);
                RR.setPosition(0.2);
                sleep(600);

                FR.setPosition(0.5);
                sleep(200);
                FRR.setPosition(0.6);
                sleep(300);
                FR.setPosition(0.7);
                sleep(600);
////
                RL.setPosition(0.5);
                sleep(200);
                RLR.setPosition(0.7);
                sleep(300);
                RL.setPosition(0.8);
                sleep(600);


                FL.setPosition(0.5);//take leg up
                sleep(200);
                FLR.setPosition(0.);//leg forward
                sleep(400);
                FL.setPosition(0.2); //leg down
                sleep(600);
//
                RR.setPosition(0.5);
                sleep(200);
                RRR.setPosition(0);
                sleep(300);
                RR.setPosition(0.2);
                sleep(600);

                FR.setPosition(0.5);
                sleep(200);
                FRR.setPosition(0.6);
                sleep(300);
                FR.setPosition(0.7);
                sleep(600);
////
                RL.setPosition(0.5);
                sleep(200);
                RLR.setPosition(0.7);
                sleep(300);
                RL.setPosition(0.8);
                sleep(600);


                FL.setPosition(0.5);//take leg up
                sleep(200);
                FLR.setPosition(0.);//leg forward
                sleep(400);
                FL.setPosition(0.2); //leg down
                sleep(600);
//
                RR.setPosition(0.5);
                sleep(200);
                RRR.setPosition(0);
                sleep(300);
                RR.setPosition(0.2);
                sleep(600);

                FR.setPosition(0.5);
                sleep(200);
                FRR.setPosition(0.6);
                sleep(300);
                FR.setPosition(0.7);
                sleep(600);
////
                RL.setPosition(0.5);
                sleep(200);
                RLR.setPosition(0.7);
                sleep(300);
                RL.setPosition(0.8);
                sleep(600);

                FL.setPosition(0.5);//take leg up
                sleep(200);
                FLR.setPosition(0.);//leg forward
                sleep(400);
                FL.setPosition(0.2); //leg down
                sleep(600);
//
                RR.setPosition(0.5);
                sleep(200);
                RRR.setPosition(0);
                sleep(300);
                RR.setPosition(0.2);
                sleep(600);

                FR.setPosition(0.5);
                sleep(200);
                FRR.setPosition(0.6);
                sleep(300);
                FR.setPosition(0.7);
                sleep(600);
////
                RL.setPosition(0.5);
                sleep(200);
                RLR.setPosition(0.7);
                sleep(300);
                RL.setPosition(0.8);
                sleep(600);








                /////////////////
//                FL.setPosition(0.5);//take leg up
//                sleep(200);
//                FLR.setPosition(0.1);//leg forward
//                sleep(400);
//                FL.setPosition(0.2); //leg down
//                sleep(600);
////
//                RR.setPosition(0.5);
//                sleep(200);
//                RRR.setPosition(0);
//                sleep(300);
//                RR.setPosition(0.2);
//                sleep(600);
//
//                FR.setPosition(0.5);
//                sleep(200);
//                FRR.setPosition(0.8);
//                sleep(300);
//                FR.setPosition(0.7);
//                sleep(600);
////
//                RL.setPosition(0.5);
//                sleep(200);
//                RLR.setPosition(0.7);
//                sleep(300);
//                RL.setPosition(0.8);
//                sleep(600);
//
//                //////////////////
//                FL.setPosition(0.5);//take leg up
//                sleep(200);
//                FLR.setPosition(0.1);//leg forward
//                sleep(400);
//                FL.setPosition(0.2); //leg down
//                sleep(600);
////
//                RR.setPosition(0.5);
//                sleep(200);
//                RRR.setPosition(0);
//                sleep(300);
//                RR.setPosition(0.2);
//                sleep(600);
//
//                FR.setPosition(0.5);
//                sleep(200);
//                FRR.setPosition(0.8);
//                sleep(300);
//                FR.setPosition(0.7);
//                sleep(600);
////
//                RL.setPosition(0.5);
//                sleep(200);
//                RLR.setPosition(0.7);
//                sleep(300);
//                RL.setPosition(0.8);
//                sleep(600);

                ///////  FL.setPosition(0.5);//take leg up
                //                sleep(200);
                //                FLR.setPosition(0.1);//leg forward
                //                sleep(400);
                //                FL.setPosition(0.2); //leg down
                //                sleep(600);
                ////
                //                RR.setPosition(0.5);
                //                sleep(200);
                //                RRR.setPosition(0);
                //                sleep(300);
                //                RR.setPosition(0.2);
                //                sleep(600);
                //
                //                FR.setPosition(0.5);
                //                sleep(200);
                //                FRR.setPosition(0.8);
                //                sleep(300);
                //                FR.setPosition(0.7);
                //                sleep(600);
                ////
                //                RL.setPosition(0.5);
                //                sleep(200);
                //                RLR.setPosition(0.7);
                //                sleep(300);
                //                RL.setPosition(0.8);
                //                sleep(600);



//                //////////////////////////////////////////
//                FL.setPosition(0.5);//take leg up
//                sleep(200);
//                FLR.setPosition(0.1);//leg forward
//                sleep(300);
//                FL.setPosition(0.2); //leg down
//                sleep(400);
////
//                RR.setPosition(0.5);
//                sleep(200);
//                RRR.setPosition(0);
//                sleep(300);
//                RR.setPosition(0.2);
//
//                FR.setPosition(0.5);
//                sleep(200);
//                FRR.setPosition(0.8);
//                sleep(300);
//                FR.setPosition(0.7);
////
//                RL.setPosition(0.5);
//                sleep(200);
//                RLR.setPosition(0.7);
//                sleep(300);
//                RL.setPosition(0.8);
//                ///////////////////////////
//                FL.setPosition(0.5);//take leg up
//                sleep(200);
//                FLR.setPosition(0.1);//leg forward
//                sleep(300);
//                FL.setPosition(0.2); //leg down
//                sleep(400);
////
//                RR.setPosition(0.5);
//                sleep(200);
//                RRR.setPosition(0);
//                sleep(300);
//                RR.setPosition(0.2);
//
//                FR.setPosition(0.5);
//                sleep(200);
//                FRR.setPosition(0.8);
//                sleep(300);
//                FR.setPosition(0.7);
////
//                RL.setPosition(0.5);
//                sleep(200);
//                RLR.setPosition(0.7);
//                sleep(300);
//                RL.setPosition(0.8);
//
//
//



            }




            //

//            if(gamepad2.b){
//                FL.setPosition(FL.getPosition()+0.01);
//            }
//            if(gamepad2.x){
//                FR.setPosition(FL.getPosition()+0.01);
//            }
//            if(gamepad2.a){
//                RR.setPosition(FL.getPosition()+0.01);
//            }if(gamepad2.y){
//                RL.setPosition(FL.getPosition()+0.01);
//            }


            if(gamepad2.dpad_up){
                FL.setPosition(FL.getPosition()-0.01);
            }
            if(gamepad2.dpad_down){
                FR.setPosition(FL.getPosition()-0.01);
            }
            if(gamepad2.dpad_right){
                RR.setPosition(FL.getPosition()-0.01);
            }if(gamepad2.dpad_left){
                RL.setPosition(FL.getPosition()-0.01);
            }






            telemetry.addData("FL",FL.getPosition());
            telemetry.addData("FR",FR.getPosition());
            telemetry.addData("RR",RR.getPosition());
            telemetry.addData("RL",RL.getPosition());
            telemetry.addData("FLR",FLR.getPosition());
            telemetry.addData("FRR",FRR.getPosition());
            telemetry.update();
//            telemetry.addData("FL",FL.getPosition());





        }







    }

    public void servoPos(double Pos){
        FL.setPosition(Pos);
        FR.setPosition(Pos);
        RL.setPosition(Pos);
        RR.setPosition(Pos);

    }
    public void servoPosR(double Pos){
        FLR.setPosition(Pos);
        FRR.setPosition(Pos);
        RLR.setPosition(Pos);
        RRR.setPosition(Pos);

    }
    public void servoPosCustom(double Pos1,double Pos2,double Pos3,double Pos4){
        FL.setPosition(Pos1);
        FR.setPosition(Pos2);
        RL.setPosition(Pos3);
        RR.setPosition(Pos4);

    }
//    public void servoPosCustomInc(){
//        FL.setPosition(FL.getPosition()+);
//        FR.setPosition(Pos2);
//        RL.setPosition(Pos3);
//        RR.setPosition(Pos4);
//
//    }




}
