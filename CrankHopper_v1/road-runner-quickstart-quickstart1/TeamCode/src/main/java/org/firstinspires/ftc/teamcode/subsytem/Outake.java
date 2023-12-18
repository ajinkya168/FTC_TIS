package org.firstinspires.ftc.teamcode.subsytem;

import static android.os.SystemClock.sleep;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.checkerframework.checker.units.qual.C;
import org.firstinspires.ftc.robotcore.external.Telemetry;

@Config
public class Outake {


    public static Servo outakeGrip,outakeArm,outpush;

    public static double gripperDropPixel1=0.5;
    public static double gripperDropPixel2=0.5;

    public static double gripperClose=0.5;
 

    public static double OuttakePixelGrip = 0.57; //0.6;//gripper close or initial position
    public static double OuttakePixelDrop = 0.2;

    public static double Outakepushout =0.465;
    public static double Outakepushin = 0.51;

    public static double outgripopen = 0.432;
    public static double outgripclose = 0.505;

    public static double armGripPos = 0.92;     //0.95
    public static double armInitPos= 0.92;      //0.95
    public static double armPlacePos=0.57;






    public Outake(HardwareMap hardwareMap, Telemetry telemetry) {
        outakeGrip = hardwareMap.get(Servo.class, "out");
        outpush = hardwareMap.get(Servo.class,"outpush");
        outakeArm = hardwareMap.get(Servo.class, "outarm");
    }

    //TODO ARM POS

    public void outakeArmInit(){
       outakeArm.setPosition(armInitPos);
    }
    public void outakeArmPlace(){
        outakeArm.setPosition(armPlacePos);
    }
    public void outakeArmGripPos(){
        outakeArm.setPosition(armGripPos);
    }

    public void outakepusho(){outpush.setPosition(Outakepushout);}
    public void outakepushi(){outpush.setPosition(Outakepushin);}

    public void gripopen(){outakeGrip.setPosition(outgripopen);}
    public void gripclose(){outakeGrip.setPosition(outgripclose);}


    public void outakeArmINC(){
        outakeArm.setPosition(outakeArm.getPosition()+0.05);
    }

    public void outakeArmDEC(){
        outakeArm.setPosition(outakeArm.getPosition()-0.05);
    }


    ////TODO GRIPPER
    public void dropPixel1(){
//        outakeGrip.setPosition(OuttakePixelDrop)

        outakeGrip.setPosition(OuttakePixelDrop);
    }
    public void dropPixel2(){
        outakeGrip.setPosition(OuttakePixelGrip);
        sleep(1000);

        outakeGrip.setPosition(OuttakePixelDrop);
    }
    public void PixelDrop(){
//        outakeGrip.setPosition(OuttakePixelDrop);
//        sleep(400);
//        outakeGrip.setPosition(OuttakePixelGrip);
//        sleep(200);
//        outakeGrip.setPosition(OuttakePixelDrop);
//        sleep(400);
//        outakeGrip.setPosition(OuttakePixelGrip);
        gripopen();
        sleep(200);
        outakepusho();
        sleep(300);
        gripclose();
        sleep(200);
        outakepushi();
    }
    public void PixelDrop2(){
        outakeGrip.setPosition(0.6494);
    }
    public void closeGripper(){
        outakeGrip.setPosition(OuttakePixelGrip);
    }
    public void openGripper(){
        outakeGrip.setPosition(OuttakePixelDrop);
    }


    public void outakeGripINC(){
        outakeGrip.setPosition(outakeGrip.getPosition()+0.05);
    }

    public void outakeGripDEC(){
        outakeGrip.setPosition(outakeGrip.getPosition()-0.05);
    }

}

