package org.firstinspires.ftc.teamcode.teleop;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.teamcode.subsytem.HangerAndDrone;

@TeleOp
public class HangerTest extends LinearOpMode {
    HangerAndDrone HangerMotor = null, HangerServo;


    double HangerMotorinit = 0.0;

    public void runOpMode(){
        HangerMotor = new HangerAndDrone(hardwareMap, telemetry);
        HangerServo = new HangerAndDrone(hardwareMap, telemetry);
        waitForStart();

        while(opModeIsActive()) {
            if (gamepad1.dpad_up) {
                int pos = HangerMotor.HangerMotor.getCurrentPosition() + 10;
                HangerMotor.HangerMotor.setTargetPosition(pos);
                HangerMotor.HangerMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                HangerMotor.HangerMotor.setPower(1);
            }
            if (gamepad1.dpad_down) {
                int pos = HangerMotor.HangerMotor.getCurrentPosition() - 10;
                HangerMotor.HangerMotor.setTargetPosition(pos);
                HangerMotor.HangerMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                HangerMotor.HangerMotor.setPower(1);
            }

            if (gamepad1.x) {
                HangerServo.hanger.setPosition(HangerServo.hanger.getPosition() + 0.01);
            }
            if (gamepad1.b) {
                HangerServo.hanger.setPosition((HangerServo.hanger.getPosition() - 0.01));
            }

            telemetry.addData("Hanger Motor: ", HangerMotor.HangerMotor.getCurrentPosition());
            telemetry.addData("hanger servo :", HangerServo.hanger.getPosition());
            telemetry.update();
        }
    }
}
