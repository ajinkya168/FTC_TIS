package org.firstinspires.ftc.teamcode;//package org.firstinspires.ftc.teamcode.tank1;

/* Copyright (c) 2017 FIRST. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided that
 * the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this list
 * of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice, this
 * list of conditions and the following disclaimer in the documentation and/or
 * other materials provided with the distribution.
 *
 * Neither the name of FIRST nor the names of its contributors may be used to endorse or
 * promote products derived from this software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
 * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */


import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.teamcode.drive.SampleTankDrive;

/*
 * This OpMode executes a Tank Drive control TeleOp a direct drive robot
 * The code is structured as an Iterative OpMode
 *
 * In this mode, the left and right joysticks control the left and right motors respectively.
 * Pushing a joystick forward will make the attached motor drive forward.
 * It raises and lowers the claw using the Gamepad Y and A buttons respectively.
 * It also opens and closes the claws slowly using the left and right Bumper buttons.
 *
 * Use Android Studio to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this OpMode to the Driver Station OpMode list
 */

@TeleOp
@Config
public class tank_2 extends LinearOpMode {

    /* Declare OpMode members. */
    public DcMotor leftDrive = null;
    public DcMotor rightDrive = null;
    public DcMotorEx lifter = null;
    public static Servo GripIn = null;
    //    public Servo    GripOut   = null;
    public Servo Arm = null;
    public Servo drone = null;
    public static double drone_pos = 0;
    public static double Grip_pos = 0.878;
    public static double arm_pos = 0.4;
    public static double arm_pos_init = 0.3;
    public static double arm_init = 0.6;
    public static double GripInPos = 0.9;
    public static double droneInit = 0.4;

    public static double droneShoot = 0;
    double clawOffset = 0;

    public static final double MID_SERVO = 0.5;
    public static final double CLAW_SPEED = 0.02;        // sets rate to move servo
    public static final double ARM_UP_POWER = 0.50;   // Run arm motor u++++++++++++++++++++++++++++++++++++++++++++++++++++++++++p at 50% power
    public static final double ARM_DOWN_POWER = -0.50;   // Run arm motor down at -25% power

    /*
     * Code to run ONCE when the driver hits INIT
     */
    @Override
    public void runOpMode() throws InterruptedException {
        // Define and Initialize Motors
        leftDrive = hardwareMap.get(DcMotor.class, "left_drive");
        rightDrive = hardwareMap.get(DcMotor.class, "right_drive");
        lifter = hardwareMap.get(DcMotorEx.class, "lifter");
        Arm = hardwareMap.get(Servo.class, "Arm");
        int pos = 0;
//        drone = hardwareMap.get(Servo.class, "drone");


        // To drive forward, most robots need the motor on one side to be reversed, because the axles point in opposite directions.
        // Pushing the left and right sticks forward MUST make robot go forward. So adjust these two lines based on your first test drive.
        // Note: The settings here assume direct drive on left and right wheels.  Gear Reduction or 90 Deg drives may require direction flips
        leftDrive.setDirection(DcMotor.Direction.FORWARD);
        rightDrive.setDirection(DcMotor.Direction.REVERSE);
        lifter.setDirection(DcMotor.Direction.REVERSE);
        lifter.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        lifter.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        lifter.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);


        // If there are encoders connected, switch to RUN_USING_ENCODER mode for greater accuracy
        // leftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        // rightDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        // Define and initialize ALL installed servos.
        GripIn = hardwareMap.get(Servo.class, "GripL");
        drone = hardwareMap.get(Servo.class, "Drone");
//        GripIn.setPosition(MID_SERVO);
//        GripOut.setPosition(MID_SERVO);

        // Send telemetry message to signify robot waiting;
        telemetry.addData(">", "Robot Ready.  Press Play.");
        telemetry.update();

        //    /*
//     * Code to run REPEATEDLY after the driver hits INIT, but before they hit PLAY
//     */
//    @Override
//    public void init_loop() {
//    }
//
//    /*
//     * Code to run ONCE when the driver hits PLA
//     */
        while(opModeInInit()) {
            GripIn.setPosition(GripInPos);
            sleep(500);
            Arm.setPosition(arm_pos_init);
            drone.setPosition(droneInit);
        }
        waitForStart();
        while (opModeIsActive()) {
            double drive;
            double turn;



            // Run wheels in tank mode (note: The joystick goes negative when pushed forward, so negate it)
            drive = -gamepad1.left_stick_y;
            turn = gamepad1.right_stick_x;

            leftDrive.setPower(Range.clip( drive + turn, -1, 1));
            rightDrive.setPower(Range.clip(drive - turn, -1, 1));

            // Use gamepad left & right Bumpers to open and close the claw
            if (gamepad1.right_bumper) {

                GripIn.setPosition(GripInPos);

            } else if (gamepad1.a) {

                GripIn.setPosition(Grip_pos);
            } else if (gamepad1.b) {

                GripIn.setPosition(0.7);
            }
//        else if (gamepad1.b){
//
//            GripIn.setPosition(servo_pos);
//        }
            else if (gamepad1.dpad_right) {
                drone.setPosition(1);

              //  Arm.setPosition(0.65);
            }
            else if(gamepad1.x){
                Arm.setPosition(arm_pos);
            }
            else if(gamepad1.y){

                Arm.setPosition(arm_init);
            }
//        else if(gamepad1.left_bumper){
//            drone.setPosition(0.5);
//        }

            // Move both servos to new position.  Assume servos are mirror image of each other.
            clawOffset = Range.clip(clawOffset, -0.5, 0.5);


            // Use gamepad buttons to move the arm up (Y) and down (A)
            if (gamepad1.dpad_up) {
                pos = 3000;
                Arm.setPosition(arm_init);
            }

             if (gamepad1.dpad_down) {
                pos = 0;
            }
            if (gamepad1.left_trigger > 0.8) {
                pos += 25;
            }
            if (gamepad1.right_trigger > 0.8) {
                pos -= 25;
                if(lifter.getCurrentPosition() < 0){
                    lifter.setTargetPosition(0);
                    lifter.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    lifter.setPower(0.8);
                }
            }
            lifter.setTargetPosition(pos);
            lifter.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
            lifter.setPower(0.8);
            // Send telemetry message to signify robot running;
            telemetry.addData("claw", "Offset = %.2f", clawOffset);
            telemetry.addData("left", "%.2f", drive);
            telemetry.addData("right", "%.2f", turn);
            telemetry.addData("lifter current", lifter.getCurrent(CurrentUnit.AMPS));
            telemetry.addData("lifter_pos", lifter.getCurrentPosition());
            telemetry.update();
        }

    }
}
//motors
//0-right_drive
//1-left_drive
//2-lifter
//
//servos
//0-out
//1-in
//2-arm
