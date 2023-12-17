package org.firstinspires.ftc.teamcode.teleop;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.OpticalDistanceSensor;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
@TeleOp
public class distanceSensor extends LinearOpMode {
    AnalogInput ds = null;
    private static final double VOLTAGE_AT_MIN_DISTANCE = 0.25;  // Replace with your sensor's value
    private static final double VOLTAGE_AT_MAX_DISTANCE = 0.55;  // Replace with your sensor's value
    private static final double MIN_DISTANCE_CM = 4;  // Replace with your sensor's value
    private static final double MAX_DISTANCE_CM = 30;
    private static double convertVoltageToDistance(double voltage) {
        // Linear interpolation based on sensor characteristics
        double distance = ((voltage - VOLTAGE_AT_MIN_DISTANCE) / (VOLTAGE_AT_MAX_DISTANCE - VOLTAGE_AT_MIN_DISTANCE))
                * (MAX_DISTANCE_CM - MIN_DISTANCE_CM) + MIN_DISTANCE_CM;

        // Ensure the kdistance is within the valid range
        return Math.max(MIN_DISTANCE_CM, Math.min(MAX_DISTANCE_CM, distance));
    }

    public void runOpMode(){
        ds = hardwareMap.get(AnalogInput.class, "ds");

        waitForStart();

        while(opModeIsActive()){
            double distance = convertVoltageToDistance(ds.getVoltage());
            telemetry.addData("Distance : ", distance);
            telemetry.update();
        }
    }
}
