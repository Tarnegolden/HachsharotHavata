package org.firstinspires.ftc.teamcode.Systems;

import com.qualcomm.hardware.rev.Rev2mDistanceSensor;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.Utils.ArrayUtils;

public class DrivingSystem {
    private final LinearOpMode opMode;
    private final DcMotor fr, br, fl, bl;
    private final Rev2mDistanceSensor low, high;

    private final double encoderResolution = 2786.2;
    private final double wheelR = 0.048;


    public DrivingSystem(LinearOpMode opMode) {
        this.opMode = opMode;
        low = opMode.hardwareMap.get(Rev2mDistanceSensor.class, "low");
        high = opMode.hardwareMap.get(Rev2mDistanceSensor.class, "high");

        fr = opMode.hardwareMap.get(DcMotor.class, "front_right");
        br = opMode.hardwareMap.get(DcMotor.class, "back_right");
        fl = opMode.hardwareMap.get(DcMotor.class, "front_left");
        bl = opMode.hardwareMap.get(DcMotor.class, "back_left");

        fr.setDirection(DcMotorSimple.Direction.REVERSE);
        bl.setDirection(DcMotorSimple.Direction.REVERSE);

        fr.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        br.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        fl.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        bl.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    public void driveByJoystick(double x1, double y1, double x2) {
        double frPower = y1 - x1 - x2;
        double brPower = y1 + x1 - x2;
        double flPower = y1 + x1 + x2;
        double blPower = y1 - x1 + x2;

        if (Math.abs(frPower) > 1 || Math.abs(brPower) > 1 || Math.abs(flPower) > 1 || Math.abs(blPower) > 1) {
            double magnitude = Math.max(
                    Math.max(Math.abs(frPower), Math.abs(brPower)),
                    Math.max(Math.abs(flPower), Math.abs(blPower))
            );

            frPower /= magnitude;
            brPower /= magnitude;
            flPower /= magnitude;
            blPower /= magnitude;
        }

        fr.setPower(frPower);
        br.setPower(brPower);
        fl.setPower(flPower);
        bl.setPower(blPower);
    }

    public void cappingSequence() {
        resetEncoders();

        double[] distanceSensorLow = new double[20];
        double[] distanceSensorHigh = new double[20];
        double[] distanceTraveled = new double[20];

        int i = 0;
        double time0 = opMode.getRuntime();
        double time = opMode.getRuntime();
        while (opMode.opModeIsActive() && opMode.getRuntime() > time0 + 10) {
            driveByJoystick(0, 0.3, 0);
            if (opMode.getRuntime() >= time + 0.5) {
                time = opMode.getRuntime();
                distanceSensorLow[i] = low.getDistance(DistanceUnit.METER);
                distanceSensorHigh[i] = high.getDistance(DistanceUnit.METER);
                distanceTraveled[i] = ((double) br.getCurrentPosition() + (double) fl.getCurrentPosition()
                                        - (double) fr.getCurrentPosition() - (double) bl.getCurrentPosition()) / 4;
                i++;
            }
        }

        double[] lowDistanceIndex = ArrayUtils.minIndex(distanceSensorLow);
        double[] highDistanceIndex = ArrayUtils.minIndex(distanceSensorHigh);

        // continue next time
    }

    public void resetEncoders() {
        fr.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        br.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        fl.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        bl.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        fr.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        br.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        fl.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        bl.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }
}
