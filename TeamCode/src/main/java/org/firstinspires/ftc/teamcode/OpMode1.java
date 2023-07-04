package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Systems.DrivingSystem;

@TeleOp(name="opmode1")
public class OpMode1 extends LinearOpMode {
    DrivingSystem drivingSystem;

    boolean previousA;

    @Override
    public void runOpMode() throws InterruptedException {
        drivingSystem = new DrivingSystem(this);

        waitForStart();

        while (opModeIsActive()) {
            drivingSystem.driveByJoystick(gamepad1.left_stick_x, -gamepad1.left_stick_y, gamepad1.right_stick_x);

            // if gamepad1.a got pressed
            if (!previousA && gamepad1.a) {
                //
            }



            previousA = gamepad1.a;
        }
    }
}
