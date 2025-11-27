package org.firstinspires.ftc.teamcode.testing;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.util.wrappers.nServo;

@Disabled
@TeleOp(name = "axon servo test", group = "hardware")
public class servoTestAxon extends LinearOpMode {

    private nServo servo;
    private final double increment = 5.0 / 355.0;
    private double targetAngle = 0;

    @Override
    public void runOpMode() throws InterruptedException {


        servo = new nServo("Servo1", hardwareMap, 0, Math.toRadians(355));

        waitForStart();

        while (opModeIsActive()) {

            if (gamepad1.dpad_up) {
                targetAngle += Math.toRadians(5.0 / 355.0 * 355);
                targetAngle = Math.min(targetAngle, servo.getMaxRadians());
                servo.setTargetAngle(targetAngle);
            }

            if (gamepad1.dpad_down) {
                targetAngle -= Math.toRadians(5.0 / 355.0 * 355);
                targetAngle = Math.max(targetAngle, 0);
                servo.setTargetAngle(targetAngle);
            }

            servo.update();

            telemetry.addData("Target Angle (rad)", targetAngle);
            telemetry.addData("Servo Position", servo.getCurrentPosition());
            telemetry.update();

            sleep(100);
        }
    }
}
