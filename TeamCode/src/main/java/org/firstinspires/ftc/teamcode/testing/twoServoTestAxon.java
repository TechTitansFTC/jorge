package org.firstinspires.ftc.teamcode.testing;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.util.wrappers.nServo;

@Disabled
@TeleOp(name = "two axon servo test", group = "hardware")
public class twoServoTestAxon extends LinearOpMode {

    private nServo servo1;
    private nServo servo2;
    private final double increment = 5.0 / 355.0;
    private double targetAngle = 0;

    @Override
    public void runOpMode() throws InterruptedException {

        servo1 = new nServo("servo1", hardwareMap, 0, Math.toRadians(355));
        servo2 = new nServo("servo2", hardwareMap, 0, Math.toRadians(355));

        waitForStart();

        while (opModeIsActive()) {

            if (gamepad1.dpad_up) {
                targetAngle += Math.toRadians(5.0 / 355.0 * 355);
                targetAngle = Math.min(targetAngle, servo1.getMaxRadians());
                servo1.setTargetAngle(targetAngle);
                servo2.setTargetAngle(targetAngle);
            }

            if (gamepad1.dpad_down) {
                targetAngle -= Math.toRadians(5.0 / 355.0 * 355);
                targetAngle = Math.max(targetAngle, 0);
                servo1.setTargetAngle(targetAngle);
                servo2.setTargetAngle(targetAngle);
            }

            servo1.update();
            servo2.update();

            telemetry.addData("Target Angle (rad)", targetAngle);
            telemetry.addData("Servo1 Pos", servo1.getCurrentPosition());
            telemetry.addData("Servo2 Pos", servo2.getCurrentPosition());
            telemetry.update();
        }
    }
}
