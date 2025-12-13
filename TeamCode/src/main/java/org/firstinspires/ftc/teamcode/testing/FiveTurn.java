package org.firstinspires.ftc.teamcode.testing;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name = "5 servo test", group = "hardware")
public class FiveTurn extends LinearOpMode {

    private Servo servo1;

    private double servo1Pos = 0.5;

    private boolean servo1Reversed = false;
    private boolean yPressedLast = false;


    private final double STEP_LARGE = 5 / 1800.0;
    private final double STEP_SMALL = 10 / 1800.0;

    @Override
    public void runOpMode() throws InterruptedException {

        servo1 = hardwareMap.get(Servo.class, "servo1");

        servo1.setPosition(servo1Pos);

        waitForStart();

        while (opModeIsActive()) {

            if (gamepad1.y && !yPressedLast) {
                servo1Reversed = !servo1Reversed;
            }
            yPressedLast = gamepad1.y;

            double step = 0.0;

            if (gamepad1.dpad_up) {
                step = STEP_LARGE;
            } else if (gamepad1.dpad_down) {
                step = -STEP_LARGE;
            }

            if (gamepad1.dpad_right) {
                step = STEP_SMALL;
            } else if (gamepad1.dpad_left) {
                step = -STEP_SMALL;
            }

            if (step != 0.0) {
                // Servo 1
                if (servo1Reversed) {
                    servo1Pos -= step;
                } else {
                    servo1Pos += step;
                }

                servo1.setPosition(servo1Pos);

                sleep(100); // debounce
            }


            telemetry.addLine("Two Axon Servo Test");
            telemetry.addData("Servo1 Dir", servo1Reversed ? "REVERSED" : "FORWARD");
            telemetry.addData("Servo1 Pos", "%.4f", servo1Pos);
            telemetry.update();
        }
    }
}