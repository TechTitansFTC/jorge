package org.firstinspires.ftc.teamcode.testing;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name = "two servo test", group = "hardware")
public class SERVOTEST extends LinearOpMode {

    private Servo servo1;
    private Servo servo2;

    private double servo1Pos = 1.0;
    private double servo2Pos = 1.0;

    private boolean servo1Reversed = false;
    private boolean servo2Reversed = false;

    private boolean yPressedLast = false;
    private boolean xPressedLast = false;



    private final double STEP_LARGE = 5 / 355.0;
    private final double STEP_SMALL = 10 / 355.0;

    @Override
    public void runOpMode() throws InterruptedException {

        servo1 = hardwareMap.get(Servo.class, "servo1");
        servo2 = hardwareMap.get(Servo.class, "servo2");

        servo1.setPosition(servo1Pos);
        servo2.setPosition(servo2Pos);

        waitForStart();

        while (opModeIsActive()) {

            if (gamepad1.y && !yPressedLast) {
                servo1Reversed = !servo1Reversed;
            }
            yPressedLast = gamepad1.y;

            if (gamepad1.x && !xPressedLast) {
                servo2Reversed = !servo2Reversed;
            }
            xPressedLast = gamepad1.x;

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

                if (servo2Reversed) {
                    servo2Pos -= step;
                } else {
                    servo2Pos += step;
                }

                servo1.setPosition(servo1Pos);
                servo2.setPosition(servo2Pos);

                sleep(100); // debounce
            }


            telemetry.addLine("Two Axon Servo Test");
            telemetry.addData("Servo1 Dir", servo1Reversed ? "REVERSED" : "FORWARD");
            telemetry.addData("Servo1 Pos", "%.4f", servo1Pos);
            telemetry.addData("Servo2 Dir", servo2Reversed ? "REVERSED" : "FORWARD");
            telemetry.addData("Servo2 Pos", "%.4f", servo2Pos);
            telemetry.update();
        }
    }
}