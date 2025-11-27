package org.firstinspires.ftc.teamcode.testers;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.subsystems.Indexer.Transfer;
import org.firstinspires.ftc.teamcode.subsystems.Intake.Bootwheel;
import org.firstinspires.ftc.teamcode.subsystems.Outtake.Hood;
import org.firstinspires.ftc.teamcode.subsystems.Outtake.Shooter;

@TeleOp(name = "Outtake Tester", group = "tuners")
public class OuttakeTest extends LinearOpMode {
    private Hood hood;
    private Shooter shooter;
    private Bootwheel intake;
    private Transfer transfer;

    @Override
    public void runOpMode() throws InterruptedException {
        DcMotor frontLeft = hardwareMap.dcMotor.get("fl");
        DcMotor backLeft = hardwareMap.dcMotor.get("bl");
        DcMotor frontRight = hardwareMap.dcMotor.get("fr");
        DcMotor backRight = hardwareMap.dcMotor.get("br");

        frontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        frontLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        backLeft.setDirection(DcMotorSimple.Direction.REVERSE);

        GoBildaPinpointDriver pp = hardwareMap.get(GoBildaPinpointDriver.class, "pinpoint");
        pp.setOffsets(127, 101.6, DistanceUnit.MM);
        pp.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);
        pp.setEncoderDirections(GoBildaPinpointDriver.EncoderDirection.REVERSED, GoBildaPinpointDriver.EncoderDirection.FORWARD);
        pp.resetPosAndIMU();

        shooter = new Shooter(hardwareMap);
        hood = new Hood(hardwareMap);
        intake = new Bootwheel(hardwareMap);
        transfer = new Transfer(hardwareMap);

        double hoodTarget = 1.0;
        double targetVel = 0;
        double curDistance = 0;

        MultipleTelemetry telemetryData = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        intake.setPower(1);

        waitForStart();

        transfer.init();

        if (isStopRequested()) return;

        while (opModeIsActive()) {
            shooter.setTargetVel(targetVel);
            hood.setPosition(hoodTarget);

            intake.update();
            shooter.update();
            hood.update();
            transfer.update();


            if (gamepad1.triangleWasPressed()) {
                targetVel += 500;
            } else if (gamepad1.circleWasPressed()) {
                targetVel += 100;
            } else if (gamepad1.crossWasPressed()) {
                targetVel -= 500;
            } else if (gamepad1.squareWasPressed()) {
                targetVel -= 100;
            }

            if (gamepad1.dpadUpWasPressed()) {
                hoodTarget += 0.01;
            } else if (gamepad1.dpadRightWasPressed()) {
                hoodTarget += 0.002;
            } else if (gamepad1.dpadDownWasPressed()) {
                hoodTarget -= 0.01;
            } else if (gamepad1.dpadLeftWasPressed()) {
                hoodTarget -= 0.002;
            }

            if (gamepad2.circleWasPressed()) {
                transfer.resetFlags();
                transfer.requestShoot();
            }


            if (gamepad1.leftBumperWasPressed()) {
                intake.setPower(0);
            } else if (gamepad1.rightBumperWasPressed()) {
                intake.setPower(1);
            }

            if (shooter.atVelocity()) {
                transfer.requestReady();
            }

            curDistance = Math.sqrt(Math.pow(Math.abs(pp.getPosX(DistanceUnit.INCH)), 2) + Math.pow(Math.abs(pp.getPosY(DistanceUnit.INCH)), 2));

            telemetryData.addData("target Vel", targetVel);
            telemetryData.addData("current Vel", shooter.getCurrentVel());
            telemetryData.addData("hood target", hoodTarget);
            telemetryData.addData("distance from goal", curDistance);
            telemetryData.addLine();
            telemetryData.addData("ready to shoot", shooter.atVelocity());
            telemetryData.addLine();
            telemetryData.addData("", transfer);
            telemetryData.update();

            double y = -gamepad1.left_stick_y; // Remember, Y stick value is reversed
            double x = gamepad1.left_stick_x * 1.1; // Counteract imperfect strafing
            double rx = gamepad1.right_stick_x;

            double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);
            double frontLeftPower = (y + x + rx) / denominator;
            double backLeftPower = (y - x + rx) / denominator;
            double frontRightPower = (y - x - rx) / denominator;
            double backRightPower = (y + x - rx) / denominator;

            frontLeft.setPower(frontLeftPower);
            backLeft.setPower(backLeftPower);
            frontRight.setPower(frontRightPower);
            backRight.setPower(backRightPower);
        }
    }
}
