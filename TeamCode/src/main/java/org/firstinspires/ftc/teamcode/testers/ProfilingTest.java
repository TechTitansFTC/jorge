package org.firstinspires.ftc.teamcode.testers;

//import com.acmerobotics.dashboard.FtcDashboard;
//import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.Constants;
import org.firstinspires.ftc.teamcode.ShooterProfiling.InterpLUT;
import org.firstinspires.ftc.teamcode.ShooterProfiling.ShooterPair;
import org.firstinspires.ftc.teamcode.subsystems.Indexer.Elevator;
import org.firstinspires.ftc.teamcode.subsystems.Indexer.Spindex;
import org.firstinspires.ftc.teamcode.subsystems.Intake.Bootwheel;
import org.firstinspires.ftc.teamcode.subsystems.Outtake.Hood;
import org.firstinspires.ftc.teamcode.subsystems.Outtake.Shooter;

@TeleOp(name = "Profiling Tester", group = "tuners")
public class ProfilingTest extends LinearOpMode {
    private Hood hood;
    private Shooter shooter;
    private Bootwheel intake;
    private Elevator elevator;
    private Spindex spindex;

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
        elevator = new Elevator(hardwareMap);
        spindex = new Spindex(hardwareMap);

        double hoodTarget = 1.0;
        double targetVel = 0;
        double curDistance = 0;

        InterpLUT table = new InterpLUT();
        table.addPoint(24, 900, 0.144);
        table.addPoint(55, 1200, 0.64);
        table.addPoint(76, 1300, 0.66);
        table.addPoint(79, 1200, 0.54);
        table.addPoint(103, 1300, .692);
        table.addPoint(130, 1500, .718);
        table.addPoint(140, 1700, .71);


//        MultipleTelemetry telemetryData = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        intake.setPower(1);

        waitForStart();

        if (isStopRequested()) return;

        while (opModeIsActive()) {
            shooter.setTargetVel(targetVel);
            hood.setPosition(hoodTarget);

            intake.update();
            shooter.update();
            hood.update();
            elevator.update();
            spindex.update();

            if (gamepad1.dpadUpWasPressed()) {
                curDistance += 5;
            } else if (gamepad1.dpadRightWasPressed()) {
                curDistance += 1;
            } else if (gamepad1.dpadDownWasPressed()) {
                curDistance -= 5;
            } else if (gamepad1.dpadLeftWasPressed()) {
                curDistance -= 1;
            }

            if (gamepad2.dpadDownWasPressed()) {
                elevator.setStatus(Constants.ElevatorPosition.DOWN);
            } else if (gamepad2.dpadUpWasPressed()) {
                elevator.setStatus(Constants.ElevatorPosition.UP);
            }

            if (gamepad2.squareWasPressed()) {
                spindex.setPosition(Constants.Positions.LEFT);
            } else if (gamepad2.triangleWasPressed()) {
                spindex.setPosition(Constants.Positions.CENTER);
            } else if (gamepad2.circleWasPressed()) {
                spindex.setPosition(Constants.Positions.RIGHT);
            }

            if (gamepad1.leftBumperWasPressed()) {
                intake.setPower(0);
            } else if (gamepad1.rightBumperWasPressed()) {
                intake.setPower(1);
            }

            if (shooter.atVelocity()) {
                gamepad1.rumble(10);
            }

            ShooterPair target = table.lookup(curDistance);
            if (target != null) {
                targetVel = target.getVel();
                hoodTarget = target.getHood();
            }

//            telemetryData.addData("target Vel", targetVel);
//            telemetryData.addData("current Vel", shooter.getCurrentVel());
//            telemetryData.addData("hood target", hoodTarget);
//            telemetryData.addData("distance from goal", curDistance);
//            telemetryData.addLine();
//            telemetryData.addData("ready to shoot", shooter.atVelocity());
//            telemetryData.update();

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
