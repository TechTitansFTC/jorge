package org.firstinspires.ftc.teamcode.opmodes;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.ShooterProfiling.InterpLUT;
import org.firstinspires.ftc.teamcode.ShooterProfiling.ShooterPair;
import org.firstinspires.ftc.teamcode.roadrunner.MecanumDrive;
import org.firstinspires.ftc.teamcode.subsystems.Indexer.Transfer;
import org.firstinspires.ftc.teamcode.subsystems.Intake.Bootwheel;
import org.firstinspires.ftc.teamcode.subsystems.Outtake.Hood;
import org.firstinspires.ftc.teamcode.subsystems.Outtake.Shooter;

@TeleOp(name = "jank Tele", group = ".")
public class jankTele extends LinearOpMode {

    private Bootwheel intake;
    private Transfer transfer;
    private Shooter flywheel;
    private Hood hood;
    private InterpLUT table;
    private ShooterPair settings;
    private Pose2d pose;
    public final static Pose2d target = new Pose2d(-52, 52, Math.toRadians(135));
    public static Pose2d init = new Pose2d(0, 0, 0);
    private MecanumDrive drive;


    @Override
    public void runOpMode() throws InterruptedException {
        intake = new Bootwheel(hardwareMap);
        transfer = new Transfer(hardwareMap);
        flywheel = new Shooter(hardwareMap);
        hood = new Hood(hardwareMap);
        drive = new MecanumDrive(hardwareMap, new Pose2d(0, 0, 0));

        transfer.init();

        table = new InterpLUT();
        table.addPoint(24, 900, 0.144);
        table.addPoint(55, 1200, 0.64);
        table.addPoint(76, 1300, 0.66);
        table.addPoint(79, 1200, 0.54);
        table.addPoint(103, 1300, .692);
        table.addPoint(130, 1500, .718);
        table.addPoint(140, 1700, .71);

        boolean shooting = false;

        MultipleTelemetry telemetryData = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        waitForStart();

        if (isStopRequested()) return;

        while (opModeIsActive()) {
            intake.update();
            transfer.update();
            flywheel.update();
            hood.update();

            drive.setDrivePowers(new PoseVelocity2d(
                    new Vector2d(
                            -0.5 * Math.tan(1.12 * gamepad1.left_stick_y),
                            -0.5 * Math.tan(1.12 * gamepad1.left_stick_x)
                    ),
                    -0.5 * Math.tan(1.12 * gamepad1.right_stick_x)
            ));

            if (gamepad1.leftBumperWasPressed()) {
                intake.setPower(0);
                transfer.resetFlags();
            }

            if (gamepad1.rightBumperWasPressed()) {
                intake.setPower(1);
            }

            if (gamepad1.circleWasPressed()) {
                shooting = true;
                transfer.requestShoot();
            }

            if (shooting) {
                setSettings();
                if (settings != null) {
                    flywheel.setTargetVel(settings.getVel());
                    hood.setPosition(settings.getHood());
                    telemetryData.addData("LUT Status", "OK");
                } else {
                    telemetryData.addData("LUT Status", "FAIL - Distance out of bounds");
                }

                if (flywheel.atVelocity()) {
                    transfer.requestReady();
                }

                if (!transfer.isShooting()) {
                    shooting = false;
                }

//                telemetryData.addData("Calculated Distance", curDistance);
                telemetryData.addData("Target Vel", flywheel.getTargetVel());
                telemetryData.addData("Current Vel", flywheel.getCurrentVel());
                telemetryData.addData("At Velocity?", flywheel.atVelocity());

            } else {
                flywheel.setTargetVel(1300);
                hood.setPosition(0.5);
            }

            Pose2d currentPose = drive.localizer.getPose();
            telemetryData.addData("Robot Pose X", currentPose.position.x);
            telemetryData.addData("Robot Pose Y", currentPose.position.y);
            telemetryData.addData("Shooting Mode", shooting);
            telemetryData.addData("Transfer State", transfer.toString());
            telemetryData.update();
        }
    }

    private void setSettings() {
        pose = drive.localizer.getPose();
        double deltaX = Math.abs(pose.position.x - target.position.x);
        double deltaY = Math.abs(pose.position.y - target.position.y);
        double curDistance = Math.sqrt(Math.pow(deltaX, 2) + Math.pow(deltaY, 2));

        settings = table.lookup(curDistance);
    }
}
