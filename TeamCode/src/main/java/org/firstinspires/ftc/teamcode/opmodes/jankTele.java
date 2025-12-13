package org.firstinspires.ftc.teamcode.opmodes;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.ShooterProfiling.InterpLUT;
import org.firstinspires.ftc.teamcode.ShooterProfiling.ShooterPair;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.subsystems.Indexer.Transfer;
import org.firstinspires.ftc.teamcode.subsystems.Intake.Bootwheel;
import org.firstinspires.ftc.teamcode.subsystems.Outtake.Hood;
import org.firstinspires.ftc.teamcode.subsystems.Outtake.Shooter;

@TeleOp(name = "jank Tele", group = ".")
public class jankTele extends LinearOpMode {

    private double curDistance;
    private Bootwheel intake;
    private Transfer transfer;
    private Shooter flywheel;
    private Hood hood;
    private InterpLUT table;
    private ShooterPair settings;
    private Pose pose;
    public static Pose target = new Pose(-52, 52, Math.toRadians(135));
    private Follower drive;


    @Override
    public void runOpMode() throws InterruptedException {
        intake = new Bootwheel(hardwareMap);
        transfer = new Transfer(hardwareMap);
        flywheel = new Shooter(hardwareMap);
        hood = new Hood(hardwareMap);
        drive = Constants.createFollower(hardwareMap);
        drive.setStartingPose(new Pose(0,0,Math.toRadians(0)));
        drive.update();

        transfer.init();

        table = new InterpLUT();
        table.addPoint(24, 1000, 0.24);
        table.addPoint(30, 1000, 0.24);
        table.addPoint(45, 1000, 0.33);
        table.addPoint(60, 1100, 0.28);
        table.addPoint(70, 1100, 0.33);

        boolean shooting = false;

        waitForStart();

        drive.startTeleOpDrive(true);

        if (isStopRequested()) return;

        while (opModeIsActive()) {
            drive.update();
            drive.updatePose();

            pose = drive.getPose();
            double deltaX = Math.abs(pose.getX() - target.getX());
            double deltaY = Math.abs(pose.getY() - target.getY());
            curDistance = Math.sqrt(Math.pow(deltaX, 2) + Math.pow(deltaY, 2));

            intake.update();
            transfer.update();
            flywheel.update();
            hood.update();

            drive.setTeleOpDrive(
                    -gamepad1.left_stick_y,
                    -gamepad1.left_stick_x,
                    -gamepad1.right_stick_x,
                    false
            );

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

            if (gamepad1.optionsWasPressed()) {
                drive.setPose(new Pose(129.5, 129.5, Math.toRadians(38)));
                target = drive.getPose();
            } else if (gamepad1.shareWasPressed()) {
                drive.setPose(new Pose(21.5, 129.5, Math.toRadians(143)));
                target = drive.getPose();
            }



            if (shooting) {
                if (setSettings()) {
                    if (settings != null) {
                        flywheel.setTargetVel(settings.getVel());
                        hood.setPosition(settings.getHood());
                        telemetry.addData("LUT Status", "OK");
                    } else {
                        telemetry.addData("LUT Status", "FAIL - Distance out of bounds");
                    }
                } else {
                    flywheel.setTargetVel(settings.getVel());
                    hood.setPosition(settings.getHood());
                    telemetry.addData("LUT", "FAR");
                }

                if (flywheel.atVelocity()) {
                    transfer.requestReady();
                }

                if (!transfer.isShooting()) {
                    shooting = false;
                }

//                telemetry.addData("Calculated Distance", curDistance);
                telemetry.addData("Target Vel", flywheel.getTargetVel());
                telemetry.addData("Current Vel", flywheel.getCurrentVel());
                telemetry.addData("At Velocity?", flywheel.atVelocity());

            } else {
                flywheel.setTargetVel(1300);
                hood.setPosition(0.5);
            }

            Pose currentPose = drive.getPose();
            telemetry.addData("Robot Pose X", currentPose.getX());
            telemetry.addData("Robot Pose Y", currentPose.getY());
            telemetry.addData("Shooting Mode", shooting);
            telemetry.addData("Transfer State", transfer.toString());
            telemetry.addData("Cur Distance", curDistance);
            telemetry.update();
        }
    }

    private boolean setSettings() {

        if (curDistance > 70) {
            settings = new ShooterPair(1400, 0.536);
            return false;
        }

        ShooterPair settings = table.lookup(curDistance);
        return true;
    }
}
