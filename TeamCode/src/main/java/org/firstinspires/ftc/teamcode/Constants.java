package org.firstinspires.ftc.teamcode;

public class Constants {
    public enum ElevatorPosition {
        UP (214/355.0, 214/355.0),
        DOWN (324/355.0, 324/355.0);

        private final double left, right;
        ElevatorPosition (double left, double right) {
            this.left = left;
            this.right = right;
        }

        public double getLeft() {
            return this.left;
        }

        public double getRight() {
            return this.right;
        }
    }
    public enum Positions {
        CENTER(140.0 / 355.0),
        LEFT(1 / 355.0),
        RIGHT(267.0 / 355.0);

        private final double position;
        Positions(double position) {
            this.position = position;
        }
        public double getPosition() {
            return this.position;
        }
    }

    //Elevator
    public static final double ELEVATOR_LEFT_UP = 230/355;
    public static final double ELEVATOR_LEFT_DOWN = 345/355;
    public static final double ELEVATOR_RIGHT_UP = 232/355;
    public static final double ELEVATOR_RIGHT_DOWN = 345/355;

    //Spindexer
    public static final double SPINDEXER_THREE = 270/355;
    public static final double SPINDEXER_TWO = 130/355;
    public static final double SPINDEXER_ONE = 0/355;
    public static final double SPINDEXER_INTAKE = 65/355;



    //Kicker
    public static final double KICKER_OUT = 344/355;
    public static final double KICKER_IN = 260/355;

    //Intake
    public static final double INTAKE_OFF = 0.0;
    public static final double INTAKE_ON = 1.0;
    public static final double INTAKE_REVERSE = -1.0;

    //Shooter
    public static final double SHOOTER_ON = 1.0;
    public static final double SHOOTER_0FF = 0.0;

    //Turret
    public static final double TURRET_INITAL = 75/355;
}

