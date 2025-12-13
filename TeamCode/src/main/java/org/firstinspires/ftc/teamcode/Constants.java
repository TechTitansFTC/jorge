package org.firstinspires.ftc.teamcode;

public class Constants {
    public enum ElevatorPosition {
        UP (ELEVATOR_LEFT_UP, ELEVATOR_RIGHT_UP),
        DOWN (ELEVATOR_LEFT_DOWN, ELEVATOR_RIGHT_DOWN);

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
        CENTER(SPINDEXER_TWO),
        LEFT(SPINDEXER_ONE),
        RIGHT(SPINDEXER_THREE);

        private final double position;
        Positions(double position) {
            this.position = position;
        }
        public double getPosition() {
            return this.position;
        }
    }

    //Elevator
    public static final double ELEVATOR_LEFT_UP = 214.0/355;
    public static final double ELEVATOR_LEFT_DOWN = 324.0/355;
    public static final double ELEVATOR_RIGHT_UP = 205.0/355;
    public static final double ELEVATOR_RIGHT_DOWN = 324.0/355;

    //Spindexer
    public static final double SPINDEXER_THREE = 265.0 / 355;
    public static final double SPINDEXER_TWO = 140.0 / 355;
    public static final double SPINDEXER_ONE = 0.0 / 355;
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

