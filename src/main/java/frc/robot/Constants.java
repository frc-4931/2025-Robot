package frc.robot;

import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.RobotBase;
import swervelib.math.Matter;

public final class Constants {
    public static final double ROBOT_MASS = (148 - 20.3) * 0.453592;
    public static final Matter CHASSIS = new Matter(new Translation3d(0, 0, Units.inchesToMeters(8)), ROBOT_MASS);
    public static final double LOOP_TIME = 0.13;
    public static final double speedMultiplier = 1;
    public static final double MAX_SPEED = Units.feetToMeters(14.5) * speedMultiplier;

    public static final class DrivebaseConstants{
        public static final double WHEEL_LOCK_TIME = 10;
    }

    public static class OperatorConstants{
        public static final double DEADBAND = 0.1;
        public static final double LEFT_Y_DEADBAND = 0.1;
        public static final double LEFT_X_DEADBAND = 0.1;
        public static final double TURN_CONSTANT = 6;

    }

    public static final Mode simMode = Mode.SIM;
  public static final Mode currentMode = RobotBase.isReal() ? Mode.REAL : simMode;

  public static enum Mode {
    /** Running on a real robot. */
    REAL,

    /** Running a physics simulator. */
    SIM,

    /** Replaying from a log file. */
    REPLAY

    }
}
