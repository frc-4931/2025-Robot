package frc.robot;

import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import swervelib.math.Matter;

public final class Constants {
    public static final double ROBOT_MASS = (148 - 20.3) * 0.453592;
    public static final Matter CHASSIS = new Matter(new Translation3d(0, 0, Units.inchesToMeters(8)), ROBOT_MASS);
    public static final double LOOP_TIME = 0.13;
    public static final double speedMultiplier = 0.3;
    ;
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

    public static final class ArmConstants {
        public static final int ARM_MOTOR_ID = 11;
        public static final int ARM_MOTOR_CURRENT_LIMIT = 60;
        public static final double ARM_MOTOR_VOLTAGE_COMP = 10;
        public static final double ARM_SPEED_DOWN = 0.13;
        public static final double ARM_SPEED_UP = -0.18;
        public static final double ARM_HOLD_DOWN = 0.01;
        public static final double ARM_HOLD_UP = -0.04;
      }

      public static final class RollerConstants {
        public static final int ROLLER_MOTOR_ID = 12;
        public static final int ROLLER_MOTOR_CURRENT_LIMIT = 60;
        public static final double ROLLER_MOTOR_VOLTAGE_COMP = 10;
        public static final double ROLLER_CORAL_OUT = -.25;
        public static final double ROLLER_ALGAE_IN = -0.4;
        public static final double ROLLER_ALGAE_OUT = 0.4;
        public static final double ROLLER_CORAL_STACK = -0.8;
        public static final double ROLLER_CORAL_ALGAE = -0.8;
      }

      public static final class ClimberConstants {
        public static final int CLIMBER_MOTOR_ID = 10;
        public static final int CLIMBER_MOTOR_CURRENT_LIMIT = 60;
        public static final double CLIMBER_MOTOR_VOLTAGE_COMP = 12;
        public static final double CLIMBER_SPEED_DOWN = -0.5;
        public static final double CLIMBER_SPEED_UP = 0.5;
      }
    
}
