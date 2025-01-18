package frc.robot.subsystems;

import java.io.ObjectInputFilter.Config;

import com.ctre.phoenix.sensors.WPI_PigeonIMU;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.pathplanner.lib.util.PathPlannerLogging;

import edu.wpi.first.hal.FRCNetComm.tResourceType;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.DriveConstants;

//This is where we intialize all the swerve wheels
//This order matters
public class SwerveSubsystem extends SubsystemBase {
    private final SwerveModule frontLeft = new SwerveModule(
            DriveConstants.kFrontLeftDriveMotorPort,
            DriveConstants.kFrontLeftTurningMotorPort,
            DriveConstants.kFrontLeftDriveAbsoluteEncoderPort,
            DriveConstants.kFrontLeftDriveAbsoluteEncoderOffsetRad,
            DriveConstants.kFrontLeftDriveAbsoluteEncoderReversed, 
            DriveConstants.kFrontLeftChassisAngularOffset);

    private final SwerveModule frontRight = new SwerveModule(
            DriveConstants.kFrontRightDriveMotorPort,
            DriveConstants.kFrontRightTurningMotorPort,
            DriveConstants.kFrontRightDriveAbsoluteEncoderPort,
            DriveConstants.kFrontRightDriveAbsoluteEncoderOffsetRad,
            DriveConstants.kFrontRightDriveAbsoluteEncoderReversed,
            DriveConstants.kFrontRightChassisAngularOffset);

    private final SwerveModule backLeft = new SwerveModule(
            DriveConstants.kBackLeftDriveMotorPort,
            DriveConstants.kBackLeftTurningMotorPort,
            DriveConstants.kBackLeftDriveAbsoluteEncoderPort,
            DriveConstants.kBackLeftDriveAbsoluteEncoderOffsetRad,
            DriveConstants.kBackLeftDriveAbsoluteEncoderReversed, 
            DriveConstants.kBackLeftChassisAngularOffset);

    private final SwerveModule backRight = new SwerveModule(
            DriveConstants.kBackRightDriveMotorPort,
            DriveConstants.kBackRightTurningMotorPort,
            DriveConstants.kBackRightDriveAbsoluteEncoderPort,
            DriveConstants.kBackRightDriveAbsoluteEncoderOffsetRad,
            DriveConstants.kBackRightDriveAbsoluteEncoderReversed, 
            DriveConstants.kBackRightChassisAngularOffset);

    //This is the gyro
    private final WPI_PigeonIMU gyro = new WPI_PigeonIMU(13);
    //Field2d is a map on Dashbaord that  shows the robots current pose
    private final Field2d m_field = new Field2d();
    //Here we intalize the odometer
    //odometry uses sensors to update the robots pose
    private SwerveDriveOdometry odometer = new SwerveDriveOdometry(
        DriveConstants.kDriveKinematics, getRotation2d(),
        new SwerveModulePosition[] {
        frontLeft.getPosition(),
        backLeft.getPosition(),
        frontRight.getPosition(),
        backRight.getPosition()
    }, new Pose2d(15.06, 7.38, new Rotation2d()));

    //Pose estimator uses vision and odometry to estamiate the pose
    //We may be able to replace swerveDriveOdomatery with this if it works.
    private final SwerveDrivePoseEstimator poseEstimator =
      new SwerveDrivePoseEstimator(
          DriveConstants.kDriveKinematics,
          gyro.getRotation2d(),
          new SwerveModulePosition[] {
            frontLeft.getPosition(),
            frontRight.getPosition(),
            backLeft.getPosition(),
            backRight.getPosition()
          },
          new Pose2d(),
          VecBuilder.fill(0.05, 0.05, Units.degreesToRadians(5)),
          VecBuilder.fill(0.5, 0.5, Units.degreesToRadians(30)));

    public SwerveSubsystem() {
        new Thread(() -> {
            try {
                Thread.sleep(1000);
                zeroHeading();
                resetOdometry(getPose());
            } catch (Exception e) { System.out.println("Exception in auto: 'SwerveSubsystem()'");
            }
        }).start();

        try{
          RobotConfig config = RobotConfig.fromGUISettings();

        //The AutoBuilder is used by pathPlanner to run Autos
        AutoBuilder.configure(
            this::getPose, 
            this::resetOdometry, 
            this::getSpeeds, 
            (speeds, feedforwards) -> driveRobotRelative(speeds),
            new PPHolonomicDriveController(
              new PIDConstants(5, 0, 0), 
              new PIDConstants(5, 0, 0)
              ), 
            config, 
            () -> {
              // Boolean supplier that controls when the path will be mirrored for the red alliance
              // This will flip the path being followed to the red side of the field.
              // THE ORIGIN WILL REMAIN ON THE BLUE SIDE

              var alliance = DriverStation.getAlliance();
              if (alliance.isPresent()) {
                return alliance.get() == DriverStation.Alliance.Red;
              }
              return false;
            },
            this // Reference to this subsystem to set requirements
            );
            }catch(Exception e){
      DriverStation.reportError("Failed to load PathPlanner config and configure AutoBuilder", e.getStackTrace());
    }


    PathPlannerLogging.setLogCurrentPoseCallback((pose) -> {
      // Do whatever you want with the pose here
      //m_field.setRobotPose(pose);
  });

  // Logging callback for target robot pose
  PathPlannerLogging.setLogTargetPoseCallback((pose) -> {
      // Do whatever you want with the pose here
      //m_field.getObject("target pose").setPose(pose);
  });

  // Logging callback for the active path, this is sent as a list of poses
  PathPlannerLogging.setLogActivePathCallback((poses) -> {
      // Do whatever you want with the poses here
      //m_field.getObject("path").setPoses(poses);
  });
    }

    //This gives use the speed of the chassis
    public ChassisSpeeds getChassisSpeed() {
        return DriveConstants.kDriveKinematics.toChassisSpeeds(frontLeft.getState(), backLeft.getState(),
            frontRight.getState(),
            backRight.getState());
      }

      //This will reset the robots current heading
    public void zeroHeading() {
        gyro.reset();
    }

    //This reset the robots heading, but as a command
    public Command zeroHeadingCommand() {
        return this.runOnce(() -> this.gyro.reset());
    }

    //Gets tyeh robtos current heading
    public double getHeading() {
        return Math.IEEEremainder(gyro.getAngle(), 360);
    }

    //Returns the robots rotation in a 2D plane
    public Rotation2d getRotation2d() {
        return gyro.getRotation2d();
    }

    //Get the robots current position of the field
    public Pose2d getPose() {
        return odometer.getPoseMeters();
    }

    //Get the robots current pose in the auto period
    public Pose2d getAutoPose() {
        updateOdometry();
        return odometer.getPoseMeters();
      }

      //Gets the swrve wheels state as an array
      //Sates measures the desired angle and speed of each wheel
      public SwerveModuleState[] getModuleStates() {
        SwerveModuleState[] states = new SwerveModuleState[4];
        states[0] = frontLeft.getState();
        states[1] = backLeft.getState();
        states [2] = frontRight.getState();
        states[3] = backRight.getState();
        return states;
      }

      //Gets the speed of the chassis
      //We have two, this one may not be necessary
      public ChassisSpeeds getSpeeds() {
        return DriveConstants.kDriveKinematics.toChassisSpeeds(getModuleStates());
      }

    //reset the robots odometry
    public void resetOdometry(Pose2d pose) {
        odometer.resetPosition(getRotation2d(), new SwerveModulePosition[] {
            frontLeft.getPosition(),
            backLeft.getPosition(),
            frontRight.getPosition(),
            backRight.getPosition()
        }, pose //new Rotation2d()
        );
    }

    public SwerveModulePosition [] getPosition(){
        return new SwerveModulePosition[] {frontLeft.getPosition(), backLeft.getPosition(), frontRight.getPosition(),
            backRight.getPosition()};
    }

    public void driveFieldRelative(ChassisSpeeds fieldRelativeSpeeds) {
        driveRobotRelative(ChassisSpeeds.fromFieldRelativeSpeeds(fieldRelativeSpeeds, getPose().getRotation()));
      }
 
    public void driveRobotRelative(ChassisSpeeds robotRelativeSpeeds){
        ChassisSpeeds targetSpeeds = ChassisSpeeds.discretize(robotRelativeSpeeds, 0.02);

        SwerveModuleState[] targetStates = DriveConstants.kDriveKinematics.toSwerveModuleStates(targetSpeeds);
        setModuleStates(targetStates);
    }
    
     public void updateOdometry() {
      poseEstimator.update(
        gyro.getRotation2d(),
        new SwerveModulePosition[] {
          frontLeft.getPosition(),
          frontRight.getPosition(),
          backLeft.getPosition(),
          backRight.getPosition()
        });
  }


    

    @Override
    public void periodic() {
        updateOdometry();
        SmartDashboard.putNumber("Robot Heading", getHeading());
        SmartDashboard.putNumber("FLposition", frontLeft.getAbsoluteEncoderRad());
        SmartDashboard.putNumber("BLposition", backLeft.getAbsoluteEncoderRad());
        SmartDashboard.putNumber("FRposition", frontRight.getAbsoluteEncoderRad());
        SmartDashboard.putNumber("BRposition", backRight.getAbsoluteEncoderRad());
        SmartDashboard.putNumber("AbsoluteEnc Voltage FL", frontLeft.getEncoder().getVoltage());
        SmartDashboard.putNumber("AbsoluteEnc Voltage BL", backLeft.getEncoder().getVoltage());
        SmartDashboard.putNumber("AbsoluteEnc Voltage FR", frontRight.getEncoder().getVoltage());
        SmartDashboard.putNumber("AbsoluteEnc Voltage BR", backRight.getEncoder().getVoltage());
        SmartDashboard.putNumber("Encoder position FL", frontLeft.getTurningPosition());
        SmartDashboard.putNumber("Encoder position FR", frontRight.getTurningPosition());
        SmartDashboard.putNumber("Encoder position BL", backLeft.getTurningPosition());
        SmartDashboard.putNumber("Encoder position BR", backRight.getTurningPosition());
        SmartDashboard.putData("Field", m_field);
        m_field.setRobotPose(poseEstimator.getEstimatedPosition());
        //updatePose();

    }

    public void stopModules() {
        frontLeft.stop();
        backLeft.stop();
        frontRight.stop();
        backRight.stop();
    }

    public void setModuleStates(SwerveModuleState[] desiredStates) {
        SwerveDriveKinematics.desaturateWheelSpeeds(desiredStates, DriveConstants.kPhysicalMaxSpeedMetersPerSecond);
        frontLeft.setDesiredState(desiredStates[0]);
        backLeft.setDesiredState(desiredStates[1]);
        frontRight.setDesiredState(desiredStates[2]);
        backRight.setDesiredState(desiredStates[3]);
    }
}