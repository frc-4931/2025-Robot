package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Meter;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.commands.PathfindingCommand;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.util.DriveFeedforwards;
import com.pathplanner.lib.util.swerve.SwerveSetpoint;
import com.pathplanner.lib.util.swerve.SwerveSetpointGenerator;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Config;
import frc.robot.Constants;
//import frc.robot.subsystems.swervedrive.Vision.Cameras;
import java.io.File;
import java.io.IOException;
import java.util.Arrays;
import java.util.Optional;
import java.util.concurrent.atomic.AtomicReference;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;
// import org.json.simple.parser.ParseException;
// import org.photonvision.targeting.PhotonPipelineResult;
import swervelib.SwerveController;
import swervelib.SwerveDrive;
import swervelib.SwerveDriveTest;
import swervelib.math.SwerveMath;
import swervelib.parser.SwerveControllerConfiguration;
import swervelib.parser.SwerveDriveConfiguration;
import swervelib.parser.SwerveParser;
import swervelib.telemetry.SwerveDriveTelemetry;
import swervelib.telemetry.SwerveDriveTelemetry.TelemetryVerbosity;

public class SwerveSubsystem extends SubsystemBase{

    private final SwerveDrive swerveDrive;

    public SwerveSubsystem(File directory){
        SwerveDriveTelemetry.verbosity = TelemetryVerbosity.HIGH;
        try{
            swerveDrive = new SwerveParser(directory).createSwerveDrive(Constants.MAX_SPEED, new Pose2d(new Translation2d(Meter.of(1), Meter.of(4)), Rotation2d.fromDegrees(0)));
        } catch (Exception e){
            throw new RuntimeException(e);
        }
        swerveDrive.setHeadingCorrection(false);
        swerveDrive.setCosineCompensator(false);
        swerveDrive.setAngularVelocityCompensation(true, true, 0.1);
        swerveDrive.setModuleEncoderAutoSynchronize(false, 1);

        setupPathPlanner();

    }

    public SwerveSubsystem(SwerveDriveConfiguration driveCfg, SwerveControllerConfiguration controllerCfg){
        swerveDrive = new SwerveDrive(driveCfg, controllerCfg, Constants.MAX_SPEED, new Pose2d(new Translation2d(Meter.of(2), Meter.of(0)), Rotation2d.fromDegrees(0)));
    }

    public void setupPathPlanner(){
        try{
            RobotConfig config = RobotConfig.fromGUISettings();

            final boolean enableFeedForward = true;

            AutoBuilder.configure(
                this::getPose, 
                this::resetOdometry, 
                this::getRobotVelocity, 
                (speedsRobotRelative, moduleFeedForwards) -> {
                    if(enableFeedForward){
                        swerveDrive.drive(
                            speedsRobotRelative,
                            swerveDrive.kinematics.toSwerveModuleStates(speedsRobotRelative),
                            moduleFeedForwards.linearForces()
                        );
                    } else{
                        swerveDrive.setChassisSpeeds(speedsRobotRelative);
                    }
                }, 
                //this::driveRobotRelative,
                new PPHolonomicDriveController(
                    new PIDConstants(5.0, 0, 0),
                    new PIDConstants(5.0, 0, 0)),
                config,
                () -> {
                        var alliance = DriverStation.getAlliance();
            if (alliance.isPresent()) {
                return alliance.get() == DriverStation.Alliance.Red;
            }
            return false;
        },
        this
      );
    }catch(Exception e){
      DriverStation.reportError("Failed to load PathPlanner config and configure AutoBuilder", e.getStackTrace());
    }
    }

    // Set up custom logging to add the current path to a field 2d widget
    // PathPlannerLogging.setLogActivePathCallback((poses) -> field.getObject("path").setPoses(poses));

    // SmartDashboard.putData("Field", field);

    // public void setupPathPlanner(){
    // // Load the RobotConfig from the GUI settings. You should probably
    // // store this in your Constants file
    // RobotConfig config;
    // try
    // {
    //   config = RobotConfig.fromGUISettings();

    //   final boolean enableFeedforward = true;
    //   // Configure AutoBuilder last
    //   AutoBuilder.configure(
    //       this::getPose,
    //       // Robot pose supplier
    //       this::resetOdometry,
    //       // Method to reset odometry (will be called if your auto has a starting pose)
    //       this::getRobotVelocity,
    //       // ChassisSpeeds supplier. MUST BE ROBOT RELATIVE
    //       (speedsRobotRelative, moduleFeedForwards) -> {
    //         if (enableFeedforward)
    //         {
    //           swerveDrive.drive(
    //               speedsRobotRelative,
    //               swerveDrive.kinematics.toSwerveModuleStates(speedsRobotRelative),
    //               moduleFeedForwards.linearForces()
    //                            );
    //         } else
    //         {
    //           swerveDrive.setChassisSpeeds(speedsRobotRelative);
    //         }
    //       },
    //       // Method that will drive the robot given ROBOT RELATIVE ChassisSpeeds. Also optionally outputs individual module feedforwards
    //       new PPHolonomicDriveController(
    //           // PPHolonomicController is the built in path following controller for holonomic drive trains
    //           new PIDConstants(5.0, 0.0, 0.0),
    //           // Translation PID constants
    //           new PIDConstants(5.0, 0.0, 0.0)
    //           // Rotation PID constants
    //       ),
    //       config,
    //       // The robot configuration
    //       () -> {
    //         // Boolean supplier that controls when the path will be mirrored for the red alliance
    //         // This will flip the path being followed to the red side of the field.
    //         // THE ORIGIN WILL REMAIN ON THE BLUE SIDE

    //         var alliance = DriverStation.getAlliance();
    //         if (alliance.isPresent())
    //         {
    //           return alliance.get() == DriverStation.Alliance.Red;
    //         }
    //         return false;
    //       },
    //       this
    //       // Reference to this subsystem to set requirements
    //                        );

    // } catch (Exception e)
    // {
    //   // Handle exception as needed
    //   e.printStackTrace();
    // }
    // }


    @Override
    public void periodic(){
    }

    public Command driveCommand(DoubleSupplier translationX, DoubleSupplier translationY, DoubleSupplier anglularRotationX){
        return run(() -> {
            swerveDrive.drive(SwerveMath.scaleTranslation(new Translation2d(translationX.getAsDouble() * swerveDrive.getMaximumChassisAngularVelocity(), translationY.getAsDouble() * swerveDrive.getMaximumChassisAngularVelocity()), .8), Math.pow(anglularRotationX.getAsDouble(), 3) * swerveDrive.getMaximumChassisAngularVelocity(), true, false);
        });
    }

    public Command driveCommand(DoubleSupplier translationX, DoubleSupplier translationY, DoubleSupplier headingX, DoubleSupplier headingY){
        return run (() -> {
            Translation2d scaledInputs = SwerveMath.scaleTranslation(new Translation2d( translationX.getAsDouble(), translationY.getAsDouble()), .8);

            driveFieldOriented(swerveDrive.swerveController.getTargetSpeeds(scaledInputs.getX(), scaledInputs.getY(), headingX.getAsDouble(), headingY.getAsDouble(), swerveDrive.getOdometryHeading().getRadians(), swerveDrive.getMaximumChassisVelocity()));
        });
    }

    public void drive(Translation2d translation, double rotation, boolean fieldRelative){
        swerveDrive.drive(translation, rotation, fieldRelative, false);
    }

    public void driveFieldOriented(ChassisSpeeds velocity){
        swerveDrive.driveFieldOriented(velocity);
    }

    public Command driveFieldOriented(Supplier<ChassisSpeeds> velocity){
        return run (() -> {
            swerveDrive.driveFieldOriented(velocity.get());
        });
    }

    public void drive(ChassisSpeeds velocity){
        swerveDrive.drive(velocity);
    }

    public SwerveDriveKinematics getKinematics(){
        return swerveDrive.kinematics;
    }

    public void resetOdometry(Pose2d initialHolonomicPose){
        swerveDrive.resetOdometry(initialHolonomicPose);
    }

    public Pose2d getPose(){
        return swerveDrive.getPose();
    }

    public void setChassisSpeeds(ChassisSpeeds chassisSpeeds){
        swerveDrive.setChassisSpeeds(chassisSpeeds);
    }

    public void postTrajectory(Trajectory trajectory){
        swerveDrive.postTrajectory(trajectory);
    }

    public void zeroGyro(){
        swerveDrive.zeroGyro();
    }

    // private boolean isRedAlliance(){
    //     return alliance.isPresent() ? alliance.get() == DriverStation.Alliance.Red : false;
    // }

    // public void zeroGyroWithAlliance()
    // {
    //     if (isRedAlliance()){
    //         zeroGyro();
    //         resetOdometry(new Pose2d(getPose().getTranslation(), Rotation2d.fromDegrees(180)));
    //     } else {
    //         zeroGyro();
    //     }
    // }
    public void setMotorBrake(boolean brake){
        swerveDrive.setMotorIdleMode(brake);
    }

    public Rotation2d getHeading(){
        return getPose().getRotation();
    }

    public ChassisSpeeds getTargetSpeeds(double xInput, double yInput, double headingX, double headingY){
        Translation2d scaledInputs = SwerveMath.cubeTranslation(new Translation2d(xInput, yInput));
        return swerveDrive.swerveController.getTargetSpeeds(scaledInputs.getX(), scaledInputs.getY(), headingX, headingY, getHeading().getRadians(), Constants.MAX_SPEED);
    }

    public ChassisSpeeds getTargetSpeeds(double xInput, double yInput, Rotation2d angle){
        Translation2d scaledInputs = SwerveMath.cubeTranslation(new Translation2d(xInput, yInput));

        return swerveDrive.swerveController.getTargetSpeeds(scaledInputs.getX(), scaledInputs.getY(), angle.getRadians(), getHeading().getRadians(), Constants.MAX_SPEED);
    }

    public ChassisSpeeds getFieldVelocity(){
        return swerveDrive.getFieldVelocity();
    }

    public ChassisSpeeds getRobotVelocity(){
        return swerveDrive.getRobotVelocity();
    }

    public SwerveController getSwerveController(){
        return swerveDrive.swerveController;
    }

    public SwerveDriveConfiguration getSwerveDriveConfiguration(){
        return swerveDrive.swerveDriveConfiguration;
    }

    // public void lock(){
    //     swerveDrive.lockpose();
    // }

    public Rotation2d getPitch(){
        return swerveDrive.getPitch();
    }

    public SwerveDrive getSwerveDrive(){
        return swerveDrive;
    }
    
}
