// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.auto.NamedCommands;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.OperatorConstants;
import frc.robot.commands.AlgieInCommand;
import frc.robot.commands.AlgieOutCommand;
import frc.robot.commands.ArmDownCommand;
import frc.robot.commands.ArmUpCommand;
import frc.robot.commands.AutoCommands;
import frc.robot.commands.CoralOutCommand;
import frc.robot.commands.CoralStackCommand;
import frc.robot.subsystems.AlgaeArm;
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.CoralDrop;
import frc.robot.subsystems.LEDs;
import frc.robot.subsystems.Roller;
import frc.robot.subsystems.SwerveSubsystem;
import java.io.File;
import com.pathplanner.lib.auto.AutoBuilder;
import swervelib.SwerveInputStream;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a "declarative" paradigm, very
 * little robot logic should actually be handled in the {@link Robot} periodic methods (other than the scheduler calls).
 * Instead, the structure of the robot (including subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {

  // Replace with CommandPS4Controller or CommandJoystick if needed
  final CommandXboxController driverXbox = new CommandXboxController(0);
  // The robot's subsystems and commands are defined here...
  private final SwerveSubsystem drivebase = new SwerveSubsystem(new File(Filesystem.getDeployDirectory(), "swerve"));

  // Button Box
  final CommandJoystick buttonBox1 = new CommandJoystick(1);
  final CommandJoystick buttonBox2 = new CommandJoystick(2);

  // Commands
  private final CoralDrop coralDrop = new CoralDrop();
  private final AlgaeArm algaeArm = new AlgaeArm();
  private final Climber climber = new Climber();
  private final Roller roller = new Roller();
  private final LEDs lEDs = new LEDs();
  private final AutoCommands autoCommands = new AutoCommands();

  // Auto Chooser
  private final SendableChooser<Command> autoChooser;

  /**
   * Converts driver input into a field-relative ChassisSpeeds that is controlled by angular velocity.
   */
  SwerveInputStream driveAngularVelocity = SwerveInputStream.of(drivebase.getSwerveDrive(), () -> driverXbox.getLeftY() * -1, () -> driverXbox.getLeftX() * -1)
                                                            .withControllerRotationAxis(driverXbox::getRightX)
                                                            .deadband(OperatorConstants.DEADBAND)
                                                            .scaleTranslation(0.8)
                                                            .allianceRelativeControl(true);

  /**
   * Clone's the angular velocity input stream and converts it to a fieldRelative input stream.
   */
  SwerveInputStream driveDirectAngle = driveAngularVelocity.copy().withControllerHeadingAxis(driverXbox::getRightX, driverXbox::getRightY)
                                                           .headingWhile(true);

  /**
   * Clone's the angular velocity input stream and converts it to a robotRelative input stream.
   */
  SwerveInputStream driveRobotOriented = driveAngularVelocity.copy().robotRelative(true)
                                                             .allianceRelativeControl(false);

  SwerveInputStream driveAngularVelocityKeyboard = SwerveInputStream.of(drivebase.getSwerveDrive(), () -> -driverXbox.getLeftY(), () -> -driverXbox.getLeftX())
                                                                    .withControllerRotationAxis(() -> driverXbox.getRawAxis(2))
                                                                    .deadband(OperatorConstants.DEADBAND)
                                                                    .scaleTranslation(0.8)
                                                                    .allianceRelativeControl(true);
  // Derive the heading axis with math!
  SwerveInputStream driveDirectAngleKeyboard = driveAngularVelocityKeyboard.copy().withControllerHeadingAxis(() -> Math.sin(driverXbox.getRawAxis(2) * Math.PI) * (Math.PI * 2), () -> Math.cos(driverXbox.getRawAxis( 2) * Math.PI) * (Math.PI * 2))
                                                                           .headingWhile(true)
                                                                           .translationHeadingOffset(true)
                                                                           .translationHeadingOffset(Rotation2d.fromDegrees(0));
  
  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    // Configure the trigger bindings
    configureBindings();

    DriverStation.silenceJoystickConnectionWarning(true);

    NamedCommands.registerCommand("FeederStart", algaeArm.ArmUp().andThen(Commands.waitSeconds(.1)).andThen(algaeArm.ArmStop()));
    NamedCommands.registerCommand("CoralDrop", Commands.waitSeconds(.2).andThen(roller.CoralSpit()).andThen(Commands.waitSeconds(.7)).andThen(roller.CoralStop()));//.andThen(algaeArm.ArmDown()).andThen(Commands.waitSeconds(.5)).andThen(algaeArm.ArmUp()).andThen(Commands.waitSeconds(.5)).andThen(algaeArm.ArmStop())));
    NamedCommands.registerCommand("Feeder", algaeArm.ArmDown().andThen(Commands.waitSeconds(.2).andThen(algaeArm.ArmStop()).andThen(Commands.waitSeconds(.5)).andThen(algaeArm.ArmUp().andThen(Commands.waitSeconds(.2))).andThen(algaeArm.ArmStop())));

    autoChooser = AutoBuilder.buildAutoChooser();
    SmartDashboard.putData("Auto Chooser", autoChooser);
  }

  /**
   * Use this method to define your trigger->command mappings. Triggers can be created via the
   * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with an arbitrary predicate, or via the
   * named factories in {@link edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for
   * {@link CommandXboxController Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller PS4}
   * controllers or {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight joysticks}.
   */
  private void configureBindings() {
    Command driveFieldOrientedDirectAngle = drivebase.driveFieldOriented(driveDirectAngle);
    Command driveFieldOrientedAnglularVelocity = drivebase.driveFieldOriented(driveAngularVelocity);
    Command driveRobotOrientedAngularVelocity = drivebase.driveFieldOriented(driveRobotOriented);
    Command driveSetpointGen = drivebase.driveWithSetpointGeneratorFieldRelative(driveDirectAngle);
    Command driveFieldOrientedDirectAngleKeyboard = drivebase.driveFieldOriented(driveDirectAngleKeyboard);
    Command driveFieldOrientedAnglularVelocityKeyboard = drivebase.driveFieldOriented(driveAngularVelocityKeyboard);
    Command driveSetpointGenKeyboard = drivebase.driveWithSetpointGeneratorFieldRelative(driveDirectAngleKeyboard);

    if (RobotBase.isSimulation()) {
      drivebase.setDefaultCommand(driveFieldOrientedDirectAngleKeyboard);
    } 
    else {
      drivebase.setDefaultCommand(driveFieldOrientedAnglularVelocity);
    }

    if (Robot.isSimulation()) {
      Pose2d target = new Pose2d(new Translation2d(1, 4),
                                 Rotation2d.fromDegrees(90));
      //drivebase.getSwerveDrive().field.getObject("targetPose").setPose(target);
      driveDirectAngleKeyboard.driveToPose(() -> target,
                                           new ProfiledPIDController(5,
                                                                     0,
                                                                     0,
                                                                     new Constraints(5, 2)),
                                           new ProfiledPIDController(5,
                                                                     0,
                                                                     0,
                                                                     new Constraints(Units.degreesToRadians(360),
                                                                                     Units.degreesToRadians(180))
                                           ));
      driverXbox.start().onTrue(Commands.runOnce(() -> drivebase.resetOdometry(new Pose2d(3, 3, new Rotation2d()))));
      driverXbox.button(1).whileTrue(drivebase.sysIdDriveMotorCommand());
      driverXbox.button(2).whileTrue(Commands.runEnd(() -> driveDirectAngleKeyboard.driveToPoseEnabled(true),
                                                     () -> driveDirectAngleKeyboard.driveToPoseEnabled(false)));

//      driverXbox.b().whileTrue(
//          drivebase.driveToPose(
//              new Pose2d(new Translation2d(4, 4), Rotation2d.fromDegrees(0)))
//                              );

    }

    if (DriverStation.isTest()) {
      drivebase.setDefaultCommand(driveFieldOrientedAnglularVelocity); // Overrides drive command above!
      driverXbox.leftBumper().onTrue(Commands.none());
      driverXbox.rightBumper().onTrue(Commands.none());
    } 
    else {
      // Reset Gyro Button
      driverXbox.y().onTrue((Commands.runOnce(drivebase::zeroGyro)));

      // X Button
      driverXbox.x().onTrue(algaeArm.ArmStop().andThen(roller.CoralStop()).andThen(climber.ClimbStop()));

      // Arm Controls 
      driverXbox.pov(0).whileTrue(new ArmUpCommand(algaeArm));
      driverXbox.pov(180).whileTrue(new ArmDownCommand(algaeArm));
      driverXbox.rightTrigger(.2).whileTrue(new ArmDownCommand(algaeArm));
      driverXbox.leftTrigger(0.2).whileTrue(new ArmUpCommand(algaeArm));

      // Climber Controls
      driverXbox.pov(90).whileTrue(climber.climbOut(150));
      driverXbox.pov(270).whileTrue(climber.climbOut(-50));
      driverXbox.back().whileTrue(climber.climbOut(0));

      // Coral Controls
      driverXbox.a().whileTrue(new CoralOutCommand(roller));
      driverXbox.b().whileTrue(new CoralStackCommand(roller));

      // Algie Controls
      driverXbox.leftBumper().whileTrue(new AlgieOutCommand(roller));
      driverXbox.rightBumper().onTrue(new AlgieInCommand(roller));

      
      //driverXbox.b().onTrue((Commands.runOnce(drivebase::resetToPose)));

      buttonBox1.button(9).whileTrue(autoCommands.ScoreCoral('A'));
      buttonBox1.button(8).whileTrue(autoCommands.ScoreCoral('B'));
      buttonBox1.button(7).whileTrue(autoCommands.ScoreCoral('C'));
      buttonBox1.button(6).whileTrue(autoCommands.ScoreCoral('D'));
      buttonBox1.button(5).whileTrue(autoCommands.ScoreCoral('G'));
      buttonBox1.button(4).whileTrue(autoCommands.ScoreCoral('H'));
      buttonBox1.button(3).whileTrue(autoCommands.ScoreCoral('I'));
      buttonBox1.button(2).whileTrue(autoCommands.ScoreCoral('J'));
      buttonBox1.button(1).whileTrue(autoCommands.ScoreCoral('K'));
      buttonBox1.button(10).whileTrue(autoCommands.ScoreCoral('L'));

      buttonBox2.button(5 ).onTrue(algaeArm.ArmStop().andThen(roller.CoralStop()).andThen(climber.ClimbStop()));
      buttonBox2.button(4).whileTrue(autoCommands.ScoreCoral('E'));
      buttonBox2.button(3).whileTrue(autoCommands.ScoreCoral('F'));
      buttonBox2.button(1).whileTrue(autoCommands.LFeeder());
      buttonBox2.button(12).whileTrue(autoCommands.RFeeder());
      buttonBox2.button(9).whileTrue(autoCommands.Process());
      buttonBox2.button(2).whileTrue(autoCommands.Climb());
      buttonBox2.button(12).whileTrue(autoCommands.RFeeder());
    }
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    return autoChooser.getSelected();
    //return null;
  }
}