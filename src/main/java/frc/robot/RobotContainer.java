// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.MathUtil;
import com.pathplanner.lib.auto.NamedCommands;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.DrivebaseConstants;
import frc.robot.Constants.OperatorConstants;
import frc.robot.commands.AlgieInCommand;
import frc.robot.commands.AlgieOutCommand;
import frc.robot.commands.ArmDownCommand;
import frc.robot.commands.ArmStop;
import frc.robot.commands.ArmUpCommand;
import frc.robot.commands.AutoCommands;
import frc.robot.commands.ClimberDownCommand;
import frc.robot.commands.ClimberUpCommand;
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

//import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.OperatorConstants;
// import frc.robot.autos.DriveForwardAuto;
// import frc.robot.autos.SimpleCoralAuto;
import frc.robot.commands.AlgieInCommand;
import frc.robot.commands.AlgieOutCommand;
import frc.robot.commands.ArmDownCommand;
import frc.robot.commands.ArmUpCommand;
import frc.robot.commands.ClimberDownCommand;
import frc.robot.commands.ClimberUpCommand;
import frc.robot.commands.CoralOutCommand;
import frc.robot.commands.CoralStackCommand;
// import frc.robot.commands.DriveCommand;
import frc.robot.subsystems.AlgaeArm;
import frc.robot.subsystems.Climber;
// import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.Roller;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;

public class RobotContainer {
  final CommandXboxController driverXbox = new CommandXboxController(0);
  final CommandJoystick buttonBox1 = new CommandJoystick(1);
  final CommandJoystick buttonBox2 = new CommandJoystick(2);
  private final SwerveSubsystem drivebase = new SwerveSubsystem(new File(Filesystem.getDeployDirectory(), "swerve"));
  //private final CoralDrop coralDrop = new CoralDrop();
  private final AlgaeArm algaeArm = new AlgaeArm();
  private final Climber climber = new Climber();
  private final Roller roller = new Roller();
  private final LEDs lEDs = new LEDs();
  private final AutoCommands autoCommands = new AutoCommands();

  SwerveInputStream driveAngularVelocity = SwerveInputStream.of(drivebase.getSwerveDrive(), () -> driverXbox.getLeftY() * -1, () -> driverXbox.getLeftX() * -1).withControllerRotationAxis(driverXbox::getRightX).deadband(OperatorConstants.DEADBAND).scaleTranslation(0.8).allianceRelativeControl(true);
  SwerveInputStream driveDirectAngle = driveAngularVelocity.copy().withControllerHeadingAxis(driverXbox::getRightX, driverXbox::getRightY).headingWhile(true);
  SwerveInputStream driveRobotOriented = driveAngularVelocity.copy().robotRelative(true).allianceRelativeControl(false);
  SwerveInputStream driveAngularVelocityKeyboard = SwerveInputStream.of(drivebase.getSwerveDrive(), () -> -driverXbox.getLeftY(), () -> -driverXbox.getLeftX()).withControllerRotationAxis(() -> driverXbox.getRawAxis(2)).deadband(OperatorConstants.DEADBAND).scaleTranslation(0.8).allianceRelativeControl(true);
  SwerveInputStream driveDirectAngleKeyboard     = driveAngularVelocityKeyboard.copy().withControllerHeadingAxis(() -> Math.sin(driverXbox.getRawAxis(2) * Math.PI) * (Math.PI * 2), () -> Math.cos(driverXbox.getRawAxis(2) * Math.PI) * (Math.PI * 2)).headingWhile(true);

  private final SendableChooser<Command> autoChooser;

  public RobotContainer() {
    DriverStation.silenceJoystickConnectionWarning(true);

    NamedCommands.registerCommand("CoralDrop", roller.CoralSpit().andThen(Commands.waitSeconds(.3)).andThen(roller.CoralStop().andThen(algaeArm.ArmDown()).andThen(Commands.waitSeconds(.5)).andThen(algaeArm.ArmUp()).andThen(Commands.waitSeconds(.5)).andThen(algaeArm.ArmStop())));

    autoChooser = AutoBuilder.buildAutoChooser();
    SmartDashboard.putData("Auto Chooser", autoChooser);



    configureBindings();
  }

  private void configureBindings() {
    Command driveFieldOrientedDirectAngle      = drivebase.driveFieldOriented(driveDirectAngle);
    Command driveFieldOrientedAnglularVelocity = drivebase.driveFieldOriented(driveAngularVelocity);
    Command driveRobotOrientedAngularVelocity  = drivebase.driveFieldOriented(driveRobotOriented);
    // Command driveSetpointGen = drivebase.driveWithSetpointGeneratorFieldRelative(
    //    driveDirectAngle);
    Command driveFieldOrientedDirectAngleKeyboard      = drivebase.driveFieldOriented(driveDirectAngleKeyboard);
    Command driveFieldOrientedAnglularVelocityKeyboard = drivebase.driveFieldOriented(driveAngularVelocityKeyboard);
   //Command driveSetpointGenKeyboard = drivebase.driveWithSetpointGeneratorFieldRelative(
//       driveDirectAngleKeyboard);

    if (RobotBase.isSimulation())
    {
      drivebase.setDefaultCommand(driveFieldOrientedDirectAngleKeyboard);
    } else
    {
      drivebase.setDefaultCommand(driveFieldOrientedAnglularVelocity);
    }

    if (Robot.isSimulation())
    {
      driverXbox.start().onTrue(Commands.runOnce(() -> drivebase.resetOdometry(new Pose2d(3, 3, new Rotation2d()))));
      //driverXbox.button(1).whileTrue(drivebase.sysIdDriveMotorCommand());

    }
    if (DriverStation.isTest())
    {
      drivebase.setDefaultCommand(driveFieldOrientedAnglularVelocity); // Overrides drive command above!

      // driverXbox.x().whileTrue(Commands.runOnce(drivebase::lock, drivebase).repeatedly());
      // driverXbox.y().whileTrue(drivebase.driveToDistanceCommand(1.0, 0.2));
      driverXbox.x().onTrue((Commands.runOnce(drivebase::zeroGyro)));
      //driverXbox.back().whileTrue(drivebase.centerModulesCommand());
      driverXbox.leftBumper().onTrue(Commands.none());
      driverXbox.rightBumper().onTrue(Commands.none());
    } else
    {
      driverXbox.pov(0).whileTrue(new ClimberUpCommand(climber));
      driverXbox.pov(180).whileTrue(new ClimberDownCommand(climber));

      driverXbox.y().onTrue((Commands.runOnce(drivebase::zeroGyro)));
      driverXbox.x().onTrue(algaeArm.ArmStop().andThen(roller.CoralStop()).andThen(climber.ClimbStop()));
      //driverXbox.b().whileTrue(runOncedrivebase.setDefaultCommand(driveRobotOrientedAngularVelocity));
      //add robot oriented button

      driverXbox.leftTrigger(.2).whileTrue(new AlgieInCommand(roller));
      driverXbox.leftBumper().whileTrue(new AlgieOutCommand(roller));

      driverXbox.pov(90).whileTrue(new ArmUpCommand(algaeArm));
      driverXbox.pov(270).whileTrue(new ArmDownCommand(algaeArm));

      driverXbox.rightTrigger(.2).whileTrue(roller.CoralSpit().andThen(Commands.waitSeconds(.3)).andThen(roller.CoralStop().andThen(algaeArm.ArmDown()).andThen(Commands.waitSeconds(.5)).andThen(algaeArm.ArmUp()).andThen(Commands.waitSeconds(.5)).andThen(algaeArm.ArmStop())));
      driverXbox.rightBumper().whileTrue(new CoralStackCommand(roller));
      driverXbox.a().whileTrue(autoCommands.FollowPath("Algae").andThen(roller.CoralSpit().andThen(Commands.waitSeconds(.3)).andThen(roller.CoralStop().andThen(algaeArm.ArmDown()).andThen(Commands.waitSeconds(.5)).andThen(algaeArm.ArmUp()).andThen(Commands.waitSeconds(.5)).andThen(algaeArm.ArmStop()))));
      //add alage coral button
      



      //Old Buttons
    //   driverXbox.a().onTrue((Commands.runOnce(drivebase::zeroGyro)));
    //   //driverXbox.b().onTrue(coralDrop.toggleRun());
    //   driverXbox.x().onTrue((Commands.runOnce(drivebase::resetToPose)));
    //   //driverXbox.x().onTrue(Commands.runOnce(drivebase::addFakeVisionReading));
    //   // driverXbox.b().whileTrue(
    //   //     drivebase.driveToPose(
    //   //         new Pose2d(new Translation2d(4, 4), Rotation2d.fromDegrees(0)))
    //   //                         );
    //   // driverXbox.start().whileTrue(Commands.runOnce(lEDs.setColor(0)));
    //   driverXbox.back().whileTrue(roller.CoralStop());
    //   //driverXbox.leftBumper().whileTrue(Commands.runOnce(drivebase::lock, drivebase).repeatedly());
    //   //driverXbox.rightBumper().whileTrue(autoCommands.PoseToPath("Test"));

      
    //   //driverXbox.leftBumper().onTrue(autoCommands.ScoreCoral('T'));
    //   //driverXbox.y().onTrue(drivebase.updatePose());
    //   buttonBox1.button(9).whileTrue(autoCommands.ScoreCoral('A'));
    //   buttonBox1.button(8).whileTrue(autoCommands.ScoreCoral('B'));
    //   buttonBox1.button(7).whileTrue(autoCommands.ScoreCoral('C'));
    //   buttonBox2.button(12).whileTrue(autoCommands.RFeeder());

      
    // //driverXbox.rightBumper().whileTrue(new AlgieInCommand(roller));
    // driverXbox.rightBumper().whileTrue(new AlgieInCommand(roller));
    
    // // Here we use a trigger as a button when it is pushed past a certain threshold
    // driverXbox.rightTrigger(.2).whileTrue(new AlgieOutCommand(roller));

    // /**
    //  * The arm will be passively held up or down after this is used,
    //  * make sure not to run the arm too long or it may get upset!
    //  */
    // driverXbox.pov(90).whileTrue(new ArmUpCommand(algaeArm));
    // driverXbox.pov(270).whileTrue(new ArmDownCommand(algaeArm));
    // driverXbox.b().onTrue(new ArmStop(algaeArm));

    // /**
    //  * Used to score coral, the stack command is for when there is already coral
    //  * in L1 where you are trying to score. The numbers may need to be tuned, 
    //  * make sure the rollers do not wear on the plastic basket.
    //  */
    // driverXbox.start().whileTrue(new CoralOutCommand(roller));
    // driverXbox.y().whileTrue(new CoralStackCommand(roller));

    // /**
    //  * POV is a direction on the D-Pad or directional arrow pad of the controller,
    //  * the direction of this will be different depending on how your winch is wound
    //  */
    // driverXbox.pov(0).whileTrue(new ClimberUpCommand(climber));
    // driverXbox.pov(180).whileTrue(new ClimberDownCommand(climber));
    }
  }

  public Command getAutonomousCommand() {
    return autoChooser.getSelected();
    //return null;
  }
}