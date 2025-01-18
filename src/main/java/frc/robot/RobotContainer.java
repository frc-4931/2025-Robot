package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.Constants.OIConstants;
import frc.robot.commands.SwerveJoystickCmd;
import frc.robot.subsystems.SwerveSubsystem;
//import frc.robot.subsystems.Vision;

public class RobotContainer {
    //This is where we intalize all the commands

    private final SwerveSubsystem swerveSubsystem = new SwerveSubsystem();
   
    private final CommandXboxController driverJoystick = new CommandXboxController(OIConstants.kDriverControllerPort);
    private final CommandJoystick buttonBox = new CommandJoystick(1);

    public RobotContainer() {
        //This is where we make the joystick command.
        swerveSubsystem.setDefaultCommand(new SwerveJoystickCmd(
                swerveSubsystem,
                //The first two lines control forward and sideways movement
                () -> -driverJoystick.getRawAxis(OIConstants.kDriverYAxis),
                () -> -driverJoystick.getRawAxis(OIConstants.kDriverXAxis),
                //Next line controls turing
                () -> -driverJoystick.getRawAxis(OIConstants.kDriverRotAxis),
                //When the right bumper is pressed the robot is in robot oriented mode
                () -> !driverJoystick.rightBumper().getAsBoolean()));

        configureButtonBindings();
    }

    private void configureButtonBindings() {
        driverJoystick.leftBumper().onTrue(swerveSubsystem.zeroHeadingCommand());
    }

    public Command getAutonomousCommand() {
        return null;
    }
}