package frc.robot.commands;

import java.io.File;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Constants;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.PathfindingCommand;
import com.pathplanner.lib.events.PointTowardsZoneTrigger;

import java.io.IOException;
import java.nio.file.Path;
import java.text.ParseException;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.auto.AutoBuilder;
import edu.wpi.first.wpilibj2.command.Command;


public class AutoCommands {
    //private final SwerveSubsystem drivebase = new SwerveSubsystem(new File(Filesystem.getDeployDirectory(), "swerve"));
    PathConstraints Pathconstraint = new PathConstraints(Constants.MAX_SPEED, 1, Units.degreesToRadians(360), Units.degreesToRadians(540));

    

    public Command driveToPose(Pose2d pose){
        return AutoBuilder.pathfindToPose(pose, Pathconstraint, 0);
  }

  public Command PoseToPath(String PathName){
    try{
      PathPlannerPath path = PathPlannerPath.fromPathFile(PathName);

      return AutoBuilder.pathfindThenFollowPath(path, Pathconstraint);
  } catch (Exception e) {
      DriverStation.reportError("Big oops: " + e.getMessage(), e.getStackTrace());
      return Commands.none();
  }
  }

  public Command FollowPath(String PathName){
    try{
        PathPlannerPath path = PathPlannerPath.fromPathFile(PathName);

        // Create a path following command using AutoBuilder. This will also trigger event markers.
        return AutoBuilder.followPath(path);
    } catch (Exception e) {
        DriverStation.reportError("Big oops: " + e.getMessage(), e.getStackTrace());
        return Commands.none();
    }
  }

  public Command PathFindToPose(Pose2d targetPose){
    Command pathfindingCommand = AutoBuilder.pathfindToPose(targetPose, Pathconstraint);
    return pathfindingCommand;

  }

  public Command ScoreCoral(char where){
    switch(where){
      case 'A':
        return PoseToPath("A");
      case 'B':
        return PoseToPath("B");
      case 'C':
        return PoseToPath("C");
      case 'D':
        return PoseToPath("D");
      case 'E':
        return PoseToPath("E");
      case 'F':
        return PoseToPath("F");
      case 'G':
        return PoseToPath("G");
      case 'H':
        return PoseToPath("H");
      case 'I':
        return PoseToPath("I");
      case 'J':
        return PoseToPath("J");
      case 'K':
        return PoseToPath("K");
      case 'L':
        return PoseToPath("L");
      case 'T':
        return PoseToPath("Test");
      default:
        return Commands.print("You should capitalize");
    }

  }

  public Command LFeeder(){
    return PoseToPath("Left");
  }

  public Command RFeeder(){
    return PoseToPath("Right");
  }

  public Command Process(){
    return PoseToPath("Process");
  }

  public Command Climb(){
    return PoseToPath("Climb");
  }

  public Command AimAtFeeder(){
    Command pathfindingCommand = AutoBuilder.pathfindToPose(new Pose2d(2, 2, Rotation2d.fromDegrees(0)), Pathconstraint);
    return pathfindingCommand;
}
    
}
