// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.auto;

import java.io.IOException;
import java.nio.file.Path;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.RamseteController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryUtil;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj2.command.RamseteCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants;
import frc.robot.subsystems.DriveSubsystem;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class runTrajectoryCommand extends SequentialCommandGroup {
  /** Creates a new runTrajectoryCommand. */
  public runTrajectoryCommand(DriveSubsystem drive, String pathFile) {

    Trajectory traj = new Trajectory();

    String pathName = "";
    String trajFile = "";

    pathName = pathFile;

    trajFile = "output/" + pathName + ".wpilib.json";

    try{
        Path pathStraight = Filesystem.getDeployDirectory().toPath().resolve(trajFile);
        traj = TrajectoryUtil.fromPathweaverJson(pathStraight);
    } catch (IOException ex) {
        DriverStation.reportError("Unable to open one or more trajectories", ex.getStackTrace());
    }
    
    drive.resetOdometry(traj.getInitialPose());

    RamseteCommand ramseteCommand =
        new RamseteCommand(
            traj,
            drive::getPose,
            new RamseteController(
                Constants.AutoConstants.kRamseteB, Constants.AutoConstants.kRamseteZeta),
            new SimpleMotorFeedforward(
                Constants.DriveConstants.ksVolts,
                Constants.DriveConstants.kvVoltSecondsPerMeter,
                Constants.DriveConstants.kaVoltSecondsSquaredPerMeter),
            Constants.DriveConstants.kDriveKinematics,
            drive::getWheelSpeeds,
            new PIDController(Constants.DriveConstants.kPDriveVel, 0, 0),
            new PIDController(Constants.DriveConstants.kPDriveVel, 0, 0),
            // RamseteCommand passes volts to the callback
            drive::tankDriveVolts,
            drive);

    addCommands(
      new WaitCommand(0.5),
      ramseteCommand.andThen(() -> drive.tankDriveVolts(0, 0))
    );
  }
}
