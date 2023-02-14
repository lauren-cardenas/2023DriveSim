// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.XboxController.Button;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.commands.auto.runTrajectoryCommand;
import frc.robot.subsystems.DriveSubsystem;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;


public class RobotContainer {
  /******SUBSYSTEMS****** */
  private final DriveSubsystem m_robotDrive = new DriveSubsystem();

  private final SendableChooser<Command> autoChooser = new SendableChooser<>();

  /******COMMANDS****** */
  private final Command runStraightTrajectory = new runTrajectoryCommand(m_robotDrive, "straight");
  private final Command runStraightBackTrajectory = new runTrajectoryCommand(m_robotDrive, "straightBack");
  private final Command runForwardLoopTrajectory = new runTrajectoryCommand(m_robotDrive, "forwardLoop");
  private final SequentialCommandGroup runMultiTrajectories = new runTrajectoryCommand(m_robotDrive, "straight").andThen(new runTrajectoryCommand(m_robotDrive, "straightBack"));


  XboxController m_driverController = new XboxController(Constants.OIConstants.kDriverControllerPort);

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    
    /******CHOOSER****** */
    SmartDashboard.putData("Auto Paths", autoChooser);
    autoChooser.setDefaultOption("Straight Back", runStraightBackTrajectory);
    autoChooser.addOption("Straight Forward", runStraightTrajectory);
    autoChooser.addOption("Loop Forward", runForwardLoopTrajectory);
    autoChooser.addOption("Multi", runMultiTrajectories);

    
    // Configure the button bindings
    configureButtonBindings();

    m_robotDrive.setDefaultCommand(
        new RunCommand(
            () ->
                m_robotDrive.arcadeDrive(
                    -m_driverController.getLeftY(), -m_driverController.getRightX()),
            m_robotDrive));
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link edu.wpi.first.wpilibj.GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then calling passing it to a
   * {@link JoystickButton}.
   */
  private void configureButtonBindings() {
    // Drive at half speed when the right bumper is held
    new JoystickButton(m_driverController, Button.kRightBumper.value)
        .onTrue(new InstantCommand(() -> m_robotDrive.setMaxOutput(0.5)))
        .onFalse(new InstantCommand(() -> m_robotDrive.setMaxOutput(1)));
  }

  public DriveSubsystem getRobotDrive() {
    return m_robotDrive;
  }

  /** Zeros the outputs of all subsystems. */
  public void zeroAllOutputs() {
    m_robotDrive.tankDriveVolts(0, 0);
  }

   public Command getAutonomousCommand(){
        return autoChooser.getSelected();
    }

}
