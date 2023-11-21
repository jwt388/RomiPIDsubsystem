// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.subsystems.DrivePIDSubsystem;
import frc.robot.subsystems.OnBoardIO;
import frc.robot.subsystems.OnBoardIO.ChannelMode;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  public final OnBoardIO m_onboardIO = new OnBoardIO(ChannelMode.OUTPUT, ChannelMode.OUTPUT); // Enable LEDs
  private final DrivePIDSubsystem m_driveSubsystem = new DrivePIDSubsystem(m_onboardIO);

  // Assumes a gamepad plugged into channel 0
  private final CommandXboxController m_driverController = new CommandXboxController(0);

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
   
    // Configure the button bindings
    configureButtonBindings();

    m_onboardIO.setGreenLed(false);
    m_onboardIO.setRedLed(false);

    SmartDashboard.putData(m_driveSubsystem);

  }

  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {

    // Move to the forward position when the 'B' button is pressed.
    m_driverController
    .b()
    .onTrue(
        Commands.runOnce(
            () -> {
              m_driveSubsystem.setGoal(Constants.kFwdPosition);
              m_driveSubsystem.enable();

            },
            m_driveSubsystem));

    // Move to the back position when the 'A' button is pressed.
    m_driverController
    .a()
    .onTrue(
        Commands.runOnce(
            () -> {
              m_driveSubsystem.setGoal(Constants.kBackPosition);
              m_driveSubsystem.enable();
            },
            m_driveSubsystem));

    // Move to start position when Y is pressed.
    m_driverController
    .y()
    .onTrue(
        Commands.runOnce(
            () -> {
              m_driveSubsystem.setGoal(Constants.kStartPosition);
              m_driveSubsystem.enable();
            },
            m_driveSubsystem));
            
    // Shift position back a small amount when the POV Down is pressed.
    m_driverController
    .povDown()
    .onTrue(
        Commands.runOnce(
          () -> {
            m_driveSubsystem.setGoal(m_driveSubsystem.decreasedGoal());
            m_driveSubsystem.enable();
          },
          m_driveSubsystem));

    // Shift position forward a small amount when the POV Up is pressed.
    m_driverController
      .povUp()
      .onTrue(
          Commands.runOnce(
            () -> {
              m_driveSubsystem.setGoal(m_driveSubsystem.increasedGoal());
              m_driveSubsystem.enable();
            },
            m_driveSubsystem));

    // Reset the encoders to zero when the 'X' button is pressed. 
    //   Should only be used when are is in neutral position.
    m_driverController.x().onTrue(Commands.runOnce(m_driveSubsystem::resetPosition));

    // Disable the drive controller when left bumper is pressed.
    m_driverController
    .leftBumper()
    .onTrue(
        Commands.runOnce(
            () -> {
              m_driveSubsystem.disable();
            },
            m_driveSubsystem));
            
    // Enable the drive controller when right bumper is pressed.
    m_driverController
    .rightBumper()
    .onTrue(
        Commands.runOnce(
            () -> {
              m_driveSubsystem.enable();
            },
            m_driveSubsystem));

    // m_driverController.y().onTrue(Commands.runOnce(m_driveSubsystem::disable));

  }

  /**
   * Disables all ProfiledPIDSubsystem and PIDSubsystem instances. This should be called on robot
   * disable to prevent integral windup.
   */
  public void disablePIDSubsystems() {
    m_driveSubsystem.disable();
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    return Commands.none();
  }

  
}
