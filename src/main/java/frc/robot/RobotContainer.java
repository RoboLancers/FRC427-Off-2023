// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.subsystems.drivetrain.Drivetrain;
import frc.robot.util.Controller;
import frc.robot.util.DriverController;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;

/*
 * Things to talk about: 
 * - FRC control system
 * - subsystems
 *  - periodic
 * - constants
 * - motors
 *  - set power
 *  - encoders
 *  - motor inversion
 *  - smart current limit
 */

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
  private final AutoPicker autoPicker; 

  // The robot's subsystems and commands are defined here...
  private final Drivetrain drivetrain = new Drivetrain();

  // Replace with CommandPS4Controller or CommandJoystick if needed
  private final DriverController driverController =
      new DriverController(0);

  // private Command command = new SwerveTunerCommand(Constants.DrivetrainConstants.BackLeft.kRotate, Constants.DrivetrainConstants.BackLeft.kDrive, Constants.DrivetrainConstants.BackLeft.kRotEncoder); 

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {

    autoPicker = new AutoPicker(drivetrain); 
    // Configure the trigger bindings
    configureBindings();

    // driverController.setChassisSpeedsSupplier(drivetrain::getChassisSpeeds); // comment in simulation
    drivetrain.setDefaultCommand(new RunCommand(() -> {
      ChassisSpeeds speeds = driverController.getDesiredChassisSpeeds(); 
      SmartDashboard.putNumber("x", speeds.vxMetersPerSecond); 
      SmartDashboard.putNumber("y", speeds.vyMetersPerSecond); 
      SmartDashboard.putNumber("rotation", speeds.omegaRadiansPerSecond); 
      drivetrain.swerveDrive(speeds);
    }, drivetrain));
  }

  /**
   * Use this method to define your trigger->command mappings. Triggers can be created via the
   * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with an arbitrary
   * predicate, or via the named factories in {@link
   * edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for {@link
   * CommandXboxController Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller
   * PS4} controllers or {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight
   * joysticks}.
   */
  private void configureBindings() {
    Controller.onPress(driverController.Y, new InstantCommand(() -> {
      drivetrain.zeroYaw();
    }));
  }

  public void doSendables() {

  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    return autoPicker.getAuto(); 
  }
}
