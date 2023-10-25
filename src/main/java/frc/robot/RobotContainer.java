// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.subsystems.arm.Arm;
import frc.robot.subsystems.arm.commands.GoToGround;
import frc.robot.subsystems.arm.commands.GoToHardStop;
import frc.robot.subsystems.drivetrain.Drivetrain;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.intake.commands.IntakeStop;
import frc.robot.subsystems.intake.commands.TakeIn;
import frc.robot.subsystems.intake.commands.TakeOut;
import frc.robot.util.DriverController;
import frc.robot.util.DriverController.Mode;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
  private final AutoPicker autoPicker; 

  // drivetrain of the robot
  private final Drivetrain drivetrain = new Drivetrain();

  //intake of the robot
  private final Intake intake = new Intake();

  //arm of the robot
  private final Arm arm = new Arm();

  // controller for the driver
  private final DriverController driverController =
      new DriverController(0);

  private final CommandXboxController manipulatorController = new CommandXboxController(1); 

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    autoPicker = new AutoPicker(drivetrain, intake, arm); 
    // Configure the trigger bindings
    configureBindings();

    // driverController.setChassisSpeedsSupplier(drivetrain::getChassisSpeeds); // comment in simulation
    // default command for drivetrain is to calculate speeds from controller and drive the robot
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
    //manipulatorController.().onTrue(new TakeIn(intake, 1));
    //manipulatorController.()onFalse(new IntakeStop(intake));

    driverController.RightTrigger
      .onTrue(new InstantCommand(() -> driverController.setSlowMode(Mode.SLOW)))
      .onFalse(new InstantCommand(() -> driverController.setSlowMode(Mode.NORMAL))); 

    new Trigger(() -> manipulatorController.getLeftY() < -0.5)
      .onTrue(new TakeIn(intake, Constants.IntakeConstants.kIntakeSpeed))
      .onFalse(new IntakeStop(intake)); 

    manipulatorController.y().onTrue(new TakeOut(intake, Constants.IntakeConstants.kShootSpeedHigh));
    manipulatorController.y().onFalse(new IntakeStop(intake));

    manipulatorController.b().onTrue(new TakeOut(intake, Constants.IntakeConstants.kShootSpeedMid));
    manipulatorController.b().onFalse(new IntakeStop(intake));

    manipulatorController.a().onTrue(new TakeOut(intake, Constants.IntakeConstants.kShootSpeedLow));
    manipulatorController.a().onFalse(new IntakeStop(intake));

    manipulatorController.rightTrigger().onTrue(new GoToGround(arm));
    manipulatorController.rightTrigger().onFalse(new GoToHardStop(arm));
  }
  // send any data as needed to the dashboard
  public void doSendables() {

  }

  // givess the currently picked auto as the chosen auto for the match
  public Command getAutonomousCommand() {
    return autoPicker.getAuto();
    // return new SwerveTurnTunerCommand(7, 8, 13); 
  }
}
