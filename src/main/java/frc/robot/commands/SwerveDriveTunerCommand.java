package frc.robot.commands;


import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.drivetrain.SwerveModule;

public class SwerveDriveTunerCommand extends CommandBase {
    SwerveModule module; 

    public SwerveDriveTunerCommand(int turn, int drive, int encoder) {
        this.module = new SwerveModule(turn, drive, encoder); 
    }

    @Override
    public void initialize() {
        SmartDashboard.putNumber("SwerveDrivekP", SmartDashboard.getNumber("SwerveDrivekP", 0));
        SmartDashboard.putNumber("SwerveDrivekI", SmartDashboard.getNumber("SwerveDrivekI", 0));
        SmartDashboard.putNumber("SwerveDrivekD", SmartDashboard.getNumber("SwerveDrivekD", 0)); 
        SmartDashboard.putNumber("SwerveDriveSetpoint", SmartDashboard.getNumber("SwerveDriveSetpoint", 0)); 
    }

    @Override
    public void execute() {
        module.setDrivePID(
            SmartDashboard.getNumber("SwerveDrivekP", Constants.DrivetrainConstants.kModuleDrive_P), 
            SmartDashboard.getNumber("SwerveDrivekI", 0), 
            SmartDashboard.getNumber("SwerveDrivekD", 0)
            );

        module.updateState(new SwerveModuleState(0, Rotation2d.fromDegrees(
            SmartDashboard.getNumber("SwerveDriveSetpoint", 0))
        ), SwerveModule.DriveState.CLOSED_LOOP);

        SmartDashboard.putNumber("SwerveTurnMeasurement", module.getCurrentState().speedMetersPerSecond); 
        SmartDashboard.putNumber("SwerveError", module.turnPIDController.getPositionError()); 
        // SmartDashboard.putNumber("Swerve", module.turnPIDController.get); 
    }

    @Override
    public boolean isFinished() {
        return false; 
    }
}
