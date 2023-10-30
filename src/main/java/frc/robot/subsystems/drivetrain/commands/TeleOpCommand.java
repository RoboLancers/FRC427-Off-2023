package frc.robot.subsystems.drivetrain.commands;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.drivetrain.Drivetrain;
import frc.robot.util.ChassisState;
import frc.robot.util.DriverController;

public class TeleOpCommand extends CommandBase {
    
    private Drivetrain m_drivetrain; 
    private DriverController m_controller; 

    public TeleOpCommand(Drivetrain drivetrain, DriverController driverController) {
        this.m_drivetrain = drivetrain; 
        this.m_controller = driverController; 

        addRequirements(this.m_drivetrain);
    } 

    @Override
    public void initialize() {
        m_drivetrain.resetLastTurnedTheta(); 
    }

    @Override
    public void execute() {
        // ensure driving does not break if gyro disconnects, will hopefully transition to robot oriented drive
        if (m_drivetrain.gyro.isConnected()) {
            ChassisState speeds = m_controller.getDesiredChassisState(); 
            SmartDashboard.putNumber("x", speeds.vxMetersPerSecond); 
            SmartDashboard.putNumber("y", speeds.vyMetersPerSecond); 
            SmartDashboard.putNumber("rotation", speeds.omegaRadians); 
            m_drivetrain.swerveDriveFieldRel(speeds);
        } else {
            ChassisSpeeds speeds = m_controller.getDesiredChassisSpeeds(); 
            SmartDashboard.putNumber("x", speeds.vxMetersPerSecond); 
            SmartDashboard.putNumber("y", speeds.vyMetersPerSecond); 
            SmartDashboard.putNumber("rotation", speeds.omegaRadiansPerSecond); 
            m_drivetrain.swerveDrive(speeds);
        }
    }

}
