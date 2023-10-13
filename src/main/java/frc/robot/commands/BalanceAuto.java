package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.drivetrain.Drivetrain;
import frc.robot.Constants;

public class BalanceAuto extends CommandBase {
    Drivetrain m_drivetrain;
    PIDController m_BalancePID = new PIDController(
        Constants.BalanceAutoConstants.kBalance_P,
        Constants.BalanceAutoConstants.kBalance_I, 
        Constants.BalanceAutoConstants.kBalance_D
    );

public BalanceAuto(Drivetrain drivetrain) {
    m_drivetrain = drivetrain;
    m_BalancePID.enableContinuousInput(-180, 180);

    addRequirements(drivetrain);

    }

    public void initialize() {
        // sets the desired angle to go towards at ZERO
        m_BalancePID.setSetpoint(0);
    }

    public void execute() {
        // Compares our current angle to the desired angle of 0 
        m_BalancePID.calculate(m_drivetrain.gyro.getPitch(), m_BalancePID.getSetpoint());
    }

    public boolean isFinished() {
        // runs and tells whether or not the command should finish
        return m_BalancePID.atSetpoint(); 
    }

    public void end(boolean interrupted) {
        // runs when the command is ended
    }
}
