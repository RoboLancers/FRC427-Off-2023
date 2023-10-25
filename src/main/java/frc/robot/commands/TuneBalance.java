package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.drivetrain.Drivetrain;
import frc.robot.util.IOUtils;

public class TuneBalance extends CommandBase {
    Drivetrain m_drivetrain;
    PIDController m_BalancePID = new PIDController(
        Constants.BalanceAutoConstants.kBalance_P,
        Constants.BalanceAutoConstants.kBalance_I, 
        Constants.BalanceAutoConstants.kBalance_D
    );
    // Establishes the drivetrain, continuous inputs and the tolerance constants
public TuneBalance(Drivetrain drivetrain) {
    m_drivetrain = drivetrain;
    m_BalancePID.enableContinuousInput(-180, 180);
    m_BalancePID.setTolerance(Constants.BalanceAutoConstants.kBalanceErrorThreshold);

    addRequirements(drivetrain);

    }

    public void initialize() {
        // sets the desired angle to go towards at ZERO
        m_BalancePID.setSetpoint(0);
    }

    public void execute() {
        // Calculates our current angle to the desired angle of 0. 
        // Then uses the calulated number in the PID to drive at certain speed to get to 0 degrees on the pad
    double speedOnDegree = m_BalancePID.calculate(m_drivetrain.gyro.getPitch(), m_BalancePID.getSetpoint());
        m_drivetrain.swerveDrive(speedOnDegree, 0, 0);
        m_BalancePID.setP(IOUtils.get("Balance P"));
        m_BalancePID.setI(IOUtils.get("Balance I"));
        m_BalancePID.setD(IOUtils.get("Balance D"));
    }

    public boolean isFinished() {
        // runs and tells whether or not the command should finish
        return m_BalancePID.atSetpoint(); 
    }

    public void end(boolean interrupted) {
        // tells the bot to STOP MOVING when alls done :)
        m_drivetrain.swerveDrive(0, 0, 0);
    }
}
