package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.drivetrain.Drivetrain;
import frc.robot.Constants;

public class BalanceAuto extends SequentialCommandGroup {
    Drivetrain m_drivetrain;
    PIDController m_BalancePID = new PIDController(
        Constants.BalanceAutoConstants.kBalance_P,
        Constants.BalanceAutoConstants.kBalance_I, 
        Constants.BalanceAutoConstants.kBalance_D
    );

public BalanceAuto(Drivetrain drivetrain) {
    addCommands(new Balance(drivetrain));
    }
}