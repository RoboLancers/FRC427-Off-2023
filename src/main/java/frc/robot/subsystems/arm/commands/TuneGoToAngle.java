package frc.robot.subsystems.arm.commands;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.arm.Arm;
import frc.robot.util.IOUtils;

public class TuneGoToAngle extends CommandBase {
    Arm m_arm;
    double m_angle;



    public TuneGoToAngle(Arm arm) {
        m_arm = arm;
        m_angle = IOUtils.get("ArmAngle");

        addRequirements(arm);

    }

    public void initialize() {
        // makes arm go to angle
        
    }

    public void execute() {
        m_arm.m_ArmPIDController.setP(IOUtils.get("Arm Go To Angle P"));
        m_arm.m_ArmPIDController.setI(IOUtils.get("Arm Go To Angle I"));
        m_arm.m_ArmPIDController.setD(IOUtils.get("Arm Go To Angle D"));
        m_arm.m_feedForward = new ArmFeedforward(IOUtils.get("Arm Go To Angle ks"), IOUtils.get("Arm Go To Angle kg"), IOUtils.get("Arm Go To Angle kv"));
 
        
        m_arm.goToAngle(IOUtils.get("ArmAngle"));

        IOUtils.set("Current angle of Arm", m_arm.getAngle());

        //m_arm.m_feedForward.calculate(m_angle, IOUtils.get("tune Go To Angle Velocity"));
    }

    public boolean isFinished() {
        // confirmation that arm is at angle
        return m_arm.isAtAngle(); 
    }

    public void end(boolean interrupted) {
        // runs when the command is ended
    }
}