package frc.robot.subsystems.drivetrain;

import com.ctre.phoenix.sensors.CANCoder;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;

// PID Controller for a single swerve pod turn module
public class SwerveTurnPIDController extends PIDController {
    // encoder for the swerve module
    private CANCoder encoder;
    
    // max & min motor speeds
    private double maxOutput = 1; 
    private double minOutput = -1; 

    public SwerveTurnPIDController(CANCoder encoder, double p, double i, double d) {
        super(p, i, d);
        this.encoder = encoder; 

        // allow the motor to rotate past 360 degrees and not break
        enableContinuousInput(0, 360);
    }

    // calculates the PID output and clamps it
    public double calculate() {
        return MathUtil.clamp(calculate(encoder.getAbsolutePosition()), minOutput, maxOutput); 
    }

    // returns max PID output
    public void setMaxOutput(double maxOutput) {
        this.maxOutput = maxOutput; 
    }

    // returns min PID output
    public void setMinOutput(double minOutput) {
        this.minOutput = minOutput; 
    }

    
}
