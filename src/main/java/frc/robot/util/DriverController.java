package frc.robot.util;

import java.util.function.Supplier;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import frc.robot.Constants;

public class DriverController extends Controller {

    private final SlewRateLimiter forwardRateLimiter = new SlewRateLimiter(Constants.DrivetrainConstants.kForwardSlewRate);
    private final SlewRateLimiter strafeRateLimiter = new SlewRateLimiter(Constants.DrivetrainConstants.kStrafeSlewRate);

    private final SlewRateLimiter turnRateLimiter = new SlewRateLimiter(Constants.DrivetrainConstants.kTurnSlewRate);

    private double deadzone; 

    public enum Mode {
        NORMAL,
        SLOW
    }
    
    private Mode mode = Mode.NORMAL;

    private Supplier<ChassisSpeeds> chassisSpeedsSupplier = () -> null; 
    private Supplier<Double> maxSpeed = () -> mode == Mode.NORMAL ? Constants.DrivetrainConstants.kMaxSpeedMetersPerSecond : Constants.DrivetrainConstants.kMaxSlowSpeedMetersPerSecond;   
    private Supplier<Double> maxRotation = () -> mode == Mode.NORMAL ? Constants.DrivetrainConstants.kMaxRotationRadPerSecond : Constants.DrivetrainConstants.kMaxSlowRotationRadPerSecond; 

    public DriverController(int port) {
        this(port, 0.05); 
        
    }

    public DriverController(int port, double deadzone) {
        super(port); 
        this.deadzone = deadzone; 
    }


    // TODO: dunno if this should being in Controller or DriverController
    public double getLeftStickX() {
        return ControllerUtils.applyDeadband(super.getLeftStickX(), deadzone);
    }

    public double getLeftStickY() {
        return ControllerUtils.applyDeadband(super.getLeftStickY(), deadzone);
    }

    public double getRightStickX() {
        return ControllerUtils.applyDeadband(super.getRightStickX(), deadzone); 
    }

    public double getRightStickY() {
        return ControllerUtils.applyDeadband(super.getRightStickY(), deadzone);
    }

    public ChassisSpeeds getDesiredChassisSpeeds() {
        // deadzone-only values
        double throttleForward = -getLeftStickY();
        double throttleStrafe = -getLeftStickX();
        double throttleTurn = -getRightStickX(); 
        
        double speedForward = ControllerUtils.squareKeepSign(throttleForward) * maxSpeed.get(); 
        double speedStrafe = ControllerUtils.squareKeepSign(throttleStrafe) * maxSpeed.get(); 
        double speedTurn = ControllerUtils.squareKeepSign(throttleTurn) * maxRotation.get(); 

        // ChassisSpeeds speeds = new ChassisSpeeds(
        //     forwardRateLimiter.calculate(speedForward), 
        //     strafeRateLimiter.calculate(speedStrafe), 
        //     turnRateLimiter.calculate(speedTurn)
        // ); 
        ChassisSpeeds speeds = new ChassisSpeeds(
            forwardRateLimiter.calculate(speedForward), 
            strafeRateLimiter.calculate(speedStrafe), 
            turnRateLimiter.calculate(speedTurn)
        ); 

        // ChassisSpeeds oldSpeeds = chassisSpeedsSupplier.get(); 
        // if (oldSpeeds != null) {
        //     forwardRateLimiter.reset(oldSpeeds.vxMetersPerSecond);
        //     strafeRateLimiter.reset(oldSpeeds.vyMetersPerSecond);
        //     turnRateLimiter.reset(oldSpeeds.omegaRadiansPerSecond);
        // }

        return speeds;  
    }

    public ChassisState getDesiredChassisState() {
        // deadzone-only values
        double throttleForward = -getLeftStickY();
        double throttleStrafe = -getLeftStickX();
        double turnX = -getRightStickX(); 
        double turnY = -getRightStickY(); 
        
        double speedForward = ControllerUtils.squareKeepSign(throttleForward) * maxSpeed.get(); 
        double speedStrafe = ControllerUtils.squareKeepSign(throttleStrafe) * maxSpeed.get(); 
        // double speedTurn = ControllerUtils.squareKeepSign(throttleTurn) * maxRotation.get(); 

        // ChassisSpeeds speeds = new ChassisSpeeds(
        //     forwardRateLimiter.calculate(speedForward), 
        //     strafeRateLimiter.calculate(speedStrafe), 
        //     turnRateLimiter.calculate(speedTurn)
        // ); 
        ChassisState state = new ChassisState(
            forwardRateLimiter.calculate(speedForward), 
            strafeRateLimiter.calculate(speedStrafe), 
            Math.atan2(turnX, turnY), // speedTurn
            turnY != 0 || turnX != 0
        ); 

        // ChassisSpeeds oldSpeeds = chassisSpeedsSupplier.get(); 
        // if (oldSpeeds != null) {
        //     forwardRateLimiter.reset(oldSpeeds.vxMetersPerSecond);
        //     strafeRateLimiter.reset(oldSpeeds.vyMetersPerSecond);
        //     turnRateLimiter.reset(oldSpeeds.omegaRadiansPerSecond);
        // }

        return state;  
    }
    
    public Mode getSlowMode() {
        return this.mode; 
    }

    public void setSlowMode(Mode mode) {
        this.mode = mode; 
    }

    public void setChassisSpeedsSupplier(Supplier<ChassisSpeeds> s) {
        this.chassisSpeedsSupplier = s; 
    }

    public void setMaxSpeedSupplier(Supplier<Double> maxSpeed) {
        this.maxSpeed = maxSpeed; 
    }

    public void setMaxRotationSupplier(Supplier<Double> maxRotation) {
        this.maxRotation = maxRotation; 
    }
}
