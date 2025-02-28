package frc.robot.commands;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Vision;
import frc.robot.subsystems.swervedrive.SwerveSubsystem;

public class SimpleAprilTagAlign extends Command {
    private final SwerveSubsystem swerve;
    private final Vision vision;
    private final String position; // "left" or "right"
   
    // Target values will come from Vision subsystem
    private final double targetTx;
    private final double targetTa = Vision.getTargetArea();
   
    // Simple proportional gains
    private final double kP_Rotation = 0.03;  // Rotation control gain
    private final double kP_Forward = 0.5;   // Forward/backward control gain
    private final double kP_Strafe = 0.03;   // Left/right control gain
   
    // Tolerances for alignment
    private final double TX_TOLERANCE = 1.0;  // degrees
    private final double TA_TOLERANCE = 0.1;  // percentage units
   
    // Counters for stability detection
    private int stableCount = 0;
    private final int REQUIRED_STABLE_CYCLES = 10;
   
    public SimpleAprilTagAlign(SwerveSubsystem swerve, Vision vision, String position) {
        this.swerve = swerve;
        this.vision = vision;
        this.position = position;
       
        // Get the target tx for the specified position
        this.targetTx = Vision.getTargetTx(position);
       
        addRequirements(swerve, vision);
    }
   
    @Override
    public void initialize() {
        System.out.println("Starting alignment to " + position + " position");
        stableCount = 0;
    }
   
    @Override
    public void execute() {
        // Make sure we have a target
        if (!vision.hasTarget()) {
            swerve.drive(new Translation2d(0, 0), 0, false);
            stableCount = 0;
            SmartDashboard.putBoolean("Aligned", false);
            return;
        }
       
        // Get current values
        double currentTx = vision.getTx();
        double currentTa = vision.getTa();
       
        // Calculate simple proportional errors
        double txError = targetTx - currentTx;
        double taError = targetTa - currentTa;
       
        // Calculate motor outputs with simple proportional control
        double rotationSpeed = txError * kP_Rotation;  // Control rotation
        double forwardSpeed = taError * kP_Forward;    // Control distance
       
        // Limit speeds for safety
        rotationSpeed = limitValue(rotationSpeed, -0.3, 0.3);
        forwardSpeed = limitValue(forwardSpeed, -0.3, 0.3);
       
        // Send commands to swerve drive (robot-relative)
        swerve.drive(new Translation2d(-forwardSpeed, 0), rotationSpeed, false);
       
        // Debug output
        SmartDashboard.putNumber("Target Tx", targetTx);
        SmartDashboard.putNumber("Current Tx", currentTx);
        SmartDashboard.putNumber("Tx Error", txError);
        SmartDashboard.putNumber("Target Ta", targetTa);
        SmartDashboard.putNumber("Current Ta", currentTa);
        SmartDashboard.putNumber("Ta Error", taError);
        SmartDashboard.putNumber("Rotation Speed", rotationSpeed);
        SmartDashboard.putNumber("Forward Speed", forwardSpeed);
       
        // Check if we're aligned
        boolean aligned = (Math.abs(txError) < TX_TOLERANCE) &&
                          (Math.abs(taError) < TA_TOLERANCE);
       
        if (aligned) {
            stableCount++;
        } else {
            stableCount = 0;
        }
       
        SmartDashboard.putBoolean("Aligned", stableCount >= REQUIRED_STABLE_CYCLES);
    }
   
    @Override
    public boolean isFinished() {
        // Command is done when we've been stable for enough cycles
        return stableCount >= REQUIRED_STABLE_CYCLES;
    }
   
    @Override
    public void end(boolean interrupted) {
        // Stop the robot
        swerve.drive(new Translation2d(0, 0), 0, false);
        System.out.println("Alignment " + (interrupted ? "interrupted" : "completed"));
    }
   
    // Helper method to limit values
    private double limitValue(double value, double min, double max) {
        if (value < min) return min;
        if (value > max) return max;
        return value;
    }
}