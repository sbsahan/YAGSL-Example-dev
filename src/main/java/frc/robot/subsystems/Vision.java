package frc.robot.subsystems;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.LimelightHelpers;

public class Vision extends SubsystemBase {
    private final String limelightName;
    private final String side;
   
    // Raw limelight values
    private double tx;
    private double ty;
    private double ta;
   
    // The calibrated values for alignment positions (will be set during calibration)
    private static final double TX_LEFT_POSITION = -14.1;  // REPLACE with measured value
    private static final double TX_RIGHT_POSITION = 1.9;  // REPLACE with measured value
    private static final double TA_TARGET_DISTANCE = 1.45; // REPLACE with measured value
   
    public Vision(String name, String side) {
        this.limelightName = name;
        this.side = side;
    }
   
    @Override
    public void periodic() {
        // Update values every periodic cycle
        updateValues();
       
        // Publish to SmartDashboard for debugging
        SmartDashboard.putNumber(side + "_tx", tx);
        SmartDashboard.putNumber(side + "_ty", ty);
        SmartDashboard.putNumber(side + "_ta", ta);
        SmartDashboard.putBoolean(side + "_hasTarget", hasTarget());
    }
   
    public void updateValues() {
        tx = LimelightHelpers.getTX(limelightName);
        ty = LimelightHelpers.getTY(limelightName);
        ta = LimelightHelpers.getTA(limelightName);
    }
   
    public boolean hasTarget() {
        return LimelightHelpers.getTV(limelightName);
    }
   
    public double getTx() {
        return tx;
    }
   
    public double getTy() {
        return ty;
    }
   
    public double getTa() {
        return ta;
    }
   
    public String getLimelightName() {
        return limelightName;
    }
   
    // Get the target tx value for a specific position
    public static double getTargetTx(String position) {
        if ("left".equals(position)) {
            return TX_LEFT_POSITION;
        } else if ("right".equals(position)) {
            return TX_RIGHT_POSITION;
        } else {
            return 0.0; // Center
        }
    }
   
    // Get the target area for the desired distance
    public static double getTargetArea() {
        return TA_TARGET_DISTANCE;
    }
}