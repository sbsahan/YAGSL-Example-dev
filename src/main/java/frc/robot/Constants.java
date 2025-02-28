// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;
public final class Constants
{

  public static final double ROBOT_MASS = 37;
  public static final double LOOP_TIME  = 0.13; 
  public static final double MAX_SPEED  = 1.8;

//  public static final class AutonConstants
//  {
//
//    public static final PIDConstants TRANSLATION_PID = new PIDConstants(0.7, 0, 0);
//    public static final PIDConstants ANGLE_PID       = new PIDConstants(0.4, 0, 0.01);
//  }

  public static final class DrivebaseConstants
  {
    public static final double WHEEL_LOCK_TIME = 10; // seconds
  }

  public static class OperatorConstants
  {
    public static final double DEADBAND        = 0.1;
    public static final double LEFT_Y_DEADBAND = 0.1;
    public static final double RIGHT_X_DEADBAND = 0.1;
    public static final double TURN_CONSTANT    = 6;
  }

  public static class LevelConstants
  {
    public static final double L0 = 0;
    public static final double L1 = 10;
    public static final double L2 = 20;
    public static final double L3 = 30;
  }

  public static class VisionConstants
  {
    public static final double TX_GOAL_RIGHT = -17;
    public static final double TX_GOAL_LEFT = -17;
    public static final double TA_GOAL = 6;
    public static final double kP = 10;
  }

  public static class IntakeConstants
  {
    public static final double INTAKE_ARM_POSITION = 1;
    public static final double INTAKE_RESET_POSITION = 0.5;
  }
  
}
