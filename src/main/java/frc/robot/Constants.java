// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
  public static class OperatorConstants {
    // Port numbers for driver and operator gamepads. These correspond with the numbers on the USB
    // tab of the DriverStation
    public static final int kDriverControllerPort = 0;
    public static final int kOperatorControllerPort = 1;
  }

  public static class DrivetrainConstants {
    // PWM ports/CAN IDs for motor controllers
    public static final int kLeftRearID = 3;
    public static final int kLeftFrontID = 1;
    public static final int kRightRearID = 4;
    public static final int kRightFrontID = 2;

    // Current limit for drivetrain motors
    public static final int STALL_CURRENT_LIMIT = 40;
    public static final int FREE_CURRENT_LIMIT = 20;
    public static final int SECONDARY_CURRENT_LIMIT = 60;

    // Encoder conversion factors
    public static final double WHEEL_DIAMETER = 0.15;
    public static final double GEAR_RATIO = 1/8.46;
    public static final double ENCODER_CONVERSION_FACTOR = WHEEL_DIAMETER * Math.PI * GEAR_RATIO;

    // PID constants for the drivetrain
    public static double DRIVE_P = 0.15;
    public static double DRIVE_I = 0.1;
    public static double DRIVE_D = 0.05;
    public static double DRIVE_TOLERANCE = 0.05;

    // Turning Constants
    public static double TURN_P = 0.0065;
    public static double TURN_I = 0.002;
    public static double TURN_D = 0.001;

    public static final double TURN_TOLERANCE_DEG = 2;
    public static final double TURN_RATE_TOLERANCE_DEG_PER_S = 3;

    // Stabilization Constants
    public static final double STABILIZATION_P = 0.0065;
    public static final double STABILIZATION_I = 0.002;
    public static final double STABILIZATION_D = 0.0;


  }

  public static class LauncherConstants {
    // PWM ports/CAN IDs for motor controllers
    public static final int LAUNCHER_RIGHT_ID = 5;
    public static final int LAUNCHER_LEFT_ID = 6;

    // Inversion
    public static final boolean LAUNCHER_LEFT_INVERTED = true;
    public static final boolean LAUNCHER_RIGHT_INVERTED = false;

    // Current limit for launcher and feed wheels
    public static final int kLauncherCurrentLimit = 80;

    // Speeds for wheels when intaking and launching. Intake speeds are negative to run the wheels
    // in reverse
    public static final double kLauncherSpeed = 1;
    public static final double kIntakeLauncherSpeed = -1;

    public static final double kLauncherDelay = 0.7;
  }

  public static class IntakeConstants {
    public static final int INTAKE_ARM_ID = 7;
    public static final int INTAKE_ROLLER_ID = 8;

    public static final boolean ARM_INVERTED = false;
    public static final boolean ROLLER_INVERTED = false;

    public static final int ARM_CURRENT_LIMIT = 40;
    public static final int ROLLER_CURRENT_LIMIT = 20;

    public static double ARM_P = 1.5;
    public static double ARM_I = 1;
    public static double ARM_D = 0.1;
    public static double ARM_IZ = 0.05;
    public static double ARM_KS = 0.05;
    public static double ARM_G = 0.04;
    public static double ARM_V = 0.095;
    public static double ARM_A = 0.0;

    public static final double ROLLER_INTAKE_SPEED = 0.6;
    public static final double ROLLER_OUTTAKE_SPEED = -0.2;
    public static double ROLLER_SHOOT_SPEED = -1;
    public static double ROLLER_RECOVER_SPEED = 1;
    public static final double ARM_GROUND_POSITION = -0.1;
    // public static final double ARM_INTAKE_POSITION = 0.5;
    public static final double ARM_STOW_POSITION = 0.48;
    public static final double ARM_AMP_POSITION = 0.22;
    public static final double ARM_AMP_SALVAGE = 0.20;
    public static final double ARM_TOLERANCE = 0.05;
    public static final double ARM_ABSOLUTE_OFFSET = -0.43;
  }
}
