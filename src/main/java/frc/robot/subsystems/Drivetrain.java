// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.kauailabs.navx.frc.AHRS;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.RelativeEncoder;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import static frc.robot.Constants.DrivetrainConstants.*;

/* This class declares the subsystem for the robot drivetrain if controllers are connected via CAN. Make sure to go to
 * RobotContainer and uncomment the line declaring this subsystem and comment the line for CanDrivetrain.
 *
 * The subsystem contains the objects for the hardware contained in the mechanism and handles low level logic
 * for control. Subsystems are a mechanism that, when used in conjuction with command "Requirements", ensure
 * that hardware is only being used by 1 command at a time.
 */
public class Drivetrain extends SubsystemBase {
    /*Class member variables. These variables represent things the class needs to keep track of and use between
    different method calls. */
    private final DifferentialDrive m_drivetrain;
    private final CANSparkMax leftFront;
    private final CANSparkMax leftRear;
    private final CANSparkMax rightFront;
    private final CANSparkMax rightRear;
    private final RelativeEncoder leftEncoder;
    private final RelativeEncoder rightEncoder;
    private final AHRS gyro = new AHRS(SPI.Port.kMXP);

    /*Constructor. This method is called when an instance of the class is created. This should generally be used to set up
     * member variables and perform any configuration or set up necessary on hardware.
     */
    public Drivetrain() {

        leftFront = new CANSparkMax(kLeftFrontID, MotorType.kBrushless);
        leftRear = new CANSparkMax(kLeftRearID, MotorType.kBrushless);
        rightFront = new CANSparkMax(kRightFrontID, MotorType.kBrushless);
        rightRear = new CANSparkMax(kRightRearID, MotorType.kBrushless);

        leftEncoder = leftFront.getEncoder();
        rightEncoder = rightFront.getEncoder();

        leftEncoder.setPositionConversionFactor(ENCODER_CONVERSION_FACTOR);
        rightEncoder.setPositionConversionFactor(ENCODER_CONVERSION_FACTOR);

        /*Sets current limits for the drivetrain motors. This helps reduce the likelihood of wheel spin, reduces motor heating
         *at stall (Drivetrain pushing against something) and helps maintain battery voltage under heavy demand */
        leftFront.setSmartCurrentLimit(STALL_CURRENT_LIMIT, FREE_CURRENT_LIMIT);
        leftFront.setSecondaryCurrentLimit(SECONDARY_CURRENT_LIMIT);
        leftRear.setSmartCurrentLimit(STALL_CURRENT_LIMIT, FREE_CURRENT_LIMIT);
        leftRear.setSecondaryCurrentLimit(SECONDARY_CURRENT_LIMIT);
        rightFront.setSmartCurrentLimit(STALL_CURRENT_LIMIT, FREE_CURRENT_LIMIT);
        rightFront.setSecondaryCurrentLimit(SECONDARY_CURRENT_LIMIT);
        rightRear.setSmartCurrentLimit(STALL_CURRENT_LIMIT, FREE_CURRENT_LIMIT);
        rightRear.setSecondaryCurrentLimit(SECONDARY_CURRENT_LIMIT);

        // Set idle modes of motors
        leftFront.setIdleMode(CANSparkMax.IdleMode.kBrake);
        rightFront.setIdleMode(CANSparkMax.IdleMode.kBrake);
        leftRear.setIdleMode(CANSparkMax.IdleMode.kBrake);
        rightRear.setIdleMode(CANSparkMax.IdleMode.kBrake);

        // Set the rear motors to follow the front motors.
        leftRear.follow(leftFront);
        rightRear.follow(rightFront);

        // Invert the left side so both side drive forward with positive motor outputs
        leftFront.setInverted(false);
        rightFront.setInverted(true);

        // Put the front motors into the differential drive object. This will control all 4 motors with
        // the rears set to follow the fronts
        m_drivetrain = new DifferentialDrive(leftFront, rightFront);
        initTelemetry();
    }

    private void initTelemetry() {
        SmartDashboard.putNumber("DRIVE_PIDF_P", DRIVE_P);
        SmartDashboard.putNumber("DRIVE_PIDF_I", DRIVE_I);
        SmartDashboard.putNumber("DRIVE_PIDF_D", DRIVE_D);
        SmartDashboard.putNumber("TURN_PIDF_P", TURN_P);
        SmartDashboard.putNumber("TURN_PIDF_I", TURN_I);
        SmartDashboard.putNumber("TURN_PIDF_D", TURN_D);
    }

    private void updateTelemetry() {
        DRIVE_P = SmartDashboard.getNumber("DRIVE_PIDF_P", DRIVE_P);
        DRIVE_I = SmartDashboard.getNumber("DRIVE_PIDF_I", DRIVE_I);
        DRIVE_D = SmartDashboard.getNumber("DRIVE_PIDF_D", DRIVE_D);
        TURN_P = SmartDashboard.getNumber("TURN_PIDF_P", TURN_P);
        TURN_I = SmartDashboard.getNumber("TURN_PIDF_I", TURN_I);
        TURN_D = SmartDashboard.getNumber("TURN_PIDF_D", TURN_D);


        SmartDashboard.putNumber("heading", getHeading());
    }


    public void periodic() {
        SmartDashboard.putNumber("Drive Encoder Position", getPosition());
        updateTelemetry();
    }

    public void resetEncoders(){
        leftEncoder.setPosition(0);
        rightEncoder.setPosition(0);
        System.out.println("Left: " + leftEncoder.getPosition());
        System.out.println("Right: " + rightEncoder.getPosition());
        assert getPosition() == 0;
    }


    /*Method to control the drivetrain using arcade drive. Arcade drive takes a speed in the X (forward/back) direction
     * and a rotation about the Z (turning the robot about it's center) and uses these to control the drivetrain motors */
    public void arcadeDrive(double speed, double rotation) {
        speed = Math.pow(speed, 3);
        rotation = Math.pow(rotation, 3);
        m_drivetrain.arcadeDrive(speed, rotation, false);
    }

    public void arcadeDriveRawTurning(double speed, double rotation){
        speed = Math.pow(speed, 3);
        m_drivetrain.arcadeDrive(speed, rotation);
    }

    public void rawArcadeDrive(double speed, double rotation){
        m_drivetrain.arcadeDrive(speed, rotation, false);
    }

    public double getPosition() {
        return (leftEncoder.getPosition() + rightEncoder.getPosition())/2;
    }

    public double getHeading(){
        return gyro.getYaw() * -1;
    }

    public double getTurnRate() {
        return gyro.getRate()*-1;
    }

    public void zeroHeading() {
        gyro.zeroYaw();
    }

    public void setHeading(double angle) {
        gyro.zeroYaw();
        gyro.setAngleAdjustment(angle);
    }
}
