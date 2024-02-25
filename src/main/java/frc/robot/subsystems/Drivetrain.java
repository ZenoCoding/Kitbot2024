// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.RelativeEncoder;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.PIDSubsystem;

import static frc.robot.Constants.DrivetrainConstants.*;

/* This class declares the subsystem for the robot drivetrain if controllers are connected via CAN. Make sure to go to
 * RobotContainer and uncomment the line declaring this subsystem and comment the line for CanDrivetrain.
 *
 * The subsystem contains the objects for the hardware contained in the mechanism and handles low level logic
 * for control. Subsystems are a mechanism that, when used in conjuction with command "Requirements", ensure
 * that hardware is only being used by 1 command at a time.
 */
public class Drivetrain extends PIDSubsystem {
    /*Class member variables. These variables represent things the class needs to keep track of and use between
    different method calls. */
    private final DifferentialDrive m_drivetrain;
    private final CANSparkMax leftFront;
    private final CANSparkMax leftRear;
    private final CANSparkMax rightFront;
    private final CANSparkMax rightRear;
    private final RelativeEncoder leftEncoder;
    private final RelativeEncoder rightEncoder;

    /*Constructor. This method is called when an instance of the class is created. This should generally be used to set up
     * member variables and perform any configuration or set up necessary on hardware.
     */
    public Drivetrain() {
        super(new PIDController(kP, kI, kD));
        getController().setTolerance(0.05);

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
        leftFront.setIdleMode(CANSparkMax.IdleMode.kCoast);
        rightFront.setIdleMode(CANSparkMax.IdleMode.kCoast);
        leftRear.setIdleMode(CANSparkMax.IdleMode.kCoast);
        rightRear.setIdleMode(CANSparkMax.IdleMode.kCoast);

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
        SmartDashboard.putNumber("DRIVE_PIDF_P", kP);
        SmartDashboard.putNumber("DRIVE_PIDF_I", kI);
        SmartDashboard.putNumber("DRIVE_PIDF_D", kD);
        SmartDashboard.putNumber("Drive SetPoint", getSetpoint());
    }

    private void updateTelemetry() {
        kP = SmartDashboard.getNumber("DRIVE_PIDF_P", kP);
        kI = SmartDashboard.getNumber("DRIVE_PIDF_I", kI);
        kD = SmartDashboard.getNumber("DRIVE_PIDF_D", kD);
        if (getController().getP() != kP
                || getController().getI() != kI
                || getController().getD() != kD) {
          System.out.println("Setting new PID");
          getController().setPID(kP, kI, kD);
        }

        if (SmartDashboard.getNumber("Drive SetPoint", getSetpoint()) != getSetpoint()) {
            setpoint(SmartDashboard.getNumber("Drive SetPoint", getSetpoint()));
            enable();
        }
    }


    public void setpoint(double setpoint) {
        SmartDashboard.putNumber("Drive SetPoint", setpoint);
        setSetpoint(setpoint);
    }

    @Override
    public void periodic() {
        super.periodic();
        SmartDashboard.putNumber("Drive Encoder Position", getMeasurement());
        updateTelemetry();
    }

    public void resetEncoders(){
        leftEncoder.setPosition(0);
        rightEncoder.setPosition(0);
        System.out.println("Left: " + leftEncoder.getPosition());
        System.out.println("Right: " + rightEncoder.getPosition());
        assert getMeasurement() == 0;
    }

    public boolean atSetpoint() {
        return getController().atSetpoint();
    }


    /*Method to control the drivetrain using arcade drive. Arcade drive takes a speed in the X (forward/back) direction
     * and a rotation about the Z (turning the robot about it's center) and uses these to control the drivetrain motors */
    public void arcadeDrive(double speed, double rotation) {
        disable();
        speed = Math.pow(speed, 3);
        rotation = Math.pow(rotation, 3);
        m_drivetrain.arcadeDrive(speed, rotation, false);
    }

    public void rawArcadeDrive(double speed, double rotation){
        m_drivetrain.arcadeDrive(speed, rotation, false);
    }

    @Override
    protected void useOutput(double output, double setpoint) {
        SmartDashboard.putNumber("DriveTrain Output: ", output);
        m_drivetrain.arcadeDrive(output, 0, false);
    }

    @Override
    public double getMeasurement() {
        return (leftEncoder.getPosition() + rightEncoder.getPosition())/2;
    }
}
