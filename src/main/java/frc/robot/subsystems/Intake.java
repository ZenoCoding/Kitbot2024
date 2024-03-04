package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.RelativeEncoder;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import java.util.function.Supplier;

import static frc.robot.Constants.IntakeConstants.*;

public class Intake extends SubsystemBase {

    private final CANSparkMax arm;
    private final CANSparkMax roller;
    private ArmFeedforward armFF = new ArmFeedforward(ARM_KS, ARM_G, ARM_V, ARM_A);
    private final PIDController armPID = new PIDController(ARM_P, ARM_I, ARM_D);
    private final RelativeEncoder armEncoder;
    private final DutyCycleEncoder throughBore;
    private double armSetpoint;

    public Intake() {
      this.arm = new CANSparkMax(INTAKE_ARM_ID, MotorType.kBrushless);
      this.roller = new CANSparkMax(INTAKE_ROLLER_ID, MotorType.kBrushless);
      arm.restoreFactoryDefaults();
      roller.restoreFactoryDefaults();

      arm.setIdleMode(CANSparkMax.IdleMode.kBrake);
      roller.setIdleMode(CANSparkMax.IdleMode.kCoast);

      arm.setInverted(ARM_INVERTED);
      roller.setInverted(ROLLER_INVERTED);

      arm.setSmartCurrentLimit(ARM_CURRENT_LIMIT);
      roller.setSmartCurrentLimit(ROLLER_CURRENT_LIMIT);

      throughBore = new DutyCycleEncoder(0);

      armEncoder = arm.getEncoder();
      armEncoder.setPositionConversionFactor(1.0 / 48.0);

      armPID.setIZone(ARM_IZ);

      resetEncoder();
      initTelemetry();

      setArmSetpoint(armEncoder.getPosition());
    }

    private void initTelemetry() {
      SmartDashboard.putNumber("PIDF_P", ARM_P);
      SmartDashboard.putNumber("PIDF_I", ARM_I);
      SmartDashboard.putNumber("PIDF_D", ARM_D);
      SmartDashboard.putNumber("PIDF_IZ", ARM_IZ);
      SmartDashboard.putNumber("PIDF_KS", ARM_KS);
      SmartDashboard.putNumber("PIDF_G", ARM_G);
      SmartDashboard.putNumber("PIDF_V", ARM_V);
      SmartDashboard.putNumber("PIDF_A", ARM_A);
      SmartDashboard.putNumber("SetPoint", armSetpoint);
    }

    private void updateTelemetry() {
      ARM_P = SmartDashboard.getNumber("PIDF_P", ARM_P);
      ARM_I = SmartDashboard.getNumber("PIDF_I", ARM_I);
      ARM_D = SmartDashboard.getNumber("PIDF_D", ARM_D);
      ARM_IZ = SmartDashboard.getNumber("PIDF_IZ", ARM_IZ);
      ARM_KS = SmartDashboard.getNumber("PIDF_KS", ARM_KS);
      ARM_G = SmartDashboard.getNumber("PIDF_G", ARM_G);
      ARM_V = SmartDashboard.getNumber("PIDF_V", ARM_V);
      ARM_A = SmartDashboard.getNumber("PIDF_A", ARM_A);
      if (armPID.getP() != ARM_P
          || armPID.getI() != ARM_I
          || armPID.getD() != ARM_D
          || armPID.getIZone() != ARM_IZ) {
        System.out.println("Setting new PID");
        armPID.setPID(ARM_P, ARM_I, ARM_D);
        armPID.setIZone(ARM_IZ);
      }
      if (armFF.kg != ARM_G || armFF.kv != ARM_V || armFF.ka != ARM_A || armFF.ks != ARM_KS)
        armFF = new ArmFeedforward(ARM_KS, ARM_G, ARM_V, ARM_A);

      armSetpoint = SmartDashboard.getNumber("SetPoint", armSetpoint);
    }

    @Override
    public void periodic() {
      SmartDashboard.putNumber("Absolute Encoder Position", throughBore.getAbsolutePosition());
      SmartDashboard.putNumber("Encoder Position", armEncoder.getPosition());
      updateTelemetry();
      setArmPosition(armSetpoint);
    }

    public Command getArmCommand(Supplier<Double> rotationSupplier) {
      return getArmCommand(rotationSupplier, ARM_TOLERANCE);
    }

    public Command getArmCommand(Supplier<Double> rotationSupplier, double tolerance) {
      return new Command() {
        @Override
        public void initialize() {
          // Initialization code here
          SmartDashboard.putNumber("Count", 0);
          setArmSetpoint(rotationSupplier.get());
        }

        @Override
        public boolean isFinished() { // if encoder is within a certain range of the setpoint
          return Math.abs(armEncoder.getPosition() - rotationSupplier.get()) < tolerance;
        }
      };
    }

    public Command getArmToGroundCommand() {
      return getArmCommand(() -> ARM_GROUND_POSITION, 0.1);
    }

    public Command getArmToStowCommand() {
      return getArmCommand(() -> ARM_STOW_POSITION);
    }

    public Command getArmToAmpCommand() {
      return getArmCommand(() -> ARM_AMP_POSITION, 0.005);
    }

    public Command getArmToAmpSalvageCommand() {
      return getArmCommand(() -> ARM_AMP_SALVAGE, 0.005);
    }

    public Command getIntakeCommand() {
      return this.startEnd(
          () -> {
            setRoller(ROLLER_INTAKE_SPEED);
            System.out.println("Intaking...");
          }, this::stopRoller);
    }

    public Command getOuttakeCommand() {
      return this.startEnd(() -> setRoller(ROLLER_OUTTAKE_SPEED), this::stopRoller);
    }

    public Command shootIntake() {
      return this.startEnd(() -> setRoller(ROLLER_SHOOT_SPEED), this::stopRoller);
    }

    public Command fastIntake() {
      return this.startEnd(() -> setRoller(ROLLER_RECOVER_SPEED), this::stopRoller);
    }

    public void setArmSetpoint(double rotations) {
      this.armSetpoint = rotations;
      SmartDashboard.putNumber("SetPoint", rotations);
    }

    private void setArmPosition(double rotations) {
      double output =
          armPID.calculate(getArmPosition(), rotations)
              + armFF.calculate(getArmPosition() * 2 * Math.PI, 0); //
      arm.set(output);
      SmartDashboard.putNumber("Error", getArmPosition() - rotations);
      SmartDashboard.putNumber("Rotations", rotations);
      SmartDashboard.putNumber("Output", output);
      SmartDashboard.putNumber("Count", SmartDashboard.getNumber("Count", 0) + 1);
    }

    public void resetEncoder() {
      armEncoder.setPosition(getAbsolutePosition());
    }

    public double getAbsolutePosition() {
      double val = throughBore.getAbsolutePosition() + ARM_ABSOLUTE_OFFSET;
      if (val > 0.7) return val - 1;
      return val;
    }

    public double getArmPosition() {
      return armEncoder.getPosition();
    }

    private void stopArm() {
      arm.stopMotor();
    }

    public void setRoller(double speed) {
      roller.set(speed);
    }

    public void stopRoller() {
      roller.set(0);
    }

    public void stop() {
      arm.stopMotor();
      roller.set(0);
    }
}
