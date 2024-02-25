package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Drivetrain;

public class DriveDistance extends Command {
    private final double distance;
    private final Drivetrain drivetrain;

    public DriveDistance(Drivetrain drivetrain, double distance) {
        // Use addRequirements() here to declare subsystem dependencies.
        // Configure additional PID options by calling getController() and getPIDController()
        this.drivetrain = drivetrain;
        this.distance = distance;
        addRequirements(drivetrain);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        drivetrain.enable();
        System.out.println("Starting Distance: " + drivetrain.getMeasurement());
        drivetrain.resetEncoders();
        System.out.println("Current Distance: "  + drivetrain.getMeasurement());
        drivetrain.setpoint(distance);
        System.out.println("Setpoint: " + drivetrain.getSetpoint());
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        System.out.println("Finished! Interrupted: " + interrupted);
        System.out.println("Position: " + drivetrain.getMeasurement() + " Setpoint: " + drivetrain.getSetpoint());

        drivetrain.rawArcadeDrive(0, 0);
        drivetrain.disable();
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return drivetrain.atSetpoint();
    }
}
