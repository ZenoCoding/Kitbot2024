package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.PIDCommand;
import frc.robot.subsystems.Drivetrain;

import static frc.robot.Constants.DrivetrainConstants.*;

public class DriveDistance extends PIDCommand {
    private final Drivetrain drivetrain;

    public DriveDistance(Drivetrain drivetrain, double distance) {
        super(
                new PIDController(DRIVE_P, DRIVE_I, DRIVE_D),
                drivetrain::getPosition,
                distance,
                output -> drivetrain.rawArcadeDrive(output, 0),
                drivetrain);

        getController().setTolerance(DRIVE_TOLERANCE);
        this.drivetrain = drivetrain;
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        super.initialize();
        drivetrain.resetEncoders();
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        super.end(interrupted);
        drivetrain.rawArcadeDrive(0, 0);
    }

    @Override
    public boolean isFinished() {
        return getController().atSetpoint();
    }
}
