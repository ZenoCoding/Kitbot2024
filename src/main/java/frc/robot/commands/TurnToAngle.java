package frc.robot.commands;


import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.PIDCommand;
import frc.robot.subsystems.Drivetrain;

import static frc.robot.Constants.DrivetrainConstants.*;


/** A command that will turn the robot to the specified angle. */
public class TurnToAngle extends PIDCommand {

    private final double targetAngleDegrees;
    /**
     * Turns to robot to the specified angle.
     *
     * @param targetAngleDegrees The angle to turn to
     * @param drive The drive subsystem to use
     */
    public TurnToAngle(double targetAngleDegrees, Drivetrain drive) {
        super(
                new PIDController(TURN_P, TURN_I, TURN_D),
                // Close loop on heading
                drive::getHeading,
                // Set reference to target
                targetAngleDegrees,
                // Pipe output to turn robot
                output -> drive.rawArcadeDrive(0, output),
                // Require the drive
                drive);

        // Set the controller to be continuous (because it is an angle controller)
        getController().enableContinuousInput(-180, 180);
        // Set the controller tolerance - the delta tolerance ensures the robot is stationary at the
        // setpoint before it is considered as having reached the reference
        getController()
                .setTolerance(TURN_TOLERANCE_DEG, TURN_RATE_TOLERANCE_DEG_PER_S);
        this.targetAngleDegrees = targetAngleDegrees;
    }

    @Override
    public void initialize() {
        super.initialize();
        System.out.println("Moving to " + targetAngleDegrees);
    }

    @Override
    public void execute() {
        super.execute();
        if (getController().getP() != TURN_P
                || getController().getI() != TURN_I
                || getController().getD() != TURN_D) {
            getController().setPID(TURN_P, TURN_I, TURN_D);
        }

    }

    @Override
    public boolean isFinished() {
        // End when the controller is at the reference.
        return getController().atSetpoint();
    }
}