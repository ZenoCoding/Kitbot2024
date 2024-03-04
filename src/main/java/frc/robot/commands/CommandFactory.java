package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Launcher;

public class CommandFactory {
    public static CommandFactory instance;
    private final Intake intake;
    private final Drivetrain drivetrain;
    private final Launcher launcher;
    public CommandFactory(Intake intake, Drivetrain drivetrain, Launcher launcher) {
        this.intake = intake;
        this.drivetrain = drivetrain;
        this.launcher = launcher;
    }

    public static CommandFactory getInstance(Intake intake, Drivetrain drivetrain, Launcher launcher){
        if (instance == null)
            instance = new CommandFactory(intake, drivetrain, launcher);
        return instance;
    }

    public Command shootNote(){
        return launcher
                .getPrepareLauncherCommand()
                .withTimeout(Constants.LauncherConstants.kLauncherDelay)
                .andThen(intake.getOuttakeCommand())
                .handleInterrupt(launcher::stop)
                .withTimeout(2);
    }

    public Command intakeDownAndRoll(){
        return intake.getArmToGroundCommand()
                .andThen(intake.getIntakeCommand());
    }

    public Command intakeDown(){
        return intake.getArmToGroundCommand();
    }


    public Command intakeUp(){
        return intake
                .getArmToStowCommand()
                .deadlineWith(intake.getIntakeCommand())
                .finallyDo(intake::stopRoller);
    }
}
