// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.PIDCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants.OperatorConstants;
import frc.robot.commands.Autos;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Launcher;

import static frc.robot.Constants.DrivetrainConstants.*;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
    // The robot's subsystems are defined here.
    private final Drivetrain drivetrain = new Drivetrain();
    private final Launcher launcher = new Launcher();
    private final Intake intake = new Intake();
    SendableChooser<Command> autoChooser = new SendableChooser<>();

    /*The gamepad provided in the KOP shows up like an XBox controller if the mode switch is set to X mode using the
     * switch on the top.*/
    private final CommandXboxController driverController =
        new CommandXboxController(OperatorConstants.kDriverControllerPort);

    private final CommandXboxController operatorController = new CommandXboxController(OperatorConstants.kOperatorControllerPort);

    /** The container for the robot. Contains subsystems, OI devices, and commands. */
    public RobotContainer() {
      // Configure the trigger bindings
      configureBindings();
      autoChooser.setDefaultOption("Two Note Center", Autos.centerAuto(drivetrain, launcher, intake));
      autoChooser.addOption("Shoot and Leave", Autos.shootLeaveAuto(drivetrain, launcher, intake));
      autoChooser.addOption("Shoot", Autos.shootAuto(drivetrain, launcher, intake));
      autoChooser.addOption("Do Nothing", Autos.doNothing());
      autoChooser.addOption("BLUE Wide Auto", Autos.wideAuto(drivetrain, launcher, intake, Autos.AllianceColor.BLUE));
      autoChooser.addOption("RED Wide Auto", Autos.wideAuto(drivetrain, launcher, intake, Autos.AllianceColor.RED));
      autoChooser.addOption("Rotation Show", Autos.rotateAuto(drivetrain));
      SmartDashboard.putData(autoChooser);
    }

    /**
     * Use this method to define your trigger->command mappings. Triggers can be accessed via the
     * named factory methods in the Command* classes in edu.wpi.first.wpilibj2.command.button (shown
     * below) or via the Trigger constructor for arbitary conditions
     */
    private void configureBindings() {
        // Set the default command for the drivetrain to drive using the joysticks
        // Commenting out this line of code makes autonomous work.
        drivetrain.setDefaultCommand(
            new RunCommand(
                () ->
                    drivetrain.arcadeDrive(
                        -driverController.getLeftY(), -driverController.getRightX()),
                drivetrain));

        // Self correcting drive command when the right bumper is held
        driverController
            .rightBumper()
                    .whileTrue(
                        new PIDCommand(
                                new PIDController(
                                        STABILIZATION_P,
                                        STABILIZATION_I,
                                        STABILIZATION_D),
                                drivetrain::getTurnRate,
                                0,
                                (double output) -> drivetrain.arcadeDriveRawTurning(-driverController.getLeftY(), output),
                                drivetrain));

        // Set up a binding to run the intake command while the operator is pressing and holding the
        // left Bumper
        driverController
            .leftBumper()
            .whileTrue(
                    intake.getArmToGroundCommand()
                    .andThen(intake.getIntakeCommand())
            )
            .onFalse(
                intake
                    .getArmToStowCommand()
                    .deadlineWith(intake.getIntakeCommand())
                    .finallyDo(intake::stopRoller));

        /*Create an inline sequence to run when the operator presses and holds the A (green) button. Run the PrepareLaunch
         * command for 1 seconds and then run the LaunchNote command */
        driverController
            .rightTrigger()
            .whileTrue(
                launcher
                    .getPrepareLauncherCommand()
                    .withTimeout(Constants.LauncherConstants.kLauncherDelay)
                    .andThen(intake.getOuttakeCommand())
                    .handleInterrupt(launcher::stop));

        driverController
            .leftTrigger()
            .whileTrue(
                intake.getArmToAmpCommand()
                    .andThen(intake.shootIntake().withTimeout(0.5))
                    .andThen(
                            intake.getArmToAmpSalvageCommand()
                                    .deadlineWith(intake.getIntakeCommand())
                    )
            )
            .onFalse(intake.getArmToStowCommand().finallyDo(intake::stopRoller));

        driverController
            .y()
            .onTrue(
                new InstantCommand(
                    () -> {
                      intake.resetEncoder();
                      System.out.println("Resetting Arm Encoder...");
                    },
                    intake));

        driverController.x().whileTrue(intake.shootIntake().handleInterrupt(intake::stopRoller));

        driverController.b().whileTrue(intake.getIntakeCommand().handleInterrupt(intake::stopRoller));

        driverController.y().whileTrue(intake.fastIntake().handleInterrupt(intake::stopRoller));

        operatorController.rightBumper().onTrue(new InstantCommand(intake::resetEncoder));
        operatorController.leftBumper().onTrue(new InstantCommand(intake::resetEncoder));
    }

    /**
     * Use this to pass the autonomous command to the main {@link Robot} class.
     *
     * @return the command to run in autonomous
     */
    public Command getAutonomousCommand() {

        return autoChooser.getSelected();
    }
}
