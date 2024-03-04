// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Launcher;

public final class Autos {


    /** Example static factory for an autonomous command. */
    public static Command centerAuto(
          Drivetrain drivetrain, Launcher launcher, Intake intake) {
      CommandFactory factory = CommandFactory.getInstance(intake, drivetrain, launcher);
      return intake.getArmToStowCommand()
              .andThen(factory.shootNote())
              .andThen(
                      factory.intakeDown()

                      // Command Group - Drive forward, then back. Deadline with intake
                      .andThen(
                              new RunCommand(() -> drivetrain.rawArcadeDrive(0.3, 0), drivetrain)
                                      .withTimeout(1.5)
                                      .deadlineWith(intake.getIntakeCommand())
                              .andThen(factory.intakeUp())
                              .andThen(
                                      new RunCommand(() -> drivetrain.rawArcadeDrive(-0.32, 0), drivetrain)
                                              .withTimeout(1.5)
                              )
                      )
              )
              .andThen(factory.shootNote());
    }

    public static Command threeNoteCenter(
            Drivetrain drivetrain, Launcher launcher, Intake intake) {
        CommandFactory factory = CommandFactory.getInstance(intake, drivetrain, launcher);
        return centerAuto(drivetrain, launcher, intake)
                .andThen(new InstantCommand(drivetrain::zeroHeading))
                .andThen(
                        factory.intakeDown()

                        // Command Group - Drive forward, then back. Deadline with intake
                        .andThen(
                                new RunCommand(() -> drivetrain.rawArcadeDrive(0.3, 0), drivetrain)
                                        .withTimeout(1)
                                        .deadlineWith(intake.getIntakeCommand())
                                        .andThen(new TurnToAngle(90, drivetrain))
                                        .andThen(new RunCommand(() -> drivetrain.rawArcadeDrive(0.4, 0), drivetrain)
                                                .withTimeout(0.7))
                                        .andThen(new RunCommand(() -> drivetrain.rawArcadeDrive(-0.4, 0), drivetrain)
                                                .withTimeout(0.7)
                                                .deadlineWith(factory.intakeUp()))
                                        .andThen(new TurnToAngle(0, drivetrain))
                                        .andThen(
                                                new RunCommand(() -> drivetrain.rawArcadeDrive(-0.4, 0), drivetrain)
                                                        .withTimeout(1)
                                        )
                        )
                );
    }

    public static Command wideAuto(
            Drivetrain drivetrain, Launcher launcher, Intake intake, AllianceColor color) {
        CommandFactory factory = CommandFactory.getInstance(intake, drivetrain, launcher);
        double startAngle = color == AllianceColor.BLUE ? -60 : -120;
        double angle1 = color == AllianceColor.BLUE ? 0 : 180;
        double angle2 = color == AllianceColor.BLUE ? 17 : 163;
        double distance = 4.2; // speaker to pivot
        double noteDistance = 4.5; // pivot to note
        double noteDistance2 = 5; // pivot to note
        return new InstantCommand(() -> drivetrain.setHeading(startAngle))
                .andThen(factory.shootNote())
                .andThen(new DriveDistance(drivetrain, distance))
                .andThen(new TurnToAngle(angle1, drivetrain))
                .andThen(factory.intakeDown()
                    .andThen(
                            new DriveDistance(drivetrain, noteDistance)
                    ).deadlineWith(intake.getIntakeCommand())
                    .andThen(
                            new DriveDistance(drivetrain, -noteDistance)
                                    .deadlineWith(factory.intakeUp())
                    )
                )
                .andThen(new TurnToAngle(startAngle, drivetrain))
                .andThen(new DriveDistance(drivetrain, -distance))
                .andThen(factory.shootNote())
                .andThen(new DriveDistance(drivetrain, distance))
                .andThen(new TurnToAngle(angle2, drivetrain))
                .andThen(factory.intakeDown()
                        .andThen(
                                new DriveDistance(drivetrain, noteDistance2)
                        ).deadlineWith(intake.getIntakeCommand())
                        .andThen(
                                new DriveDistance(drivetrain, -noteDistance2)
                                        .deadlineWith(factory.intakeUp())
                        )
                )
                .andThen(new TurnToAngle(startAngle, drivetrain))
                .andThen(new DriveDistance(drivetrain, -distance))
                .andThen(factory.shootNote());
    }


    public static Command shootLeaveAuto(Drivetrain drivetrain, Launcher launcher, Intake intake){
        CommandFactory factory = CommandFactory.getInstance(intake, drivetrain, launcher);
        return intake.getArmToStowCommand()
                .andThen(factory.shootNote())
                .andThen(
                        new RunCommand(() -> drivetrain.rawArcadeDrive(0.3, 0), drivetrain)
                                .withTimeout(3)
                );
    }

    public static Command shootAuto(Drivetrain drivetrain, Launcher launcher, Intake intake){
      CommandFactory factory = CommandFactory.getInstance(intake, drivetrain, launcher);
      return intake.getArmToStowCommand()
              .andThen(factory.shootNote());
    }

    public static Command doNothing(){
        return new InstantCommand(() -> System.out.println("Doing nothing!"));
    }

    public static Command rotateAuto(Drivetrain drivetrain){
        return new InstantCommand(drivetrain::zeroHeading)
        .andThen(new TurnToAngle(30, drivetrain))
                .andThen(new TurnToAngle(-30, drivetrain))
                .andThen(new TurnToAngle(0, drivetrain));
    }

    public enum AllianceColor {
        RED, BLUE
    }



  private Autos() {
    throw new UnsupportedOperationException("This is a utility class!");
  }
}
