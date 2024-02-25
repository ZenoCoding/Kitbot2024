// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import static frc.robot.Constants.LauncherConstants.*;

public class Launcher extends SubsystemBase {
  private final CANSparkMax launchLeft;
  private final CANSparkMax launchRight;

  /** Creates a new Launcher. */
  public Launcher() {
    launchLeft = new CANSparkMax(LAUNCHER_LEFT_ID, MotorType.kBrushed);
    launchRight = new CANSparkMax(LAUNCHER_RIGHT_ID, MotorType.kBrushed);

    launchLeft.setInverted(LAUNCHER_LEFT_INVERTED);
    launchRight.setInverted(LAUNCHER_RIGHT_INVERTED);

    launchLeft.setSmartCurrentLimit(kLauncherCurrentLimit);
    launchRight.setSmartCurrentLimit(kLauncherCurrentLimit);
  }

  public Command getPrepareLauncherCommand() {
    return this.startEnd(() -> this.setLaunch(kLauncherSpeed), () -> {});
  }

  // An accessor method to set the speed (technically the output percentage) of the launch wheel
  public void setLaunch(double speed) {
    launchLeft.set(speed);
    launchRight.set(speed * 0.9);
  }

  // A helper method to stop both wheels. You could skip having a method like this and call the
  // individual accessors with speed = 0 instead
  public void stop() {
    launchLeft.set(0);
    launchRight.set(0);
  }
}
