// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkFlex;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Shooter extends SubsystemBase {
  private CANSparkFlex m_topMotor = new CANSparkFlex(8, MotorType.kBrushless);
  private CANSparkFlex m_bottomMotor = new CANSparkFlex(7, MotorType.kBrushless);

  private double topSpeed = 1;
  private double bottomSpeed = 1;
  /** Creates a new Shooter. */
  public Shooter() {}

  public Command intake() {
    return startEnd(() -> {
      m_topMotor.set(-.5);
      m_bottomMotor.set(-.5);
    }, () -> {m_topMotor.set(0);m_bottomMotor.set(0);});
  }
  public Command run() {
    Timer topBottomDelay = new Timer();
    return new FunctionalCommand(
    () -> {
      m_topMotor.set(topSpeed);
      topBottomDelay.restart();
    },
     () -> {
        if (topBottomDelay.hasElapsed(.2)) {
          m_bottomMotor.set(bottomSpeed);
        }
     },
     b -> {},
     () -> false,
     this);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
