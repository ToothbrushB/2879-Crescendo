// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkBase;
import com.revrobotics.CANSparkFlex;
import com.revrobotics.CANSparkLowLevel.MotorType;

import com.revrobotics.CANSparkMax;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import java.util.concurrent.atomic.AtomicBoolean;
import java.util.concurrent.atomic.AtomicInteger;

public class Shooter extends SubsystemBase {
  // TODO: get these ids
  private CANSparkFlex m_topLeftMotor = new CANSparkFlex(7, MotorType.kBrushless);
  private CANSparkFlex m_bottomMotor = new CANSparkFlex(8, MotorType.kBrushless);
  private CANSparkMax m_topRightMotor = new CANSparkMax(10, MotorType.kBrushless);
  // right side shooter is 10

  private double topSpeed = 1;
  private double bottomSpeed = 1;
  /** Creates a new Shooter. */
  public Shooter() {
    m_topRightMotor.setIdleMode(CANSparkBase.IdleMode.kCoast);
    m_topLeftMotor.setIdleMode(CANSparkBase.IdleMode.kCoast);
    m_bottomMotor.setIdleMode(CANSparkBase.IdleMode.kCoast);

    m_topRightMotor.setInverted(true);
    m_topLeftMotor.setInverted(false);
    m_topLeftMotor.setInverted(false);

    m_topLeftMotor.burnFlash();
    m_topRightMotor.burnFlash();
    m_bottomMotor.burnFlash();
  }

  public Command intake() {
    return startEnd(() -> {
      m_topLeftMotor.set(-1);
      m_bottomMotor.set(-1);
      m_topRightMotor.set(-1);
    }, this::off);
  }
  public Command run() {
    Timer topBottomDelay = new Timer();
    AtomicInteger step = new AtomicInteger(0);
    return new FunctionalCommand(
    () -> {
      m_bottomMotor.set(-1);
      topBottomDelay.restart();
      step.set(0);
    },
     () -> {
        if (topBottomDelay.hasElapsed(.3) && step.get() < 1) {
          m_topLeftMotor.set(topSpeed);
          m_topRightMotor.set(topSpeed);
          topBottomDelay.restart();
          step.set(1);
        }
        if (topBottomDelay.hasElapsed(.5) && step.get() == 1) {
          m_bottomMotor.set(bottomSpeed);
          step.set(2);
        }
     },
     b -> {off();},
     () -> false,
     this);
  }

  private void off() {
    m_bottomMotor.set(0);
    m_topLeftMotor.set(0);
    m_topRightMotor.set(0);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
