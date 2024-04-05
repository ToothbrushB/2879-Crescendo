// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkLimitSwitch;
//import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkBase.SoftLimitDirection;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.SparkLimitSwitch.Type;

import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.*;
import edu.wpi.first.wpilibj2.command.button.Trigger;

import java.util.concurrent.atomic.AtomicBoolean;

import static frc.robot.Constants.SB_TAB;

public class Climber extends SubsystemBase {
  private final CANSparkMax m_right = new CANSparkMax(9, MotorType.kBrushless);
  private final CANSparkMax m_left = new CANSparkMax(6, MotorType.kBrushless); // TODO actually id this thing
  private final SparkLimitSwitch m_leftSwitch;
  private final SparkLimitSwitch m_rightSwitch;
  private final RelativeEncoder m_leftEncoder;
  private final RelativeEncoder m_rightEncoder;
  private final SparkPIDController m_leftPid;
  private final SparkPIDController m_rightPid;

  private boolean isUp;
  private final double UP_POSITION = 50;
  private final double DOWN_POSITION = 1;

  /** Creates a new Climber. */
  public Climber() {
    m_rightSwitch = m_right.getForwardLimitSwitch(Type.kNormallyOpen); // TODO figure this out (open/closed, forward/reverse)
    m_leftSwitch = m_right.getReverseLimitSwitch(Type.kNormallyOpen);
    m_leftEncoder = m_left.getEncoder();
    m_rightEncoder = m_right.getEncoder();
    m_leftPid = m_left.getPIDController();
    m_rightPid = m_right.getPIDController();

    m_leftSwitch.enableLimitSwitch(false);
    m_rightSwitch.enableLimitSwitch(false);

    m_left.setInverted(false);
    m_right.setInverted(false);
    m_left.setSoftLimit(SoftLimitDirection.kReverse, 0); // TODO figure this out
    m_right.setSoftLimit(SoftLimitDirection.kReverse, 0); // TODO figure this out
    m_left.setIdleMode(IdleMode.kBrake);
    m_right.setIdleMode(IdleMode.kBrake);

//
    m_leftPid.setFeedbackDevice(m_leftEncoder);
    m_rightPid.setFeedbackDevice(m_rightEncoder);
//
    m_leftPid.setP(1); // TODO tune PID
    m_rightPid.setP(1);


    m_left.burnFlash();
    m_right.burnFlash();

    new Trigger(m_leftSwitch::isPressed)
      .whileTrue(
        runOnce(() -> {m_leftEncoder.setPosition(0);m_left.set(0);})
      );

    new Trigger(m_rightSwitch::isPressed)
      .whileTrue(
        runOnce(() -> {m_rightEncoder.setPosition(0);m_left.set(0);})
    );
    SB_TAB.addDouble("right climber", m_rightEncoder::getPosition);
    SB_TAB.addDouble("left climber", m_leftEncoder::getPosition);

  }

  public Command toggle() {
    return Commands.either(down(), up(), isUp());
  }
  public Trigger isUp() {
    return new Trigger(() -> isUp);
  }

  public Command up() {
    return runOnce(() -> {
      m_leftPid.setReference(UP_POSITION, ControlType.kPosition); // TODO figure this out
      m_rightPid.setReference(UP_POSITION, ControlType.kPosition);
      isUp = true;
    });
  }
  
  public Command down() {
    return runOnce(() -> {
      m_leftPid.setReference(DOWN_POSITION, ControlType.kPosition);
      m_rightPid.setReference(DOWN_POSITION, ControlType.kPosition);
      isUp = false;
    });
  }

  public Command dutyCycleDown() {
//    AtomicBoolean isDone = new AtomicBoolean(false);
//    return new FunctionalCommand(
//        () -> {m_left.set(-.2); m_right.set(-.2);},
//        () -> {
//          if (m_rightSwitch.isPressed() || m_leftSwitch.isPressed()) {
//            isDone.set(true);
//            m_left.set(0); m_right.set(0);
//          }
//        },
//        b -> {m_left.set(0); m_right.set(0);},
//        isDone::get
//    );
    return new StartEndCommand(() -> {m_left.set(-.2); m_right.set(-.2);}, () -> {m_left.set(0); m_right.set(0);});
  }
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putNumber("left encoder", m_leftEncoder.getPosition());
    SmartDashboard.putNumber("right encoder", m_rightEncoder.getPosition());
  }
}
