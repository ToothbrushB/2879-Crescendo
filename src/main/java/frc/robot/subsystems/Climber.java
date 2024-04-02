// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkLimitSwitch;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkBase.SoftLimitDirection;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.SparkLimitSwitch.Type;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;

public class Climber extends SubsystemBase {
  private final CANSparkMax m_left = new CANSparkMax(6, MotorType.kBrushless);
  private final CANSparkMax m_right = new CANSparkMax(9, MotorType.kBrushless); // TODO actually id this thing
  private final SparkLimitSwitch m_leftSwitch;
  private final SparkLimitSwitch m_rightSwitch;
  private final RelativeEncoder m_leftEncoder;
  private final RelativeEncoder m_rightEncoder;
  private final SparkPIDController m_leftPid;
  private final SparkPIDController m_rightPid;

  private boolean isUp;
  private final double UP_POSITION = 9;
  private final double DOWN_POSITION = 0;

  /** Creates a new Climber. */
  public Climber() {
    m_leftSwitch = m_left.getForwardLimitSwitch(SparkLimitSwitch.Type.kNormallyClosed); // TODO figure this out (open/closed, forward/reverse)
    m_rightSwitch = m_left.getReverseLimitSwitch(SparkLimitSwitch.Type.kNormallyClosed); 
    m_leftEncoder = m_left.getEncoder();
    m_rightEncoder = m_right.getEncoder();
    m_leftPid = m_left.getPIDController();
    m_rightPid = m_right.getPIDController();

    m_leftSwitch.enableLimitSwitch(false);
    m_rightSwitch.enableLimitSwitch(false);
    
    m_left.setInverted(false);
    m_right.setInverted(false);
    m_left.setSoftLimit(SoftLimitDirection.kForward, 10); // TODO figure this out
    m_right.setSoftLimit(SoftLimitDirection.kForward, 10); // TODO figure this out


    m_leftPid.setFeedbackDevice(m_leftEncoder);
    m_rightPid.setFeedbackDevice(m_rightEncoder);

    m_leftPid.setP(0); // TODO tune PID
    m_rightPid.setP(0);


    m_left.burnFlash();
    m_right.burnFlash();

    new Trigger(m_leftSwitch::isPressed)
      .debounce(.2) // TODO figure out debounce
      .onTrue(
        runOnce(() -> m_leftEncoder.setPosition(0))
      );

    new Trigger(m_rightSwitch::isPressed)
      .debounce(.2)
      .onTrue(
        runOnce(() -> m_rightEncoder.setPosition(0))
    );
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
      m_leftPid.setReference(0, ControlType.kPosition);
      m_rightPid.setReference(0, ControlType.kPosition); 
      isUp = false;
    });
  }

  @Override
  public void periodic() {
    
    // This method will be called once per scheduler run
  }
}
