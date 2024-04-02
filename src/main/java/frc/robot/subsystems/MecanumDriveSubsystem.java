// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.function.DoubleSupplier;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.hardware.TalonFX;
import com.kauailabs.navx.frc.AHRS;
import com.pathplanner.lib.auto.AutoBuilder;

import edu.wpi.first.math.estimator.MecanumDrivePoseEstimator;
import edu.wpi.first.math.kinematics.MecanumDriveKinematics;
import edu.wpi.first.math.kinematics.MecanumDriveOdometry;
import edu.wpi.first.wpilibj.drive.MecanumDrive;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class MecanumDriveSubsystem extends SubsystemBase {
    private TalonFX frontLeft = new TalonFX(2);
    private TalonFX rearLeft = new TalonFX(3);
    private TalonFX frontRight = new TalonFX(4);
    private TalonFX rearRight = new TalonFX(5);
    private StatusSignal<Double> flPos;
    private StatusSignal<Double> frPos;
    private StatusSignal<Double> rlPos;
    private StatusSignal<Double> rrPos;

    private MecanumDrive m_MecanumDrive = new MecanumDrive(frontLeft, rearLeft, frontRight, rearRight);  
    private AHRS m_gyro = new AHRS();
    private MecanumDrivePoseEstimator m_odometry;

    public MecanumDriveSubsystem() {
        frontRight.setInverted(true); // TODO Which ones are inverted?
        rearRight.setInverted(true);
        Constants.SB_TAB.add(m_gyro);

    }
    public Command joystickDrive(DoubleSupplier x, DoubleSupplier y, DoubleSupplier rot) {
        return runEnd(() ->m_MecanumDrive.driveCartesian(x.getAsDouble(),y.getAsDouble(),rot.getAsDouble(), m_gyro.getRotation2d())
, () -> {});
    }

    @Override
    public void periodic() {
        // This method will be called once per scheduler run
    }

    @Override
    public void simulationPeriodic() {
        // This method will be called once per scheduler run during simulation
    }
}
