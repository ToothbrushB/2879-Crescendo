// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.function.DoubleSupplier;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.math.estimator.MecanumDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.MecanumDriveKinematics;
import edu.wpi.first.math.kinematics.MecanumDriveWheelPositions;
import edu.wpi.first.math.kinematics.MecanumDriveWheelSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.drive.MecanumDrive;
import edu.wpi.first.wpilibj.motorcontrol.MotorController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class MecanumDriveSubsystem extends SubsystemBase {
    private final double WHEEL_CIRCUMFERENCE = Units.inchesToMeters(2)*2*Math.PI;
    private final MecanumDriveKinematics kinematics = new MecanumDriveKinematics( // TODO DO THIS
        new Translation2d(),
        new Translation2d(),
        new Translation2d(),
        new Translation2d()
    );


    // TODO: find inverted and wheel radius
    private Wheel m_frontLeft = new Wheel(new Wheel.WheelConstants(2,Units.inchesToMeters(1),false, 1));
    private Wheel m_frontRight = new Wheel(new Wheel.WheelConstants(3,Units.inchesToMeters(1),true, 1));
    private Wheel m_rearLeft = new Wheel(new Wheel.WheelConstants(4,Units.inchesToMeters(1),false, 1));
    private Wheel m_rearRight = new Wheel(new Wheel.WheelConstants(5,Units.inchesToMeters(1),true, 1));

    private AHRS m_gyro = new AHRS();
    private MecanumDrivePoseEstimator m_odometry;

    public MecanumDriveSubsystem() {
        Constants.SB_TAB.add(m_gyro);
        m_odometry = new MecanumDrivePoseEstimator(kinematics, m_gyro.getRotation2d(), new MecanumDriveWheelPositions(0,0,0,0), new Pose2d());
//        AutoBuilder.configureHolonomic(m_odometry::getEstimatedPosition, p -> m_odometry.resetPosition(p), );
    }
    public Command joystickDrive(DoubleSupplier x, DoubleSupplier y, DoubleSupplier rot) {
        MecanumDriveWheelSpeeds speeds =  kinematics.toWheelSpeeds(new ChassisSpeeds(x.getAsDouble(), y.getAsDouble(), rot.getAsDouble()));

    }

    @Override
    public void periodic() {
        // This method will be called once per scheduler run
        m_odometry.update(m_gyro.getRotation2d(),
            new MecanumDriveWheelPositions(
                m_frontLeft.getLatencyCompensatedPosition(),
                m_frontRight.getLatencyCompensatedPosition(),
                m_rearLeft.getLatencyCompensatedPosition(),
                m_rearRight.getLatencyCompensatedPosition()
            ));
    }

    @Override
    public void simulationPeriodic() {
        // This method will be called once per scheduler run during simulation
    }

    private class Wheel implements MotorController {
        private final TalonFX m_motor;
        private final StatusSignal<Double> m_position;
        private final StatusSignal<Double> m_velocity;
        private final double circumference;
        private final VoltageOut m_voltageControl = new VoltageOut(0);
        private double volts = 0;
        private WheelConstants constants;
        public Wheel(WheelConstants constants) {
            this.constants = constants;
            m_motor = new TalonFX(constants.id);
            m_motor.setInverted(constants.inverted);

            circumference = constants.radius * 2 * Math.PI;
            m_position = m_motor.getPosition();
            m_velocity = m_motor.getVelocity();

            m_position.setUpdateFrequency(250);
            m_velocity.setUpdateFrequency(250);
        }

        public double getLatencyCompensatedPosition() {
            return BaseStatusSignal.getLatencyCompensatedValue(m_position.refresh(), m_velocity.refresh())*circumference;
        }

        public double getVelocity() {
            return m_velocity.getValueAsDouble()*circumference;
        }

        @Override
        public void set(double s) {
            volts = s / constants.speedAt12V * 12;
            m_voltageControl.withOutput(volts);
        }

        @Override
        public double get() {
            return volts/12 * constants.speedAt12V;
        }

        @Override
        public void setInverted(boolean b) {
            throw new UnsupportedOperationException("Please use WheelConstants to set this.");
        }

        @Override
        public boolean getInverted() {
            return m_motor.getInverted();
        }

        @Override
        public void disable() {
            m_motor.disable();
        }

        @Override
        public void stopMotor() {
            m_motor.stopMotor();
        }

        private record WheelConstants(int id, double radius, boolean inverted, double speedAt12V) {}
    }
}
