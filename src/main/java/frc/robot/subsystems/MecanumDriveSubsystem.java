// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.kauailabs.navx.frc.AHRS;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import com.pathplanner.lib.util.PIDConstants;
import com.pathplanner.lib.util.ReplanningConfig;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.estimator.MecanumDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.MecanumDriveKinematics;
import edu.wpi.first.math.kinematics.MecanumDriveWheelPositions;
import edu.wpi.first.math.kinematics.MecanumDriveWheelSpeeds;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.Distance;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.Velocity;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

import java.util.Optional;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

import static edu.wpi.first.units.Units.*;

public class MecanumDriveSubsystem extends SubsystemBase {
    private final MecanumDriveWheelPositions wheelPositions = new MecanumDriveWheelPositions();
    private final MecanumDriveWheelSpeeds wheelSpeeds = new MecanumDriveWheelSpeeds();
    private final Measure<Velocity<Distance>> kSpeedAt12VoltsMps = MetersPerSecond.of(4); // TODO: this is a guess, we can figure it out later lol

    // THE ORDER IS: FL, FR, RL, RR
    private final MecanumDriveKinematics kinematics = new MecanumDriveKinematics( // TODO DO THIS, X first, then Y. Both are in meters. Use this to help you determine signs. https://docs.wpilib.org/en/stable/docs/software/basic-programming/coordinate-system.html
        new Translation2d(1,1),
        new Translation2d(1,1),
        new Translation2d(1,1),
        new Translation2d(1,1)
    );

    private final double kDriveRadius;


    // TODO: find inverted (it should either be left or right side if you did it right i believe) and wheel radius and gear ratio. DOUBLE CHECK WHEEL IDS. Gear ratio is sensor to mechanism (how many rotations of motor lead to one rotation of wheel?)
    private final Wheel m_frontLeft = new Wheel(new Wheel.WheelConstants(2, Inches.of(2),false, kSpeedAt12VoltsMps, 1));
    private final Wheel m_frontRight = new Wheel(new Wheel.WheelConstants(3,Inches.of(2),true, kSpeedAt12VoltsMps, 1));
    private final Wheel m_rearLeft = new Wheel(new Wheel.WheelConstants(4,Inches.of(2),false, kSpeedAt12VoltsMps, 1));
    private final Wheel m_rearRight = new Wheel(new Wheel.WheelConstants(5,Inches.of(2),true, kSpeedAt12VoltsMps, 1));

    private final AHRS m_gyro = new AHRS();
    private final MecanumDrivePoseEstimator m_odometry;

    public MecanumDriveSubsystem() {
        Constants.SB_TAB.add(m_gyro);
        m_odometry = new MecanumDrivePoseEstimator(kinematics, m_gyro.getRotation2d(), new MecanumDriveWheelPositions(0,0,0,0), new Pose2d());
        kDriveRadius = Math.max(kinematics.getFrontLeft().getNorm(), Math.max(kinematics.getFrontRight().getNorm(), Math.max(kinematics.getRearLeft().getNorm(), kinematics.getRearRight().getNorm())));
        AutoBuilder.configureHolonomic(
            m_odometry::getEstimatedPosition,
            this::seedFieldRelative,
            this::getChassisSpeeds,
            this::velocityDrive,
            new HolonomicPathFollowerConfig(new PIDConstants(10, 0, 0),
                new PIDConstants(10, 0, 0),
                kSpeedAt12VoltsMps.in(MetersPerSecond),
                kDriveRadius,
                new ReplanningConfig()),
            () -> {
                Optional<DriverStation.Alliance> alliance = DriverStation.getAlliance();
                return alliance.filter(value -> value == DriverStation.Alliance.Red).isPresent();
            }, // Change this if the path needs to be flipped on red vs blue
            this); // Subsystem for requirements
    }

    public MecanumDriveWheelSpeeds getWheelSpeeds() {
        wheelSpeeds.frontLeftMetersPerSecond = m_frontLeft.getVelocity();
        wheelSpeeds.frontRightMetersPerSecond = m_frontRight.getVelocity();
        wheelSpeeds.rearLeftMetersPerSecond = m_rearLeft.getVelocity();
        wheelSpeeds.rearRightMetersPerSecond = m_rearRight.getVelocity();
        return wheelSpeeds;
    }
    public ChassisSpeeds getChassisSpeeds() {
        return kinematics.toChassisSpeeds(getWheelSpeeds());
    }

    public Pose2d getEstimatedPosition() {
        return m_odometry.getEstimatedPosition();
    }

    public void setVisionMeasurementStdDevs(Matrix<N3, N1> visionMeasurementStdDevs) {
        m_odometry.setVisionMeasurementStdDevs(visionMeasurementStdDevs);
    }

    public void addVisionMeasurement(Pose2d visionRobotPoseMeters, double timestampSeconds) {
        m_odometry.addVisionMeasurement(visionRobotPoseMeters, timestampSeconds);
    }

    public void addVisionMeasurement(Pose2d visionRobotPoseMeters, double timestampSeconds, Matrix<N3, N1> visionMeasurementStdDevs) {
        m_odometry.addVisionMeasurement(visionRobotPoseMeters, timestampSeconds, visionMeasurementStdDevs);
    }

    public void seedFieldRelative(Pose2d location) {
        m_odometry.resetPosition(m_gyro.getRotation2d(), wheelPositions, location);
    }
    public Command velocityDrive(Supplier<ChassisSpeeds> speedsSupplier) {
        return runEnd(() -> {
            MecanumDriveWheelSpeeds s = kinematics.toWheelSpeeds(speedsSupplier.get());
            m_frontLeft.setMps(s.frontLeftMetersPerSecond);
            m_frontRight.setMps(s.frontRightMetersPerSecond);
            m_rearLeft.setMps(s.rearLeftMetersPerSecond);
            m_rearRight.setMps(s.rearRightMetersPerSecond);
        }, () -> {});
    }

    public Command velocityDrive(ChassisSpeeds speeds) {
        MecanumDriveWheelSpeeds s = kinematics.toWheelSpeeds(speeds);
        return runEnd(() -> {
            m_frontLeft.setMps(s.frontLeftMetersPerSecond);
            m_frontRight.setMps(s.frontRightMetersPerSecond);
            m_rearLeft.setMps(s.rearLeftMetersPerSecond);
            m_rearRight.setMps(s.rearRightMetersPerSecond);
        }, () -> {});
    }

    public Command joystickDrive(DoubleSupplier x, DoubleSupplier y, DoubleSupplier rot) {
        ChassisSpeeds s = new ChassisSpeeds();
        return velocityDrive(() -> {
            s.vxMetersPerSecond = x.getAsDouble();
            s.vyMetersPerSecond = y.getAsDouble();
            s.omegaRadiansPerSecond = rot.getAsDouble();
            return s;
        });
    }

    public MecanumDriveWheelPositions getWheelPositions() {
        wheelPositions.frontLeftMeters = m_frontLeft.getLatencyCompensatedPosition();
        wheelPositions.frontLeftMeters = m_frontRight.getLatencyCompensatedPosition();
        wheelPositions.frontLeftMeters = m_rearLeft.getLatencyCompensatedPosition();
        wheelPositions.frontLeftMeters = m_rearRight.getLatencyCompensatedPosition();
        return wheelPositions;
    }
    @Override
    public void periodic() {
        // This method will be called once per scheduler run
        m_odometry.update(m_gyro.getRotation2d(),getWheelPositions());
    }

    @Override
    public void simulationPeriodic() {
        // This method will be called once per scheduler run during simulation
    }

    private class Wheel {
        private final TalonFX m_motor;
        private final StatusSignal<Double> m_position;
        private final StatusSignal<Double> m_velocity;
        private final double circumference;
        private final VoltageOut m_voltageControl = new VoltageOut(0);
        private double volts = 0;
        private final double speedAt12V;

        public Wheel(WheelConstants constants) {
            speedAt12V = constants.speedAt12V.in(Meters.per(Second));

            m_motor = new TalonFX(constants.id);
            m_motor.setInverted(constants.inverted);
            m_motor.getConfigurator().apply(new FeedbackConfigs().withSensorToMechanismRatio(constants.gearRatio));

            circumference = constants.radius.in(Meters) * 2 * Math.PI;
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

        public void setMps(double s) {
            volts = s / speedAt12V * 12;
            m_voltageControl.withOutput(volts);
        }


        private record WheelConstants(int id, Measure<Distance> radius, boolean inverted, Measure<Velocity<Distance>> speedAt12V, double gearRatio) {}
    }
}
