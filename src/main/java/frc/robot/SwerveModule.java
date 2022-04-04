// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.ctre.phoenix.sensors.AbsoluteSensorRange;
import com.ctre.phoenix.sensors.CANCoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.RelativeEncoder;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.simulation.EncoderSim;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class SwerveModule {
    // private static final double kWheelRadius = 0.0508;
    // private static final int kEncoderResolution = 4096;

    // private static final double kModuleMaxAngularVelocity = 2 *
    // Drivetrain.kMaxAngularSpeed;
    // private static final double kModuleMaxAngularAcceleration = 4 * Math.PI; //
    // radians per second squared

    private final WPI_TalonFX driveMotor;
    private final CANSparkMax turningMotor;

    private final CANCoder absEncoder;
    private final RelativeEncoder turningEncoder;
    // private double turnMotorAngle;
    private final double DriveScaleFactor = 44836;

    private final PIDController drivePIDController = new PIDController(0.5, 0.0, 0.0);

    // private final ProfiledPIDController turningPIDController = new
    // ProfiledPIDController(5.0, 0.0, 0.0,
    // new TrapezoidProfile.Constraints(kModuleMaxAngularVelocity,
    // kModuleMaxAngularAcceleration));
    private final PIDController turningPIDController = new PIDController(5.0, 0.0, 0.0);

    private SimpleMotorFeedforward driveFeedForward;

    private double turnVoltCommand;
    private double driveVoltCommand;
    private SwerveConstants constants;
    private double oldTurnAngle = 0.0;

    // Using FlywheelSim as a stand-in for a simple motor
    private FlywheelSim m_turnMotorSim;
    private FlywheelSim m_driveMotorSim;
    private EncoderSim driveEncoderSim;
    private EncoderSim turnEncoderSim;
    private static int encoderIndex = 0;
    private double zeroAngle;

    /**
     * Constructs a SwerveModule.
     *
     * @param driveMotorChannel   ID for the drive motor.
     * @param turningMotorChannel ID for the turning motor.
     * @param canCoderID          ID for the CANCoder (mounted to turning motor.)
     */
    public SwerveModule(SwerveConstants cornerConstants) {
        constants = cornerConstants;
        driveMotor = new WPI_TalonFX(CanIDConstants.SWERVE_DRIVES[constants.Id]);
        turningMotor = new CANSparkMax(CanIDConstants.SWERVE_ROTS[constants.Id], MotorType.kBrushless);
        turningEncoder = turningMotor.getEncoder();
        turningEncoder.setPositionConversionFactor(6.82);
        driveMotor.setNeutralMode(NeutralMode.Brake);
        turningMotor.setIdleMode(IdleMode.kBrake);

        /*
         * TODO: Turn on hardware PID control
         * var encoder = turningMotor.getEncoder();
         * encoder.setInverted(false);
         * //This example assumes a 4" wheel on a 15:1 reduction
         * encoder.setPositionConversionFactor(factor);
         * CANPIDController pidController = turningMotor.getPIDController();
         * pidController.setP(p);
         * pidController.setI(i);
         * pidController.setD(d);
         * pidController.setFF(ff);
         * pidController.setIZone(IZone); //IZone is the amount of error before the I
         * term is considered. It is designed so you don't get a ton of I-term build up
         * at the beginning of the PID control. You set it when you want I to kick in
         * near the end of control. It should be positive, 0 disables it.
         * pidController.setOutputRange(min, max);
         */

        absEncoder = new CANCoder(CanIDConstants.SWERVE_ROT_SENSORS[constants.Id]);
        absEncoder.setPositionToAbsolute();
        absEncoder.configAbsoluteSensorRange(AbsoluteSensorRange.Signed_PlusMinus180, 0);
        zeroAngle = CanIDConstants.SWERVE_ZEROS[constants.Id];

        // Limit the PID Controller's input range between -pi and pi and set the input
        // to be continuous.
        turningPIDController.enableContinuousInput(-Math.PI, Math.PI);

        driveFeedForward = new SimpleMotorFeedforward(0, constants.DriveMotorKv, constants.DriveMotorKa);

        if (Robot.isSimulation()) {
            m_turnMotorSim = new FlywheelSim(
                    LinearSystemId.identifyVelocitySystem(constants.TurnMotorKv, constants.TurnMotorKa),
                    constants.TurnMotor,
                    constants.TurnMotorGearRatio);

            m_driveMotorSim = new FlywheelSim(
                    LinearSystemId.identifyVelocitySystem(constants.DriveMotorKv, constants.DriveMotorKa),
                    constants.DriveMotor,
                    constants.DriveMotorGearRatio);

            Encoder enc = new Encoder(encoderIndex, encoderIndex + 1);
            enc.setDistancePerPulse(0.001);
            driveEncoderSim = new EncoderSim(enc);

            enc = new Encoder(encoderIndex + 2, encoderIndex + 3);
            enc.setDistancePerPulse(0.001);
            turnEncoderSim = new EncoderSim(enc);
            encoderIndex += 4;
        }
    }

    /**
     * Returns the current state of the module.
     *
     * @return The current state of the module.
     */
    public SwerveModuleState getState() {
        return new SwerveModuleState(getVelocity(), getRotation());
    }

    public double getVelocity() {
        if (Robot.isReal()) {
            return driveMotor.getSelectedSensorVelocity() / DriveScaleFactor;
        }
        return driveEncoderSim.getRate();
    }

    public double getDistance() {
        if (Robot.isReal()) {
            return driveMotor.getSelectedSensorPosition() / DriveScaleFactor;
        }
        return driveEncoderSim.getDistance();
    }

    public double getAbsoluteAngle() {
        if (Robot.isReal()) {
            return -absEncoder.getAbsolutePosition() + zeroAngle;
        }
        return turnEncoderSim.getDistance();
    }

    public Rotation2d getRotation() {
        return new Rotation2d(Math.toRadians(getAbsoluteAngle()));
    }

    /**
     * Sets the desired state for the module.
     *
     * @param desiredState Desired state with speed and angle.
     */
    public void setDesiredState(SwerveModuleState desiredState) {
        // Optimize the reference state to avoid spinning further than 90 degrees
        SwerveModuleState state = SwerveModuleState.optimize(desiredState, getRotation());
        // Calculate the drive output from the drive PID controller.
        final double driveOutput = drivePIDController.calculate(getVelocity(), state.speedMetersPerSecond);
        final double driveFeedforward = driveFeedForward.calculate(state.speedMetersPerSecond);

        // Calculate the turning motor output from the turning PID controller.
        // if(oldTurnAngle == 0) {
        // oldTurnAngle = Math.toRadians(getAbsoluteAngle());
        // }
        // double newTurnAngle = (1 - 0.4) * oldTurnAngle + 0.4 *
        // Math.toRadians(getAbsoluteAngle());
        final double turnOutput = turningPIDController.calculate(Math.toRadians(getAbsoluteAngle()),
                state.angle.getRadians());
        // oldTurnAngle = Math.toRadians(getAbsoluteAngle());

        turnVoltCommand = -turnOutput;
        // DataLogManager.log(constants.Name + " Turn volt command: " +
        // turnVoltCommand);
        driveVoltCommand = driveOutput + driveFeedforward;

        if (Math.abs(driveVoltCommand) <= 0.01) {
            turnVoltCommand = 0;
        }

        // DataLogManager.log(constants.Name + " Drive volt command: " +
        // driveVoltCommand);

        driveMotor.setVoltage(driveVoltCommand);
        turningMotor.setVoltage(turnVoltCommand);

        /*
         * TODO: use hardware control of motor control instead of SW
         * //set the motor to 10 revolutions. We should divide the encoder to degrees
         * for better control
         * pidController.setReference(10.0, ControlType.kPosition);
         */
    }

    public void setDrive(double value) {
        driveMotor.set(ControlMode.PercentOutput, value);
        driveVoltCommand = value * RobotController.getBatteryVoltage();
    }

    public void simulationPeriodic(double rate) {
        // we need to calculate the motor velocities and encoder positions since they
        // aren't real here
        m_turnMotorSim.setInputVoltage(turnVoltCommand);
        m_driveMotorSim.setInputVoltage(driveVoltCommand);

        m_turnMotorSim.update(rate);
        m_driveMotorSim.update(rate);

        // Calculate distance traveled using RPM * dt
        double dist = turnEncoderSim.getDistance();
        dist -= Math.toDegrees(m_turnMotorSim.getAngularVelocityRadPerSec() * rate);
        turnEncoderSim.setDistance(dist);

        dist = driveEncoderSim.getDistance();
        dist += m_driveMotorSim.getAngularVelocityRadPerSec() * rate;
        driveEncoderSim.setDistance(dist);
        driveEncoderSim.setRate(m_driveMotorSim.getAngularVelocityRadPerSec());
    }

    public void putSmartDashboard() {
        SmartDashboard.putNumber(constants.Name + "/driveEncoderRaw", getDistance());
        SmartDashboard.putNumber(constants.Name + "/driveVelocity", getVelocity());
        SmartDashboard.putNumber(constants.Name + "/absEncoderZeroed", getAbsoluteAngle());
        SmartDashboard.putNumber(constants.Name + "/absEncoderRaw", absEncoder.getAbsolutePosition());
        SmartDashboard.putNumber(constants.Name + "/turnEncoderRaw", turningEncoder.getPosition());
        SmartDashboard.putNumber(constants.Name + "/turnVoltCommand", turnVoltCommand);
        SmartDashboard.putNumber(constants.Name + "/driveVoltCommand", driveVoltCommand);
    }

    public void setBrakeMode(boolean brake) {
        NeutralMode neutralMode = NeutralMode.Coast;
        IdleMode idleMode = IdleMode.kCoast;
        if (brake) {
            neutralMode = NeutralMode.Brake;
            idleMode = IdleMode.kBrake;
        }
        driveMotor.setNeutralMode(neutralMode);
        turningMotor.setIdleMode(idleMode);
    }
}
