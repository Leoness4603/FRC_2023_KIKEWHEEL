// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.VictorSPXConfiguration;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.ADXRS450_Gyro;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.PowerDistribution.ModuleType;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.interfaces.Gyro;
import frc.robot.Constants;
import frc.robot.Constants.DIO;
import frc.robot.Constants.IDcan;
import frc.robot.Constants.kChassis;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import com.kauailabs.navx.AHRSProtocol;
import com.kauailabs.navx.frc.AHRS;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

public class Chassis extends SubsystemBase {
  /** Creates a new Chassis. */

  WPI_VictorSPX m_rear_left = new WPI_VictorSPX(Constants.IDcan.Chassis.m_rear_left);
  WPI_VictorSPX m_bhnd_left = new WPI_VictorSPX(Constants.IDcan.Chassis.m_bhnd_left);
  WPI_VictorSPX m_rear_right = new WPI_VictorSPX(Constants.IDcan.Chassis.m_rear_right);
  CANSparkMax m_bhnd_right = new CANSparkMax(IDcan.Chassis.m_bhnd_right, MotorType.kBrushed);
  
  PIDController controllerChassis = new PIDController(kChassis.PIDValues.kP, kChassis.PIDValues.kI, kChassis.PIDValues.kD);
  
  AHRS ahrs;

  PowerDistribution PDP = new PowerDistribution(0, ModuleType.kCTRE);


  private DifferentialDrive drive;

  public Chassis() {
    m_rear_left.configFactoryDefault();
    m_rear_right.configFactoryDefault();
    m_bhnd_left.follow(m_rear_left);

    MotorControllerGroup leftMotors = new MotorControllerGroup(m_rear_left, m_bhnd_left);
    MotorControllerGroup rightMotors = new MotorControllerGroup(m_rear_right, m_bhnd_right);

    leftMotors.setInverted(true);
    rightMotors.setInverted(false);

    drive = new DifferentialDrive(leftMotors, rightMotors);

    ahrs = new AHRS(SPI.Port.kMXP);
    ahrs.calibrate();

  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    log();
  }

  public double getAngle() {
    return ahrs.getAngle();
  }

  public void setPoint(double setPoint) {
    controllerChassis.setSetpoint(setPoint);
  }

  public PIDController getChassisController() {
    return controllerChassis;
  }

  public void driveA(double movement, double rotation, double maxOutput) {
    this.drive.setMaxOutput(maxOutput);
    this.drive.arcadeDrive(movement , rotation);
    this.drive.feed();
  }

  public void log() {
    SmartDashboard.putBoolean("navX conectado", ahrs.isConnected());
    SmartDashboard.putNumber("Total Current:", PDP.getTotalCurrent());
  }
  
  public void stop() {
    this.drive.stopMotor();
  }
}

