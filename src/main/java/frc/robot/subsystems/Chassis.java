// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.interfaces.Gyro;
import frc.robot.Constants;
import frc.robot.Constants.DIO;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import com.kauailabs.navx.AHRSProtocol;
import com.kauailabs.navx.frc.AHRS;

public class Chassis extends SubsystemBase {
  /** Creates a new Chassis. */

  WPI_VictorSPX m_rear_left = new WPI_VictorSPX(Constants.IDcan.Chassis.m_rear_left);
  WPI_VictorSPX m_bhnd_left = new WPI_VictorSPX(Constants.IDcan.Chassis.m_bhnd_left);
  WPI_VictorSPX m_rear_right = new WPI_VictorSPX(Constants.IDcan.Chassis.m_rear_right);
  WPI_VictorSPX m_bhnd_right = new WPI_VictorSPX(Constants.IDcan.Chassis.m_bhnd_right);

  Encoder encoderRight = new Encoder(DIO.kEncoderRight, DIO.kEncoderRight2);
  Encoder encoderLeft = new Encoder(DIO.kEncoderLeft, DIO.kEncoderLeft2);
  
  AHRS ahrs;
  Gyro gyro;

  private Joystick drivejoystick;

  private DifferentialDrive drive;

  public Chassis(Joystick driveJoystick) {
    m_bhnd_left.follow(m_rear_left);
    m_bhnd_right.follow(m_rear_right);
    MotorControllerGroup leftMotors = new MotorControllerGroup(m_rear_left, m_bhnd_left);
    MotorControllerGroup rightMotors = new MotorControllerGroup(m_rear_right, m_bhnd_right);

    drive = new DifferentialDrive(leftMotors, rightMotors);

    encoderLeft.setDistancePerPulse(.001);
    encoderRight.setDistancePerPulse(.001);

    ahrs = new AHRS(SPI.Port.kMXP);
    
    
    this.drivejoystick = driveJoystick;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    ahrs.calibrate();
  }

  public double getAngle() {
    return ahrs.getAngle();
  }

  public double getDistanceR() {
    return encoderRight.getDistance();
  }

  public double getDistanceL() {
    return encoderLeft.getDistance();
  }

  public double getAverageDistance() {
    return (getDistanceL() + getDistanceR()) / 2;
  }

  public void driveA(double movement, double rotation) {
    this.drive.setMaxOutput(0.6);
    this.drive.arcadeDrive(movement, rotation, true);
    this.drive.feed();
  }

  public void driveAMAX(double movement, double rotation) {
    this.drive.setMaxOutput(1);
    this.drive.arcadeDrive(movement, rotation, true);
    this.drive.feed();
  }

  public void driveT(double movement, double rotation) {
    this.drive.setMaxOutput(0.6);
    this.drive.tankDrive(movement, rotation, true);
    this.drive.feed();
  }

  public void driveTMAX(double movement, double rotation) {
    this.drive.setMaxOutput(1);
    this.drive.tankDrive(movement, rotation, true);
    this.drive.feed();
  }

  public void driveC(double movement, double rotation) {
    this.drive.setMaxOutput(0.6);
    this.drive.curvatureDrive(movement, rotation, false);
    this.drive.feed();
  }

  public void driveCMEN(double movement, double rotation) {
    this.drive.setMaxOutput(1);
    this.drive.curvatureDrive(movement, rotation, false);
    this.drive.feed();
  }

  public void log() {
    SmartDashboard.putNumber("Distancia izquierda :", getDistanceL());
    SmartDashboard.putNumber("Distancia derecha : ", getDistanceR());
    SmartDashboard.putBoolean("navX conectado", ahrs.isConnected());
  }
  
  public void stop() {
    this.drive.arcadeDrive(0, 0);
    this.drive.curvatureDrive(0, 0, false);
    this.drive.tankDrive(0, 0);
  }

  public Joystick getJoystick() {
    return drivejoystick;
  }
}
