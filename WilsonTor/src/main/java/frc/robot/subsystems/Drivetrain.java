/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.command.Subsystem;
import frc.robot.RobotMap;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANEncoder;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import edu.wpi.first.wpilibj.Spark;

import edu.wpi.first.wpilibj.drive.DifferentialDrive;

//PID
import edu.wpi.first.wpilibj.controller.PIDController;
import com.kauailabs.navx.frc.AHRS;
import edu.wpi.first.wpilibj.SPI;

// limelight stuff
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import frc.robot.commands.Drive;

/** Drivetrain class w/ limelight vision tracking */
public class Drivetrain extends Subsystem {
  // Drivetrain

  Spark leftMotor = new Spark(RobotMap.MOTOR_LEFT_ID);
  Spark rightMotor = new Spark(RobotMap.MOTOR_RIGHT_ID);

  DifferentialDrive dualDrive = new DifferentialDrive(leftMotor, rightMotor);

  // limelight table to read offset value from
  NetworkTable limelightTable = NetworkTableInstance.getDefault().getTable("limelight");

  // average
  double average = 0.0;

  // Gyro
  public AHRS gyro;

  public void initializeGyro() {
    try {
      gyro = new AHRS(SPI.Port.kMXP);
    } catch (RuntimeException ex) {
      DriverStation.reportError("Error instantiating navX MXP", true);
    }
  }

  public void calibrate() {
    gyro.zeroYaw();
  }

  // PID
  double P = 0.09;
  double I = 0;
  double D = 0;
  PIDController pid = new PIDController(P, I, D);

  // wrappers

  public void driveStraight(double speed) {
    dualDrive.arcadeDrive(speed, pid.calculate(gyro.getAngle(), 0));
    SmartDashboard.putNumber("gyro", gyro.getAngle());
  }

  public void drive(double speed, double rotation) {
    dualDrive.arcadeDrive(speed, rotation);
  }

  public void oldDrive(double leftSpeed, double rightSpeed) {
    dualDrive.tankDrive(leftSpeed, rightSpeed);
  }

  public void rotateToAngle(double angle) {
    dualDrive.arcadeDrive(0, pid.calculate(gyro.getAngle(), angle));
    SmartDashboard.putNumber("gyro", gyro.getAngle());
  }

  @Override
  public void initDefaultCommand() {
    // Set the default command for a subsystem here.
    // setDefaultCommand(new MySpecialCommand());
    setDefaultCommand(new Drive());
  }
}
