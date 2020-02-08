/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.command.Subsystem;

import frc.robot.RobotMap;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import frc.robot.commands.MailboxSet;

/** Add your docs here. */
public class Mailbox extends Subsystem {
  // Put methods for controlling this subsystem
  // here. Call these from Commands.

  // TalonSRX mailboxMotor = new TalonSRX(RobotMap.MOTOR_MAILBOX_ID);

  // Wrapper method
  public void turn(double voltagePercent) {
    // mailboxMotor.set(ControlMode.PercentOutput, voltagePercent);
  }

  @Override
  public void initDefaultCommand() {
    // Set the default command for a subsystem here.
    // setDefaultCommand(new MySpecialCommand());
    setDefaultCommand(new MailboxSet());
  }
}
