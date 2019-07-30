/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import com.kauailabs.navx.frc.*;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.XboxController;

public class Robot extends TimedRobot {

  TalonSRX leftOne = new TalonSRX(1);
  TalonSRX leftTwo = new TalonSRX(2);
  TalonSRX rightOne = new TalonSRX(3);
  TalonSRX rightTwo = new TalonSRX(4);

  XboxController controller = new XboxController(0);
  GenericHID.Hand leftHand = GenericHID.Hand.kLeft;
  GenericHID.Hand rightHand = GenericHID.Hand.kRight;

  AHRS ahrs = new AHRS(SPI.Port.kMXP);

  @Override
  public void robotInit() {

  }

  @Override
  public void autonomousInit() {

  }

  @Override
  public void autonomousPeriodic() {

  }

  @Override
  public void teleopInit() {
    leftOne.setSelectedSensorPosition(0);
    rightOne.setSelectedSensorPosition(0);

    ahrs.reset();
  }

  @Override
  public void teleopPeriodic() {
    standardDrive();
    System.out.println(ahrs.getAngle() * 4.5);
  }

  public void standardDrive() {
    double throttle = 0;
    double turn = 0;

    if (controller.getY(leftHand) > 0.05 || controller.getY(leftHand) < -0.05)
      throttle = controller.getY(leftHand);

    if (controller.getX(rightHand) > 0.05 || controller.getX(rightHand) < -0.05)
      turn = controller.getX(rightHand);

    leftOne.set(ControlMode.PercentOutput, turn - throttle);
    leftTwo.set(ControlMode.PercentOutput, turn - throttle);

    rightOne.set(ControlMode.PercentOutput, turn + throttle);
    rightTwo.set(ControlMode.PercentOutput, turn + throttle);
  }
}