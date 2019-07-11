/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import edu.wpi.first.wpilibj.*;
import com.ctre.phoenix.*;
import com.kauailabs.navx.frc.*;

public class Robot extends TimedRobot {

  TalonSRX leftOne = new TalonSRX(1);
  TalonSRX leftTwo = new TalonSRX(2);
  TalonSRX rightOne = new TalonSRX(3);
  TalonSRX rightTwo = new TalonSRX(4);

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

  }

  @Override
  public void teleopPeriodic() {

  }
}