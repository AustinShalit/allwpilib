// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package edu.wpi.first.wpilibj.templates.educational;

import edu.wpi.first.hal.HAL;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotBase;

/** Educational robot base class. */
public class EducationalRobot extends RobotBase {
  public void robotInit() {}

  public void disabled() {}

  public void run() {}

  public void autonomous() {
    run();
  }

  public void teleop() {
    run();
  }

  public void test() {
    run();
  }

  private volatile boolean m_exit;

  @Override
  public void startCompetition() {
    robotInit();

    // Tell the DS that the robot is ready to be enabled
    HAL.observeUserProgramStarting();

    while (!Thread.currentThread().isInterrupted() && !m_exit) {
      if (isDisabled()) {
        DriverStation.InDisabled(true);
        disabled();
        DriverStation.InDisabled(false);
        while (isDisabled()) {
          DriverStation.waitForData();
        }
      } else if (isAutonomous()) {
        DriverStation.InAutonomous(true);
        autonomous();
        DriverStation.InAutonomous(false);
        while (isAutonomousEnabled()) {
          DriverStation.waitForData();
        }
      } else if (isTest()) {
        DriverStation.InTest(true);
        test();
        DriverStation.InTest(false);
        while (isTest() && isEnabled()) {
          DriverStation.waitForData();
        }
      } else {
        DriverStation.InOperatorControl(true);
        teleop();
        DriverStation.InOperatorControl(false);
        while (isOperatorControlEnabled()) {
          DriverStation.waitForData();
        }
      }
    }
  }

  @Override
  public void endCompetition() {
    m_exit = true;
  }
}
