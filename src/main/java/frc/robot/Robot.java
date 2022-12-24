// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.spikes2212.control.FeedForwardSettings;
import com.spikes2212.control.PIDSettings;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.math.trajectory.TrajectoryUtil;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.commands.FollowTrajectory;
import frc.robot.subsystems.Drivetrain;

import java.io.IOException;
import java.nio.file.Path;

public class Robot extends TimedRobot {

  @Override
  public void robotInit() {
    Trajectory trajectory;
    try {
      trajectory = TrajectoryUtil.fromPathweaverJson(
              Path.of(Filesystem.getDeployDirectory().getPath(), "testtrajectory.json"));
    } catch (IOException e) {
      throw new RuntimeException(e);
    }
    FollowTrajectory followTrajectory = new FollowTrajectory(trajectory,
            new Drivetrain(null, null, null, 0),
            new PIDSettings(0, 0, 0), FeedForwardSettings.EMPTY_FFSETTINGS);
  }

  @Override
  public void robotPeriodic() {
    CommandScheduler.getInstance().run();
  }

  @Override
  public void disabledInit() {

  }

  @Override
  public void disabledPeriodic() {

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

  @Override
  public void testInit() {
    CommandScheduler.getInstance().cancelAll();
  }

  @Override
  public void testPeriodic() {

  }

  @Override
  public void simulationInit() {

  }

  @Override
  public void simulationPeriodic() {

  }
}
