// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
package frc.robot.autos;


import frc.robot.subsystems.Swerve;


import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;




public class PathPlanerAuto extends SequentialCommandGroup {
  /** Creates a new PathPlanerAuto. */
  public PathPlanerAuto(Swerve s_Swerve) {

    PathPlannerTrajectory traj = PathPlanner.loadPath("ExamplePath", 7,2);
    addCommands(
      s_Swerve.followTrajectoryCommand(traj, true)
    );
  }
}
