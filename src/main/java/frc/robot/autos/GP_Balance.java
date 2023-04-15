// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
package frc.robot.autos;


import frc.robot.subsystems.Swerve;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

import java.util.HashMap;
import java.util.List;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.auto.PIDConstants;
import com.pathplanner.lib.auto.SwerveAutoBuilder;

import frc.robot.subsystems.Brazo;
import frc.robot.subsystems.Gripper;
import frc.robot.Constants;
import frc.robot.commands.BooleanArmControl;




public class GP_Balance extends SequentialCommandGroup {

  public GP_Balance(Swerve s_Swerve, Brazo b_Brazo, Gripper g_Gripper) {

    final List<PathPlannerTrajectory> pathGroup = PathPlanner.loadPathGroup(
    "GP&Balance", 
    new PathConstraints(2, 2),
    new PathConstraints(4, 3), 
    new PathConstraints(4, 3),
    new PathConstraints(1, 0.3));

    HashMap<String, Command> eventMap = new HashMap<>();
    eventMap.put("putAway", new BooleanArmControl(b_Brazo, true, false, false, false, false, false, false));
    eventMap.put("intakeDown", new BooleanArmControl(b_Brazo, false, true, false, false, false, false, false));
    eventMap.put("intakeMid", new BooleanArmControl(b_Brazo, false, false, true, false, false, false, false));
    eventMap.put("intakeHigh", new BooleanArmControl(b_Brazo, false, false, false, true, false, false, false));
    eventMap.put("come", new InstantCommand(() -> g_Gripper.come()));
    eventMap.put("escupe", new InstantCommand(() -> g_Gripper.escupe()));
    eventMap.put("quieto", new InstantCommand(() -> g_Gripper.quieto()));
    eventMap.put("agarra", new BooleanArmControl(b_Brazo, false, true, false, false, false, false, true));




    SwerveAutoBuilder autoBuilder = new SwerveAutoBuilder(
    s_Swerve::getPose, // Pose2d supplier
    s_Swerve::resetOdometry, // Pose2d consumer, used to reset odometry at the beginning of auto
    Constants.Swerve.swerveKinematics, // SwerveDriveKinematics
    new PIDConstants(0.5, 0.0, 0.0), // PID constants to correct for translation error (used to create the X and Y PID controllers)
    new PIDConstants(0.5, 0.1, 0.0), // PID constants to correct for rotation error (used to create the rotation controller)
    s_Swerve::setModuleStatesAuto, // Module states consumer used to output to the drive subsystem
    eventMap,
    false, // Should the path be automatically mirrored depending on alliance color. Optional, defaults to true
    s_Swerve // The drive subsystem. Used to properly set the requirements of path following commands
);


  Command fullAuto = autoBuilder.fullAuto(pathGroup);


    
  addCommands(
    fullAuto
  );
  }
}
