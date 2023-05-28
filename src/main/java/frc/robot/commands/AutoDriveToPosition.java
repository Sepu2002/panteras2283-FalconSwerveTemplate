// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;
import frc.robot.subsystems.Limelight;
import frc.robot.subsystems.Swerve;

import com.pathplanner.lib.PathPlannerTrajectory;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class AutoDriveToPosition extends CommandBase {
  /** Creates a new AutoDriveToPosition. */
  private Swerve s_Swerve;    
  private Limelight l_Limelight;
  private PathPlannerTrajectory traj;

  public AutoDriveToPosition(
    Limelight l_Limelight, 
    Swerve s_Swerve
  ) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.s_Swerve = s_Swerve;
    this.l_Limelight=l_Limelight;
    addRequirements(s_Swerve, l_Limelight);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    this.traj = s_Swerve.generate_AP_Path();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    l_Limelight.SelectPipeline(1);
    l_Limelight.LEDsOFF();
    s_Swerve.followTrajectoryCommand(traj,false);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
