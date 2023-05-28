// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.Limelight;
import frc.robot.subsystems.Swerve;

public class TurboTeleopSwerve extends CommandBase {
  /** Creates a new TurboTeleopSwerve. */
  
    private Swerve s_Swerve;
    private Limelight l_Limelight;
    private DoubleSupplier translationSup;
    private DoubleSupplier strafeSup;
    private DoubleSupplier rotationSup;
    private BooleanSupplier robotCentricSup;

  public TurboTeleopSwerve(
    Limelight l_Limelight,
    Swerve s_Swerve, 
    DoubleSupplier translationSup, 
    DoubleSupplier strafeSup, 
    DoubleSupplier rotationSup, 
    BooleanSupplier robotCentricSup
  ) {
    // Use addRequirements() here to declare subsystem dependencies.
        this.s_Swerve = s_Swerve;
        this.l_Limelight=l_Limelight;
        addRequirements(s_Swerve, l_Limelight);

        this.translationSup = translationSup;
        this.strafeSup = strafeSup;
        this.rotationSup = rotationSup;
        this.robotCentricSup = robotCentricSup;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
        double translationVal = (Math.pow(MathUtil.applyDeadband(translationSup.getAsDouble(), Constants.stickDeadband), 1));
        double strafeVal = (Math.pow(MathUtil.applyDeadband(strafeSup.getAsDouble(), Constants.stickDeadband), 1));
        double rotationVal = Math.pow(MathUtil.applyDeadband(rotationSup.getAsDouble(), Constants.stickDeadband), 1);

        l_Limelight.SelectPipeline(1);
        l_Limelight.LEDsOFF();
        s_Swerve.drive(
        new Translation2d(translationVal, strafeVal).times(Constants.Swerve.turbomaxSpeed), 
        rotationVal * Constants.Swerve.maxAngularVelocity, 
        !robotCentricSup.getAsBoolean(), 
        true
    );
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
