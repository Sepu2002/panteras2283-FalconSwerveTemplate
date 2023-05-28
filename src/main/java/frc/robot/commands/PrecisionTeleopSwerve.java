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

public class PrecisionTeleopSwerve extends CommandBase {
  /** Creates a new PrecisionTeleopSwerve. */
  private Swerve s_Swerve;
  private Limelight l_Limelight;
  private DoubleSupplier translationSup;
  private DoubleSupplier strafeSup;
  private DoubleSupplier rotationSup;
  private BooleanSupplier robotCentricSup;
  private BooleanSupplier front;
  private BooleanSupplier back;
  private BooleanSupplier left;
  private BooleanSupplier right;

  public PrecisionTeleopSwerve(
    Limelight l_Limelight,
    Swerve s_Swerve, 
    DoubleSupplier translationSup, 
    DoubleSupplier strafeSup, 
    DoubleSupplier rotationSup, 
    BooleanSupplier robotCentricSup,
    BooleanSupplier front,
    BooleanSupplier back,
    BooleanSupplier left,
    BooleanSupplier right
    
  ) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.s_Swerve = s_Swerve;
    this.l_Limelight=l_Limelight;
    addRequirements(s_Swerve, l_Limelight);

    this.translationSup = translationSup;
    this.strafeSup = strafeSup;
    this.rotationSup = rotationSup;
    this.robotCentricSup = robotCentricSup;
    this.front=front;
    this.back=back;
    this.left=left;
    this.right=right;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

        double translationVal = (Math.pow(MathUtil.applyDeadband(translationSup.getAsDouble(), Constants.stickDeadband), 1));
        double strafeVal = (Math.pow(MathUtil.applyDeadband(strafeSup.getAsDouble(), Constants.stickDeadband), 1));
        double rotationVal = Math.pow(MathUtil.applyDeadband(rotationSup.getAsDouble(), Constants.stickDeadband), 1);
        double targetAngle=0;

        l_Limelight.SelectPipeline(1);
        l_Limelight.LEDsOFF();

        /*Lock robot to specific angles (Disables rotation joystick while specific orientation is selected) */
        if(front.getAsBoolean()==true || back.getAsBoolean()==true||left.getAsBoolean()==true||right.getAsBoolean()==true){
          if(front.getAsBoolean()==true){
              targetAngle=0;
          }
          else if(back.getAsBoolean()==true){
              targetAngle=180;
          }
          else if(left.getAsBoolean()==true){
              targetAngle=90;
          }
          else if(right.getAsBoolean()==true){
              targetAngle=270;
          }

        else{
          s_Swerve.lockedrive(
          new Translation2d(translationVal, strafeVal).times(Constants.Swerve.precisionmaxSpeed),  
          !robotCentricSup.getAsBoolean(), 
          true, targetAngle
      );}
          
      }

        l_Limelight.SelectPipeline(1);
        l_Limelight.LEDsOFF();
        s_Swerve.drive(
        new Translation2d(translationVal, strafeVal).times(Constants.Swerve.precisionmaxSpeed), 
        rotationVal * Constants.Swerve.precisionmaxAngularVelocity, 
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
