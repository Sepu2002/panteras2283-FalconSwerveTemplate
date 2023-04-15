// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
package frc.robot.commands;

import java.util.function.BooleanSupplier;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Brazo;

public class ArmControl extends CommandBase {
  private Brazo Brazo;
  private BooleanSupplier Closed;
  private BooleanSupplier Low;
  private BooleanSupplier Mid;
  private BooleanSupplier High;
  private BooleanSupplier RielAtras;
  private BooleanSupplier Retraer;
  private BooleanSupplier Extender;


  /** Creates a new ArmControl. */
  public ArmControl(Brazo Brazo, BooleanSupplier Closed, BooleanSupplier Low, BooleanSupplier Mid, BooleanSupplier High, BooleanSupplier rielAtrass, BooleanSupplier Retraer, BooleanSupplier Extender) {
    this.Brazo = Brazo;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(Brazo);

    this.Closed = Closed;
    this.Low = Low;
    this.High = High;
    this.Mid = Mid;
    this.RielAtras = rielAtrass;
    this.Retraer=Retraer;
    this.Extender=Extender;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if(Closed.getAsBoolean() == true){
      Brazo.Closed();
      System.out.println("cerrado");
    }
    else if(Low.getAsBoolean() == true){
      Brazo.Lowlevel(Extender.getAsBoolean(),Retraer.getAsBoolean());
      System.out.println("Nivel bajo");
    }
    else if(Mid.getAsBoolean() == true){
     Brazo.Midlevel(Extender.getAsBoolean(),Retraer.getAsBoolean());
     System.out.println("Nivel medio");
    }
    else if(High.getAsBoolean() == true){
      Brazo.Highlevel(Extender.getAsBoolean(),Retraer.getAsBoolean());
      System.out.println("Nivel alto");
    }


    else {
      //Brazo.RielAtras();
      //System.out.println("Riel elastico");
      }
 
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
