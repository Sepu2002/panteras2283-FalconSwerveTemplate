// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Brazo;

public class BooleanArmControl extends CommandBase {
  private Brazo Brazo;
  private Boolean Closed;
  private Boolean Low;
  private Boolean Mid;
  private Boolean High;
  private Boolean RielAtras;
  private Boolean Retraer;
  private Boolean Extender;


  /** Creates a new ArmControl. */
  public BooleanArmControl(Brazo Brazo, Boolean Closed, Boolean Low, Boolean Mid, Boolean High, Boolean rielAtrass, Boolean Retraer, Boolean Extender) {
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
    if(Closed == true){
      Brazo.Closed();
      System.out.println("cerrado");
    }
    else if(Low == true){
      Brazo.Lowlevel(Extender,Retraer);
      System.out.println("Nivel bajo");
    }
    else if(Mid == true){
     Brazo.Midlevel(Extender,Retraer);
     System.out.println("Nivel medio");
    }
    else if(High == true){
      Brazo.Highlevel(Extender,Retraer);
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
