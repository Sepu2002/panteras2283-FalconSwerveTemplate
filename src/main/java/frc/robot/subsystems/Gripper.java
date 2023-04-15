// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;
import edu.wpi.first.wpilibj.Timer;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Gripper extends SubsystemBase {
  private final CANSparkMax Rightmotor = new CANSparkMax(Constants.RightGripperMotorID, MotorType.kBrushed);
  private final CANSparkMax Leftmotor = new CANSparkMax(Constants.LeftGripperMotorID, MotorType.kBrushed);

  /** Creates a new Gripper. */
  public Gripper() {
    //Leftmotor.setInverted(true);
  } 

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public void come(){
    Rightmotor.set(-0.7);
    Leftmotor.set(0.7);

  }

  public void escupe(){
    Rightmotor.set(0.5);
    Leftmotor.set(-0.5);  
  }

  public void quieto(){
    Rightmotor.set(0);
    Leftmotor.set(0);
  }

  public void delay(double i){
    Timer.delay(i);
  }
}
