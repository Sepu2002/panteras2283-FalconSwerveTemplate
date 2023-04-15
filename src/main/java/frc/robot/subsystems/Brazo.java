package frc.robot.subsystems;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.ctre.phoenix.motorcontrol.TalonFXFeedbackDevice;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.CANSparkMaxLowLevel;
import edu.wpi.first.wpilibj.Timer;

import frc.robot.Constants;

// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

public class Brazo extends SubsystemBase {
  private final TalonFX CamMotor = new TalonFX(Constants.CamMotorID);
  private final CANSparkMax RailMotor = new CANSparkMax(Constants.RailMotorID, MotorType.kBrushless);
  private SparkMaxPIDController PIDControl;
  private RelativeEncoder Encoder;

  /** Creates a new Brazo. */
  public Brazo() {
    CamMotor.configSelectedFeedbackSensor(TalonFXFeedbackDevice.IntegratedSensor, 0, 0);
    CamMotor.configNominalOutputForward(0.1, 0);
    CamMotor.config_kP(7, 0.1, 0);
    CamMotor.config_kI(7, 0, 0);
    CamMotor.config_kD(7, 0, 0);
    CamMotor.configPeakOutputForward(0.6);
    CamMotor.configPeakOutputReverse(-0.6);
    CamMotor.setInverted(false);
    CamMotor.setSensorPhase(false);

    PIDControl = RailMotor.getPIDController();
    Encoder = RailMotor.getEncoder();

    PIDControl.setP(Constants.railKP);
    PIDControl.setI(Constants.railKI);
    PIDControl.setD(Constants.railKD);
    PIDControl.setIZone(Constants.railKIz);
    PIDControl.setFF(Constants.railKFF);
    PIDControl.setOutputRange(Constants.KMinOutput, Constants.kMaxOutput);
    

  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putNumber("Encoder leva", CamMotor.getSensorCollection().getIntegratedSensorPosition());
    SmartDashboard.putNumber("Encoder Riel", RailMotor.getEncoder().getPosition());

  }

  public void Closed() {
    CamMotor.set(TalonFXControlMode.Position, -20000);
    //PIDControl.setReference(1, ControlType.kPosition);
    PIDControl.setReference(0, ControlType.kPosition);
  }

  public void Lowlevel(boolean drop, boolean lift) {
    CamMotor.set(TalonFXControlMode.Position, -45000);
    delay(0.1);
    if(drop==true){
    PIDControl.setReference(-100, ControlType.kPosition);
    }
   else if(lift==true){
    PIDControl.setReference(-70, ControlType.kPosition);

    }
  else{
    PIDControl.setReference(-90, ControlType.kPosition);

    }
  }

  public void Midlevel(boolean drop, boolean lift) {
    CamMotor.set(TalonFXControlMode.Position, -130000);
    delay(0.1);
    if(drop==true){
      PIDControl.setReference(-73, ControlType.kPosition);
      }
     else if(lift==true){
      PIDControl.setReference(-50, ControlType.kPosition);
  
      }
    else{
      PIDControl.setReference(-60, ControlType.kPosition);
  
      }
  }

  public void Highlevel(boolean drop, boolean lift) {
    CamMotor.set(TalonFXControlMode.Position, -175000);
    delay(0.1);
    if(drop==true){
      PIDControl.setReference(-30, ControlType.kPosition);
      }
     else if(lift==true){
      PIDControl.setReference(-40, ControlType.kPosition);
  
      }
    else{
      PIDControl.setReference(-80, ControlType.kPosition);
  
      }
  }

  public void retraer(){
    double ajust = Encoder.getPosition() +10;
    if (ajust<=15){
      PIDControl.setReference(ajust, ControlType.kPosition);}
  }

  public void expandir(){
    double ajust = Encoder.getPosition() -10;
    if (ajust>=-115){
    PIDControl.setReference(ajust, ControlType.kPosition);}

  }

  public void detener(){
    RailMotor.set(0);
  }

  public void RielAtras(){
    PIDControl.setReference(0, ControlType.kPosition);


  }

  /*Autonomos */
  
  public void HighCamAuto(){
    CamMotor.set(TalonFXControlMode.Position, -175000);
  }

  public void HighRielAuto(){
    PIDControl.setReference(-80, ControlType.kPosition);
  }
  /*Autonomos */

  public void delay(double i){
    Timer.delay(i);
  }

 



  /* 
  public object delay(int i){
    return null;
  }
  */
  
}
