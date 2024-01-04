// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class SDBMotorSim extends SubsystemBase {

public enum ControlType {
    BANG_BANG,
    PID,
    PROFILE
  }

  ControlType _controlType;

  DCMotorSim _mechanisem; 
  PIDController _controller;
  ProfiledPIDController _profController;
  double[] _delayArray;

  final double delaySecs = 0.1;
  final int delayRoborioPeriods = (int)(delaySecs / 0.02);
  final String _name;
  final double ks = DCMotor.getMiniCIM(1).KvRadPerSecPerVolt;
  int i = 0;

  /** Creates a new motorSim. */
  public SDBMotorSim(String name, ControlType control){
    _controlType = control;
    _name = name;
    _mechanisem = new DCMotorSim(DCMotor.getMiniCIM(2),50,5);
    _controller = new PIDController(5, 0, 0);
    _profController = new ProfiledPIDController(5, 0,0, new TrapezoidProfile.Constraints(50,20));

    SmartDashboard.putNumber("kP"+_name, 1);
    // SmartDashboard.putNumber("kI"+_name, 0);
    SmartDashboard.putNumber("kD"+_name, 0);

    SmartDashboard.putNumber("maxVel", 1);
    SmartDashboard.putNumber("maxAcc", 1);

    _delayArray = new double[delayRoborioPeriods];

    for(int j = 0; j < _delayArray.length;j++) {
      _delayArray[j] = 0;
    }
  }

  @Override
  public void periodic() {
    switch(_controlType) {
      case BANG_BANG: 
        if(Math.abs(_delayArray[i % _delayArray.length]-_controller.getSetpoint()) < 0.5)
        break;
        if(_delayArray[i % _delayArray.length] < _controller.getSetpoint()){
          _mechanisem.setInputVoltage(clamp(_controller.getP(),-12,12));
          break;
        }
          _mechanisem.setInputVoltage(clamp(-_controller.getP(),-12,12));
        break;
      case PID: 
         _mechanisem.setInputVoltage(clamp(_controller.calculate(_delayArray[i % _delayArray.length]),-12,12));
      break;

      case PROFILE:
        _mechanisem.setInputVoltage(clamp(_profController.calculate(_delayArray[i % _delayArray.length], _controller.getSetpoint())+_profController.getSetpoint().velocity/ks,-12,12));
      break;
    }
    _delayArray[i%_delayArray.length] = _mechanisem.getAngularPositionRad();
    i++;
    _mechanisem.update(0.02);
    SmartDashboard.putNumber("pos"+_name,_mechanisem.getAngularPositionRad());
    _controller.setP(SmartDashboard.getNumber("kP"+_name, 0));
    // _controller.setI(SmartDashboard.getNumber("kI"+_name, 0));
    _controller.setD(SmartDashboard.getNumber("kD"+_name, 0));

    _profController.setP(SmartDashboard.getNumber("kP"+_name, 0));
    // _profController.setI(SmartDashboard.getNumber("kI"+_name, 0));
    _profController.setD(SmartDashboard.getNumber("kD"+_name, 0));
    _controller.setSetpoint(SmartDashboard.getNumber("setpoint", 0));
    // This method will be called once per scheduler run

    SmartDashboard.putNumber("vel"+_name, _mechanisem.getAngularVelocityRadPerSec());

    if(_controlType == ControlType.PROFILE)
      _profController.setConstraints(new TrapezoidProfile.Constraints(SmartDashboard.getNumber("maxVel", 0),SmartDashboard.getNumber("maxAcc", 0)));
  }

  double clamp(double value, double min, double max) {
    return Math.max(min, Math.min(max, value));
}   

}