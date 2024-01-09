// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

//en esta clase se definen las constantes que usara el robot para su sistema
public final class Constants {
  
  //Constantes de las salidas maximas en los subsistemas
  public final static class SubsystemMaxOutput {
    // multiplicadres de los sistemas rango(-1, 1)
    public final static double kArm_MaxOutput = .5 ;
    public final static double kChassis_MaxOutput = .50;
    public final static double kClaw_MaxOutput = .4;
    public final static double kForearm_MaxOutput = .99;
  }
  //Constantes en el IdCAN de controladores
  public final static class IDcan {
    //Id de los controladores al chassis
    public final static class Chassis {
      public final static int m_rear_left = 1;
      public final static int m_bhnd_left = 3;
      public final static int m_rear_right = 2;
      public final static int m_bhnd_right = 5;
    }
    //Id de los controladores al brazo
    public final static class Arm {
      public final static int kARM_Motor = 6;
    }    
  }
  //valores PID del antebrazo
  public final static class ForeArm {
    public final static class PIDValues{
      public final static double kP = 0,
                                 kI = 0,
                                 kD = 0;
    }
  }
  //Valores PID del chassis
  public final static class kChassis {
    public final static class PIDValues {
      public final static double kP = 0,
                                 kI = 0,
                                 kD = 0;
    }
  }
  //puertos del PWM para los subsistemas
  public final static class IoPWM {
    //Puerto PWM de la garra
    public final static class Claw {
      public final static int kCLAW_Motor = 0;
    }
    //Puerto PWM del antebrazo
    public final static class Forearm {
      public final static int kForearm_Motor = 1;
    }
  }
  
  //Puertos de los DIO
  public final static class DIO {

  }
  //Entradas USB del Joystick 
  public final static class Joystick {
    public final static int kJoystick_Port = 0;
    public final static int kSecond_Joystick_Port = 1;
    public final static int kThird_Joystick_Port = 2;
  }
  //puertos para los sensores
  public final static class Sensors{
    public final static int kCameraID = 0;
  }

}
