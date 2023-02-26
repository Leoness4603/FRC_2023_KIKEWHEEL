// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

public final class Constants {
  
  
  public final static class IDcan {
    public final static class Chassis {
      public final static int m_rear_left = 1;
      public final static int m_bhnd_left = 4;
      public final static int m_rear_right = 3;
      public final static int m_bhnd_right = 2;
    }

    public final static class Claw {
      public final static int kCLAW_Motor = 6;
    }
    public final static class Arm {
      public final static int kARM_Motor = 5;
    }    
  }
  
  public final static class IoPWM {
    public final static class Forearm {
      public final static int kForearm_Motor = 0;
      public final static int kForearm_Motor2 = 1;
    }
  }
  

  public final static class DIO {
    public final static int kEncoderRight = 2;
    public final static int kEncoderRight2 = 3;
    public final static int kEncoderLeft = 4;
    public final static int kEncoderLeft2 = 5;
    public final static int kEncoderForeArm = 0;
    public final static int kEncoderForeArm2 = 1;
  }

  public final static class Joystick {
    public final static int kJoystick_Port = 0;
    public final static int kSecond_Joystick_Port = 1;
    public final static int kThird_Joystick_Port = 2;
  }

  public final static class Sensors{
    public final static int kCameraID = 1;
    public final static int kCameraID2 = 2;
  }

}
