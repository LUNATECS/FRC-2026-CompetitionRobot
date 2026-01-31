// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
  public static class OperatorConstants {
    public static final int kDriverControllerPort = 0;
  }

  public static class Intake_Pivot_SubSystem_Constants{
    public static final int CAN_ID_INTAKE_PIVOT = 9;
  }

  public static class Intake_SubSystem_Constants{
    public static final int CAN_ID_INTAKE = 10;
  }

  public static class Hopper_SubSystem_Constants{
    public static final int CAN_ID_HOPPER = 11;
  }

  public static class Hopper_Feeder_SubSystem_Constants{
    public static final int CAN_ID_FEEDER_BELTS = 12;
  }

  public static class Shooter_Feeder_SubSystem_Constants{
    public static final int CAN_ID_SHOOTER_FEEDER = 13;
  }

  public static class Shooter_SubSystem_Constants{
    public static final int CAN_ID_SHOOTER1 = 14;
    public static final int CAN_ID_SHOOTER2 = 15;
    public static final int CAN_ID_SHOOTER3 = 16;
    public static final int CAN_ID_SHOOTER4 = 17;
  }

  public static class Hood_SubSystem_Constants{
    public static final int CAN_ID_HOOD = 18;
  }

  public static class Climber_SubSystem_Constants{
    public static final int CAN_ID_CLIMBER = 19;
  }
}
