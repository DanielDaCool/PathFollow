package frc.robot.subsystems.amp;

import static frc.robot.subsystems.chassis.ChassisConstants.MOTOR_PULSES_PER_ROTATION;

import org.opencv.core.Mat;

public class AmpConstants{
    public static class AmpDeviceID{
      public static final int ARM_MOTOR_ID = 21;      
      public static final int LOCK_MOTOR_ID = 22;
      public static final int NEO1 = 23;
      public static final int INTAKE_MOTOR_ID = 24;
      public static final int LIGHT_LIMIT = 0;
      public static final int MAGNETIC_SENSOR_ID = 0;
    }
    public static class ConvertionParams {
      public static final double M1GearRatio = 1/128.0;
      public static final double NEO1GearRatio = 1/8.0;
      public static final double INTAKE_GEAR_RATIO = 1/8.0;

      public static final double MOTOR_PULSES_PER_SPIN = 2048;
      public static final double NEO_PULES_PER_REV = 42;


      public static final double PULSE_PER_RAD = MOTOR_PULSES_PER_SPIN/M1GearRatio/2/Math.PI;

      public static final double MOTOR_PULSES_PER_ANGLE = MOTOR_PULSES_PER_SPIN/M1GearRatio/360;
    }
    public static class Parameters{
      public static final double Deadband = 0.2;

      public static final double LOCK_MAX_AMPER = 5;
      public static final double LOCK_POWER = 0.5;
      public static final double UNLOCK_POWER = -0.2;
      public static final double UNLOCK_TIME = 0.7;


      public static final double ARM_UPPER_POSITION_ANGLE = Math.toRadians(55);
      public static final double ARM_SENSOR_POSITION_ANGLE = Math.toRadians(-52);
      public static final double ARM_HONE_POSITION_ANGLE = Math.toRadians(-55);
      public static final double ARM_POSITION_ERROR = Math.toRadians(3);

      public static final double ARM_KP = 0.0570401;
      public static final double ARM_KI = 0;
      public static final double ARM_KD = 0;

      public static final double MAX_ARM_VEL_OPEN = 2*Math.PI;
      public static final double MAX_ARM_ACCEL_OPEN = 5*Math.PI;

      public static final double NUMBER_OF_ROTATION = 0.8;
      public static final double SENSOR_TO_REST_DIST = NUMBER_OF_ROTATION/(ConvertionParams.NEO1GearRatio);
      public static final double NOTE_VOLTAGE = 1;
      public static final double CRITICAL_CURRENT = 0;
      public static final double INTAKE_TRANSFER_POWER = 0.4;
      public static final double INTAKE_POWER = 0.4;
      public static final double INTAKE_PRE_LIMIT_POWER = 0.4;


  

    
    }
    public static class armStatesParameters{
      /* the three numbers in each array is for every state of the arm
        * 
        * 0 = the arm is going up and less then angle 36
        * 1 = the arm is going up and more then angle 36 but less then 72
        * 2 = the arm is going up and more then angle 72 until 110
        * 3 = the arm is going down and more then angle 55
        * 4 = the arm is going down from 55 to 0 
        */
        public static final double KS = 0.1802966;
        public static final double KV = 0.13205069;
        public static final double KA = 0.1531499;
        public static final double Kcos = -0.073106;


        public static final double[] openFF = {0, 0, 0};
        public static final double[] closeFF = {0, 0, 0};

    }


    public static class CommandParams {
    
      public static final double NUM_OF_ROTATION = 5;
      public static final double v1 = 0.5;
      public static final double v2 = 0.25;

      public static final double ANGLE_RADIANS_AMP = 0;
      public static final double MAX_VEL_RAD = 0;
      public static final double MAX_ACCEL_RAD = 0;
    public static final double ANGLE_RADIANS_CLOSE = 0;



      
    }

  }