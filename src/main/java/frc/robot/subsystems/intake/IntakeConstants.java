package frc.robot.subsystems.intake;

public class IntakeConstants {

    public static class IntakeDeviceID{
        public static final int MOTOR = 41;
        public static final int LIGHT_LIMIT = 0;
        public static final int MECHANICAL_LIMIT = 0;
      }
      public static class ConvertionParams {
        public static final double MOTOR_GEAR_RATIO = 1/1.9; // (1/number)
        public static final double MOTOR_PULSES_PER_SPIN = 2048;
      }
      public static class Parameters{
        public static final double DEADBAND = 0.2;
        public static final boolean IS_INVERTED = true;
        public static final double NOT_PRESENCE_VOLTAGE = 3.3;

        public static final double MIN_CURRENT_TO_AMP = 50;
        public static final double MIN_CURRENT_TO_SHOOTER = 10;
        
  
  
        public static final double INTAKE_POWER = 1;
        public static final double INTAKE_TRANSFER_POWER = 1;
        public static final double NUM_FINAL_ROTATIONS = 10;
        public static final double SENSOR_TO_REST_DIST = ConvertionParams.MOTOR_PULSES_PER_SPIN/ConvertionParams.MOTOR_GEAR_RATIO*NUM_FINAL_ROTATIONS;
        public static final double INTAKE_PRE_LIMIT_POWER = 1;
  
        public static final double SHOOT_POWER = 0;
        public static final double SHOOT_TIME = 0;

        public static final double CRITICAL_CURRENT = 100;
        public static final double CRITICAL_CURRENT_WHEN_SHOOTER = 120;
  
        
        public static final double DISPENSE_POWER = -0.4;
        public static final double DISPENSE_TIME = 1;
  
  
        public static final double KP = 0.0;
        public static final double KI = 0.000;
        public static final double KD = 0;
   
        public static final double KS1 = 0.0;
        public static final double KG1 = 0.00;
        public static final double KA1 = 0.00;
        public static final double KV1 = 0.0;
      }

    
}