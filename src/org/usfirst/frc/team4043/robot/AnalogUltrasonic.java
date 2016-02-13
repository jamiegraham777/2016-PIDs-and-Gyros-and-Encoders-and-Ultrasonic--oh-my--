package org.usfirst.frc.team4043.robot;


public class AnalogUltrasonic extends AnalogInput{

	public static class DistanceUnits {

        /**
         * The integer value representing this enumeration
         */
        public final int value;
        static final int INCHES_VAL = 0;
        static final int CENTIMETERS_VAL = 0;
        /** Inches */
        public static final DistanceUnits INCHES = new DistanceUnits(INCHES_VAL);
        /** Centimeters */
        public static final DistanceUnits CENTIMETERS = new DistanceUnits(CENTIMETERS_VAL);

        private DistanceUnits(int value) {
            this.value = value;
        }
    }
    private DistanceUnits units;
    private double scaleFactorCm, scaleFactorIn;

    /**
     * Create a new instance of an ultrasonic rangefinder assuming the Analog Module
     * is plugged into its default slot and given the slot, channel, units and scaling factor.
     *
     * @param channel The channel the sonar is plugged into on the Analog Breakout.
     * @oaram units The units used in the scaling factor.
     * @param scaleFactor The scaling factor used to convert voltage to real world measurements (mV/units),
     * assuming you are giving the ultrasonic rangefinder a 5V supply.
     * For example, the LV-MaxSonar-EZ1 from the 2012 FRC KOP's scaling factor is 9.8 mV/in. with a 5V supply. 
     */

	/**
     * Create a new instance of an ultrasonic rangefinder assuming the Analog Module
     * given the slot, channel, units and scaling factor.
     * 
     * @param channel The channel the sonar is plugged into on the Analog Breakout.
     * @oaram units The units used in the scaling factor.
     * @param scaleFactor The scaling factor used to convert voltage to real world measurements (mV/units),
     * assuming you are giving the ultrasonic rangefinder a 5V supply.
     * For example, the LV-MaxSonar-EZ1 from the 2012 FRC KOP's scaling factor is 9.8 mV/in. with a 5V supply. 
     */
    public AnalogUltrasonic( int channel, DistanceUnits units, double scaleFactor) {
        super(channel);

        if (units == DistanceUnits.CENTIMETERS) {
            this.scaleFactorCm = scaleFactor;
            this.scaleFactorIn = scaleFactor * 2.54;
            this.units = DistanceUnits.CENTIMETERS;
        } else if (units == DistanceUnits.INCHES) {
            this.scaleFactorIn = scaleFactor;
            this.scaleFactorCm = scaleFactor / 2.54;
            this.units = DistanceUnits.INCHES;
        }
    }

    /**
     * Converts the voltage from the getVoltage() method of the AnalogChannel class
     * to inches as reported by the ultrasonic rangefinder.
     * 
     * @return Ultrasonic rangefinder distance in inches.
     */
    public double getIn() {

        double in;

        in = (double) (getVoltage() * 1000) / scaleFactorIn;

        return in;
    }

    /**
     * Converts the voltage from the getAverageVoltage() method of the AnalogChannel class
     * to inches as reported by the ultrasonic rangefinder.
     * 
     * @return Average ultrasonic rangefinder distance in inches.
     */
    public double getAvgIn() {

        double in;

        in = (double) (getAverageVoltage() * 1000) / scaleFactorIn;

        return in;
    }

    /**
     * Converts the voltage from the getVoltage() method of the AnalogChannel class
     * to centimeters as reported by the ultrasonic rangefinder.
     * 
     * @return Ultrasonic rangefinder distance in centimeters.
     */
    public double getCm() {

        double cm;

        cm = (double) (getVoltage() * 1000) / scaleFactorCm;

        return cm;
    }

    /**
     * Converts the voltage from the getAverageVoltage() method of the AnalogChannel class
     * to centimeters as reported by the ultrasonic rangefinder.
     * 
     * @return Average ultrasonic rangefinder distance in centimeters.
     */
    public double getAvgCm() {

        double cm;

        cm = (double) (getAverageVoltage() * 1000) / scaleFactorCm;

        return cm;
    }

    /**
     * Converts the voltage from the getVoltage() method of the AnalogChannel class
     * to distance as scaled by the units given in the constructor.
     * 
     * @return Distance reported by the ultrasonic rangefinder scaled by the given units in the constructor.
     */
    public double getDistance() {
        double distance = 0;
        
        if (units == DistanceUnits.CENTIMETERS) {
            distance = (double) (getVoltage() ) / scaleFactorCm;
        } else if (units == DistanceUnits.INCHES) {
            distance = (double) (getVoltage() ) / scaleFactorIn;
        }
        
        return distance;
    }
    
    /**
     * Converts the voltage from the getAverageVoltage() method of the AnalogChannel class
     * to distance as scaled by the units given in the constructor.
     * 
     * @return Distance reported by the ultrasonic rangefinder scaled by the given units in the constructor.
     */
    public double getAverageDistance() {
        double distance = 0;
        
        if (units == DistanceUnits.CENTIMETERS) {
            distance = (double) (getAverageVoltage() ) / scaleFactorCm;
        } else if (units == DistanceUnits.INCHES) {
            distance = (double) (getAverageVoltage() ) / scaleFactorIn;
        }
        
        return distance;
    }
}
