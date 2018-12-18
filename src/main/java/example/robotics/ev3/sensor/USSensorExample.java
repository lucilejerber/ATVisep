package example.robotics.ev3.sensor;

import ev3dev.sensors.Battery;
import ev3dev.sensors.ev3.EV3UltrasonicSensor;
import ev3dev.sensors.ev3.EV3IRSensor;
import lejos.hardware.port.SensorPort;
import lejos.robotics.SampleProvider;
import lejos.utility.Delay;
import ev3dev.sensors.ev3.EV3ColorSensor;

public class USSensorExample {

	private static EV3UltrasonicSensor us1 = new EV3UltrasonicSensor(SensorPort.S1);
	public static EV3IRSensor ir1 = new EV3IRSensor(SensorPort.S2);

	// Color sensor
	private static EV3ColorSensor color = new EV3ColorSensor(SensorPort.S3);



	public static void main(String[] args) {

		final int iteration_threshold = 60*10;
		
		for(int i = 0; i <= iteration_threshold; i++) {
			System.out.println(" ---------- Iteration " + i + " ---------- ");
			System.out.println("The difference in distance between the two telemeters is : " + telemeterF());
			System.out.println("RED : " + getRedIntensity());
			Delay.msDelay(1500);
		}
		
		
	}
	
	public static float telemeterF(){
		float d1 = 0, d2 = 0, delta = 0;
		
		d1 = getdistanceUS() ;
		d2 = getdistanceIR() ;
		delta = d1-d2;
			
		if (delta < -2) {
			System.out.println("Go to left");
		} else if (delta > 2) {
			System.out.println("Go to rigth");
		} else {
			System.out.println("Go straigth");
		}
		
		return delta;
	}
	
	public static float getdistanceUS(){
		final SampleProvider sp = us1.getDistanceMode();

		float distanceValue = 0;

		final int iteration_threshold = 100;

		float [] sample = new float[sp.sampleSize()];
		sp.fetchSample(sample, 0);
		distanceValue = sample[0];

		System.out.println("Distance US: " + distanceValue);

		return distanceValue;
	}
	
	public static float getdistanceIR(){
		final SampleProvider sp = ir1.getDistanceMode();

		float distanceValue = 0;

		float [] sample = new float[sp.sampleSize()];
		sp.fetchSample(sample, 0);
		distanceValue = sample[0];

		System.out.println("Distance IR: " + distanceValue);
	
		return distanceValue;
	}
	
	public static float getRedIntensity () {
		int value = 0;
		
		System.out.println("Switching to Red Mode");
		SampleProvider sp = color.getRedMode();
		
		int sampleSize = sp.sampleSize();
		float[] sample = new float[sampleSize];
        	
        sp.fetchSample(sample, 0);
		value = (int) sample[0];
			
		System.out.println("Red intensity: " + value);
		
		return value;
	}
	


}
