package example;

import ev3dev.actuators.lego.motors.EV3LargeRegulatedMotor;
import ev3dev.sensors.ev3.EV3UltrasonicSensor;
import ev3dev.sensors.ev3.EV3TouchSensor;
import ev3dev.sensors.ev3.EV3ColorSensor;
import ev3dev.sensors.ev3.EV3IRSensor;
import ev3dev.sensors.Battery;

import lejos.hardware.port.MotorPort;
import lejos.hardware.port.SensorPort;
import lejos.robotics.SampleProvider;
import lejos.utility.Delay;

public class ATV {
	// Telemeters
	private static EV3UltrasonicSensor us1 = new EV3UltrasonicSensor(SensorPort.S1);
	//public static EV3IRSensor ir1 = new EV3IRSensor(SensorPort.S1);
	public static EV3IRSensor ir1 = new EV3IRSensor(SensorPort.S2);

	// Color sensor
	private static EV3ColorSensor color = new EV3ColorSensor(SensorPort.S3);

	// Touch sensor
	private static EV3TouchSensor touch1 = new EV3TouchSensor(SensorPort.S4);

	// Motors
	final static EV3LargeRegulatedMotor mA = new EV3LargeRegulatedMotor(MotorPort.A);
	final static EV3LargeRegulatedMotor mB = new EV3LargeRegulatedMotor(MotorPort.B);
	final static EV3LargeRegulatedMotor mC = new EV3LargeRegulatedMotor(MotorPort.C);

	// Variables for movement
	public static int multiplicateur = 1 ;
	public static float forces[][] = {{0.58F, -0.33F, 0.33F},{-0.58F, -0.33F, 0.33F},{0, 0.67F, 0.33F}}; 
	public static float direction[] = {0, 0, 0};
	public static float valeursInterMoteurs[] = {0, 0, 0};
	public static int valeursMoteurs[] = {0, 0, 0} ;
	public static int iteration_threshold = 10;

	// ATV s'arrete si ISS en envoi info panic button actived
	public static boolean IsPanicButton = false ;
	public static boolean isRed = false ;


	///////////////////////////////
	// MAIN
	///////////////////////////////

	public static void main(String[] args) {

		while((getTouchValue() < 1 ) && (IsPanicButton == false)) {

			//if ((telemeterF() < 2) && (telemeterF() > -2)) {
				//if ((getRedIntensity() > 2) || isRed) {System.out.println("Red detected Move toward the ISS");
			System.out.println("---------------------");
			
				if ((getRedIntensity() > 2) ) {
					// isRed = true;
					while (getTouchValue() < 1) {
						seMettreDroit();
						System.out.println("Red detected Move toward the ISS");
						multiplicateur = 1;
						directionDroit(forces, direction, valeursInterMoteurs, valeursMoteurs, mA, mB, mC);
					} 
						System.out.println("Is docked");
						mA.stop();
						mC.stop();
						mB.stop();
						IsPanicButton = true;
				
				} else {
					if ((getdistanceIR() > 70) || (telemeterD() == Float.NEGATIVE_INFINITY) || (telemeterD() == Float.POSITIVE_INFINITY)){
						System.out.println("IR distance above 70 cm");
						multiplicateur = 5;
						System.out.println("Turn");
						deplacement90fluide(forces, direction, valeursInterMoteurs, valeursMoteurs, mA, mB, mC);
					} else if (telemeterD() > 25) {
						System.out.println("Distance above 25 cm");
						System.out.println("Move toward the ISS");
						multiplicateur = 2;
						directionDroit(forces, direction, valeursInterMoteurs, valeursMoteurs, mA, mB, mC);
					} else if (telemeterD()  < 25) {
						System.out.println("Distance below 25 cm");
						System.out.println("Go to the left");
						multiplicateur = 5;
						deplacementDroite(forces, direction, valeursInterMoteurs, valeursMoteurs, mA, mB, mC); 
					} /*else if (telemeterD()  < 6) {
						mA.stop();
						mB.stop();
						mC.stop();
					}*/
				}
			
			System.out.println("The difference in distance between the two telemeters is : " + telemeterF());
			System.out.println("The average in distance between the two telemeters is : " + telemeterD());
			//System.out.println("RED : " + getRedIntensity());
			System.out.println("Touch value : " + getTouchValue());
			//Delay.msDelay(3000);
			//}


		}

	}

	public static void deplacement90fluide(float forces[][], float direction[], float valeursInterMoteurs[], int valeursMoteurs[], EV3LargeRegulatedMotor mA, EV3LargeRegulatedMotor mB, EV3LargeRegulatedMotor mC)
	{
		seMettreDroit();
		deplacementDroite(forces, direction, valeursInterMoteurs, valeursMoteurs, mA, mB, mC); 
		Delay.msDelay(1000);
		direction[0] = 0;
		direction[1] = 1;
		direction[2] = 1;
		multiplicationMatrice(forces, direction, valeursInterMoteurs, valeursMoteurs);
		lancement(valeursMoteurs, mA, mB, mC);
		Delay.msDelay(2000);
		deplacementDroite(forces, direction, valeursInterMoteurs, valeursMoteurs, mA, mB, mC); 
		Delay.msDelay(100);
		seMettreDroit();
	}
	
	public static void seMettreDroit()
	{
		if (telemeterF() < -2) {
			System.out.println("Go to rigth");
			rotationDroite();
		} 
		if (telemeterF() > 2) {
			System.out.println("Go to left");
			rotationGauche();
		}
	}

	public static void rotationGauche()
	{

		mA.setSpeed(100);
		mA.backward();
		mC.stop();
		mB.stop();
	}	
	public static void rotationDroite()
	{

		mB.setSpeed(100);
		mA.stop();
		mC.stop();
		mB.forward();
		
	}

	
	public static void directionDroit(float forces[][], float direction[], float valeursInterMoteurs[], int valeursMoteurs[], EV3LargeRegulatedMotor mA, EV3LargeRegulatedMotor mB, EV3LargeRegulatedMotor mC)
	{

		direction[0] = 1;
		direction[1] = 0;
		direction[2] = 0;
		multiplicationMatrice(forces, direction, valeursInterMoteurs, valeursMoteurs);
		lancement(valeursMoteurs, mA, mB, mC);
	}

	public static void deplacementDroite(float forces[][], float direction[], float valeursInterMoteurs[], int valeursMoteurs[], EV3LargeRegulatedMotor mA, EV3LargeRegulatedMotor mB, EV3LargeRegulatedMotor mC)
	{

		direction[0] = 0;
		direction[1] = 1;
		direction[2] = 0;
		multiplicationMatrice(forces, direction, valeursInterMoteurs, valeursMoteurs);
		lancement(valeursMoteurs, mA, mB, mC);
	}

	public static void deplacementGauche(float forces[][], float direction[], float valeursInterMoteurs[], int valeursMoteurs[], EV3LargeRegulatedMotor mA, EV3LargeRegulatedMotor mB, EV3LargeRegulatedMotor mC)
	{

		direction[0] = 0;
		direction[1] = -1;
		direction[2] = 0;
		multiplicationMatrice(forces, direction, valeursInterMoteurs, valeursMoteurs);
		lancement(valeursMoteurs, mA, mB, mC);
	}

	public static void lancement(int valeursMoteurs[], EV3LargeRegulatedMotor mA, EV3LargeRegulatedMotor mB, EV3LargeRegulatedMotor mC)
	{
		//System.out.println("Starting motors");

		if(valeursMoteurs[0] < 0)
		{
			mA.setSpeed(-multiplicateur * valeursMoteurs[0]);
			mA.forward();
		}
		else
		{
			mA.setSpeed(multiplicateur * valeursMoteurs[0]);
			mA.backward();
		}

		if(valeursMoteurs[1] < 0)
		{
			mB.setSpeed(-multiplicateur * valeursMoteurs[1]);
			mB.forward();
		}
		else
		{
			mB.setSpeed(multiplicateur * valeursMoteurs[1]);
			mB.backward();
		}
		if(valeursMoteurs[2] < 0)
		{
			mC.setSpeed(-multiplicateur * valeursMoteurs[2]);
			mC.forward();
		}
		else
		{
			mC.setSpeed(multiplicateur * valeursMoteurs[2]);
			mC.backward();
		}
		//System.out.println(String.format("Large Motor is moving: %s at speed %d", mA.isMoving(), mA.getSpeed()));

	}

	public static void multiplicationMatrice(float forces[][], float direction[], float valeursInterMoteurs[], int valeursMoteurs[])
	{
		valeursMoteurs[0] = 0;
		valeursMoteurs[1] = 0;
		valeursMoteurs[2] = 0;
		valeursInterMoteurs[0] = 0;
		valeursInterMoteurs[1] = 0;
		valeursInterMoteurs[2] = 0;
		for(int i = 0; i < 3; i++)
		{
			for (int j = 0; j < 3; j++)
			{
				valeursInterMoteurs[i] = valeursInterMoteurs[i] + forces[i][j] * direction[j];
			}
		}
		for(int i = 0; i < 3; i++)
		{
			valeursMoteurs[i] = (int)(valeursInterMoteurs[i] * 100);
		}
	}

	public static float telemeterF(){
		float d1 = 0, d2 = 0, delta = 0;

		d1 = getdistanceIR() ;
		d2 = getdistanceUS();
		delta = d1-d2;

		return delta;
	}

	// Fonction pour savoir si ATV est tjrs a plus de 20cm de ISS
	public static float telemeterD(){
		float d1 = 0, d2 = 0, moyenne = 0;
		d1 = getdistanceIR() ;
		d2 = getdistanceUS() ;
		moyenne=(d1+d2)/2;
		return moyenne;
	}

	public static float getdistanceUS () {
		final SampleProvider sp = us1.getDistanceMode();

		float distanceValue = 0;

		final int iteration_threshold = 100;

		float [] sample = new float[sp.sampleSize()];
		sp.fetchSample(sample, 0);
		distanceValue = sample[0];

		System.out.println("Distance US: " + distanceValue);

		return distanceValue;
	}

	public static float getdistanceIR () {
		SampleProvider sp = ir1.getDistanceMode();

		float distanceValue = 0;

		float [] sample = new float[sp.sampleSize()];
		sp.fetchSample(sample, 0);
		distanceValue = sample[0];

		System.out.println("Distance IR: " + distanceValue);
		distanceValue =  distanceValue - 4;
		return (distanceValue);
	}

	public static float getRedIntensity () {
		int value = 0;

		SampleProvider sp = color.getRedMode();

		int sampleSize = sp.sampleSize();
		float[] sample = new float[sampleSize];

		sp.fetchSample(sample, 0);
		value = (int) sample[0];

		//System.out.println("Red intensity: " + value);

		return value;
	}

	public static int getTouchValue () {
		int touchValue = 0;

		SampleProvider sp = touch1.getTouchMode();

		int sampleSize = sp.sampleSize();
		float[] sample = new float[sampleSize];

		sp.fetchSample(sample, 0);
		touchValue = (int) sample[0];

		//System.out.println("Touch value : " + touchValue);

		return touchValue;
	}



}






