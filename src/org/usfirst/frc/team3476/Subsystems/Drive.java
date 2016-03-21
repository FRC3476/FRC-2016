	package org.usfirst.frc.team3476.Subsystems;

import org.usfirst.frc.team3476.Main.Subsystem;
import org.usfirst.frc.team3476.Utility.ManualHandler;
import org.usfirst.frc.team3476.Utility.OrangeUtility;
import org.usfirst.frc.team3476.Utility.RunningAverage;
import org.usfirst.frc.team3476.Utility.Control.AsymmetricalRamper;
import org.usfirst.frc.team3476.Utility.Control.BangBang;
import org.usfirst.frc.team3476.Utility.Control.DifferentialAnalogGyro;
import org.usfirst.frc.team3476.Utility.Control.DonutDrive;
import org.usfirst.frc.team3476.Utility.Control.MedianEncoder;
import org.usfirst.frc.team3476.Utility.Control.MedianEncoderPair;
import org.usfirst.frc.team3476.Utility.Control.PIDOutputWrapper;
import org.usfirst.frc.team3476.Utility.Control.Ramper;
import org.usfirst.frc.team3476.Utility.Control.SimpleRamper;

import edu.wpi.first.wpilibj.PIDController;
import edu.wpi.first.wpilibj.PIDSource;
import edu.wpi.first.wpilibj.RobotDrive;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.interfaces.Gyro;

public class Drive implements Subsystem
{
	private final String[] autoCommands = {"turn", "drive", "driven", "shiftit", "clear"};
	private final String[] constants = {"DRIVEDEAD", "DRIVESTRAIGHTDEAD", "TURNDEAD", "USELEFT",
										"USERIGHT", "STRAIGHTP", "STRAIGHTI", "STRAIGHTD", "DRIVEP",
										"DRIVEI", "DRIVED", "TURNP", "TURNI", "TURND", "SHIFTINGSPEED",
										"SHIFTINGHYS", "DRIVEOUTPUTRANGE", "STRAIGHTOUTPUTRANGE",
										"TURNTIMEOUT", "TURNDONUT", "TURNCLAMP", "DONETIME", "STRAIGHTDONUT",
										"ACCELRATE", "DECELRATE", "UNITSPERSEC", "SLOWSPEED", "SPECIALDIST"};
	final int ENCODERSAMPLES = 16;
	final long MANUALTIMEOUT = 50;//in ms
	
	private boolean done, driveStraight, simple, autoShifting, USELEFT, USERIGHT, clear;
	private double scaledTurnTimeout;
	private double 	DRIVEDEAD, DRIVESTRAIGHTDEAD, TURNDEAD, DRIVEP, DRIVEI, DRIVED, TURNP, TURNI, TURND,
					STRAIGHTP, STRAIGHTI, STRAIGHTD, SHIFTINGSPEED, SHIFTINGHYS, DRIVEOUTPUTRANGE,
					STRAIGHTOUTPUTRANGE, TURNTIMEOUT, TURNDONUT, TURNCLAMP, DONETIME, STRAIGHTDONUT,
					ACCELRATE, DECELRATE, UNITSPERSEC, SLOWSPEED, SPECIALDIST;
	
	private ManualHandler driveManual, shifterManual;
	
	AsymmetricalRamper driveRamp;
	
	private MedianEncoder left, right;
	private MedianEncoderPair both;
	private Gyro gyro;
	private RunningAverage encoderAvg, avgRate;
	private DonutDrive driveTrain;
	private Solenoid shifters;
	
	private PIDController drive, turn, straightTurn;
	private PIDOutputWrapper driveWrapper, turnWrapper, straightWrapper;
	private BangBang driven;
	
	private SubsystemTask task;
	private Thread driveThread;
	
	public enum ShiftingState {LOW, HIGH}
	ShiftingState shiftingState;
	
	Timer doneTimer;
	Timer mainTimer;
	private boolean autoDone;
	
	public Drive(MedianEncoder leftin, MedianEncoder rightin, Gyro gyroin, DonutDrive driveTrainin,
			Solenoid shiftersin)
	{
		autoDone = false;
		done = true;
		driveStraight = true;
		simple = false;
		clear = true;
		autoShifting = true;
		shiftingState = ShiftingState.LOW;
		doneTimer = new Timer();
		doneTimer.stop();
		doneTimer.reset();
		
		mainTimer = new Timer();
		mainTimer.stop();
		mainTimer.reset();
		
		left = leftin;
		right = rightin;
		both = new MedianEncoderPair(left, right);
		gyro = gyroin;
		driveTrain = driveTrainin;
		shifters = shiftersin;
		shifterManual = new ManualHandler(50);
		
		encoderAvg = new RunningAverage(ENCODERSAMPLES);
		avgRate = new RunningAverage(ENCODERSAMPLES);
		
		//Driving PID
		driveWrapper = new PIDOutputWrapper(false);
		drive = new PIDController(0, 0, 0, both, driveWrapper);
		drive.disable();
		
		//Turn PID
		turnWrapper = new PIDOutputWrapper(true);
		turn = new PIDController(0, 0, 0, (PIDSource)gyro, turnWrapper);
		turn.disable();
		
		//Drivestraight PID
		straightWrapper = new PIDOutputWrapper(true);
		straightTurn = new PIDController(0, 0, 0, (PIDSource)gyro, straightWrapper);
		straightTurn.disable();
		
		driven = new BangBang(new double[]{0, 0});
		
		task = new SubsystemTask(this, 10);
		driveThread = new Thread(task, "driveThread");
		driveThread.start();
		
		driveManual = new ManualHandler(MANUALTIMEOUT);
		
		driveRamp = new AsymmetricalRamper();
	}
	
	@Override
	public String[] getAutoCommands()
	{
		return autoCommands;
	}

	@Override
	public synchronized void doAuto(double[] params, String command)
	{
		autoShifting = false;
		done = false;
		autoDone = false;
		clear = false;
		mainTimer.reset();
		mainTimer.start();
		doneTimer.reset();
		doneTimer.start();
		switch(command)
		{
			case "turn":
				executeTurn(params[0]);
				break;
			case "drive":
				executeDrive(params[0]);
				break;
			case "driven":
				executeSimpleDrive(params[0], params[1]);
				break;
			case "shiftit":
				setShifterState(params[0] == 1 ? ShiftingState.HIGH : ShiftingState.LOW);
				done = true;
				break;
			case "clear":
				clear = true;
				done = true;
				break;
		}
	}

	@Override
	public synchronized boolean isAutoDone()
	{
		return autoDone;
	}

	@Override
	public String[] getConstantRequest()
	{
		return constants;
	}

	@Override
	public synchronized void returnConstantRequest(double[] constantsin)
	{
		int i = 0;
		DRIVEDEAD = constantsin[i];
		i++;//1
		DRIVESTRAIGHTDEAD = constantsin[i];
		i++;//2
		TURNDEAD = constantsin[i];
		i++;//3
		USELEFT = constantsin[i] == 1;
		i++;//4
		USERIGHT = constantsin[i] == 1;
		i++;//5
		STRAIGHTP = constantsin[i];
		i++;//6
		STRAIGHTI = constantsin[i];
		i++;//7
		STRAIGHTD = constantsin[i];
		i++;//8
		DRIVEP = constantsin[i];
		i++;//9
		DRIVEI = constantsin[i];
		i++;//10
		DRIVED = constantsin[i];
		i++;//11
		TURNP = constantsin[i];
		i++;//12
		TURNI = constantsin[i];
		i++;//13
		TURND = constantsin[i];
		i++;//14
		SHIFTINGSPEED = constantsin[i];
		i++;//15
		SHIFTINGHYS = constantsin[i];
		i++;//16
		DRIVEOUTPUTRANGE = constantsin[i];
		i++;//17
		STRAIGHTOUTPUTRANGE = constantsin[i];
		i++;//18
		TURNTIMEOUT = constantsin[i];
		i++;//19
		TURNDONUT = constantsin[i];
		i++;//20
		TURNCLAMP = constantsin[i];
		i++;//21
		DONETIME = constantsin[i];
		i++;//22
		STRAIGHTDONUT = constantsin[i];
		i++;//23
		ACCELRATE = constantsin[i];
		i++;//24
		DECELRATE = constantsin[i];
		i++;//25
		UNITSPERSEC = constantsin[i];
		i++;//26
		SLOWSPEED = constantsin[i];
		i++;//27
		SPECIALDIST = constantsin[i];
		
		driveRamp.setUnitspersecond(UNITSPERSEC != 0);
		driveRamp.setAccelRate(ACCELRATE);
		driveRamp.setDecelRate(DECELRATE);
		
		driveTrain.setDonutParams(STRAIGHTDONUT, TURNDONUT);
		
		//System.out.println("P: " + TURNP + " I: " + TURNI + " D: " + TURND + " Isvalid: " + constantsin.length);
		both.setUse(USELEFT, USERIGHT);
	}

	@Override
	public synchronized void update()
	{
		if(driveManual.isTimeUp())
		{
			driveTrain.setClamp(1);
			driveTrain.setScale(false);
			if(!clear)
			{
					if (!done)
					{
						if (driveStraight)
						{
							if (!simple)
							{
								driveTrain.arcadeDrive(driveWrapper.getOutput(), straightWrapper.getOutput());
							}
							else
							{
								double bangbang = specialBangBang(both.getDistance());
								//System.out.println("Simple drive with bang bang: " + bangbang + " error: " + driven.getError(both.getDistance()));
								//System.out.println("Straight drive: " + straightTurn.get() + " error: " + straightTurn.getError());
								driveTrain.arcadeDrive(bangbang, straightWrapper.getOutput());
								//System.out.println("Drive setpoint: " + driven.getSetpoint() + " Current pos: " + both.getDistance());
							}
						}
						else//Turning
						{
							if(mainTimer.get() > scaledTurnTimeout) done = true;
							//System.out.print("Turning with output: " + turnWrapper.getOutput() + "  ");
							//System.out.println("Gyro: "  + gyro.calcDiff());
							driveTrain.arcadeDrive(0, OrangeUtility.donut(turnWrapper.getOutput(), TURNDONUT));
						}
						
						//Check if we're done here 
						//TODO: Decide if the drive needs to be in the deadzone for multiple iterations
						boolean driveDone = Math.abs(drive.getSetpoint() - encoderAvg.pidGet()) < DRIVEDEAD;
						boolean drivenDone = Math.abs(driven.getSetpoint() - both.getDistance()) < DRIVEDEAD;
						boolean turnDone = Math.abs(turn.getError()) < TURNDEAD;
						//System.out.println("Turn PID output: " + turn.get() + " error: " + (turn.getError()));
						if(!(driveStraight ? (simple ? drivenDone : driveDone) : turnDone))
						{
							doneTimer.reset();
						}
						if(doneTimer.get() > DONETIME)
						{
							done = true;
						}
						autoDone = done;
					}
					else
					{
							driveTrain.arcadeDrive(0, 0);
					}
			}
			else
			{
					driveTrain.arcadeDrive(0, 0);
			}
		}
		else//manual override
		{
			driveTrain.setClamp(1);
			driveTrain.setScale(true);
		}
		
		if(shifterManual.isTimeUp())
		{
			if(autoShifting)
			{
				switch(shiftingState)
		    	{
		    		case HIGH:
		    			//System.out.print("Case: HIGH, Rate = " + Math.abs(avgRate.getAverage()));
		    			if(Math.abs(avgRate.getAverage()) <= SHIFTINGSPEED - SHIFTINGHYS)
		    			{
		    				shiftingState = ShiftingState.LOW;
		    				setShifterState(shiftingState);
		    			}
		    			break;
		    		case LOW:
		    			//System.out.print("Case: LOW, Rate = " + Math.abs(avgRate.getAverage()));
		    			if(Math.abs(avgRate.getAverage()) >= SHIFTINGSPEED + SHIFTINGHYS)
	    				{
		    				shiftingState = ShiftingState.HIGH;
		    				setShifterState(shiftingState);
	    				}
		    			break;
		    	}
			}
		}
	}
	
	/**
	 * Performs an autonomous PID controlled turn.
	 * @param delta the angle to turn relative to the current angle.
	 */
	public synchronized void executeTurn(double delta)
	{
		simple = false;
		driveStraight = false;
		scaledTurnTimeout = Math.abs(TURNTIMEOUT*delta);
		
		//turn init
		gyro.reset();
		turn.reset();
		turn.setSetpoint(delta);
		turn.setPID(TURNP, TURNI, TURND);
		//subtraction so that the output to the arcade drive does not exceed TURNCLAMP after scaling by donut
		turn.setOutputRange(TURNDONUT - TURNCLAMP, TURNCLAMP - TURNDONUT);
		turn.enable();
	}
	
	/**
	 * Performs an autonomous PID controlled driving maneuver.
	 * Keeps itself on a straight heading with a modified turning PID.
	 * @param delta the distance in inches to move.
	 */
	public synchronized void executeDrive(double delta)
	{
		simple = false;
		driveStraight = true;
		
		//drive init
		drive.reset();
		drive.setSetpoint(both.getDistance() + delta);
		drive.setPID(DRIVEP, DRIVEI, DRIVED);
		drive.setOutputRange(-DRIVEOUTPUTRANGE, DRIVEOUTPUTRANGE);
		drive.enable();
		
		//straightTurn init
		gyro.reset();
		straightTurn.reset();
		straightTurn.setSetpoint(0);
		straightTurn.setPID(STRAIGHTP, STRAIGHTI, STRAIGHTD);
		straightTurn.setOutputRange(-STRAIGHTOUTPUTRANGE, STRAIGHTOUTPUTRANGE);
		straightTurn.enable();
	}
	
	/**
	 * Performs an autonomous BangBang controlled driving maneuver.
	 * Keeps itself on a straight heading with a modified turning PID.
	 * @param delta the distance in inches to move.
	 * @param percentSpeed the 
	 */
	public synchronized void executeSimpleDrive(double delta, double percentSpeed)
	{
		simple = true;
		driveStraight = true;
		driven.setOutputrange(new double[]{percentSpeed/100, -percentSpeed/100});
		driven.setSetpoint(both.getDistance() + delta);
		
		gyro.reset();
		straightTurn.reset();
		straightTurn.setSetpoint(0);
		straightTurn.setPID(STRAIGHTP, STRAIGHTI, STRAIGHTD);
		straightTurn.enable();
	}
	
	/**
	 * Controls the drive base with arcade drive values.
	 * @param move the movement of y value
	 * @param rotate the rotation or x value
	 */
	public void manualDrive(double move, double rotate)
	{
		driveManual.poke();
		driveTrain.arcadeDrive(move, rotate);
	}
	
	public void killDriveManual()
	{
		driveManual.kill();
	}
	
	/**
	 * Controls the drive base with augmented arcade drive values.
	 * @param move the joystick y axis
	 * @param rotate the joystick x axis
	 */
	public void augmentedDrive(double move, double rotate)
	{
		move = driveRamp.doubleAction(move);
		
		manualDrive(move, rotate);
	}
	
	public void resetRamp()
	{
		driveRamp.reset();
	}
	
	/**
	 * Controls the shifters with a boolean.
	 * @param state the state of the shifters
	 */
	public void manualShift(ShiftingState state)
	{
		shifterManual.poke();
		setShifterState(state);
	}
	
	public String toString()
	{
		return "Drive";
	}
	
	public void stopThreads()
	{
		task.hold();
	}
	
	public void terminateThreads()
	{
		task.terminate();
		try
		{
			driveThread.join();
			System.out.println("Ended " + this + " thread.");
		}
		catch(InterruptedException e)
		{
			System.out.println("Ended " + this + " thread.");
		}
	}
	
	/**
	 * Tells the drive system to automatically shift or not.
	 * @param auto whether or not to auto-shift
	 */
	public void autoShifting(boolean auto)
	{
		autoShifting = auto;
	}
	
	/**
	 * Sets the shifter state.
	 * @param state the state to shift to
	 */
	public synchronized void setShifterState(ShiftingState state)
	{
		switch(state)
		{
			case HIGH:
				shifters.set(true);
				break;
			case LOW:
				shifters.set(false);
				break;
		}
	}
	
	@Override
	public void startThreads()
	{
		task.resume();
	}

	@Override
	public void end()
	{
		clear = true;
		setShifterState(ShiftingState.LOW);
	}
	
	/**
	 * Augments the simple BangBang driving algorithm by slowing down near the end if faster than a certain value.
	 * @param process the process variable - distance
	 * @return the modified output
	 */
	private double specialBangBang(double process)
	{
		if(Math.abs(driven.getError(process)) < SPECIALDIST)
		{
			double speed = Math.min(Math.abs(driven.getOutputrange()[0]), SLOWSPEED);
			driven.setOutputrange(new double[]{speed, -speed});
		}
		return driven.output(process);
	}

	@Override
	public boolean threadsActive()
	{
		return task.isActive();
	}
}
