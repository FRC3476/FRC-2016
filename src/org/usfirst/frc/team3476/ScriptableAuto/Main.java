package org.usfirst.frc.team3476.ScriptableAuto;

import java.io.IOException;
import java.util.ArrayList;
import java.util.Arrays;

import org.usfirst.frc.team3476.Communications.Dashcomm;
import org.usfirst.frc.team3476.Main.*;
import org.usfirst.frc.team3476.Subsystems.StartSubsystem;

/**
 * The main autonomous class.
 * @author Anthony Demetrescu
 *
 */
public class Main
{
	Parser par;
	Subsystem[] systems;
	boolean stop;
	AutoTask auto;
	Thread autoThread;
	
	/**
	 * Constructs an auto Main object.
	 */
	public Main(){}
	
	/**
	 * Testing constructor. Not for robot use.
	 * @param year the constants identifier
	 * @param systemsin the array of robot systems on this robot
	 * @param workingScript the test workingScript String to use
	 * @param constants the constants String to use
	 */
	public Main(String year, Subsystem[] systemsin, String script, String constants)
	{
		par = new Parser(script, constants, year);
		systems = systemsin;
		passConstants();
	}
	
	/**
	 * Initializes this Main with the given year for constants, and the list of subsystems on the robot.
	 * No NT or connection required side effects.
	 * @param year the constants identifier
	 * @param systemsin the array of robot systems on this robot
	 */
	public void initialize(String year, Subsystem[] systemsin)
	{
		systems = systemsin;
		par = new Parser(getScript(), getConstants(), year);
		sendCheckText();
		passConstants();
		stop = false;
		
		auto = new AutoTask(this);
		autoThread = new Thread(auto, "autoThread");
	}
	
	/**
	 * Starts the autonomous thread.
	 */
	public void startThread()
	{
		stop = false;
		if(autoThread.getState() != Thread.State.NEW)
		{
			autoThread = new Thread(auto, "autoThread");
		}
		autoThread.start();//This thread calls start in Main
	}
	
	/**
	 * Starts autonomous. Called by the auto Thread only.
	 */
	private void start()
	{
		System.out.println("start() called");
		reset();
		par.resetScript();
		System.out.println("Script: " + par.getWorkingScript());
		ArrayList<CommandBlock> curCommands;
		Subsystem current;
		boolean done;
		
		while(par.hasNextLine())
		{
			if(stop)break;
			done = false;
			curCommands = par.nextLine();
			Dashcomm.put("command", "" + curCommands);
			while (!done)//Keep going until line is done (ArrayList is empty)
			{
				if(stop)break;
				done = true;
				for(CommandBlock block : curCommands)//Go thru each CommandBlock on this line
				{
					if(stop)break;
					if(block.hasNext())//If there is another command, do things
					{
						current = findSubsystem(block.getCommand());//Grab the subsystem that deals with this command
						if(!block.getCommand().isStarted())//If the command has not been started (new command), start it (duh)
						{							
							current.doAuto(block.getCommand().getParams(), block.getCommand().getName());
							block.getCommand().start();
						}
						else
						{
							if(current.isAutoDone())//If the subsystem is done, remove the command from the queue
							{
								block.finishCommand();
							}
						}
						done = false;
					}
					else{/*No more commands, leave it alone, we are iterating over this ArrayList - throws ConcurrentModificationException*/}
				}
			}
		}
	}
	
	/**
	 * Returns the driver selected autonomous workingScript.
	 * @return the String representation of the workingScript
	 */
	private String getScript()
	{
		return Dashcomm.get("auto/java auto text", "no auto text");
	}
	
	private String[] getScripts()
	{
		return Dashcomm.get("auto/script/scripts", new String[]{"no scripts"});
	}
	
	private String[] getScriptNames()
	{
		return Dashcomm.get("auto/script/script names", new String[]{"no script names"});
	}
	
	private String getSelected()
	{
		return Dashcomm.get("auto/script/selected", "no selection");
	}
	
	/**
	 * Returns the constants files String.
	 * @return the String representation of the constants files
	 */
	private String getConstants()
	{
		return Dashcomm.get("auto/java constants", "no constants");
	}
	
	/**
	 * Passes constants to all subsystems.
	 */
	public void passConstants()
	{
		for(Subsystem current : systems)
		{
			//Return requested constants to the subsystem
			if(current != null) //Whether the subsystem exists or not
			{
				//System.out.println("Current not null");
				String[] request = current.getConstantRequest();
				if(request != null)
				{
					//System.out.println("request not null");
					double[] response = new double[request.length];
					//System.out.println("double received");
					try
					{
						for (int i = 0; i < request.length; i++)
						{
							//System.out.println("getConstant Request 1");
							response[i] = par.getConstant(request[i]);
							//System.out.println("getConstantRequest 2");
						}
						current.returnConstantRequest(response);
						//System.out.println("tryblock done");
					}
					catch (IOException e)
					{
						for (int i = 0; i < response.length; i++)
							response[i] = 0.0;
						current.returnConstantRequest(response);
						System.out.print("exception" + e.getMessage());
					}
					//System.out.println("if done");
				}
				else
				{
					//System.out.println("Subsystem " + current + " gave a null constants request.");
				}
			}
		}
		//System.out.println("passConstants");
	}
	
	/**
	 * Finds the first subsystem that the input command can apply to.
	 * @param command the command to search for
	 * @return the subsystem that can execute the command
	 */
	private Subsystem findSubsystem(Command command)
	{
		for(Subsystem toSearch : systems)
		{
			if(toSearch != null && toSearch.getAutoCommands() != null)
			{
				for(String searchString : toSearch.getAutoCommands())
				{
					if(searchString.equals(command.getName()))
					{
						return toSearch;
					}
				}
			}
		}
		System.out.println("Command \"" + command.getName() + "\" not found.");

		return new ErrorSystem();
	}
	
	/**
	 * Sends the echo of the workingScript back to the Dashboard so that communications can be verified.
	 */
	public void sendCheckText()
	{
		Dashcomm.put("auto/java check text", par.getSelectedScript());
	}
	
	/**
	 * Safely stops the auto Thread through the use of a gateway variable. This method will only return when the Thread has stopped.
	 * @param autoThread the Thread to stop
	 */
	public synchronized void stop()
	{
		stop = true;
		try
		{
			autoThread.join();
		}
		catch(InterruptedException e){}
	}
	
	/**
	 * Calls stopThreads() on every subsystem to halt execution.
	 */
	public synchronized void stopSubsystems()
	{
		for(Subsystem sys: systems)
		{
			if(sys != null)
			{
				sys.stopThreads();
			}
		}
	}
	
	/**
	 * Calls startThreads() on every subsystem to resume execution.
	 */
	public synchronized void startSubsystems()
	{
		for(Subsystem sys: systems)
		{
			if(sys != null)
			{
				sys.startThreads();
			}
		}
	}
	
	/**
	 * Method used to ensure a certain state when starting auto.
	 * Clears the robot drive.
	 */
	private synchronized void reset()
	{
		stop = false;
		robotDriveClear();
	}
	
	/**
	 * Finds the drive subsystem and sets the drive values to zero to appease watchdog when auto is not running.
	 */
	public synchronized void robotDriveClear()
	{
		//System.out.println("robotDriveClear");
		for(Subsystem sys : systems)
		{
			if(sys != null)
			{
				if(sys.toString().toLowerCase().indexOf("drive") != -1)
				{
					StartSubsystem.init(sys);
					return;
				}
			}
		}
		throw new NullPointerException("No drive subsystem found to appease watchdog.");
	}
	
	/**
	 * Updates the workingScript and constants for the Parser so that it can parse the latest selected autonomous and get the latest constants.
	 */
	public void updateData()
	{
		par.update(getScripts(), getScriptNames(), getSelected(), getConstants());
		passConstants();
		sendCheckText();
	}
	
	/**
	 * The task that runs autonomous.
	 * @author Anthony Demetrescu
	 *
	 */
	private class AutoTask implements Runnable
    {
		Main automain;
		public AutoTask(Main automainin)
		{
			automain = automainin;
		}
		
		@Override
		public void run()
		{
			automain.start();
		}
    }
}