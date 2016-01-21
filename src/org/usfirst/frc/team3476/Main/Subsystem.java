package org.usfirst.frc.team3476.Main;

public interface Subsystem
{
	/**
	 * Returns the commands this subsystem can execute.
	 * @return String array containing the subsystem's applicable commands
	 */
	String[] getAutoCommands();
	
	/**
	 * A method to be called by Main to invoke a command from the script.
	 * @param params the parameters to the command to be used with it
	 * @param command the command to be invoked
	 */
	void doAuto(double[] params, String command);
	
	/**
	 * @return True if auto is done
	 */
	boolean isAutoDone();
	
	/**
	 * Method to ask for the request of constants.
	 * Called by Main Thread, not the subsystem Threads.
	 * Should be followed by a call to returnConstantRequest() with the requested constants.
	 * @return the names of the constants to be returned
	 */
	String[] getConstantRequest();
	
	/**
	 * Method to update the state of the constants of this subsystem.
	 * Called by Main Thread, not the subsystem Threads.
	 * @param constantsin the returned constants
	 */
	void returnConstantRequest(double[] constantsin);
	
	/**
	 * Updates the subsystems (called by Thread).
	 */
	void update();
	
	/**
	 * Halts active Threads.
	 */
	void stopThreads();
	
	/**
	 * Resumes active Threads.
	 */
	void startThreads();
	
	/**
	 * Ends all Threads.
	 */
	void terminateThreads();
	
	/**
	 * The method called to shut down the subsystem (safe state).
	 */
	void end();
}
