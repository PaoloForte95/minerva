package se.oru.assignment.assignment_oru.util;

/**
 * This class is used to set the robotType
 * @author paolodell
 *
 */

public abstract class RobotsType {

	public interface ROBOT_TYPE{

	}
	
	/**
	 * Some type of mobile robots
	 * @author pofe
	 *
	 */
	public static enum MOBILE_ROBOT implements ROBOT_TYPE {ARTICULATED, CARLIKE, FORKLIFT, EXCAVATOR} 
	
	
}



