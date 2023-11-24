package se.oru.assignment.assignment_oru;

import com.vividsolutions.jts.geom.Coordinate;

import aima.core.util.datastructure.Pair;
import se.oru.assignment.assignment_oru.util.RobotsType.MOBILE_ROBOT;
import se.oru.assignment.assignment_oru.util.RobotsType.ROBOT_TYPE;
import se.oru.coordination.coordination_oru.ConstantAccelerationForwardModel;
import se.oru.coordination.coordination_oru.ForwardModel;

public class Robot {
	
	 protected int robotID;
	 protected ROBOT_TYPE robotType;
	 protected Coordinate[] footprint;
	 protected ForwardModel fm = null;
	 protected double capacity;
	 protected double vel;
	 protected double acc;

	 // DEFAULT VARIABLES
	 /**
		 * The default footprint used for robots if none is specified.
		 * NOTE: coordinates in footprints must be given in in CCW or CW order. 
	*/
	protected static Coordinate[] DEFAULT_FOOTPRINT = new Coordinate[] {
			 	new Coordinate(-1.0,0.5),
				new Coordinate(1.0,0.5),
				new Coordinate(1.0,-0.5),
				new Coordinate(-1.0,-0.5)
	};

	protected static ROBOT_TYPE DEFAULT_TYPE = MOBILE_ROBOT.CARLIKE;

	/**
		 * Create a new {@link Robot} 
		 * @param robotID -> The ID of the Robot
		 * @param robotType -> The type of the Robot
		 * @param footprint -> The footprint of the robot. 
		 * @param fm -> The forward model of the robot.
		 */
	 public Robot(int robotID, ROBOT_TYPE robotType,Coordinate[] footprint, double vel, double acc) {
		 this.robotID = robotID;
		 this.robotType = robotType;
		 this.footprint = footprint;
		 this.vel = vel;
		 this.acc = acc;
		 this.fm = new ConstantAccelerationForwardModel(acc, vel, 1000.0, 1000, 30);	
		 this.capacity = 1.0;
	 }
	 /**
	  * Create a new {@link Robot}; footprint is set to default footprint and default type is = 1
	  * @param robotID ->  The ID of the Robot
	  * @param startingPosition -> The Starting Position of the Robot.
	  */
	 public Robot(int robotID) {
		
		 this(robotID, DEFAULT_TYPE,DEFAULT_FOOTPRINT,6, 4);
	 }
	 
	 /**
	  * Create a new {@link Robot}; footprint is set to default footprint and default type is = 1
	  * @param robotID ->  The ID of the Robot
	  * @param RobotType -> The type of the Robot
	  */
	 public Robot(int robotID,ROBOT_TYPE robotType) {
		 this(robotID,robotType,DEFAULT_FOOTPRINT,6, 4);
	 }
	 
	 
	 /**
	  * Create a new {@link Robot}; footprint is set to default footprint and default type is = 1
	  * @param robotID ->  The ID of the Robot
	  * @param RobotType -> The type of the Robot
	  * @param footprint -> The footprint of the robot
	  */
	 public Robot(int robotID,ROBOT_TYPE robotType,Coordinate[] footprint) {
		 this(robotID,robotType,footprint, 6, 4);
	 }

	 public int getRobotID() {
		 return this.robotID;
	 }

	
	 public ROBOT_TYPE getType() {
		 return this.robotType;
	 }

	 public void setType(ROBOT_TYPE robotType) {
		 this.robotType= robotType;;
	 }
	 

	 public  Coordinate[] getFootprint() {
		 return this.footprint;
	 }

	 public void setFootprint(Coordinate[] footprint) {
		 this.footprint= footprint;;
	 }
	 
	 public  Pair<Double,Double> getKinematicModel() {
		 return new Pair<Double,Double>(vel,acc);
	 }
	 
	 public void setKinematicModel(double vel, double acc) {
		this.vel = vel;
		this.acc = acc;
		this.fm = new ConstantAccelerationForwardModel(acc, vel, 1000.0, 1000, 30);
	 }

	 public void setCapacity(double capacity){
		this.capacity = capacity;
	 }

	 public double getCapacity(){
		return this.capacity;
	 }

	 public double getVel(){
		return this.vel;
	 }

	 public double getAcc(){
		return this.acc;
	 }

	 public String toString() {
		return "Robot " + robotID +" of type " + robotType ; 
	}


}








