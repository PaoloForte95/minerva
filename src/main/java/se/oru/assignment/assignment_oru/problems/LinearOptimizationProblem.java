package se.oru.assignment.assignment_oru.problems;

import java.util.ArrayList;
import java.util.Calendar;
import java.util.TreeSet;
import java.util.HashMap;

import org.apache.commons.lang.ArrayUtils;
import org.metacsp.multi.spatioTemporal.paths.PoseSteering;
import org.metacsp.multi.spatioTemporal.paths.TrajectoryEnvelope;
import org.metacsp.multi.spatioTemporal.paths.TrajectoryEnvelope.SpatialEnvelope;
import org.metacsp.utility.UI.Callback;
import org.metacsp.utility.logging.MetaCSPLogging;

import aima.core.util.datastructure.Pair;

import com.vividsolutions.jts.geom.Coordinate;
import com.vividsolutions.jts.geom.Geometry;
import com.vividsolutions.jts.geom.Polygon;

import se.oru.coordination.coordination_oru.AbstractTrajectoryEnvelopeCoordinator;
import se.oru.coordination.coordination_oru.CriticalSection;
import se.oru.coordination.coordination_oru.Mission;
import se.oru.coordination.coordination_oru.RobotReport;
import se.oru.coordination.coordination_oru.motionplanning.AbstractMotionPlanner;
import se.oru.coordination.coordination_oru.util.Missions;
import se.oru.assignment.assignment_oru.ComputePathCallback;
import se.oru.assignment.assignment_oru.IndexedDelay;
import se.oru.assignment.assignment_oru.Robot;
import se.oru.assignment.assignment_oru.Task;
import se.oru.assignment.assignment_oru.TaskAssignmentCallback;
import se.oru.assignment.assignment_oru.delay.EclDelayEvaluator;
import se.oru.assignment.assignment_oru.delay.SimpleDelayEvaluator;
import se.oru.assignment.assignment_oru.fleetmasterinterface.AbstractFleetMasterInterface;
import se.oru.assignment.assignment_oru.fleetmasterinterface.FleetMasterInterface;
import se.oru.assignment.assignment_oru.methods.AbstractOptimizationAlgorithm;
import se.oru.assignment.assignment_oru.util.RobotsType.MOBILE_ROBOT;
import se.oru.assignment.assignment_oru.util.TaskFleetVisualization;



/**
 * Class to create an online Single Task, Single Robot, Instantaneous Assignment (ST–SR–IA) Multi-Robot Task Assignment(MRTA) problem. 
 * The problem is posed as an online Optimal Assignment Problem (OAP)
	 * The constraints considered are :
	 * 1) Each Task can be assign only to a robot;
	 * 2) Each Robot can perform only a task at time;
	 * 3) Each robot perform a task following a single path.
	 * This class used the coordination_oru interface to coordinate the robots while driving.
 * @author pofe
 */
public class LinearOptimizationProblem extends LinearOptimization{
	 
		//Weights for the Interference free-cost functions
		protected double pathLengthWeight = 1;
		protected double arrivalTimeWeight = 0;
		protected double tardinessWeight = 0;
		
		//Normalizing factors
		protected double sumTardiness = 1;
		protected double sumMaxPathsLength = 1; //This normalizing factor is obtained by summing the longest paths for each idle robot in the fleet
		protected double sumArrivalTime = 1; //This normalizing factor is the sum of arrival time of completing the longest path  for each robot
		
		protected double slowestRobotVelocity;
		protected double slowestRobotAcceleration;
		
		//Parameters for time analysis
		protected long timeRequiretoEvaluatePaths;
		protected long timeRequiretofillInPall;
		protected long timeRequiretoComputeCriticalSection;
		protected long timeRequiretoComputePathsDelay;

		protected int numAllocation;

		protected boolean checkBlocking;

		//Paths 
		protected HashMap<Integer, PoseSteering[]> pathsToTargetGoal =  new HashMap<Integer, PoseSteering[]>();
		protected ArrayList <SpatialEnvelope> pathsDrivingRobots = new ArrayList <SpatialEnvelope>();
		
		//FleetMaster Interface Parameters	
		protected AbstractFleetMasterInterface eclInterface;
		protected boolean propagateDelays;

		//Coordinator
		protected AbstractTrajectoryEnvelopeCoordinator coordinator;

		//Thread Parameters 
		protected int CONTROL_PERIOD_TASK = 20000;
		protected static int EFFECTIVE_CONTROL_PERIOD_TASK = 0;

		//ROADMAP Parameters
		protected String scenario;
		protected boolean saveFutureAllocations = false;



		/**
		 * The default footprint used for robots if none is specified.
		 * NOTE: coordinates in footprints must be given in in CCW or CW order. 
		*/
		private static Coordinate[] DEFAULT_FOOTPRINT = new Coordinate[] {
			new Coordinate(-1.0,0.5),
			new Coordinate(1.0,0.5),
			new Coordinate(1.0,-0.5),
			new Coordinate(-1.0,-0.5)
		};

		//How to compute paths, default mechanism
		protected ComputePathCallback pathCB = new ComputePathCallback() {
			@Override
			public PoseSteering[] computePath(Task task, int pathNumber, RobotReport rr) {
				AbstractMotionPlanner rsp =  coordinator.getMotionPlanner(rr.getRobotID()).getCopy(true);
				rsp.setStart(rr.getPose());
				rsp.setGoals(task.getStartPose(),task.getGoalPose());
				rsp.setFootprint(coordinator.getFootprint(rr.getRobotID()));
				
				if (!rsp.plan()) {
					System.out.println("Robot" + rr.getRobotID() +" cannot reach the Target End of Task " + task.getID());
					//the path to reach target end not exits
					pathsToTargetGoal.put(rr.getRobotID()*numTaskAug*alternativePaths+task.getID()*alternativePaths+pathNumber, null);		
					return null;
				}			
				return rsp.getPath();
			}
		};

		/**
		 * Enable the saving of the scenarios (Missions + paths) for all the optimization problems that will be solved.
		 * 
		 */
		public void saveAllScenarios() {
			this.saveFutureAllocations = true;
			
		}

		/**
		 * Load a Scenario (paths + missions) 
		 * @param scenario ->  Scenario to load
		 */
		
		 public void loadScenario(String scenario) {
			this.scenario = scenario;
		}

				/**
		 * Specifies anything that should be done when a task is assigned to a robot.
		 * @param taskCB
		 */
		public void setTaskAssignmentCallback(TaskAssignmentCallback taskCB) {
			this.taskCB = taskCB;
		}

		/**
			 * Add a {@link ComputePathCallback} that is triggered whenever a path is computed.
			 * @param cb The {@link ComputePathCallback} to be called whenever a path is computed.
			 */
			public void setComputePathCallback(ComputePathCallback cb) {
				this.pathCB = cb;
		}

			
		/**
		 * Load an assignment.
		 * @param assignment ->  Assignment to load 
		 */
		
		 public void loadAssignment(int [][][] assignment) {
			this.optimalAssignment = assignment;
		}

		/**
		 * Set the Fleet Visualization.
		 * @param viz -> An instance of a TaskFleetVisualization
		 */
				
		public void setFleetVisualization(TaskFleetVisualization viz) {
			this.viz = viz;
		}


		/**
		 *Create an Single Task, Single Robot, Instantaneous Assignment (ST–SR–IA) Multi-Robot Task Assignment(MRTA) problem posed as an Optimal Assignment Problem.
		 */
		public LinearOptimizationProblem(){
			this(false);
		}


		/**
		 *Create an Single Task, Single Robot, Instantaneous Assignment (ST–SR–IA) Multi-Robot Task Assignment(MRTA) problem posed as an Optimal Assignment Problem.
		 */
		public LinearOptimizationProblem(boolean instanceFleetMaster){
			super();
			if(instanceFleetMaster){
				instantiateFleetMaster(0.1, false);
				propagateDelays = false;
				delayEvaluator = new EclDelayEvaluator(this.eclInterface);
			}else {
				delayEvaluator = new SimpleDelayEvaluator();
			}
			metaCSPLogger = MetaCSPLogging.getLogger(this.getClass());
			numAllocation = 1;
			checkBlocking = false;
			
		}

		@Override
		public boolean addRobot(Robot robot) {
			if(delayEvaluator instanceof EclDelayEvaluator){
				int robotID = robot.getRobotID();
				eclInterface.setTrajParams(robotID, coordinator.getRobotMaxVelocity(robotID), coordinator.getRobotMaxAcceleration(robotID));
				if(robot.getType() == MOBILE_ROBOT.ARTICULATED){
					eclInterface.setRobotType(robotID, 1);
				}
				else if(robot.getType() == MOBILE_ROBOT.CARLIKE){
					eclInterface.setRobotType(robotID, 2);
				}
				else {
					metaCSPLogger.severe(("Other cases are not supported yet. Please select between ARTICULATED and CARLIKE"));
				}
			}
			return super.addRobot(robot);
		}

		/**
		 * Return the biggest footprint of the robots in fleet
		*/
		protected Coordinate[] getMaxFootprint() {
			double maxArea = 0.0;
			Coordinate[] maxFootprint = DEFAULT_FOOTPRINT;
			for(Robot rb :  robots){
				Geometry fpGeom = TrajectoryEnvelope.createFootprintPolygon(rb.getFootprint());
				double robotFootprintArea = fpGeom.getArea();
				if(robotFootprintArea > maxArea) {
					maxArea = robotFootprintArea;
					maxFootprint = fpGeom.getCoordinates();
				}
			}
			return maxFootprint;
		}

		/**
		 * Enable and initialize the fleetmaster library to estimate precedences to minimize the overall completion time.
		 * Note: this function should be called before placing the first robot.
		 * ATTENTION: If dynamic_size is <code>false</code>, then the user should check that all the paths will lay in the given area.
		 * @param origin_x The x coordinate (in meters and in global inertial frame) of the lower-left pixel of fleetmaster GridMap.
		 * @param origin_y The y coordinate (in meters and in global inertial frame) of the lower-left pixel of fleetmaster GridMap.
		 * @param origin_theta The theta coordinate (in rads) of the lower-left pixel map (counterclockwise rotation). Many parts of the system currently ignore it.
		 * @param resolution The resolution of the map (in meters/cell), 0.01 <= resolution <= 1. It is assumed this parameter to be global among the fleet.
		 * 					 The highest the value, the less accurate the estimation, the lowest the more the computational effort.
		 * @param width Number of columns of the map (>= 1) if dynamic sizing is not enabled.
		 * @param height Number of rows of the map (>= 1) if dynamic sizing is not enabled.
		 * @param dynamic_size If <code>true</code>, it allows to store only the bounding box containing each path.
		 * @param propagateDelays If <code>true</code>, it enables the delay propagation.
		 * @param debug If <code>true</code>, it enables writing to screen debugging info.
		 */
		public void instantiateFleetMaster(double origin_x, double origin_y, double origin_theta, double resolution, long width, long height, boolean dynamic_size, boolean propagateDelays, boolean debug) {
			this.eclInterface = new FleetMasterInterface(origin_x, origin_y, origin_theta, resolution, width, height, dynamic_size, debug);
			this.eclInterface.setDefaultFootprint(DEFAULT_FOOTPRINT);
			this.propagateDelays = propagateDelays;
		}
		
		/**
		 * Enable and initialize the fleetmaster library to estimate precedences to minimize the overall completion time
		 * while minimizing the computational requirements (bounding box are used to set the size of each path-image).
		 * Note: this function should be called before placing the first robot.
		 * @param resolution The resolution of the map (in meters/cell), 0.01 <= resolution <= 1. It is assumed this parameter to be global among the fleet.
		 * 					 The highest the value, the less accurate the estimation, the lowest the more the computational effort.
		 * @param propagateDelays If <code>true</code>, it enables the delay propagation.
		 */
		public void instantiateFleetMaster(double resolution, boolean propagateDelays) {
			instantiateFleetMaster(0., 0., 0., resolution, 1000, 1000, true, propagateDelays, false);
		}
			

		protected Pair<Double,Double> estimateTimeToCompletionDelays(int robot1ID,PoseSteering[] pss1, int robot2ID,PoseSteering[] pss2){
			return delayEvaluator.evaluatePathDelay(robot1ID,pss1,robot2ID,pss2);
		}

		/**
		 * Delete one or multiple tasks from the queue to avoid blocking. Blocking happens a task execution is blocked by the execution 
		 * of another task (i.e. two tasks may have a common or near goal location). 
		 * The deleted task will be postponed and execute later. If deadlines are present, them will be considering into the evaluation
		 * (the task with the shortest deadline will be prioritized)
		*/
		protected void checkOnBlocking() { 
			int tasksPosponed = 0;
			Coordinate[] taskFootprint = getMaxFootprint();
			for(int j=0; j < taskQueue.size(); j++ ) {
				Task currentTask = taskQueue.get(j);
				double xTask=currentTask.getGoalPose().getX();
				double yTask=currentTask.getGoalPose().getY();
				double dist1 = taskFootprint[0].distance(taskFootprint[1])/2;
				double dist2 = taskFootprint[1].distance(taskFootprint[2])/2;
				Coordinate Taskfootprint1 = new Coordinate((xTask-dist1),(yTask+dist2));
				Coordinate Taskfootprint2 = new Coordinate((xTask+dist1),(yTask+dist2));
				Coordinate Taskfootprint3 = new Coordinate((xTask+dist1),(yTask-dist2));
				Coordinate Taskfootprint4 = new Coordinate((xTask-dist1),(yTask-dist2));
				Polygon ll = TrajectoryEnvelope.createFootprintPolygon(Taskfootprint1,Taskfootprint2,Taskfootprint3,Taskfootprint4);
				
				for(int k = 0; k < taskQueue.size(); k++ ) {
					if(k != j) {
						Task taskProva2 = taskQueue.get(k);
						double xTask2=taskProva2.getGoalPose().getX();
						double yTask2=taskProva2.getGoalPose().getY();
						Coordinate Taskfootprint5 = new Coordinate((xTask2-dist1),(yTask2+dist1));
						Coordinate Taskfootprint6 = new Coordinate((xTask2+dist1),(yTask2+dist2));
						Coordinate Taskfootprint7 = new Coordinate((xTask2+dist1),(yTask2-dist2));
						Coordinate Taskfootprint8 = new Coordinate((xTask2-dist1),(yTask2-dist2));
						Polygon gg = TrajectoryEnvelope.createFootprintPolygon(Taskfootprint5,Taskfootprint6,Taskfootprint7,Taskfootprint8);
						if(ll.intersects(gg) ) {
							if(taskProva2.getDeadline() == -1 && currentTask.getDeadline() == -1) {
								taskQueue.remove(k);
								taskPosponedQueue.add(taskProva2);
								tasksPosponed += 1;
							}else {
								if (taskProva2.getDeadline() == -1 && currentTask.getDeadline() != -1){
									taskQueue.remove(k);
									taskPosponedQueue.add(taskProva2);
									tasksPosponed += 1;

								}else if(taskProva2.getDeadline() != -1 && currentTask.getDeadline() == -1) {
									taskQueue.remove(j);
									taskPosponedQueue.add(currentTask);
									tasksPosponed += 1;
									j -=1;
									break;
								}else {
									if(taskProva2.getDeadline() > currentTask.getDeadline() ) {
										taskQueue.remove(k);
										taskPosponedQueue.add(taskProva2);
										tasksPosponed += 1;
									}else {
										taskQueue.remove(j);
										taskPosponedQueue.add(currentTask);
										tasksPosponed += 1;
										j -=1;
										break;
									}	
								}
							}
						}
					}
				}
			} //end for loop
			metaCSPLogger.info("Task posponed for avoid blocking : " + tasksPosponed);
			}

			/**
			 * Set the Coordinator. The coordinator will be used to manage the fleet and solve the coordination problem (avoid collision)
			 * @param coordinator -> An instance of a AbstractTrajectoryEnvelopeCoordinator
			 */
			
			public void setCoordinator(AbstractTrajectoryEnvelopeCoordinator coordinator) {
				this.coordinator = coordinator;
			}

			/**
			 * Set the weights of cost functions considered in function B (i.e. all costs related to the single robot) for this Optimization Problem. These must be numbers between 0 and 1.
			 * Three costs functions are considered into B for this optimization problem: path length, nominal arrival time, tardiness. This function allow to set the weight (i.e. the importance) to each of then
			 * Default values are (1,0,0)
			 * @param pathLengthWeight ->  The path length weight;
			 * @param arrivalTimeWeight -> The arrival time weight;
			 * @param tardinessWeight -> The tardiness weight
			 */
			
			public void setCostFunctionsWeight(double pathLengthWeight,double arrivalTimeWeight,double tardinessWeight) {
				if(pathLengthWeight <0 || arrivalTimeWeight < 0 ||  tardinessWeight < 0) {
					throw new Error("Weights cannot be  numbers less than 0!");
				}
				double sumWeight = pathLengthWeight + arrivalTimeWeight + tardinessWeight;
				if(sumWeight != 1) {
					throw new Error("The sum of weights must be equal to 1!");
				}
				this.pathLengthWeight = pathLengthWeight;
				this.arrivalTimeWeight = arrivalTimeWeight;
				this.tardinessWeight = tardinessWeight;
			}
			
			
			/**
			 * Computes the minimum values of maximum velocity and acceleration considering all robots of the fleet. 
			 * This is used as a estimation of the longest nominal time (i.e. time without interference among robots) to complete a path. 
			 */
			
			protected void computeMaxVelandAccel() {
				double maxVel = Integer.MAX_VALUE;
				double maxAcc = Integer.MAX_VALUE;
				for(int robotID: coordinator.getIdleRobots()) {
					if(maxVel > coordinator.getRobotMaxVelocity(robotID)) {
						maxVel = coordinator.getRobotMaxVelocity(robotID);
					}
					if(maxAcc > coordinator.getRobotMaxAcceleration(robotID)){
						maxAcc = coordinator.getRobotMaxAcceleration(robotID);
					}
				}
				this.slowestRobotVelocity = maxVel;
				this.slowestRobotAcceleration = maxAcc;
			}
			
		/**
		 * Check if a goal can be reached by at least one robot of the Fleet . If not, a dummy robot and task are added for each 
		 * unreachable location, to make the problem square (equal number of robot and task). If one location cannot be reached
		 * the task associated to that location will be assigned to a virtual robot, while a virtual task will be associated to 
		 * a real robot (which one will depend on other tasks).
		 * @param PAll -> the 3D matrix of all paths 
		 * @return An updated PAll incremented by 1 on each direction (i.e. i, j, p_ij)
		 */
		
		protected  double [][][] checkTargetGoals (double [][][] PAll){
			for (int j= 0; j< PAll[0].length ; j++) {
				boolean targetEndCanBeReach = false;
				for (int i = 0; i < PAll.length; i++) {
					for (int s = 0; s < alternativePaths; s++) {
						if(PAll[i][j][s] != Double.POSITIVE_INFINITY) {
							targetEndCanBeReach = true;
						}
					}
				}
				//no robot can reach the target end -> need to introduce a dummy task and robot 
				if(!targetEndCanBeReach) {
					dummyRobot += 1 ;
					dummyTask += 1 ;
					numRobotAug += 1;
					numTaskAug += 1;
		
				}	
			}
			double [][][] PAllAug = new double [numRobotAug][numTaskAug][alternativePaths];
			int robotID = 0;
			int taskID = 0;
			for(int i = 0;i < numRobotAug; i++) {
				for(int j = 0; j<  numTaskAug;j++) {
					for (int s = 0; s < alternativePaths; s++) {
						//copy the values from Pall
						if(i < PAll.length && j< PAll[0].length) {
							PAllAug[i][j][s] = PAll[i][j][s];
						}else {
							if(i < robotsIDs.size()) {
								robotID = robotsIDs.get(i);						
							}else {
								robotID = this.virtualRobotID -1 ;
								robotsIDs.add(robotID);
							}
							if (j < tasksIDs.size()) {
								taskID = tasksIDs.get(j);
							}else {
								taskID = virtualTaskID - 1;
								this.virtualTaskID -= 1;
								tasksIDs.add(taskID);
							}
							PAllAug[i][j][s] = 1;
							pathsToTargetGoal.put(robotID*numTaskAug*alternativePaths+taskID*alternativePaths+s, null); //FIXME null may be not correct
					
						}
					}
						
				}
			}
			return PAllAug;
		}
			
		/**
		 * Evaluate the PAll matrix, that is a matrix that contains all path for each possible combination of robot
		 * and task
		 * If a path between a couple of robot and task does not exists, the cost is consider infinity.
		 * @return The PAll matrix
		 */
		protected double [][][] evaluatePAll(){
			
		
			if(this.slowestRobotAcceleration == 0 && this.slowestRobotVelocity == 0) {
				computeMaxVelandAccel();
			}
			
			//Evaluate the path length for the actual couple of task and ID
			//Initialize the sum of max paths lengths and time to do it for each robot
			//This cost are used then for normalizing cost
			double sumPathsLength = 0;
			double sumArrivalTime = 0;
			double [][][] PAll = new double[numRobotAug][numTaskAug][alternativePaths];
			
			
			
			long timeInitial = Calendar.getInstance().getTimeInMillis();
			if(scenario != null) {
				double [][][] PAllScenario = evaluatePAllWithScenario();
				return PAllScenario;
			}
			for (int robotID : robotsIDs) {
				
				//double maxPathLength = 1;
				//int robotIndex = robotsIDs.indexOf(robotID);
				for (int taskID : tasksIDs ) {
					int taskIndex = tasksIDs.indexOf(taskID);
					
					//Evaluate path Length
					boolean typesAreEqual = false;
					if (realRobotsIDs.contains(robotID) && realTasksIDs.contains(taskID) ) {
					//typesAreEqual = taskQueue.get(taskIndex).isCompatible(coordinator.getRobot(robotID)); getRobotTypes
					typesAreEqual = taskQueue.get(taskIndex).isCompatible(getRobotType(robotID));
					}
					else {
						//Considering a dummy robot or  a dummy task -> they don't have type
						typesAreEqual = true;
					}
					for(int path = 0;path < alternativePaths; path++) {
						final int pathID = path;
						if(typesAreEqual) { // only if robot and task have the same types
		
							
								// evaluatePathLength(robotID,taskID,pathID,tec);	
								new Thread("Robot" + robotID) {
									public void run() {
											evaluatePathLength(robotID,taskID,pathID);								
									}		
								}.start();
								//Take time to evaluate the path
					
						}			 
						else {
								pathsToTargetGoal.put(robotID*numTaskAug*alternativePaths+taskID*alternativePaths+path, null);		
						}	
						}
				}//For Task
		
			}
			boolean allResultsReady = false;
			while (!allResultsReady) {
				allResultsReady = true;
				
				for (int robotID : realRobotsIDs) {
					for (int task : tasksIDs) {
						for(int path=0;path < alternativePaths ; path ++) {
							if (!pathsToTargetGoal.containsKey(robotID*numTaskAug*alternativePaths+task*alternativePaths+path) )  {
								allResultsReady = false;
							}
						}
						
					}
					try { Thread.sleep(500); }
					catch (InterruptedException e) {
						e.printStackTrace();
					}
					}
					
			}
			metaCSPLogger.severe("All paths are computed.");
			long timeFinal = Calendar.getInstance().getTimeInMillis();
			long timeRequired = timeFinal- timeInitial;
			timeRequiretoEvaluatePaths = timeRequiretoEvaluatePaths + timeRequired;
			long timeInitial2 = Calendar.getInstance().getTimeInMillis();
			for (int robotID : robotsIDs) {
				int robotindex = robotsIDs.indexOf(robotID);
				double maxPathLength = 1;
				for (int taskID : tasksIDs ) {
					int taskIndex = tasksIDs.indexOf(taskID);
					double pathLength = Double.POSITIVE_INFINITY;
					//Evaluate path Length
					boolean typesAreEqual = false;
					if (taskIndex < taskQueue.size() && robotindex < realRobotsIDs.size() ) {
					//typesAreEqual = taskQueue.get(taskIndex).isCompatible(coordinator.getRobot(robotID));
					typesAreEqual = taskQueue.get(taskIndex).isCompatible(getRobotType(robotID));
					}
					else {
						//Considering a dummy robot or  a dummy task -> they don't have type
						typesAreEqual = true;
					}
					for(int path = 0;path < alternativePaths; path++) {
						if(typesAreEqual) { 
							if(pathsToTargetGoal.get(robotID*numTaskAug*alternativePaths+taskID*alternativePaths+path) != null) {
								pathLength = Missions.getPathLength(pathsToTargetGoal.get(robotID*numTaskAug*alternativePaths+taskID*alternativePaths+path));
							}
							if ( pathLength > maxPathLength && pathLength != Double.POSITIVE_INFINITY) {
									maxPathLength = pathLength;
							}
						}
						
						PAll[robotindex][taskIndex][path] = pathLength;
					}//For path
				}//For task
				//Sum the max path length for each robot
				sumPathsLength += maxPathLength;
				//Sum the arrival time for the max path length
				sumArrivalTime += computeArrivalTime(maxPathLength,this.slowestRobotVelocity,this.slowestRobotAcceleration);
				}
			//Take the time to fill in the PAll Matrix
			long timeFinal2 = Calendar.getInstance().getTimeInMillis();
			long timeRequired2 = timeFinal2- timeInitial2;
			timeRequiretofillInPall = timeRequiretofillInPall + timeRequired2;
			double [][][] PAllAug =  checkTargetGoals(PAll);
			//Save the sum of max paths length to normalize path length cost
			this.sumMaxPathsLength = sumPathsLength;
			//Save the sum of arrival time considering max paths length to normalize delay cost
			this.sumArrivalTime = sumArrivalTime;
			//Return the cost of path length
			//metaCSPLogger.info("Scenario is saved with name" + " scenario#" + numAllocation);
			//Missions.saveScenario("scenario# "+ numAllocation);
			if(saveFutureAllocations == true) {
				numAllocation += 1;
			}
			
			//Remove all the missions from Missions set stored in the coordinator.
			for (int robotID : realRobotsIDs) {
				int cont = 0;
				//for (int taskID : tasksIDs ) {
					if(Missions.getMissions(robotID) != null){
						if(cont < Missions.getMissions(robotID).size()) {
							Mission m1 = Missions.getMission(robotID, cont);
							Missions.removeMissions(m1);
							
							//cont +=1;
						}
					}
					
				//}
				
			}
			return PAllAug;
			}
		

		protected synchronized double evaluatePathLength(int robotID , int taskID, int alternativePath){
			//Evaluate the path length for the actual couple of task and ID
			//Initialize the path length to infinity
			double pathLength = Double.POSITIVE_INFINITY;
			//take index positon of robotID in Robot set
			int robotindex = realRobotsIDs.indexOf(robotID);
			// Only for real robots and tasks
			if (realRobotsIDs.contains(robotID) && realTasksIDs.contains(taskID)) {
				//Take the state for the i-th Robot
				RobotReport rr = coordinator.getRobotReport(realRobotsIDs.get(robotindex));
				if (rr == null) {
					metaCSPLogger.severe("RobotReport not found for Robot" + robotID + ".");
					throw new Error("RobotReport not found for Robot" + robotID + ".");
				}
				//Evaluate the path from the Robot Starting Pose to Task End Pose
				int taskIndex = realTasksIDs.indexOf(taskID);
				AbstractMotionPlanner rsp =  coordinator.getMotionPlanner(robotID).getCopy(true);
				
				rsp.setStart(rr.getPose());
				rsp.setGoals(taskQueue.get(taskIndex).getStartPose(),taskQueue.get(taskIndex).getGoalPose());
				rsp.setFootprint(coordinator.getFootprint(robotID));
				
				if (!rsp.plan()) {
					System.out.println("Robot" + robotID +" cannot reach the Target End of Task " + taskID);
					//the path to reach target end not exits
					pathsToTargetGoal.put(robotID*numTaskAug*alternativePaths+taskID*alternativePaths+alternativePath, null);		
					//Infinity cost is returned 
					
					return pathLength;
					
				}			
				//If the path exists
				//Take the Pose Steering representing the path
				PoseSteering[] pss = rsp.getPath();
				
				System.out.println("Robot " +robotID +" taskID "+ taskID +" throw Path " + pss[pss.length-1].getX() + " " + pss[pss.length-1].getY() + " " + pss[pss.length-1].getTheta());
				
				
				//Add the path to the FleetMaster Interface -> this is necessary for F function
				//addPath(robotID, pss.hashCode(), pss, null, coordinator.getFootprint(robotID)); 
				//SpatialEnvelope se1 = TrajectoryEnvelope.createSpatialEnvelope(pss,tec.getFootprint(robotID));
				//Geometry kk = se1.getPolygon();
				//addPath(robotID, pss.hashCode(), pss, kk, tec.getFootprint(robotID));
				
				//Save the path to Task in the path set
				pathsToTargetGoal.put(robotID*numTaskAug*alternativePaths+taskID*alternativePaths+alternativePath, pss);
				//Take the Path Length
				Mission m1 = new Mission(robotID,pss);
				Missions.enqueueMission(m1);
				pathLength = Missions.getPathLength(pss);
				
			} else { //There also virtual robot and task are considered 
				//There are considered real robot and dummy task
				if (realRobotsIDs.size() >= taskQueue.size() && realRobotsIDs.contains(robotID)){ //dummy task -> The Robot receive the task to stay in starting position
					//The second condition is used in the special case in which we have that one robot cannot be 
					//assigned to any tasks due to its type, so we must add a dummy robot and a dummy task, but we 
					//Create the task to stay in robot starting position
					PoseSteering[] dummyTask = new PoseSteering[1];
					//Take the state for the i-th Robot
					RobotReport rr = coordinator.getRobotReport(realRobotsIDs.get(robotindex));
					if (rr == null) {
						metaCSPLogger.severe("RobotReport not found for Robot" + robotID + ".");
						throw new Error("RobotReport not found for Robot" + robotID + ".");
					}
					//take the starting position of the robot
					dummyTask[0] = new PoseSteering(rr.getPose(),0);
					//Add the path to the FleetMaster Interface -> so it can be considered as an obstacle from 
					//the motion planner
					
					System.out.println("Robot " +robotID +" taskID "+ taskID +" throw Path " + dummyTask[dummyTask.length-1].getX() + " " + dummyTask[dummyTask.length-1].getY() + " " + dummyTask[dummyTask.length-1].getTheta());
					
					//addPath(robotID, dummyTask.hashCode(), dummyTask, null, coordinator.getFootprint(robotID));
					//Save the path to Dummy Task 
					pathsToTargetGoal.put(robotID*numTaskAug*alternativePaths+taskID*alternativePaths+alternativePath, dummyTask);		
					//Consider a minimal pathLength
					pathLength = 1;
					Mission m1 = new Mission(robotID,dummyTask);
					Missions.enqueueMission(m1);
			
					return pathLength;
				}
				else { //There are considered dummy robot and real task
					//dummy robot -> Consider a only virtual Robot 
					PoseSteering[] dummyRobot = new PoseSteering[1];
					dummyRobot[0] = new PoseSteering(taskQueue.get(0).getGoalPose(),0);
					pathsToTargetGoal.put(robotID*numTaskAug*alternativePaths+taskID*alternativePaths+alternativePath, dummyRobot);
					pathLength = 1;
					Mission m1 = new Mission(robotID,dummyRobot);
					Missions.enqueueMission(m1);
					return pathLength;
				}	
			}
			return pathLength;
		}
		

		
		
		
		/**
		 * Evaluate the 3D matrix Pall for all the possible combination of paths p_ij to reach a task j for a robot i, using a pre-computed Scenario
		 * @return the 3D matrix Pall of all possible paths, where each element of the matrix [i] [i] [p_ij] represents the length of the path for the robot i to reach the end of task j 
		 */
		protected double [][][] evaluatePAllWithScenario(){
			
			//Evaluate the path length for all the possible paths for the actual couple of task and robot
			//This cost are used then for normalizing the cost
			double sumPathsLength = 0;
			double sumArrivalTime = 0;

			Missions.loadScenario(scenario);
			
			double [][][] PAll = new double[numRobotAug][numTaskAug][alternativePaths];
			for (int robotID : robotsIDs) {
				int robotindex = robotsIDs.indexOf(robotID);
				double maxPathLength = 1;
				for (int taskID : tasksIDs ) {
					int taskIndex = tasksIDs.indexOf(taskID);
					double pathLength = Double.POSITIVE_INFINITY;
					//Evaluate path Length
					boolean typesAreEqual = false;
					if (taskIndex < taskQueue.size() && robotindex < realRobotsIDs.size() ) {
					//typesAreEqual = taskQueue.get(taskIndex).isCompatible(coordinator.getRobot(robotID));
					typesAreEqual = taskQueue.get(taskIndex).isCompatible(getRobotType(robotID));
					
					}
					else {
						//Considering a dummy robot or  a dummy task -> same types since there are fictitious 
						typesAreEqual = true;
					}
					for(int path = 0;path < alternativePaths; path++) {
						if(typesAreEqual) { //try to find the mission only if robot and task are of same types  
					
								int missionNumber = 0;
								
								
								//if(IDsIdleRobots.contains(robotID)&& taskIndex < taskQueue.size()) {
								if(realRobotsIDs.contains(robotID) && taskIndex < taskQueue.size()) {
									
									while(missionNumber < Missions.getMissions(robotID).size() ) {
									//while(Missions.getMissions(robotID).size() != 0 ) {
										boolean finalPositionX = ArrayUtils.isEquals(Missions.getMission(robotID, missionNumber).getToPose().getX(),taskQueue.get(taskIndex).getGoalPose().getX());
										boolean finalPositionY = ArrayUtils.isEquals(Missions.getMission(robotID, missionNumber).getToPose().getY(),taskQueue.get(taskIndex).getGoalPose().getY());
										boolean initialPositionX = ArrayUtils.isEquals(Missions.getMission(robotID, missionNumber).getFromPose().getX(),coordinator.getRobotReport(robotID).getPose().getX());
										boolean initialPositionY = ArrayUtils.isEquals(Missions.getMission(robotID, missionNumber).getFromPose().getY(),coordinator.getRobotReport(robotID).getPose().getY());
										//FIXME Missing check on steering -> it may introduce some errors 
										
										
										//if( ArrayUtils.isEquals(Missions.getMission(robotID, cont).getToPose().toString(),taskQueue.get(taskIndex).getGoalPose().toString())) {
										if( initialPositionX && initialPositionY && finalPositionX && finalPositionY) {
											PoseSteering[] pss = Missions.getMission(robotID, missionNumber).getPath();
											//addPath(robotID, pss.hashCode(), pss, null, coordinator.getFootprint(robotID));
											pathsToTargetGoal.put(robotID*numTaskAug*alternativePaths+taskID*alternativePaths+path, pss);
											pathLength = Missions.getPathLength(pss);
							
										}
										missionNumber +=1;
										
										
									}
								}else {
									if (realRobotsIDs.size() >= taskQueue.size() && realRobotsIDs.contains(robotID)){
										PoseSteering[] pss = new PoseSteering[] {new PoseSteering(coordinator.getRobotReport(robotID).getPose(),0)};
										pathLength =1 ;
										pathsToTargetGoal.put(robotID*numTaskAug*alternativePaths+taskID*alternativePaths+path, pss);
										//Missions.removeMissions(Missions.getMission(robotID, cont));
										missionNumber += 1;
										
										
									}else {
										PoseSteering[] pss = new PoseSteering[] {new PoseSteering(taskQueue.get(0).getGoalPose(),0)};
										pathLength =1 ;
										pathsToTargetGoal.put(robotID*numTaskAug*alternativePaths+taskID*alternativePaths+path, pss);
									}
								}

								
								
							
							if ( pathLength > maxPathLength && pathLength != Double.POSITIVE_INFINITY) {
									maxPathLength = pathLength;
								}
						}else {
							pathsToTargetGoal.put(robotID*numTaskAug*alternativePaths+taskID*alternativePaths+path, null);		
						}
		
						PAll[robotindex][taskIndex][path] = pathLength;
					}//For path
				}//For task

				//Sum the max path length for each robot
				sumPathsLength += maxPathLength;
				//Sum the arrival time for the max path length
				sumArrivalTime += computeArrivalTime(maxPathLength,this.slowestRobotVelocity,this.slowestRobotAcceleration);
				}
			double [][][] PAllAug =  checkTargetGoals(PAll);
			//Save the sum of max paths length to normalize path length cost
			this.sumMaxPathsLength = sumPathsLength;
			//Save the sum of arrival time considering max paths length to normalize delay cost
			this.sumArrivalTime = sumArrivalTime;
			
			//Return the cost of path length	

			return PAllAug;
			}
		
		
		
		/**
		 * Evaluate the cost associated to time delay on completion of a task for a specific robot, due to interference with other robot
		 * and precedence constraints. The cost is evaluated considering the intersection between the path of robot i-th
		 * with the paths of other robots, considering the actual Assignment. Also paths related to already driving robot
		 * are considered.
		 * @param robot -> The i-th Robot
		 * @param task -> The j-th Task
		 * @param pathID -> The s-th path
		 * @return The cost associated to the delay on completion of task j for robot i due to interference with other robot
		 */
		protected double evaluatePathDelay(int robotID ,int taskID,int pathID){
			CriticalSection[][][][] cssMatrix = new CriticalSection [realRobotsIDs.size()][realTasksIDs.size()][alternativePaths][1];

			long timeInitial2 = 0;

			int robotIndex = robotsIDs.indexOf(robotID);
			int taskIndex = tasksIDs.indexOf(taskID);
			//Evaluate the delay time on completion time for the actual couple of task and ID
			//Initialize the time delay 
			double delay = 0;
			//Considering the Actual Assignment 
			if (currentAssignment[robotIndex][taskIndex][pathID]>0) {
				
				// Only for real robots and tasks
				if (realRobotsIDs.contains(robotID) && realTasksIDs.contains(taskID)) {
					//Take the Pose steering relate to i-th robot and j-th task from path set
					//PoseSteering[] pss1 = pathsToTargetGoalTotal.get((robot-1)*numTaskAug*maxNumPaths + task*maxNumPaths +pathID);
					PoseSteering[] pss1 = pathsToTargetGoal.get(robotID*numTaskAug*alternativePaths+taskID*alternativePaths+pathID);
					if(pss1 == null) {
						return delay;
					}
					
					//Initialize Array of delays for the two robots
					TreeSet<IndexedDelay> te1TCDelays = new TreeSet<IndexedDelay>() ;
					TreeSet<IndexedDelay> te2TCDelays = new TreeSet<IndexedDelay>() ;
					//Compute the spatial Envelope for the i-th Robot
					
					SpatialEnvelope se1 = TrajectoryEnvelope.createSpatialEnvelope(pss1,coordinator.getFootprint(robotID));
					//Evaluate other path depending from the Assignment Matrix
					for(int secondRobotID : realRobotsIDs) {
						int secondRobotIndex = realRobotsIDs.indexOf(secondRobotID);
						for(int secondTaskID: realTasksIDs) {
							int secondTaskIndex = realTasksIDs.indexOf(secondTaskID);
							for(int s = 0;s < alternativePaths; s++) {
								
								if (currentAssignment [secondRobotIndex][secondTaskIndex][s] > 0 && secondRobotID != robotID && secondTaskID != taskID) {
										//Take the path of this second robot from path set
										
										PoseSteering[] pss2 = pathsToTargetGoal.get(secondRobotID*numTaskAug*alternativePaths  + secondTaskID*alternativePaths+s);
										if (pss2 != null) {//is == null if robotType is different to Task type
											
											
											//Evaluate the Spatial Envelope of this second Robot
											SpatialEnvelope se2 = TrajectoryEnvelope.createSpatialEnvelope(pss2,coordinator.getFootprint(secondRobotID));
											long timeInitial = Calendar.getInstance().getTimeInMillis();
											//Compute the Critical Section between this 2 robot
											CriticalSection [] css = new CriticalSection[1];
											
											/*
											
											if(criticalSections.containsKey(robotID*numTaskAug*maxNumPaths+taskID*maxNumPaths+pathID) ) {
												css= criticalSections.get(robotID*numTaskAug*maxNumPaths+taskID*maxNumPaths+pathID)[secondRobotIndex][secondTaskIndex][s];
												if(css.length == 1) {
													css = AbstractTrajectoryEnvelopeCoordinator.getCriticalSections(se1, se2,true, Math.min(tec.getFootprintPolygon(robotID).getArea(),tec.getFootprintPolygon(secondRobotID).getArea()));
													
												}
											}else {
												css = AbstractTrajectoryEnvelopeCoordinator.getCriticalSections(se1, se2,true, Math.min(tec.getFootprintPolygon(robotID).getArea(),tec.getFootprintPolygon(secondRobotID).getArea()));
												cssMatrix[secondRobotIndex][secondTaskIndex][s] = css;
												criticalSections.put(robotID*numTaskAug*maxNumPaths+taskID*maxNumPaths+pathID, cssMatrix);
												//Evaluate the time to compute critical Section
												long timeFinal = Calendar.getInstance().getTimeInMillis();
												long timeRequired = timeFinal- timeInitial;
												timeRequiretoComputeCriticalSection = timeRequiretoComputeCriticalSection + timeRequired;
												fileStream1.println(timeRequired+"");
						
											}
											
											*/
											
											
											css = AbstractTrajectoryEnvelopeCoordinator.getCriticalSections(se1, se2,true, Math.min(coordinator.getFootprintPolygon(robotID).getArea(),coordinator.getFootprintPolygon(secondRobotID).getArea()));
											cssMatrix[secondRobotIndex][secondTaskIndex][s] = css;
											//criticalSections.put(robotID*numTaskAug*maxNumPaths+taskID*maxNumPaths+pathID, cssMatrix);
											//Evaluate the time to compute critical Section
											long timeFinal = Calendar.getInstance().getTimeInMillis();
											long timeRequired = timeFinal- timeInitial;
											timeRequiretoComputeCriticalSection = timeRequiretoComputeCriticalSection + timeRequired;
											//fileStream1.println(timeRequired+"");
											
											
											timeInitial2 = Calendar.getInstance().getTimeInMillis();
											//Compute the delay due to precedence constraint in Critical Section
											for (int g = 0; g < css.length; g++) {
												//Pair<Double, Double> a1 = estimateTimeToCompletionDelays(pss1.hashCode(),pss1,te1TCDelays,pss2.hashCode(),pss2,te2TCDelays, css[g]);
												
												metaCSPLogger.info("Evaluating delay between Robot " + robotID+ " executing Task " + taskID + " and Robot" + secondRobotID +" executing Task " + secondTaskID);
												Pair<Double, Double> a1 = estimateTimeToCompletionDelays(robotID,pss1,secondRobotID,pss2);
												double delayCriticalSection = Math.min(a1.getFirst(), a1.getSecond());
												//fileStream3.println(a1.getFirst() + " " +  a1.getSecond() + " " + robotID + " " + secondRobotID+ " ");
												if(delayCriticalSection < 0 ) {
													delay += 0;
												}else if(delayCriticalSection == Double.POSITIVE_INFINITY) {
													delay += 10000;
												}else {
													delay += delayCriticalSection;
												}
											}

									
									//Take the paths of driving robots from coordinator
									pathsDrivingRobots = coordinator.getDrivingEnvelope();
								//Evaluate the delay time due to already driving robots
									for(int k = 0; k < pathsDrivingRobots.size(); k++) {
										System.out.println("TEST DRIVING!");
										CriticalSection [] cssDrivingRobot = AbstractTrajectoryEnvelopeCoordinator.getCriticalSections(se1, pathsDrivingRobots.get(k),true, Math.min(coordinator.getFootprintPolygon(robotID).getArea(),coordinator.getFootprintPolygon(secondRobotID).getArea()));
										for (int b = 0; b < cssDrivingRobot.length; b++) {
											//Pair<Double, Double> a1 = estimateTimeToCompletionDelays(pss1.hashCode(),pss1,te1TCDelays,pathsDrivingRobots.get(k).getPath().hashCode(),pathsDrivingRobots.get(k).getPath(),te2TCDelays, cssDrivingRobot[b]);
											Pair<Double, Double> a1 = estimateTimeToCompletionDelays(robotID,pss1,secondRobotID,pss2);
											double delayCriticalSection = Math.min(a1.getFirst(), a1.getSecond());
											if(delayCriticalSection < 0 ) {
												delay += 0;
											}else if(delayCriticalSection == Double.POSITIVE_INFINITY) {
												delay += 10000;
											}else {
												delay += delayCriticalSection;
											}
										}
									}
									
									
									long timeFinal2 = Calendar.getInstance().getTimeInMillis();
									long timeRequired2 = timeFinal2- timeInitial2;
									timeRequiretoComputePathsDelay = timeRequiretoComputePathsDelay + timeRequired2;
									//fileStream2.println(timeRequired2+"");
									}
								} 
							}	
					}	
				}
					//fileStream3.println(" ");
					
				} else { //There also virtual robot and task are considered
					//the delay associated to dummy robot and task is considerd 0
					return delay;
				}
			}
			
			//return the delay for the i-th robot and the j-th task due to interference with other robots
			return delay;
			}
		@Override
		public double evaluateInterferenceCost(int robotID ,int taskID,int pathID) {
			double pathDelayCost = evaluatePathDelay(robotID,taskID,pathID)/sumArrivalTime;
			return pathDelayCost;
		}
		
		
		
		protected double computeArrivalTime(double pathLength,double vel,double acc){
			//Compute the arrival time of this path, considering a robot alone with a velocity trapezoidal model
			
			if(this.slowestRobotAcceleration == 0 && this.slowestRobotVelocity == 0) {
				computeMaxVelandAccel();
			}
			
			if(pathLength == Double.POSITIVE_INFINITY) {
				pathLength = Integer.MAX_VALUE;
			}
			double Pmax = Math.pow(vel, 2)/(2*acc);
			double arrivalTime = 1;
			if(pathLength > Pmax) {
				//Use trapezoidal profile
				arrivalTime = pathLength/vel + vel/acc;
			}else {
				arrivalTime = (2*pathLength)/vel;
			}
			//Return the arrival time 
			return arrivalTime;
		}
		
		
		/**
		 * Compute the nominal (i.e. suppose that the robot is alone in the map) arrival time (i.e. time to reach the target position of a task). 
		 * @param PAll ->  paths length Matrix
		 * @return Cost nominal arrival time matrix 
		 */
		protected double [][][] computeArrivalTime(double[][][]PAll){
			//Compute the arrival time of this path, considering a robot alone with a velocity trapezoidal model
			double [][][] arrivalTimeMatrix = new double [numRobotAug][numTaskAug][alternativePaths];
			for (int robotID : realRobotsIDs ) {
				int i = robotsIDs.indexOf(robotID);
				for (int taskID : realTasksIDs ) {
					int j = tasksIDs.indexOf(taskID);
					for(int path = 0;path < alternativePaths; path++) {
						double vel = coordinator.getRobotMaxVelocity(robotID);
						double acc = coordinator.getRobotMaxAcceleration(robotID);			 
						arrivalTimeMatrix[i][j][path] = computeArrivalTime(PAll[i][j][path],vel,acc);
					}			
				}
			}
			
			//Return the arrival time 
			return arrivalTimeMatrix;
		}
		
		/**
		 * Evaluate the tardiness in completion of a task, for all the task set . The tardiness is the defined as the further time required to complete a task
		 * after the deadline 
		 * @param PAll -> paths length Matrix
		 * @return Cost Tardiness matrix 
		 */
		
		protected double[][][] computeTardiness(double [][][]PAll) {
			double [][][] tardinessMatrix = new double [numRobotAug][numTaskAug][alternativePaths];
			for (int robotID : realRobotsIDs ) {
				int i = robotsIDs.indexOf(robotID);
				for (int taskID : realTasksIDs ) {
					int j = tasksIDs.indexOf(taskID);
						for(int path = 0;path < alternativePaths; path++) {					
							//tardinessMatrix[i][j][path] = computeTardiness(robotID,taskID,PAll[i][j][path]);
							double tardiness = 0;
							if(realTasksIDs.contains(taskID)) {
								int taskIndex = realTasksIDs.indexOf(taskID);
								if (taskQueue.get(taskIndex).isDeadlineSpecified()) { // Compute tardiness only if specified in task constructor
									double deadline = taskQueue.get(taskIndex).getDeadline();  //Expressed in seconds
									double vel = coordinator.getRobotMaxVelocity(robotID);
									double acc = coordinator.getRobotMaxAcceleration(robotID);
									double completionTime = computeArrivalTime(PAll[i][j][path],vel,acc) + taskQueue.get(taskIndex).getOperationTime();
									tardiness = Math.max(0, (completionTime-deadline));
								}	
							}
							tardinessMatrix[i][j][path] = tardiness;
							sumTardiness += tardiness;			
					}		
				}
			}
			return tardinessMatrix;
		}

		protected double [][][] evaluateBFunction(){

			double[][][] PAll = evaluatePAll();
			double [][][] tardinessMatrix = computeTardiness(PAll);
			double [][][] arrivalTimeMatrix = computeArrivalTime(PAll);
			double [][][] BFunction = new double [numRobotAug][numTaskAug][alternativePaths];
			interferenceFreeCostMatrix = new double [numRobotAug][numTaskAug][alternativePaths];
			if(linearWeight == 1) {
				for (int i = 0 ; i < numRobotAug; i++) {
					for (int j = 0 ; j < numTaskAug; j++) {
						for(int path = 0;path < alternativePaths; path++) {
							BFunction[i][j][path] = pathLengthWeight*PAll[i][j][path]/sumMaxPathsLength + tardinessWeight*tardinessMatrix[i][j][path]/sumTardiness + arrivalTimeWeight*arrivalTimeMatrix[i][j][path]/sumArrivalTime;
							interferenceFreeCostMatrix[i][j][path] = BFunction[i][j][path];
							//costValuesMatrix[i][j][path] =  PAll[i][j][path]/sumMaxPathsLength+ tardinessMatrix[i][j][path]/sumTardiness + arrivalTimeMatrix[i][j][path]/sumArrivalTime;
						}
						
					}
				}
			}
			else {
				for (int i = 0 ; i < numRobotAug; i++) {
					for (int j = 0 ; j < numTaskAug; j++) {
						for(int path = 0;path < alternativePaths; path++) {
							BFunction[i][j][path] = pathLengthWeight*PAll[i][j][path]/sumMaxPathsLength+ tardinessWeight*tardinessMatrix[i][j][path]/sumTardiness;
							//costValuesMatrix[i][j][path] = PAll[i][j][path]/sumMaxPathsLength+ tardinessMatrix[i][j][path]/sumTardiness;
							interferenceFreeCostMatrix[i][j][path] = BFunction[i][j][path] ;
						}
						

		
					}
				}
			}
			
			return BFunction;
		}

		public void checkforBlocking(){
			this.checkBlocking = true;
		}
		

		public int [][][] findOptimalAssignment(AbstractOptimizationAlgorithm optimizationSolver){
			realRobotsIDs = coordinator.getIdleRobots();
			if(checkBlocking){
				checkOnBlocking();
			}
			return super.findOptimalAssignment(optimizationSolver);
		}


		Callback cb = new Callback() {
			private long lastUpdate = Calendar.getInstance().getTimeInMillis();
			@Override
			public void performOperation() {
				long timeNow = Calendar.getInstance().getTimeInMillis();
				if (timeNow-lastUpdate > 1000) {
					lastUpdate = timeNow;
				}
			}
		};
		
		public void startTaskAssignment(AbstractOptimizationAlgorithm optimizationSolver) {
			//Create meta solver and solver
			this.optimizationSolver = optimizationSolver;
			//Start a thread that checks and enforces dependencies at every clock tick
			this.setupInferenceCallback();
		}

		protected void setupInferenceCallback() {
			
			Thread TaskAssignmentThread = new Thread("Task Assignment") {
				private long threadLastUpdate = Calendar.getInstance().getTimeInMillis();
				@Override
				
				public void run() {
					while (true) {
						System.out.println("Thread Running");
						if (!taskQueue.isEmpty() && coordinator.getIdleRobots().size() > 2) {
							int [][][] assignmentMatrix = findOptimalAssignment(optimizationSolver);
							printOptimalAssignment();
							allocateTaskstoRobots(assignmentMatrix);
							System.out.print("Task to be completed "+ taskQueue.size());
							if(taskPosponedQueue.size() !=0) {
								taskQueue.addAll(taskPosponedQueue);
								taskPosponedQueue.removeAll(taskPosponedQueue);
							}
						}
						
						//Sleep a little...
						if (CONTROL_PERIOD_TASK > 0) {
							try { 
								System.out.println("Thread Sleeping");
								Thread.sleep(CONTROL_PERIOD_TASK); } //Thread.sleep(Math.max(0, CONTROL_PERIOD-Calendar.getInstance().getTimeInMillis()+threadLastUpdate)); }
							catch (InterruptedException e) { e.printStackTrace(); }
						}

						long threadCurrentUpdate = Calendar.getInstance().getTimeInMillis();
						EFFECTIVE_CONTROL_PERIOD_TASK = (int)(threadCurrentUpdate-threadLastUpdate);
						threadLastUpdate = threadCurrentUpdate;
						
						if (cb != null) cb.performOperation();

					}
				}
			};
			TaskAssignmentThread.setPriority(Thread.MAX_PRIORITY);
			TaskAssignmentThread.start();	
		}

		/**
		 * Allocate the tasks to the robots
		 * @param AssignmentMatrix -> An (sub)optimal assignment Matrix of the actual optimization problem
		 */
		protected void allocateTaskstoRobots(int [][][] AssignmentMatrix){
			getProblemInfo();
			for (int robotID : robotsIDs ) {
				int i = robotsIDs.indexOf(robotID);
				for (int taskID : tasksIDs ) {
					int j = tasksIDs.indexOf(taskID);
					for(int s = 0; s < alternativePaths; s++) {
						if (AssignmentMatrix[i][j][s] > 0) {
							if (i < realRobotsIDs.size()) { //Considering only real Robot
								PoseSteering[] pss = pathsToTargetGoal.get(robotID*numTaskAug*alternativePaths+taskID*alternativePaths+s);
								//PoseSteering[] pss = new PoseSteering[1];
								//pss[0] = new PoseSteering(tec.getRobotReport(robotID).getPose(),0);
								//For Dispatch mission
								if (j < taskQueue.size() && pss != null) {
									//removePath(pss.hashCode());
									taskQueue.get(j).assignRobot(robotID);
									taskQueue.get(j).setPaths(pss);
									Mission[] robotMissions = taskQueue.get(j).getMissions();
									if (this.taskCB != null) this.taskCB.onTaskAssignment(i,j,s,robotMissions);							  
									viz.displayTask(taskQueue.get(j).getStartPose(), taskQueue.get(j).getGoalPose(),taskID, "red");
									//tec.addMissions(new Mission(IDsIdleRobots[i],pss));
									metaCSPLogger.info("Task # "+ taskID + " is assigned");
									metaCSPLogger.info("Robot " + robotID +" is assigned to Task "+ taskID +" through Path " + (s+1));
									//tec.setTaskAssigned(robotID,taskID);
									coordinator.addMissions(robotMissions);
								}else {
									metaCSPLogger.info("Virtual Task # "+ taskID + " is assigned to a real robot");
								}
							}else{
								metaCSPLogger.info("Task # "+ taskID + " is not assigned to a real robot");
								
							}
						}
						//Remove path from the path set
						pathsToTargetGoal.remove(robotID*numTaskAug*alternativePaths+taskID*alternativePaths+s);
					}
					
				}
			}
			
			//Remove Assigned Tasks from the set	
			int i = 0;
			int cont = 0;
			while (i < Math.min(realRobotsIDs.size(), taskQueue.size())) {
				if (taskQueue.size() == 0 || taskQueue.size() <= i) {
					break;
				}
				if (taskQueue.get(i).isTaskAssigned()){
					taskQueue.remove(i);
					metaCSPLogger.info("Task # "+ (cont+1) + " is removed from Task set");
				}else {
					i = i+1;
				}
				cont +=1;	
				
			}
			metaCSPLogger.info("Remaining tasks: "+ taskQueue.size());
			robotsIDs.removeAll(robotsIDs);
			tasksIDs.removeAll(tasksIDs);
			realTasksIDs.removeAll(realTasksIDs);
			realRobotsIDs.removeAll(realRobotsIDs);
			optimalAssignment = null;
			
		}//End Task Assignment Function
	} //End class

