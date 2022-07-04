package se.oru.assignment.assignment_oru;

import java.util.Calendar;

import org.apache.commons.lang.ArrayUtils;
import org.metacsp.multi.spatioTemporal.paths.PoseSteering;

import com.google.ortools.linearsolver.MPConstraint;
import com.google.ortools.linearsolver.MPObjective;
import com.google.ortools.linearsolver.MPSolver;
import com.google.ortools.linearsolver.MPVariable;
import com.vividsolutions.jts.geom.Coordinate;
import com.vividsolutions.jts.geom.Geometry;

import se.oru.assignment.assignment_oru.methods.AbstractOptimizationAlgorithm;
import se.oru.coordination.coordination_oru.Mission;
import se.oru.coordination.coordination_oru.util.Missions;

public class SimpleOptimizationProblem extends AbstractOptimizationProblem {
	
	//Parameters of weights in Optimization Problem for Interference free-cost function
	protected double pathLengthWeight = 1;
	protected double arrivalTimeWeight = 0;
	protected double tardinessWeight = 0;
	
	//Normalizing factors
	protected double sumTardiness = 1;
	protected double sumMaxPathsLength = 1; //This normalizing factor is obtained by summing the longest paths for each idle robot in the fleet
	protected double sumArrivalTime = 1; //This normalizing factor is the sum of arrival time of completing the longest path  for each robot
			
	protected double slowestRobotVelocity;
	protected double slowestRobotAcceleration;
	
	protected int numAllocation = 1;
	
//	//Parameters for time analysis
//	protected long timeRequiretoEvaluatePaths;
//	protected long timeRequiretofillInPall;
//	protected long timeRequiretoComputeCriticalSection;
//	protected long timeRequiretoComputePathsDelay;
//	protected long initialTime;

	@Override
	public double evaluateMachineProductivity(int robotID, int taskID, int pathID, double pathDelay) {
		// TODO Auto-generated method stub
		return 0;
	}

	@Override
	public double evaluateInterferenceCost(int robotID, int taskID, int pathID, double[][][] assignmentMatrix) {
		// TODO Auto-generated method stub
		return 0;
	}
	
	/**
	 * Add a path to the fleetmaster interface
	 * @param robotID -> The ID of the robot
	 * @param pathID -> the ID of the path
	 * @param pss -> the path expressed as a PoseSteering vector
	 * @param boundingBox -> the bounding box of the path
	 * @param coordinates -> footprint of the robot 
	 */
	protected void addPath(int robotID, int pathID, PoseSteering[] pss, Geometry boundingBox, Coordinate... coordinates) {
		// TODO Add path
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
				 if (taskIndex < numberTasks && robotindex < numberRobots ) {
				 //typesAreEqual = taskQueue.get(taskIndex).isCompatible(coordinator.getRobot(robotID));
				 typesAreEqual = taskQueue.get(taskIndex).isCompatible(getRobotTypes(robotID));
				 
				 }
				 else {
					 //Considering a dummy robot or  a dummy task -> same types since there are fictitious 
					 typesAreEqual = true;
				 }
				 for(int path = 0;path < alternativePaths; path++) {
					 if(typesAreEqual) { //try to find the mission only if robot and task are of same types  
				
						 	int missionNumber = 0;
						 	
						 	
						 	//if(IDsIdleRobots.contains(robotID)&& taskIndex < taskQueue.size()) {
						 	if(IDsIdleRobots.contains(robotID) && taskIndex < taskQueue.size()) {
						 		
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
							 			addPath(robotID, pss.hashCode(), pss, null, coordinator.getFootprint(robotID));
							 			pathsToTargetGoal.put(robotID*numTaskAug*alternativePaths+taskID*alternativePaths+path, pss);
										pathLength = Missions.getPathLength(pss);
						
							 		 }
						 			missionNumber +=1;
							 		
							 		
							 	}
						 	}else {
						 		if (numberRobots >= numberTasks && IDsIdleRobots.contains(robotID)){
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
	 * Compute the nominal (i.e. suppose that the robot is alone in the map) arrival time (i.e. time to reach the target position of a task). 
	 * @param PAll ->  paths length Matrix
	 * @return Cost nominal arrival time matrix 
	 */
	protected double [][][] computeArrivalTime(double[][][]PAll){
		//Compute the arrival time of this path, considering a robot alone with a velocity trapezoidal model
		double [][][] arrivalTimeMatrix = new double [numRobotAug][numTaskAug][alternativePaths];
		for (int robotID : IDsIdleRobots ) {
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
				 if (IDsIdleRobots.contains(robotID) && realTasksIDs.contains(taskID) ) {
				 //typesAreEqual = taskQueue.get(taskIndex).isCompatible(coordinator.getRobot(robotID)); getRobotTypes
				 typesAreEqual = taskQueue.get(taskIndex).isCompatible(getRobotTypes(robotID));
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
			
			for (int robotID : IDsIdleRobots) {
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
				 if (taskIndex < numberTasks && robotindex < numberRobots ) {
				 //typesAreEqual = taskQueue.get(taskIndex).isCompatible(coordinator.getRobot(robotID));
				 typesAreEqual = taskQueue.get(taskIndex).isCompatible(getRobotTypes(robotID));
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
		for (int robotID : IDsIdleRobots) {
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
	
	/**
	 * Evaluate the tardiness in completion of a task, for all the task set . The tardiness is the defined as the further time required to complete a task
	 * after the deadline 
	 * @param PAll -> paths length Matrix
	 * @return Cost Tardiness matrix 
	 */
	
	protected double[][][] computeTardiness(double [][][]PAll) {
		double [][][] tardinessMatrix = new double [numRobotAug][numTaskAug][alternativePaths];
		for (int robotID : IDsIdleRobots ) {
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

	/**
	 * Evaluate the overall B function, that is the function that consider interference free costs
	 * Costs considered:
	 * 1) Path Length
	 * 2) Tardiness
	 * Each cost is already normalized;
	 * @return The interference free cost matrix
	 */
	protected double [][][] evaluateBFunction(){
		double[][][] PAll = evaluatePAll();
		double [][][] tardinessMatrix = computeTardiness(PAll);
		double [][][] BFunction = new double [numRobotAug][numTaskAug][alternativePaths];
		interferenceFreeCostMatrix = new double [numRobotAug][numTaskAug][alternativePaths];
		if(linearWeight == 1) {
			double [][][] arrivalTimeMatrix = computeArrivalTime(PAll);
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
	

	/**
	 * Build the optimization problem. Define a decision variable X_ijp_{ijs} as a binary variable in which i indicate
	 * the robot id, j the tasks, and p_{ijs} is the s-th path for the robot i-th to reach the target position of task j. The problem is build as an ST-ST-IA problem, i.e.:
	 * the constraints considered are :
	 * 1) Each Task can be assign only to a robot;
	 * 2) Each Robot can perform only a task at time;
	 * @return A constrained optimization problem without the objective function
	 */
	protected MPSolver buildOptimizationProblem() {
		//Initialize a linear solver 
		
		
		MPSolver optimizationProblem = new MPSolver(
				"TaskAssignment", MPSolver.OptimizationProblemType.CBC_MIXED_INTEGER_PROGRAMMING);
		
		//MPSolver optimizationProblem = new MPSolver(
				//"TaskAssignment", MPSolver.OptimizationProblemType.CBC_MIXED_INTEGER_PROGRAMMING);
		//START DECISION VARIABLE VARIABLE
		MPVariable [][][] decisionVariable = new MPVariable[numRobotAug][numTaskAug][alternativePaths];
		for (int i = 0; i < numRobotAug; i++) {
			 for (int j = 0; j < numTaskAug; j++) {
				 for(int s = 0; s < alternativePaths; s++) {
					 decisionVariable[i][j][s] = optimizationProblem.makeBoolVar("x"+"["+i+","+j+","+s+"]");
					
				 }
				
			 }
		}
		//END DECISION VARIABLE
		//////////////////////////
		// START CONSTRAINTS
		//Each Robot can be assign only to a Task	    
		 for (int i = 0; i < numRobotAug; i++) {
			 //Initialize the constraint
			 //MPConstraint c0 = optimizationProblem.makeConstraint(-Double.POSITIVE_INFINITY, 1);
			 MPConstraint c0 = optimizationProblem.makeConstraint(1, 1);
			 for (int j = 0; j < numTaskAug; j++) {
				 for(int s = 0; s < alternativePaths; s++) {
					 //Build the constraint
					 c0.setCoefficient(decisionVariable[i][j][s], 1); 
				 }
				
			 }
		 }
		
		//Each task can be performed only by a robot
		 for (int j = 0; j < numTaskAug; j++) {
			//Initialize the constraint
			 MPConstraint c0 = optimizationProblem.makeConstraint(1, 1); 
			 for (int i = 0; i < numRobotAug; i++) {
				 for(int s = 0; s < alternativePaths; s++) {
					 //Build the constraint
					 c0.setCoefficient(decisionVariable[i][j][s], 1); 
				 } 		
			 }
		 }
	
		 for (int robotID : robotsIDs ) {
				int i = robotsIDs.indexOf(robotID);
				for (int taskID : tasksIDs ) {
					int j = tasksIDs.indexOf(taskID);
					for(int s = 0; s < alternativePaths; s++) {
							 if (i < numberRobots) { //Considering only real Robot
								 PoseSteering[] pss = pathsToTargetGoal.get(robotID*numTaskAug*alternativePaths+taskID*alternativePaths+s);
								 if(pss==null) {
									 MPConstraint c3 = optimizationProblem.makeConstraint(0,0);
									 c3.setCoefficient(decisionVariable[i][j][s],1); 
								 }
							 }
					}
				}
		 }
		 
		//END CONSTRAINTS
		//In case of having more task than robots, the task with a closest deadline are set with a higher priority
		 if(taskQueue.size() > IDsIdleRobots.size()) {
			 sortTaskByDeadline();
			//Each task can be performed only by a robot
			 for (int j = 0; j < taskQueue.size(); j++) {
				//Initialize the constraint
				 if(taskQueue.get(j).isPriority()) {
					 MPConstraint c3 = optimizationProblem.makeConstraint(1, 1); 
					 for (int i = 0; i < IDsIdleRobots.size(); i++) {
						 for(int s = 0; s < alternativePaths; s++) {
							 //Build the constraint
							 c3.setCoefficient(decisionVariable[i][j][s], 1); 
						 } 		
					 }
				 }
			 }
		 }
		/////////////////////////////////////////////////
		return optimizationProblem;	
	}
	
	
//	/**
//	 * Create the optimization problem with part of the Objective Function (Function B). Define a decision variable X_ijp_{ijs} as a binary variable in which i indicate
//	 * the robot id, j the tasks, and p_{ijs} is the s-th path for the robot i-th to reach the target position of task j. The problem is build as an ST-ST-IA problem, i.e.:
//	 * 1) Each Task can be assign only to a robot;
//	 * 2) Each Robot can perform only a task at time.
//	 * The objective function is defined as alpha*B + (1-alpha)*F 
//	 * where B includes the cost related to single robot ({@link #evaluateBFunction}), while F ({@link #evaluateInterferenceCost}) includes all costs related to interference
//	 * @return A constrained optimization problem
//	 */
//	protected MPSolver createOptimizationProblem() {
//		this.initialTime = 	Calendar.getInstance().getTimeInMillis();
//		//Perform a check in order to avoid blocking
//		//checkOnBlocking(tec);
//		//Take the number of tasks
//		numberTasks = taskQueue.size();
//		//Get free robots and their IDs
//		numberRobots = coordinator.getIdleRobots().size();
//		IDsIdleRobots = coordinator.getIdleRobots();
//		//Evaluate dummy robot and dummy task
//		checkOnBlocking();
//		dummyRobotorTask();
//		initializeRobotsIDs();
//		initializeTasksIDs();
//		double[][][] BFunction = evaluateBFunction();
//		
//		
//		
//		//Build the optimization problem
//		MPSolver optimizationProblem = buildOptimizationProblem();
//		
//		
//		//Modify the coefficients of the objective function
//		MPVariable [][][] decisionVariable = tranformArray(optimizationProblem); 
//	    /////////////////////////////////
//	    //START OBJECTIVE FUNCTION		
//	    MPObjective objective = optimizationProblem.objective();
//    	 for (int i = 0; i < numRobotAug; i++) {
//			 for (int j = 0; j < numTaskAug; j++) {
//				 for(int s = 0; s < alternativePaths; s++) {
//					 double singleRobotCost  =  BFunction[i][j][s];
//					 if ( singleRobotCost != Double.POSITIVE_INFINITY) {
//						 //Set the coefficient of the objective function with the normalized path length
//						 objective.setCoefficient(decisionVariable[i][j][s], singleRobotCost); 
//					 }else { // if the path does not exists or the robot type is different from the task type 
//						//the path to reach the task not exists
//						//the decision variable is set to 0 -> this allocation is not valid
//						MPConstraint c3 = optimizationProblem.makeConstraint(0,0);
//						c3.setCoefficient(decisionVariable[i][j][s],1); 
//					 }
//				 }
//			 }			 
//		 }
//		//Define the problem as a minimization problem
//		objective.setMinimization();
//		//END OBJECTIVE FUNCTION
//		return optimizationProblem;	
//	}
	
//	public void startTaskAssignment(AbstractOptimizationAlgorithm optimizationSolver ) {
//		//Create meta solver and solver
//		this.optimizationSolver = optimizationSolver;
//		numberRobots = coordinator.getIdleRobots().size();
//		//Start a thread that checks and enforces dependencies at every clock tick
//		this.setupInferenceCallback();
//
//	}
	
//	protected void setupInferenceCallback() {
//		
//		Thread TaskAssignmentThread = new Thread("Task Assignment") {
//			private long threadLastUpdate = Calendar.getInstance().getTimeInMillis();
//			@Override
//			
//			public void run() {
//				while (true) {
//					System.out.println("Thread Running");
//					if (!taskQueue.isEmpty() && coordinator.getIdleRobots().size() > 2) {
//						optimizationModel = createOptimizationProblem();
//						double [][][] assignmentMatrix = findOptimalAssignment(optimizationSolver);
//						for (int i = 0; i < assignmentMatrix.length; i++) {
//							int robotID = robotsIDs.get((i));
//							for (int j = 0; j < assignmentMatrix[0].length; j++) {
//								int taskID = tasksIDs.get((j));
//								for(int s = 0; s < alternativePaths; s++) {
//									System.out.println("x"+"["+(i+1)+","+(j+1)+","+(s+1)+"]"+" is "+ assignmentMatrix[i][j][s]);
//									if (assignmentMatrix[i][j][s] == 1) {
//										System.out.println("Robot " + robotID +" is assigned to Task "+ taskID +" through Path " + (s+1));
//									}
//								}
//									
//							} 
//						}
//						allocateTaskstoRobots(assignmentMatrix);
//						System.out.print("Task to be completed "+ taskQueue.size());
//						optimizationModel.clear();
//						if(taskPosponedQueue.size() !=0) {
//							taskQueue.addAll(taskPosponedQueue);
//							taskPosponedQueue.removeAll(taskPosponedQueue);
//						}
//					}
//					
//					//Sleep a little...
//					if (CONTROL_PERIOD_TASK > 0) {
//						try { 
//							System.out.println("Thread Sleeping");
//							Thread.sleep(CONTROL_PERIOD_TASK); } //Thread.sleep(Math.max(0, CONTROL_PERIOD-Calendar.getInstance().getTimeInMillis()+threadLastUpdate)); }
//						catch (InterruptedException e) { e.printStackTrace(); }
//					}
//
//					long threadCurrentUpdate = Calendar.getInstance().getTimeInMillis();
//					EFFECTIVE_CONTROL_PERIOD_TASK = (int)(threadCurrentUpdate-threadLastUpdate);
//					threadLastUpdate = threadCurrentUpdate;
//					
//					if (cb != null) cb.performOperation();
//
//				}
//			}
//		};
//		TaskAssignmentThread.setPriority(Thread.MAX_PRIORITY);
//		TaskAssignmentThread.start();
//		
//	}

}
