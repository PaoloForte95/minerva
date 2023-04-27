package se.oru.assignment.assignment_oru;

import java.io.BufferedWriter;
import java.io.FileWriter;
import java.io.IOException;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.Calendar;
import java.util.Comparator;
import java.util.List;
import java.util.logging.Logger;

import se.oru.assignment.assignment_oru.methods.AbstractOptimizationAlgorithm;
import se.oru.assignment.assignment_oru.util.TaskFleetVisualization;
import se.oru.assignment.assignment_oru.util.robotType.ROBOT_TYPE;
import se.oru.assignment.assignment_oru.util.StringUtils;


/**
 *  Class to create an online Single Task, Single Robot, Instantaneous Assignment (ST–SR–IA) Multi-Robot Task Assignment(MRTA) problem. The problem is posed as an online Optimal Assignment Problem (OAP).
 * @author pofe
 *
 */
public abstract class AbstractOptimizationProblem{
	
		/**
		 * Sort the set of tasks task by deadlines
		 *
		 */
		class SortByDeadline implements Comparator<Task>{
			@Override
			public int compare(Task task1, Task task2) {
				return (int) (task1.getDeadline()-task2.getDeadline());
			}
		}
	
		/**
		 * Sort the set of tasks by deadlines. Tasks with the closest deadline are prioritized. The number of prioritized tasks depends on the number of available robots.
		*/
		protected void sortTaskByDeadline() {
			ArrayList <Task> sortedtaskArray = new ArrayList <Task>();
			for(int j=0; j < taskQueue.size(); j++ ) {
				sortedtaskArray.add(taskQueue.get(j));
			}
			sortedtaskArray.sort(new SortByDeadline());
			for(int i=0;i < realRobotsIDs.size(); i++) {
				int index = taskQueue.indexOf(sortedtaskArray.get(i));
				if(taskQueue.get(index).getDeadline() != -1) {
					taskQueue.get(index).setPriority(true);
				}
				
			}
		}
	
		public static String TITLE = "assignment_oru - Robot-agnostic online task assignment for multiple robots";
		public static String COPYRIGHT = "Copyright \u00a9 2020-" + Calendar.getInstance().get(Calendar.YEAR) + " Paolo Forte";
		public static String[] CONTRIBUTORS = {"Paolo Forte", "Anna Mannucci", "Federico Pecora"};
	
		//null -> public (GPL3) license
		public static String LICENSE = null;
	
		public static String PUBLIC_LICENSE = "This program comes with ABSOLUTELY NO WARRANTY. "
				+ "This program is free software: you can redistribute it and/or modify it under the "
				+ "terms of the GNU General Public License as published by the Free Software Foundation, "
				+ "either version 3 of the License, or (at your option) any later version. see LICENSE for details.";
		public static String PRIVATE_LICENSE = "This program comes with ABSOLUTELY NO WARRANTY. "
				+ "This program has been licensed to " + LICENSE + ". The licensee may "
				+ "redistribute it under certain conditions; see LICENSE for details.";
	
		//Force printing of (c) and license upon class loading
		static { printLicense(); }

		//Fixed Value
		protected int virtualRobotID = Integer.MAX_VALUE; 
		protected int virtualTaskID = Integer.MAX_VALUE;
	
		//Optimization Problem Parameters
		//protected int numberRobots;
		//protected int numberTasks;
		protected int numRobotAug;
		protected int numTaskAug;
		protected int alternativePaths;
		protected int dummyRobot;
		protected int dummyTask;
		protected ArrayList <Robot> robots;
		protected ArrayList <Task> taskQueue;
		protected ArrayList <Task> taskPosponedQueue;	
		protected ArrayList <Integer> realRobotsIDs;  //Set of real robots
		protected ArrayList <Integer> realTasksIDs; //Set of real tasks
		protected ArrayList <Integer> robotsIDs; //Set of all robots (both real and dummy)
		protected ArrayList <Integer> tasksIDs; //Set of all tasks (both real and dummy)

		protected double linearWeight;
		protected double [][][] interferenceFreeCostMatrix;
		protected double [][][] interferenceCostMatrix;

		//Solutions
		protected int [][][] optimalAssignment;
		protected List <int [][][]> feasibleSolutions;
		
		//ROADMAP Parameters
		protected String scenario;
		protected boolean saveFutureAllocations = false;
		
		//Logger 
		protected Logger metaCSPLogger;	

		//Solver
		protected AbstractOptimizationAlgorithm optimizationSolver;
		
		//Visualization parameters
		protected TaskFleetVisualization viz = null;

		//Callbacks
		protected TaskAssignmentCallback taskCB = null;
		protected ComputePathCallback pathCB = null;

		/**
		 * Get the assignment matrix for the current solution
		 * @return
		 */
		public abstract int[][][] getAssignmentMatrix();

		/**
		 * Get the robot type of the specific robot.
		 * @param robotID -> the ID of the robot
		 * @return The robot type of the specific robot.
		 */
		
		public ROBOT_TYPE getRobotType(int robotID){
			for(Robot rb : robots){
				if(rb.getRobotID() == robotID){
					return robots.get(robots.indexOf(rb)).getType();
				}
			}
			metaCSPLogger.severe("I know nothing about the robot" + robotID + ". Please check if the ID is correct");
			throw new Error("Cannot get the type of robot" + robotID);
		}
			
		/**
		 * Enable the saving of the scenarios (Missions + paths) for all the optimization problems that will be solved.
		 * 
		 */
		public void saveAllScenarios() {
			this.saveFutureAllocations = true;
			
		}

		/**
		 * Set the maximum alternative number of paths to reach a goal for each robot.
		 * @param alternativePaths -> number of path to reach a goal
		 */
		public void setmaxNumberOfAlternativePaths(int alternativePaths) {
			this.alternativePaths = alternativePaths;
		}
	
		
		/** 
		 * Get the maximum alternative number of paths to reach a goal for each robot.
		 */
		public int getmaxNumberOfAlternativePaths() {
			return this.alternativePaths;
		}
		
		/**
		 * Get the interference free cost matrix associated to the problem (B function)
		 * @return
		 */
		public double [][][] getInterferenceFreeCostMatrix() {
			return this.interferenceFreeCostMatrix;
		}

		/**
		 * Set the matrix of interference free costs (B Function). 
		 * @param costs
		 */
		public void setInterferenceFreeCostMatrix(double [][][] costs){
			interferenceFreeCostMatrix = Arrays.stream(costs).map(double[][]::clone).toArray(double[][][]::new);
		}

		/**
		 * Get the interference free cost matrix associated to the problem (F function)
		 * @return
		 */
		public double [][][] getInterferenceCostMatrix() {
			return this.interferenceCostMatrix;
		}

		/**
		 * Set the matrix of interference free costs (F Function). 
		 * @param costs
		 */
		protected void setInterferenceCostMatrix(double [][][] costs){
			interferenceCostMatrix = Arrays.stream(costs).map(double[][]::clone).toArray(double[][][]::new);
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
		 * Load a Scenario (paths + missions) 
		 * @param scenario ->  Scenario to load
		 */
		
		public void loadScenario(String scenario) {
			this.scenario = scenario;
		}
		
		/**
		 * Load an assignment.
		 * @param assignment ->  Assignment to load 
		 */
		
		public void loadAssignment(int [][][] assignment) {
			this.optimalAssignment = assignment;
		}

		/**
		 * Set the linear weight used in Optimization Problem. This parameter sets the weight (i.e. the importance) of B and F. If alpha = 1 (default value) only the B function is minimized.
		 * linearWeight must be > 0 and < 1. 
		 * More this value is close to 0, more is the importance given to F Function. More this value is close to 1, more is the importance given to B function.
		 * @param alpha -> the linear weight value.
		 */
		
		public void setLinearWeight(double alpha) {
			if(alpha < 0 || alpha > 1) {
				throw new Error("The linear weigth must be > 0 and < 1!");
			}
			metaCSPLogger.info("alpha is set to : " + alpha);
			this.linearWeight = alpha;
		}	
		
		/**
		 * Get the linear weight used in Optimization Problem.This parameter sets the weight of B and F. If alpha = 1 (default value) only the B function is minimized.
		 * @return the value of the linear weight alpha
		 */
		
		public double getLinearWeight() {
			return this.linearWeight;
		}	
		
		
	/**
	 * Set the Fleet Visualization.
	 * @param viz -> An instance of a TaskFleetVisualization
	 */
			
	public void setFleetVisualization(TaskFleetVisualization viz) {
		this.viz = viz;
	}

	/**
	 * Get the IDs of all the robots considered into the problem (both real and virtual)
	 * @return a set of all the robots IDs
	 */
	
	 public ArrayList <Integer> getRobotsIDs() {
		return this.robotsIDs;
	}
	
	/**
	 * Get the IDs of all the idle robots considered into the problem
	 * @return a set of all the robots IDs
	 */
	
	public ArrayList <Integer> getRealRobotsIDs() {
		return this.realRobotsIDs;
	}

	/**
	 * Get the IDs of all the tasks considered into the problem (both real and virtual)
	*/
	
	public ArrayList <Integer> getTasksIDs() {
		return this.tasksIDs;
	}
	
	/**
	 	Get the compute optimal assignment for this optimization problem once it is computed by the {@link #findOptimalAssignment} method. 
	 	* @return Optimal Assignment Matrix
	 */
	public int[][][] getOptimalAssignment(){
		if (this.optimalAssignment != null){
			return this.optimalAssignment;
		}
		throw new Error("Cannot predict the future. Before getting the optimal assignment compute it!");
	}

	/**
	 * Get the current set of tasks.
	 * @return The current set of tasks.
	 */
	public ArrayList<Task> getTaskQueue() {
		return this.taskQueue;
	}

	/**
	 * Initialize the IDs of all the robots considered into the problem (both real and virtual)
	*/
	
	protected void initializeRobotsIDs() {
		int virtualRobotID = this.virtualRobotID;
		for(int i= 0; i < numRobotAug; i++) {
			if(i < realRobotsIDs.size()) {
				robotsIDs.add(realRobotsIDs.get(i));	
			}else {
				robotsIDs.add(virtualRobotID);
				virtualRobotID = virtualRobotID-1;
				this.virtualRobotID -=1;
			}	
		}
	}

	/**
	 * Initialize the IDs of all the tasks considered into the problem (both real and virtual)
	 */
	
	protected void initializeTasksIDs() {
		int virtaulTaskID = this.virtualTaskID;
		for(int i= 0; i < numTaskAug; i++) {
			if(i < taskQueue.size()) {
				realTasksIDs.add(taskQueue.get(i).getID());
				tasksIDs.add(taskQueue.get(i).getID());
			}else {
				tasksIDs.add(virtaulTaskID);
				virtaulTaskID = virtaulTaskID-1;
				this.virtualTaskID -=1;
			}	
		}
	}
	
	/**
	 * Add a robot to robots set
	 * @param task -> the robot to add
	 * @return -> True if robot is added correctly, otherwise false
	 */
	public boolean addRobot(Robot robot) {
		if (robot == null) {
			metaCSPLogger.severe("No robot to add. Please give a correct robot.");
			throw new Error("Cannot add the robot");
		}
		metaCSPLogger.info(robot.toString() +  " has been added" );
		return robots.add(robot);
	}
	


	/**
	 * Add a Task to tasks set
	 * @param task -> the task to add
	 * @return -> true if task is added correctly, otherwise false
	 */
	public boolean addTask(Task task) {
		if (task == null) {
			metaCSPLogger.severe("No task to add. Please give a correct Task.");
			throw new Error("Cannot add the task");
		}
		metaCSPLogger.info(task.toString() +  " has been added" );
		return taskQueue.add(task);
	}
	
	/**
	 * Compute the number of Dummy Robots and/or Tasks to add to the problem. Consider the possibility to have a different number of robots (N) and tasks (M). If N > M, dummy tasks are 
	 * considered, where a dummy task is a task for which a robot stay in starting position; while if M > N dummy robots are considered, where a dummy robot is only a virtual robot. 
	 */
	protected void dummyRobotorTask() {

		int numberRobots = realRobotsIDs.size();
		int numberTasks = taskQueue.size();

		numRobotAug = realRobotsIDs.size();
		numTaskAug = numberTasks;
		//Restore initial value for dummy robot and task
		dummyTask = 0;
		dummyRobot = 0;
		//Considering the possibility to have n != m
		//First check is related to numbers of real robots and tasks associated to the problem
		//If n > m -> we have dummy robot
		if (numberRobots > numberTasks) {
			dummyTask = numberRobots - numberTasks;
			numTaskAug = numberTasks + dummyTask;
		}
		//If n < m -> we have dummy tasks
		else if (numberRobots < numberTasks) {
			dummyRobot = numberTasks - numberRobots;
			numRobotAug = numberRobots + dummyRobot;
		}
		//A second check is : check if a robot cannot perform at least one task of the set
		//A dummy robot and task will be added for each robot in this case 
		if (dummyTask == 0 || dummyRobot != 0) {
			for (int i = 0; i < numberRobots; i++) {
				boolean canExecuteATask = false;
				 for (int j = 0; j < numberTasks; j++) {
					 //check if robot can be assigned to one task
					 //if (taskQueue.get(j).getTaskType() == tec.getRobotType(IDsIdleRobots[i])) {getRobotTypes
					 //if (taskQueue.get(j).isCompatible(tec.getRobot(IDsIdleRobots.get(i)))) {
					 if (taskQueue.get(j).isCompatible(getRobotType(realRobotsIDs.get(i)))) {
						 canExecuteATask = true;
						 
					 }
				 }
				 //the robot cannot be assigned to any task -> add a dummy robot and task
				 if (!canExecuteATask) {
					 dummyRobot += 1 ;
					 dummyTask += 1 ;
					 numRobotAug += 1;
					 numTaskAug += 1;
				 }
			}
		}
		//If A task cannot be assigned to any robot
		if (dummyRobot == 0 || dummyTask != 0) {
			for (int i = 0; i < numberTasks; i++) {
				boolean flagAllocateTask = false;
				 for (int j = 0; j < numberRobots; j++) {
					//check if task can be assigned to one robot
					 //if (taskQueue.get(i).getTaskType() == tec.getRobotType(IDsIdleRobots[j])) {
					 //if (taskQueue.get(i).isCompatible(tec.getRobot(IDsIdleRobots.get(j)))) {
					if (taskQueue.get(i).isCompatible(getRobotType(realRobotsIDs.get(j)))) {	 
						 flagAllocateTask = true;
					 }
				 }
				 //the task cannot be assigned to any robot -> add a dummy robot and task
				 if (!flagAllocateTask) {
					 dummyRobot += 1 ;
					 dummyTask += 1 ;
					 numRobotAug += 1;
					 numTaskAug += 1;
				 }
			}
		}
	}

	/**
	 * Evaluate the overall B function, that is the function that consider interference free costs
	 * Costs considered:
	 * 1) Path Length
	 * 2) Tardiness
	 * Each cost is already normalized;
	 * @return The interference free cost matrix
	*/
	protected abstract double [][][] evaluateBFunction();

	/**
	 * Get the interference free cost associated to the single robot.
	 * @param robotID -> ID of the robot
	 * @param taskID -> ID of the task
	 * @param path -> index of the path
	 * @return
	 */

	public double getRobotSingleCost(int robotID, int taskID, int path){
		int i = robotsIDs.indexOf(robotID);
		int j = tasksIDs.indexOf(taskID);
		return this.interferenceFreeCostMatrix[i][j][path];
		
	}
	
	/**
	 * Evaluate the interference cost for the specific robot (robotID) to perform the  specific task (taskID) following the specific path (pathID)
	 * @param robotID -> ID of the robot that perform the task
	 * @param taskID -> ID of the task that the robot will perform
	 * @param pathID -> ID of the path that the robot will follow
	 * @param assignmentMatrix -> Assignment Matrix (necessary to compute interference with other robots)
	 */
	
	public abstract double evaluateInterferenceCost(int robotID ,int taskID,int pathID,int [][][] assignmentMatrix);
	
	/** 
	 * Find the optimal assignment using the selected algorithm. 
	 * @param optimizationSolver -> An instance of a {@link AbstractOptimizationAlgorithm}.
	 * @return The optimal assignment
	 */
	public abstract int [][][] findOptimalAssignment(AbstractOptimizationAlgorithm optimizationSolver);


	/**
	 * Get the number of feasible solution for this optimization problem.
	 * @return The number of feasible solution 
	 */
	
	 public int numberOfFeasibleSolutions(){	
		int solutions = 0;
		int i,fact = 1;  
		int number = Math.max(numRobotAug, numTaskAug);  
		for(i=1;i <= number;i++){    
			fact=fact*i;    
		}    
		solutions = fact*alternativePaths;
	    return solutions;
	}
	
	/**
	 * Get the information for this optimization problem.
	*/
	
	protected void getProblemInfo() {
		metaCSPLogger.info("Number of Robot : " + realRobotsIDs.size());
		metaCSPLogger.info("Number of Task : " + taskQueue.size());
		metaCSPLogger.info("Number of dummy Robot : " + dummyRobot);
		metaCSPLogger.info("Number of dummy Task : " + dummyTask);
		metaCSPLogger.info("Total Number of Robot : " + robotsIDs.size());
		metaCSPLogger.info("Total Number of Task : " + tasksIDs.size());
		metaCSPLogger.info("Number of feasible solutions: " + numberOfFeasibleSolutions());
		metaCSPLogger.info("Linear Weight is set:" + getLinearWeight());
	}

	/**
	 * Print the assignment given as input
	 * @param assignmentMatrix
	 */
	 private void printAssignment(int [][][] assignmentMatrix){
		for (int i = 0; i < assignmentMatrix.length; i++) {
			int robotID = robotsIDs.get((i));
			for (int j = 0; j < assignmentMatrix[0].length; j++) {
				int taskID = tasksIDs.get((j));
				for(int s = 0; s < alternativePaths; s++) {
					//metaCSPLogger.info("Decision variable x"+"["+(i+1)+","+(j+1)+","+(s+1)+"]"+" is: "+ assignmentMatrix[i][j][s]);
					if (assignmentMatrix[i][j][s] == 1) {
						metaCSPLogger.info("Robot " + robotID +" is assigned to Task "+ taskID +" through Path " + (s+1));
					}
				}
					
			} 
		}
	}

	/**
	 * Print the optimal assignment 
	 */
	public void printOptimalAssignment(){
		if(optimalAssignment!= null){
			printAssignment(optimalAssignment);
			return ;
		}
		throw new Error("Cannot predict the future. Before printing the optimal assignment, compute it!");
	}


	public void printFeasibleAssignments(){
		if(feasibleSolutions == null){
			throw new Error("Cannot predict the future. Before printing the feasible solutions, compute them!");
		}
		metaCSPLogger.info("Found " + feasibleSolutions.size() + " possible assignment");
		for(int[][][] solution: feasibleSolutions){
			printAssignment(solution);
			metaCSPLogger.info("--------");
		}
	}

	/**
		 * Save the assignment Matrix in the specific file. If the file does not exist, it will be created.
		 * @param filename -> name of the file where to save the matrix
		 * @param optimalAssignmentMatrix -> the task assignment matrix
	*/
	public static void saveAssignmentMatrixinFile(String filename, int[][][] optimalAssignmentMatrix, boolean append) {
		    try {
		        BufferedWriter bw = new BufferedWriter(new FileWriter((filename),append));
		        bw.write("{{");
		        for (int i = 0; i < optimalAssignmentMatrix.length; i++) {
		        	for (int j = 0; j < optimalAssignmentMatrix[i].length; j++) {
		        		bw.write("{");
		        		for (int s = 0; s < optimalAssignmentMatrix[i][j].length; s++) {
		        			bw.write(optimalAssignmentMatrix[i][j][s]+"");
		        		}
		        		bw.write("}");
						if( j < optimalAssignmentMatrix[i].length-1 ){
							bw.write(",");
						}
		        		
		        		
		        	}
					if(i < optimalAssignmentMatrix.length-1) {
						bw.write("}");
						bw.write(",");
					}
					else{
						bw.write("}}");
					}

		            bw.newLine();
					if(i < optimalAssignmentMatrix.length-1){
						bw.write("{");
					}
		           
		        }
		        bw.write("-------------");
		        bw.newLine();
		        bw.flush();
		        bw.close();
		    } catch (IOException e) {}
	}

	
	private static void printLicense() {
		System.out.println("\n"+AbstractOptimizationProblem.TITLE);
		String cpr = AbstractOptimizationProblem.COPYRIGHT;
		for (String cont : AbstractOptimizationProblem.CONTRIBUTORS) cpr += ", " + cont;
		List<String> cprJust = StringUtils.fitWidth(cpr, 77, 0);
		for (String st : cprJust) System.out.println(st);
		System.out.println();
		if (AbstractOptimizationProblem.LICENSE != null) {
			List<String> lic = StringUtils.fitWidth(AbstractOptimizationProblem.PRIVATE_LICENSE, 72, 5);
			for (String st : lic) System.out.println(st);
		}
		else {
			List<String> lic = StringUtils.fitWidth(AbstractOptimizationProblem.PUBLIC_LICENSE, 72, 5);
			for (String st : lic) System.out.println(st);
		}
		System.out.println();
	}
	}

