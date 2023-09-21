package se.oru.assignment.assignment_oru.problems;


import java.util.List;

import org.metacsp.utility.logging.MetaCSPLogging;

import com.google.ortools.linearsolver.*;
import com.google.ortools.Loader;

import se.oru.assignment.assignment_oru.methods.AbstractOptimizationAlgorithm;


/**
 *  Class to create an online Single Task, Single Robot, Instantaneous Assignment (ST–SR–IA) Multi-Robot Task Assignment(MRTA) problem. The problem is posed as an online linear Optimal Assignment Problem (OAP)
 * The constraints considered are :
 * 1) Each Task can be assign only to a robot;
 * 2) Each Robot can perform only a task at time;
 * 3) Each robot perform a task following a single path.
 * An instantiatable {@link LinearOptimization} must provide an implementation of the {@link #findOptimalAssignment()} function to evaluate the optimization problem, and an implementations of
 * the {@link #evaluateBFunction()} and {@link #evaluateInterferenceCost} methods to evaluate the interference free cost and the interference cost respectively.
 * @author pofe
 *
 */
public class LinearOptimization extends AbstractOptimization<MPSolver> {
	 
	
	protected MPVariable[][][] decisionVariables;

	public LinearOptimization(){
		super();
		Loader.loadNativeLibraries();
		metaCSPLogger = MetaCSPLogging.getLogger(this.getClass());

	}

	/**
	 * Transform a 1D array of MPVariable into a 3D MATRIX  
	 * @param optimizationProblem -> The optimization problem defined with {@link #createOptimizationProblem }
	 * @return 3D Matrix of Decision Variable of the input problem
	*/
	protected MPVariable [][][] tranformArray(MPSolver optimizationProblem) {
		//Take the vector of Decision Variable from the Optimization Problem
		MPVariable [] array1D = optimizationProblem.variables();
		MPVariable [][][] decisionVariable = new MPVariable [numRobotAug][numTaskAug][alternativePaths];
		//Store them in a 2D Matrix
	    for (int i = 0; i < numRobotAug; i++) {
			 for (int j = 0; j < numTaskAug; j++) {
				 for (int s = 0; s < alternativePaths; s++) {
					 decisionVariable[i][j][s] = array1D[i*numTaskAug*alternativePaths+j*alternativePaths+s];
				 }
				 
			 }
	    }
		return decisionVariable;
	}
	
	/**
	 * Impose a constraint on the optimization problem on the previous optimal solution in order to not consider that solution anymore.
	 * @param assignmentMatrix -> The Assignment Matrix of the actual optimal solution
	 * @return An updated optimization problem  with an additional constraint on the previous optimal solution found. 
	 */
	public void constraintOnPreviousSolution(int [][][] assignmentMatrix) {
		//Take decision Variable from Optimization Problem
		MPVariable [][][] DecisionVariable = tranformArray(model);
		//Initialize a Constraint
		//MPConstraint c2 = optimizationProblem.makeConstraint(-Double.POSITIVE_INFINITY,1);
		MPConstraint c2 = model.makeConstraint(0,numRobotAug-1);
		//Define the actual optimal solution as a Constraint in order to not consider more it
    	for (int i = 0; i < numRobotAug; i++) {
    		for (int j = 0; j < numTaskAug; j++) {
    			for(int s = 0;s < alternativePaths; s++) {
    					if (assignmentMatrix[i][j][s] > 0) {
    						c2.setCoefficient(DecisionVariable[i][j][s],1);
    					}
    			}
    		}		
		 }
	}

	
	public void constraintOnCostSolution(double objectiveValue) {
		//Take the vector of Decision Variable from the input solver
		MPVariable [][][] decisionVariable = tranformArray(model);
		//Add tolerance
		objectiveValue = objectiveValue + 0.0005;
		//Initialize a Constraint
		MPConstraint c3 = model.makeConstraint(-Double.POSITIVE_INFINITY,objectiveValue);
		//Define a constraint for which the next optimal solutions considering only B must have a cost less than objectiveValue
    	for (int i = 0; i < numRobotAug; i++) {
    		for (int j = 0; j < numTaskAug; j++) {
    			for(int s = 0;s < alternativePaths; s++) {
					double value = model.objective().getCoefficient(model.variables()[i*numTaskAug*alternativePaths+j*alternativePaths+s]);
    				c3.setCoefficient(decisionVariable[i][j][s],value);
    			}
    		}		
		 }
	}

	
	/**
	 * Compute all the feasible solutions for this optimization problem.
	 * @return The list of all feasible solutions
	 */

	public List <int [][][]> getFeasibleSolutions(){
		//Create a new optimization problem

		MPSolver optimizationProblem = initializeOptiVars();
	
	    MPSolver.ResultStatus resultStatus = optimizationProblem.solve();
  
	    while(resultStatus != MPSolver.ResultStatus.INFEASIBLE ) {
			//Solve the optimization Problem
    		resultStatus = optimizationProblem.solve();
			if (resultStatus == MPSolver.ResultStatus.INFEASIBLE) {
				break;
			}
    		//If The solution is feasible increment the number of feasible solution
    		MPVariable [][][] decisionVariable = tranformArray(optimizationProblem);

			int [][][] assignmentMatrix = new int [numRobotAug][numTaskAug][alternativePaths];	
			//Store decision variable values in a Matrix
			for (int i = 0; i < numRobotAug; i++) {
				for (int j = 0; j < numTaskAug; j++) {
					for(int s = 0;s < alternativePaths; s++) {
						assignmentMatrix[i][j][s] = (int) decisionVariable[i][j][s].solutionValue();
					}
				}
			}

    		this.feasibleSolutions.add(assignmentMatrix);
			metaCSPLogger.info(feasibleSolutions.size() + " Solution(s) found");
    		
			//Add the constraint to actual solution -> in order to consider this solution as already found  
			MPVariable [][][] DecisionVariable = tranformArray(optimizationProblem);
			//Initialize a Constraint
			//MPConstraint c2 = optimizationProblem.makeConstraint(-Double.POSITIVE_INFINITY,1);
			MPConstraint c2 = optimizationProblem.makeConstraint(0,numRobotAug-1);
			//Define the actual optimal solution as a Constraint in order to not consider more it
			for (int i = 0; i < numRobotAug; i++) {
				for (int j = 0; j < numTaskAug; j++) {
					for(int s = 0;s < alternativePaths; s++) {
							if (assignmentMatrix[i][j][s] > 0) {
								c2.setCoefficient(DecisionVariable[i][j][s],1);
							}
					}
				}		
			}

	    }
		//Return the set of all Feasible solutions
	    return this.feasibleSolutions;
	    
	}

	/**
	 * Create the optimization problem with part of the Objective Function (Function B). Define a decision variable X_ijp_{ijs} as a binary variable in which i indicate
	 * the robot id, j the tasks, and p_{ijs} is the s-th path for the robot i-th to reach the target position of task j. The problem is build as an ST-ST-IA problem, i.e.:
	 * 1) Each Task can be assign only to a robot;
	 * 2) Each Robot can perform only a task at time.
	 * The objective function is defined as alpha*B + (1-alpha)*F 
	 * where B includes the cost related to single robot computed by the function ({@link #evaluateBFunction}), while F ({@link #evaluateInterferenceCost}) includes all costs related to interference
	 * between the robots.
	 * @return A constrained optimization problem
	 */
	protected MPSolver createOptimizationProblem() {
		//Perform a check in order to avoid blocking
		//checkOnBlocking(tec);
		//Evaluate dummy robot and dummy task
		dummyRobotorTask();
		initializeRobotsIDs();
		initializeTasksIDs();
		double[][][] BFunction = evaluateBFunction();
		//Build the optimization problem
		MPSolver optimizationProblem = initializeOptiVars();

		//Modify the coefficients of the objective function
		MPVariable [][][] decisionVariable = tranformArray(optimizationProblem); 
	    /////////////////////////////////
	    //START OBJECTIVE FUNCTION		
	    MPObjective objective = optimizationProblem.objective();
    	 for (int i = 0; i < numRobotAug; i++) {
			 for (int j = 0; j < numTaskAug; j++) {
				 for(int s = 0; s < alternativePaths; s++) {
					double singleRobotCost = 1;
					if(interferenceFreeCostMatrix!= null){
						 singleRobotCost  =  BFunction[i][j][s];
					}		
					 if ( singleRobotCost != Double.POSITIVE_INFINITY) {
						 //Set the coefficient of the objective function with the normalized path length
						 objective.setCoefficient(decisionVariable[i][j][s], singleRobotCost); 
					 }
				 }
			 }			 
		 }
		//Define the problem as a minimization problem
		objective.setMinimization();
		//END OBJECTIVE FUNCTION
		return optimizationProblem;	
	}

	/**
	 * Build the optimization problem. Define a decision variable X_ijp_{ijs} as a binary variable in which i indicate
	 * the robot id, j the tasks, and p_{ijs} is the s-th path for the robot i-th to reach the target position of task j. The problem is build as an ST-ST-IA problem, i.e.:
	 * the constraints considered are :
	 * 1) Each Task can be assign only to a robot;
	 * 2) Each Robot can perform only a task at time;
	 * @return A constrained optimization problem without the objective function
	 */
	protected MPSolver initializeOptiVars() {
		//Initialize a MP linear solver 
		MPSolver optimizationProblem = new MPSolver("TaskAssignment", MPSolver.OptimizationProblemType.CBC_MIXED_INTEGER_PROGRAMMING);
		
		//START DECISION VARIABLE VARIABLE
		decisionVariables = new MPVariable[numRobotAug][numTaskAug][alternativePaths];
		for (int i = 0; i < numRobotAug; i++) {
			 for (int j = 0; j < numTaskAug; j++) {
				 for(int s = 0; s < alternativePaths; s++) {
					decisionVariables[i][j][s] = optimizationProblem.makeBoolVar("x"+"["+i+","+j+","+s+"]");
					
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
					 c0.setCoefficient(decisionVariables[i][j][s], 1); 
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
					 c0.setCoefficient(decisionVariables[i][j][s], 1); 
				 } 		
			 }
		 }

		 //Type Constraint 
		 for (int robotID : robotsIDs ) {
			int i = robotsIDs.indexOf(robotID);
			for (int taskID : tasksIDs ) {
				int j = tasksIDs.indexOf(taskID);
				for(int s = 0; s < alternativePaths; s++) {
						if (i < realRobotsIDs.size()) { //Considering only real Robot
							double pss = interferenceFreeCostMatrix[i][j][s];
							//Type is different or path does not exists
							if(realTasksIDs.contains(taskID)){
								if(pss == Double.POSITIVE_INFINITY || !taskQueue.get(j).isCompatible(getRobotType(robotID))) {
								MPConstraint c3 = optimizationProblem.makeConstraint(0,0);
								c3.setCoefficient(decisionVariables[i][j][s],1); 
							}
							}				
						}
				}
			}
		}
		
		//In the case of having more tasks than robots, the tasks with the closest deadline that are set with a higher priority are imposed as a constraint into the problem in order 
		//to execute them during the current assignment.
		if(taskQueue.size() > realRobotsIDs.size()) {
			sortTaskByDeadline();
			//Each task can be performed only by a robot
			for (int j = 0; j < taskQueue.size(); j++) {
				//Initialize the constraint
				if(taskQueue.get(j).isPriority()) {
					MPConstraint c3 = optimizationProblem.makeConstraint(1, 1); 
					for (int i = 0; i < realRobotsIDs.size(); i++) {
						for(int s = 0; s < alternativePaths; s++) {
							//Build the constraint
							c3.setCoefficient(decisionVariables[i][j][s], 1); 
						} 		
					}
				}
			}
		}
		//END CONSTRAINTS
		/////////////////////////////////////////////////
		return optimizationProblem;	
	}

	public problemStatus solve(){
		MPSolver.ResultStatus resultStatus = model.solve();
		if(resultStatus == MPSolver.ResultStatus.INFEASIBLE){
			return problemStatus.valueOf(resultStatus.toString());
		}
		//Store decision variable values in a Matrix
		for (int i = 0; i < numRobotAug; i++) {
			for (int j = 0; j < numTaskAug; j++) {
				for(int s = 0;s < alternativePaths; s++) {
					currentAssignment[i][j][s] = (int) decisionVariables[i][j][s].solutionValue();
				}
			}
		}
		constraintOnPreviousSolution(getAssignmentMatrix());
		return problemStatus.valueOf(resultStatus.toString());
	}

	protected void clear(){
		model.clear();
	}

	public int [][][] findOptimalAssignment(AbstractOptimizationAlgorithm optimizationSolver){
		model = createOptimizationProblem();
		return super.findOptimalAssignment(optimizationSolver);
	}

	}

