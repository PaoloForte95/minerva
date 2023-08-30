package se.oru.assignment.assignment_oru.problems;



import java.util.ArrayList;

import java.util.List;

import org.metacsp.utility.logging.MetaCSPLogging;

import com.google.ortools.sat.CpModel;
import com.google.ortools.sat.CpSolver;
import com.google.ortools.sat.CpSolverStatus;
import com.google.ortools.sat.LinearExpr;
import com.google.ortools.sat.LinearExprBuilder;
import com.google.ortools.sat.Literal;
import com.google.ortools.Loader;

import se.oru.assignment.assignment_oru.methods.AbstractOptimizationAlgorithm;


/**
 *  Class to create an online Single Task, Single Robot, Instantaneous Assignment (ST–SR–IA) Multi-Robot Task Assignment(MRTA) problem. The problem is posed as an online linear Optimal Assignment Problem (OAP)
 * The constraints considered are :
 * 1) Each Task can be assign only to a robot;
 * 2) Each Robot can perform only a task at time;
 * 3) Each robot perform a task following a single path.
 * An instantiatable {@link ConstraintOptimization} must provide an implementation of the {@link #buildOptimizationProblem()} function to create the optimization problem, and an implementations of
 * the {@link #evaluateBFunction()} and {@link #evaluateInterferenceCost} methods to evaluate the interference free cost and the interference cost respectively.
 * @author pofe
 *
 */
public class ConstraintOptimization extends AbstractOptimizationProblem<CpModel>{
	 
	protected CpModel model;
	protected CpSolver solver;
	protected Literal[][][] decisionVariables;
	

	public ConstraintOptimization(){
		super();
		Loader.loadNativeLibraries();
		metaCSPLogger = MetaCSPLogging.getLogger(this.getClass());
		this.model = new CpModel();
		
	}

	/**
	 * Impose a constraint on the optimization problem on previous optimal solution in order to not consider more it
	 * @param optimizationProblem -> An optimization problem  defined with {@link #buildOptimizationProblem},{@link #buildOptimizationProblemWithB} or {@link #buildOptimizationProblemWithBNormalized} in which a solution is found
	 * @param assignmentMatrix -> The Assignment Matrix of the actual optimal solution
	 * @return Optimization Problem updated with the new constraint on previous optimal solution found  
	 */
	public void constraintOnPreviousSolution(int [][][] assignmentMatrix) {
		//Initialize a Constraint
		LinearExprBuilder c2 = LinearExpr.newBuilder();
		//Define the actual optimal solution as a constraint in order to not consider more it
		for (int robot = 0; robot < assignmentMatrix.length; robot++) {
			for (int task = 0; task < assignmentMatrix[0].length; task++) {
				for(int path = 0; path < assignmentMatrix[0][0].length; path++) {
					if (assignmentMatrix[robot][task][path] > 0) {
						c2.addTerm(decisionVariables[robot][task][path],1);
						
					}
				}
			}
		}
		model.addLessOrEqual(c2,assignmentMatrix.length-1);
    	//Return the updated Optimization Problem
	}

	
	/**
	 * Impose a constraint on the optimization problem on previous optimal solution cost in order to prune all solutions with a cost higher
	 * than objectiveValue . In this manner is possible to avoid some cases.
	 * @param optimizationProblem -> An optimization problem  defined with {@link #buildOptimizationProblem}
	 * @param assignmentMatrix -> The Assignment Matrix of the actual optimal solution
	 * @return Optimization Problem updated with the new constraint on optimal solution cost 
	 */
	
	public void constraintOnCostSolution(double objectiveValue) {
		//Initialize a Constraint
		LinearExprBuilder c3 = LinearExpr.newBuilder();
		//Add tolerance
		objectiveValue = objectiveValue + 0.0005;
		//Define a constraint for which the next optimal solutions considering only B must have a cost less than objectiveValue
    	for (int robot = 0; robot < numRobotAug; robot++) {
			for (int task = 0; task < numTaskAug; task++) {
				for(int path = 0; path < alternativePaths; path++) {
					long coeff = (long) interferenceFreeCostMatrix[robot][task][path];
					c3.addTerm(decisionVariables[robot][task][path],coeff);
    			}
    		}		
		 }
		 model.addLessOrEqual(c3,(long) objectiveValue);
	}


	/**
	 * Get all the feasible solutions for this optimization problem.
	 * @return The list of all feasible solutions
	 */

	public List <int [][][]> getFeasibleSolutions(){
		//Define the optimization problem
		//return this.feasibleSolutions;
		
		CpModel optimizationProblem = buildOptimizationProblem();
	    CpSolverStatus resultStatus =  solver.solve(optimizationProblem);
	    while(resultStatus != CpSolverStatus.INFEASIBLE) {
			//Solve the optimization Problem
    		resultStatus = solver.solve(optimizationProblem);
			    		
    		if (resultStatus == CpSolverStatus.INFEASIBLE) {
    			break;
    		}
    		//If The solution is feasible increment the number of feasible solution
    		int [][][] assignmentMatrix = getAssignmentMatrix();
			this.feasibleSolutions.add(assignmentMatrix);
			metaCSPLogger.info(feasibleSolutions.size() + " Solutions found");
			//Add the constraint to actual solution -> in order to consider this solution as already found  
					//Initialize a Constraint
			LinearExprBuilder c2 = LinearExpr.newBuilder();
			for (int robot = 0; robot < assignmentMatrix.length; robot++) {
				for (int task = 0; task < assignmentMatrix[0].length; task++) {
					for(int path = 0; path < assignmentMatrix[0][0].length; path++) {
						if (assignmentMatrix[robot][task][path] > 0) {
							c2.addTerm(decisionVariables[robot][task][path],1);
							
						}
					}
				}
			}
			optimizationProblem.addLessOrEqual(c2,assignmentMatrix.length-1);
	    }
		//Return the set of all Feasible solutions
	    return this.feasibleSolutions;
	    }
	
	
	/**
	 * Build the optimization problem. User need to define a decision variable as a boolean variable and the constraints associated to the problem.
	 * @return A constrained optimization problem
	 */
	protected CpModel buildOptimizationProblem(){
		//Declare the CP-SAT model
		CpModel model = new CpModel();

		//Creates binary integer variables for the problem
		decisionVariables = new Literal[numRobotAug][numTaskAug][alternativePaths];
		for (int robot = 0; robot < numRobotAug; robot++) {
			for (int task = 0; task < numTaskAug; task++) {
				for(int path = 0; path < alternativePaths; path++) {
					decisionVariables[robot][task][path] = model.newBoolVar("x"+"["+robot+","+task+","+path+"]");
				}
			}
		}

		//Creates the constraints for the problem
		for (int robot = 0; robot < numRobotAug; robot++) {
			List<Literal> tasksCons = new ArrayList<>();
			for (int task = 0; task < numTaskAug; task++) {
				for(int path = 0; path < alternativePaths; path++) {
					//Build the constraint
					tasksCons.add(decisionVariables[robot][task][path]); 
				}
				model.addAtMostOne(tasksCons);
			}

		}
		
		// Each task is assigned to exactly one robot.
		for (int task = 0; task < numTaskAug; task++) {
			List<Literal> robotsCons = new ArrayList<>();
			for (int robot = 0; robot < numRobotAug; robot++) {
				for(int path = 0; path < alternativePaths; path++) {
					robotsCons.add(decisionVariables[robot][task][path]);
				}
			}
			model.addExactlyOne(robotsCons);
		}


		 //Type Constraint 
		 for (int robotID : robotsIDs ) {
			int i = robotsIDs.indexOf(robotID);
			LinearExprBuilder capabilityConstraint = LinearExpr.newBuilder();
			for (int taskID : tasksIDs ) {
				int j = tasksIDs.indexOf(taskID);
				for(int s = 0; s < alternativePaths; s++) {
						if (i < realRobotsIDs.size()) { //Considering only real Robot
							double pss = interferenceFreeCostMatrix[i][j][s];
							//Type is different or path does not exists
							if(pss == Double.POSITIVE_INFINITY || !taskQueue.get(j).isCompatible(getRobotType(robotID))) {
								capabilityConstraint.addTerm(decisionVariables[i][j][s],1);
							}
						}
				}
				model.addEquality(capabilityConstraint, 0);
			}
		}

		return model;
	}

	/**
	 * Create the optimization problem with part of the Objective Function (Function B). Define a decision variable X_ijp_{ijs} as a binary variable in which i indicate
	 * the robot id, j the tasks, and p_{ijs} is the s-th path for the robot i-th to reach the target position of task j. The problem is build as an ST-ST-IA problem, i.e.:
	 * 1) Each Task can be assign only to a robot;
	 * 2) Each Robot can perform only a task at time.
	 * The objective function is defined as alpha*B + (1-alpha)*F 
	 * where B includes the cost related to single robot ({@link #evaluateBFunction}), while F ({@link #evaluateInterferenceCost}) includes all costs related to interference
	 * @return A constrained optimization problem
	 */
	protected CpModel createOptimizationProblem() {
		dummyRobotorTask();
		initializeRobotsIDs();
		initializeTasksIDs();
		double[][][] BFunction = evaluateBFunction();
		
		//Build the optimization problem
		CpModel optimizationProblem = buildOptimizationProblem();
		
	    /////////////////////////////////
	    //Create the objective function for the problem.
		LinearExprBuilder objective = LinearExpr.newBuilder();
    	 for (int i = 0; i < numRobotAug; i++) {
			 for (int j = 0; j < numTaskAug; j++) {
				 for(int s = 0; s < alternativePaths; s++) {
					 double singleRobotCost  =  BFunction[i][j][s];
					 if ( singleRobotCost != Double.POSITIVE_INFINITY) {
						 //Set the coefficient of the objective function with the normalized path length
						 objective.addTerm(decisionVariables[i][j][s], (long)singleRobotCost); 
					 }
				 }
			 }			 
		 }
		//Define the problem as a minimization problem
		optimizationProblem.minimize(objective);
		//END OBJECTIVE FUNCTION
		return optimizationProblem;	
	}

	/**
	 * Get the model of the optimization function (mathematical model).
	 * @return The model of this optimization problem.
	 */
	
	public CpModel getModel() {
		return this.model;
	}

	/**
	 * Get the CP solver used to find a solution
	*/
	public CpSolver getSolver() {
		return this.solver;
	}

	protected double [][][] evaluateBFunction(){
		return this.interferenceFreeCostMatrix;
	}

	public double evaluateInterferenceCost(int robotID ,int taskID,int pathID,int [][][] assignmentMatrix){
		return interferenceCostMatrix[robotID][taskID][pathID];
	}

	public problemStatus solve(){
		CpSolverStatus resultStatus = solver.solve(model);
		//Add the constraint to actual solution in order to consider this solution as already found  
		if(resultStatus == CpSolverStatus.INFEASIBLE){
			return problemStatus.valueOf(resultStatus.toString());
		}
		//Store decision variable values in a Matrix
		for (int i = 0; i < numRobotAug; i++) {
			for (int j = 0; j < numTaskAug; j++) {
				for(int s = 0;s < alternativePaths; s++) {
					Boolean assigned = solver.booleanValue(decisionVariables[i][j][s]);
					currentAssignment[i][j][s] = 0;
					if(assigned){
						currentAssignment[i][j][s] = 1;
					}
				}
			}
		}
		constraintOnPreviousSolution(currentAssignment);
		return problemStatus.valueOf(resultStatus.toString());
	}


	public int [][][] findOptimalAssignment(AbstractOptimizationAlgorithm optimizationSolver){
		model = createOptimizationProblem();
		solver = new CpSolver();
		return super.findOptimalAssignment(optimizationSolver);
	}

	}

