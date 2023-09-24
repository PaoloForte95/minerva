package se.oru.assignment.assignment_oru.problems;

import java.util.logging.Logger;

import org.metacsp.multi.spatioTemporal.paths.PoseSteering;
import org.metacsp.utility.logging.MetaCSPLogging;

import aima.core.util.datastructure.Pair;



public abstract class AbstractDelayEvaluator {


    protected Logger logger = MetaCSPLogging.getLogger(this.getClass());

    public abstract Pair<Double, Double> evaluatePathDelay(int robot1ID,PoseSteering[] pss1, int robot2ID,PoseSteering[] pss2);
    
}
