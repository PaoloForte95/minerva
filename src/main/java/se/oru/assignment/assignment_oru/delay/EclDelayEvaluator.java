package se.oru.assignment.assignment_oru.delay;

import java.util.logging.Logger;

import org.metacsp.multi.spatioTemporal.paths.PoseSteering;
import org.metacsp.utility.logging.MetaCSPLogging;

import aima.core.util.datastructure.Pair;
import se.oru.assignment.assignment_oru.fleetmasterinterface.AbstractFleetMasterInterface;

public final class EclDelayEvaluator implements AbstractDelayEvaluator {
    protected AbstractFleetMasterInterface eclInterface;
    protected Logger logger = MetaCSPLogging.getLogger(this.getClass());

    public EclDelayEvaluator(AbstractFleetMasterInterface eclInterface){
        super();
        this.eclInterface = eclInterface;
    }


    @Override
    public Pair<Double, Double> evaluatePathDelay(int robot1ID, PoseSteering[] pss1, int robot2ID, PoseSteering[] pss2) {
        if (this.eclInterface != null){
            return eclInterface.computeTimeDelayWPath(pss1, pss2, robot1ID, robot2ID);
        }
        return new Pair<Double, Double> (Double.NaN, Double.NaN);
    }

    @Override
    public Pair<Double, Double> evaluatePathDelay(int robot1ID, PoseSteering[] pss1, double[] curvs1, int robot2ID, PoseSteering[] pss2, double[] curvs2) {
        if (this.eclInterface != null){
            return eclInterface.computeTimeDelayWPath(pss1, curvs1, pss2,curvs2, robot1ID, robot2ID);
        }
        return new Pair<Double, Double> (Double.NaN, Double.NaN);
    }
    
}
