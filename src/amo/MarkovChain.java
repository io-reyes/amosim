package amo;


public class MarkovChain {
    private double[][] transitionProbs;
    private int[] stateValues;
    private int currentState, numStates;
    private boolean symmetric;
    
    /**
     * 
     * @param stateValues The value associated with each state.
     * @param startState Starting state.
     * @param symmetric True if a-->b transition probability = b-->a
     */
    public MarkovChain(int[] stateValues, int startState, boolean symmetric){
        // save the arguments
        this.stateValues  = stateValues;
        this.currentState = startState; 
        this.symmetric    = symmetric;
        this.numStates    = stateValues.length;
        
        // initialize the transition probability table to all 0s
        this.transitionProbs = new double[this.numStates][this.numStates];
        for(int i = 0; i < this.numStates; i++)
            for(int j = 0; j < this.numStates; j++)
                this.transitionProbs[i][j] = 0;
    }
    
    /**
     * Copy constructor.
     * @param m Markov chain to deep-copy
     */
    public MarkovChain(MarkovChain m){
        transitionProbs = m.transitionProbs;
        stateValues     = m.stateValues;
        currentState    = m.currentState;
        numStates       = m.numStates;
        symmetric       = m.symmetric;
    }
    
    /**
     * Sets the transition probability between two nodes.
     * 
     * @param src Source node
     * @param dest Destination node
     * @param prob Probability to be assigned (0 <= prob <= 1)
     */
    public void setTransition(int src, int dest, double prob){
        // check the inputs: within bounds for the source and destination ,and valid probability
        if(src < 0 || src >= transitionProbs.length || dest < 0 || dest > transitionProbs.length || prob < 0 || prob > 1)
            return;
        
        transitionProbs[src][dest] = prob;
        transitionProbs[dest][src] = symmetric ? prob : transitionProbs[dest][src];     // set other direction if symmetric
    }
    
    /**
     * 
     * @return The next value in the Markov chain
     */
    public int getNextValue(){
        double sum    = 0;
        double random = Math.random();
        int index     = 0;
        
        while(sum < random)
            sum += transitionProbs[currentState][index++];
        
        currentState = index - 1;
        return stateValues[currentState];
    }
    
    /**
     * 
     * @return The current value in the Markov chain without changing its state.
     */
    public int getCurrentValue(){
        return stateValues[currentState];
    }
    
    /**
     * 
     * @return An N x 2 matrix, where the first column contains a Markov state value and the second column
     *         has the transition probability to that value from the current state.
     */
    public double[][] getCurrentTransitions(){
        double[][] transitionsFromHere = new double[stateValues.length][2];
        
        for(int n = 0; n < stateValues.length; n++){
            transitionsFromHere[n][0] = stateValues[n];
            transitionsFromHere[n][1] = transitionProbs[currentState][n];
        }
        
        return transitionsFromHere;
    }
    
    /**
     * Sets the current state of the Markov chain
     * 
     * @param currentState
     */
    public void setState(int currentState){
        if(currentState >= 0 && currentState < numStates)
            this.currentState = currentState;
    }
}
