package amo;


public class Network {
    // network status codes
    public static final int NET_IDLE         = 0;   // network is free
    public static final int NET_TRANSMITTING = 1;   // network is currently in the middle of a transmission
    public static final int NET_FORECAST     = 2;   // network is sharing its latency forecast with UAVs 
    
    // network fields
    private MarkovChain transmitRates;              // Markov chain of various transmission rates
    private int forecastInterval;                   // number of time steps between latency forecast updates
    private int forecastLength;                     // number of time steps necessary to broadcast latency forecasts
    
    private int status;                             // the network's status
    private PointCloud currentData;                 // current data being transmitted
    private int bytesRemaining;                     // the current number of bytes left to transmit
    
    private int lastForecast;                       // time when the last forecast completed
    private int statusBeforeForecast;               // network status right before a forecast update happens
    private int forecastTimeLeft;                   // time left until the forecast update is completed
    
    private MarkovChain forecast;                   // Markov chain (transmission rates and transition probabilities) updated at the last forecast time
    
    /**
     * 
     * @param transmitRates         markov chain of various transmission rates
     * @param forecastInterval      number of time steps between latency forecast updates
     * @param forecastLegnth        number of time steps necessary to broadcast latency forecasts
     */
    public Network(MarkovChain transmitRates, int forecastInterval, int forecastLength){
        this.transmitRates    = transmitRates;
        this.forecastInterval = forecastInterval;
        this.forecastLength   = forecastLength;
        
        this.lastForecast     = -1;
        this.forecastTimeLeft = forecastLength; 
        this.forecast         = new MarkovChain(transmitRates);
        
        status = NET_IDLE;
    }
    
    
    /**
     * Simulate one time step of the network. Determine the next transmission rate and attempt to transmit any current
     * data "in the pipe" at that rate.
     * @param currentTime   current simulated time for this step
     * @return              the transmitted point cloud if it has just finished, null otherwise
     */
    public PointCloud step(int currentTime){
        // get the new transmit rate
        int currentTransmitRate = transmitRates.getNextValue();
        
        // perform a latency forecast update if enough time has elapsed since the last one
        if(currentTime == 0 || (status != NET_FORECAST &&  currentTime - lastForecast >= forecastInterval)){
            // save the current state of the system if this is just the first step of the update and set status to forecasting
            statusBeforeForecast = status;
            status               = NET_FORECAST;
            lastForecast         = currentTime;                 // save the time of this forecast
            forecastTimeLeft     = forecastLength;
        } 
        else if(status == NET_FORECAST){
            // decrement the forecast timer
            forecastTimeLeft--;
            
            // check if forecasting finished; make the appropriate updates if done
            if(forecastTimeLeft == 0){
                status           = statusBeforeForecast;        // reset status to the previous one

                forecast = new MarkovChain(transmitRates);      // build a new forecast
            }
        }
        else if(status == NET_TRANSMITTING){
            // transmit at the current rate
            bytesRemaining += -currentTransmitRate;
            
            // check if the transmission finished; update the data set and set network status accordingly
            if(bytesRemaining <= 0){
                status = NET_IDLE;
                
                return currentData;
            }
        }
        
        return null;
    }
    
    /**
     * Attempt to begin transmitting a point cloud.
     * @param               p point cloud to transmit
     * @param currentTime   current simulated time
     * @return              true if the point cloud has been successfully accepted for transmission
     */
    public boolean startTransmit(PointCloud p, int currentTime){
        // check network status, terminate if busy
        if(status == NET_TRANSMITTING)
            return false;
        
        // record the transmit start time
        p.transmitUpdate(currentTime);
        
        // set the status and current transmitting data
        status          = NET_TRANSMITTING;
        currentData     = p;
        bytesRemaining  = p.getBytes(); 
        
        return true;
    }
    
    /**
     * 
     * @return  the network status; can be Network.NET_IDLE or Network.NET_TRANSMITTING
     */
    public int getStatus(){
        return status;
    }
    
    public MarkovChain getForecast(){
        return forecast;
    }
}
