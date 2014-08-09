package amo;

import java.io.File;
import java.io.FileWriter;
import java.io.IOException;
import java.util.ArrayList;


public class SimTest {

    /**
     * @param args
     */
    public static void main(String[] args) {
        // AMO
        
        // simulation parameters
        int NUM_TIMESTEPS  = 3000;
        int TRIALS         = 1;
        int START          = 0;
        String RESULTS_DIR = "output/amo";
        
        // UAV parameters
        final int NUM_UAVS       = 32;
        final double DETECT_RATE = 30.0;
        final int ATR_FRACTION   = 10;
        String CLOUD_DIR         = "data/ptclouds";
        String ESTS_PATH         = "data/performanceCurves.mat";
        
        // network parameters
        int[] transmitRates = new int[7];
        transmitRates[0]    = 5625000;                                      // CITATION: maximum TCDL rate in bytes = 45 Mbits/s (http://articles.janes.com/articles/Janes-C4I-Systems/Tactical-Common-Data-Link-TCDL-airborne-data-terminal-United-States.html)
        transmitRates[1]    = (int)Math.ceil(transmitRates[0] * (5 / 6.0));// 0x -> (5/6)x of the maximum rate
        transmitRates[2]    = (int)Math.ceil(transmitRates[0] * (4 / 6.0));
        transmitRates[3]    = (int)Math.ceil(transmitRates[0] * (3 / 6.0));
        transmitRates[4]    = (int)Math.ceil(transmitRates[0] * (2 / 6.0));
        transmitRates[5]    = (int)Math.ceil(transmitRates[0] * (1 / 6.0));
        transmitRates[6]    = 0;
        MarkovChain m       = new MarkovChain(transmitRates, 0, true);      // initialize the Markov chain with the above states
        m.setTransition(0, 0, 1.0);                                         // constant max-transmit state   
        m.setTransition(5, 5, 1.0);
        
        int FORECAST_FREQ   = 100;                                          // interval between system state updates
        int FORECAST_LEN    = 2;                                            // how long an update takes
        
        // ground parameters
        ArrayList<Integer> library = new ArrayList<Integer>();
        for(int i = 1; i <= 55; i++)                                        // add the military models (#1 - #55) to the reference library
            if(i != 24)                                                     // ignore the MLRS model
                library.add(i);
        final int NUM_MACHINES           = 4;                               // number of independent processing elements 
        final int XYZ_PER_SECOND         = 500;                             // CITATION: Cho, 2006, page 12
        final int SEGMENTED_PER_SECOND   = 500000;                          // Assuming negligible segmentation delay
        final int LIKELIHOODS_PER_SECOND = 7700;                            // CITATION: Experimentally-determined on GPU
        final String LIKES_DIR           = "data/likelihoods";
        final int DESIRED_LATENCY        = 120;                             // objective function desired maximum latency
        final double DESIRED_ACCURACY    = 0.95;                            // objective function desired minimum accuracy
        final boolean ACCURACY_PRIORITY  = true;                            // preference for accuracy over latency (or vice versa)
        
        // Create the output folder if it doesn't already exist
        File resultsDir = new File(RESULTS_DIR);
        if(!resultsDir.exists()) {
        	try {
        		resultsDir.mkdirs();
        	}
        	catch(SecurityException e) {
        		System.out.println(e.getMessage());
        		return;
        	}
        }
        
        // initialize and run the simulation for the specified # of trials
        for(int t = START; t < TRIALS; t++){
            Simulation s = new Simulation(NUM_TIMESTEPS);
            s.initializeUAVs(NUM_UAVS, DETECT_RATE, CLOUD_DIR, ESTS_PATH, XYZ_PER_SECOND, SEGMENTED_PER_SECOND, LIKELIHOODS_PER_SECOND / ATR_FRACTION, LIKES_DIR, library);
            s.initializeNetwork(m, FORECAST_FREQ, FORECAST_LEN);
            s.initializeGroundStation(NUM_MACHINES, XYZ_PER_SECOND, SEGMENTED_PER_SECOND, LIKELIHOODS_PER_SECOND, LIKES_DIR, library);
            s.initializeObjective(DESIRED_ACCURACY, DESIRED_LATENCY, ACCURACY_PRIORITY);
            
            // run the sim
            int n = 0;       
            while(s.step()){
                if(n % 100 == 0)
                    System.out.println(t + " - " + n);
                
                n++;
            }
            
            // write the results for each detection to a text file
            // for each row: DetectID, ObjectID, Policy, Detect->classify, Classification
            File outFile      = new File(RESULTS_DIR + "/" + t + ".txt");
            FileWriter writer = null;
            try {
                writer = new FileWriter(outFile, true);
            } catch (IOException e) {
                e.printStackTrace();
            }
            
            for(PointCloud p : s.getResults()){
                int detectID          = p.getDetectNumber();
                int objectID          = p.getPointCloudNumber();
                int policy            = p.getDecisionFormat();
                int detectToProcessed = p.getProcessedTime() - p.getDetectTime();
                int classification    = p.getClassification();
                
                try {
                    writer.write(detectID + "," + objectID + "," + policy + "," + detectToProcessed + "," + classification + "\n");
                } catch (IOException e) {
                    e.printStackTrace();
                }
            }
            try {
                writer.close();
                System.out.println("Done writing " + outFile.getAbsolutePath());
            } catch (IOException e) {
                e.printStackTrace();
            }
            
            s = null;
        }
    }

}
