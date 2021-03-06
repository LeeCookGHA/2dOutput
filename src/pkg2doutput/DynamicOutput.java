package pkg2doutput;

import java.awt.Color;
import java.awt.Dimension;
import java.awt.event.MouseListener;
import java.awt.event.MouseEvent;
import javax.swing.*;

public class DynamicOutput extends JPanel
        implements MouseListener {
    int dummy = 0;
    CanvasArea canvasArea;
    static final String NEWLINE = System.getProperty("line.separator");

    // Graphics variables
    private JFrame frame;
    static final int GRAPHICS_WIDTH = 2001;
    static final int GRAPHICS_HEIGHT = 2001;
    private static final long serialVersionUID = 1L;
    public static boolean frameInitialised = false;
    public boolean appInitialised = false;
    private double outputScale = 5d;
    Vector2 outputCenter = new Vector2();

    // Photo Diode Info
    final static double maxAoI = 90d;
    final static double relPowerRatioVsAoI = 1d/maxAoI;  // Approx to linear 0.9==25deg
    final static double maxDist = 20d;
    final static double relPowerRatioVsDist = 1d/maxDist; // Approx drops to zero at 20m
    final static double minRelPower = 0.05;
    
    // Sensor Cluster Info
    static int CLUSTER_SIZE = 32;
    Vector3[] clusterSensorPositions = new Vector3[CLUSTER_SIZE]; // Meters
    Vector3[] clusterSensorNormalPositions = new Vector3[CLUSTER_SIZE]; // Meters
    Spherical3[] rawBaseToSensorSpherical = new Spherical3[CLUSTER_SIZE];
    Spherical3 rawSensorsCentroid = new Spherical3();
    Spherical3[] zeroedRawBaseToSensorSpherical = new Spherical3[CLUSTER_SIZE];
    boolean[] base1ToSensorVisible = new boolean[CLUSTER_SIZE];
    int numberOfVisibleSensors = 0;

    // General items
    Vector3 zeroVector = new Vector3(0, 0, 0); // meters

    // =================================================================
    // =================================================================
    //                    EXPOSED POSER VARIABLES
    // =================================================================
    // =================================================================
    Vector3 vectorEstimate = new Vector3();
    Vector3 positionEstimate = new Vector3();
    Spherical3 bearingEstimate = new Spherical3();

    final static double icoSphereRadius = 0.5d; // Use to determine bearing estimates
    boolean[] icoSpherePointWithinAoI = new boolean[ICOSPHERE_POINTS];
    final static double sensorMaxFoR = 80d; // Use to determine bearing estimates
    Vector3[] clusterSensorEstimatedPositions = new Vector3[CLUSTER_SIZE]; // Meters
    Spherical3[] clusterSensorEstimateSpherical = new Spherical3[CLUSTER_SIZE];
    Spherical3[] zeroedClusterSensorEstimateSpherical = new Spherical3[CLUSTER_SIZE];
    double[] clusterSensorSortedListAngles = new double[CLUSTER_SIZE];
    int[] clusterSensorSortedListSensorId = new int[CLUSTER_SIZE];
    Spherical3[] perspectivePointsFromEstimate = new Spherical3[CLUSTER_SIZE];
    Spherical3 perspectivePointsCentroid = new Spherical3();
    double averageMagnitudeFromPerspectiveCentroid = 0d;
    Spherical3 estimatedSensorsCentroid = new Spherical3();
    Vector2 centroidDelta = new Vector2();

    
    
    private void initialiseClassVariables() {
        for (int count = 0; count < CLUSTER_SIZE; count++) {
            clusterSensorPositions[count] = new Vector3();
            clusterSensorNormalPositions[count] = new Vector3();

            rawBaseToSensorSpherical[count] = new Spherical3();
            zeroedRawBaseToSensorSpherical[count] = new Spherical3();

            clusterSensorEstimatedPositions[count] = new Vector3();
            clusterSensorEstimateSpherical[count] = new Spherical3();
            zeroedClusterSensorEstimateSpherical[count] = new Spherical3();

            perspectivePointsFromEstimate[count] = new Spherical3();
        }
    }




    
    // Run through the points of an IcoSphere and see which are covered by the
    // FoR of each of the sensors which have been lit by the (H/V) laser sweeps.
    // Take an average of the bearings for those IcoSphere points to get an
    // initial bearing for the poser.
    private void getInitialBearingFromIcoSphere() {
        System.out.printf("\n\rgetInitialBearingFromIcoSphere:");
        // IcoSphere items
        int tempCount = 0;
        double tempIcoSphereAoI = 0.0d;
        Vector3 tempIcoSphereVector = new Vector3();
        Vector3 tempNormalsVector = new Vector3();
        Spherical3 tempIcoSphereSpherical = new Spherical3();
        
        // Create the estaimted position vectors
        vectorEstimate.set(0, 0, 0);

        for (int spCount = 0; spCount < ICOSPHERE_POINTS; spCount++) {
            icoSpherePointWithinAoI[spCount] = true;
        }

        // Search for positions in the IcoSphere to give a general 
        for (int count = 0; count < CLUSTER_SIZE; count++) {
            for (int spCount = 0; spCount < ICOSPHERE_POINTS; spCount++) {
                if ((icoSpherePointWithinAoI[spCount])&&(base1ToSensorVisible[count])) {
                    tempIcoSphereVector.set(icoSphereCoords[spCount][0]*icoSphereRadius, icoSphereCoords[spCount][1]*icoSphereRadius, icoSphereCoords[spCount][2]*icoSphereRadius);
                    tempIcoSphereAoI = angleThreePointsVector3(clusterSensorNormalPositions[count], clusterSensorPositions[count], tempIcoSphereVector);
                    icoSpherePointWithinAoI[spCount] = (tempIcoSphereAoI < Math.toRadians(sensorMaxFoR));
                }
            }
        }

        System.out.printf("\n\r   IcoSphere Lit: ");
        tempIcoSphereVector.set(0d, 0d, 0d);
        for (int spCount = 0; spCount < ICOSPHERE_POINTS; spCount++) {
            if (icoSpherePointWithinAoI[spCount]) {
                tempIcoSphereVector.set(icoSphereCoords[spCount][0], icoSphereCoords[spCount][1], icoSphereCoords[spCount][2]);
                vectorEstimate.add(tempIcoSphereVector);
                tempCount++;
                System.out.printf("   %d", spCount);
            }
        }
        
        vectorEstimate.x /= tempCount;
        vectorEstimate.y /= tempCount;
        vectorEstimate.z /= tempCount;
        positionEstimate.set(0d, 0d, 0d);
        positionEstimate.sub(vectorEstimate);
    }

    // Get the estimated range from the initial bearing and 
    private void reviseRangeFromSensorAngles() {
//        System.out.printf("\n\r\n\rreviseRangeFromSensorAngles:");
        double maxMeasuredAngle = 0d;
        double tempEstimatedAngle = 0d;
        double elDiff;
        double azDiff;
        int heldOuter = 0;
        int heldInner = 1;
        Vector3 estimateOffset = new Vector3();

        for (int outerCount = 0; outerCount < (CLUSTER_SIZE-1); outerCount++) {
            if(base1ToSensorVisible[outerCount]){
                for (int innerCount = (outerCount+1); innerCount < CLUSTER_SIZE; innerCount++) {
                    if(base1ToSensorVisible[innerCount]){
                        azDiff = rawBaseToSensorSpherical[outerCount].az - rawBaseToSensorSpherical[innerCount].az;
                        elDiff = rawBaseToSensorSpherical[outerCount].el - rawBaseToSensorSpherical[innerCount].el;
                        tempEstimatedAngle = Math.sqrt((azDiff*azDiff)+(elDiff*elDiff));
                        if (tempEstimatedAngle > maxMeasuredAngle){
                            maxMeasuredAngle = tempEstimatedAngle;
                            heldOuter = outerCount;
                            heldInner = innerCount;
                        }
                    }
                }
            }
        }
        System.out.printf("\n\r\n\r   First Sensor(%d), Second Sensor(%d)\n\r   Measured Range angle (%2.10f)", heldOuter, heldInner, Math.toDegrees(maxMeasuredAngle));
        estimateOffset.set(vectorEstimate);

        // Get calculated and the angle from estimated data
        tempEstimatedAngle = angleThreePointsVector3(clusterSensorPositions[heldOuter], vectorEstimate, clusterSensorPositions[heldInner]);
        System.out.printf("\n\r   Initial Range Angle (%2.10f)", Math.toDegrees(tempEstimatedAngle));

        estimateOffset.scale (tempEstimatedAngle/maxMeasuredAngle);
        vectorEstimate.set(estimateOffset);
        tempEstimatedAngle = angleThreePointsVector3(clusterSensorPositions[heldOuter], vectorEstimate, clusterSensorPositions[heldInner]);
        System.out.printf("\n\r   Scaled Range Angle (%2.10f)", Math.toDegrees(tempEstimatedAngle));
        System.out.printf("\n\r   EST Psn: (%2.10f, %2.10f, %2.10f)", vectorEstimate.x, vectorEstimate.y, vectorEstimate.z);
    }
    

    // Get the estimated range from the initial bearing and 
    private void reviseRangeFromSensorLitAndEstSensorPosn() {
//        System.out.printf("\n\r\n\rreviseRangeFromSensorLitAndEstSensorPosn:");
        double visMeasuredAngle = 0d;
        double estMeasuredAngle = 0d;
        double elDiff;
        double azDiff;

        // Find the pair with the widest angle
        azDiff = zeroedRawBaseToSensorSpherical[8].az;
        elDiff = zeroedRawBaseToSensorSpherical[8].el;
        visMeasuredAngle = Math.sqrt((azDiff*azDiff)+(elDiff*elDiff));

        azDiff = zeroedClusterSensorEstimateSpherical[8].az;
        elDiff = zeroedClusterSensorEstimateSpherical[8].el;
        estMeasuredAngle = Math.sqrt((azDiff*azDiff)+(elDiff*elDiff));
        
        bearingEstimate.r /= (visMeasuredAngle/estMeasuredAngle);
        vectorEstimate.setFromSpherical(bearingEstimate);
    }


    
    // Create the values from the system, giving which sensors within the cluster
    // are lit and what the bearing is to that sensor from the base
    // ALL ANGLES ARE BASED AROUND THE CLUSTER BEING AT ZERO
    private void getEstimatedSensorAngles() {
//        System.out.printf("\n\rgetEstimatedSensorAngles:");
    estimatedSensorsCentroid.az = 0d;
    estimatedSensorsCentroid.el = 0d;
    estimatedSensorsCentroid.r = 0d;

        // Setup sensor cluster detector positions and normal positions (wrt to the detector)
        for (int count = 0; count < CLUSTER_SIZE; count++) {
            if(base1ToSensorVisible[count]){
                // Get the Spherical Co-ords for the sensors
                clusterSensorEstimatedPositions[count].set(vectorEstimate);
                clusterSensorEstimatedPositions[count].add(clusterSensorPositions[count]);
                clusterSensorEstimateSpherical[count].setFromVector3(clusterSensorEstimatedPositions[count]);
                estimatedSensorsCentroid.az += clusterSensorEstimateSpherical[count].az;
                estimatedSensorsCentroid.el += clusterSensorEstimateSpherical[count].el;
                estimatedSensorsCentroid.r++;
            }
        }
        estimatedSensorsCentroid.az /= estimatedSensorsCentroid.r;
        estimatedSensorsCentroid.el /= estimatedSensorsCentroid.r;
        
        bearingEstimate.setFromVector3(vectorEstimate);

        // Run through the cluster and zero for the centroid
        for (int count = 0; count < CLUSTER_SIZE; count++) {
            if(base1ToSensorVisible[count]){
                zeroedClusterSensorEstimateSpherical[count].az = clusterSensorEstimateSpherical[count].az - bearingEstimate.az;
                zeroedClusterSensorEstimateSpherical[count].el = clusterSensorEstimateSpherical[count].el - bearingEstimate.el;
//                System.out.printf("\n\r   EST %d: ", count);
//                System.out.printf("(%3.18f, %3.18f, %2.10f), ", Math.toDegrees(zeroedClusterSensorEstimateSpherical[count].az), Math.toDegrees(zeroedClusterSensorEstimateSpherical[count].el), zeroedClusterSensorEstimateSpherical[count].r);
            }
        }
        System.out.print("\n\r");
    }
    
    
    // Create the values from the CNLohrs readins
    private void getCNLohrReadings() {
        System.out.printf("\n\r\n\rgetCNLohrReadings:");
        rawSensorsCentroid.setFromRads(0d, 0d, 0d);

        // Setup sensor cluster detector positions and normal positions (wrt to the detector)
        for (int count = 0; count < CLUSTER_SIZE; count++) {
            clusterSensorPositions[count].x = devicePoints[count][2];
            clusterSensorPositions[count].y = devicePoints[count][0];
            clusterSensorPositions[count].z = devicePoints[count][1];

            clusterSensorNormalPositions[count].x = clusterSensorPositions[count].x + deviceNormals[count][2]; // Meters
            clusterSensorNormalPositions[count].y = clusterSensorPositions[count].y + deviceNormals[count][0]; // Meters
            clusterSensorNormalPositions[count].z = clusterSensorPositions[count].z + deviceNormals[count][1]; // Meters
            
            if ((cnlohrHMDReadings[count][0]<0.000001) && (cnlohrHMDReadings[count][1]<0.000001)){
                base1ToSensorVisible[count] = false;
            } else {
                base1ToSensorVisible[count] = true;
                numberOfVisibleSensors++;

                // Get the Spherical Co-ords for the sensors
                rawBaseToSensorSpherical[count].setFromDegrees(cnlohrHMDReadings[count][0], cnlohrHMDReadings[count][1], 1);
                rawSensorsCentroid.az += rawBaseToSensorSpherical[count].az;
                rawSensorsCentroid.el += rawBaseToSensorSpherical[count].el;
                rawSensorsCentroid.r++;
            }
        }
        rawSensorsCentroid.az /= rawSensorsCentroid.r;
        rawSensorsCentroid.el /= rawSensorsCentroid.r;
        rawSensorsCentroid.r = 1d;

        // Run through the cluster and print the results
//        System.out.printf("\n\r   Base 1 Posn (x:%2.10f, y:%2.10f, z:%2.10f)", base1OriginPosition.x, base1OriginPosition.y, base1OriginPosition.z);
//        System.out.printf("\n\r   Clus to Base Brng (a:%2.10f, e:%2.10f, r:%2.10f), ", actualBearingFromBase.az, actualBearingFromBase.el, actualBearingFromBase.r);
//        System.out.print("\n\r   Sens: (Base1 centric x, y, z), (Az Angle, El Angle, Range), AoI, Relative Power");
        for (int count = 0; count < CLUSTER_SIZE; count++) {
            if(base1ToSensorVisible[count]){
                zeroedRawBaseToSensorSpherical[count].az = rawBaseToSensorSpherical[count].az - rawSensorsCentroid.az;
                zeroedRawBaseToSensorSpherical[count].el = rawBaseToSensorSpherical[count].el - rawSensorsCentroid.el;
                System.out.printf("\n\r   CNL %d: ", count);
                System.out.printf("(%3.18f, %3.18f, %2.10f), ", Math.toDegrees(zeroedRawBaseToSensorSpherical[count].az), Math.toDegrees(zeroedRawBaseToSensorSpherical[count].el), zeroedRawBaseToSensorSpherical[count].r);
            }
        }
//        System.out.print("\n\r");
    }




// Works out and sorts the 2d angles from vertical
    private void get2dAnglesFromEstimate(){
        Vector2 point1 = new Vector2();
        double angle = 0;

        for (int count = 0; count < CLUSTER_SIZE; count++) {
            clusterSensorSortedListSensorId[count] = -1;
            clusterSensorSortedListAngles[count] = 1000;
        }
        
        // Lit list with angle
        int litListCount = 0;
        for (int count = 0; count < CLUSTER_SIZE; count++) {
            if(base1ToSensorVisible[count]){
                point1.set(zeroedClusterSensorEstimateSpherical[count].az, zeroedClusterSensorEstimateSpherical[count].el);
                angle = angleFromVerticalVector2(point1);
//                System.out.printf("\n\r %d(x:%2.10f, y:%2.10f): %2.10f", count, point1.x, point1.y, Math.toDegrees(angle));
                clusterSensorSortedListSensorId[litListCount] = count;
                clusterSensorSortedListAngles[litListCount] = angle;
                litListCount++;
            }
        }
        
        double tempDouble = 0;
        int tempInt = 0;
        int maxSortLit = numberOfVisibleSensors;
        for (int outerCount = 0; outerCount < numberOfVisibleSensors-1; outerCount++) {
            for (int innerCount = 0; innerCount < maxSortLit-1; innerCount++) {
                if (clusterSensorSortedListAngles[innerCount] > clusterSensorSortedListAngles[innerCount+1]){
                    tempDouble = clusterSensorSortedListAngles[innerCount];
                    tempInt = clusterSensorSortedListSensorId[innerCount];
                    clusterSensorSortedListAngles[innerCount] = clusterSensorSortedListAngles[innerCount+1];
                    clusterSensorSortedListSensorId[innerCount] = clusterSensorSortedListSensorId[innerCount+1];
                    clusterSensorSortedListAngles[innerCount+1] = tempDouble;
                    clusterSensorSortedListSensorId[innerCount+1] = tempInt;
                }
            }
            maxSortLit--;
        }
    }


    private void getPerspectiveCentroidFromLiveData(){
//        System.out.printf("\n\r\n\rgetPerspectiveCentroidFromLiveData");
//        System.out.printf(" (numberOfVisibleSensors:%d)", numberOfVisibleSensors);

        Vector2 PtA = new Vector2();
        Vector2 PtB = new Vector2();
        Vector2 PtC = new Vector2();
        
        Vector2 VecBtoA = new Vector2();
        double  AngleBtoA;
        Vector2 PtE = new Vector2();
        Vector2 PtI = new Vector2();
        Vector2 PtF = new Vector2();

        Vector2 VecBtoC = new Vector2();
        double  AngleBtoC;
        Vector2 PtG = new Vector2();
        Vector2 PtJ = new Vector2();
        Vector2 PtH = new Vector2();

        double reflectDistFH = 0;
        Vector2 PtD = new Vector2();
        Vector2 PtFinal = new Vector2();
        
        Vector2 perspectivePointsCentroidVec2 = new Vector2();
        
        // Clear out all the points
        for (int count = 0; count < CLUSTER_SIZE; count++) {
            perspectivePointsFromEstimate[count].r = 0d;
        }

        perspectivePointsCentroid.setFromRads(0d, 0d, 0.0d);

        // Calculate the first sensor in the list
        PtA.set(zeroedRawBaseToSensorSpherical[clusterSensorSortedListSensorId[numberOfVisibleSensors-1]].az, zeroedRawBaseToSensorSpherical[clusterSensorSortedListSensorId[numberOfVisibleSensors-1]].el);
        PtB.set(zeroedRawBaseToSensorSpherical[clusterSensorSortedListSensorId[0]].az,   zeroedRawBaseToSensorSpherical[clusterSensorSortedListSensorId[0]].el);
        PtC.set(zeroedRawBaseToSensorSpherical[clusterSensorSortedListSensorId[1]].az, zeroedRawBaseToSensorSpherical[clusterSensorSortedListSensorId[1]].el);

        // Get offsets A from B
        VecBtoA.set(PtA).sub(PtB);
        AngleBtoA = clusterSensorSortedListAngles[0] - clusterSensorSortedListAngles[numberOfVisibleSensors-1];
        System.out.printf("\n\r %d, AngleBtoA(%2.18f)", 0, Math.toDegrees(AngleBtoA));
        VecBtoA.scale(Math.abs(1.0d/Math.tan(AngleBtoA)));
        PtE.set(VecBtoA).rot90AntiClockwise().add(PtA);
        PtI.set(VecBtoA).rot90Clockwise().add(PtA);
        if (AngleBtoA < (Math.PI/2d)){
            PtF.set(PtE).sub(PtB).scale(0.5d).add(PtB);
        } else {
            PtF.set(PtI).sub(PtB).scale(0.5d).add(PtB);
        }

        // Get offsets C from B
        VecBtoC.set(PtC).sub(PtB);
        AngleBtoC = clusterSensorSortedListAngles[1] - clusterSensorSortedListAngles[0];
        System.out.printf(", AngleBtoC(%2.18f)", Math.toDegrees(AngleBtoC));
        VecBtoC.scale(Math.abs(1.0d/Math.tan(AngleBtoC)));
        PtG.set(VecBtoC).rot90Clockwise().add(PtC);
        PtJ.set(VecBtoC).rot90AntiClockwise().add(PtC);
        if (AngleBtoC < (Math.PI/2d)){
            PtH.set(PtG).sub(PtB).scale(0.5d).add(PtB);
        } else {
            PtH.set(PtJ).sub(PtB).scale(0.5d).add(PtB);
        }

        reflectDistFH = Math.cos(angleThreePointsVector2(PtB, PtF, PtH)) * PtB.distance(PtF);
        PtD.set(PtH).sub(PtF).scale(reflectDistFH/PtF.distance(PtH)).add(PtF);
        PtFinal.set(PtD).sub(PtB).scale(2.0d).add(PtB);
        perspectivePointsFromEstimate[clusterSensorSortedListSensorId[0]].setFromRads(PtFinal.x, PtFinal.y, 1.0d);
        perspectivePointsCentroid.az += PtFinal.x;
        perspectivePointsCentroid.el += PtFinal.y;
        perspectivePointsCentroid.r++;
        System.out.printf(", PtFinal:(%2.18f, %2.18f)", Math.toDegrees(PtFinal.x), Math.toDegrees(PtFinal.y));

        // Run through the middle sensors (skip first and last)
        for (int outerCount = 1; outerCount < numberOfVisibleSensors-1; outerCount++) {
            PtA.set(zeroedRawBaseToSensorSpherical[clusterSensorSortedListSensorId[outerCount-1]].az, zeroedRawBaseToSensorSpherical[clusterSensorSortedListSensorId[outerCount-1]].el);
            PtB.set(zeroedRawBaseToSensorSpherical[clusterSensorSortedListSensorId[outerCount]].az,   zeroedRawBaseToSensorSpherical[clusterSensorSortedListSensorId[outerCount]].el);
            PtC.set(zeroedRawBaseToSensorSpherical[clusterSensorSortedListSensorId[outerCount+1]].az, zeroedRawBaseToSensorSpherical[clusterSensorSortedListSensorId[outerCount+1]].el);
            
            // Get offsets A from B
            VecBtoA.set(PtA).sub(PtB);
            AngleBtoA = clusterSensorSortedListAngles[outerCount] - clusterSensorSortedListAngles[outerCount-1];
            System.out.printf("\n\r %d, AngleBtoA(%2.18f)", outerCount, Math.toDegrees(AngleBtoA));
            VecBtoA.scale(Math.abs(1.0d/Math.tan(AngleBtoA)));
            PtE.set(VecBtoA).rot90AntiClockwise().add(PtA);
            PtI.set(VecBtoA).rot90Clockwise().add(PtA);
            if (AngleBtoA < (Math.PI/2d)){
                PtF.set(PtE).sub(PtB).scale(0.5d).add(PtB);
            } else {
                PtF.set(PtI).sub(PtB).scale(0.5d).add(PtB);
            }

            // Get offsets C from B
            VecBtoC.set(PtC).sub(PtB);
            AngleBtoC = clusterSensorSortedListAngles[outerCount+1] - clusterSensorSortedListAngles[outerCount];
            System.out.printf(", AngleBtoC(%2.18f)", Math.toDegrees(AngleBtoC));
            VecBtoC.scale(Math.abs(1.0d/Math.tan(AngleBtoC)));
            PtG.set(VecBtoC).rot90Clockwise().add(PtC);
            PtJ.set(VecBtoC).rot90AntiClockwise().add(PtC);
            if (AngleBtoC < (Math.PI/2d)){
                PtH.set(PtG).sub(PtB).scale(0.5d).add(PtB);
            } else {
                PtH.set(PtJ).sub(PtB).scale(0.5d).add(PtB);
            }
            
            reflectDistFH = Math.cos(angleThreePointsVector2(PtB, PtF, PtH)) * PtB.distance(PtF);
            PtD.set(PtH).sub(PtF).scale(reflectDistFH/PtF.distance(PtH)).add(PtF);
            PtFinal.set(PtD).sub(PtB).scale(2.0d).add(PtB);
            perspectivePointsFromEstimate[clusterSensorSortedListSensorId[outerCount]].setFromRads(PtFinal.x, PtFinal.y, 1.0d);
            perspectivePointsCentroid.az += PtFinal.x;
            perspectivePointsCentroid.el += PtFinal.y;
            perspectivePointsCentroid.r++;
            System.out.printf(", PtFinal:(%2.18f, %2.18f)", Math.toDegrees(PtFinal.x), Math.toDegrees(PtFinal.y));
        }
        
        // Calculate the last sensor in the list
        PtA.set(zeroedRawBaseToSensorSpherical[clusterSensorSortedListSensorId[numberOfVisibleSensors-2]].az, zeroedRawBaseToSensorSpherical[clusterSensorSortedListSensorId[numberOfVisibleSensors-2]].el);
        PtB.set(zeroedRawBaseToSensorSpherical[clusterSensorSortedListSensorId[numberOfVisibleSensors-1]].az,   zeroedRawBaseToSensorSpherical[clusterSensorSortedListSensorId[numberOfVisibleSensors-1]].el);
        PtC.set(zeroedRawBaseToSensorSpherical[clusterSensorSortedListSensorId[0]].az, zeroedRawBaseToSensorSpherical[clusterSensorSortedListSensorId[0]].el);

        // Get offsets A from B
        VecBtoA.set(PtA).sub(PtB);
        AngleBtoA = clusterSensorSortedListAngles[numberOfVisibleSensors-1] - clusterSensorSortedListAngles[numberOfVisibleSensors-2];
        System.out.printf("\n\r %d, AngleBtoA(%2.18f)", numberOfVisibleSensors-1, Math.toDegrees(AngleBtoA));
        VecBtoA.scale(Math.abs(1.0d/Math.tan(AngleBtoA)));
        PtE.set(VecBtoA).rot90AntiClockwise().add(PtA);
        PtI.set(VecBtoA).rot90Clockwise().add(PtA);
        if (AngleBtoA < (Math.PI/2d)){
            PtF.set(PtE).sub(PtB).scale(0.5d).add(PtB);
        } else {
            PtF.set(PtI).sub(PtB).scale(0.5d).add(PtB);
        }

        // Get offsets C from B
        VecBtoC.set(PtC).sub(PtB);
        AngleBtoC = clusterSensorSortedListAngles[0] - clusterSensorSortedListAngles[numberOfVisibleSensors-1];
        System.out.printf(", AngleBtoC(%2.18f)", Math.toDegrees(AngleBtoC));
        VecBtoC.scale(Math.abs(1.0d/Math.tan(AngleBtoC)));
        PtG.set(VecBtoC).rot90Clockwise().add(PtC);
        PtJ.set(VecBtoC).rot90AntiClockwise().add(PtC);
        if (AngleBtoC < (Math.PI/2d)){
            PtH.set(PtG).sub(PtB).scale(0.5d).add(PtB);
        } else {
            PtH.set(PtJ).sub(PtB).scale(0.5d).add(PtB);
        }

        reflectDistFH = Math.cos(angleThreePointsVector2(PtB, PtF, PtH)) * PtB.distance(PtF);
        PtD.set(PtH).sub(PtF).scale(reflectDistFH/PtF.distance(PtH)).add(PtF);
        PtFinal.set(PtD).sub(PtB).scale(2.0d).add(PtB);
        perspectivePointsFromEstimate[clusterSensorSortedListSensorId[numberOfVisibleSensors-1]].setFromRads(PtFinal.x, PtFinal.y, 1.0d);
        perspectivePointsCentroid.az += PtFinal.x;
        perspectivePointsCentroid.el += PtFinal.y;
        perspectivePointsCentroid.r++;
        System.out.printf(", PtFinal:(%2.18f, %2.18f)", Math.toDegrees(PtFinal.x), Math.toDegrees(PtFinal.y));

        
        perspectivePointsCentroid.az /= perspectivePointsCentroid.r;
        perspectivePointsCentroid.el /= perspectivePointsCentroid.r;
        perspectivePointsCentroid.r = 1d;
        System.out.printf("\n\r perspectivePointsCentroid:(a:%3.18f, e:%3.18f)", Math.toDegrees(perspectivePointsCentroid.az), Math.toDegrees(perspectivePointsCentroid.el));
    }

    


    private void refinePerspectiveCentroid(){
        double[] distanceList = new double[CLUSTER_SIZE];
        double tempAzDist;
        double tempElDist;
        double tempDouble;
        int tempInt = 0;
        int maxSortLit = numberOfVisibleSensors;
        
        
        for (int outerCount = 0; outerCount < numberOfVisibleSensors-1; outerCount++) {
            for (int recalcCount = 0; recalcCount < maxSortLit-1; recalcCount++) {
                tempAzDist = (perspectivePointsCentroid.az - perspectivePointsFromEstimate[clusterSensorSortedListSensorId[recalcCount]].az);
                tempElDist = (perspectivePointsCentroid.el - perspectivePointsFromEstimate[clusterSensorSortedListSensorId[recalcCount]].el);
                distanceList[recalcCount] = Math.sqrt((tempAzDist*tempAzDist)+(tempElDist*tempElDist));
            }
            for (int innerCount = 0; innerCount < maxSortLit-1; innerCount++) {
                if (distanceList[innerCount] > distanceList[innerCount+1]){
                    tempInt = clusterSensorSortedListSensorId[innerCount];
                    tempDouble = distanceList[innerCount];
                    clusterSensorSortedListSensorId[innerCount] = clusterSensorSortedListSensorId[innerCount+1];
                    distanceList[innerCount] = distanceList[innerCount+1];
                    clusterSensorSortedListSensorId[innerCount+1] = tempInt;
                    distanceList[innerCount+1] = tempDouble;
                }
            }
            maxSortLit--;
            for (int innerCount = 0; innerCount < maxSortLit-1; innerCount++) {
            perspectivePointsCentroid.setFromRads(0,0,0);
                perspectivePointsCentroid.az += perspectivePointsFromEstimate[clusterSensorSortedListSensorId[innerCount]].az;
                perspectivePointsCentroid.el += perspectivePointsFromEstimate[clusterSensorSortedListSensorId[innerCount]].el;
                perspectivePointsCentroid.r++;
                perspectivePointsCentroid.az /= perspectivePointsCentroid.r;
                perspectivePointsCentroid.el /= perspectivePointsCentroid.r;
                perspectivePointsCentroid.r = 1d;
            }
        }

        perspectivePointsFromEstimate[clusterSensorSortedListSensorId[numberOfVisibleSensors-1]].setFromRads(PtFinal.x, PtFinal.y, 1.0d);
        perspectivePointsCentroid.az += PtFinal.x;
    }









    double angleFromVerticalVector2(Vector2 inVec) {
        double VecMag = Math.sqrt(inVec.x*inVec.x+inVec.y*inVec.y);
        double VecNormy = inVec.y/VecMag;
        double AngAB = Math.acos(VecNormy);
        if (inVec.x < 0){
            AngAB = (2*Math.PI) - AngAB;
        }
        return AngAB;
    }


    double angleThreePointsVector2(Vector2 Start, Vector2 Centre, Vector2 End) {
        double V1x = Start.x-Centre.x; double V1y = Start.y-Centre.y;
        double V2x = End.x-Centre.x; double V2y = End.y-Centre.y;
        double V1mag = Math.sqrt(V1x*V1x+V1y*V1y);
        double V2mag = Math.sqrt(V2x*V2x+V2y*V2y);
        double V1Normx = V1x/V1mag; double V1Normy = V1y/V1mag;
        double V2Normx = V2x/V2mag; double V2Normy = V2y/V2mag;
        double Vdot = V1Normx*V2Normx+V1Normy*V2Normy;
        double AngAB = Math.acos(Vdot);
        return AngAB;
    }




    
    double angleThreePointsVector3(Vector3 Start, Vector3 Centre, Vector3 End) {
        double V1x = Start.x-Centre.x; double V1y = Start.y-Centre.y; double V1z = Start.z-Centre.z;
        double V2x = End.x-Centre.x; double V2y = End.y-Centre.y; double V2z = End.z-Centre.z;
        double V1mag = Math.sqrt(V1x*V1x+V1y*V1y+V1z*V1z);
        double V2mag = Math.sqrt(V2x*V2x+V2y*V2y+V2z*V2z);
        double V1Normx = V1x/V1mag; double V1Normy = V1y/V1mag; double V1Normz = V1z/V1mag; 
        double V2Normx = V2x/V2mag; double V2Normy = V2y/V2mag; double V2Normz = V2z/V2mag; 
        double Vdot = V1Normx*V2Normx+V1Normy*V2Normy+V1Normz*V2Normz;
        double AngAB = Math.acos(Vdot);
        return AngAB;
    }



    
    private void runApplication() {
        initialiseClassVariables();
//        getLitSensorAngles();
        getCNLohrReadings();
        getInitialBearingFromIcoSphere();
        reviseRangeFromSensorAngles();
        getEstimatedSensorAngles();
        get2dAnglesFromEstimate();
        getPerspectiveCentroidFromLiveData();

        System.out.printf("\n\r\n\r WHITE   : zeroedRawBaseToSensorSpherical");
        System.out.printf("\n\r RED     : bearingEstimate");
        System.out.printf("\n\r YELLO   : zeroedClusterSensorEstimateSpherical");
        System.out.printf("\n\r PINK    : estimatedSensorsCentroid");
        System.out.printf("\n\r MAGENTA : perspectivePointsCentroid");
        System.out.printf("\n\r\n\rDONE.\n\r ");
    }


    public static void main(String[] args) {
        try {
            UIManager.setLookAndFeel("com.sun.java.swing.plaf.windows.WindowsLookAndFeel");
        } catch (UnsupportedLookAndFeelException ex) {
            ex.printStackTrace();
        } catch (IllegalAccessException ex) {
            ex.printStackTrace();
        } catch (InstantiationException ex) {
            ex.printStackTrace();
        } catch (ClassNotFoundException ex) {
            ex.printStackTrace();
        }

        //Schedule a job for the event dispatch thread:
        //creating and showing this application's GUI.
        javax.swing.SwingUtilities.invokeLater(new Runnable() {
            public void run() {
                createAndShowGUI();
            }
        });

    }

    private static void createAndShowGUI() {
        //Create and set up the window.
        JFrame frame = new JFrame("DynamicOutput");
        frame.setDefaultCloseOperation(JFrame.EXIT_ON_CLOSE);
        
        //Create and set up the content pane.
        JComponent newContentPane = new DynamicOutput();
        newContentPane.setOpaque(true); //content panes must be opaque
        frame.setContentPane(newContentPane);
        
        //Display the window.
        frame.pack();
        frame.setVisible(true);
        frameInitialised = true;
    }
    
    public DynamicOutput() {
//        super(new GridLayout(0,1));
        canvasArea = new CanvasArea(Color.black);
        canvasArea.minSize.setSize(GRAPHICS_WIDTH, GRAPHICS_HEIGHT);
        add(canvasArea);
        
        //Register for mouse events on canvasArea and the panel.
        canvasArea.addMouseListener(this);
        addMouseListener(this);

        setPreferredSize(new Dimension(GRAPHICS_WIDTH, GRAPHICS_HEIGHT));
    }
    
    
    public void mouseClicked(MouseEvent e) {
        double rangeScaling = 0d;
        String buttonString;
        double centroidOffsetScale = 3d;
        if (MouseEvent.BUTTON1 == e.getButton()){
            buttonString = "B1(Zm-)";
            outputScale *= 1.5d;
            System.out.printf("\n\r B1(Zm-) %2.5f)", outputScale);
//        } else if (MouseEvent.BUTTON2 == e.getButton()){
//            buttonString = "B1(Cen)";
//            outputCenter.set(canvasArea.getXfromPix(e.getX()), canvasArea.getYfromPix(e.getY()));
//            System.out.append("\n\rMouse " + buttonString + " clicked (# of clicks: " + e.getClickCount() + ")"
//                    + " (" + e.getX() + "," + e.getY() + ")"
//                    + " (" + canvasArea.getXfromPix(e.getX()) + "," + canvasArea.getYfromPix(e.getY()) + ")"
//                    + " detected on "
//                    + e.getComponent().getClass().getName());
        } else if (MouseEvent.BUTTON3 == e.getButton()){
            buttonString = "B3(Zm+)";
            outputScale *= 1d/1.5d;
            System.out.printf("\n\r B3(Zm+) %2.5f)", outputScale);
        } else {
            System.out.printf("\n\r\n\r+++++  RECALCULATING  +++++");
            for (int count = 0; count < 25; count++) {
    //            System.out.printf("\n\r perspectivePointsCentroid0:(%2.18f, %2.18f, %2.18f)", Math.toDegrees(perspectivePointsCentroid.az), Math.toDegrees(perspectivePointsCentroid.el), perspectivePointsCentroid.r);
    //            System.out.printf("\n\r bearingEstimate0:(%2.18f, %2.18f, %2.18f)", Math.toDegrees(bearingEstimate.az), Math.toDegrees(bearingEstimate.el), bearingEstimate.r);
    //            System.out.printf("\n\r vectorEstimate0 :(%2.18f, %2.18f, %2.18f)", vectorEstimate.x, vectorEstimate.y, vectorEstimate.z);
                centroidDelta.set(perspectivePointsCentroid.az, perspectivePointsCentroid.el);
                bearingEstimate.az -= (centroidDelta.x * centroidOffsetScale);
                bearingEstimate.el -= (centroidDelta.y * centroidOffsetScale);
                vectorEstimate.setFromSpherical(bearingEstimate);
                positionEstimate.set(0d,0d,0d);
                positionEstimate.sub(vectorEstimate);

                getEstimatedSensorAngles();
                reviseRangeFromSensorLitAndEstSensorPosn();
                get2dAnglesFromEstimate();
                getPerspectiveCentroidFromLiveData();
            }
        }
        refreshCanvas();
        canvasArea.circleAtScaled(0, 0, Color.BLUE, 5);
    }

    private void refreshCanvas() {
//        System.out.printf("\n\r\n\r *** REFRESH:");
//        System.out.printf("\n\r perspectivePointsCentroid:(%2.18f, %2.18f)", Math.toDegrees(perspectivePointsCentroid.az), Math.toDegrees(perspectivePointsCentroid.el));
//        System.out.printf("\n\r bearingEstimate9:(%2.18f, %2.18f, %2.18f)", Math.toDegrees(bearingEstimate.az), Math.toDegrees(bearingEstimate.el), bearingEstimate.r);
//        System.out.printf("\n\r vectorEstimate9 :(%2.18f, %2.18f, %2.18f)", vectorEstimate.x, vectorEstimate.y, vectorEstimate.z);

        canvasArea.rescaleWindow(outputScale);
        canvasArea.recenterWindow(0, 0 );
        canvasArea.clearWindow();

        centroidDelta.set(perspectivePointsCentroid.az, perspectivePointsCentroid.el).sub(bearingEstimate.az, bearingEstimate.el);

        // Actual and estimated bearing to device center
//        canvasArea.circleAtScaled(Math.toDegrees(actualBearingFromBase.az), Math.toDegrees(actualBearingFromBase.el), Color.BLUE, 4);
        canvasArea.circleAtScaled(Math.toDegrees(bearingEstimate.az), Math.toDegrees(bearingEstimate.el), Color.RED, 4);

//        canvasArea.circleAtScaled(Math.toDegrees(rawSensorsCentroid.az), Math.toDegrees(rawSensorsCentroid.el), Color.CYAN, 4);
        canvasArea.circleAtScaled(Math.toDegrees(estimatedSensorsCentroid.az), Math.toDegrees(estimatedSensorsCentroid.el), Color.PINK, 4);

        canvasArea.circleAtScaled(Math.toDegrees(perspectivePointsCentroid.az), Math.toDegrees(perspectivePointsCentroid.el), Color.MAGENTA, 4);

        // Output actual sensor angles
        for (int count = 0; count < CLUSTER_SIZE; count++) {
            if (base1ToSensorVisible[count]) {
//                System.out.printf("\n\r   %d: ", count);
//                System.out.printf("(%3.18f, %3.18f, %2.10f), ", zeroedRawBaseToSensorSpherical[count].az, zeroedRawBaseToSensorSpherical[count].el, zeroedRawBaseToSensorSpherical[count].r);
                canvasArea.circleAtScaled(Math.toDegrees(zeroedRawBaseToSensorSpherical[count].az), Math.toDegrees(zeroedRawBaseToSensorSpherical[count].el), Color.WHITE, 4);
//                System.out.printf("(%3.18f, %3.18f, %2.10f), ", zeroedClusterSensorEstimateSpherical[count].az, zeroedClusterSensorEstimateSpherical[count].el, zeroedClusterSensorEstimateSpherical[count].r);
                canvasArea.circleAtScaled(Math.toDegrees(zeroedClusterSensorEstimateSpherical[count].az), Math.toDegrees(zeroedClusterSensorEstimateSpherical[count].el), Color.YELLOW, 4);
                if (perspectivePointsFromEstimate[count].r > 0.5d) {
                    canvasArea.circleAtScaled(Math.toDegrees(perspectivePointsFromEstimate[count].az), Math.toDegrees(perspectivePointsFromEstimate[count].el), Color.GREEN, 4);
//                    canvasArea.lineBetweenScaled(Math.toDegrees(zeroedRawBaseToSensorSpherical[count].az), Math.toDegrees(zeroedRawBaseToSensorSpherical[count].el), Math.toDegrees(perspectivePointsFromEstimate[count].az), Math.toDegrees(perspectivePointsFromEstimate[count].el), Color.WHITE);
                }
            }
        }
    }


    public void mouseEntered(MouseEvent e) {
        if ((false == appInitialised)&&frameInitialised) {
            appInitialised = true;
            runApplication();
        }
        refreshCanvas();
    }
    
    // Add these for the listener implementation
    public void mousePressed(MouseEvent e)  {dummy++;}
    public void mouseReleased(MouseEvent e)  {dummy++;}
    public void mouseExited(MouseEvent e) {dummy++;}


    static double[][] devicePoints = {
        {0.08518143743276596,0.017062144353985786,0.04640356823801994},
        {0.09299874305725098,-9.77110757958144e-05,0.03490303456783295},
        {0.0866357758641243,0.016550032421946526,0.020586593076586723},
        {0.0896136462688446,0.029156366363167763,0.0296088345348835},
        {0.07996707409620285,0.04522520303726196,0.03478708118200302},
        {0.05082200840115547,0.0525379441678524,0.03328508138656616},
        {0.02431630529463291,0.0200039092451334,0.05943312123417854},
        {0.04736604541540146,0.03358921408653259,0.05357927456498146},
        {0.04778143763542175,-0.034000154584646225,0.05348391830921173},
        {0.05795735865831375,-3.651010774774477e-05,0.05651696398854256},
        {0.02757195383310318,-0.051707036793231964,0.046649035066366196},
        {0.05145823583006859,-0.05293474718928337,0.03312348574399948},
        {0.08054577559232712,-0.04522349312901497,0.03467874228954315},
        {0.08995519578456879,-0.029309064149856567,0.02968563325703144},
        {0.0868583470582962,-0.016645202413201332,0.020546138286590576},
        {0.08528480678796768,-0.01717553101480007,0.04645363613963127},
        {-0.04789695516228676,0.03364776074886322,0.05359702929854393},
        {-0.02451552450656891,0.020054345950484276,0.05939493328332901},
        {-0.05120636895298958,0.05282235145568848,0.033184923231601715},
        {-0.08026022464036942,0.045291341841220856,0.034813448786735535},
        {-0.08975434303283691,0.02939225733280182,0.029623806476593018},
        {-0.08676112443208694,0.01667257957160473,0.02072800137102604},
        {-0.09296955168247223,0.00019559808424673975,0.034909311681985855},
        {-0.08538919687271118,0.01735016517341137,0.046313270926475525},
        {-0.08526882529258728,-0.017100226134061813,0.046251364052295685},
        {-0.08669501543045044,-0.016456371173262596,0.020705312490463257},
        {-0.08958882093429565,-0.0292942076921463,0.029727233573794365},
        {-0.08019855618476868,-0.04522521793842316,0.0346868671476841},
        {-0.05091847851872444,-0.052784282714128494,0.03316209465265274},
        {-0.027258513495326042,-0.051615241914987564,0.04688679054379463},
        {-0.0580756776034832,6.801447852922138e-06,0.05650037154555321},
        {-0.047557104378938675,-0.03394269943237305,0.0535212866961956}
        };
    static double[][] deviceNormals = {
        {0.6565292477607727,0.08003702759742737,0.7500423192977905},
        {1,0,0},
        {0.9510334134101868,0.1922958791255951,-0.24198685586452484},
        {0.8633409738540649,0.26114100217819214,-0.43179601430892944},
        {0.5620832443237305,0.8270804286003113,-0.0007020003395155072},
        {0.5567418932914734,0.8186168074607849,-0.1410849690437317},
        {0.12751400470733643,0.36096900701522827,0.9238190054893494},
        {0.19732795655727386,0.7212077975273132,0.6640188097953796},
        {0.19732792675495148,-0.7205037474632263,0.6647827625274658},
        {0.4602000117301941,0.003066000062972307,0.8878099918365479},
        {0.025263000279664993,-0.7483329772949219,0.6628419756889343},
        {0.5567419528961182,-0.8187658786773682,-0.14021699130535126},
        {0.5620829463005066,-0.8270809054374695,0.00017499997920822352},
        {0.8633410930633545,-0.26159802079200745,-0.4315190613269806},
        {0.9510335326194763,-0.19255191087722778,-0.24178287386894226},
        {0.6565289497375488,-0.07924199104309082,0.7501268982887268},
        {-0.19732795655727386,0.7212077975273132,0.6640188097953796},
        {-0.12751400470733643,0.36096900701522827,0.9238190054893494},
        {-0.5567418932914734,0.8186168074607849,-0.1410849690437317},
        {-0.5620832443237305,0.8270804286003113,-0.0007020003395155072},
        {-0.8633409738540649,0.26114100217819214,-0.43179601430892944},
        {-0.9510334134101868,0.1922958791255951,-0.24198685586452484},
        {-1,0,0},
        {-0.6565292477607727,0.08003702759742737,0.7500423192977905},
        {-0.6565289497375488,-0.07924199104309082,0.7501268982887268},
        {-0.9510335326194763,-0.19255191087722778,-0.24178287386894226},
        {-0.8633410930633545,-0.26159802079200745,-0.4315190613269806},
        {-0.5620829463005066,-0.8270809054374695,0.00017499997920822352},
        {-0.5567419528961182,-0.8187658786773682,-0.14021699130535126},
        {-0.025263000279664993,-0.7483329772949219,0.6628419756889343},
        {-0.4602000117301941,0.003066000062972307,0.8878099918365479},
        {-0.19732792675495148,-0.7205037474632263,0.6647827625274658}
        };
    
    static int ICOSPHERE_POINTS = 320;
    static double[][] icoSphereCoords = {
    {0.048151, -0.14819, -0.987786},
    {0.722992, -0.412071, -0.554509},
    {-0.12606, -0.091587, -0.987786},
    {-0.12606, 0.091587, -0.987786},
    {0.048151, 0.14819, -0.987786},
    {0.819296, -0.412071, -0.398687},
    {-0.138726, -0.906533, -0.398689},
    {-0.905035, -0.148193, -0.398685},
    {-0.420608, 0.814945, -0.39869},
    {0.645086, 0.651853, -0.398688},
    {0.771146, -0.560264, -0.302388},
    {-0.294548, -0.906533, -0.302388},
    {-0.953185, 0, -0.302386},
    {-0.294548, 0.906533, -0.302389},
    {0.771146, 0.560265, -0.302387},
    {0.342697, -0.758339, 0.554509},
    {-0.615326, -0.560263, 0.554509},
    {-0.722992, 0.412072, 0.554509},
    {0.168486, 0.814943, 0.554509},
    {0.827125, 0.091587, 0.554505},
    {0.12606, 0.091587, 0.987786},
    {0.391194, 0.09145, 0.915753},
    {0.644124, 0.091451, 0.759434},
    {0.233376, 0.169558, 0.957489},
    {0.207859, 0.343786, 0.915754},
    {0.516845, 0.183046, 0.836281},
    {0.49112, 0.35682, 0.794658},
    {0.333798, 0.434985, 0.836282},
    {0.286017, 0.584336, 0.759438},
    {0.752031, 0.16956, 0.636945},
    {0.726114, 0.343786, 0.595458},
    {0.598705, 0.434986, 0.672561},
    {0.551342, 0.584336, 0.59546},
    {0.393647, 0.662827, 0.636948},
    {0.342697, 0.758339, 0.554509},
    {-0.048151, 0.148191, 0.987786},
    {0.033908, 0.400305, 0.915754},
    {0.112066, 0.640855, 0.759438},
    {-0.089143, 0.274349, 0.957489},
    {-0.262729, 0.30392, 0.915754},
    {-0.014375, 0.548111, 0.836282},
    {-0.187594, 0.577343, 0.794659},
    {-0.310547, 0.451877, 0.836282},
    {-0.467354, 0.452586, 0.759438},
    {0.071126, 0.767619, 0.636948},
    {-0.102585, 0.796808, 0.595461},
    {-0.228689, 0.703819, 0.672562},
    {-0.385367, 0.704925, 0.59546},
    {-0.508743, 0.579205, 0.636947},
    {-0.615326, 0.560263, 0.554509},
    {-0.155816, 0, 0.987786},
    {-0.370234, 0.155951, 0.915754},
    {-0.574859, 0.304617, 0.759438},
    {-0.28847, 0, 0.957489},
    {-0.370234, -0.155951, 0.915754},
    {-0.525729, 0.155706, 0.836281},
    {-0.607057, 0, 0.794658},
    {-0.525729, -0.155706, 0.836281},
    {-0.574859, -0.304617, 0.759438},
    {-0.708071, 0.304853, 0.636946},
    {-0.78951, 0.148666, 0.59546},
    {-0.740043, 0, 0.67256},
    {-0.78951, -0.148666, 0.59546},
    {-0.708071, -0.304853, 0.636946},
    {-0.722992, -0.412072, 0.554509},
    {-0.048151, -0.148191, 0.987786},
    {-0.262729, -0.30392, 0.915754},
    {-0.467354, -0.452586, 0.759438},
    {-0.089143, -0.274349, 0.957489},
    {0.033908, -0.400305, 0.915754},
    {-0.310547, -0.451877, 0.836282},
    {-0.187593, -0.577344, 0.794659},
    {-0.014375, -0.548111, 0.836282},
    {0.112066, -0.640855, 0.759438},
    {-0.508743, -0.579205, 0.636947},
    {-0.385367, -0.704925, 0.59546},
    {-0.228689, -0.703819, 0.672562},
    {-0.102585, -0.796808, 0.595461},
    {0.071125, -0.767619, 0.636948},
    {0.168486, -0.814943, 0.554509},
    {0.12606, -0.091587, 0.987786},
    {0.207859, -0.343786, 0.915754},
    {0.286017, -0.584336, 0.759438},
    {0.233376, -0.169558, 0.957489},
    {0.391194, -0.09145, 0.915753},
    {0.333798, -0.434985, 0.836282},
    {0.49112, -0.35682, 0.794658},
    {0.516845, -0.183046, 0.836281},
    {0.644124, -0.091451, 0.759434},
    {0.393647, -0.662827, 0.636948},
    {0.551342, -0.584336, 0.59546},
    {0.598705, -0.434986, 0.672561},
    {0.726114, -0.343786, 0.595458},
    {0.752031, -0.16956, 0.636945},
    {0.827126, -0.091587, 0.554505},
    {0.905035, 0.148193, 0.398685},
    {0.936571, 0.303922, 0.174544},
    {0.888267, 0.45259, -0.078388},
    {0.896269, 0.274349, 0.348473},
    {0.803908, 0.400308, 0.439871},
    {0.886873, 0.45188, 0.096235},
    {0.794655, 0.577349, 0.187594},
    {0.754418, 0.548114, 0.361145},
    {0.629137, 0.640858, 0.439872},
    {0.797217, 0.579207, -0.170187},
    {0.704932, 0.70493, -0.078388},
    {0.703826, 0.703824, 0.096235},
    {0.578466, 0.796813, 0.174546},
    {0.537886, 0.767622, 0.348475},
    {0.420608, 0.814945, 0.39869},
    {0.138726, 0.906533, 0.398689},
    {0.000363, 0.984649, 0.174545},
    {-0.155953, 0.984649, -0.078389},
    {0.016035, 0.93718, 0.348476},
    {-0.1323, 0.888262, 0.439872},
    {-0.155708, 0.983104, 0.096236},
    {-0.303533, 0.934171, 0.187594},
    {-0.288163, 0.886868, 0.361146},
    {-0.415083, 0.796378, 0.439872},
    {-0.304507, 0.937183, -0.170186},
    {-0.452594, 0.888265, -0.078389},
    {-0.451883, 0.886871, 0.096236},
    {-0.579061, 0.796381, 0.174546},
    {-0.563838, 0.748767, 0.348476},
    {-0.645086, 0.651853, 0.398689},
    {-0.819296, 0.412071, 0.398687},
    {-0.936345, 0.304618, 0.174544},
    {-0.98465, 0.155951, -0.078388},
    {-0.886357, 0.304854, 0.348476},
    {-0.885671, 0.148666, 0.43987},
    {-0.983104, 0.155706, 0.096236},
    {-0.982247, -0.000001, 0.187592},
    {-0.932509, 0, 0.361146},
    {-0.885671, -0.148666, 0.43987},
    {-0.985412, 0, -0.170183},
    {-0.984649, -0.155951, -0.078388},
    {-0.983104, -0.155707, 0.096237},
    {-0.936345, -0.304618, 0.174544},
    {-0.886357, -0.304854, 0.348476},
    {-0.819296, -0.412072, 0.398687},
    {-0.645086, -0.651853, 0.398689},
    {-0.579061, -0.796381, 0.174546},
    {-0.452594, -0.888265, -0.078389},
    {-0.563838, -0.748767, 0.348476},
    {-0.415083, -0.796379, 0.439871},
    {-0.451883, -0.886871, 0.096236},
    {-0.303534, -0.934171, 0.187593},
    {-0.288164, -0.886868, 0.361146},
    {-0.1323, -0.888262, 0.439871},
    {-0.304507, -0.937183, -0.170186},
    {-0.155953, -0.984649, -0.078388},
    {-0.155709, -0.983104, 0.096235},
    {0.000363, -0.984649, 0.174545},
    {0.016034, -0.937181, 0.348476},
    {0.138726, -0.906533, 0.398689},
    {0.420608, -0.814945, 0.398689},
    {0.578465, -0.796814, 0.174546},
    {0.704932, -0.70493, -0.078389},
    {0.537887, -0.767622, 0.348475},
    {0.629137, -0.640859, 0.439871},
    {0.703826, -0.703823, 0.096234},
    {0.794655, -0.577349, 0.187592},
    {0.754418, -0.548114, 0.361144},
    {0.803908, -0.400308, 0.439869},
    {0.797217, -0.579207, -0.170188},
    {0.888267, -0.45259, -0.078388},
    {0.886873, -0.451881, 0.096233},
    {0.936571, -0.303923, 0.174544},
    {0.896269, -0.27435, 0.348473},
    {0.905035, -0.148193, 0.398685},
    {0.294548, 0.906533, 0.302389},
    {0.452594, 0.888265, 0.078389},
    {0.579061, 0.796381, -0.174545},
    {0.304507, 0.937183, 0.170186},
    {0.155953, 0.984649, 0.078388},
    {0.451884, 0.886871, -0.096236},
    {0.303534, 0.934171, -0.187594},
    {0.155708, 0.983104, -0.096236},
    {-0.000363, 0.984649, -0.174545},
    {0.563838, 0.748766, -0.348476},
    {0.415083, 0.796378, -0.439872},
    {0.288163, 0.886868, -0.361147},
    {0.1323, 0.888262, -0.439872},
    {-0.016035, 0.93718, -0.348476},
    {-0.138726, 0.906533, -0.398689},
    {-0.771146, 0.560265, 0.302387},
    {-0.704932, 0.70493, 0.078389},
    {-0.578465, 0.796814, -0.174546},
    {-0.797217, 0.579207, 0.170187},
    {-0.888267, 0.45259, 0.078388},
    {-0.703826, 0.703824, -0.096235},
    {-0.794654, 0.57735, -0.187594},
    {-0.886873, 0.45188, -0.096234},
    {-0.93657, 0.303924, -0.174545},
    {-0.537885, 0.767622, -0.348476},
    {-0.629137, 0.640858, -0.439872},
    {-0.754417, 0.548115, -0.361144},
    {-0.803908, 0.400308, -0.43987},
    {-0.896269, 0.274351, -0.348473},
    {-0.905035, 0.148193, -0.398685},
    {-0.771146, -0.560265, 0.302387},
    {-0.888267, -0.45259, 0.078388},
    {-0.936571, -0.303924, -0.174545},
    {-0.797217, -0.579207, 0.170187},
    {-0.704932, -0.70493, 0.078389},
    {-0.886873, -0.45188, -0.096234},
    {-0.794654, -0.57735, -0.187594},
    {-0.703826, -0.703824, -0.096235},
    {-0.578465, -0.796814, -0.174546},
    {-0.896269, -0.274351, -0.348473},
    {-0.803908, -0.400308, -0.439871},
    {-0.754417, -0.548115, -0.361145},
    {-0.629137, -0.640858, -0.439872},
    {-0.537885, -0.767622, -0.348476},
    {-0.420608, -0.814945, -0.398689},
    {0.294548, -0.906533, 0.302388},
    {0.155953, -0.984649, 0.078388},
    {-0.000363, -0.984649, -0.174545},
    {0.304507, -0.937183, 0.170186},
    {0.452594, -0.888265, 0.078389},
    {0.155708, -0.983104, -0.096236},
    {0.303534, -0.934171, -0.187594},
    {0.451884, -0.886871, -0.096236},
    {0.579061, -0.796381, -0.174545},
    {-0.016035, -0.937181, -0.348476},
    {0.1323, -0.888262, -0.439872},
    {0.288164, -0.886868, -0.361147},
    {0.415083, -0.796378, -0.439872},
    {0.563838, -0.748766, -0.348476},
    {0.645086, -0.651853, -0.398689},
    {0.953186, 0, 0.302386},
    {0.984649, -0.155951, 0.078389},
    {0.936345, -0.304618, -0.174544},
    {0.985412, 0, 0.170184},
    {0.984649, 0.155951, 0.078389},
    {0.983104, -0.155706, -0.096236},
    {0.982247, 0, -0.187592},
    {0.983104, 0.155706, -0.096236},
    {0.936345, 0.304618, -0.174544},
    {0.886357, -0.304854, -0.348476},
    {0.885671, -0.148666, -0.43987},
    {0.932509, 0, -0.361146},
    {0.885671, 0.148666, -0.43987},
    {0.886357, 0.304854, -0.348476},
    {0.819296, 0.412071, -0.398687},
    {0.615326, 0.560263, -0.554509},
    {0.467353, 0.452587, -0.759438},
    {0.262728, 0.303921, -0.915754},
    {0.508743, 0.579205, -0.636947},
    {0.385368, 0.704925, -0.59546},
    {0.310547, 0.451878, -0.836282},
    {0.187593, 0.577345, -0.794658},
    {0.228688, 0.70382, -0.672562},
    {0.102585, 0.796808, -0.59546},
    {0.089144, 0.27435, -0.957489},
    {-0.033908, 0.400305, -0.915754},
    {0.014375, 0.548113, -0.836281},
    {-0.112066, 0.640855, -0.759438},
    {-0.071126, 0.767619, -0.636947},
    {-0.168486, 0.814943, -0.554509},
    {-0.342697, 0.758339, -0.554509},
    {-0.286018, 0.584335, -0.759438},
    {-0.20786, 0.343785, -0.915754},
    {-0.393648, 0.662826, -0.636948},
    {-0.551342, 0.584336, -0.59546},
    {-0.333798, 0.434984, -0.836282},
    {-0.491122, 0.356818, -0.794657},
    {-0.598706, 0.434985, -0.672561},
    {-0.726114, 0.343786, -0.595458},
    {-0.233376, 0.169558, -0.957489},
    {-0.391194, 0.09145, -0.915753},
    {-0.516846, 0.183045, -0.83628},
    {-0.644125, 0.09145, -0.759434},
    {-0.752031, 0.169559, -0.636945},
    {-0.827125, 0.091587, -0.554505},
    {-0.827125, -0.091587, -0.554505},
    {-0.644124, -0.091451, -0.759434},
    {-0.391194, -0.091451, -0.915753},
    {-0.752031, -0.169559, -0.636945},
    {-0.726114, -0.343786, -0.595458},
    {-0.516846, -0.183046, -0.83628},
    {-0.491122, -0.35682, -0.794656},
    {-0.598706, -0.434986, -0.672561},
    {-0.551343, -0.584337, -0.595459},
    {-0.233377, -0.169558, -0.957489},
    {-0.20786, -0.343785, -0.915754},
    {-0.333798, -0.434986, -0.836281},
    {-0.286018, -0.584335, -0.759438},
    {-0.393647, -0.662827, -0.636947},
    {-0.342697, -0.758339, -0.554509},
    {0.722992, 0.412072, -0.554509},
    {0.78951, 0.148666, -0.59546},
    {0.78951, -0.148666, -0.595459},
    {0.708071, 0.304853, -0.636946},
    {0.574859, 0.304617, -0.759438},
    {0.740043, 0, -0.67256},
    {0.607058, 0, -0.794658},
    {0.525729, 0.155706, -0.836281},
    {0.370234, 0.15595, -0.915754},
    {0.708071, -0.304854, -0.636946},
    {0.574859, -0.304617, -0.759438},
    {0.525729, -0.155706, -0.83628},
    {0.370234, -0.15595, -0.915754},
    {0.288471, 0, -0.957489},
    {0.155817, 0, -0.987786},
    {-0.168486, -0.814943, -0.554509},
    {-0.112065, -0.640855, -0.759438},
    {-0.033907, -0.400305, -0.915754},
    {-0.071125, -0.767619, -0.636948},
    {0.102585, -0.796808, -0.595461},
    {0.014375, -0.548112, -0.836282},
    {0.187595, -0.577345, -0.794658},
    {0.228689, -0.703819, -0.672562},
    {0.385367, -0.704925, -0.59546},
    {0.089144, -0.27435, -0.957489},
    {0.262729, -0.30392, -0.915754},
    {0.310548, -0.451878, -0.836281},
    {0.467353, -0.452587, -0.759438},
    {0.508744, -0.579205, -0.636947},
    {0.615326, -0.560263, -0.554509}};


    static double[][] cnlohrHMDReadings = {
    {0.0, 0.0},
    {0.0, 0.0},
    {0.0, 0.0},
    {0.0, 0.0},
    {0.0, 0.0},
    {0.0, 0.0},
    {0.0, 0.0},
    {0.0, 0.0},
    {81.84643427, 91.50531148},
    {82.31771155, 91.63383504},
    {81.4962762, 91.18193372},
    {81.40893373, 91.61839361},
    {81.56903293, 92.13439535},
    {0.0, 0.0},
    {0.0, 0.0},
    {82.07521818, 92.15180248},
    {0.0, 0.0},
    {0.0, 0.0},
    {0.0, 0.0},
    {0.0, 0.0},
    {0.0, 0.0},
    {0.0, 0.0},
    {0.0, 0.0},
    {0.0, 0.0},
    {0.0, 0.0},
    {0.0, 0.0},
    {0.0, 0.0},
    {81.12127192, 89.29136365},
    {0.0, 0.0},
    {81.34157045, 90.20862003},
    {81.99500414, 89.59022408},
    {81.58244065, 89.82019964}};
}
