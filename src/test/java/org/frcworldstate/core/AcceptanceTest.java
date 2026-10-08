package org.frcworldstate.core;

public final class AcceptanceTest {
    private AcceptanceTest() {}
    public static void main(String[] args) throws Exception {
        TrackerWorkerTest.run();
        EnvelopeTest.run();
        PickupSafetyTest.run();
        PredictionContractTest.main(new String[0]);
        System.out.println("All pure-Java acceptance suites passed (CPU synthetic; no hardware evidence).");
    }
}
