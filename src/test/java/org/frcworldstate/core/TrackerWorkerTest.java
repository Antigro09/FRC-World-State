package org.frcworldstate.core;

import java.util.*;
import java.util.concurrent.*;
import static org.frcworldstate.core.Geometry.*;
import static org.frcworldstate.core.World.*;

public final class TrackerWorkerTest {
    private static int checks;
    private static void check(boolean b,String m){checks++;if(!b)throw new AssertionError(m);}
    private static void near(double a,double b){check(Math.abs(a-b)<1e-8,"expected "+b+" actual "+a);}
    static EgoState ego(long us,double x,double y,double heading){return new EgoState(us,new Pose2(new Vec2(x,y),heading),Velocity2.zero(),new Uncertainty(.01,0,.01),true);}
    static final FieldIdentity FIELD=new FieldIdentity("offseason-2026","synthetic-10x10","test-v1");
    static WorldEngine engine(){return engine(false);}
    static WorldEngine engine(boolean velocity){return new WorldEngine(new WorldEngine.Config(8,16,3,500_000,700_000,20_000,30_000,.8,.5,.25,.01,4,2,.02,10,.05,3,.2,.01,velocity),new PoseHistory(20,2_000_000),FIELD);}
    static ObservationFrame frame(long epoch,String source,String boot,String group,long seq,long capture,long pub,double x,double y){
        return new ObservationFrame(epoch,new SourceStamp(source,boot,"cal1","mount1",group,seq,capture,pub,0),List.of(new Observation("det-"+seq,"piece",.9,new Vec2(x,y),new Uncertainty(.01,0,.01))));
    }
    public static void run() throws Exception {
        Pose2 red=allianceToBlue(new Pose2(new Vec2(1,2),0),true,10,8);near(red.position().x(),9);near(red.position().y(),6);near(Math.abs(red.headingRad()),Math.PI);
        Pose2 pose=new Pose2(new Vec2(3,4),Math.PI/2);Vec2 p=pose.toField(new Vec2(2,1));near(p.x(),2);near(p.y(),6);near(pose.toRobot(p).x(),2);
        check(TimeDomains.nt2026MetadataUs(1234567)==1234567,"2026 us");check(TimeDomains.nt2027Alpha7MetadataToUs(1234567999)==1234567,"2027 floor ns");
        check(TimeDomains.jsonUsToNs(1234567)==1234567000L,"JSON remains us");
        boolean badCovariance=false;try{new Uncertainty(0,1e-7,0);}catch(IllegalArgumentException ex){badCovariance=true;}check(badCovariance,"non-PSD near-zero covariance rejected");
        boolean overflow=false;try{TimeDomains.jsonUsToNs(Long.MAX_VALUE);}catch(ArithmeticException e){overflow=true;}check(overflow,"checked overflow");
        PoseHistory history=new PoseHistory(3,200_000);history.add(ego(100_000,0,0,Math.toRadians(179)));history.add(ego(200_000,2,0,Math.toRadians(-179)));
        near(history.sample(150_000).orElseThrow().fieldPose().position().x(),1);near(Math.abs(history.sample(150_000).orElseThrow().fieldPose().headingRad()),Math.PI);
        check(history.sample(99_999).isEmpty()&&history.sample(200_001).isEmpty(),"explicit bounds reject clamp");
        history.add(ego(300_000,3,0,0));history.add(ego(400_000,4,0,0));check(history.sample(100_000).isEmpty(),"history capacity");
        WorldEngine e=engine();e.addEgo(ego(100_000,1,2,Math.PI/2));e.addEgo(ego(200_000,3,2,Math.PI/2));
        var f=frame(0,"cam","boot","group",1,150_000,200_000,1,0);
        check(e.accept(f,200_000).status()==WorldEngine.FrameStatus.ACCEPTED,"capture accepted");
        var s=e.publish(ego(200_000,3,2,Math.PI/2));near(s.tracks().get(0).positionM().x(),2);near(s.tracks().get(0).positionM().y(),3);
        check(s.tracks().get(0).lifecycle()==TrackLifecycle.TENTATIVE&&s.tracks().get(0).kind()==EstimateKind.PROPAGATED,"propagation cannot confirm a tentative track");
        check(e.accept(f,200_000).status()==WorldEngine.FrameStatus.DUPLICATE,"duplicate");
        check(e.accept(frame(0,"cam","boot","group",0,160_000,200_000,1,0),200_000).status()==WorldEngine.FrameStatus.OUT_OF_ORDER,"sequence out of order");
        check(e.accept(frame(0,"cam","boot","group",2,201_000,201_000,1,0),200_000).status()==WorldEngine.FrameStatus.FUTURE,"future capture");
        check(e.accept(frame(0,"other","boot","other",1,90_000,200_000,1,0),200_000).status()==WorldEngine.FrameStatus.HISTORY_BOUNDS,"history bounds");
        e.addEgo(ego(250_000,3,2,Math.PI/2));
        var correlated=frame(0,"cam2","boot","group",1,160_000,200_000,1,0);
        // Match capture-time field location (ego interpolation at160k => x2.2).
        check(e.accept(correlated,200_000).acceptedMeasurements()==0,"correlated camera not independent");
        e.addEgo(ego(300_000,3,2,Math.PI/2));
        check(e.accept(frame(0,"cam","boot","group",2,250_000,300_000,1,.8),300_000).acceptedMeasurements()==1,"identity continues despite detection ID change");
        s=e.publish(ego(300_000,3,2,Math.PI/2));check(s.tracks().size()==1,"no identity switch duplicate");
        long id=s.tracks().get(0).id();near(s.tracks().get(0).velocityMps().norm(),0); // ego pose/velocity disagreement suppresses false inferred object motion
        WorldEngine moving=engine(true);moving.addEgo(ego(100_000,0,0,0));moving.addEgo(ego(200_000,0,0,0));
        moving.accept(frame(0,"c","b","g",1,100_000,200_000,1,1),200_000);
        moving.accept(frame(0,"c","b","g",2,200_000,200_000,1.2,1),200_000);
        near(moving.publish(ego(200_000,0,0,0)).tracks().get(0).velocityMps().x(),2);
        e.correctPoses(q->new Pose2(q.position().add(new Vec2(5,0)),q.headingRad()),300_000);
        s=e.publish(ego(300_000,8,2,Math.PI/2));check(s.tracks().get(0).id()==id,"correction retains identity");near(s.tracks().get(0).velocityMps().norm(),0);near(s.tracks().get(0).positionM().x(),7.2);
        var empty=new ObservationFrame(0,new SourceStamp("cam","boot","cal1","mount1","group",3,300_000,300_000,0),List.of());
        check(e.accept(empty,300_000).acceptedMeasurements()==0,"empty frame");check(e.publish(ego(350_000,8,2,Math.PI/2)).tracks().size()==1,"empty/occluded not free");
        check(e.publish(ego(950_001,8,2,Math.PI/2)).tracks().isEmpty(),"track expiry");check(e.history().size()==3,"bounded snapshots");
        var restart=frame(0,"cam","newboot","group",0,950_001,950_001,1,0);
        check(e.accept(restart,950_001).status()==WorldEngine.FrameStatus.SOURCE_RESET&&e.epoch()==1,"source restart resets epoch");
        check(e.accept(f,950_001).status()==WorldEngine.FrameStatus.WRONG_EPOCH,"old epoch rejected");
        e.reset(ResetReason.LOCALIZATION_HARD_RESET,FIELD,true,1_000_000);check(e.poseHistory().retained().isEmpty()&&e.history().isEmpty(),"hard reset clears affected histories");
        check(e.rawHistory().isEmpty(),"raw history reset");
        var malformed=new Observation("bad","piece",.1,new Vec2(1,1),new Uncertainty(.01,0,.01));
        WorldEngine quality=engine();quality.addEgo(ego(100_000,0,0,0));quality.addEgo(ego(200_000,0,0,0));
        check(quality.accept(new ObservationFrame(0,f.stamp(),List.of(malformed)),200_000).acceptedMeasurements()==0,"quality gate");
        check(replay().equals(replay()),"deterministic replay IDs and decisions");
        extraTrackerCases();workers();plannerValidation();System.out.println("TrackerWorkerTest: "+checks+" checks passed");
    }
    private static List<WorldSnapshot> replay(){WorldEngine e=engine();e.addEgo(ego(100_000,0,0,0));e.addEgo(ego(200_000,0,0,0));e.accept(frame(0,"c","b","g",1,100_000,200_000,1,1),200_000);e.publish(ego(200_000,0,0,0));e.publish(ego(250_000,0,0,0));return e.history();}
    private static void extraTrackerCases() {
        WorldEngine e=engine();e.addEgo(ego(100_000,0,0,0));e.addEgo(ego(200_000,0,0,0));e.addEgo(ego(300_000,0,0,0));
        e.accept(frame(0,"a","b","g1",1,100_000,100_000,1,1),100_000);
        e.accept(frame(0,"b","b","g2",1,110_000,110_000,1,1),110_000);
        check(e.accept(frame(0,"a","b","g1",2,120_000,120_000,1,1),120_000).acceptedMeasurements()==0,"per-group correlation history survives interleaving");
        check(e.accept(frame(0,"a","b","g1",3,200_000,200_000,1.2,1),200_000).acceptedMeasurements()==1,"uncorrelated later measurement");
        check(e.accept(frame(0,"delayed","b","g3",1,100_000,300_000,1,1),300_000).status()==WorldEngine.FrameStatus.OUT_OF_ORDER,"global delayed camera rejected");
        check(e.accept(frame(0,"future-pub","b","g3",1,250_000,301_000,1,1),300_000).status()==WorldEngine.FrameStatus.FUTURE,"publication distinct future check");
        check(e.accept(frame(0,"stale","b","g4",1,100_000,700_001,1,1),700_001).status()==WorldEngine.FrameStatus.STALE,"stale capture");
        check(e.decisionLog().stream().anyMatch(x->x.input() instanceof WorldEngine.FrameInput),"replay retains full input payload");
        SourceStamp signed=new SourceStamp("a","b","cal","mount","g",1,100_000,100_000,-.3);near(signed.targetHeightM(),-.3);
        long epoch=e.epoch();e.addEgo(ego(400_000,5,0,0));check(e.epoch()==epoch+1&&e.rawHistory().isEmpty(),"unannounced large ego innovation hard resets");
        WorldEngine stationary=engine();stationary.addEgo(ego(100_000,0,0,0));stationary.addEgo(ego(200_000,0,0,0));
        stationary.accept(frame(0,"c","b","g",1,100_000,100_000,1,0),100_000);long version=stationary.publish(ego(200_000,0,0,0)).obstacleMapVersion();
        stationary.accept(frame(0,"c","b","g",2,200_000,200_000,1,0),200_000);check(stationary.publish(ego(200_000,0,0,0)).obstacleMapVersion()==version,"stationary repeated measurement doesn't churn map version");
        double withHeading=stationary.history().get(0).tracks().get(0).uncertainty().maxVariance();check(withHeading>.02,"heading lever-arm uncertainty added");
    }
    private static void workers() throws Exception {
        CountDownLatch active=new CountDownLatch(1),release=new CountDownLatch(1),latest=new CountDownLatch(1);
        List<Integer> computed=Collections.synchronizedList(new ArrayList<>());
        try(LatestWorker<Integer,Integer> worker=new LatestWorker<>("test-worker",job->{computed.add(job.request());if(job.request()==1){active.countDown();if(!release.await(2,TimeUnit.SECONDS))throw new TimeoutException();}if(job.request()==3)latest.countDown();return job.request();})) {
            worker.submit(1);check(active.await(2,TimeUnit.SECONDS),"worker active");worker.submit(2);long g=worker.submit(3);release.countDown();check(latest.await(2,TimeUnit.SECONDS),"latest request runs");
            Optional<LatestWorker.Completion<Integer>> c=Optional.empty();long deadline=System.nanoTime()+TimeUnit.SECONDS.toNanos(2);
            while(c.isEmpty()&&System.nanoTime()<deadline){c=worker.poll();if(c.isEmpty())Thread.sleep(1);}
            check(c.isPresent()&&c.get().generation()==g&&c.get().result()==3,"only latest completion");check(computed.equals(List.of(1,3)),"pending replaced no unbounded queue");
            check(worker.metrics().replaced()==1&&worker.metrics().dropped()>=1,"observable overload");worker.cancelAll();check(worker.poll().isEmpty(),"cancel drops completions");
        }
    }
    private static void plannerValidation(){
        var start=new Pose2(new Vec2(1,1),0);var goal=new PlannerBackend.Goal(new Pose2(new Vec2(3,1),0),.05,.05,.1);
        var r=new PlannerBackend.Request("r","t",0,1,0,FIELD,start,Velocity2.zero(),goal,new PlannerBackend.Footprint(.5,.5),new PlannerBackend.Constraints(1,1,1),new PlannerBackend.Bounds(0,0,10,10),
            List.of(new PlannerBackend.Obstacle("wall",new Vec2(2,1),.2,.1,1_000_000,false)),100_000,500_000,new PlannerBackend.SearchBudget(System.nanoTime(),1_000_000_000),()->false);
        var s=new WorldSnapshot(1,0,0,FIELD,ego(100_000,1,1,0),List.of());
        var result=new PlannerBackend.Result("r","t",0,1,0,PlannerBackend.Status.SUCCESS,new PlannerBackend.GeometricPath(List.of(start,goal.pose())),null,1000,500_000,"fake","synthetic");
        check(PlannerValidation.validate(r,result,s,0,0,200_000).status()==PlannerBackend.Status.INVALID_INPUT,"independent whole path collision validation");
        check(PlannerValidation.validate(r,result,s,1,0,200_000).status()==PlannerBackend.Status.STALE_RESULT,"old async epoch");
        var trajectory=new PlannerBackend.TimedTrajectory(List.of(new PlannerBackend.TimedPoint(0,start,Velocity2.zero()),new PlannerBackend.TimedPoint(1,goal.pose(),new Velocity2(new Vec2(100,0),0))));
        var timed=new PlannerBackend.Result("r","t",0,1,0,PlannerBackend.Status.SUCCESS,result.path(),trajectory,1000,500_000,"fake","unchecked timed data");
        check(PlannerValidation.validate(r,timed,s,0,0,200_000).status()==PlannerBackend.Status.INVALID_INPUT,"unchecked timed trajectory cannot pass geometry-only gate");
        var wrongField=new WorldSnapshot(1,0,0,new FieldIdentity("other","other","other"),s.ego(),List.of());
        check(PlannerValidation.validate(r,result,wrongField,0,0,200_000).status()==PlannerBackend.Status.STALE_RESULT,"original request field identity gated");
    }
}
