package org.frcworldstate.core;

import java.util.Optional;
import java.util.Objects;
import java.util.concurrent.atomic.AtomicBoolean;

/** One worker + one replaceable pending slot + one latest completion. Callbacks must cooperate with cancellation. */
public final class LatestWorker<Q,R> implements AutoCloseable {
    public record Job<Q>(long generation,Q request,PlannerBackend.Cancellation cancellation) {}
    public record Completion<R>(long generation,R result,String error,long durationNanos) {}
    @FunctionalInterface public interface Backend<Q,R> { R compute(Job<Q> job) throws Exception; }
    public record Metrics(long submitted,long replaced,long cancelled,long dropped,long failures,boolean running,boolean pending,long activeAgeNanos,boolean workerAlive) {}
    private record Slot<Q>(long generation,Q request,AtomicBoolean cancelled) {}
    private final Backend<Q,R> backend;
    private final Thread thread;
    private Slot<Q> pending,active;
    private Completion<R> completed;
    private boolean closed;
    private long generation,submitted,replaced,cancelled,dropped,failures,activeStartNanos;
    public LatestWorker(String name,Backend<Q,R> backend) {
        this.backend=Objects.requireNonNull(backend);thread=new Thread(this::run,Geometry.id(name));thread.setDaemon(true);thread.start();
    }
    /** O(1), latest request replaces pending work and invalidates active/old completions. */
    public synchronized long submit(Q request) {
        if(closed)throw new IllegalStateException("closed");Objects.requireNonNull(request);submitted++;
        if(pending!=null){pending.cancelled.set(true);replaced++;}
        if(active!=null&&!active.cancelled.getAndSet(true))cancelled++;
        if(completed!=null){completed=null;dropped++;}
        pending=new Slot<>(++generation,request,new AtomicBoolean());notifyAll();return generation;
    }
    public synchronized Optional<Completion<R>> poll() {
        Completion<R> r=completed;completed=null;return Optional.ofNullable(r);
    }
    public synchronized void cancelAll() {
        generation++; if(pending!=null){pending.cancelled.set(true);pending=null;cancelled++;}
        if(active!=null&&!active.cancelled.getAndSet(true))cancelled++;completed=null;
    }
    public synchronized Metrics metrics() { return new Metrics(submitted,replaced,cancelled,dropped,failures,active!=null,pending!=null,active==null?0:System.nanoTime()-activeStartNanos,thread.isAlive()); }
    private void run() {
        while(true) {
            Slot<Q> s;
            synchronized(this) {
                while(pending==null&&!closed)try{wait();}catch(InterruptedException e){if(closed)return;}
                if(closed)return;s=pending;pending=null;active=s;activeStartNanos=System.nanoTime();
            }
            long start=System.nanoTime();R result=null;String error="";
            try{result=backend.compute(new Job<>(s.generation,s.request,s.cancelled::get));}
            catch(Exception | AssertionError | LinkageError e){error=e.getClass().getSimpleName()+": "+String.valueOf(e.getMessage());}
            long duration=System.nanoTime()-start;
            synchronized(this) {
                active=null;if(!error.isEmpty())failures++;
                if(!closed&&!s.cancelled.get()&&s.generation==generation)completed=new Completion<>(s.generation,result,error,duration);
                else dropped++;
            }
        }
    }
    /** Nonblocking. Does not wait on potentially ill-behaved injected backend. */
    @Override public synchronized void close() { closed=true;cancelAll();notifyAll();thread.interrupt(); }
}
