package edu.wpi.first.wpilibj.command;

// From https://github.com/2729StormRobotics/Storm2014

import java.util.Enumeration;

/** A Command that exposes more internals to the world. */
public class PublicCommand extends Command {
    private final Command _c;

    public PublicCommand(Command c) {
        _c = c;
        for(Enumeration e = c.getRequirements();e.hasMoreElements();) {
            requires((Subsystem) e.nextElement());
        }
    }
    public Enumeration getRequirements() {
        return _c.getRequirements();
    }

    public void initialize() {
        if(_c != null) {
            _c.initialize();
        }
    }
    public void _initialize() {
        if(_c != null) {
            _c._initialize();
        }
    }

    public void execute() {
        if(_c != null) {
            _c.execute();
        }
    }
    public void _execute() {
        if(_c != null) {
            _c._execute();
        }
    }

    public boolean isFinished() {
        return _c == null || _c.isFinished();
    }

    public void end() {
        if(_c != null) {
            _c.end();
        }
    }
    public void _end() {
        if(_c != null) {
            _c._end();
        }
    }

    public void interrupted() {
        if(_c != null) {
            _c.interrupted();
        }
    }
    public void _interrupted() {
        if(_c != null) {
            _c._interrupted();
        }
    }
    public Command getCommand() {
        return _c;
    }
}