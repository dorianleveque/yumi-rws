MODULE ProgramPoseL

    VAR jointtarget position := [[0.0,-130.0,30.0,0.0,40.0,0.0], [135.0,0,0,0,0,0]];
    VAR num speed := 7;
    
    PROC main()
        WHILE TRUE DO
            MoveAbsJ position, v1000,\V:=speed,fine,tool0;
        ENDWHILE
    ENDPROC
    
ENDMODULE