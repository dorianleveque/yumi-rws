MODULE LeadThrough
  
    !***********************************************************
    ! Program data
    !***********************************************************
   
    VAR bool activateLeadThrough := false;
    
    !***********************************************************
    !
    ! Procedure main
    !
    !***********************************************************
    PROC main()
        TPErase;
        TPWrite "RAPID started";
        
        WHILE TRUE DO
            IF activateLeadThrough THEN
                SetLeadThrough \On;
            ELSE
                SetLeadThrough \Off;
            ENDIF
        ENDWHILE
    ENDPROC
ENDMODULE