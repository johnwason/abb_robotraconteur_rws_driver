MODULE Module1
        CONST robtarget Target_10:=[[499.999992196,-150.00001257,624.999987626],[0.707106781,0.000000013,0.707106781,-0.00000001],[-1,-1,0,1],[9E+09,9E+09,9E+09,9E+09,9E+09,9E+09]];
    CONST robtarget Target_20:=[[499.99999087,-248.387690305,730.747856915],[0.686438597,0.16971169,0.686438612,-0.169711601],[-1,0,-1,1],[9E+09,9E+09,9E+09,9E+09,9E+09,9E+09]];
    CONST robtarget Target_30:=[[499.999991024,166.254404463,624.999979073],[0.707106773,0,0.70710679,-0.000000005],[0,0,-1,1],[9E+09,9E+09,9E+09,9E+09,9E+09,9E+09]];
    CONST robtarget Target_40:=[[499.999995076,166.254390858,806.452197896],[0.695967512,0.125016908,0.695967503,-0.125016918],[0,0,-1,1],[9E+09,9E+09,9E+09,9E+09,9E+09,9E+09]];
    
    LOCAL VAR egmident egmID1;
    LOCAL VAR egmstate egmSt1;
    
    PERS string test_var_str:="";
    PERS num test_var_num:=0;

    PROC main()
        !Add your code here
        StartEGMStreaming;
        
        
        Path_10;
    ENDPROC
    PROC Path_10()
        MoveL Target_10,v1000,z100,tool0\WObj:=wobj0;
        MoveL Target_20,v1000,z100,tool0\WObj:=wobj0;
        MoveL Target_30,v1000,z100,tool0\WObj:=wobj0;
        MoveL Target_40,v1000,z100,tool0\WObj:=wobj0;
    ENDPROC
    PROC StartEGMStreaming()
        egmSt1:=EGMGetState(egmID1);
        IF egmSt1 = EGM_STATE_RUNNING THEN
            TPWrite "EGM already running";
            RETURN;
        ENDIF
        TPWrite "Starting EGM";
        EGMReset egmID1;
        WaitTime 0.005;
        EGMGetId egmID1;
        egmSt1:=EGMGetState(egmID1);
        
        EGMSetupUC ROB_1,egmID1,"default","UCdevice"\Joint\CommTimeout:=1000000;
        EGMStreamStop egmID1;
        WaitTime 0.5;
        EGMStreamStart egmID1\SampleRate:=4;
    ENDPROC
ENDMODULE