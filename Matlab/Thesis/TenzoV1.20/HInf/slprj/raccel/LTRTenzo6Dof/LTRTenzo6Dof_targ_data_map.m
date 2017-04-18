  function targMap = targDataMap(),

  ;%***********************
  ;% Create Parameter Map *
  ;%***********************
      
    nTotData      = 0; %add to this count as we go
    nTotSects     = 2;
    sectIdxOffset = 0;
    
    ;%
    ;% Define dummy sections & preallocate arrays
    ;%
    dumSection.nData = -1;  
    dumSection.data  = [];
    
    dumData.logicalSrcIdx = -1;
    dumData.dtTransOffset = -1;
    
    ;%
    ;% Init/prealloc paramMap
    ;%
    paramMap.nSections           = nTotSects;
    paramMap.sectIdxOffset       = sectIdxOffset;
      paramMap.sections(nTotSects) = dumSection; %prealloc
    paramMap.nTotData            = -1;
    
    ;%
    ;% Auto data (rtP)
    ;%
      section.nData     = 50;
      section.data(50)  = dumData; %prealloc
      
	  ;% rtP.A0
	  section.data(1).logicalSrcIdx = 0;
	  section.data(1).dtTransOffset = 0;
	
	  ;% rtP.B0
	  section.data(2).logicalSrcIdx = 1;
	  section.data(2).dtTransOffset = 64;
	
	  ;% rtP.C0
	  section.data(3).logicalSrcIdx = 2;
	  section.data(3).dtTransOffset = 96;
	
	  ;% rtP.KAoss
	  section.data(4).logicalSrcIdx = 3;
	  section.data(4).dtTransOffset = 128;
	
	  ;% rtP.KBossw
	  section.data(5).logicalSrcIdx = 4;
	  section.data(5).dtTransOffset = 192;
	
	  ;% rtP.KCoss
	  section.data(6).logicalSrcIdx = 5;
	  section.data(6).dtTransOffset = 256;
	
	  ;% rtP.Kopt
	  section.data(7).logicalSrcIdx = 6;
	  section.data(7).dtTransOffset = 320;
	
	  ;% rtP.amplitudeNoise
	  section.data(8).logicalSrcIdx = 7;
	  section.data(8).dtTransOffset = 352;
	
	  ;% rtP.amplitudePertIn
	  section.data(9).logicalSrcIdx = 8;
	  section.data(9).dtTransOffset = 353;
	
	  ;% rtP.amplitudePertOut
	  section.data(10).logicalSrcIdx = 9;
	  section.data(10).dtTransOffset = 354;
	
	  ;% rtP.cstPertIn
	  section.data(11).logicalSrcIdx = 10;
	  section.data(11).dtTransOffset = 355;
	
	  ;% rtP.cstPertOut
	  section.data(12).logicalSrcIdx = 11;
	  section.data(12).dtTransOffset = 356;
	
	  ;% rtP.omegaNoise
	  section.data(13).logicalSrcIdx = 12;
	  section.data(13).dtTransOffset = 357;
	
	  ;% rtP.omegaPertIN
	  section.data(14).logicalSrcIdx = 13;
	  section.data(14).dtTransOffset = 358;
	
	  ;% rtP.omegaPertOut
	  section.data(15).logicalSrcIdx = 14;
	  section.data(15).dtTransOffset = 359;
	
	  ;% rtP.randomAmpNoise
	  section.data(16).logicalSrcIdx = 15;
	  section.data(16).dtTransOffset = 360;
	
	  ;% rtP.SinOut_Bias
	  section.data(17).logicalSrcIdx = 16;
	  section.data(17).dtTransOffset = 361;
	
	  ;% rtP.SinOut_Phase
	  section.data(18).logicalSrcIdx = 17;
	  section.data(18).dtTransOffset = 362;
	
	  ;% rtP.Constant_Value
	  section.data(19).logicalSrcIdx = 18;
	  section.data(19).dtTransOffset = 363;
	
	  ;% rtP.Switch_Threshold
	  section.data(20).logicalSrcIdx = 19;
	  section.data(20).dtTransOffset = 364;
	
	  ;% rtP.Processo1_X0
	  section.data(21).logicalSrcIdx = 20;
	  section.data(21).dtTransOffset = 365;
	
	  ;% rtP.Ps4_A
	  section.data(22).logicalSrcIdx = 21;
	  section.data(22).dtTransOffset = 366;
	
	  ;% rtP.Ps4_C
	  section.data(23).logicalSrcIdx = 22;
	  section.data(23).dtTransOffset = 368;
	
	  ;% rtP.Constant2_Value
	  section.data(24).logicalSrcIdx = 23;
	  section.data(24).dtTransOffset = 370;
	
	  ;% rtP.Ps5_A
	  section.data(25).logicalSrcIdx = 24;
	  section.data(25).dtTransOffset = 371;
	
	  ;% rtP.Ps5_C
	  section.data(26).logicalSrcIdx = 25;
	  section.data(26).dtTransOffset = 373;
	
	  ;% rtP.Ps1_A
	  section.data(27).logicalSrcIdx = 26;
	  section.data(27).dtTransOffset = 375;
	
	  ;% rtP.Ps1_C
	  section.data(28).logicalSrcIdx = 27;
	  section.data(28).dtTransOffset = 377;
	
	  ;% rtP.kalman_X0
	  section.data(29).logicalSrcIdx = 28;
	  section.data(29).dtTransOffset = 379;
	
	  ;% rtP.SineIn_Bias
	  section.data(30).logicalSrcIdx = 29;
	  section.data(30).dtTransOffset = 380;
	
	  ;% rtP.SineIn_Phase
	  section.data(31).logicalSrcIdx = 30;
	  section.data(31).dtTransOffset = 381;
	
	  ;% rtP.Constant_Value_gakmm5of0a
	  section.data(32).logicalSrcIdx = 31;
	  section.data(32).dtTransOffset = 382;
	
	  ;% rtP.Switch_Threshold_b2p15hzzi4
	  section.data(33).logicalSrcIdx = 32;
	  section.data(33).dtTransOffset = 383;
	
	  ;% rtP.Constant1_Value
	  section.data(34).logicalSrcIdx = 33;
	  section.data(34).dtTransOffset = 384;
	
	  ;% rtP.Ps1_A_cnuk0h4tar
	  section.data(35).logicalSrcIdx = 34;
	  section.data(35).dtTransOffset = 385;
	
	  ;% rtP.Ps1_C_gfroigzrf4
	  section.data(36).logicalSrcIdx = 35;
	  section.data(36).dtTransOffset = 386;
	
	  ;% rtP.Gain_Gain
	  section.data(37).logicalSrcIdx = 36;
	  section.data(37).dtTransOffset = 387;
	
	  ;% rtP.u2sen05t_Bias
	  section.data(38).logicalSrcIdx = 37;
	  section.data(38).dtTransOffset = 388;
	
	  ;% rtP.u2sen05t_Phase
	  section.data(39).logicalSrcIdx = 38;
	  section.data(39).dtTransOffset = 389;
	
	  ;% rtP.UniformRandomNumber_Seed
	  section.data(40).logicalSrcIdx = 39;
	  section.data(40).dtTransOffset = 390;
	
	  ;% rtP.FromWs_Time0
	  section.data(41).logicalSrcIdx = 40;
	  section.data(41).dtTransOffset = 391;
	
	  ;% rtP.FromWs_Data0
	  section.data(42).logicalSrcIdx = 41;
	  section.data(42).dtTransOffset = 397;
	
	  ;% rtP.FromWs_Time0_nyx1zce205
	  section.data(43).logicalSrcIdx = 42;
	  section.data(43).dtTransOffset = 403;
	
	  ;% rtP.FromWs_Data0_cogeshtz5t
	  section.data(44).logicalSrcIdx = 43;
	  section.data(44).dtTransOffset = 413;
	
	  ;% rtP.FromWs_Time0_lk1gq232s2
	  section.data(45).logicalSrcIdx = 44;
	  section.data(45).dtTransOffset = 423;
	
	  ;% rtP.FromWs_Data0_igdugvbssl
	  section.data(46).logicalSrcIdx = 45;
	  section.data(46).dtTransOffset = 429;
	
	  ;% rtP.FromWs_Time0_jv04arnttw
	  section.data(47).logicalSrcIdx = 46;
	  section.data(47).dtTransOffset = 435;
	
	  ;% rtP.FromWs_Data0_pirid4ox1s
	  section.data(48).logicalSrcIdx = 47;
	  section.data(48).dtTransOffset = 445;
	
	  ;% rtP.Gain_Gain_l2aa1kgqdh
	  section.data(49).logicalSrcIdx = 48;
	  section.data(49).dtTransOffset = 455;
	
	  ;% rtP.Gain_Gain_gpep4bzie5
	  section.data(50).logicalSrcIdx = 49;
	  section.data(50).dtTransOffset = 456;
	
      nTotData = nTotData + section.nData;
      paramMap.sections(1) = section;
      clear section
      
      section.nData     = 1;
      section.data(1)  = dumData; %prealloc
      
	  ;% rtP.ManualSwitch1_CurrentSetting
	  section.data(1).logicalSrcIdx = 50;
	  section.data(1).dtTransOffset = 0;
	
      nTotData = nTotData + section.nData;
      paramMap.sections(2) = section;
      clear section
      
    
      ;%
      ;% Non-auto Data (parameter)
      ;%
    

    ;%
    ;% Add final counts to struct.
    ;%
    paramMap.nTotData = nTotData;
    


  ;%**************************
  ;% Create Block Output Map *
  ;%**************************
      
    nTotData      = 0; %add to this count as we go
    nTotSects     = 1;
    sectIdxOffset = 0;
    
    ;%
    ;% Define dummy sections & preallocate arrays
    ;%
    dumSection.nData = -1;  
    dumSection.data  = [];
    
    dumData.logicalSrcIdx = -1;
    dumData.dtTransOffset = -1;
    
    ;%
    ;% Init/prealloc sigMap
    ;%
    sigMap.nSections           = nTotSects;
    sigMap.sectIdxOffset       = sectIdxOffset;
      sigMap.sections(nTotSects) = dumSection; %prealloc
    sigMap.nTotData            = -1;
    
    ;%
    ;% Auto data (rtB)
    ;%
      section.nData     = 23;
      section.data(23)  = dumData; %prealloc
      
	  ;% rtB.oaz5l4crsf
	  section.data(1).logicalSrcIdx = 0;
	  section.data(1).dtTransOffset = 0;
	
	  ;% rtB.m4f4roaota
	  section.data(2).logicalSrcIdx = 1;
	  section.data(2).dtTransOffset = 1;
	
	  ;% rtB.l3ya5q3dsy
	  section.data(3).logicalSrcIdx = 2;
	  section.data(3).dtTransOffset = 2;
	
	  ;% rtB.oxa3bf04z4
	  section.data(4).logicalSrcIdx = 3;
	  section.data(4).dtTransOffset = 6;
	
	  ;% rtB.kk4pom5z0m
	  section.data(5).logicalSrcIdx = 4;
	  section.data(5).dtTransOffset = 7;
	
	  ;% rtB.c1do0agcsm
	  section.data(6).logicalSrcIdx = 5;
	  section.data(6).dtTransOffset = 8;
	
	  ;% rtB.aolioyabae
	  section.data(7).logicalSrcIdx = 6;
	  section.data(7).dtTransOffset = 9;
	
	  ;% rtB.mg0az2nzgj
	  section.data(8).logicalSrcIdx = 7;
	  section.data(8).dtTransOffset = 10;
	
	  ;% rtB.dt0bxeukl1
	  section.data(9).logicalSrcIdx = 8;
	  section.data(9).dtTransOffset = 11;
	
	  ;% rtB.m5z5sj11wq
	  section.data(10).logicalSrcIdx = 9;
	  section.data(10).dtTransOffset = 12;
	
	  ;% rtB.px0r25lgtw
	  section.data(11).logicalSrcIdx = 10;
	  section.data(11).dtTransOffset = 13;
	
	  ;% rtB.lk5cyp2tjd
	  section.data(12).logicalSrcIdx = 11;
	  section.data(12).dtTransOffset = 14;
	
	  ;% rtB.exersch3pk
	  section.data(13).logicalSrcIdx = 12;
	  section.data(13).dtTransOffset = 15;
	
	  ;% rtB.fxeigzy1kf
	  section.data(14).logicalSrcIdx = 13;
	  section.data(14).dtTransOffset = 23;
	
	  ;% rtB.fwztmrgmqg
	  section.data(15).logicalSrcIdx = 14;
	  section.data(15).dtTransOffset = 24;
	
	  ;% rtB.k3xhv4ixsk
	  section.data(16).logicalSrcIdx = 15;
	  section.data(16).dtTransOffset = 25;
	
	  ;% rtB.ajlqbn3dtj
	  section.data(17).logicalSrcIdx = 16;
	  section.data(17).dtTransOffset = 26;
	
	  ;% rtB.obr0rt4zff
	  section.data(18).logicalSrcIdx = 17;
	  section.data(18).dtTransOffset = 30;
	
	  ;% rtB.odgn1acl0s
	  section.data(19).logicalSrcIdx = 18;
	  section.data(19).dtTransOffset = 38;
	
	  ;% rtB.gjftqhkwh2
	  section.data(20).logicalSrcIdx = 19;
	  section.data(20).dtTransOffset = 41;
	
	  ;% rtB.fezrnkyxs4
	  section.data(21).logicalSrcIdx = 20;
	  section.data(21).dtTransOffset = 44;
	
	  ;% rtB.anbxmjzqcz
	  section.data(22).logicalSrcIdx = 21;
	  section.data(22).dtTransOffset = 48;
	
	  ;% rtB.klytvkrzsu
	  section.data(23).logicalSrcIdx = 22;
	  section.data(23).dtTransOffset = 49;
	
      nTotData = nTotData + section.nData;
      sigMap.sections(1) = section;
      clear section
      
    
      ;%
      ;% Non-auto Data (signal)
      ;%
    

    ;%
    ;% Add final counts to struct.
    ;%
    sigMap.nTotData = nTotData;
    


  ;%*******************
  ;% Create DWork Map *
  ;%*******************
      
    nTotData      = 0; %add to this count as we go
    nTotSects     = 5;
    sectIdxOffset = 1;
    
    ;%
    ;% Define dummy sections & preallocate arrays
    ;%
    dumSection.nData = -1;  
    dumSection.data  = [];
    
    dumData.logicalSrcIdx = -1;
    dumData.dtTransOffset = -1;
    
    ;%
    ;% Init/prealloc dworkMap
    ;%
    dworkMap.nSections           = nTotSects;
    dworkMap.sectIdxOffset       = sectIdxOffset;
      dworkMap.sections(nTotSects) = dumSection; %prealloc
    dworkMap.nTotData            = -1;
    
    ;%
    ;% Auto data (rtDW)
    ;%
      section.nData     = 1;
      section.data(1)  = dumData; %prealloc
      
	  ;% rtDW.dcnnnsw5q4
	  section.data(1).logicalSrcIdx = 0;
	  section.data(1).dtTransOffset = 0;
	
      nTotData = nTotData + section.nData;
      dworkMap.sections(1) = section;
      clear section
      
      section.nData     = 23;
      section.data(23)  = dumData; %prealloc
      
	  ;% rtDW.ci4od0mju3.LoggedData
	  section.data(1).logicalSrcIdx = 1;
	  section.data(1).dtTransOffset = 0;
	
	  ;% rtDW.at54wibglc.LoggedData
	  section.data(2).logicalSrcIdx = 2;
	  section.data(2).dtTransOffset = 1;
	
	  ;% rtDW.mwii1vbwa3.LoggedData
	  section.data(3).logicalSrcIdx = 3;
	  section.data(3).dtTransOffset = 2;
	
	  ;% rtDW.obppf3fwyv.LoggedData
	  section.data(4).logicalSrcIdx = 4;
	  section.data(4).dtTransOffset = 3;
	
	  ;% rtDW.nkasfhaegb.LoggedData
	  section.data(5).logicalSrcIdx = 5;
	  section.data(5).dtTransOffset = 4;
	
	  ;% rtDW.d4slrxby1k.LoggedData
	  section.data(6).logicalSrcIdx = 6;
	  section.data(6).dtTransOffset = 5;
	
	  ;% rtDW.dcwsregl10.LoggedData
	  section.data(7).logicalSrcIdx = 7;
	  section.data(7).dtTransOffset = 6;
	
	  ;% rtDW.n42qfil22t.LoggedData
	  section.data(8).logicalSrcIdx = 8;
	  section.data(8).dtTransOffset = 7;
	
	  ;% rtDW.klvshyuq1n.LoggedData
	  section.data(9).logicalSrcIdx = 9;
	  section.data(9).dtTransOffset = 8;
	
	  ;% rtDW.i4oe2vbkte.LoggedData
	  section.data(10).logicalSrcIdx = 10;
	  section.data(10).dtTransOffset = 9;
	
	  ;% rtDW.kuqz0fzjc5.LoggedData
	  section.data(11).logicalSrcIdx = 11;
	  section.data(11).dtTransOffset = 10;
	
	  ;% rtDW.iqmfoh3acq.TimePtr
	  section.data(12).logicalSrcIdx = 12;
	  section.data(12).dtTransOffset = 11;
	
	  ;% rtDW.luaideq0uj.TimePtr
	  section.data(13).logicalSrcIdx = 13;
	  section.data(13).dtTransOffset = 12;
	
	  ;% rtDW.hkxntuepnx.TimePtr
	  section.data(14).logicalSrcIdx = 14;
	  section.data(14).dtTransOffset = 13;
	
	  ;% rtDW.anvo41nxpw.TimePtr
	  section.data(15).logicalSrcIdx = 15;
	  section.data(15).dtTransOffset = 14;
	
	  ;% rtDW.ed3up5fmtr.LoggedData
	  section.data(16).logicalSrcIdx = 16;
	  section.data(16).dtTransOffset = 15;
	
	  ;% rtDW.pi5lysgpgt.LoggedData
	  section.data(17).logicalSrcIdx = 17;
	  section.data(17).dtTransOffset = 16;
	
	  ;% rtDW.nozkwoectk.LoggedData
	  section.data(18).logicalSrcIdx = 18;
	  section.data(18).dtTransOffset = 20;
	
	  ;% rtDW.ftg1pj35ds.LoggedData
	  section.data(19).logicalSrcIdx = 19;
	  section.data(19).dtTransOffset = 21;
	
	  ;% rtDW.gd0ec4ttnt.LoggedData
	  section.data(20).logicalSrcIdx = 20;
	  section.data(20).dtTransOffset = 22;
	
	  ;% rtDW.bxixgcgfkj.LoggedData
	  section.data(21).logicalSrcIdx = 21;
	  section.data(21).dtTransOffset = 23;
	
	  ;% rtDW.iob4th0cd0.LoggedData
	  section.data(22).logicalSrcIdx = 22;
	  section.data(22).dtTransOffset = 24;
	
	  ;% rtDW.fk1piers5l.LoggedData
	  section.data(23).logicalSrcIdx = 23;
	  section.data(23).dtTransOffset = 28;
	
      nTotData = nTotData + section.nData;
      dworkMap.sections(2) = section;
      clear section
      
      section.nData     = 1;
      section.data(1)  = dumData; %prealloc
      
	  ;% rtDW.awp2bz51sp
	  section.data(1).logicalSrcIdx = 24;
	  section.data(1).dtTransOffset = 0;
	
      nTotData = nTotData + section.nData;
      dworkMap.sections(3) = section;
      clear section
      
      section.nData     = 12;
      section.data(12)  = dumData; %prealloc
      
	  ;% rtDW.iqdzkc4qec
	  section.data(1).logicalSrcIdx = 25;
	  section.data(1).dtTransOffset = 0;
	
	  ;% rtDW.brdcus5eot
	  section.data(2).logicalSrcIdx = 26;
	  section.data(2).dtTransOffset = 2;
	
	  ;% rtDW.axjurmlx3x.PrevIndex
	  section.data(3).logicalSrcIdx = 27;
	  section.data(3).dtTransOffset = 3;
	
	  ;% rtDW.kv51vtwzgi
	  section.data(4).logicalSrcIdx = 28;
	  section.data(4).dtTransOffset = 4;
	
	  ;% rtDW.oekfi2tozz
	  section.data(5).logicalSrcIdx = 29;
	  section.data(5).dtTransOffset = 8;
	
	  ;% rtDW.ch2s1cyhcx.PrevIndex
	  section.data(6).logicalSrcIdx = 30;
	  section.data(6).dtTransOffset = 9;
	
	  ;% rtDW.a4hbqd2cbd
	  section.data(7).logicalSrcIdx = 31;
	  section.data(7).dtTransOffset = 10;
	
	  ;% rtDW.hjtypnuisf
	  section.data(8).logicalSrcIdx = 32;
	  section.data(8).dtTransOffset = 12;
	
	  ;% rtDW.oh5roumt0o.PrevIndex
	  section.data(9).logicalSrcIdx = 33;
	  section.data(9).dtTransOffset = 13;
	
	  ;% rtDW.elawudclil
	  section.data(10).logicalSrcIdx = 34;
	  section.data(10).dtTransOffset = 14;
	
	  ;% rtDW.iabwzwhwhy
	  section.data(11).logicalSrcIdx = 35;
	  section.data(11).dtTransOffset = 18;
	
	  ;% rtDW.lqstcqepka.PrevIndex
	  section.data(12).logicalSrcIdx = 36;
	  section.data(12).dtTransOffset = 19;
	
      nTotData = nTotData + section.nData;
      dworkMap.sections(4) = section;
      clear section
      
      section.nData     = 2;
      section.data(2)  = dumData; %prealloc
      
	  ;% rtDW.htlyqa0psc
	  section.data(1).logicalSrcIdx = 37;
	  section.data(1).dtTransOffset = 0;
	
	  ;% rtDW.baoant1mbx
	  section.data(2).logicalSrcIdx = 38;
	  section.data(2).dtTransOffset = 1;
	
      nTotData = nTotData + section.nData;
      dworkMap.sections(5) = section;
      clear section
      
    
      ;%
      ;% Non-auto Data (dwork)
      ;%
    

    ;%
    ;% Add final counts to struct.
    ;%
    dworkMap.nTotData = nTotData;
    


  ;%
  ;% Add individual maps to base struct.
  ;%

  targMap.paramMap  = paramMap;    
  targMap.signalMap = sigMap;
  targMap.dworkMap  = dworkMap;
  
  ;%
  ;% Add checksums to base struct.
  ;%


  targMap.checksum0 = 3634810543;
  targMap.checksum1 = 2521803458;
  targMap.checksum2 = 2534976825;
  targMap.checksum3 = 85240381;

