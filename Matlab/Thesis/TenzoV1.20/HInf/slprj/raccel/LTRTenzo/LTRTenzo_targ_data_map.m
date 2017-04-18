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
      section.nData     = 63;
      section.data(63)  = dumData; %prealloc
      
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
	
	  ;% rtP.Ps1_A
	  section.data(22).logicalSrcIdx = 21;
	  section.data(22).dtTransOffset = 366;
	
	  ;% rtP.Ps1_C
	  section.data(23).logicalSrcIdx = 22;
	  section.data(23).dtTransOffset = 368;
	
	  ;% rtP.Constant2_Value
	  section.data(24).logicalSrcIdx = 23;
	  section.data(24).dtTransOffset = 370;
	
	  ;% rtP.Ps2_A
	  section.data(25).logicalSrcIdx = 24;
	  section.data(25).dtTransOffset = 371;
	
	  ;% rtP.Ps2_C
	  section.data(26).logicalSrcIdx = 25;
	  section.data(26).dtTransOffset = 373;
	
	  ;% rtP.Ps3_A
	  section.data(27).logicalSrcIdx = 26;
	  section.data(27).dtTransOffset = 375;
	
	  ;% rtP.Ps3_C
	  section.data(28).logicalSrcIdx = 27;
	  section.data(28).dtTransOffset = 377;
	
	  ;% rtP.Ps4_A
	  section.data(29).logicalSrcIdx = 28;
	  section.data(29).dtTransOffset = 379;
	
	  ;% rtP.Ps4_C
	  section.data(30).logicalSrcIdx = 29;
	  section.data(30).dtTransOffset = 381;
	
	  ;% rtP.FromWs_Time0
	  section.data(31).logicalSrcIdx = 30;
	  section.data(31).dtTransOffset = 383;
	
	  ;% rtP.FromWs_Data0
	  section.data(32).logicalSrcIdx = 31;
	  section.data(32).dtTransOffset = 388;
	
	  ;% rtP.FromWs_Time0_h113reko55
	  section.data(33).logicalSrcIdx = 32;
	  section.data(33).dtTransOffset = 393;
	
	  ;% rtP.FromWs_Data0_kupdvn51v2
	  section.data(34).logicalSrcIdx = 33;
	  section.data(34).dtTransOffset = 399;
	
	  ;% rtP.FromWs_Time0_acysjor4ip
	  section.data(35).logicalSrcIdx = 34;
	  section.data(35).dtTransOffset = 405;
	
	  ;% rtP.FromWs_Data0_ohxionynup
	  section.data(36).logicalSrcIdx = 35;
	  section.data(36).dtTransOffset = 409;
	
	  ;% rtP.Constant1_Value
	  section.data(37).logicalSrcIdx = 36;
	  section.data(37).dtTransOffset = 413;
	
	  ;% rtP.SineIn_Bias
	  section.data(38).logicalSrcIdx = 37;
	  section.data(38).dtTransOffset = 414;
	
	  ;% rtP.SineIn_Phase
	  section.data(39).logicalSrcIdx = 38;
	  section.data(39).dtTransOffset = 415;
	
	  ;% rtP.Constant_Value_gakmm5of0a
	  section.data(40).logicalSrcIdx = 39;
	  section.data(40).dtTransOffset = 416;
	
	  ;% rtP.Switch_Threshold_b2p15hzzi4
	  section.data(41).logicalSrcIdx = 40;
	  section.data(41).dtTransOffset = 417;
	
	  ;% rtP.kalman_X0
	  section.data(42).logicalSrcIdx = 41;
	  section.data(42).dtTransOffset = 418;
	
	  ;% rtP.Saturation_UpperSat
	  section.data(43).logicalSrcIdx = 42;
	  section.data(43).dtTransOffset = 419;
	
	  ;% rtP.Saturation_LowerSat
	  section.data(44).logicalSrcIdx = 43;
	  section.data(44).dtTransOffset = 420;
	
	  ;% rtP.Constant1_Value_ngreypf4ru
	  section.data(45).logicalSrcIdx = 44;
	  section.data(45).dtTransOffset = 421;
	
	  ;% rtP.Ps1_A_cnuk0h4tar
	  section.data(46).logicalSrcIdx = 45;
	  section.data(46).dtTransOffset = 422;
	
	  ;% rtP.Ps1_C_gfroigzrf4
	  section.data(47).logicalSrcIdx = 46;
	  section.data(47).dtTransOffset = 423;
	
	  ;% rtP.Gain_Gain
	  section.data(48).logicalSrcIdx = 47;
	  section.data(48).dtTransOffset = 424;
	
	  ;% rtP.u2sen05t_Bias
	  section.data(49).logicalSrcIdx = 48;
	  section.data(49).dtTransOffset = 425;
	
	  ;% rtP.u2sen05t_Phase
	  section.data(50).logicalSrcIdx = 49;
	  section.data(50).dtTransOffset = 426;
	
	  ;% rtP.UniformRandomNumber_Seed
	  section.data(51).logicalSrcIdx = 50;
	  section.data(51).dtTransOffset = 427;
	
	  ;% rtP.FromWs_Time0_dnxa5yquln
	  section.data(52).logicalSrcIdx = 51;
	  section.data(52).dtTransOffset = 428;
	
	  ;% rtP.FromWs_Data0_pvombv1exi
	  section.data(53).logicalSrcIdx = 52;
	  section.data(53).dtTransOffset = 432;
	
	  ;% rtP.FromWs_Time0_an32oq3ef4
	  section.data(54).logicalSrcIdx = 53;
	  section.data(54).dtTransOffset = 436;
	
	  ;% rtP.FromWs_Data0_apflh1ms4l
	  section.data(55).logicalSrcIdx = 54;
	  section.data(55).dtTransOffset = 446;
	
	  ;% rtP.FromWs_Time0_bxwfouleny
	  section.data(56).logicalSrcIdx = 55;
	  section.data(56).dtTransOffset = 456;
	
	  ;% rtP.FromWs_Data0_gglwjxomlu
	  section.data(57).logicalSrcIdx = 56;
	  section.data(57).dtTransOffset = 466;
	
	  ;% rtP.FromWs_Time0_dgntxkplur
	  section.data(58).logicalSrcIdx = 57;
	  section.data(58).dtTransOffset = 476;
	
	  ;% rtP.FromWs_Data0_n4fvl010iy
	  section.data(59).logicalSrcIdx = 58;
	  section.data(59).dtTransOffset = 486;
	
	  ;% rtP.FromWs_Time0_jrdfiknsxo
	  section.data(60).logicalSrcIdx = 59;
	  section.data(60).dtTransOffset = 496;
	
	  ;% rtP.FromWs_Data0_i5qxt5w12n
	  section.data(61).logicalSrcIdx = 60;
	  section.data(61).dtTransOffset = 504;
	
	  ;% rtP.Gain_Gain_l2aa1kgqdh
	  section.data(62).logicalSrcIdx = 61;
	  section.data(62).dtTransOffset = 512;
	
	  ;% rtP.Gain_Gain_gpep4bzie5
	  section.data(63).logicalSrcIdx = 62;
	  section.data(63).dtTransOffset = 513;
	
      nTotData = nTotData + section.nData;
      paramMap.sections(1) = section;
      clear section
      
      section.nData     = 2;
      section.data(2)  = dumData; %prealloc
      
	  ;% rtP.ManualSwitch_CurrentSetting
	  section.data(1).logicalSrcIdx = 63;
	  section.data(1).dtTransOffset = 0;
	
	  ;% rtP.ManualSwitch1_CurrentSetting
	  section.data(2).logicalSrcIdx = 64;
	  section.data(2).dtTransOffset = 1;
	
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
      
	  ;% rtB.mbsguhsyce
	  section.data(1).logicalSrcIdx = 0;
	  section.data(1).dtTransOffset = 0;
	
	  ;% rtB.gxh0zyu42a
	  section.data(2).logicalSrcIdx = 1;
	  section.data(2).dtTransOffset = 1;
	
	  ;% rtB.j4tv1cqyj2
	  section.data(3).logicalSrcIdx = 2;
	  section.data(3).dtTransOffset = 2;
	
	  ;% rtB.gbemrwhpr5
	  section.data(4).logicalSrcIdx = 3;
	  section.data(4).dtTransOffset = 6;
	
	  ;% rtB.bdgzagdxra
	  section.data(5).logicalSrcIdx = 4;
	  section.data(5).dtTransOffset = 7;
	
	  ;% rtB.bxeh32cm4i
	  section.data(6).logicalSrcIdx = 5;
	  section.data(6).dtTransOffset = 15;
	
	  ;% rtB.ej3n31dyxa
	  section.data(7).logicalSrcIdx = 6;
	  section.data(7).dtTransOffset = 16;
	
	  ;% rtB.mfm3tpnt2p
	  section.data(8).logicalSrcIdx = 7;
	  section.data(8).dtTransOffset = 17;
	
	  ;% rtB.hjr0spzzaf
	  section.data(9).logicalSrcIdx = 8;
	  section.data(9).dtTransOffset = 25;
	
	  ;% rtB.csh4yoneye
	  section.data(10).logicalSrcIdx = 9;
	  section.data(10).dtTransOffset = 29;
	
	  ;% rtB.fgbq3jefrx
	  section.data(11).logicalSrcIdx = 10;
	  section.data(11).dtTransOffset = 33;
	
	  ;% rtB.d4wqki33zw
	  section.data(12).logicalSrcIdx = 11;
	  section.data(12).dtTransOffset = 34;
	
	  ;% rtB.d1v04lphp2
	  section.data(13).logicalSrcIdx = 12;
	  section.data(13).dtTransOffset = 35;
	
	  ;% rtB.hzoqt0nc3o
	  section.data(14).logicalSrcIdx = 13;
	  section.data(14).dtTransOffset = 36;
	
	  ;% rtB.otap4hfn5r
	  section.data(15).logicalSrcIdx = 14;
	  section.data(15).dtTransOffset = 37;
	
	  ;% rtB.cfdpbmvjgv
	  section.data(16).logicalSrcIdx = 15;
	  section.data(16).dtTransOffset = 38;
	
	  ;% rtB.edtxh40dpx
	  section.data(17).logicalSrcIdx = 16;
	  section.data(17).dtTransOffset = 39;
	
	  ;% rtB.gz1cngd0fd
	  section.data(18).logicalSrcIdx = 17;
	  section.data(18).dtTransOffset = 40;
	
	  ;% rtB.exb2j2haif
	  section.data(19).logicalSrcIdx = 18;
	  section.data(19).dtTransOffset = 48;
	
	  ;% rtB.ajinddapxi
	  section.data(20).logicalSrcIdx = 19;
	  section.data(20).dtTransOffset = 51;
	
	  ;% rtB.oyi1nkeowt
	  section.data(21).logicalSrcIdx = 20;
	  section.data(21).dtTransOffset = 54;
	
	  ;% rtB.kwjrknjxrb
	  section.data(22).logicalSrcIdx = 21;
	  section.data(22).dtTransOffset = 58;
	
	  ;% rtB.fd3jf50j03
	  section.data(23).logicalSrcIdx = 22;
	  section.data(23).dtTransOffset = 59;
	
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
      section.nData     = 5;
      section.data(5)  = dumData; %prealloc
      
	  ;% rtDW.cx2opkhkks
	  section.data(1).logicalSrcIdx = 0;
	  section.data(1).dtTransOffset = 0;
	
	  ;% rtDW.pzg1f0zcsj
	  section.data(2).logicalSrcIdx = 1;
	  section.data(2).dtTransOffset = 1;
	
	  ;% rtDW.gs42o1safk
	  section.data(3).logicalSrcIdx = 2;
	  section.data(3).dtTransOffset = 2;
	
	  ;% rtDW.bthpfg5v51
	  section.data(4).logicalSrcIdx = 3;
	  section.data(4).dtTransOffset = 3;
	
	  ;% rtDW.gzecfrgu4x
	  section.data(5).logicalSrcIdx = 4;
	  section.data(5).dtTransOffset = 4;
	
      nTotData = nTotData + section.nData;
      dworkMap.sections(1) = section;
      clear section
      
      section.nData     = 28;
      section.data(28)  = dumData; %prealloc
      
	  ;% rtDW.pp4sameqxw.TimePtr
	  section.data(1).logicalSrcIdx = 5;
	  section.data(1).dtTransOffset = 0;
	
	  ;% rtDW.pszu0ywji3.TimePtr
	  section.data(2).logicalSrcIdx = 6;
	  section.data(2).dtTransOffset = 1;
	
	  ;% rtDW.gmpueztb1y.TimePtr
	  section.data(3).logicalSrcIdx = 7;
	  section.data(3).dtTransOffset = 2;
	
	  ;% rtDW.p00itljrev.LoggedData
	  section.data(4).logicalSrcIdx = 8;
	  section.data(4).dtTransOffset = 3;
	
	  ;% rtDW.nzgtrzui4f.LoggedData
	  section.data(5).logicalSrcIdx = 9;
	  section.data(5).dtTransOffset = 4;
	
	  ;% rtDW.gx3ie0fm1y.LoggedData
	  section.data(6).logicalSrcIdx = 10;
	  section.data(6).dtTransOffset = 5;
	
	  ;% rtDW.ej3s3keedd.LoggedData
	  section.data(7).logicalSrcIdx = 11;
	  section.data(7).dtTransOffset = 6;
	
	  ;% rtDW.djeg5hbiuo.LoggedData
	  section.data(8).logicalSrcIdx = 12;
	  section.data(8).dtTransOffset = 7;
	
	  ;% rtDW.i3tuhdmgb2.LoggedData
	  section.data(9).logicalSrcIdx = 13;
	  section.data(9).dtTransOffset = 8;
	
	  ;% rtDW.kcxqmgqjiz.LoggedData
	  section.data(10).logicalSrcIdx = 14;
	  section.data(10).dtTransOffset = 9;
	
	  ;% rtDW.a2kafglxv4.LoggedData
	  section.data(11).logicalSrcIdx = 15;
	  section.data(11).dtTransOffset = 10;
	
	  ;% rtDW.h5swgbqjqf.LoggedData
	  section.data(12).logicalSrcIdx = 16;
	  section.data(12).dtTransOffset = 11;
	
	  ;% rtDW.p4kdljjvdf.LoggedData
	  section.data(13).logicalSrcIdx = 17;
	  section.data(13).dtTransOffset = 12;
	
	  ;% rtDW.nrvudg0031.LoggedData
	  section.data(14).logicalSrcIdx = 18;
	  section.data(14).dtTransOffset = 13;
	
	  ;% rtDW.if2ae1wsvj.LoggedData
	  section.data(15).logicalSrcIdx = 19;
	  section.data(15).dtTransOffset = 14;
	
	  ;% rtDW.mq2efjljlp.TimePtr
	  section.data(16).logicalSrcIdx = 20;
	  section.data(16).dtTransOffset = 15;
	
	  ;% rtDW.morfgxatez.TimePtr
	  section.data(17).logicalSrcIdx = 21;
	  section.data(17).dtTransOffset = 16;
	
	  ;% rtDW.d2vfv2ffdo.TimePtr
	  section.data(18).logicalSrcIdx = 22;
	  section.data(18).dtTransOffset = 17;
	
	  ;% rtDW.ctabrgnsxy.TimePtr
	  section.data(19).logicalSrcIdx = 23;
	  section.data(19).dtTransOffset = 18;
	
	  ;% rtDW.nl25ijg5ia.TimePtr
	  section.data(20).logicalSrcIdx = 24;
	  section.data(20).dtTransOffset = 19;
	
	  ;% rtDW.eolcyfqkw1.LoggedData
	  section.data(21).logicalSrcIdx = 25;
	  section.data(21).dtTransOffset = 20;
	
	  ;% rtDW.mottvgtufs.LoggedData
	  section.data(22).logicalSrcIdx = 26;
	  section.data(22).dtTransOffset = 21;
	
	  ;% rtDW.d3pymdz53u.LoggedData
	  section.data(23).logicalSrcIdx = 27;
	  section.data(23).dtTransOffset = 25;
	
	  ;% rtDW.aol5vuyb0a.LoggedData
	  section.data(24).logicalSrcIdx = 28;
	  section.data(24).dtTransOffset = 26;
	
	  ;% rtDW.fd4jgacxhq.LoggedData
	  section.data(25).logicalSrcIdx = 29;
	  section.data(25).dtTransOffset = 27;
	
	  ;% rtDW.dp4oobqveb.LoggedData
	  section.data(26).logicalSrcIdx = 30;
	  section.data(26).dtTransOffset = 28;
	
	  ;% rtDW.bqfop01i2t.LoggedData
	  section.data(27).logicalSrcIdx = 31;
	  section.data(27).dtTransOffset = 29;
	
	  ;% rtDW.jnlsbkpkpg.LoggedData
	  section.data(28).logicalSrcIdx = 32;
	  section.data(28).dtTransOffset = 33;
	
      nTotData = nTotData + section.nData;
      dworkMap.sections(2) = section;
      clear section
      
      section.nData     = 1;
      section.data(1)  = dumData; %prealloc
      
	  ;% rtDW.hjtfckrdib
	  section.data(1).logicalSrcIdx = 33;
	  section.data(1).dtTransOffset = 0;
	
      nTotData = nTotData + section.nData;
      dworkMap.sections(3) = section;
      clear section
      
      section.nData     = 25;
      section.data(25)  = dumData; %prealloc
      
	  ;% rtDW.mwqrktpz41
	  section.data(1).logicalSrcIdx = 34;
	  section.data(1).dtTransOffset = 0;
	
	  ;% rtDW.eknmvbtv5u
	  section.data(2).logicalSrcIdx = 35;
	  section.data(2).dtTransOffset = 1;
	
	  ;% rtDW.gooaqjnwdd.PrevIndex
	  section.data(3).logicalSrcIdx = 36;
	  section.data(3).dtTransOffset = 2;
	
	  ;% rtDW.ndd51edmhg
	  section.data(4).logicalSrcIdx = 37;
	  section.data(4).dtTransOffset = 3;
	
	  ;% rtDW.jobrukao3e
	  section.data(5).logicalSrcIdx = 38;
	  section.data(5).dtTransOffset = 5;
	
	  ;% rtDW.pxxh1knxpv.PrevIndex
	  section.data(6).logicalSrcIdx = 39;
	  section.data(6).dtTransOffset = 6;
	
	  ;% rtDW.fpdlolyb1y
	  section.data(7).logicalSrcIdx = 40;
	  section.data(7).dtTransOffset = 7;
	
	  ;% rtDW.ga0dnoquno
	  section.data(8).logicalSrcIdx = 41;
	  section.data(8).dtTransOffset = 8;
	
	  ;% rtDW.mgial2fbbs.PrevIndex
	  section.data(9).logicalSrcIdx = 42;
	  section.data(9).dtTransOffset = 9;
	
	  ;% rtDW.jisc0stmja
	  section.data(10).logicalSrcIdx = 43;
	  section.data(10).dtTransOffset = 10;
	
	  ;% rtDW.bhjarlltx0
	  section.data(11).logicalSrcIdx = 44;
	  section.data(11).dtTransOffset = 11;
	
	  ;% rtDW.k2mrpavnr3.PrevIndex
	  section.data(12).logicalSrcIdx = 45;
	  section.data(12).dtTransOffset = 12;
	
	  ;% rtDW.mmq01gbox5
	  section.data(13).logicalSrcIdx = 46;
	  section.data(13).dtTransOffset = 13;
	
	  ;% rtDW.gml05gaiwr
	  section.data(14).logicalSrcIdx = 47;
	  section.data(14).dtTransOffset = 17;
	
	  ;% rtDW.p0ybzcrnsf.PrevIndex
	  section.data(15).logicalSrcIdx = 48;
	  section.data(15).dtTransOffset = 18;
	
	  ;% rtDW.p1tupjnl2m
	  section.data(16).logicalSrcIdx = 49;
	  section.data(16).dtTransOffset = 19;
	
	  ;% rtDW.pt0evqezqm
	  section.data(17).logicalSrcIdx = 50;
	  section.data(17).dtTransOffset = 23;
	
	  ;% rtDW.dppeb2fcvo.PrevIndex
	  section.data(18).logicalSrcIdx = 51;
	  section.data(18).dtTransOffset = 24;
	
	  ;% rtDW.jt4kc1ii0s
	  section.data(19).logicalSrcIdx = 52;
	  section.data(19).dtTransOffset = 25;
	
	  ;% rtDW.hns1rud2yi
	  section.data(20).logicalSrcIdx = 53;
	  section.data(20).dtTransOffset = 29;
	
	  ;% rtDW.jdmacw4ijl.PrevIndex
	  section.data(21).logicalSrcIdx = 54;
	  section.data(21).dtTransOffset = 30;
	
	  ;% rtDW.mek4mei4m5
	  section.data(22).logicalSrcIdx = 55;
	  section.data(22).dtTransOffset = 31;
	
	  ;% rtDW.ppttotus2n
	  section.data(23).logicalSrcIdx = 56;
	  section.data(23).dtTransOffset = 34;
	
	  ;% rtDW.i3b2ntrlap.PrevIndex
	  section.data(24).logicalSrcIdx = 57;
	  section.data(24).dtTransOffset = 35;
	
	  ;% rtDW.ls41xbqrac
	  section.data(25).logicalSrcIdx = 58;
	  section.data(25).dtTransOffset = 36;
	
      nTotData = nTotData + section.nData;
      dworkMap.sections(4) = section;
      clear section
      
      section.nData     = 2;
      section.data(2)  = dumData; %prealloc
      
	  ;% rtDW.c0c55fi3jq
	  section.data(1).logicalSrcIdx = 59;
	  section.data(1).dtTransOffset = 0;
	
	  ;% rtDW.omgte3tg31
	  section.data(2).logicalSrcIdx = 60;
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


  targMap.checksum0 = 4144187791;
  targMap.checksum1 = 48234202;
  targMap.checksum2 = 2997501980;
  targMap.checksum3 = 3888407977;

