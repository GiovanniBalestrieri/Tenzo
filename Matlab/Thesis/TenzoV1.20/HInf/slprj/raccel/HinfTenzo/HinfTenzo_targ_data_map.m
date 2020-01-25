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
      section.nData     = 62;
      section.data(62)  = dumData; %prealloc
      
	  ;% rtP.A0
	  section.data(1).logicalSrcIdx = 0;
	  section.data(1).dtTransOffset = 0;
	
	  ;% rtP.B0
	  section.data(2).logicalSrcIdx = 1;
	  section.data(2).dtTransOffset = 64;
	
	  ;% rtP.C0
	  section.data(3).logicalSrcIdx = 2;
	  section.data(3).dtTransOffset = 96;
	
	  ;% rtP.amplitudeNoise
	  section.data(4).logicalSrcIdx = 3;
	  section.data(4).dtTransOffset = 128;
	
	  ;% rtP.amplitudePertOutZ
	  section.data(5).logicalSrcIdx = 4;
	  section.data(5).dtTransOffset = 129;
	
	  ;% rtP.amplitudePertOutptp
	  section.data(6).logicalSrcIdx = 5;
	  section.data(6).dtTransOffset = 130;
	
	  ;% rtP.cstPertOut
	  section.data(7).logicalSrcIdx = 6;
	  section.data(7).dtTransOffset = 131;
	
	  ;% rtP.omegaNoise
	  section.data(8).logicalSrcIdx = 7;
	  section.data(8).dtTransOffset = 132;
	
	  ;% rtP.omegaPertOut
	  section.data(9).logicalSrcIdx = 8;
	  section.data(9).dtTransOffset = 133;
	
	  ;% rtP.randomAmpNoise
	  section.data(10).logicalSrcIdx = 9;
	  section.data(10).dtTransOffset = 134;
	
	  ;% rtP.Ps1_A
	  section.data(11).logicalSrcIdx = 10;
	  section.data(11).dtTransOffset = 135;
	
	  ;% rtP.Ps1_C
	  section.data(12).logicalSrcIdx = 11;
	  section.data(12).dtTransOffset = 137;
	
	  ;% rtP.Ps1_A_j1blkzytxk
	  section.data(13).logicalSrcIdx = 12;
	  section.data(13).dtTransOffset = 139;
	
	  ;% rtP.Ps1_C_a4z2vpeibb
	  section.data(14).logicalSrcIdx = 13;
	  section.data(14).dtTransOffset = 141;
	
	  ;% rtP.Processo1_X0
	  section.data(15).logicalSrcIdx = 14;
	  section.data(15).dtTransOffset = 143;
	
	  ;% rtP.Ps4_A
	  section.data(16).logicalSrcIdx = 15;
	  section.data(16).dtTransOffset = 144;
	
	  ;% rtP.Ps4_C
	  section.data(17).logicalSrcIdx = 16;
	  section.data(17).dtTransOffset = 146;
	
	  ;% rtP.Constant2_Value
	  section.data(18).logicalSrcIdx = 17;
	  section.data(18).dtTransOffset = 148;
	
	  ;% rtP.Ps5_A
	  section.data(19).logicalSrcIdx = 18;
	  section.data(19).dtTransOffset = 149;
	
	  ;% rtP.Ps5_C
	  section.data(20).logicalSrcIdx = 19;
	  section.data(20).dtTransOffset = 151;
	
	  ;% rtP.Ps1_A_g40xqitwcm
	  section.data(21).logicalSrcIdx = 20;
	  section.data(21).dtTransOffset = 153;
	
	  ;% rtP.Ps1_C_jxo2syryaa
	  section.data(22).logicalSrcIdx = 21;
	  section.data(22).dtTransOffset = 155;
	
	  ;% rtP.Constant1_Value
	  section.data(23).logicalSrcIdx = 22;
	  section.data(23).dtTransOffset = 157;
	
	  ;% rtP.Ps1_A_cnuk0h4tar
	  section.data(24).logicalSrcIdx = 23;
	  section.data(24).dtTransOffset = 158;
	
	  ;% rtP.Ps1_C_gfroigzrf4
	  section.data(25).logicalSrcIdx = 24;
	  section.data(25).dtTransOffset = 159;
	
	  ;% rtP.Gain_Gain
	  section.data(26).logicalSrcIdx = 25;
	  section.data(26).dtTransOffset = 160;
	
	  ;% rtP.HInfinity_A
	  section.data(27).logicalSrcIdx = 26;
	  section.data(27).dtTransOffset = 161;
	
	  ;% rtP.HInfinity_B
	  section.data(28).logicalSrcIdx = 27;
	  section.data(28).dtTransOffset = 765;
	
	  ;% rtP.HInfinity_C
	  section.data(29).logicalSrcIdx = 28;
	  section.data(29).dtTransOffset = 877;
	
	  ;% rtP.HInfinity_X0
	  section.data(30).logicalSrcIdx = 29;
	  section.data(30).dtTransOffset = 989;
	
	  ;% rtP.Ps1_A_p4fqdq0nyf
	  section.data(31).logicalSrcIdx = 30;
	  section.data(31).dtTransOffset = 990;
	
	  ;% rtP.Ps1_C_iunwv5jncy
	  section.data(32).logicalSrcIdx = 31;
	  section.data(32).dtTransOffset = 992;
	
	  ;% rtP.Constant1_Value_onxqoxz4b2
	  section.data(33).logicalSrcIdx = 32;
	  section.data(33).dtTransOffset = 994;
	
	  ;% rtP.Constant2_Value_honrvu2u0e
	  section.data(34).logicalSrcIdx = 33;
	  section.data(34).dtTransOffset = 995;
	
	  ;% rtP.seno_Amp
	  section.data(35).logicalSrcIdx = 34;
	  section.data(35).dtTransOffset = 996;
	
	  ;% rtP.seno_Bias
	  section.data(36).logicalSrcIdx = 35;
	  section.data(36).dtTransOffset = 997;
	
	  ;% rtP.seno_Phase
	  section.data(37).logicalSrcIdx = 36;
	  section.data(37).dtTransOffset = 998;
	
	  ;% rtP.Switch1_Threshold
	  section.data(38).logicalSrcIdx = 37;
	  section.data(38).dtTransOffset = 999;
	
	  ;% rtP.Switch2_Threshold
	  section.data(39).logicalSrcIdx = 38;
	  section.data(39).dtTransOffset = 1000;
	
	  ;% rtP.Constant_Value
	  section.data(40).logicalSrcIdx = 39;
	  section.data(40).dtTransOffset = 1001;
	
	  ;% rtP.Constant2_Value_maeni3njql
	  section.data(41).logicalSrcIdx = 40;
	  section.data(41).dtTransOffset = 1002;
	
	  ;% rtP.SineOut_Bias
	  section.data(42).logicalSrcIdx = 41;
	  section.data(42).dtTransOffset = 1003;
	
	  ;% rtP.SineOut_Phase
	  section.data(43).logicalSrcIdx = 42;
	  section.data(43).dtTransOffset = 1004;
	
	  ;% rtP.Switch_Threshold
	  section.data(44).logicalSrcIdx = 43;
	  section.data(44).dtTransOffset = 1005;
	
	  ;% rtP.Switch1_Threshold_a0fpu4ohya
	  section.data(45).logicalSrcIdx = 44;
	  section.data(45).dtTransOffset = 1006;
	
	  ;% rtP.Constant_Value_gsg5wrsjtq
	  section.data(46).logicalSrcIdx = 45;
	  section.data(46).dtTransOffset = 1007;
	
	  ;% rtP.SineOut_Bias_amml0uhsh5
	  section.data(47).logicalSrcIdx = 46;
	  section.data(47).dtTransOffset = 1008;
	
	  ;% rtP.SineOut_Phase_ek4wf0nzh3
	  section.data(48).logicalSrcIdx = 47;
	  section.data(48).dtTransOffset = 1009;
	
	  ;% rtP.Switch_Threshold_je2vilxnqj
	  section.data(49).logicalSrcIdx = 48;
	  section.data(49).dtTransOffset = 1010;
	
	  ;% rtP.u2sen05t_Bias
	  section.data(50).logicalSrcIdx = 49;
	  section.data(50).dtTransOffset = 1011;
	
	  ;% rtP.u2sen05t_Phase
	  section.data(51).logicalSrcIdx = 50;
	  section.data(51).dtTransOffset = 1012;
	
	  ;% rtP.UniformRandomNumber_Seed
	  section.data(52).logicalSrcIdx = 51;
	  section.data(52).dtTransOffset = 1013;
	
	  ;% rtP.FromWs_Time0
	  section.data(53).logicalSrcIdx = 52;
	  section.data(53).dtTransOffset = 1014;
	
	  ;% rtP.FromWs_Data0
	  section.data(54).logicalSrcIdx = 53;
	  section.data(54).dtTransOffset = 1020;
	
	  ;% rtP.FromWs_Time0_mdh1vup2gy
	  section.data(55).logicalSrcIdx = 54;
	  section.data(55).dtTransOffset = 1026;
	
	  ;% rtP.FromWs_Data0_amonalu2nw
	  section.data(56).logicalSrcIdx = 55;
	  section.data(56).dtTransOffset = 1032;
	
	  ;% rtP.FromWs_Time0_ni0stwjpd1
	  section.data(57).logicalSrcIdx = 56;
	  section.data(57).dtTransOffset = 1038;
	
	  ;% rtP.FromWs_Data0_bui33k5vfq
	  section.data(58).logicalSrcIdx = 57;
	  section.data(58).dtTransOffset = 1044;
	
	  ;% rtP.FromWs_Time0_ovshumjb0b
	  section.data(59).logicalSrcIdx = 58;
	  section.data(59).dtTransOffset = 1050;
	
	  ;% rtP.FromWs_Data0_cjzjycivf4
	  section.data(60).logicalSrcIdx = 59;
	  section.data(60).dtTransOffset = 1060;
	
	  ;% rtP.Saturation1_UpperSat
	  section.data(61).logicalSrcIdx = 60;
	  section.data(61).dtTransOffset = 1070;
	
	  ;% rtP.Saturation1_LowerSat
	  section.data(62).logicalSrcIdx = 61;
	  section.data(62).dtTransOffset = 1071;
	
      nTotData = nTotData + section.nData;
      paramMap.sections(1) = section;
      clear section
      
      section.nData     = 1;
      section.data(1)  = dumData; %prealloc
      
	  ;% rtP.ManualSwitch1_CurrentSetting
	  section.data(1).logicalSrcIdx = 62;
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
      section.nData     = 31;
      section.data(31)  = dumData; %prealloc
      
	  ;% rtB.mikmmv5pjy
	  section.data(1).logicalSrcIdx = 0;
	  section.data(1).dtTransOffset = 0;
	
	  ;% rtB.iglrdpeo0u
	  section.data(2).logicalSrcIdx = 1;
	  section.data(2).dtTransOffset = 1;
	
	  ;% rtB.cuhucxnwbl
	  section.data(3).logicalSrcIdx = 2;
	  section.data(3).dtTransOffset = 2;
	
	  ;% rtB.ndvxwhqhzh
	  section.data(4).logicalSrcIdx = 3;
	  section.data(4).dtTransOffset = 6;
	
	  ;% rtB.kohnzyllz4
	  section.data(5).logicalSrcIdx = 4;
	  section.data(5).dtTransOffset = 7;
	
	  ;% rtB.lywwonuzni
	  section.data(6).logicalSrcIdx = 5;
	  section.data(6).dtTransOffset = 8;
	
	  ;% rtB.cgqon1bp5k
	  section.data(7).logicalSrcIdx = 6;
	  section.data(7).dtTransOffset = 9;
	
	  ;% rtB.ccnrqukp0p
	  section.data(8).logicalSrcIdx = 7;
	  section.data(8).dtTransOffset = 10;
	
	  ;% rtB.o2ra02ecjo
	  section.data(9).logicalSrcIdx = 8;
	  section.data(9).dtTransOffset = 12;
	
	  ;% rtB.g3cxppgcad
	  section.data(10).logicalSrcIdx = 9;
	  section.data(10).dtTransOffset = 13;
	
	  ;% rtB.nq0m4x3haw
	  section.data(11).logicalSrcIdx = 10;
	  section.data(11).dtTransOffset = 17;
	
	  ;% rtB.ffcuwotdjl
	  section.data(12).logicalSrcIdx = 11;
	  section.data(12).dtTransOffset = 18;
	
	  ;% rtB.h0k2fzanuq
	  section.data(13).logicalSrcIdx = 12;
	  section.data(13).dtTransOffset = 19;
	
	  ;% rtB.l3ibipirjv
	  section.data(14).logicalSrcIdx = 13;
	  section.data(14).dtTransOffset = 20;
	
	  ;% rtB.o4bvr14r1c
	  section.data(15).logicalSrcIdx = 14;
	  section.data(15).dtTransOffset = 21;
	
	  ;% rtB.iklusr35kc
	  section.data(16).logicalSrcIdx = 15;
	  section.data(16).dtTransOffset = 22;
	
	  ;% rtB.elhmsr12bw
	  section.data(17).logicalSrcIdx = 16;
	  section.data(17).dtTransOffset = 23;
	
	  ;% rtB.epfm0vmgr0
	  section.data(18).logicalSrcIdx = 17;
	  section.data(18).dtTransOffset = 24;
	
	  ;% rtB.dx0ibnxabm
	  section.data(19).logicalSrcIdx = 18;
	  section.data(19).dtTransOffset = 25;
	
	  ;% rtB.ahq3gaiq3g
	  section.data(20).logicalSrcIdx = 19;
	  section.data(20).dtTransOffset = 26;
	
	  ;% rtB.jfilnvyqtk
	  section.data(21).logicalSrcIdx = 20;
	  section.data(21).dtTransOffset = 27;
	
	  ;% rtB.bnmysexqkh
	  section.data(22).logicalSrcIdx = 21;
	  section.data(22).dtTransOffset = 28;
	
	  ;% rtB.afiufrufpa
	  section.data(23).logicalSrcIdx = 22;
	  section.data(23).dtTransOffset = 29;
	
	  ;% rtB.eivqzdcosm
	  section.data(24).logicalSrcIdx = 23;
	  section.data(24).dtTransOffset = 30;
	
	  ;% rtB.bnmtoyr0ou
	  section.data(25).logicalSrcIdx = 24;
	  section.data(25).dtTransOffset = 31;
	
	  ;% rtB.ji3tio223s
	  section.data(26).logicalSrcIdx = 25;
	  section.data(26).dtTransOffset = 32;
	
	  ;% rtB.jf1eral2ve
	  section.data(27).logicalSrcIdx = 26;
	  section.data(27).dtTransOffset = 33;
	
	  ;% rtB.ngbxrb2p33
	  section.data(28).logicalSrcIdx = 27;
	  section.data(28).dtTransOffset = 37;
	
	  ;% rtB.c33unhaobk
	  section.data(29).logicalSrcIdx = 28;
	  section.data(29).dtTransOffset = 41;
	
	  ;% rtB.l5s1awmxqx
	  section.data(30).logicalSrcIdx = 29;
	  section.data(30).dtTransOffset = 42;
	
	  ;% rtB.nue240oiht
	  section.data(31).logicalSrcIdx = 30;
	  section.data(31).dtTransOffset = 43;
	
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
      
	  ;% rtDW.ox0fllt2el
	  section.data(1).logicalSrcIdx = 0;
	  section.data(1).dtTransOffset = 0;
	
      nTotData = nTotData + section.nData;
      dworkMap.sections(1) = section;
      clear section
      
      section.nData     = 17;
      section.data(17)  = dumData; %prealloc
      
	  ;% rtDW.dbuidsfmth.LoggedData
	  section.data(1).logicalSrcIdx = 1;
	  section.data(1).dtTransOffset = 0;
	
	  ;% rtDW.gm2yjyanxh.LoggedData
	  section.data(2).logicalSrcIdx = 2;
	  section.data(2).dtTransOffset = 1;
	
	  ;% rtDW.pd1kq3bjlk.LoggedData
	  section.data(3).logicalSrcIdx = 3;
	  section.data(3).dtTransOffset = 2;
	
	  ;% rtDW.ifwd1amcve.LoggedData
	  section.data(4).logicalSrcIdx = 4;
	  section.data(4).dtTransOffset = 3;
	
	  ;% rtDW.fcc0uv3oo0.LoggedData
	  section.data(5).logicalSrcIdx = 5;
	  section.data(5).dtTransOffset = 4;
	
	  ;% rtDW.fet3nngant.LoggedData
	  section.data(6).logicalSrcIdx = 6;
	  section.data(6).dtTransOffset = 5;
	
	  ;% rtDW.hwrxgbpfob.LoggedData
	  section.data(7).logicalSrcIdx = 7;
	  section.data(7).dtTransOffset = 6;
	
	  ;% rtDW.nnbpbghk5r.LoggedData
	  section.data(8).logicalSrcIdx = 8;
	  section.data(8).dtTransOffset = 7;
	
	  ;% rtDW.pvfh3hvin4.LoggedData
	  section.data(9).logicalSrcIdx = 9;
	  section.data(9).dtTransOffset = 8;
	
	  ;% rtDW.ed21y4p5jz.LoggedData
	  section.data(10).logicalSrcIdx = 10;
	  section.data(10).dtTransOffset = 9;
	
	  ;% rtDW.fnl2bzwupn.LoggedData
	  section.data(11).logicalSrcIdx = 11;
	  section.data(11).dtTransOffset = 10;
	
	  ;% rtDW.euqmzld2p5.LoggedData
	  section.data(12).logicalSrcIdx = 12;
	  section.data(12).dtTransOffset = 11;
	
	  ;% rtDW.lmen2irgnh.TimePtr
	  section.data(13).logicalSrcIdx = 13;
	  section.data(13).dtTransOffset = 12;
	
	  ;% rtDW.kxuk4bilk3.TimePtr
	  section.data(14).logicalSrcIdx = 14;
	  section.data(14).dtTransOffset = 13;
	
	  ;% rtDW.jmmcgd4r4a.TimePtr
	  section.data(15).logicalSrcIdx = 15;
	  section.data(15).dtTransOffset = 14;
	
	  ;% rtDW.mzmo2dtgp1.TimePtr
	  section.data(16).logicalSrcIdx = 16;
	  section.data(16).dtTransOffset = 15;
	
	  ;% rtDW.kbnckqutms.LoggedData
	  section.data(17).logicalSrcIdx = 17;
	  section.data(17).dtTransOffset = 16;
	
      nTotData = nTotData + section.nData;
      dworkMap.sections(2) = section;
      clear section
      
      section.nData     = 1;
      section.data(1)  = dumData; %prealloc
      
	  ;% rtDW.ic2yhbr32w
	  section.data(1).logicalSrcIdx = 18;
	  section.data(1).dtTransOffset = 0;
	
      nTotData = nTotData + section.nData;
      dworkMap.sections(3) = section;
      clear section
      
      section.nData     = 13;
      section.data(13)  = dumData; %prealloc
      
	  ;% rtDW.ihm14vuvou
	  section.data(1).logicalSrcIdx = 19;
	  section.data(1).dtTransOffset = 0;
	
	  ;% rtDW.mizp5pcuu3
	  section.data(2).logicalSrcIdx = 20;
	  section.data(2).dtTransOffset = 2;
	
	  ;% rtDW.oosdo3rh02.PrevIndex
	  section.data(3).logicalSrcIdx = 21;
	  section.data(3).dtTransOffset = 3;
	
	  ;% rtDW.fth52qijjp
	  section.data(4).logicalSrcIdx = 22;
	  section.data(4).dtTransOffset = 4;
	
	  ;% rtDW.j5kfgeryef
	  section.data(5).logicalSrcIdx = 23;
	  section.data(5).dtTransOffset = 6;
	
	  ;% rtDW.l2pbteie2e.PrevIndex
	  section.data(6).logicalSrcIdx = 24;
	  section.data(6).dtTransOffset = 7;
	
	  ;% rtDW.pshgckdv2q
	  section.data(7).logicalSrcIdx = 25;
	  section.data(7).dtTransOffset = 8;
	
	  ;% rtDW.nm2quutkdt
	  section.data(8).logicalSrcIdx = 26;
	  section.data(8).dtTransOffset = 10;
	
	  ;% rtDW.agvnfrpxdp.PrevIndex
	  section.data(9).logicalSrcIdx = 27;
	  section.data(9).dtTransOffset = 11;
	
	  ;% rtDW.mx32usueux
	  section.data(10).logicalSrcIdx = 28;
	  section.data(10).dtTransOffset = 12;
	
	  ;% rtDW.i3sszjlhmx
	  section.data(11).logicalSrcIdx = 29;
	  section.data(11).dtTransOffset = 16;
	
	  ;% rtDW.n0unfq1ojc.PrevIndex
	  section.data(12).logicalSrcIdx = 30;
	  section.data(12).dtTransOffset = 17;
	
	  ;% rtDW.dbgdqimhbs
	  section.data(13).logicalSrcIdx = 31;
	  section.data(13).dtTransOffset = 18;
	
      nTotData = nTotData + section.nData;
      dworkMap.sections(4) = section;
      clear section
      
      section.nData     = 5;
      section.data(5)  = dumData; %prealloc
      
	  ;% rtDW.nx03g0oqvx
	  section.data(1).logicalSrcIdx = 32;
	  section.data(1).dtTransOffset = 0;
	
	  ;% rtDW.k2y005jslo
	  section.data(2).logicalSrcIdx = 33;
	  section.data(2).dtTransOffset = 1;
	
	  ;% rtDW.lsikoznc1f
	  section.data(3).logicalSrcIdx = 34;
	  section.data(3).dtTransOffset = 2;
	
	  ;% rtDW.pvf1npkacw
	  section.data(4).logicalSrcIdx = 35;
	  section.data(4).dtTransOffset = 3;
	
	  ;% rtDW.p424nko4vw
	  section.data(5).logicalSrcIdx = 36;
	  section.data(5).dtTransOffset = 4;
	
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


  targMap.checksum0 = 2359241291;
  targMap.checksum1 = 787542146;
  targMap.checksum2 = 808962859;
  targMap.checksum3 = 974030338;

