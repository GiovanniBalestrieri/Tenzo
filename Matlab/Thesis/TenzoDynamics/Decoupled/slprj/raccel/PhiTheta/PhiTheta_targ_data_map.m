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
      section.nData     = 39;
      section.data(39)  = dumData; %prealloc
      
	  ;% rtP.SatThrust_UpperSat
	  section.data(1).logicalSrcIdx = 0;
	  section.data(1).dtTransOffset = 0;
	
	  ;% rtP.SatThrust_LowerSat
	  section.data(2).logicalSrcIdx = 1;
	  section.data(2).dtTransOffset = 1;
	
	  ;% rtP.Ps_A
	  section.data(3).logicalSrcIdx = 2;
	  section.data(3).dtTransOffset = 2;
	
	  ;% rtP.Ps_C
	  section.data(4).logicalSrcIdx = 3;
	  section.data(4).dtTransOffset = 4;
	
	  ;% rtP.FromWs_Time0
	  section.data(5).logicalSrcIdx = 4;
	  section.data(5).dtTransOffset = 6;
	
	  ;% rtP.FromWs_Data0
	  section.data(6).logicalSrcIdx = 5;
	  section.data(6).dtTransOffset = 12;
	
	  ;% rtP.u3sen80t_Amp
	  section.data(7).logicalSrcIdx = 6;
	  section.data(7).dtTransOffset = 18;
	
	  ;% rtP.u3sen80t_Bias
	  section.data(8).logicalSrcIdx = 7;
	  section.data(8).dtTransOffset = 19;
	
	  ;% rtP.u3sen80t_Freq
	  section.data(9).logicalSrcIdx = 8;
	  section.data(9).dtTransOffset = 20;
	
	  ;% rtP.u3sen80t_Phase
	  section.data(10).logicalSrcIdx = 9;
	  section.data(10).dtTransOffset = 21;
	
	  ;% rtP.Ps1_A
	  section.data(11).logicalSrcIdx = 10;
	  section.data(11).dtTransOffset = 22;
	
	  ;% rtP.Ps1_C
	  section.data(12).logicalSrcIdx = 11;
	  section.data(12).dtTransOffset = 23;
	
	  ;% rtP.Gain_Gain
	  section.data(13).logicalSrcIdx = 12;
	  section.data(13).dtTransOffset = 24;
	
	  ;% rtP.usen05t_Amp
	  section.data(14).logicalSrcIdx = 13;
	  section.data(14).dtTransOffset = 25;
	
	  ;% rtP.usen05t_Bias
	  section.data(15).logicalSrcIdx = 14;
	  section.data(15).dtTransOffset = 26;
	
	  ;% rtP.usen05t_Freq
	  section.data(16).logicalSrcIdx = 15;
	  section.data(16).dtTransOffset = 27;
	
	  ;% rtP.usen05t_Phase
	  section.data(17).logicalSrcIdx = 16;
	  section.data(17).dtTransOffset = 28;
	
	  ;% rtP.Ps1_A_e4tfyewuhv
	  section.data(18).logicalSrcIdx = 17;
	  section.data(18).dtTransOffset = 29;
	
	  ;% rtP.Ps1_C_pry1qgt440
	  section.data(19).logicalSrcIdx = 18;
	  section.data(19).dtTransOffset = 30;
	
	  ;% rtP.Gain_Gain_hvers1lhk2
	  section.data(20).logicalSrcIdx = 19;
	  section.data(20).dtTransOffset = 31;
	
	  ;% rtP.Ps_A_awgvmdvyjk
	  section.data(21).logicalSrcIdx = 20;
	  section.data(21).dtTransOffset = 32;
	
	  ;% rtP.Ps_C_axb255v5cb
	  section.data(22).logicalSrcIdx = 21;
	  section.data(22).dtTransOffset = 34;
	
	  ;% rtP.usen05t_Amp_jzvjbf0x3j
	  section.data(23).logicalSrcIdx = 22;
	  section.data(23).dtTransOffset = 36;
	
	  ;% rtP.usen05t_Bias_o1wafowfst
	  section.data(24).logicalSrcIdx = 23;
	  section.data(24).dtTransOffset = 37;
	
	  ;% rtP.usen05t_Freq_pv2suynl3g
	  section.data(25).logicalSrcIdx = 24;
	  section.data(25).dtTransOffset = 38;
	
	  ;% rtP.usen05t_Phase_eh1ecmzpx1
	  section.data(26).logicalSrcIdx = 25;
	  section.data(26).dtTransOffset = 39;
	
	  ;% rtP.UniformRandomNumber_Minimum
	  section.data(27).logicalSrcIdx = 26;
	  section.data(27).dtTransOffset = 40;
	
	  ;% rtP.UniformRandomNumber_Maximum
	  section.data(28).logicalSrcIdx = 27;
	  section.data(28).dtTransOffset = 41;
	
	  ;% rtP.UniformRandomNumber_Seed
	  section.data(29).logicalSrcIdx = 28;
	  section.data(29).dtTransOffset = 42;
	
	  ;% rtP.usen05t_Amp_onejysspqe
	  section.data(30).logicalSrcIdx = 29;
	  section.data(30).dtTransOffset = 43;
	
	  ;% rtP.usen05t_Bias_k0lwz1nqbi
	  section.data(31).logicalSrcIdx = 30;
	  section.data(31).dtTransOffset = 44;
	
	  ;% rtP.usen05t_Freq_lnfmendplc
	  section.data(32).logicalSrcIdx = 31;
	  section.data(32).dtTransOffset = 45;
	
	  ;% rtP.usen05t_Phase_a1d0o3bxhg
	  section.data(33).logicalSrcIdx = 32;
	  section.data(33).dtTransOffset = 46;
	
	  ;% rtP.UniformRandomNumber_Minimum_dr3kabpyij
	  section.data(34).logicalSrcIdx = 33;
	  section.data(34).dtTransOffset = 47;
	
	  ;% rtP.UniformRandomNumber_Maximum_duhygprnbe
	  section.data(35).logicalSrcIdx = 34;
	  section.data(35).dtTransOffset = 48;
	
	  ;% rtP.UniformRandomNumber_Seed_mavckwa2ow
	  section.data(36).logicalSrcIdx = 35;
	  section.data(36).dtTransOffset = 49;
	
	  ;% rtP.Cs1_A
	  section.data(37).logicalSrcIdx = 36;
	  section.data(37).dtTransOffset = 50;
	
	  ;% rtP.Cs1_C
	  section.data(38).logicalSrcIdx = 37;
	  section.data(38).dtTransOffset = 52;
	
	  ;% rtP.Cs1_D
	  section.data(39).logicalSrcIdx = 38;
	  section.data(39).dtTransOffset = 54;
	
      nTotData = nTotData + section.nData;
      paramMap.sections(1) = section;
      clear section
      
      section.nData     = 3;
      section.data(3)  = dumData; %prealloc
      
	  ;% rtP.ManualSwitch1_CurrentSetting
	  section.data(1).logicalSrcIdx = 39;
	  section.data(1).dtTransOffset = 0;
	
	  ;% rtP.ManualSwitch1_CurrentSetting_a1sxsoirxs
	  section.data(2).logicalSrcIdx = 40;
	  section.data(2).dtTransOffset = 1;
	
	  ;% rtP.ManualSwitch_CurrentSetting
	  section.data(3).logicalSrcIdx = 41;
	  section.data(3).dtTransOffset = 2;
	
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
      section.nData     = 11;
      section.data(11)  = dumData; %prealloc
      
	  ;% rtB.mk4dpnamwc
	  section.data(1).logicalSrcIdx = 0;
	  section.data(1).dtTransOffset = 0;
	
	  ;% rtB.o3ul0nzcwe
	  section.data(2).logicalSrcIdx = 1;
	  section.data(2).dtTransOffset = 1;
	
	  ;% rtB.dewu3oxpw1
	  section.data(3).logicalSrcIdx = 2;
	  section.data(3).dtTransOffset = 2;
	
	  ;% rtB.opuqvtwzzt
	  section.data(4).logicalSrcIdx = 3;
	  section.data(4).dtTransOffset = 3;
	
	  ;% rtB.awmfihve5b
	  section.data(5).logicalSrcIdx = 4;
	  section.data(5).dtTransOffset = 4;
	
	  ;% rtB.ngxpbqhzzn
	  section.data(6).logicalSrcIdx = 5;
	  section.data(6).dtTransOffset = 5;
	
	  ;% rtB.dgroez0siq
	  section.data(7).logicalSrcIdx = 6;
	  section.data(7).dtTransOffset = 6;
	
	  ;% rtB.kqoy4n5gjo
	  section.data(8).logicalSrcIdx = 7;
	  section.data(8).dtTransOffset = 7;
	
	  ;% rtB.pydoj1kzpi
	  section.data(9).logicalSrcIdx = 8;
	  section.data(9).dtTransOffset = 8;
	
	  ;% rtB.blhrfoy5vb
	  section.data(10).logicalSrcIdx = 9;
	  section.data(10).dtTransOffset = 9;
	
	  ;% rtB.pk1lzuz22z
	  section.data(11).logicalSrcIdx = 10;
	  section.data(11).dtTransOffset = 10;
	
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
    nTotSects     = 4;
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
      section.nData     = 2;
      section.data(2)  = dumData; %prealloc
      
	  ;% rtDW.fmiwcm0sxa
	  section.data(1).logicalSrcIdx = 0;
	  section.data(1).dtTransOffset = 0;
	
	  ;% rtDW.mqn0il2cto
	  section.data(2).logicalSrcIdx = 1;
	  section.data(2).dtTransOffset = 1;
	
      nTotData = nTotData + section.nData;
      dworkMap.sections(1) = section;
      clear section
      
      section.nData     = 6;
      section.data(6)  = dumData; %prealloc
      
	  ;% rtDW.bvmrb3pj3h.TimePtr
	  section.data(1).logicalSrcIdx = 2;
	  section.data(1).dtTransOffset = 0;
	
	  ;% rtDW.amidzcc4fy.LoggedData
	  section.data(2).logicalSrcIdx = 3;
	  section.data(2).dtTransOffset = 1;
	
	  ;% rtDW.lrhqyru4a2.LoggedData
	  section.data(3).logicalSrcIdx = 4;
	  section.data(3).dtTransOffset = 2;
	
	  ;% rtDW.hl4rq4szdl.LoggedData
	  section.data(4).logicalSrcIdx = 5;
	  section.data(4).dtTransOffset = 3;
	
	  ;% rtDW.pnnqiesode.LoggedData
	  section.data(5).logicalSrcIdx = 6;
	  section.data(5).dtTransOffset = 4;
	
	  ;% rtDW.hognujx3kq.LoggedData
	  section.data(6).logicalSrcIdx = 7;
	  section.data(6).dtTransOffset = 5;
	
      nTotData = nTotData + section.nData;
      dworkMap.sections(2) = section;
      clear section
      
      section.nData     = 2;
      section.data(2)  = dumData; %prealloc
      
	  ;% rtDW.hiarr4fcu2
	  section.data(1).logicalSrcIdx = 8;
	  section.data(1).dtTransOffset = 0;
	
	  ;% rtDW.nf5mz1kdkf
	  section.data(2).logicalSrcIdx = 9;
	  section.data(2).dtTransOffset = 1;
	
      nTotData = nTotData + section.nData;
      dworkMap.sections(3) = section;
      clear section
      
      section.nData     = 1;
      section.data(1)  = dumData; %prealloc
      
	  ;% rtDW.kuvxmwsxqi.PrevIndex
	  section.data(1).logicalSrcIdx = 10;
	  section.data(1).dtTransOffset = 0;
	
      nTotData = nTotData + section.nData;
      dworkMap.sections(4) = section;
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


  targMap.checksum0 = 2450082240;
  targMap.checksum1 = 2327675151;
  targMap.checksum2 = 2800790287;
  targMap.checksum3 = 2590810103;

