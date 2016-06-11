
typedef struct mypid
{
  s32 	gP;
  s32 	gI;
  s32 	gD;
  s32	vP;
  s32	vI;
  s32	vD;
  s32   R;
  s32 	Y;
  s32   YRow;
  s32   YOffset;
  s32 	Err;
  s32	ErrOld;
  s32 	SUp;
  s32 	SDn;
  s32   PcutU;
  s32   PcutD;
  s32 	DcutU;
  s32 	DcutD;

  // Flag Info Bits
  // BitNo    Meaning -> 1 On;  0 Off

  // 0  		1     Manual
  // 1  		1     Integral Enabled
  // 2  		1	  Err = Y - R;

  u32  	FlagInfo;
  s32 	ConOffset;
  s32 	Control;
} MYPID;

void PIDReset(MYPID* pid)
{
	pid->ConOffset  = 0;
	pid->Control	= 0;
	pid->DcutD		= 0;
	pid->DcutU		= 0;
	pid->Err		= 0;
	pid->ErrOld		= 0;
	pid->FlagInfo	= 0;
	pid->PcutD		= 0;
	pid->PcutU		= 0;
	pid->R			= 0;
	pid->SDn		= 0;
	pid->SUp		= 0;
	pid->Y			= 0;
	pid->YOffset	= 0;
	pid->YRow		= 0;
	pid->gD			= 0;
	pid->gI			= 0;
	pid->gP			= 0;
	pid->vD			= 0;
	pid->vI			= 0;
	pid->vP			= 0;

}

void PIDSetControlOffset(MYPID* pid, s32 Offset)
{
	pid->ConOffset = Offset;
}

void PIDSetSat(MYPID* pid, s32 SatU, s32 SatD, s32 PCutU, s32 PCutD, s32 DCutU, s32 DCutD)
{
	pid->SUp = SatU;
	pid->SDn = SatD;
	pid->PcutU = PCutU;
	pid->PcutD = PCutD;
	pid->DcutU = DCutU;
	pid->DcutD = DCutD;
	return;
}

void PIDSetInfo(MYPID* pid, s32 Info){
	pid->FlagInfo = 0;
	pid->FlagInfo = Info;
	return;
}

void PIDSetPara(MYPID* pid, s32 PGain, s32 IGain, s32 DGain)
{
	pid->gP  = PGain;
	pid->gI  = IGain;
	pid->gD  = DGain;
	return;
}

s32 PIDGetControl(MYPID* pid){ return pid->Control; }

void PIDDo(MYPID* pid)
{
	// 0..4095
	s32 ControlTest;

	pid->ErrOld = pid->Err;

	pid->Y = pid->YRow - pid->YOffset;

	// If Positive Feedback stabilization
	if(pid->FlagInfo & 4){ pid->Err =  pid->Y - pid->R; }
	else				 { pid->Err =  pid->R - pid->Y; }

	// P
	pid->vP = pid->gP * pid->Err;
	if(pid->vP > pid->PcutU){pid->vP = pid->PcutU; }
	if(pid->vP < pid->PcutD){pid->vP = pid->PcutD; }

	// I
	if((pid->gI > 0) && (pid->FlagInfo & 2) && (pid->Err != 0))
	{
		pid->vI += pid->gI * pid->Err;
	}

	// D
	if((pid->gD > 0) && (pid->Err != pid->ErrOld))
	{
		pid->vD = pid->gD *(pid->Err - pid->ErrOld);
		// D Cut
		if(pid->vD > pid->DcutU){pid->vD = pid->DcutU; }
		if(pid->vD < pid->DcutD){pid->vD = pid->DcutD; }
	}
	ControlTest = pid->vP + pid->vI + pid->vD;
	// Anti WindUp
	if(ControlTest > pid->SUp){ pid->Control = pid->SUp; pid->FlagInfo &= ~2; }
	if(ControlTest < pid->SDn){ pid->Control = pid->SDn; pid->FlagInfo &= ~2; }
	if((ControlTest >= pid->SDn) && (ControlTest <= pid->SUp))
	{
		// Enable I-term
		pid->FlagInfo |= 2;
		pid->Control  = ControlTest;
	}
}

void PIDMaglevInit(MYPID* pid)
{
	//  100563,   // Kr
    //       1,   // Ki
	//   50033    // Kd

	// 210563,    // Kr
	//      1,    // Ki
	//  87033     // Kd

	// 420563,    // Kr
    //      1,    // Ki
	// 157033     // Kd

	PIDReset(pid);
	PIDSetInfo(pid,6);
	PIDSetPara(pid,
		    743056,   // Kr
			     3,   // Ki
		     41431    // Kd
			  );
	PIDSetSat(pid,
	              2000*MYONE,                 // Sat PID Up
	             -2000*MYONE,                 // Sat PID Down
		          1800*MYONE,                 // Sat P   Up
		         -1800*MYONE,                 // Sat P   Down
		          1400*MYONE,                 // Sat D   Up
		         -1400*MYONE                  // Sat D   Down
		  );
	PIDSetControlOffset(pid,2000*MYONE);
}


