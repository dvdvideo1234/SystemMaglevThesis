/*
 * common.c
 *
 *  Created on: May 17, 2013
 *      Author: DVD
 */
#define MYONE 100000
#define MYONE_PWM 100

static u8   NumberBuffer[36];
static u8*  Nstr;
static s32  NumberToConvert;
static u8   FlagSign;
static s32  A,B,C;


void Delay(u32 len)
{
	while(len--);
}

void Delayms(u32 ms)
{
	ms *= 11538;
	while(ms--);
}

s32 Abs(s32 Num)
{
   if(Num >= 0) return Num;
   return -Num;
}

u32 MirrorNum(u32 Num)
{
	   u8 i;
	   u32 Out = 0;
	   for(i=0;i<32;i++)
	         {
	             Out >>= 1;
	             Out |= (Num & 0x80000000);
	             Num <<= 1;
	         }
	   return Out;
}


s32 FracNum(s32 Num, s32 C, s32 Z)
{
    Num = Num * C;
    Num = Num / Z;
	return Num;
}

u8 DigitsCount(s32 Num, u32 NumBase)
{
    u8 Cnt = 0;
    while(Num)
    {
        Cnt++;
        Num /= NumBase;
    }
    return Cnt;
}

u8 StrLen(u8* Str)
{
	u8 Ind;
    while(Str[Ind++] != '\0');
	return Ind-1;
}

void PutDigit(u8 Digit)
{
    *(--Nstr) = Digit;
}

s32 InitNum(s32 Num)
{
	FlagSign = 0;
	Nstr = &NumberBuffer[36];
	PutDigit('\0');
	if(Num < 0)
    {
           FlagSign = '-';
           return -Num;
    }
    return Num;
}

void PutOneDigit(u32 NumBase)
{
     u8 Tmp = (NumberToConvert % NumBase);
     if(Tmp > 9){ Tmp += 7; }
  	 PutDigit(Tmp + 48);
  	 NumberToConvert /= NumBase;
}

void PutNumberString(u32 NumBase)
{
    while(NumberToConvert)
    {
    	PutOneDigit(NumBase);
    }
}

u8* Bin2Str(s32 Num)
{
    u8 RestToFill;
    u8 Flag = 0;
    if(Num < 0)
    {
       Num &= 0x7FFFFFFF;
       Flag = 0xFF;
    }
	NumberToConvert = InitNum(Num);
	PutNumberString(2);
	RestToFill = 32 - DigitsCount(Num,2);
	while(RestToFill)
    {
       if(Flag && (RestToFill == 1))
       {
           PutDigit('1');
       }else{
           PutDigit('0');
       }
       RestToFill--;
    }
    return Nstr;
}

u8* Dec2NcharsStr(s32 Num ,u8 DigCount)
{
	u8 ExtraDigits = 0;
	NumberToConvert = InitNum(Num);
	ExtraDigits = DigCount - DigitsCount(Num,10);
	PutNumberString(10);
	while(ExtraDigits--){ PutDigit(95); }
	return Nstr;
}

u8* Dec2Str(s32 Num)
{
	NumberToConvert = InitNum(Num);
	if(Num){  PutNumberString(10); }
	else   {  PutDigit('0');       }
    if(FlagSign){ PutDigit(FlagSign); }
    return Nstr;
}

u8* FloatDec2Str(s32 Num)
{
	u32 FractionalCnt = DigitsCount(MYONE,10)-1;
	NumberToConvert = InitNum(Num);
	if(NumberToConvert)
	{
   		while(FractionalCnt--){ PutOneDigit(10); }
   		PutDigit('.');
   		PutNumberString(10);
        if(Abs(Num) < MYONE){ PutDigit('0'); }
	}else{
		while(FractionalCnt--){ PutDigit('0'); }
		PutDigit('.');
		PutDigit('0');
	}
    if(FlagSign){ PutDigit(FlagSign); }
    return Nstr;
}

u8* PWM2Str(u32 Num)
{
	if(Num > MAX_VALUE_PWM){ Num = MAX_VALUE_PWM; }
	Num = 100*MYONE_PWM*Num;
	Num = Num / MAX_VALUE_PWM;
	NumberToConvert = InitNum(Num);
	// PutDigit('%');
	if(NumberToConvert)
	{
   		PutOneDigit(10);
   		PutOneDigit(10);
   		PutDigit('.');
   		PutNumberString(10);
        if(Abs(Num) < MYONE_PWM){ PutDigit('0'); }
	}else{
		PutDigit('0');
		PutDigit('0');
		PutDigit('.');
		PutDigit('0');
	}
	return Nstr;
}

s32 SumArr(s32* Arr, u32 ArrLen)
{
	s32 Sum = 0;
	while(ArrLen)
    {
       Sum += Arr[ArrLen-1];
       ArrLen--;
    }
    return Sum;
}

void FillArr(s32* Arr, u32 ArrLen,s32 Value)
{
    while(ArrLen)
    {
       Arr[ArrLen-1] = Value;
       ArrLen--;
    }
}

s32 NumPowOf(s32 Num, s32 Pow)
{
    s32 Data = Num;
    if(Pow <  0){ return 0; }
    if(Pow == 0){ return 1; }
    if(Pow > 0)
    {
        Pow--;
        while(Pow)
        {
            Data *= Num;
            Pow--;
        }
    }
    return Data;
}


s32 QuadraticEquation(s32 Num)
{
    if(Num < 0){ return 0; }
    if(Num > 1244){ Num = 1244; }
    // 1234567 = 1.234567
    A = 591;
    B = 1725684;
    C = 190058487;
    // Calc Result
    A = A*NumPowOf(Num,2);
    A = A/100;
    B = Num * B;
    B = B/100;
    C = C/100;
    Num = MYONE/10000;
    return Num*(A+B+C);
}

s32 LinearEquation(s32 Num)
{
    if(Num < 0){ return 0; }
    if(Num > 70000){ Num = 70000; }
    B =  27354;
    C = -2200000;
    B = (B*Num);
    Num = MYONE/10000;
    return Num*(B+C);
}
