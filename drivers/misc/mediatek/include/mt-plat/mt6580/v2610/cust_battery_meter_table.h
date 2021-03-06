#ifndef _CUST_BATTERY_METER_TABLE_H
#define _CUST_BATTERY_METER_TABLE_H

#include <mt-plat/battery_meter.h>

/* ============================================================*/
/* define*/
/* ============================================================*/
#define BAT_NTC_10 1
#define BAT_NTC_47 0

#if (BAT_NTC_10 == 1)
#define RBAT_PULL_UP_R	16900
#define RBAT_PULL_DOWN_R	27000
#endif

#if (BAT_NTC_47 == 1)
#define RBAT_PULL_UP_R	61900
#define RBAT_PULL_DOWN_R	100000
#endif
#define RBAT_PULL_UP_VOLT	1800



/* ============================================================*/
/* ENUM*/
/* ============================================================*/

/* ============================================================*/
/* structure*/
/* ============================================================*/

/* ============================================================*/
/* typedef*/
/* ============================================================*/
typedef struct _BATTERY_PROFILE_STRUCT {
	signed int percentage;
	signed int voltage;
} BATTERY_PROFILE_STRUCT, *BATTERY_PROFILE_STRUCT_P;

typedef struct _R_PROFILE_STRUCT {
	signed int resistance; /* Ohm*/
	signed int voltage;
} R_PROFILE_STRUCT, *R_PROFILE_STRUCT_P;

typedef enum {
	T1_0C,
	T2_25C,
	T3_50C
} PROFILE_TEMPERATURE;

/* ============================================================*/
/* External Variables*/
/* ============================================================*/

/* ============================================================*/
/* External function*/
/* ============================================================*/

/* ============================================================*/
/* <DOD, Battery_Voltage> Table*/
/* ============================================================*/
#if (BAT_NTC_10 == 1)
	BATT_TEMPERATURE Batt_Temperature_Table[] = {
        {-30,127476},
        {-25,96862},
        {-20,74354},
        {-15,57626},
        {-10,45068},
        { -5,35548},
        {  0,28267},
        {  5,22650},
        { 10,18280},
        { 15,14855},
        { 20,12151},
        { 25,10000},
        { 30,8279},
        { 35,6892},
        { 40,5768},
        { 45,4852},
        { 50,4101},
        { 55,3483},
        { 60,2970},
        { 65,2544},
        { 70,2188},
        { 75,1889},
        { 80,1637}
};
#endif

#if (BAT_NTC_47 == 1)
	BATT_TEMPERATURE Batt_Temperature_Table[] = {
		{-20, 483954},
		{-15, 360850},
		{-10, 271697},
		{ -5, 206463},
		{  0, 158214},
		{  5, 122259},
		{ 10, 95227},
		{ 15, 74730},
		{ 20, 59065},
		{ 25, 47000},
		{ 30, 37643},
		{ 35, 30334},
		{ 40, 24591},
		{ 45, 20048},
		{ 50, 16433},
		{ 55, 13539},
		{ 60, 11210}
	};
#endif

/* T0 -10C*/
BATTERY_PROFILE_STRUCT battery_profile_t0[] =
{
    { 0  , 4336},
    { 2  , 4305},
    { 4  , 4279},
    { 6  , 4257},
    { 8  , 4236},
    { 10 , 4217},
    { 12 , 4199},
    { 14 , 4181},
    { 16 , 4164},
    { 18 , 4146},
    { 20 , 4129},
    { 22 , 4114},
    { 24 , 4102},
    { 26 , 4089},
    { 28 , 4073},
    { 30 , 4047},
    { 32 , 4016},
    { 34 , 3991},
    { 36 , 3972},
    { 38 , 3956},
    { 40 , 3943},
    { 42 , 3932},
    { 44 , 3921},
    { 46 , 3911},
    { 48 , 3900},
    { 50 , 3889},
    { 52 , 3879},
    { 54 , 3869},
    { 56 , 3859},
    { 58 , 3850},
    { 60 , 3842},
    { 62 , 3835},
    { 64 , 3828},
    { 66 , 3821},
    { 68 , 3815},
    { 70 , 3809},
    { 72 , 3805},
    { 74 , 3800},
    { 76 , 3798},
    { 78 , 3794},
    { 80 , 3792},
    { 82 , 3790},
    { 84 , 3788},
    { 86 , 3785},
    { 88 , 3783},
    { 90 , 3780},
    { 92 , 3777},
    { 94 , 3773},
    { 96 , 3769},
    { 98 , 3764},
    { 100, 3758},
    { 100, 3752},
    { 100, 3747},
    { 100, 3743},
    { 100, 3739},
    { 100, 3735},
    { 100, 3731},
    { 100, 3728},
    { 100, 3724},
    { 100, 3720},
    { 100, 3717},
    { 100, 3714},
    { 100, 3711},
    { 100, 3710},
    { 100, 3708},
    { 100, 3706},
    { 100, 3705},
    { 100, 3709},
    { 100, 3704},
    { 100, 3704}
};

/* T1 0C */
BATTERY_PROFILE_STRUCT battery_profile_t1[] =
{
    { 0  , 4307},
    { 1  , 4284},
    { 4  , 4264},
    { 7  , 4246},
    { 9  , 4228},
    { 11 , 4210},
    { 13 , 4193},
    { 16 , 4177},
    { 18 , 4160},
    { 20 , 4144},
    { 22 , 4127},
    { 24 , 4110},
    { 27 , 4098},
    { 29 , 4087},
    { 31 , 4075},
    { 33 , 4054},
    { 36 , 4023},
    { 38 , 3997},
    { 40 , 3978},
    { 42 , 3964},
    { 44 , 3951},
    { 47 , 3939},
    { 49 , 3928},
    { 51 , 3916},
    { 53 , 3904},
    { 56 , 3891},
    { 58 , 3880},
    { 60 , 3869},
    { 62 , 3859},
    { 64 , 3850},
    { 67 , 3842},
    { 69 , 3834},
    { 71 , 3827},
    { 73 , 3821},
    { 76 , 3814},
    { 78 , 3809},
    { 80 , 3804},
    { 82 , 3799},
    { 84 , 3795},
    { 87 , 3792},
    { 89 , 3789},
    { 91 , 3788},
    { 93 , 3785},
    { 96 , 3783},
    { 98 , 3780},
    { 100, 3777},
    { 100, 3774},
    { 100, 3769},
    { 100, 3764},
    { 100, 3759},
    { 100, 3752},
    { 100, 3743},
    { 100, 3735},
    { 100, 3726},
    { 100, 3714},
    { 100, 3704},
    { 100, 3699},
    { 100, 3695},
    { 100, 3693},
    { 100, 3689},
    { 100, 3683},
    { 100, 3674},
    { 100, 3658},
    { 100, 3641},
    { 100, 3623},
    { 100, 3606},
    { 100 ,3589 },
    { 100 ,3575 },
    { 100 ,3575 },
    { 100 ,3575 }
};

/* T2 25C*/
BATTERY_PROFILE_STRUCT battery_profile_t2[] = 
{
    { 0  , 4304},
    { 1  , 4281},
    { 3  , 4258},
    { 5  , 4236},
    { 6  , 4214},
    { 8  , 4193},
    { 10 , 4174},
    { 11 , 4157},
    { 13 , 4140},
    { 14 , 4125},
    { 16 , 4110},
    { 18 , 4095},
    { 19 , 4079},
    { 21 , 4062},
    { 22 , 4045},
    { 24 , 4029},
    { 26 , 4012},
    { 27 , 3995},
    { 29 , 3978},
    { 30 , 3962},
    { 32 , 3948},
    { 34 , 3937},
    { 35 , 3926},
    { 37 , 3916},
    { 38 , 3906},
    { 40 , 3895},
    { 41 , 3883},
    { 43 , 3871},
    { 45 , 3860},
    { 46 , 3849},
    { 48 , 3839},
    { 49 , 3830},
    { 51 , 3822},
    { 53 , 3814},
    { 54 , 3809},
    { 56 , 3804},
    { 57 , 3798},
    { 59 , 3793},
    { 61 , 3788},
    { 62 , 3783},
    { 64 , 3779},
    { 65 , 3775},
    { 67 , 3772},
    { 69 , 3768},
    { 70 , 3764},
    { 72 , 3758},
    { 73 , 3754},
    { 75 , 3749},
    { 77 , 3745},
    { 78 , 3741},
    { 80 , 3737},
    { 81 , 3731},
    { 83 , 3723},
    { 85 , 3715},
    { 86 , 3707},
    { 88 , 3699},
    { 89 , 3690},
    { 91 , 3684},
    { 93 , 3679},
    { 94 , 3673},
    { 96 , 3660},
    { 98 , 3627},
    { 99 , 3563},
    { 100 ,3322 },
    { 100 ,3329 },
    { 100 ,3312 },
    { 100 ,3304 },
    { 100 ,3299 },
    { 100 ,3296 },
    { 100 ,3296 }
};

/* T3 50C*/
BATTERY_PROFILE_STRUCT battery_profile_t3[] =
{
    { 0  , 4301},
    { 1  , 4278},
    { 3  , 4255},
    { 5  , 4233},
    { 6  , 4211},
    { 8  , 4189},
    { 10 , 4170},
    { 11 , 4154},
    { 13 , 4137},
    { 14 , 4120},
    { 16 , 4104},
    { 18 , 4088},
    { 19 , 4072},
    { 21 , 4056},
    { 22 , 4041},
    { 24 , 4026},
    { 26 , 4012},
    { 27 , 3997},
    { 29 , 3982},
    { 30 , 3968},
    { 32 , 3954},
    { 34 , 3942},
    { 35 , 3930},
    { 37 , 3918},
    { 38 , 3907},
    { 40 , 3893},
    { 42 , 3878},
    { 43 , 3864},
    { 45 , 3852},
    { 46 , 3842},
    { 48 , 3832},
    { 50 , 3824},
    { 51 , 3815},
    { 53 , 3808},
    { 54 , 3802},
    { 56 , 3796},
    { 58 , 3790},
    { 59 , 3785},
    { 61 , 3779},
    { 62 , 3774},
    { 64 , 3770},
    { 66 , 3765},
    { 67 , 3761},
    { 69 , 3757},
    { 71 , 3749},
    { 72 , 3742},
    { 74 , 3735},
    { 75 , 3729},
    { 77 , 3722},
    { 79 , 3716},
    { 80 , 3711},
    { 82 , 3705},
    { 83 , 3697},
    { 85 , 3691},
    { 87 , 3684},
    { 88 , 3678},
    { 90 , 3673},
    { 91 , 3669},
    { 93 , 3665},
    { 95 , 3660},
    { 96 , 3638},
    { 98 , 3594},
    { 99 , 3488},
    { 100 ,3304 },
    { 100 ,3280 },
    { 100 ,3271 },
    { 100 ,3268 },
    { 100 ,3266 },
    { 100 ,3266 },
    { 100 ,3266 }
};

/* battery profile for actual temperature. The size should be the same as T1, T2 and T3*/
BATTERY_PROFILE_STRUCT battery_profile_temperature[] =
{
    {0, 0},
    {0, 0},
    {0, 0},
    {0, 0},
    {0, 0},
    {0, 0},
    {0, 0},
    {0, 0},
    {0, 0},
    {0, 0},
    {0, 0},
    {0, 0},
    {0, 0},
    {0, 0},
    {0, 0},
    {0, 0},
    {0, 0},
    {0, 0},
    {0, 0},
    {0, 0},
    {0, 0},
    {0, 0},
    {0, 0},
    {0, 0},
    {0, 0},
    {0, 0},
    {0, 0},
    {0, 0},
    {0, 0},
    {0, 0},
    {0, 0},
    {0, 0},
    {0, 0},
    {0, 0},
    {0, 0},
    {0, 0},
    {0, 0},
    {0, 0},
    {0, 0},
    {0, 0},
    {0, 0},
    {0, 0},
    {0, 0},
    {0, 0},
    {0, 0},
    {0, 0},
    {0, 0},
    {0, 0},
    {0, 0},
    {0, 0},
    {0, 0},
    {0, 0},
    {0, 0},
    {0, 0},
    {0, 0},
    {0, 0},
    {0, 0},
    {0, 0},
    {0, 0},
    {0, 0},
    {0, 0},
    {0, 0},
    {0, 0},
    {0, 0},
    {0, 0},
    {0, 0},
    {0, 0},
    {0, 0},
    {0, 0},
    {0, 0}
};

/* ============================================================*/
/* <Rbat, Battery_Voltage> Table*/
/* ============================================================*/
/* T0 -10C*/
R_PROFILE_STRUCT r_profile_t0[] =
{
    { 1503 , 4336 },
    { 1503 , 4305 },
    { 1480 , 4279 },
    { 1448 , 4257 },
    { 1413 , 4236 },
    { 1383 , 4217 },
    { 1355 , 4199 },
    { 1328 , 4181 },
    { 1303 , 4164 },
    { 1280 , 4146 },
    { 1260 , 4129 },
    { 1245 , 4114 },
    { 1238 , 4102 },
    { 1233 , 4089 },
    { 1225 , 4073 },
    { 1198 , 4047 },
    { 1160 , 4016 },
    { 1138 , 3991 },
    { 1125 , 3972 },
    { 1115 , 3956 },
    { 1110 , 3943 },
    { 1108 , 3932 },
    { 1108 , 3921 },
    { 1105 , 3911 },
    { 1100 , 3900 },
    { 1098 , 3889 },
    { 1093 , 3879 },
    { 1093 , 3869 },
    { 1088 , 3859 },
    { 1090 , 3850 },
    { 1093 , 3842 },
    { 1098 , 3835 },
    { 1103 , 3828 },
    { 1105 , 3821 },
    { 1108 , 3815 },
    { 1113 , 3809 },
    { 1123 , 3805 },
    { 1128 , 3800 },
    { 1140 , 3798 },
    { 1145 , 3794 },
    { 1160 , 3792 },
    { 1173 , 3790 },
    { 1190 , 3788 },
    { 1203 , 3785 },
    { 1223 , 3783 },
    { 1243 , 3780 },
    { 1263 , 3777 },
    { 1285 , 3773 },
    { 1310 , 3769 },
    { 1335 , 3764 },
    { 1360 , 3758 },
    { 1380 , 3752 },
    { 1368 , 3747 },
    { 1360 , 3743 },
    { 1350 , 3739 },
    { 1338 , 3735 },
    { 1328 , 3731 },
    { 1320 , 3728 },
    { 1313 , 3724 },
    { 1303 , 3720 },
    { 1293 , 3717 },
    { 1288 , 3714 },
    { 1280 , 3711 },
    { 1275 , 3710 },
    { 1273 , 3708 },
    { 1268 , 3706 },
    { 1265 , 3705 },
    { 1275 , 3709 },
    { 1260 , 3704 },
    { 1263 , 3704 }
};

/* T1 0C*/
R_PROFILE_STRUCT r_profile_t1[] = 
{
    { 449, 4307},
    { 449, 4284},
    { 446, 4264},
    { 442, 4246},
    { 442, 4228},
    { 439, 4210},
    { 436, 4193},
    { 434, 4177},
    { 431, 4160},
    { 430, 4144},
    { 429, 4127},
    { 424, 4110},
    { 427, 4098},
    { 434, 4087},
    { 439, 4075},
    { 433, 4054},
    { 417, 4023},
    { 408, 3997},
    { 404, 3978},
    { 402, 3964},
    { 402, 3951},
    { 399, 3939},
    { 398, 3928},
    { 393, 3916},
    { 390, 3904},
    { 385, 3891},
    { 383, 3880},
    { 382, 3869},
    { 382, 3859},
    { 383, 3850},
    { 385, 3842},
    { 386, 3834},
    { 389, 3827},
    { 392, 3821},
    { 395, 3814},
    { 399, 3809},
    { 404, 3804},
    { 407, 3799},
    { 410, 3795},
    { 412, 3792},
    { 417, 3789},
    { 426, 3788},
    { 429, 3785},
    { 437, 3783},
    { 445, 3780},
    { 453, 3777},
    { 465, 3774},
    { 474, 3769},
    { 486, 3764},
    { 500, 3759},
    { 515, 3752},
    { 531, 3743},
    { 553, 3735},
    { 578, 3726},
    { 603 ,3714 },
    { 633 ,3704 },
    { 671 ,3699 },
    { 717 ,3695 },
    { 722 ,3693 },
    { 715 ,3689 },
    { 706 ,3683 },
    { 693 ,3674 },
    { 671 ,3658 },
    { 646 ,3641 },
    { 622 ,3623 },
    { 595 ,3606 },
    { 570, 3589},
    { 551, 3575},
    { 551, 3575},
    { 551, 3575}
};

/* T2 25C*/
R_PROFILE_STRUCT r_profile_t2[] = 
{
    { 160 , 4304 },
    { 160 , 4281 },
    { 160 , 4258 },
    { 162 , 4236 },
    { 164 , 4214 },
    { 167 , 4193 },
    { 167 , 4174 },
    { 169 , 4157 },
    { 169 , 4140 },
    { 171 , 4125 },
    { 171 , 4110 },
    { 171 , 4095 },
    { 171 , 4079 },
    { 171 , 4062 },
    { 173 , 4045 },
    { 178 , 4029 },
    { 180 , 4012 },
    { 180 , 3995 },
    { 178 , 3978 },
    { 178 , 3962 },
    { 176 , 3948 },
    { 176 , 3937 },
    { 176 , 3926 },
    { 176 , 3916 },
    { 173 , 3906 },
    { 173 , 3895 },
    { 173 , 3883 },
    { 167 , 3871 },
    { 160 , 3860 },
    { 153 , 3849 },
    { 151 , 3839 },
    { 151 , 3830 },
    { 151 , 3822 },
    { 149 , 3814 },
    { 149 , 3809 },
    { 153 , 3804 },
    { 153 , 3798 },
    { 155 , 3793 },
    { 155 , 3788 },
    { 155 , 3783 },
    { 155 , 3779 },
    { 155 , 3775 },
    { 155 , 3772 },
    { 155 , 3768 },
    { 158 , 3764 },
    { 153 , 3758 },
    { 155 , 3754 },
    { 153 , 3749 },
    { 155 , 3745 },
    { 155 , 3741 },
    { 160 , 3737 },
    { 160 , 3731 },
    { 158 , 3723 },
    { 155 , 3715 },
    { 153 , 3707 },
    { 151 , 3699 },
    { 149 , 3690 },
    { 149 , 3684 },
    { 151 , 3679 },
    { 160 , 3673 },
    { 173 , 3660 },
    { 171 , 3627 },
    { 171 , 3563 },
    { 171 , 3322 },
    { 192 , 3329 },
    { 191 , 3312 },
    { 173 , 3304 },
    { 164 , 3299 },
    { 155 , 3296 },
    { 155 , 3296 }
};

/* T3 50C*/
R_PROFILE_STRUCT r_profile_t3[] = 
{
    { 113 , 4301 },
    { 113 , 4278 },
    { 113 , 4255 },
    { 115 , 4233 },
    { 115 , 4211 },
    { 115 , 4189 },
    { 113 , 4170 },
    { 115 , 4154 },
    { 115 , 4137 },
    { 115 , 4120 },
    { 118 , 4104 },
    { 118 , 4088 },
    { 118 , 4072 },
    { 120 , 4056 },
    { 123 , 4041 },
    { 125 , 4026 },
    { 128 , 4012 },
    { 125 , 3997 },
    { 125 , 3982 },
    { 130 , 3968 },
    { 130 , 3954 },
    { 133 , 3942 },
    { 133 , 3930 },
    { 133 , 3918 },
    { 135 , 3907 },
    { 138 , 3893 },
    { 133 , 3878 },
    { 125 , 3864 },
    { 123 , 3852 },
    { 123 , 3842 },
    { 120 , 3832 },
    { 120 , 3824 },
    { 115 , 3815 },
    { 115 , 3808 },
    { 115 , 3802 },
    { 118 , 3796 },
    { 120 , 3790 },
    { 125 , 3785 },
    { 125 , 3779 },
    { 128 , 3774 },
    { 130 , 3770 },
    { 128 , 3765 },
    { 128 , 3761 },
    { 130 , 3757 },
    { 125 , 3749 },
    { 125 , 3742 },
    { 128 , 3735 },
    { 128 , 3729 },
    { 125 , 3722 },
    { 125 , 3716 },
    { 125 , 3711 },
    { 128 , 3705 },
    { 128 , 3697 },
    { 133 , 3691 },
    { 130 , 3684 },
    { 130 , 3678 },
    { 128 , 3673 },
    { 128 , 3669 },
    { 130 , 3665 },
    { 135 , 3660 },
    { 125 , 3638 },
    { 150 , 3594 },
    { 165 , 3488 },
    { 223 , 3304 },
    { 165 , 3280 },
    { 143 , 3271 },
    { 135 , 3268 },
    { 128 , 3266 },
    { 128 , 3266 },
    { 128 , 3266 }
};

/* r-table profile for actual temperature. The size should be the same as T1, T2 and T3*/
R_PROFILE_STRUCT r_profile_temperature[] = 
{
    {0, 0},
    {0, 0},
    {0, 0},
    {0, 0},
    {0, 0},
    {0, 0},
    {0, 0},
    {0, 0},
    {0, 0},
    {0, 0},
    {0, 0},
    {0, 0},
    {0, 0},
    {0, 0},
    {0, 0},
    {0, 0},
    {0, 0},
    {0, 0},
    {0, 0},
    {0, 0},
    {0, 0},
    {0, 0},
    {0, 0},
    {0, 0},
    {0, 0},
    {0, 0},
    {0, 0},
    {0, 0},
    {0, 0},
    {0, 0},
    {0, 0},
    {0, 0},
    {0, 0},
    {0, 0},
    {0, 0},
    {0, 0},
    {0, 0},
    {0, 0},
    {0, 0},
    {0, 0},
    {0, 0},
    {0, 0},
    {0, 0},
    {0, 0},
    {0, 0},
    {0, 0},
    {0, 0},
    {0, 0},
    {0, 0},
    {0, 0},
    {0, 0},
    {0, 0},
    {0, 0},
    {0, 0},
    {0, 0},
    {0, 0},
    {0, 0},
    {0, 0},
    {0, 0},
    {0, 0},
    {0, 0},
    {0, 0},
    {0, 0},
    {0, 0},
    {0, 0},
    {0, 0},
    {0, 0},
    {0, 0},
    {0, 0},
    {0, 0}
};

/* ============================================================*/
/* function prototype*/
/* ============================================================*/
int fgauge_get_saddles(void);
BATTERY_PROFILE_STRUCT_P fgauge_get_profile(unsigned int temperature);

int fgauge_get_saddles_r_table(void);
R_PROFILE_STRUCT_P fgauge_get_profile_r_table(unsigned int temperature);

#endif

