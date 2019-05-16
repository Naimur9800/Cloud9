#ifdef IAP_PORTION
const u8 holitech_fw[]=
{ 
#include "holitech_V2610_2BB2_C60B.i"
};
struct vendor_map
{
	int vendor_id;
	char vendor_name[30];
	uint8_t* fw_array;
};
const struct vendor_map g_vendor_map[]=
{
	{0x2610,"HOLITECH",holitech_fw},
	{0x2BB2,"HOLITECH",holitech_fw},
};

#endif/*IAP_PORTION*/
