/*
 * GENERATED FILE - DO NOT EDIT
 * (c) Code Red Technologies Ltd, 2008-2021
 * (c) NXP Semiconductors 2013-2021
 * Linker script memory definitions
 * Created from LinkMemoryTemplate
 * By LPCXpresso v7.2.0 [Build 153] [2014-05-19]  on Mon Jul 26 23:35:28 CEST 2021)
*/

MEMORY
{
  /* Define each memory region */
  MFlash32 (rx) : ORIGIN = 0x0, LENGTH = 0x8000 /* 32K bytes */
  RamLoc6 (rwx) : ORIGIN = 0x10000000, LENGTH = 0x1800 /* 6K bytes */
  RamUsb2 (rwx) : ORIGIN = 0x20004000, LENGTH = 0x800 /* 2K bytes */


}
  /* Define a symbol for the top of each memory region */
  __top_MFlash32 = 0x0 + 0x8000;
  __top_RamLoc6 = 0x10000000 + 0x1800;
  __top_RamUsb2 = 0x20004000 + 0x800;



