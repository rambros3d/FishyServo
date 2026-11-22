MEMORY
{
  /* Application slot: 104 KiB (0x0800_2000 .. 0x0801_C000) */
  /* Bank 2 Sectors 14 & 15: 16 KiB (0x0801_C000 .. 0x0802_0000) reserved for H5FlashStorage */
  FLASH : ORIGIN = 0x08002000, LENGTH = 104K
  /* Reserve top 32 bytes (0x2000_7FE0..0x2000_8000) for SoftResetMailbox */
  RAM   : ORIGIN = 0x20000000, LENGTH = 32736
}
_stack_start = 0x20007FE0;

