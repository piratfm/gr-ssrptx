/* -*- c++ -*- */
/*
 * Copyright 2003,2004,2005 Free Software Foundation, Inc.
 * This file is part of the SDRTX project.
 * 
 * The SDRTX project is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2, or (at your option)
 * any later version.
 * 
 * The SDRTX project is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 * 
 * You should have received a copy of the GNU General Public License
 * along with the SDRTX project; see the file COPYING.  If not, write to
 * the Free Software Foundation, Inc., 59 Temple Place - Suite 330,
 * Boston, MA 02111-1307, USA.
 */

#include "sdrtx_prims.h"
#include "sdrtx_commands.h"
#include "sdrtx_ids.h"
#include <usb.h>
#include <errno.h>
#include <stdio.h>
#include <unistd.h>
#include <stdlib.h>
#include <string.h>
#include <ctype.h>
#include <time.h>		// FIXME should check with autoconf (nanosleep)
#include <algorithm>

static const int FIRMWARE_HASH_SLOT	= 0;

static const int hash_slot_addr[2] = {
  SDRTX_HASH_SLOT_0_ADDR,
  SDRTX_HASH_SLOT_1_ADDR
};

//REMOVE static char *sdrtx_firmware_filename = "rxS_512.ihx";

static char *std_paths[] = {
  ".",
  "/usr/share/libgnuradio-ssrptx",
  "/usr/local/share/libgnuradio-ssrptx",
  0
};


static char *
find_file (const char *filename, int hw_rev) 
{
  char **sp = std_paths;
  static char path[1000];

  while (*sp){
      //snprintf (path, sizeof (path), "%s/rev%d/%s", *sp, hw_rev, filename); //don't need rev support now
      snprintf (path, sizeof (path), "%s/%s", *sp, filename);
    if (access (path, R_OK) == 0)
      return path;
    sp++;
  }
  return 0;
}

void
sdrtx_one_time_init ()  
{
  static bool first = true;
  sdrtx_load_status_t status;

  if (first){
    first = false;
    usb_init ();			// usb library init
    usb_find_busses ();
    usb_find_devices ();
        
  }
}

void
sdrtx_rescan ()
{
  usb_find_busses ();
  usb_find_devices ();
}


// ----------------------------------------------------------------
// Danger, big, fragile KLUDGE.  The problem is that we want to be
// able to get from a usb_dev_handle back to a usb_device, and the
// right way to do this is buried in a non-installed include file.

static struct usb_device *
dev_handle_to_dev (usb_dev_handle *udh)
{
  struct usb_dev_handle_kludge {
    int			 fd;
    struct usb_bus	*bus;
    struct usb_device	*device;
  };

  return ((struct usb_dev_handle_kludge *) udh)->device;
}

// ----------------------------------------------------------------

/*
 * q must be a real SDRTX, not an FX2.  Return its hardware rev number.
 */
static int
_sdrtx_hw_rev (struct usb_device *q)
{
    //return q->descriptor.bcdDevice & 0x00FF; //FIXME put rev in custom USB info
    return 0;
}

/*
 * q must be a real SDRTX, not an FX2.  Return true if it's configured.
 */
static bool
_sdrtx_configured_p (struct usb_device *q)
{
  return (q->descriptor.bcdDevice & 0xFF00) != 0; //FIXME is this valid?
}

bool
sdrtx_sdrtx_p (struct usb_device *q)
{
  return (q->descriptor.idVendor == USB_VID_FSF
	  && q->descriptor.idProduct == USB_PID_FSF_SDRTX);  //FIXME is this valid?
}

bool
sdrtx_fx2_p (struct usb_device *q)
{
  return (q->descriptor.idVendor == USB_VID_CYPRESS
	  && q->descriptor.idProduct == USB_PID_CYPRESS_FX2);
}

bool
sdrtx_unconfigured_sdrtx_p (struct usb_device *q)
{
  return sdrtx_sdrtx_p (q) && !_sdrtx_configured_p (q);
}

bool
sdrtx_configured_sdrtx_p (struct usb_device *q)
{
  return sdrtx_sdrtx_p (q) && _sdrtx_configured_p (q);
}

// ----------------------------------------------------------------

struct usb_device *
sdrtx_find_device (int nth, bool fx2_ok_p)
{
  struct usb_bus *p;
  struct usb_device *q;
  int	 n_found = 0;

  sdrtx_one_time_init ();
  
  p = usb_busses;
  while (p != NULL){
    q = p->devices;
    while (q != NULL){
      if (sdrtx_sdrtx_p (q) || (fx2_ok_p && sdrtx_fx2_p (q))){
	if (n_found == nth)	// return this one
	  return q;
	n_found++;		// keep looking
      }
      q = q->next;
    }
    p = p->next;
  }
  return 0;	// not found
}

static struct usb_dev_handle *
sdrtx_open_interface (struct usb_device *dev, int interface, int altinterface)
{
  struct usb_dev_handle *udh = usb_open (dev);
  if (udh == 0)
    {
      fprintf (stderr, "sdrtx_open_interface: usb_open failed");
      return 0;
    }
  
  if (dev != dev_handle_to_dev (udh)){
    fprintf (stderr, "%s:%d: internal error!\n", __FILE__, __LINE__);
    abort ();
  }

  if (usb_claim_interface (udh, interface) < 0){
    usb_close (udh);
    fprintf (stderr, "sdrtx_open_interface: usb_claim_interface failed\n");
    return 0;
  }

  if (usb_set_altinterface (udh, altinterface) < 0){
    usb_release_interface (udh, interface);
    fprintf (stderr, "sdrtx_open_interface: usb_set_altinterface failed\n");
    usb_close (udh);
    return 0;
  }

  return udh;
}

struct usb_dev_handle *
sdrtx_open_cmd_interface (struct usb_device *dev)
{
  return sdrtx_open_interface (dev, SDRTX_CMD_INTERFACE, SDRTX_CMD_ALTINTERFACE);
}

struct usb_dev_handle *
sdrtx_open_tx_interface (struct usb_device *dev)
{
  return sdrtx_open_interface (dev, SDRTX_TX_INTERFACE, SDRTX_TX_ALTINTERFACE);
}

bool
sdrtx_close_interface (struct usb_dev_handle *udh)
{
  // we're assuming that closing an interface automatically releases it.
  return usb_close (udh) == 0;
}

// ----------------------------------------------------------------
// write internal ram using Cypress vendor extension

static bool
write_internal_ram (struct usb_dev_handle *udh, unsigned char *buf,
		    int start_addr, size_t len)
{
  int addr;
  int n;
  int a;
  int quanta = MAX_EP0_PKTSIZE;

  for (addr = start_addr; addr < start_addr + (int) len; addr += quanta){
    n = len + start_addr - addr;
    if (n > quanta)
      n = quanta;

    a = usb_control_msg (udh, 0x40, 0xA0,
			 addr, 0, (char *)(buf + (addr - start_addr)), n, 1000);

    if (a < 0){
      fprintf(stderr,"write_internal_ram failed: %s\n", usb_strerror());
      return false;
    }
  }
  return true;
}

// ----------------------------------------------------------------
// whack the CPUCS register using the upload RAM vendor extension

static bool
reset_cpu (struct usb_dev_handle *udh, bool reset_p)
{
  unsigned char v;

  if (reset_p)
    v = 1;		// hold processor in reset
  else
    v = 0;	        // release reset

  return write_internal_ram (udh, &v, 0xE600, 1);
}

// ----------------------------------------------------------------
// Load intel format file into cypress FX2 (8051)

static bool
_sdrtx_load_firmware (struct usb_dev_handle *udh, const char *filename,
		     unsigned char hash[SDRTX_HASH_SIZE])
{
  FILE	*f = fopen (filename, "ra");
  if (f == 0){
    perror (filename);
    return false;
  }

  if (!reset_cpu (udh, true))	// hold CPU in reset while loading firmware
    goto fail;

  
  char s[1024];
  int length;
  int addr;
  int type;
  unsigned char data[256];
  unsigned char checksum, a;
  unsigned int b;
  int i;

  while (!feof(f)){
    fgets(s, sizeof (s), f); /* we should not use more than 263 bytes normally */
    if(s[0]!=':'){
      fprintf(stderr,"%s: invalid line: \"%s\"\n", filename, s);
      goto fail;
    }
    sscanf(s+1, "%02x", &length);
    sscanf(s+3, "%04x", &addr);
    sscanf(s+7, "%02x", &type);

    if(type==0){

      a=length+(addr &0xff)+(addr>>8)+type;
      for(i=0;i<length;i++){
	sscanf (s+9+i*2,"%02x", &b);
	data[i]=b;
	a=a+data[i];
      }

      sscanf (s+9+length*2,"%02x", &b);
      checksum=b;
      if (((a+checksum)&0xff)!=0x00){
	fprintf (stderr, "  ** Checksum failed: got 0x%02x versus 0x%02x\n", (-a)&0xff, checksum);
	goto fail;
      }
      if (!write_internal_ram (udh, data, addr, length))
	goto fail;
    }
    else if (type == 0x01){      // EOF
      break;
    }
    else if (type == 0x02){
      fprintf(stderr, "Extended address: whatever I do with it?\n");
      fprintf (stderr, "%s: invalid line: \"%s\"\n", filename, s);
      goto fail;
    }
  }

  // we jam the hash value into the FX2 memory before letting
  // the cpu out of reset.  When it comes out of reset it
  // may renumerate which will invalidate udh.

  
  
  if (!sdrtx_set_hash (udh, FIRMWARE_HASH_SLOT, hash))
      fprintf (stderr, "sdrtx: failed to write firmware hash slot\n");
  

  if (!reset_cpu (udh, false))		// take CPU out of reset
    goto fail;

  fclose (f);
  return true;

 fail:
  fclose (f);
  return false;
}

// ----------------------------------------------------------------
// write vendor extension command to SDRTX

static int
write_cmd (struct usb_dev_handle *udh,
	   int request, int value, int index,
	   unsigned char *bytes, int len)
{
  int	requesttype = (request & 0x80) ? VRT_VENDOR_IN : VRT_VENDOR_OUT;

  int r = usb_control_msg (udh, requesttype, request, value, index,
			   (char *) bytes, len, 1000);
  if (r < 0){
    // we get EPIPE if the firmware stalls the endpoint.
    if (errno != EPIPE)
      fprintf (stderr, "usb_control_msg failed: %s\n", usb_strerror ());
  }

  return r;
}

// ----------------------------------------------------------------

bool
sdrtx_set_hash (struct usb_dev_handle *udh, int which,
	       const unsigned char hash[SDRTX_HASH_SIZE])
{
    which &= 1;
  
  // we use the Cypress firmware down load command to jam it in.
  int r = usb_control_msg (udh, 0x40, 0xa0, hash_slot_addr[which], 0,
			   (char *) hash, SDRTX_HASH_SIZE, 1000);
  return r == SDRTX_HASH_SIZE;
}

bool
sdrtx_get_hash (struct usb_dev_handle *udh, int which, 
	       unsigned char hash[SDRTX_HASH_SIZE])
{
    which &= 1;
  
  // we use the Cypress firmware upload command to fetch it.
  int r = usb_control_msg (udh, 0xc0, 0xa0, hash_slot_addr[which], 0,
			   (char *) hash, SDRTX_HASH_SIZE, 1000);
  return r == SDRTX_HASH_SIZE;
}

//FIXME - implement? What does this do?
static bool
sdrtx_set_switch (struct usb_dev_handle *udh, int cmd_byte, bool on)
{
  return write_cmd (udh, cmd_byte, on, 0, 0, 0) == 0;
}

bool
sdrtx_set_sleep_bits (struct usb_dev_handle *udh, int bits, int mask) //FIXME is this relevant?
{
  switch (_sdrtx_hw_rev (dev_handle_to_dev (udh))){
  case 0:
    bits &= 0xf;
    mask &= 0xf;
    return write_cmd (udh, VRQ_SET_SLEEP_BITS, (mask << 8) | bits, 0, 0, 0) == 0;
  default:
    return true;
  }
}

// ----------------------------------------------------------------
// conditional load stuff

static int
hexval (unsigned char ch)
{
  if ('0' <= ch && ch <= '9')
    return ch - '0';

  if ('a' <= ch && ch <= 'f')
    return ch - 'a' + 10;

  if ('A' <= ch && ch <= 'F')
    return ch - 'A' + 10;

  return -1;
}

static bool
compute_hash (const char *filename, unsigned char hash[SDRTX_HASH_SIZE])
{
  memset (hash, 0, SDRTX_HASH_SIZE);

  char cmd[1000];
  unsigned char line[32];
  
  snprintf (cmd, sizeof (cmd), "md5sum %s", filename);
  FILE *fp = popen (cmd, "r");
  if (fp == 0)
    goto fail;
  
  if (fread (line, 1, 2*SDRTX_HASH_SIZE, fp) != 2*SDRTX_HASH_SIZE){
    perror ("sdrtx_prims: fread of md5sum pipe");
    goto fail;
  }

  if (pclose (fp) != 0){
    fp = 0;
    goto fail;
  }

  for (int i = 0; i < SDRTX_HASH_SIZE; i++){
    int	hi = hexval (line[2*i + 0]);
    int lo = hexval (line[2*i + 1]);

    if (hi < 0 || lo < 0)
      goto fail;

    hash[i] = (hi << 4) | lo;
  }

  return true;

 fail:
  if (fp)
    pclose (fp);
  return false;
}

static sdrtx_load_status_t
sdrtx_conditionally_load_something (struct usb_dev_handle *udh,
				   const char *filename,
				   bool force,
				   int slot,
				   bool loader (struct usb_dev_handle *,
						const char *,
						unsigned char [SDRTX_HASH_SIZE]))
{
  unsigned char file_hash[SDRTX_HASH_SIZE];
  unsigned char sdrtx_hash[SDRTX_HASH_SIZE];
  
  if (access (filename, R_OK) != 0){
    perror (filename);
    return ULS_ERROR;
  }

  if (!compute_hash (filename, file_hash))
    return ULS_ERROR;

  
  if (!force
      && sdrtx_get_hash (udh, slot, sdrtx_hash)
      && memcmp (file_hash, sdrtx_hash, SDRTX_HASH_SIZE) == 0)
    return ULS_ALREADY_LOADED;
  
  bool r = loader (udh, filename, file_hash);

  if (!r)
    return ULS_ERROR;

  return ULS_OK;
}

sdrtx_load_status_t
sdrtx_load_firmware (struct usb_dev_handle *udh,
		    const char *filename,
		    bool force)
{
  return sdrtx_conditionally_load_something (udh, filename, force,
					    FIRMWARE_HASH_SLOT,
					    _sdrtx_load_firmware);
}

usb_dev_handle *
sdrtx_open_nth_cmd_interface (int nth)
{
  struct usb_device *udev = sdrtx_find_device (nth, true);
  if (udev == 0){
    fprintf (stderr, "sdrtx: failed to find sdrtx[%d]\n", nth);
    return 0;
  }

  struct usb_dev_handle *udh;

  udh = sdrtx_open_cmd_interface (udev);
  if (udh == 0){
    // FIXME this could be because somebody else has it open.
    // We should delay and retry...
    fprintf (stderr, "sdrtx_open_nth_cmd_interface: open_cmd_interface failed\n");
    usb_strerror ();
    return 0;
  }

  return udh;
 }

static bool
our_nanosleep (const struct timespec *delay)
{
  struct timespec	new_delay = *delay;
  struct timespec	remainder;

  while (1){
    int r = nanosleep (&new_delay, &remainder);
    if (r == 0)
      return true;
    if (errno == EINTR)
      new_delay = remainder;
    else {
      perror ("nanosleep");
      return false;
    }
  }
}

static bool
mdelay (int millisecs)
{
  struct timespec	ts;
  ts.tv_sec = millisecs / 1000;
  ts.tv_nsec = (millisecs - (1000 * ts.tv_sec)) * 1000000;
  return our_nanosleep (&ts);
}

sdrtx_load_status_t
sdrtx_load_firmware_nth (int nth, const char *filename, bool force){
  struct usb_dev_handle *udh = sdrtx_open_nth_cmd_interface (nth);
  if (udh == 0)
    return ULS_ERROR;

  sdrtx_load_status_t s = sdrtx_load_firmware (udh, filename, force);
  sdrtx_close_interface (udh);

  switch (s){

  case ULS_ALREADY_LOADED:		// nothing changed...
    return ULS_ALREADY_LOADED;
    break;

  case ULS_OK:
    // we loaded firmware successfully.

    // It's highly likely that the board will renumerate (simulate a
    // disconnect/reconnect sequence), invalidating our current
    // handle.

    // FIXME.  Turn this into a loop that rescans until we refind ourselves
    
    struct timespec	t;	// delay for 1 second
    t.tv_sec = 2;
    t.tv_nsec = 0;
    our_nanosleep (&t);

    usb_find_busses ();		// rescan busses and devices
    usb_find_devices ();

    return ULS_OK;

  default:
  case ULS_ERROR:		// some kind of problem
    return ULS_ERROR;
  }
}

static void
load_status_msg (sdrtx_load_status_t s, const char *type, const char *filename)
{
  switch (s){
  case ULS_ERROR:
    fprintf (stderr, "sdrtx: failed to load %s %s.\n", type, filename);
    break;
    
  case ULS_ALREADY_LOADED:
    fprintf (stderr, "sdrtx: %s %s already loaded.\n", type, filename);
    break;

  case ULS_OK:
    fprintf (stderr, "sdrtx: %s %s loaded successfully.\n", type, filename);
    break;
  }
}

bool
sdrtx_load_standard_bits (int nth, char* firmware_filename, bool force)
{
  sdrtx_load_status_t 	s;
  char			*filename;
  int hw_rev;

  // first, figure out what hardware rev we're dealing with
  {
    struct usb_device *udev = sdrtx_find_device (nth, true);
    if (udev == 0){
      fprintf (stderr, "sdrtx: failed to find sdrtx[%d]\n", nth);
      return false;
    }
    hw_rev = _sdrtx_hw_rev (udev);
  }

  // start by loading the firmware
  
  filename = find_file (firmware_filename, hw_rev);
  if (filename == 0){
    fprintf (stderr, "Can't find firmware: %s\n", firmware_filename);
    return false;
  }

  s = sdrtx_load_firmware_nth (nth, filename, force);
  load_status_msg (s, "firmware", filename);

  if (s == ULS_ERROR)
    return false;

  return true;
}

//FIXME - implement?
bool
_sdrtx_get_status (struct usb_dev_handle *udh, int which, bool *trouble)
{
  unsigned char	status;
  *trouble = true;
  
  if (write_cmd (udh, VRQ_GET_STATUS, 0, which,
		 &status, sizeof (status)) != sizeof (status))
    return false;

  *trouble = status;
  return true;
}

//FIXME - implement?
bool
sdrtx_i2c_write (struct usb_dev_handle *udh, int i2c_addr,
		const void *buf, int len)
{
  if (len < 1 || len > MAX_EP0_PKTSIZE)
    return false;

  return write_cmd (udh, VRQ_I2C_WRITE, i2c_addr, 0,
		    (unsigned char *) buf, len) == len;
}

//FIXME - implement?
bool
sdrtx_i2c_read (struct usb_dev_handle *udh, int i2c_addr,
	       void *buf, int len)
{
  if (len < 1 || len > MAX_EP0_PKTSIZE)
    return false;

  return write_cmd (udh, VRQ_I2C_READ, i2c_addr, 0,
		    (unsigned char *) buf, len) == len;
}

//FIXME - implement?
bool
sdrtx_spi_write (struct usb_dev_handle *udh,
		int adr, unsigned long long int dat, int len)
{
  int index=0, value=0;

  if (len < 0 || len > MAX_EP0_PKTSIZE)
    return false;

	switch(len) {
		case 8:
			index = ((dat >> 8) & 0xff) << 8 | (dat & 0xff);
			value = ((dat >> 24) & 0xff) << 8 | ((dat >> 16) & 0xff);
			printf("send[0] %04x | %04x\n", index, value);
			write_cmd(udh, 0x04, value, index, NULL, 0);
			index = ((dat >> 40) & 0xff) << 8 | ((dat >> 32) & 0xff);
			value = ((dat >> 56) & 0xff) << 8 | ((dat >> 48) & 0xff);
			write_cmd(udh, 0x03, value, index, NULL, 0);
			printf("send[1] %04x | %04x\n", index, value);
			break;
		case 6:
			value = ((dat >> 8) & 0xff) << 8 | (dat & 0xff);
			printf("send[0] %04x | %04x\n", index, value);
			write_cmd(udh, 0x04, value, index, NULL, 0);
			index = ((dat >> 24) & 0xff) << 8 | ((dat >> 16) & 0xff);
			value = ((dat >> 40) & 0xff) << 8 | ((dat >> 32) & 0xff);
			write_cmd(udh, 0x03, value, index, NULL, 0);
			printf("send[1] %04x | %04x\n", index, value);
			break;
		case 4:
			value = ((dat >> 24) & 0xff) << 8 | ((dat >> 16) & 0xff);
			index = ((dat >> 8) & 0xff) << 8 | (dat & 0xff);
			write_cmd(udh, 0x03, value, index, NULL, 0);
			printf("send[1] %04x | %04x\n", index, value);
			break;
		case 2:
			value = ((dat >> 8) & 0xff) << 8 | (dat & 0xff);
			write_cmd(udh, 0x03, value, index, NULL, 0);
			printf("send[1] %04x | %04x\n", index, value);
			break;
		default:
			printf("bad length, must be 2/4/6 or 8 bytes\n");
	}

	index = len << 8 | adr;
//	if(!todev) index |= 0x80;

  int usblen = write_cmd(udh, 0x02, 0x00, index, NULL, 0);

  return (usblen == 0x00);

}

//FIXME - implement?
bool
sdrtx_spi_read (struct usb_dev_handle *udh,
	       int adr, unsigned long long int *dat_out, int len)
{
  unsigned long long int dat = 0;
  int i, index=0, value=0;
  *dat_out = 0;

  if (len < 0 || len > MAX_EP0_PKTSIZE)
    return false;

	index = len << 8 | adr;
	index |= 0x80;

	int usblen = write_cmd(udh, 0x02, 0x00, index, NULL, 0);
	if(usblen != 0x00)
		printf("bad len: %d\n", usblen);
	//else
	//	printf("ok\n");

		unsigned char buf[8];
		usblen = write_cmd(udh, 0x82, 0, 0, buf, 8);
		//printf("%d\n", len);
		if(usblen < 0)
			return 0;
		printf("%02x.%02x.%02x.%02x.%02x.%02x.%02x.%02x\n",
			buf[0],buf[1],buf[2],buf[3],buf[4],buf[5],buf[6],buf[7]);
		for(i=0;i<len;i++)
			dat = dat << 8 | buf[i];
		switch(len) {
			case 2:
				printf("reg[%02x]:%d = 0x%04llx\n", adr, len, dat);
				break;
			case 4:
				printf("reg[%02x]:%d = 0x%08llx\n", adr, len, dat);
				break;
			case 6:
				printf("reg[%02x]:%d = 0x%012llx\n", adr, len, dat);
				break;
			case 8:
			default:
				printf("reg[%02x]:%d = 0x%016llx\n", adr, len, dat);
				break;
		}

  *dat_out = dat;
  return (usblen == 8);
}

static const int EEPROM_PAGESIZE = 16;
//FIXME - implement?
bool
sdrtx_eeprom_write (struct usb_dev_handle *udh, int i2c_addr,
		   int eeprom_offset, const void *buf, int len)
{
  unsigned char cmd[2];
  const unsigned char *p = (unsigned char *) buf;
  
  // The simplest thing that could possibly work:
  //   all writes are single byte writes.
  //
  // We could speed this up using the page write feature,
  // but we write so infrequently, why bother...

  while (len-- > 0){
    cmd[0] = eeprom_offset++;
    cmd[1] = *p++;
    bool r = sdrtx_i2c_write (udh, i2c_addr, cmd, sizeof (cmd));
    mdelay (10);		// delay 10ms worst case write time
    if (!r)
      return false;
  }
  
  return true;
}

//FIXME - implement?
bool
sdrtx_eeprom_read (struct usb_dev_handle *udh, int i2c_addr,
		  int eeprom_offset, void *buf, int len)
{
  unsigned char *p = (unsigned char *) buf;

  // We setup a random read by first doing a "zero byte write".
  // Writes carry an address.  Reads use an implicit address.

  unsigned char cmd[1];
  cmd[0] = eeprom_offset;
  if (!sdrtx_i2c_write (udh, i2c_addr, cmd, sizeof (cmd)))
    return false;

  while (len > 0){
    int n = std::min (len, MAX_EP0_PKTSIZE);
    if (!sdrtx_i2c_read (udh, i2c_addr, p, n))
      return false;
    len -= n;
    p += n;
  }
  return true;
}
 
