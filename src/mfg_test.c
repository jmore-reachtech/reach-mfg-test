#define _GNU_SOURCE
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <getopt.h>
#include <ctype.h>
#include <stdint.h>
#include <string.h>
#include <time.h>
#include <asm/types.h>
#include <sys/types.h>
#include <sys/ioctl.h>
#include <sys/stat.h>
#include <sys/wait.h>
#include <fcntl.h>
#include <termios.h>
#include <errno.h>
#include <sys/mman.h>
#include <linux/spi/spidev.h>
#include <linux/input.h>
#include <sys/socket.h>
#include <net/if.h>
#include <linux/can.h>

#define VERSION     "0.9a"

typedef struct {
   unsigned int width;
   unsigned int height;
   unsigned int planes;
   unsigned short bitcount;
   unsigned int size;
   unsigned int offset;
} BITMAPINFOHEADER;

typedef struct {
   unsigned char blue;
   unsigned char green;
   unsigned char red;
} PIXEL;

typedef int     bool;
#define TRUE    1
#define FALSE   0

#undef  _I2C_BLOCK_WRITE
#undef  _I2C_BLOCK_READ

#ifdef DWG_X86_TEST
static inline int i2c_smbus_write_byte_data(fd, off, buf) { return 0; }
static inline int i2c_smbus_read_byte_data(fd, off) { return 0; }
#endif // DWG_X86_TEST

// Colors for text formatting
#define ATTR_RED        "\033[0;31m"    /* 0 -> normal; 31 -> red */
#define ATTR_GREEN      "\033[0;32m"    /* 0 -> normal; 32 -> green */
#define ATTR_UNKNOWN    "\033[0;33m"    /* 0 -> normal; 33 -> yellow */
#define ATTR_NONE       "\033[0m"       /* to flush the previous property */
#define STR_PASS        ATTR_GREEN "[PASS]" ATTR_NONE
#define STR_FAIL        ATTR_RED "[FAIL]" ATTR_NONE
#define STR_UNTESTED    ATTR_UNKNOWN "[UN-TESTED]" ATTR_NONE

typedef int (*testFunc_t)(void);

#define NELEM(a) (sizeof(a) / sizeof((a)[0]))

#define NETWORK_ETHERNET        0
#define NETWORK_USBOTG          1
#define NETWORK_CAN             2

const char *RtcIfList[] = { "ETHERNET", "USBOTG" };

typedef struct {
    char    if_name[32];
    #define _NET_USBOTG_LOADED  (0x01 << 0)
    #define _NET_CAN_LOADED     (0x01 << 1)
    #define _NET_INTERFACE_UP   (0x01 << 2)
    uint8_t flags;
} ethIf_t;

#define GPIO_PATH       "/sys/class/gpio"
#define GPIO_PIN_BASE   160
#define GPIO_NUM_PINS   8

typedef struct {
    char    if_path[64];
    uint8_t pin;
    #define _GPIO_DIR_UNKNOWN   0
    #define _GPIO_DIR_INPUT     1
    #define _GPIO_DIR_OUTPUT    2
    uint8_t direction;
    #define _GPIO_POL_UNKNOWN   0
    #define _GPIO_POL_NORMAL    1
    #define _GPIO_POL_INVERT    2
    uint8_t polarity;
    #define _GPIO_PIN_EXPORTED  (0x01 << 0)
    #define _GPIO_PIN_OPEN      (0x01 << 1)
    uint8_t flags;
} gpio_t;

#define _USB_SERIAL             0
#define _USB_MASS_STORAGE       1
#define _USB_BARCODE_SCANNER    2
uint8_t UsbList[8];
uint8_t NumUsbList = 0;

static const uint8_t gpio_map[] = { 3, 2, 1, 0, 4, 5, 6, 7 };
static const uint8_t reach_oui[] = { 0x30, 0x68, 0x8C };

#define USER_INPUT_STRING   "{ASK_USER}"

#ifndef  DBG_BUFFER_LEN
#  define DBG_BUFFER_LEN    1024
#endif

struct {
    #define _DBG_DATA_ON        0x00000010L
    #define _DBG_VERBOSE_ON     0x00000020L
    #define _DBG_KEYEVENT_ON    0x00000040L
    uint32_t    debug;
    bool        verbose;
    bool        dryrun;
    uint32_t    repeat;
    char       *file_path;
    char       *tests;
    char       *mac_address;
    struct {
        char       *if_name;
        char       *server_ip;
        char       *local_ip;
    }           ethernet;
    struct {
        char       *if_name;
        char       *remote_macaddr;
        char       *server_ip;
        char       *local_ip;
    }           usbotg;
    uint8_t     rtc_if;
    uint32_t    rs485_baud;
    uint32_t    rs232_baud;
    uint32_t    buffer_size;
    char       *i2c_path;
    uint8_t     i2c_test_addr;
    uint8_t     i2c_test_offset;
    uint8_t     i2c_gpio_addr;
    uint8_t     i2c_gpio_offset;
    uint8_t     spi_bus;
    char	   *image_dir;
} g_info = {
    .debug = 0x0,
    .verbose = FALSE,
    .dryrun = FALSE,
    .repeat = 1,
    .file_path = "file-32M",
    .tests = NULL,
    .mac_address = NULL,
    .ethernet = {
        .if_name = "eth0",
        .server_ip = "10.10.10.2",
        .local_ip = "10.10.10.3",
    },
    .usbotg = {
        .if_name = "usb0",
        .remote_macaddr = "10:20:30:40:50:60",
        .server_ip = "10.10.10.1",
        .local_ip = "10.10.10.4",
    },
    .rtc_if = NETWORK_USBOTG,
    .rs485_baud = B4000000,
    .rs232_baud = B1152000,
    .buffer_size = 512,
    .i2c_path = "/dev/i2c-1",
    .i2c_test_addr = 0x20,
    .i2c_test_offset = 0x00,
    .i2c_gpio_addr = 0x3E,
    .i2c_gpio_offset = 0x00,
    .spi_bus = 1,
    .image_dir = "/home/root/",
};

static int execute_cmd_ex(const char *cmd, char *result, int result_size);
static int execute_cmd(const char *cmd);

static ethIf_t *network_open(uint8_t if_type, uint8_t instance);
static int network_close(ethIf_t *ep);

const struct {
    uint32_t    key;
    uint32_t    speed;
} BaudTable[] = {
    { B0      , 0       },
    { B50     , 50      },
    { B75     , 75      },
    { B110    , 110     },
    { B134    , 134     },
    { B150    , 150     },
    { B200    , 200     },
    { B300    , 300     },
    { B600    , 600     },
    { B1200   , 1200    },
    { B1800   , 1800    },
    { B2400   , 2400    },
    { B4800   , 4800    },
    { B9600   , 9600    },
    { B19200  , 19200   },
    { B38400  , 38400   },
    { B57600  , 57600   },
    { B115200 , 115200  },
    { B230400 , 230400  },
    { B460800 , 460800  },
    { B500000 , 500000  },
    { B576000 , 576000  },
    { B921600 , 921600  },
    { B1000000, 1000000 },
    { B1152000, 1152000 },
    { B1500000, 1500000 },
    { B2000000, 2000000 },
    { B2500000, 2500000 },
    { B3000000, 3000000 },
    { B3500000, 3500000 },
    { B4000000, 4000000 },
};

/* This following is copied from <linux/fb.h>.  fb.h includes i2c.h 
 * which conflicts with lm-senors i2c-dev-user.h */
struct fb_bitfield {
	__u32 offset;			/* beginning of bitfield	*/
	__u32 length;			/* length of bitfield		*/
	__u32 msb_right;		/* != 0 : Most significant bit is */ 
					/* right */ 
};

struct fb_fix_screeninfo {
	char id[16];			/* identification string eg "TT Builtin" */
	unsigned long smem_start;	/* Start of frame buffer mem */
					/* (physical address) */
	__u32 smem_len;			/* Length of frame buffer mem */
	__u32 type;			/* see FB_TYPE_*		*/
	__u32 type_aux;			/* Interleave for interleaved Planes */
	__u32 visual;			/* see FB_VISUAL_*		*/ 
	__u16 xpanstep;			/* zero if no hardware panning  */
	__u16 ypanstep;			/* zero if no hardware panning  */
	__u16 ywrapstep;		/* zero if no hardware ywrap    */
	__u32 line_length;		/* length of a line in bytes    */
	unsigned long mmio_start;	/* Start of Memory Mapped I/O   */
					/* (physical address) */
	__u32 mmio_len;			/* Length of Memory Mapped I/O  */
	__u32 accel;			/* Indicate to driver which	*/
					/*  specific chip/card we have	*/
	__u16 capabilities;		/* see FB_CAP_*			*/
	__u16 reserved[2];		/* Reserved for future compatibility */
};

struct fb_var_screeninfo {
	__u32 xres;			/* visible resolution		*/
	__u32 yres;
	__u32 xres_virtual;		/* virtual resolution		*/
	__u32 yres_virtual;
	__u32 xoffset;			/* offset from virtual to visible */
	__u32 yoffset;			/* resolution			*/

	__u32 bits_per_pixel;		/* guess what			*/
	__u32 grayscale;		/* 0 = color, 1 = grayscale,	*/
					/* >1 = FOURCC			*/
	struct fb_bitfield red;		/* bitfield in fb mem if true color, */
	struct fb_bitfield green;	/* else only length is significant */
	struct fb_bitfield blue;
	struct fb_bitfield transp;	/* transparency			*/	

	__u32 nonstd;			/* != 0 Non standard pixel format */

	__u32 activate;			/* see FB_ACTIVATE_*		*/

	__u32 height;			/* height of picture in mm    */
	__u32 width;			/* width of picture in mm     */

	__u32 accel_flags;		/* (OBSOLETE) see fb_info.flags */

	/* Timing: All values in pixclocks, except pixclock (of course) */
	__u32 pixclock;			/* pixel clock in ps (pico seconds) */
	__u32 left_margin;		/* time from sync to picture	*/
	__u32 right_margin;		/* time from picture to sync	*/
	__u32 upper_margin;		/* time from sync to picture	*/
	__u32 lower_margin;
	__u32 hsync_len;		/* length of horizontal sync	*/
	__u32 vsync_len;		/* length of vertical sync	*/
	__u32 sync;			/* see FB_SYNC_*		*/
	__u32 vmode;			/* see FB_VMODE_*		*/
	__u32 rotate;			/* angle we rotate counter clockwise */
	__u32 colorspace;		/* colorspace for FOURCC-based modes */
	__u32 reserved[4];		/* Reserved for future compatibility */
};

#define FBIOGET_VSCREENINFO	0x4600
#define FBIOPUT_VSCREENINFO	0x4601
#define FBIOGET_FSCREENINFO	0x4602

/****************************************************************************
 * fbimage
 */
int fbimage(char *image_path)
{
    int fbfd 				= 0;
    int tfd 				= 0;
    char buf[8];
    size_t nbytes			= 0;
	ssize_t bytes_read		= 0;
    struct fb_var_screeninfo vinfo;
    struct fb_fix_screeninfo finfo;
    long int screensize 	= 0;
    char *fbp 				= 0;
    int x = 0, y 			= 0;
    long int location 		= 0;

    FILE *image;
    BITMAPINFOHEADER bih;

    /* Open the file for reading and writing */
    fbfd = open("/dev/fb0", O_RDWR);
    if (fbfd == -1) {
        perror("Error: cannot open framebuffer device");
        exit(1);
    }
    /* printf("The framebuffer device was opened successfully.\n"); */

    /* Get fixed screen information */
    if (ioctl(fbfd, FBIOGET_FSCREENINFO, &finfo) == -1) {
        perror("Error reading fixed information");
        exit(2);
    }

    /* Get variable screen information */
    if (ioctl(fbfd, FBIOGET_VSCREENINFO, &vinfo) == -1) {
        perror("Error reading variable information");
        exit(3);
    }

    /* printf("%dx%d, %dbpp\n", vinfo.xres, vinfo.yres, vinfo.bits_per_pixel); */

    /* Figure out the size of the screen in bytes */
    screensize = vinfo.xres * vinfo.yres * vinfo.bits_per_pixel / 8;

    /* Map the device to memory */
    fbp = (char *)mmap(0, screensize, PROT_READ | PROT_WRITE, MAP_SHARED, fbfd, 0);
    if ((int)fbp == -1) {
        perror("Error: failed to map framebuffer device to memory");
        exit(4);
    }

	if(g_info.debug) {
		printf("Opening file: %s\n", image_path);
	}
    if(!(image = fopen(image_path, "rb+")))
    {
        printf("Error opening image file!\n");
        munmap(fbp, screensize);
        close(fbfd);
        return 1;
    }

    /* Get the size of the file */
    fseek(image,2,SEEK_SET);
    fread(&bih.size,4,1,image);
    //printf("Size=%d\n",bih.size);
    fseek(image,18,SEEK_SET);
    fread(&bih.width,4,1,image);
    fseek(image,22,SEEK_SET);
    fread(&bih.height,4,1,image);
    //printf("Width=%d\tHeight=%d\n",bih.width,bih.height);
    fseek(image,26,SEEK_SET);
    fread(&bih.planes,2,1,image);
    //printf("Number of planes:%d\n",bih.planes);
    fseek(image,28,SEEK_SET);
    fread(&bih.bitcount,2,1,image);
    //printf("Bit Count:%d\n",bih.bitcount);
    fseek(image,10,SEEK_SET);
    fread(&bih.offset,4,1,image);
    //printf("Offset:%d\n",bih.offset);

    PIXEL pic[bih.width*bih.height*2],p;


    fseek(image,bih.offset,SEEK_SET);

    int j=0;
    uint counter;
    for (counter = 0; counter <= (bih.size-54); counter += 3)
    {
        fread(&p,sizeof(p),1,image);

        if(!feof(image))
        {
            pic[j]=p;
            //printf("%d= %d %d %d ",j+54,pic[j].blue,pic[j].green,pic[j].red);
            j++;
        }
    }

    j=0;
    // Figure out where in memory to put the pixel
    for (y = 0; y < bih.height; y++) {
        for (x = 0; x < bih.width; x++) {

            location = (x+vinfo.xoffset) * (vinfo.bits_per_pixel/8) +
                       (y+vinfo.yoffset) * finfo.line_length;

            if (vinfo.bits_per_pixel == 32) {
                *(fbp + location) = pic[j].blue;        // Blue
                *(fbp + location + 1) = pic[j].green;   // Green
                *(fbp + location + 2) = pic[j].red;     // Red
                *(fbp + location + 3) = 0;      // No transparency
            } else { //assume 16bpp
                int b = pic[j].blue;
                int g = pic[j].green;
                int r = pic[j].red;
                unsigned short int t = (b<<8 & 0xf800) | (g << 3 & 0x7e0) | (r >> 3);
                *((unsigned short int*)(fbp + location)) = t; 
            }
            j++; //increment pixel pointer.
        }
    }
    fclose(image);

    munmap(fbp, screensize);
    close(fbfd);
    
    /* open stdin */
    tfd = open("/dev/stdin", 0);
    if (tfd == -1) {
        perror("Error: cannot open stdin");
        exit(1);
    }
    
    printf("Press any key to continue...\n");
    nbytes = sizeof(buf);
    bytes_read = read(tfd, buf, nbytes);
    if (g_info.verbose) { 
		fprintf(stdout, "Debug: %s: Read %d bytes.\n", __FUNCTION__,bytes_read);
	}
            
    close(tfd);
    return 0;
}


int fbutil(int r, int g, int b)
{
	int fd = 0;
	struct fb_var_screeninfo vinfo;
    struct fb_fix_screeninfo finfo;
    long int screensize = 0;
    char *fbp = 0;
    int x = 0, y = 0;
    long int location = 0;
    int count = 0;
    
	// Open the file for reading and writing
    fd = open("/dev/fb0", O_RDWR);
    if (fd == -1) {
        perror("Error: cannot open framebuffer device");
        exit(1);
    }
    //printf("The framebuffer device was opened successfully.\n");
    
    // Get fixed screen information
    if (ioctl(fd, FBIOGET_FSCREENINFO, &finfo) == -1) {
        perror("Error reading fixed information");
        exit(2);
    }

    // Get variable screen information
    if (ioctl(fd, FBIOGET_VSCREENINFO, &vinfo) == -1) {
        perror("Error reading variable information");
        exit(3);
    }

    //printf("%dx%d, %dbpp\n", vinfo.xres, vinfo.yres, vinfo.bits_per_pixel);
    //printf("xoffset: %d yoffset: %d\n", vinfo.xoffset, vinfo.yoffset);

    // Figure out the size of the screen in bytes
    screensize = vinfo.xres * vinfo.yres * vinfo.bits_per_pixel / 8;
	
	//printf("Screensize: %lu\n",screensize);
	
	// Map the device to memory
    fbp = (char *)mmap(0, screensize, PROT_READ | PROT_WRITE, MAP_SHARED, fd, 0);
    if ((int)fbp == -1) {
        perror("Error: failed to map framebuffer device to memory");
        exit(4);
    }
    //printf("The framebuffer device was mapped to memory successfully.\n");
		
	x = 0; y = 0; /* Where we are going to put the pixel */

	/* Figure out where in memory to put the pixel */
	location = (x+vinfo.xoffset) * (vinfo.bits_per_pixel/8) + (y+vinfo.yoffset) * finfo.line_length;
	//printf("Starting at location: %lu\n", location);	
	while ( location < screensize)
	{
		*(fbp + location) = b;     /* blue         */
		*(fbp + location + 1) = g; /* green        */
		*(fbp + location + 2) = r; /* red          */
		*(fbp + location + 3) = 0x0;  /* transparency */
		location += 4;
		count++;
	}
	
	//printf("location: %lu count: %d\n",location,count);
	
	munmap(fbp, screensize);
	close(fd);
	
	return 0;
}

/****************************************************************************
 * DbgDataToString
 */
static const char *
DbgDataToString(
    const void *    pVData,
    unsigned long   offset,
    unsigned long   length,
    char *          pBuffer,
    unsigned long   bufferLength
    )
{
    static char     Buffer[DBG_BUFFER_LEN] = { 0 }; /* Debug only! */
    const char      digits[] = "0123456789ABCDEF";
    const unsigned char *pData = (const unsigned char *) pVData;
    char           *pB;
    const unsigned char *pD;
    unsigned long   x;
    unsigned char   val;

    /* If the user buffer was not supplied, use the internal buffer. */
    if (pBuffer == NULL)
    {
        pBuffer = Buffer;
        bufferLength = sizeof(Buffer) - 1;
    }

    pB = pBuffer;

    if (pData != NULL)
    {
        if (length < bufferLength)
        {
            pD = (pData + offset);
            for (x = 0; x < length; x++)
            {
                if ((pD[x] >= ' ') && (pD[x] <= '~'))
                {
                    *(pB++) = pD[x];
                }
                else
                {
                    val = pD[x];

                    *(pB++) = '[';
                    *(pB++) = digits[(val >> 4) & 0x0F];
                    *(pB++) = digits[val & 0x0F];
                    *(pB++) = ']';
                }

                if (x >= (bufferLength - 12))
                {
                    *(pB++) = '.';
                    *(pB++) = '.';
                    *(pB++) = '.';
                    break;
                }
            }
        }
        else
        {
            *(pB++) = '.';
            *(pB++) = '.';
            *(pB++) = '.';
        }
        *(pB++) = '\0';

#ifdef DWG_DBG_TRUNCATE
        /* Truncate lines that are too long. */
        if ((pB - pBuffer) > 512)
        {
            pBuffer[0] = '.';
            pBuffer[1] = '.';
            pBuffer[2] = '.';
            pBuffer[3] = '\0';
        }
#endif // DWG_DBG_TRUNCATE
    }
    else
    {
        *(pB++) = '<';
        *(pB++) = 'N';
        *(pB++) = 'U';
        *(pB++) = 'L';
        *(pB++) = 'L';
        *(pB++) = '>';
        *(pB++) = '\0';
    }

    return pBuffer;
}

/****************************************************************************
 * DbgOctet
 */
static const char *
DbgOctet(
    const unsigned char * pData,
    unsigned char   length,
    char *          pBuffer,
    unsigned long   bufferLength
    )
{
    static char     Buffer[32] = { 0 }; /* Debug only! */
    const char      digits[] = "0123456789ABCDEF";
    char           *pB;
    const unsigned char *pD;
    unsigned long   x;
    unsigned char   val;

    /* If the user buffer was not supplied, use the internal buffer. */
    if (pBuffer == NULL)
    {
        pBuffer = Buffer;
        bufferLength = sizeof(Buffer) - 1;
    }

    pB = pBuffer;

    if (pData != NULL)
    {
        pD = pData;
        for (x = 0; x < length; x++)
        {
            val = pD[x];

            if (x != 0) *(pB++) = ':';
            *(pB++) = digits[(val >> 4) & 0x0F];
            *(pB++) = digits[val & 0x0F];
        }
        *(pB++) = '\0';
    }
    else
    {
        *(pB++) = '<';
        *(pB++) = 'N';
        *(pB++) = 'U';
        *(pB++) = 'L';
        *(pB++) = 'L';
        *(pB++) = '>';
        *(pB++) = '\0';
    }

    return pBuffer;
}

/****************************************************************************
 * baud_key_to_str
 */
static const char *
baud_key_to_str(
    uint32_t    baud_key
    )
{
    static char buffer[32] = { 0 };
    int x;

    for (x = 0; x < NELEM(BaudTable); x++)
    {
        if (BaudTable[x].key == baud_key)
        {
            break;
        }
    }

    if (x >= NELEM(BaudTable))
    {
        sprintf(buffer, "{Unknown<%06oo>}", baud_key);
    }
    else
    {
        uint32_t speed = BaudTable[x].speed;

        sprintf(buffer, "%ubps", speed);
    }

    return buffer;
}

/****************************************************************************
 * baud_str_to_key
 */
static uint32_t
baud_str_to_key(const char *baud_str)
{
    uint32_t baud_key = __MAX_BAUD;
    char *baud_buf = NULL;
    char buf[32];
    char *bp;
    int x;

    baud_buf = strdup(baud_str);
    if ((bp = strstr(baud_buf, "bps")) != NULL)
    {
        *bp = '\0';
    }

    for (x = 0; x < NELEM(BaudTable); x++)
    {
        sprintf(buf, "%u", BaudTable[x].speed);
        if (strcasecmp(buf, baud_buf) == 0)
        {
            baud_key = BaudTable[x].key;
            break;
        }
    }

    if (x >= NELEM(BaudTable))
    {
        fprintf(stderr, "Warning: Unknown baud rate '%s'. Defaulting to %s.\n", baud_str, baud_key_to_str(baud_key));
    }

    if (baud_buf != NULL)
    {
        free(baud_buf);
        baud_buf = NULL;
    }

    return baud_key;
}

/****************************************************************************
 * network_open
 */
static ethIf_t *
network_open(uint8_t if_type, uint8_t instance)
{
    ethIf_t *ep = NULL;
    char if_name[32];
    const char *ip_address = NULL;
    const char *server_ip = NULL;
    const char *remote_macaddr = NULL;
    char cmd[128];
    int rv = 0;
    int n;

    *if_name = '\0';
    switch (if_type)
    {
    case NETWORK_ETHERNET:
        sprintf(if_name, "eth%d", instance);
        ip_address = g_info.ethernet.local_ip;
        server_ip = g_info.ethernet.server_ip;
        break;

    case NETWORK_USBOTG:
        sprintf(if_name, "usb%d", instance);
        ip_address = g_info.usbotg.local_ip;
        server_ip = g_info.usbotg.server_ip;
        remote_macaddr = g_info.usbotg.remote_macaddr;
        break;

    case NETWORK_CAN:
        sprintf(if_name, "can%d", instance);
        break;
    }

    n = sizeof(*ep);
    if ((ep = malloc(n)) == NULL)
    {
        fprintf(stderr, "Error: %s: malloc(%d) failed: %s [%d]\n", __FUNCTION__, n, strerror(errno), errno);
        goto e_network_open;
    }
    memset(ep, 0, n);

    strcpy(ep->if_name, if_name);

    if ((if_type == NETWORK_USBOTG) && (remote_macaddr != NULL))
    {
        sprintf(cmd, "modprobe g_ether host_addr=%s", remote_macaddr);
        rv = execute_cmd(cmd);
        if (rv < 0)
        {
            fprintf(stderr, "Error: %s: execute_cmd('%s') failed: %s [%d]\n", __FUNCTION__, cmd, strerror(errno), errno);
            goto e_network_open;
        }
        ep->flags |= _NET_USBOTG_LOADED;
    }

    if (if_type == NETWORK_CAN)
    {
        sprintf(cmd, "modprobe flexcan");
        rv = execute_cmd(cmd);
        if (rv < 0)
        {
            fprintf(stderr, "Error: %s: execute_cmd('%s') failed: %s [%d]\n", __FUNCTION__, cmd, strerror(errno), errno);
            goto e_network_open;
        }
        ep->flags |= _NET_CAN_LOADED;

        sprintf(cmd, "ifconfig %s up", if_name);
        rv = execute_cmd(cmd);
        if (rv < 0)
        {
            fprintf(stderr, "Error: %s: execute_cmd('%s') failed: %s [%d]\n", __FUNCTION__, cmd, strerror(errno), errno);
            goto e_network_open;
        }
    }

    if (ip_address != NULL)
    {
        sprintf(cmd, "ifconfig %s %s netmask 255.255.255.0 up", if_name, ip_address);
        rv = execute_cmd(cmd);
        if (rv < 0)
        {
            fprintf(stderr, "Error: %s: execute_cmd('%s') failed: %s [%d]\n", __FUNCTION__, cmd, strerror(errno), errno);
            goto e_network_open;
        }
    }

    if (server_ip != NULL)
    {
        for (n = 0; n < 10; n++)
        {
            sprintf(cmd, "ping -c 1 -W 1 %s", server_ip);
            rv = execute_cmd(cmd);
            if (rv >= 0)
            {
                break;
            }
        }

        if (n >= 10)
        {
            fprintf(stderr, "Error: %s: timeout trying to ping server %s\n", __FUNCTION__, server_ip);
            goto e_network_open;
        }
    }
    ep->flags |= _NET_INTERFACE_UP;

e_network_open:
    if (!(ep->flags & _NET_INTERFACE_UP))
    {
        if (ep != NULL)
        {
            network_close(ep);
            ep = NULL;
        }
    }

    return ep;
}

/****************************************************************************
 * network_close
 */
static int
network_close(ethIf_t *ep)
{
    int status = 0;
    char cmd[128];
    int rv = 0;

    if (ep != NULL)
    {
        if (ep->flags & _NET_INTERFACE_UP)
        {
            ep->flags &= ~_NET_INTERFACE_UP;

            sprintf(cmd, "ifconfig %s down", ep->if_name);
            rv = execute_cmd(cmd);
            if (rv < 0)
            {
                fprintf(stderr, "Error: %s: execute_cmd('%s') failed: %s [%d]\n", __FUNCTION__, cmd, strerror(errno), errno);
                status = rv;
            }
        }

        if (ep->flags & _NET_USBOTG_LOADED)
        {
            ep->flags &= ~_NET_USBOTG_LOADED;

            sprintf(cmd, "rmmod g_ether");
            rv = execute_cmd(cmd);
            if (rv < 0)
            {
                fprintf(stderr, "Error: %s: execute_cmd('%s') failed: %s [%d]\n", __FUNCTION__, cmd, strerror(errno), errno);
                status = rv;
            }
        }

        if (ep->flags & _NET_CAN_LOADED)
        {
            ep->flags &= ~_NET_CAN_LOADED;

            sprintf(cmd, "rmmod flexcan");
            rv = execute_cmd(cmd);
            if (rv < 0)
            {
                fprintf(stderr, "Error: %s: execute_cmd('%s') failed: %s [%d]\n", __FUNCTION__, cmd, strerror(errno), errno);
                status = rv;
            }
        }

        free(ep);
        ep = NULL;
    }

    return status;
}

/****************************************************************************
 * execute_cmd_ex
 */
static int
execute_cmd_ex(const char *cmd, char *result, int result_size)
{
    int status = 0;
    FILE *fp = NULL;
    char *icmd = NULL;
    int n;

    if (g_info.debug & _DBG_VERBOSE_ON) fprintf(stdout, "Debug: %s(\"%s\")\n", __FUNCTION__, cmd);

    if ((result != NULL) && (result_size > 0))
    {
        *result = '\0';
    }

    n = strlen(cmd) + 16;
    if ((icmd = malloc(n)) == NULL)
    {
        fprintf(stderr, "Error: %s: malloc(%d) failed: %s [%d]\n", __FUNCTION__, n, strerror(errno), errno);
        status = -1;
        goto e_execute_cmd_ex;
    }
    strcpy(icmd, cmd);
    strcat(icmd, " 2>&1");

    if ((fp = popen(icmd, "r")) != NULL)
    {
        char buf[256];
        int x;
        int p_status;

        buf[0] = '\0';

        while (fgets(buf, sizeof(buf), fp) != NULL)
        {
            // Strip trailing whitespace
            for (x = strlen(buf); (x > 0) && isspace(buf[x - 1]); x--); buf[x] = '\0';
            if (buf[0] == '\0') continue;

            if (result != NULL)
            {
                if ((strlen(result) + strlen(buf) + 4) < result_size)
                {
                    strcat(result, buf);
                    strcat(result, "\n");
                }
                else
                {
                    strcat(result, "...\n");
                    result = NULL;
                }
            }

            if (g_info.debug & _DBG_VERBOSE_ON) fprintf(stdout, "Debug: %s:   '%s'\n", __FUNCTION__, buf);
        }

        p_status = pclose(fp);
        if (p_status == -1)
        {
            fprintf(stderr, "Error: %s: pclose() failed: %s [%d]\n", __FUNCTION__, strerror(errno), errno);
            status = -1;
            goto e_execute_cmd_ex;
        }
        else if (WIFEXITED(p_status))
        {
            int e_status;

            e_status = (int8_t) WEXITSTATUS(p_status);

            if (e_status != 0)
            {
                if (g_info.verbose) fprintf(stderr, "Error: %s: Command '%s' exited with status %d\n", __FUNCTION__, cmd, e_status);
                status = -1;
            }
            else
            {
                if (g_info.debug & _DBG_VERBOSE_ON) fprintf(stdout, "Debug: %s: Command '%s' exited with status %d\n", __FUNCTION__, cmd, e_status);
            }
        }
        else if (WIFSIGNALED(p_status))
        {
            int sig;

            sig = WTERMSIG(p_status);
            fprintf(stderr, "Error: %s: Command '%s' was killed with signal %d\n", __FUNCTION__, cmd, sig);
            status = -1;
        }
        else
        {
            fprintf(stderr, "Error: %s: Command '%s' exited for unknown reason\n", __FUNCTION__, cmd);
            status = -1;
        }

        fp = NULL;
    }
    else
    {
        fprintf(stderr, "Error: %s: popen('%s') failed: %s [%d]\n", __FUNCTION__, icmd, strerror(errno), errno);
        status = -1;
        goto e_execute_cmd_ex;
    }

e_execute_cmd_ex:
    if (icmd != NULL)
    {
        free(icmd);
        icmd = NULL;
    }

    return status;
}

/****************************************************************************
 * execute_cmd
 */
static int
execute_cmd(const char *cmd)
{
    return execute_cmd_ex(cmd, NULL, 0);
}

/****************************************************************************
 * test_Ethernet
 */
static int
test_Ethernet(void)
{
    int status = 0;
    const char local_file[] = "/tmp/EthernetDownload";
    ethIf_t *ep = NULL;
    char cmd[128];
    int rv = 0;

    #define _ETHERNET_INTERFACE_UP    (0x01 << 0)
    #define _ETHERNET_FILE_DOWNLOADED (0x01 << 1)
    uint8_t flags = 0x0;

    //DWG J6:Ethernet <==> Host via Ethernet
    // Host
    //   - Enable web server
    //
    // Target
    //   ifconfig eth0 x.x.x.2 netmask 255.255.255.0 up
    //   ping x.x.x.1
    //   cd /tmp
    //   wget http://x.x.x.1/~reach/File-32M

    /* FIX: */
    fprintf(stdout, "Ethernet Test Not Implemented! \n");
    return 1;

    ep = network_open(NETWORK_ETHERNET, 0);
    if (ep == NULL)
    {
        fprintf(stderr, "Error: %s: network_open() failed: %s [%d]\n", __FUNCTION__, strerror(errno), errno);
        status = -1;
        goto e_test_Ethernet;
    }
    flags |= _ETHERNET_INTERFACE_UP;

    flags |= _ETHERNET_FILE_DOWNLOADED;
    sprintf(cmd, "wget -O %s http://%s/%s", local_file, g_info.ethernet.server_ip, g_info.file_path);
    rv = execute_cmd(cmd);
    if (rv < 0)
    {
        fprintf(stderr, "Error: %s: execute_cmd('%s') failed: %s [%d]\n", __FUNCTION__, cmd, strerror(errno), errno);
        status = rv;
        goto e_test_Ethernet;
    }

e_test_Ethernet:
    if (flags & _ETHERNET_FILE_DOWNLOADED)
    {
        sprintf(cmd, "rm -f %s", local_file);
        rv = execute_cmd(cmd);
        if (rv < 0)
        {
            fprintf(stderr, "Error: %s: execute_cmd('%s') failed: %s [%d]\n", __FUNCTION__, cmd, strerror(errno), errno);
            status = rv;
        }
        flags &= ~_ETHERNET_FILE_DOWNLOADED;
    }

    if (flags & _ETHERNET_INTERFACE_UP)
    {
        if (ep != NULL)
        {
            rv = network_close(ep);
            if (rv < 0)
            {
                fprintf(stderr, "Error: %s: network_close() failed: %s [%d]\n", __FUNCTION__, strerror(errno), errno);
                status = -1;
            }
            ep = NULL;
        }
        flags &= ~_ETHERNET_INTERFACE_UP;
    }

    return status;
}

/****************************************************************************
 * test_LCD
 */
static int
test_LCD(void)
{
    int status = 1;
    //char cmd[128];
    char image[256] = {0};
    
    /* load the rgb image */
    strcat(image,g_info.image_dir);
    strcat(image,"fruit_girl.bmp");
    
    status = fbimage(image);
    
    image[0] = '\0';
    fbutil(0x0,0x0,0x0);
    
    //DWG J13:LCD
    // Target
    //   Display patterns on the screen using QML with pass/fail buttons

    return status;
}

/****************************************************************************
 * test_Backlight
 */
static int
test_Backlight(void)
{
    int status = 0;
    int rv = 0;
    char path[128];
    FILE *fp = NULL;
    int delay = 50 * 1000;
    int num_powercycles = 4;
    
    /* FIX: */
    fprintf(stdout, "Backlight Test Not Implemented! \n");
    return 1;

    fbutil(0xff,0xff,0xff);

    //DWG J13:Backlight
    // Target
    //   Display slider on the screen using QML that adjusts the screen brightness with pass/fail buttons

    sprintf(path, "/sys/class/backlight/mxs-bl/brightness");
    fp = fopen(path, "ab");
    if (fp != NULL)
    {
        int brightness;

        // Backlight brightness ramp down
        brightness = 100;
        while (brightness >= 0)
        {
            if (g_info.verbose) fprintf(stdout, "Debug: %s: Brightness %d.\n", __FUNCTION__, brightness);
            rv = fprintf(fp, "%d", brightness); fflush(fp);
            if (rv < 0)
            {
                fprintf(stderr, "Error: %s: fprintf('%s') failed: %s [%d]\n", __FUNCTION__, path, strerror(errno), errno);
                status = -1;
                goto e_test_Backlight;
            }

            usleep(delay);
            brightness -= 10;
        }

        // Backlight brightness ramp up
        brightness = 0;
        while (brightness <= 100)
        {
            if (g_info.verbose) fprintf(stdout, "Debug: %s: Brightness %d.\n", __FUNCTION__, brightness);
            rv = fprintf(fp, "%d", brightness); fflush(fp);
            if (rv < 0)
            {
                fprintf(stderr, "Error: %s: fprintf('%s') failed: %s [%d]\n", __FUNCTION__, path, strerror(errno), errno);
                status = -1;
                goto e_test_Backlight;
            }

            usleep(delay);
            brightness += 10;
        }

        fclose(fp);
        fp = NULL;
    }
    else
    {
        fprintf(stderr, "Error: %s: fopen('%s') failed: %s [%d]\n", __FUNCTION__, path, strerror(errno), errno);
        status = -1;
        goto e_test_Backlight;
    }

    sprintf(path, "/sys/class/backlight/mxs-bl/bl_power");
    fp = fopen(path, "ab");
    if (fp != NULL)
    {
        int x;

        // Backlight power on/off
        for (x = 0; x < num_powercycles; x++)
        {
            usleep(delay * 10);

            if (g_info.verbose) fprintf(stdout, "Debug: %s: Backlight power %s.\n", __FUNCTION__, x & 1 ? "on" : "off");
            rv = fprintf(fp, "%d", (x & 1) ^ 1); fflush(fp);
            if (rv < 0)
            {
                fprintf(stderr, "Error: %s: fprintf('%s') failed: %s [%d]\n", __FUNCTION__, path, strerror(errno), errno);
                status = -1;
                goto e_test_Backlight;
            }
        }

        fclose(fp);
        fp = NULL;
    }
    else
    {
        fprintf(stderr, "Error: %s: fopen('%s') failed: %s [%d]\n", __FUNCTION__, path, strerror(errno), errno);
        status = -1;
        goto e_test_Backlight;
    }

e_test_Backlight:
    if (fp != NULL)
    {
        fclose(fp);
        fp = NULL;
    }

    return status;
}

/****************************************************************************
 * assign_MacAddress
 */
static int
assign_MacAddress(const char *mac_addr_str)
{
    int status = 0;
    char path[128];
    FILE *fp = NULL;
    int rv = 0;
    char buf[128];
    uint8_t blen;
    uint8_t mac_addr[6];
    int x;

    //DWG
    // Target
    //   Read MAC Address from scanner and program OTP

    blen = 0;
    strcpy(buf, mac_addr_str);
    for (x = 0; x < strlen(mac_addr_str); x++)
    {
        switch (mac_addr_str[x])
        {
        // Strip out the following characters
        case ':':
            break;
        default:
            buf[blen++] = mac_addr_str[x];
            break;
        }
    }
    buf[blen] = '\0';

    if (g_info.debug & _DBG_DATA_ON) fprintf(stdout, "Debug: %s: MAC address buffer '%s'[%d]\n", __FUNCTION__, DbgDataToString(buf, 0, blen, NULL, 0), blen);

    // Strip trailing whitespace
    for (x = strlen(buf); (x > 0) && isspace(buf[x - 1]); x--); buf[x] = '\0';
    if (buf[0] == '\0')
    {
        fprintf(stderr, "Error: %s: empty MAC address string\n", __FUNCTION__);
        status = -1;
        goto e_assign_MacAddress;
    }
    blen = x;

    // The string for the MAC address needs to be 6 octets (or 12 characters)
    if (blen != (NELEM(mac_addr) * 2))
    {
        fprintf(stderr, "Error: %s: non-MAC address string '%s'\n", __FUNCTION__, buf);
        status = -1;
        goto e_assign_MacAddress;
    }

    // Convert the string into a MAC address array
    uint64_t val = strtoull(buf, NULL, 16);
    for (x = 0; x < NELEM(mac_addr); x++)
    {
        mac_addr[(NELEM(mac_addr) - x) - 1] = (uint8_t) (val >> (8 * x));
    }

    // Make sure the MAC address has the expected OUI
    if (memcmp(mac_addr, reach_oui, sizeof(reach_oui)) != 0)
    {
        char dbuf1[32];
        char dbuf2[32];

        fprintf(stderr, "Error: %s: MAC address OUI (%s) does not match expected OUI (%s)\n", __FUNCTION__, DbgOctet(mac_addr, NELEM(mac_addr), dbuf1, sizeof(dbuf1)), DbgOctet(reach_oui, NELEM(reach_oui), dbuf2, sizeof(dbuf2)));
        status = -1;
        goto e_assign_MacAddress;
    }

    /* this is for the lower 16 bits - 8C:XX:XX:XX*/
    sprintf(path, "/sys/fsl_otp/HW_OCOTP_MAC0");
    fp = fopen(path, "a+b");
    if (fp != NULL)
    {
        uint32_t new_otp = (mac_addr[2] << 24) | (mac_addr[3] << 16) | (mac_addr[4] << 8) | mac_addr[5];
        uint32_t cur_otp = ~0x0;

        //fprintf(stdout, "New OTP:0x%X \n",new_otp);
        //goto e_assign_MacAddress;

        rv = fscanf(fp, "%X", &cur_otp);
        if (rv != 1)
        {
            fprintf(stderr, "Error: %s: unable to read current OTP value from '%s' [%d]\n", __FUNCTION__, path, rv);
            status = -1;
            goto e_assign_MacAddress;
        }
        rewind(fp);

        if (g_info.verbose) fprintf(stdout, "Debug: %s: OTP Current:0x%08X New:0x%08X.\n", __FUNCTION__, cur_otp, new_otp);

        cur_otp &= 0xFFFFFFFF;

        //fprintf(stdout, "Current OTP:0x%X \n",cur_otp);

        if (cur_otp == new_otp)
        {
            if (g_info.verbose) fprintf(stdout, "Debug: %s: OTP value 0x%08X already matches value in '%s'.\n", __FUNCTION__, new_otp, path);
            goto mac1_begin;
        }

        if (cur_otp != 0x0)
        {
            fprintf(stderr, "Error: %s: '%s' OTP has already been programmed to 0x%08X. Cannot set new value of 0x%08X\n", __FUNCTION__, path, cur_otp, new_otp);
            status = -1;
            goto mac1_begin;
        }

        if (g_info.verbose) fprintf(stdout, "Debug: %s: write OTP value 0x%08X to '%s'.\n", __FUNCTION__, new_otp, path);
        if (!g_info.dryrun)
        {
            rv = fprintf(fp, "0x%08X", new_otp); fflush(fp);
            if (rv < 0)
            {
                fprintf(stderr, "Error: %s: fprintf('%s') failed: %s [%d]\n", __FUNCTION__, path, strerror(errno), errno);
                status = -1;
                goto e_assign_MacAddress;
            }
        }
        else
        {
            fprintf(stdout, "DryRun: %s: write OTP value 0x%08X to '%s'.\n", __FUNCTION__, new_otp, path);
        }
        rewind(fp);

        rv = fscanf(fp, "%X", &cur_otp);
        if (rv != 1)
        {
            fprintf(stderr, "Error: %s: unable to re-read current OTP value from '%s' [%d]\n", __FUNCTION__, path, rv);
            status = -1;
            goto mac1_begin;
        }

        if (cur_otp != new_otp)
        {
            fprintf(stderr, "Error: %s: OTP value ('%s') verify failed. Expected:0x%08X Actual:0x%08X\n", __FUNCTION__, path, new_otp, cur_otp);
            status = -1;
            goto e_assign_MacAddress;
        }

        fclose(fp);
        fp = NULL;
    }
    else
    {
        fprintf(stderr, "Error: %s: fopen('%s') failed: %s [%d]\n", __FUNCTION__, path, strerror(errno), errno);
        status = -1;
        goto e_assign_MacAddress;
    }

mac1_begin:
    /* this is for the upper 12 bits - XX:XX*/
    sprintf(path, "/sys/fsl_otp/HW_OCOTP_MAC1");
    fp = fopen(path, "a+b");
    if (fp != NULL)
    {
        uint32_t new_otp = (mac_addr[0] << 8) | (mac_addr[1]);
        uint32_t cur_otp = ~0x0;

        fprintf(stdout, "New OTP:0x%X \n",new_otp);
        //goto e_assign_MacAddress;

        rv = fscanf(fp, "%X", &cur_otp);
        if (rv != 1)
        {
            fprintf(stderr, "Error: %s: unable to read current OTP value from '%s' [%d]\n", __FUNCTION__, path, rv);
            status = -1;
            goto e_assign_MacAddress;
        }
        rewind(fp);

        if (g_info.verbose) fprintf(stdout, "Debug: %s: OTP Current:0x%08X New:0x%08X.\n", __FUNCTION__, cur_otp, new_otp);

        cur_otp &= 0xFFFFFFFF;

        //fprintf(stdout, "Current OTP:0x%X \n",cur_otp);

        if (cur_otp == new_otp)
        {
            if (g_info.verbose) fprintf(stdout, "Debug: %s: OTP value 0x%08X already matches value in '%s'.\n", __FUNCTION__, new_otp, path);
            goto e_assign_MacAddress;
        }

        if (cur_otp != 0x0)
        {
            fprintf(stderr, "Error: %s: '%s' OTP has already been programmed to 0x%08X. Cannot set new value of 0x%08X\n", __FUNCTION__, path, cur_otp, new_otp);
            status = -1;
            goto e_assign_MacAddress;
        }

        if (g_info.verbose) fprintf(stdout, "Debug: %s: write OTP value 0x%08X to '%s'.\n", __FUNCTION__, new_otp, path);
        if (!g_info.dryrun)
        {
            rv = fprintf(fp, "0x%08X", new_otp); fflush(fp);
            if (rv < 0)
            {
                fprintf(stderr, "Error: %s: fprintf('%s') failed: %s [%d]\n", __FUNCTION__, path, strerror(errno), errno);
                status = -1;
                goto e_assign_MacAddress;
            }
        }
        else
        {
            fprintf(stdout, "DryRun: %s: write OTP value 0x%08X to '%s'.\n", __FUNCTION__, new_otp, path);
        }
        rewind(fp);

        rv = fscanf(fp, "%X", &cur_otp);
        if (rv != 1)
        {
            fprintf(stderr, "Error: %s: unable to re-read current OTP value from '%s' [%d]\n", __FUNCTION__, path, rv);
            status = -1;
            goto e_assign_MacAddress;
        }

        if (cur_otp != new_otp)
        {
            fprintf(stderr, "Error: %s: OTP value ('%s') verify failed. Expected:0x%08X Actual:0x%08X\n", __FUNCTION__, path, new_otp, cur_otp);
            status = -1;
            goto e_assign_MacAddress;
        }

        fclose(fp);
        fp = NULL;
    }
    else
    {
        fprintf(stderr, "Error: %s: fopen('%s') failed: %s [%d]\n", __FUNCTION__, path, strerror(errno), errno);
        status = -1;
        goto e_assign_MacAddress;
    }
e_assign_MacAddress:
    if (fp != NULL)
    {
        fclose(fp);
        fp = NULL;
    }

    return status;
}

struct {
    const char *part;
    const char *name;
    testFunc_t  func;
#define _TEST_NONE  (0x00)
#define _TEST_P1    (0x01 << 0)
#define _TEST_P2    (0x01 << 1)
    uint8_t     flags;
} MfgTests[] = {
    { "J3",     "Ethernet",         test_Ethernet,     _TEST_P1 },
#define TESTS_PHASE1    12                                
    { "J13",    "LCD",              test_LCD,          _TEST_P2 },
    { "J13",    "Backlight",        test_Backlight,    _TEST_P2 },
};

/****************************************************************************
 * usage
 */
static void
usage(const char *prog_name)
{
    fprintf(stdout, "Usage: %s [options]\n", prog_name);
    fprintf(stdout, "  Perform manufacturing tests\n");
    fprintf(stdout, "\n");
    fprintf(stdout, "Options:\n");
    fprintf(stdout, "  -h, --help                   Display this help and exit\n");
    fprintf(stdout, "  --version                    Report program version\n");
    fprintf(stdout, "  -v, --verbose                Enable verbose debug (default:%s)\n", g_info.verbose ? "Enabled" : "Disabled");
    fprintf(stdout, "  -s, --server {ip_addr}       IP address of Ethernet server (default:%s)\n", g_info.ethernet.server_ip ? g_info.ethernet.server_ip : "None");
    fprintf(stdout, "  -l, --local {ip_addr}        IP address of local Ethernet interface (default:%s)\n", g_info.ethernet.local_ip ? g_info.ethernet.local_ip : "None");
    fprintf(stdout, "  -t, --tests {test}           List of tests to run (default:%s)\n", g_info.tests);
    fprintf(stdout, "  --list-tests                 List all available manufacturing tests\n");
    fprintf(stdout, "  --mac-address[={mac_addr}]   Set Ethernet interface MAC address to either the specified\n");
    fprintf(stdout, "                               {mac_addr} (%s:xx:xx:xx), or if not specified, the\n", DbgOctet(reach_oui, NELEM(reach_oui), NULL, 0));
    fprintf(stdout, "                               MAC address will be read use the USB barcode scanner\n");
    fprintf(stdout, "  --repeat={n_repeat}          Repeat the list of tests (default:%d)\n", g_info.repeat);
    fprintf(stdout, "  --dry-run                    Do not perform OTP write of MAC address or other permanent changes.\n");
    fprintf(stdout, "  --image-dir={path}           Directory with framebuffer test images.\n");
    fprintf(stdout, "  --version                    Display version information and exit\n");
}

/****************************************************************************
 * main
 */
int
main(int argc, char *argv[])
{
    int ret = 0;
    int opt;
    int option_index = 0;
    int x;
    bool bHelp = FALSE;
    bool bTestList = FALSE;
    uint32_t test_mask;
    char *p;

    static const char *short_options = "s:l:t:vh";
    static const struct option long_options[] = {
        { "debug",       optional_argument,  0, 0 },
        { "version",     no_argument,        0, 0 },
        { "verbose",     no_argument,        0, 'v' },
        { "dry-run",     no_argument,        0, 0 },
        { "server",      required_argument,  0, 's' },
        { "local",       required_argument,  0, 'l' },
        { "tests",       required_argument,  0, 't' },
        { "list-tests",  no_argument,        0, 0 },
        { "mac-address", optional_argument,  0, 0 },
        { "repeat",      required_argument,  0, 0 },
        { "image-dir",   required_argument,  0, 0 },
        { "help",        no_argument,        0, 'h' },
        { 0, 0, 0, 0 },
    };

    g_info.tests = strdup("all");

    while ((opt = getopt_long(argc, argv, short_options,
        long_options, &option_index)) != -1)
    {
        switch (opt)
        {
        case 0:
            switch (option_index)
            {
            case 0: // debug
                if (optarg)
                {
                    g_info.debug = strtoul(optarg, NULL, 0);
                }
                else
                {
                    g_info.debug = ~0x0;
                }

                fprintf(stdout, "%s: Debug 0x%X\n", argv[0], g_info.debug);
                break;

            case 1: // version
                fprintf(stdout, "%s: Version %s\n", argv[0], VERSION);
                ret = 0;
                goto e_main;
                break;

            case 2: // verbose
                g_info.verbose = TRUE;
                break;

            case 3: // dry-run
                g_info.dryrun = TRUE;
                break;

            case 4: // server
                if (optarg)
                {
                    g_info.ethernet.server_ip = strdup(optarg);
                }
                break;

            case 5: // local
                if (optarg)
                {
                    g_info.ethernet.local_ip = strdup(optarg);
                }
                break;

            case 6: // tests
                if (optarg)
                {
                    g_info.tests = strdup(optarg);
                }
                break;

            case 7: // list-tests
                bTestList = TRUE;
                break;

            case 8: // mac-address
                if (optarg)
                {
                    g_info.mac_address = strdup(optarg);
                }
                else
                {
                    g_info.mac_address = strdup(USER_INPUT_STRING);
                }
                break;

            case 9: // rs485-baud
                if (optarg)
                {
                    g_info.rs485_baud = baud_str_to_key(optarg);
                }
                break;

            case 10: // rs232-baud
                if (optarg)
                {
                    g_info.rs232_baud = baud_str_to_key(optarg);
                }
                break;

            case 11: // buffer-size
                if (optarg)
                {
                    g_info.buffer_size = strtoul(optarg, NULL, 0);
                }
                break;

            case 12: // i2c-addr
                if (optarg)
                {
                    g_info.i2c_test_addr = (uint8_t) strtoul(optarg, NULL, 0);
                }
                break;

            case 13: // i2c-offset
                if (optarg)
                {
                    g_info.i2c_test_offset = (uint8_t) strtoul(optarg, NULL, 0);
                }
                break;

            case 14: // spi-bus
                if (optarg)
                {
                    g_info.spi_bus = (uint8_t) strtoul(optarg, NULL, 0);
                }
                break;

            case 15: // rtc-if
                if (optarg)
                {
                    for (x = 0; x < NELEM(RtcIfList); x++)
                    {
                        if (strcasecmp(optarg, RtcIfList[x]) == 0)
                        {
                            g_info.rtc_if = x;
                            break;
                        }
                    }
                    if (x >= NELEM(RtcIfList))
                    {
                        fprintf(stdout, "Error: Unknown rtc-if option '%s'\n", optarg);
                        ret = 1;
                    }
                }
                break;

            case 16: // repeat
                if (optarg)
                {
                    g_info.repeat = strtoul(optarg, NULL, 0);
                }
                break;
                
            case 17: // image-dir
                if (optarg)
                {
                    g_info.image_dir = strdup(optarg);
                }
                break;

            default:
                fprintf(stdout, "Option '%s'", long_options[option_index].name);
                if (optarg)
                {
                    fprintf(stdout, " with arg '%s'", optarg);
                }
                fprintf(stdout, "\n");
            }
            break;

        case 'v':
            g_info.verbose = TRUE;
            break;

        case 's':
            if (optarg)
            {
                g_info.ethernet.server_ip = strdup(optarg);
            }
            break;

        case 'l':
            if (optarg)
            {
                g_info.ethernet.local_ip = strdup(optarg);
            }
            break;

        case 't':
            if (optarg)
            {
                g_info.tests = strdup(optarg);
            }
            break;

        case 'h':
            bHelp = TRUE;
            break;

        case '?':
            // Error already handled by getopt_long()
            ret = 1;
            break;

        default:
            fprintf(stdout, "Unknown option '%c'\n", opt);
            ret = 1;
            break;
        }
    }

    if (bHelp)
    {
        usage(argv[0]);
    }
	
    if (bTestList)
    {
        char phase_str[64];

        for (x = 0; x < NELEM(MfgTests); x++)
        {
            phase_str[0] = '\0';
            if (MfgTests[x].flags & _TEST_P1)
            {
                strcat(phase_str, "Phase1");
            }
            if (MfgTests[x].flags & _TEST_P2)
            {
                if (phase_str[0] != '\0') strcat(phase_str, "/");
                strcat(phase_str, "Phase2");
            }
            fprintf(stdout, "  %-8s %-15s %s\n", MfgTests[x].part, MfgTests[x].name, phase_str);
        }
    }

    if (bHelp || bTestList)
    {
        ret = 0;
        goto e_main;
    }


    if (ret != 0)
    {
        goto e_main;
    }

    if (g_info.mac_address != NULL)
    {
        int rv;

        rv = assign_MacAddress(g_info.mac_address);
        fprintf(stdout, "Action: Assign MAC Address: %s\n", (rv < 0) ? STR_FAIL : ((rv == 0) ? STR_PASS : STR_UNTESTED));
        goto e_main;
    }

    test_mask = 0x0;
    for (p = strtok(g_info.tests, ","); p != NULL; p = strtok(NULL, ","))
    {
        if (strcasecmp(p, "all") == 0)
        {
            test_mask = ~0x0;
        }
        else if (strcasecmp(p, "phase1") == 0)
        {
            for (x = 0; x < NELEM(MfgTests); x++)
            {
                if (MfgTests[x].flags & _TEST_P1)
                {
                    test_mask |= (0x01 << x);
                }
            }
        }
        else if (strcasecmp(p, "phase2") == 0)
        {
            for (x = 0; x < NELEM(MfgTests); x++)
            {
                if (MfgTests[x].flags & _TEST_P2)
                {
                    test_mask |= (0x01 << x);
                }
            }
        }
        else
        {
            for (x = 0; x < NELEM(MfgTests); x++)
            {
                if ((strcasecmp(p, MfgTests[x].name) == 0) ||
                    (strcasestr(p, MfgTests[x].part) != NULL))
                {
                    test_mask |= (0x01 << x);
                }
            }
        }
    }

    uint32_t loop;

    int n_digits = 0;
    for (loop = 1; loop <= g_info.repeat; loop *= 10)
    {
        n_digits++;
    }

    for (loop = 0; loop < g_info.repeat; loop++)
    {
        char loop_str[64];

        if (g_info.repeat > 1)
        {
            sprintf(loop_str, "%*u: ", n_digits, loop + 1);
        }
        else
        {
            loop_str[0] = '\0';
        }

        for (x = 0; x < NELEM(MfgTests); x++)
        {
            int rv;
            char name[32];

            if (!(test_mask & (0x01 << x))) continue;

            sprintf(name, "%s (%s)", MfgTests[x].name, MfgTests[x].part);

            if (g_info.verbose) fprintf(stdout, "Debug: %sTesting %s\n", loop_str, name);
            rv = MfgTests[x].func();
            fprintf(stderr, "Test: %s%-20s %s\n", loop_str, name, (rv < 0) ? STR_FAIL : ((rv == 0) ? STR_PASS : STR_UNTESTED));
        }
    }

e_main:

    return ret;
}

