#include <stdlib.h>
#include <unistd.h>
#include <stdio.h>
#include <fcntl.h>
#include <errno.h>
#include <linux/fb.h>
#include <sys/mman.h>
#include <sys/ioctl.h>

typedef struct {
   unsigned int width;
   unsigned int height;
   unsigned int planes;
   unsigned short bitcount;
   unsigned int size;
} BITMAPINFOHEADER;

typedef struct {
   unsigned char blue;
   unsigned char green;
   unsigned char red;
} PIXEL;

int main(int argc, char **argv)
{
    int fbfd 				= 0;
    int tfd 				= 0;
    char buf[16];
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

    if(argc < 2) {
        printf("bmp file required!");
        return 1;
    }

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

    printf("Opening file: %s\n", argv[1]);
    if(!(image = fopen(argv[1], "rb+")))
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

    PIXEL pic[bih.width*bih.height*2],p;

    fseek(image,54,SEEK_SET);

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
                j++; //increment pixel pointer.
            }

        }
    }
    fclose(image);

    munmap(fbp, screensize);
    close(fbfd);
    
    /* open the touch device */
    tfd = open("/dev/input/touchscreen0", O_RDONLY);
    if (tfd == -1) {
        perror("Error: cannot open touch device");
        exit(1);
    }
    printf("Touch image to continue \n");
    nbytes = sizeof(buf);
    bytes_read = read(tfd, buf, nbytes);
            
    close(tfd);
    return 0;
}

