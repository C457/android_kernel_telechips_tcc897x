#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <sys/mman.h>
#include <sys/ioctl.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <stdint.h>
#include <linux/types.h>
#include <sys/mman.h>
#include <linux/fb.h>
#include <sys/kd.h>
#include <pthread.h>
#include <errno.h>
#include <stdbool.h>
#include <sys/poll.h>
#include <getopt.h>
#include <termios.h>
#include <ctype.h>
#define CONFIG_TCC_REAR_CAMERA_CPU_WITH_OVERSCAN
#include "../rear_cam_drv.h"
#include "../tcc_rear_cam_ioctl.h"
#include "pg002.h"

void help_msg(char* argv)
{
	printf("------------------------------------------------------------\n"
           "|  Usage: %s \n"
           "|  Caution some command do not working on TCC893X or TCC892X\n"           
           "| ===============\n"
           "| <Show/Hide Rear-camera but rear-cam use hw resource>\n" 
           "|    ex) show  \n"
           "|       $ --show=1 or -v 1 \n"
           "|    ex) hide \n"
           "|       $ --show=0 or -v 0 \n"
           "| <Show/Hide Rear-camera with release hw resource>\n" 
           "|    ex) show  \n"
           "|       $ --hw_show=1 or -h 1 \n"
           "|    ex) hide \n"
           "|       $ --hw_show=0 or -h 0 \n"
           "| <Show/Hide Parking guide view with release hw resource>\n" 
           "|    ex) show  \n"
           "|       $ --guide_line=1 or -g 1 \n"
           "|    ex) hide \n"
           "|       $ --guide_line=0 or -g 0 \n"
           "| <Enable/Disable Rear-gear event detect >\n" 
           "|    ex) Enable  \n"
           "|       $ --set_rgear_event=1 or -s 1 \n"
           "|    ex) Disable \n"
           "|       $ --set_rgear_event=0 or -s 0 \n"
           "| <Get view/hide or rear-gear status of rear-camera  >\n" 
           "|    ex)   \n"
           "|       $ --get_status or -i  \n"
           "| <Get view/hide or rear-gear status of rear-camera when rear-gear-event is up/down. >\n" 
           "|    ex)   \n"
           "|       $ --get_status_event or -b  \n"
           "| <Get parking-guide-line buffer address and than Set new data. >\n" 
           "|    ex)   buffer[0] \n"
           "|       $ --get_line_buffer=0 or -l 0  \n"
           "|    ex)   buffer[1] \n"
           "|       $ --get_line_buffer=1 or -l 1  \n"           
           "| <Release video-in hw resource. >\n" 
           "|    ex)   \n"
           "|       $ --release_resource or -r  \n"
           "| <Set Overscan. >\n" 
           "|    ex)   \n"
           "|       $ --overscan=posx,posy,overscan_width_factor,overscan_height_factor\n"
           "| <Set parking-guide-line size > \n"
           "|    ex)   \n"
           "|	$ -a size_width,size_height \n"
           "| <Set parking-guide-line position >\n"
           "|    ex)  \n"
           "|       $ -z position_x,position_y \n"
           "| <Set preview position > \n"
           "|    ex) \n"
           "|       $ -x position_x,position_y \n"
           "| <Set preview size > \n"
           "|    ex) \n"
           "|      $ -c size_width,size_height \n"
           "|\n"
           "|\n"
           "------------------------------------------------------------\n",argv);
}


int main(int argc, char* argv[])
{
	int                 dev_int;
	struct pollfd       Events[ 1];
	int                 retval,count;
	unsigned long rear_event_flag;
	unsigned long rear_linebuf_addr;
	unsigned long width,height,lineBufSize;
	char *LineBuf;
	char switch_num;
	int option_idx = 0;
	int opt_val;

	static struct option long_opt[] = {
		{"show", required_argument, 0, 'v'},
		{"guide_line", required_argument, 0, 'g'},
		{"set_rgear_event",required_argument,0, 's'},
		{"get_status", no_argument, 0, 'i'},
		{"release_resource", no_argument, 0, 'r'},
		{"get_status_event", no_argument, 0, 'b'},
		{"get_line_buffer", no_argument, 0, 'l'},
        	{"overscan", required_argument, 0, 'o'},
   		{"parking_guide_line_size", required_argument, 0, 'a'},
		{"parking_guide_line_position", required_argument, 0, 'z'},
		{"preview_position", required_argument, 0, 'x'},
		{"preview_size", required_argument, 0, 'c'},
		{0, 0, 0, 0}
	};

	if (argc < 2) {
		help_msg(argv[0]);
		return -1;
	}
	else {

		dev_int   = open("/dev/rear_cam_ctrl",O_RDWR);

		if(dev_int < 0){
			printf("err dev_int\n");
			return -1;
		}

		printf("program start\n");

		while(1)
		{
			switch_num = getopt_long(argc, argv,"irbl:v:h:g:s:z:x:c:a:" , long_opt, &option_idx); 
			if (switch_num == -1) { break; }
			if(optarg) opt_val = atoi(optarg);
			
			switch(switch_num)
			{
				case 0:
					if (long_opt[option_idx].flag != 0)
						break;
					printf ("option %s", long_opt[option_idx].name);

					if (optarg)
						printf (" with arg %s\n", optarg);
					break;
					
				case 'v':

					if(opt_val){
					       ioctl(dev_int,RCAM_DIRECT_ON);
					}
					else{
					       ioctl(dev_int,RCAM_DIRECT_OFF);
					}

   					printf("RCAM_DIRECT_: 0x%x \n", opt_val);
				       break;

				case 'g':

					if(opt_val){
					       ioctl(dev_int,RCAM_LINE_ON);
					}
					else{
					       ioctl(dev_int,RCAM_LINE_OFF);
					}
	   					printf("RCAM_RESOURCE_: 0x%x \n", opt_val);
				       break;
					   
				case 'i':
				       ioctl(dev_int,RCAM_GET_REAR_STATUS,&rear_event_flag);
					printf("RCAM_GET_REAR_STATUS: 0x%x \n", rear_event_flag);

					break;

				case 'b':
				       ioctl(dev_int,RCAM_WAIT_REAR_EVENT,&rear_event_flag);
					printf("RCAM_WAIT_REAR_EVENT: 0x%x \n", rear_event_flag);
					break;

				case 'r':
				       ioctl(dev_int,RCAM_RELEASE_VIDEO);
					printf("RCAM_RELEASE_VIDEO: Video path off \n");
					break;

				case 's':
					if(opt_val){						
						ioctl(dev_int,RCAM_REAR_EVENT_SET);
						printf("RCAM_REAR_EVENT_SET: Set Rear-gear event \n");
					}
					else {
						ioctl(dev_int,RCAM_REAR_EVENT_UNSET);
						printf("RCAM_REAR_EVENT_SET: Un-Set Rear-gear event \n");
					}
					
					break;

				case 'l':
					if(optarg) {
						RCAM_LINE_BUFFER_INFO rcam_line_buffer_info;
						
	                        		sscanf(optarg, "%d",
										&rcam_line_buffer_info.index);

		                            ioctl(dev_int,RCAM_LINEBUF_ADDR_WITH_INDEX, &rcam_line_buffer_info);

						LineBuf = mmap(0,rcam_line_buffer_info.buffer_size ,PROT_WRITE | PROT_READ, MAP_SHARED, dev_int,
							(off_t) rcam_line_buffer_info.rear_linebuf_addr);

						if(LineBuf == MAP_FAILED){
							perror(" LineBuf mmap error");
							break;
						}
						else{
							printf("mmap : 0x%x\n",LineBuf);
						}

						/* Draw Line */
						memcpy((unsigned char*)LineBuf ,pg002_img,pg002_img_len);

						ioctl(dev_int,RCAM_LINE_UPDATE,1);
			 
						munmap(LineBuf,lineBufSize);
					}
				       break;
				case 'o':
					if(optarg) {
	                        		RCAM_SCALE_OVERSCAN_INFO rcam_scale_overscan;
	                        		sscanf(optarg, "%d,%d,%d,%d", &rcam_scale_overscan.overscan_position_x, &rcam_scale_overscan.overscan_position_y, &rcam_scale_overscan.overscan_width_factor, &rcam_scale_overscan.overscan_height_factor);

		                            ioctl(dev_int,RCAM_SCALE_OVERSCAN, &rcam_scale_overscan);
	                    		}
	                    		break;
				case 'a':			// set parking line size
					if(optarg) {
	                        		RCAM_LINE_SIZE_INFO rcam_line_size_info;
						
	                        		sscanf(optarg, "%d,%d", &rcam_line_size_info.line_x, &rcam_line_size_info.line_y);
		                            ioctl(dev_int,RCAM_LINE_SET_SIZE, &rcam_line_size_info);
	                    		}						
					break;
				case 'z':			// set parking guide line position
					if(optarg) {
	                        		RCAM_LINE_POSITION_INFO rcam_line_position_info;
						
	                        		sscanf(optarg, "%d,%d", &rcam_line_position_info.line_x, &rcam_line_position_info.line_y);
		                            ioctl(dev_int,RCAM_LINE_SET_POSITION, &rcam_line_position_info);
	                    		}						
					break;
			  	case 'x':			// set preview position
					if(optarg) {
						RCAM_PREVIEW_POSITION_INFO rcam_preview_position_info;
						
	                        		sscanf(optarg, "%d,%d", &rcam_preview_position_info.preview_position_x, &rcam_preview_position_info.preview_position_y);
		                            ioctl(dev_int,RCAM_PREVIEW_SET_POSITION, &rcam_preview_position_info);
	                    		}					
					break;
				case 'c':			// set preview size
					if(optarg) {
						RCAM_PREVIEW_SIZE_INFO rcam_preview_size_info;
						
	                        		sscanf(optarg, "%d,%d", &rcam_preview_size_info.preview_width, &rcam_preview_size_info.preview_height);
		                            ioctl(dev_int,RCAM_PREVIEW_SET_SIZE, &rcam_preview_size_info);
	                    		}				
					break;
			  default :
			  	printf("Wrong options \n");
					break;
			}

			break;
		}
	       close(dev_int);
	} 
       return 0;
} 
