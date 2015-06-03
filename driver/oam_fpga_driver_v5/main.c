#include <stdio.h> 
#include <fcntl.h> 
#include <string.h>
#include "oam_fpga_drv.h"

/* A example for dealing with the io_port */
int main(void)
{
	int				i, j;
	int				flag;
	int				fd, rc, data;
	oam_fpga_reg			reg;
	char				* opr_name;
	char				* dev_name	= "/dev/oam_fpga_drv";

   	printf( "\t Open the device : %s\n", dev_name ); 
   	if( (fd = open( dev_name, O_RDWR )) == -1 )
   	{
   		printf( "\n\t Cann't open the device : %s\n", dev_name ); 
		return (0);
   	}

	memset(&reg, 0, sizeof(reg));	

   	flag	= 1;
   	while( flag )
   	{
   		printf( "\n\t read[1] or write[2] or reset[3] or quit[0]: " );
    	scanf( "%d", &data );
    	
   		switch( data )
   		{
   			case 0:
	    		flag	= 0;
    		break;
    		case 1:
    			opr_name	= "STATUS_OAM_FPGA_GET";
    			
		    	printf( "\t\t input the cid (hex): " );
		    	scanf( "%x", &data );
				reg.cid = data;
				
				printf( "\t\t input the addr (hex): " );
		    	scanf( "%x", &data );
				reg.addr = data;

				printf( "\t\t input the len (hex): " );
				scanf( "%x", &data );
				reg.len	= data;
				if( reg.len > sizeof(reg.buf) )
				{
					printf( "\n\t\t The size must be less than %d\n", sizeof(reg.buf) ); 
					flag	= 0;
				}
				else
				{
					printf( "\t\t read the register on the %02X:%02X:%02X\n\t\t", reg.cid, reg.addr, reg.len);

					if( (rc = ioctl( fd, STATUS_OAM_FPGA_GET, (unsigned long)(&reg) )) < 0 )
					{
						printf( "\t\t Cann't %s the device : rc = %d \n", opr_name, rc ); 
						flag	= 0;
					}
					else
					{
						for( i = 0; i < reg.len; i++ )
							printf( "0x%02X ", reg.buf[i] );
						printf( "\n" );
    				}
    			}
    		break;
    		case 2:
    			opr_name	= "STATUS_OAM_FPGA_SET";
    			
    			printf( "\t\t input the cid (hex): " );
		    	scanf( "%x", &data );
				reg.cid = data;
    			
		    	printf( "\t\t input the addr (hex): " );
		    	scanf( "%x", &data );
				reg.addr = data;

				printf( "\t\t input the len (hex): " );
				scanf( "%x", &data );
				reg.len	= data;
				if( reg.len > sizeof(reg.buf) )
				{
					printf( "\n\t\t The size must be less than %d\n", sizeof(reg.buf) ); 
					flag	= 0;
				}
				else
				{
					for( i = 0; i < reg.len; i++ )
					{
						printf( "\t\t input number_%d (hex): ", i );
						scanf( "%x", &data );
						reg.buf[i]	= data;
					}
					
					printf( "\t\t write the register on the %02X:%02X:%02X\n\t\t ", reg.cid, reg.addr, reg.len );
					for( i = 0; i < reg.len; i++ )
						printf( "0x%02X ", reg.buf[i] );
					printf( "\n" );
						
					if( (rc = ioctl( fd, STATUS_OAM_FPGA_SET, (unsigned long)(&reg) ))  < 0 )
    				{
    					printf( "\t\t Cann't %s the device : rc = %d \n", opr_name , rc ); 
    					flag	= 0;
    				}
    				else
    				{
    					printf( "\t\t %s the device \n", opr_name  ); 
    				}
				}
    		break;
    		
    	}
	}

	if( (rc = close( fd )) )
	{
		printf( "\t Cann't close the device : %s \n", dev_name );
	}
	else
	{
		printf( "\t Close the device : %s \n", dev_name ); 
	}

    return (0);
} 
