#include <stdio.h> 
#include <fcntl.h> 
#include <string.h>

#include "spi_drv.h"

/* A example for dealing with the io_port */
int main(void)
{
	char	* dev_name	= "/dev/spi_drv";
	spi_reg	reg;
	int fd, flag = 1, i;
	u8 data = 0;
	
   	printf( "\t Open the device : %s\n", dev_name );
   	if( (fd = open( dev_name, O_RDWR )) == -1 )
   	{
   		printf( "\n\t Cann't open the device : %s\n", dev_name );
		return (0);
   	}

	memset(&reg, 0, sizeof(reg));
	
	while( flag )
   	{
		data = 0;
   		printf( "\n\t read[1] or write[2] or reset[3] or quit[0]: " );
    	scanf( "%d", &data );
    	
   		switch( data )
   		{
   			case 0:
	    		flag = 0;
				break;
    		case 1:
		    	printf( "\t\t input the addr (hex): " );
		    	scanf( "%x", &data );
				reg.addr = data;
				printf( "\t\t input the size (hex): " );
				scanf( "%x", &data );
				reg.size = data;
				if( reg.size > MULTI_REG_LEN_MAX )
				{
					printf( "\n\t\t The size must be less than %d\n", sizeof(reg.pbuf) ); 
					flag = 0;
				}
				else
				{
					printf( "\t\t read the register on the %02X:%02X\n\t\t", reg.addr, reg.size);
					ioctl(fd, STATUS_SPI_GET, &reg);
					for( i = 0; i < reg.size; i++ )
						printf( "0x%04X ", reg.pbuf[i]);
					printf( "\n" );
    			}
    		break;
    		case 2:
		    	printf( "\t\t input the addr (hex): " );
		    	scanf( "%x", &data );
				reg.addr = data;
				printf( "\t\t input the size (hex): " );
				scanf( "%x", &data );
				reg.size	= data;
				if( reg.size > MULTI_REG_LEN_MAX )
				{
					printf( "\n\t\t The size must be less than %d\n", sizeof(reg.pbuf) ); 
					flag	= 0;
				}
				else
				{
					for( i = 0; i < reg.size; i++ )
					{
						printf( "\t\t input number_%d (hex): ", i );
						scanf( "%x", &data );
						reg.pbuf[i]	= data;
					}
					
					printf( "\t\t write the register on the %02X:%02X\n\t\t ", reg.addr, reg.size );
					for( i = 0; i < reg.size; i++ )
						printf( "0x%04X ", reg.pbuf[i] );
					printf( "\n" );
						
					ioctl(fd, STATUS_SPI_SET, &reg);
				}
    		break;
    	}
	}

	close(fd);

    return (0);
} 
