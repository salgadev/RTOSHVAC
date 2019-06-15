#include <stdio.h>
#include <string.h>
#include <unistd.h>
#include <fcntl.h>
#include <errno.h>
#include <termios.h>
#include <stdlib.h>
#include <sys/io.h>

#define parallel 0x378 //Parallel port address

struct variables{
      int fd; 
      char *data;
      int ssize; 
      int time;
      };
struct variables v;
struct timespec next;
float t=0;
v.ssize=1; 
v.time=500000;

int *tthread_serie(void *args){
   clock_gettime(CLOCK_REALTIME, &next);
   next.tv_sec = next.tv_sec + 0.05; // otros 5ms
   next.tv_nsec = 0;
   v.fd = open("/dev/ttyS0", O_RDWR | O_NOCTTY | O_NDELAY);
   if (v.fd == -1)
      { 
      perror("\n ERROR: Unable to open port. /dev/ttyS0 - \n");
      }
      else
	fcntl(v.fd, F_SETFL, 0);
      printf("\r   Port open... id: %d \n",v.fd);
      return (v.fd);
    }

void *tthread_config(void *args){
   struct timespec next;
   clock_gettime(CLOCK_REALTIME, &next);
   next.tv_sec = next.tv_sec + 0.05; // 5 ms
   next.tv_nsec = 0;
   clock_nanosleep(CLOCK_REALTIME, TIMER_ABSTIME, &next, NULL);
   struct termios options;
   tcgetattr(v.fd, &options);
   cfsetispeed(&options, B300);
   cfsetospeed(&options, B300);
   options.c_cflag |= (CLOCAL | CREAD);
   options.c_cflag &= ~Cssize; 
   options.c_cflag |= CS8;    
   options.c_cflag &= ~PARENB;
   options.c_cflag &= ~CSTOPB;
   options.c_cflag &= ~Cssize;
   options.c_cflag |= CS8;
   options.c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG);
   tcsetattr(v.fd, TCSANOW, &options);
   printf("\r   Port configured succesfully\n");
   }

int *tthread_leer(int serial_fd, char *data, int ssize, int timeout_usec){
   struct timespec next;
   clock_gettime(CLOCK_REALTIME, &next);
   next.tv_sec = next.tv_sec + 0.1; // 100 ms
   next.tv_nsec = 0;
   clock_nanosleep(CLOCK_REALTIME, TIMER_ABSTIME, &next, NULL);
   fd_set fds;
   struct timeval timeout;
   int count=0;
   int ret;
   int n;
   do {
      FD_ZERO(&fds);
      FD_SET (serial_fd, &fds);
      timeout.tv_sec = 0;  
      timeout.tv_usec = timeout_usec;
      ret=select (FD_SETssize,&fds, NULL, NULL,&timeout);
      if (ret==1) {
         printf("\r Communication established \n");
         n=read (serial_fd, &data, v.ssize); 
         printf("   n = %d\n", n);
         printf("data = %p\n ", data);
         count+=1;
         data=0;
         }
      }while (count<ssize && ret==1);
   return n;
   }

float *tthread_temp(int adc){
   struct timespec next;
   clock_gettime(CLOCK_REALTIME, &next);
   next.tv_sec = next.tv_sec + 0.01; // 10 ms
   next.tv_nsec = 0;
   clock_nanosleep(CLOCK_REALTIME, TIMER_ABSTIME, &next, NULL);
   printf("\r adc = \n",adc);
   float volt=0;
   volt=adc/255; 
   t=volt*4.8*100;
   }

main(){
   int x; //adc variable
   unsigned int p = 0x01; //indicates 1 output
   float res; //temp value in ºC 
   printf("\r \n");
   if(ioperm(parallel,1,1)){ //parallel port permission
      perror("ERROR: Not allowed.");
      exit(1);
      }
   pthread_t serie; //calls tthread openning the port
   pthread_create(&serie, NULL, &tthread_serie, NULL);
   pthread_join(serie, &v.fd);
   pthread_t config; //calls tthread configuring the port
   pthread_create(&config, NULL, &tthread_config, NULL);
   pthread_join(config, NULL);
   while(1){
      pthread_t leer; //calls tthread reading the port
      pthread_create(&leer, NULL, &tthread_leer, &v);
      pthread_join(leer, &x);
      pthread_t celsius; //calls tthread converting the data
      pthread_create(&celsius, NULL, &tthread_temp, &x);
      pthread_join(celsius, &res);
      printf("\r x = %d \n",x);
      if(res>20){ //if over 20 C, turn on compressor (power switching)
         outb(p,parallel); // port output (ON)
         }
      if(res<10){ //if below 10 C, shutdown compressor (power switching
         outb(0,parallel); // puerto output (OFF)         
         }

      printf("\r \n");
      }
   }
