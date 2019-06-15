#include <stdio.h>
#include <string.h>
#include <unistd.h>
#include <fcntl.h>
#include <errno.h>
#include <termios.h>
#include <stdlib.h>
#include <sys/io.h>

#define paralelo 0x378 //pto paralelo

struct variables{
      int fd; 
      char *datos;
      int tamanio; 
      int tiempo;
      };
struct variables v;
struct timespec next;
float t=0;
v.tamanio=1; 
v.tiempo=500000;

int *hilo_serie(void *args){
   clock_gettime(CLOCK_REALTIME, &next);
   next.tv_sec = next.tv_sec + 0.05; // otros 5ms
   next.tv_nsec = 0;
   v.fd = open("/dev/ttyS0", O_RDWR | O_NOCTTY | O_NDELAY);
   if (v.fd == -1)
      { 
      perror("\n ERROR: No es posible abrir el puerto. /dev/ttyS0 - \n");
      }
      else
	fcntl(v.fd, F_SETFL, 0);
      printf("\r   Puerto abierto... id: %d \n",v.fd);
      return (v.fd);
    }

void *hilo_config(void *args){
   struct timespec next;
   clock_gettime(CLOCK_REALTIME, &next);
   next.tv_sec = next.tv_sec + 0.05; // 5 miliseg
   next.tv_nsec = 0;
   clock_nanosleep(CLOCK_REALTIME, TIMER_ABSTIME, &next, NULL);
   struct termios options;
   tcgetattr(v.fd, &options);
   cfsetispeed(&options, B300);
   cfsetospeed(&options, B300);
   options.c_cflag |= (CLOCAL | CREAD);
   options.c_cflag &= ~CSIZE; 
   options.c_cflag |= CS8;    
   options.c_cflag &= ~PARENB;
   options.c_cflag &= ~CSTOPB;
   options.c_cflag &= ~CSIZE;
   options.c_cflag |= CS8;
   options.c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG);
   tcsetattr(v.fd, TCSANOW, &options);
   printf("\r   Puerto configurado exitosamente\n");
   }

int *hilo_leer(int serial_fd, char *data, int size, int timeout_usec){
   struct timespec next;
   clock_gettime(CLOCK_REALTIME, &next);
   next.tv_sec = next.tv_sec + 0.1; // 100 miliseg
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
      ret=select (FD_SETSIZE,&fds, NULL, NULL,&timeout);
      if (ret==1) {
         printf("\r Comunicacion establecida \n");
         n=read (serial_fd, &data, v.tamanio); 
         printf("   n = %d\n", n);
         printf("data = %p\n ", data);
         count+=1;
         data=0;
         }
      }while (count<size && ret==1);
   return n;
   }

float *hilo_temp(int adc){
   struct timespec next;
   clock_gettime(CLOCK_REALTIME, &next);
   next.tv_sec = next.tv_sec + 0.01; // 10 miliseg
   next.tv_nsec = 0;
   clock_nanosleep(CLOCK_REALTIME, TIMER_ABSTIME, &next, NULL);
   printf("\r adc = \n",adc);
   float volt=0;
   volt=adc/255; 
   t=volt*4.8*100;
   }

main(){
   int x; //valor leido del adc
   unsigned int p = 0x01; //indica 1 salida
   float res; //valor temp ºC
   printf("\r \n");
   if(ioperm(paralelo,1,1)){ //dar permisos pto paralelo
      perror("ERROR: No tiene los permisos necesarios.");
      exit(1);
      }
   pthread_t serie; //llama al hilo que abre el puerto
   pthread_create(&serie, NULL, &hilo_serie, NULL);
   pthread_join(serie, &v.fd);
   pthread_t config; //llama al hilo que config el puerto
   pthread_create(&config, NULL, &hilo_config, NULL);
   pthread_join(config, NULL);
   while(1){
      pthread_t leer; //llama al hilo que leer el puerto
      pthread_create(&leer, NULL, &hilo_leer, &v);
      pthread_join(leer, &x);
      pthread_t celsius; //llama al hilo que convierte el dato
      pthread_create(&celsius, NULL, &hilo_temp, &x);
      pthread_join(celsius, &res);
      printf("\r x = %d \n",x);
      if(res>20){ //mayor a 20 C, prender el compresor
         outb(p,paralelo); // salida del puerto (ON)
         }
      if(res<10){ //menor a 10 C, apagar compresor
         outb(0,paralelo); // salida del puerto (OFF)         
         }

      printf("\r \n");
      }
   }
