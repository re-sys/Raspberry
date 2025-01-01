#include <stdio.h>
#include <wiringPi.h>
#define LED0 24
#define LED1 25
int main(){
	wiringPiSetup();
	pinMode(LED0, OUTPUT);
	pinMode(LED1, OUTPUT);
	printf("hello world");
	while(1){
		printf("Set green : light\n");
		digitalWrite(LED0, HIGH);
		digitalWrite(LED1, LOW);
		delay(1000);
 
		printf("Set red : light\n");
		digitalWrite(LED1, HIGH);
		digitalWrite(LED0, LOW);
		delay(1000);
	}
}
