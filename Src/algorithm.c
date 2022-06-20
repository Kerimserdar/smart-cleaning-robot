#include <stdio.h>
#include <stdlib.h>
#include <stdbool.h>  
#include <time.h>
#include "functions.h"
#include "main.h"
#include "lcd_txt.h"
//#include <Windows.h> // not window icin Windows.h yapmalısın (unistd.h for unix)

struct Point* createPosList(struct Point currentLocation)
{
	struct Point *posList = malloc(sizeof(struct Point)*4); //there are 4 possibilities for one point
	int x = currentLocation.x;
	int y = currentLocation.y;
	
	posList[0].x = x+1; posList[0].y = y;
	
	for (int i=0; i<4; i++) // basta tüm degerleri true yapiyoruz sonradan elendikçe false olacak
		posList[i].active=true;


	if (x != 0)	{
		posList[1].x = x-1; posList[1].y = y; // x degeri 0 dan kücük olmamali buraa x != 0 dan x in sifirdan kücük
						      // oldugu degeri anlamaliyiz cünkü cihazi odanin köselerinden birakiyoruz
	}

	else{
		posList[1].active = false; // if durumu çalismaz ise 1 array elemani defaul olarak 0,0 oldugu icin buna ihtiyac var
		posList[1].x = -777; posList[1].y= -777;
		posList[1].mark = 'o'; // yeni duvar var isede bu bir obstacle olmus oluyor
	}

	posList[2].x = x; posList[2].y = y+1;

	if (y != 0) 
	{
		posList[3].x = x; posList[3].y = y-1; // y degeri 0 dan kücük olmamali
	}
	else{
		posList[3].active = false;
		posList[3].mark = 'o';
		posList[3].x = -999; posList[3].y= -999;

	}

	return posList;
}

struct Point randomSelection(struct Point *posList){
	int count = 0;// counts the number of active point in possList
	//int *randomArray = malloc(sizeof(int)*4);

	for (int i=0; i < 4; i++)
	{
		if (posList[i].active == true)
			count++;
	}
	
	//printf("random selection methodu count degeri: %d\n", count);
	
	int *randomArray2 = malloc(sizeof(int)*count);
	//int count2 = count;	
	int count2 = 0;
	
	for (int i=0; i < 4; i++)
	{
		if (posList[i].active == true)
		{
			randomArray2[count2] = i; // posList in index numarasini kayit ediyorum
			count2++;
		}
	}

	for (int i= 0; i<count; i++)
	{
		//printf("index: %d and element %d \n", i, randomArray2[i]);
	}

	if (count != 0)
	{ // in this condition all around of current point surrounded which means it can not move anywhere
	 	//srand(time(0));
		srand(55);
		int rand_num = rand() %count;
		//printf("random_num: %d\n ", rand_num);
		return posList[randomArray2[rand_num]];
	}
	else 
	{
		struct Point temp ;
		temp.x = -500; // eger temp yapisinin x degeri -500 gibi bir deger ise (bunun posList te olusmasi imkansi) bu random islemmini yapil
			       // madigi anlamina geliyor
		return temp;
	}
}

void initArray(Array *a, size_t initialSize) {
  a->array = malloc(initialSize * sizeof(struct Point));
  a->used = 0;
  a->size = initialSize;
}

void insertArray(Array *a, struct Point point) {
  // a->used is the number of used entries, because a->array[a->used++] updates a->used only *after* the array has been accessed.
  // Therefore a->used can go up to a->size 
  if (a->used == a->size) {
    a->size *= 2;
    a->array = realloc(a->array, a->size * sizeof(struct Point));
  }
  a->array[a->used++] = point;
}

void freeArray(Array *a) {
  free(a->array);
  a->array = NULL;
  a->used = a->size = 0;
}

/*int printPointList(struct Point *list, int size)
{
	for (int i = 0; i < size; i++)
	{
		printf("list[%d] x,y: %d,%d\n", i, list[i].x, list[i].y);		
	}
	return 0;
}*/

void checkObstacleList(struct Point *posList, Array obstacle)
{
	// bu iç içe döngü sayesinde possibility listesiden obstacle listesindeki noktarı silmiş olacagız 
	// bu döngüyü buradan kaldırım  functions.h dosyasında bir adet fonksiyon olusturacagım
	for (int i = 0; i < 4; i++) // 4 posbList in boyutu
	{
		if (posList[i].active==true) // posList teki koordinatların aktif yani önceden listeler tarafında elenmemiş olması gerekiyor
		{
			for (int j=0; j < obstacle.used; j++)
			{
				//printf("(%d,%d) is element of posList\n", posList[i].x, posList[i].y);
				//printf("(%d,%d) is element of traceList\n", obstacle.array[j].x, obstacle.array[j].y);
				if ((posList[i].x == obstacle.array[j].x)&& (posList[i].y == obstacle.array[j].y))
				{
				
					posList[i].mark = 'o';
					posList[i].active = false;
					//printf("aktiflik durumu: %d\n", posList[i].active);
					//printf("(%d,%d) is deleted from posssibility list due to OBSTACLE LIST\n", posList[i].x, posList[i].y);
				}
			}
		}
	}
	//printf("%d :", list[2].x);
}


void checkTraceList(struct Point *posList, Array trace)
{
	// bu iç içe döngü sayesinde possibility listesiden trace listesindeki noktarı silmiş olacagız 
	// bu döngüyü buradan kaldırım  functions.h dosyasında bir adet fonksiyon olusturacagım
	for (int i = 0; i < 4; i++) // 4 posbList in boyutu
	{
		if (posList[i].active==true) // posList teki koordinatların aktif yani önceden listeler tarafında elenmemiş olması gerekiyor
		{
			for (int j=0; j < trace.used; j++)
			{
				//printf("(%d,%d) is element of posList\n", posList[i].x, posList[i].y);
				//printf("(%d,%d) is element of traceList\n", trace.array[j].x, trace.array[j].y);
				if ((posList[i].x == trace.array[j].x)&& (posList[i].y == trace.array[j].y))
				{
					posList[i].active = false;
					posList[i].mark = 't';
					//printf("aktiflik durumu: %d\n", posList[i].active);
					//printf("(%d,%d) is deleted from posssibility list due to TRACE LIST\n", posList[i].x, posList[i].y);
				}
			}
		}
	}
	//printf("%d :", list[2].x);
}

int checkDeviceSurrounded(struct Point *posList)
{
	int count = 0; // this calculate the number of marked field as "o" 
	// bu method sayesinde cihazın engeller ile cevrili olup olmadıgına bakacagız
	for (int i = 0; i < 4; i++) // 4 posbList in boyutu
	{
		if (posList[i].mark == 'o') // bu durum posListteki duvar ile ilgili olan kısımları iceriyor  
		{
			count++;
		}
	}
	//printf("checkDevice methodu count sayısı : %d\n", count);
	return count;	
}

int getInfoDistanceSensor(int size)
{
	if (size < 50)
		return 0;
	else 
		return 1;
}

/*void movingForward()
{
	printf("moving forward\n");
}

void turnRight(int number)
{
	printf("%d times turn right\n",number);
}

void turnLeft(int number)
{
	printf("%d times turn left\n", number);
}*/

int detectDirection(struct Point selectedPoint, struct Point currentLocation, int direction)
{
	int tempDirection;
	if(selectedPoint.x == currentLocation.x)
	{
		if(selectedPoint.y-currentLocation.y > 0)
			tempDirection = 1;
		else 
			tempDirection = 3;
	}
	else
	{
		if(selectedPoint.x-currentLocation.x > 0)
			tempDirection = 2;
		else 
			tempDirection = 4;
	}

	//printf("temp direction degieri : %d\n", tempDirection);

	int result=(tempDirection%4)-(direction%4);

	if (result == 0) // && getInfoDistanceSensor(30) == 1)// burayı çıkarman gerek
	{
		tempDirection = direction ; // eger result 0 ise herhangi bir yöne dönmeye lüzum yok 
		//movingForward(); // burada direction güncllenebilir getinfoSensor a burada luzum yok gibi
		//printf("herhangi bir yöne dönmüyoruz\n");
	}
	else if (result < 0)
	{
		if (result == -1)
			TurnLeft();

		else if (result == -2){

			TurnLeft();
		}
		else 
			TurnRight();
	}

	else
	{
		if (result == 1)
			TurnRight();

		else if (result == 2){
			TurnRight();
		}
		else 
			TurnLeft();
	}

	direction = tempDirection; // direction is updated
	
	return direction;
}

//
//	burada artık trace silme işlemi yapacagız else icinde 	
//
//	algoritma tamamlandı test edildi bir sorun görünmüyor
//	ekran kaydı icin :
//		https://drive.google.com/drive/folders/15HiI0xDkxcNrlEpfiRDk-Gs0XuOUhxv5?usp=sharing
//

static Array trace;
static Array obstacle;
static struct Point *posList;
static struct Point currentLocation;
int direction = 1;

void initialize(void){
	initArray(&obstacle,5);
	initArray(&trace,5);

			// 1 is up 2 is right 3;below 4; left
 
	currentLocation.x = 0;
	currentLocation.y= 0;
	
	posList = malloc(sizeof(struct Point)*4); 

	//printf("MAIN current location (%d,%d)\n", currentLocation.x, currentLocation.y);	
	//printf("MAİN possible list elements: \n");	
	posList = createPosList(currentLocation);
	//printPointList(posList, 4);
	
	
	//recursive(trace, obstacle,posList,&direction, currentLocation);	
}

void recursive(int val) // burada currentPoint te olmalı
{
	int distance = val; // this comes from distance sensor
	// direction ile ilgili degiskenlerin burada kalmasına gerek yok

	checkObstacleList(posList,obstacle);// checkobstacle de burada durmalı o da güncelleniiyor. Obstacle listesi daha öncelikli cünkü cihazın etrafı sarılı ise trace i bosaltmanın bir mantıgı yok
	checkTraceList(posList, trace);// trace güncellenecegi icin burada kalması daha iyi
	struct Point selectedPoint = randomSelection(posList);
	//printf("(%d,%d) degeleri:(random selection) ", selectedPoint.x, selectedPoint.y);
	
	if (selectedPoint.x != -500)
	{// selectedPoint in x degeri -500 degil ise posList te hala random secebiliegimiz eleman var demektir
	 
		direction = detectDirection(selectedPoint, currentLocation, direction);// önemli direction u kontrol et
		//printf("yeni direction : %d\n", *direction);
		
		//printf("*****enter the distance value :");
		//scanf("%d", &distance);
		if (getInfoDistanceSensor(distance) == 1) // parametre olarak 30 gönderdim rastgele seçtim--> 50 nin altı otamatik olarak obstacle demek
		{// is there obstace in direction

			MoveForward();	
			insertArray(&trace, currentLocation); //trace liste en sonki noktayı ekliyoruz
			currentLocation = selectedPoint;

			//printf("FONKSİYON UPDATE current location (%d,%d)\n", currentLocation.x, currentLocation.y);	
			
			//printf("FONKSİYON -UPDATE trace array elemanları\n");
			//printPointList(trace.array,trace.used);
			
			//printf("pFONKSİYON -UPDATE possible list elements: \n");	
			posList = createPosList(currentLocation); // burada ileri yöne hareket ettiğimz icin yeni point icin ihtiamller listesi olusturuyoruz
			//printPointList(posList, 4);

			//sleep(2);			
			//recursive(trace,obstacle,posList,direction, currentLocation, val);// burada obstacle sabit digerleri degisiyor
			// burada tekrar recursive fonksiyon cagrılmalı
		}
		else {
			insertArray(&obstacle, selectedPoint);// önünde engel çıkan point i(random seçmiştik) obstacle listesine ekliyoruz
			// ONEMLI insert ettikten sonra selectedPoint in "active" degerini false yapmalısın ki posList den deger düssün
			// ONEMLI2 GEREK YOK CÜNKÜ checkObstacleList methodu zaten bunu yapacak
			//
			//printf("(%d,%d) point added into OBSTACLE LIST\n",selectedPoint.x, selectedPoint.y);
			//printf("OBSTACLE array elemanları\n");
			//printPointList(obstacle.array,obstacle.used);
			//sleep(2);			
			//recursive(trace,obstacle,posList,direction,currentLocation, val); // bu parametrelerden sadece obstacle list degisiyor
			// burada ise tekrar random atmalsın
			// ayrıca engeller listesine bir point dege eklemelisin
		}
	}
	else {
		// chech whether current point surrended by obstacle list if so stop the mahine no need for flushing trace list
		// burada tüm obstacle listesini dolan eger bir x ve y 0 dan büyük ise bu işlemi yapmalısın cünkü duvar durumuda var duvar 
		// yanında isede cihaz sıkışmış olur yani trace listi silmenin bir mantıg yok
		//
		//
		//trace listesini bosaltırken posList in marked field in bosalt ayrıca active durumunuda bostalmalısını
		//
		if (checkDeviceSurrounded(posList) == 4){
			//printf("Device surrounded by obstacle can not move anymore\n");
			HAL_GPIO_WritePin(led2_GPIO_Port, led2_Pin, GPIO_PIN_RESET);
	    HAL_GPIO_WritePin(led3_GPIO_Port, led3_Pin, GPIO_PIN_RESET);
			HAL_GPIO_WritePin(led1_GPIO_Port, led1_Pin, GPIO_PIN_SET);
			Stop();
		}
		
		else{ // bu durumda trace listesini boşaltıyoruz tekrar baslıyoruz
		      //
		      	//printf("\n\n TRACE lıst and POSLIST deleted\n\n");
			freeArray(&trace); // trace array inin siliyoruz
			free(posList);
			
			//Array trace; // tekrar boş olarak trace listesi olustur
			initArray(&trace,5);
			
			posList = malloc(sizeof(struct Point)*4); 	

			//printf("ELSE possible list elements: \n");	
			posList = createPosList(currentLocation);
			//printPointList(posList, 4);
			//recursive(trace,obstacle,posList,direction,currentLocation, val); 
		}	
	}
}
