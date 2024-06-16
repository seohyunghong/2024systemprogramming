#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <wiringPi.h>
#include <wiringPiI2C.h>
#include <fcntl.h>
#include <unistd.h>
#include <sys/ioctl.h>
#include <linux/i2c-dev.h>
#include <i2c/smbus.h>
#include "QR_test.hpp"
#include <pthread.h>
#include <sys/socket.h>
#include "server.h"
#include <stdbool.h>
#include <limits.h>
#include <arpa/inet.h>
#include <semaphore.h>
#include <netinet/in.h>
#include <time.h>



#define PORT 8373
#define IP "192.168.0.5"

#define SIZE 5
#define ITEM_INTERVAL 20
#define BOMB_INTERVAL 120


// Motor pins
#define MOTOR1 1
#define MOTOR2 0


// Tracing sensors
#define Left1 27
#define Left2 22
#define Right1 17
#define Right2 4


// LED
#define LED1 21
#define LED2 20


u_int8_t device_addr = 0x16;


int file;
const char *filename = "/dev/i2c-1";

int bombcnt = 4;
typedef struct {
    int x, y;
    int score;
} Node1;

typedef struct {
    Node1* nodes[SIZE * SIZE];
    int count;
} PriorityQueue;

typedef struct {
    int x, y; // Parent node coordinates
} ParentNode;

// 초기화 함수
void initializeNodes(Node1 nodes[SIZE][SIZE]) {
    for (int i = 0; i < SIZE; i++) {
        for (int j = 0; j < SIZE; j++) {
            nodes[i][j].x = i;
            nodes[i][j].y = j;
            nodes[i][j].score = 0;
        }
    }
}

// 우선순위 큐 관련 함수들
void pqPush(PriorityQueue* pq, Node1* node) {
    pq->nodes[pq->count++] = node;
}

Node1* pqPop(PriorityQueue* pq) {
    if (pq->count == 0) return NULL;
    int bestIndex = 0;
    for (int i = 1; i < pq->count; i++) {
        if (pq->nodes[i]->score > pq->nodes[bestIndex]->score) {
            bestIndex = i;
        }
    }
    Node1* bestNode = pq->nodes[bestIndex];
    pq->nodes[bestIndex] = pq->nodes[--pq->count];
    return bestNode;
}

bool isValid(int x,int y) {
    return x >= 0 && x < SIZE && y >= 0 && y < SIZE;
}

void updateItems(Node1 nodes[SIZE][SIZE], int row, int col, int score) {
    nodes[row][col].score = score;
    //printf("Item added at (%d, %d) with score %d\n", row, col, nodes[row][col].score);
}

void updateBombs(Node1 nodes[SIZE][SIZE], int row, int col) {
    nodes[row][col].score = -8;
    printf("Bomb added at (%d, %d)\n", row, col);
}

void printGrid(Node1 nodes[SIZE][SIZE]) {
    for (int i = 0; i < SIZE; i++) {
        for (int j = 0; j < SIZE; j++) {
            printf("%2d ", nodes[i][j].score);
        }
        printf("\n");
    }
    printf("\n");
}

int prevX = 0;
int prevY = 0;

void printPath(ParentNode parent[SIZE][SIZE], int startX, int startY, int endX, int endY) {
    int currentX = endX, currentY = endY;
    printf("Path: (%d, %d) ", endX, endY);
    while (currentX != startX || currentY != startY) {
        printf(" <- (%d, %d) ", parent[currentX][currentY].x, parent[currentX][currentY].y);
        int tempX = parent[currentX][currentY].x;
        int tempY = parent[currentX][currentY].y;
        prevX = currentX;
        prevY = currentY;
        currentX = tempX;
        currentY = tempY;
    }

    printf("\n");
}

void dijkstra(Node1 nodes[SIZE][SIZE], int startX, int startY, int endX, int endY) {
    int dist[SIZE][SIZE];
    int sptSet[SIZE][SIZE];
    ParentNode parent[SIZE][SIZE];

    for (int i = 0; i < SIZE; i++) {
        for (int j = 0; j < SIZE; j++) {
            dist[i][j] = INT_MIN;
            sptSet[i][j] = false;
            parent[i][j].x = -1;
            parent[i][j].y = -1;
        }
    }

    PriorityQueue pq = { .count = 0 };
    dist[startX][startY] = 0;
    pqPush(&pq, &nodes[startX][startY]);

    int directions[4][2] = { {0, 1}, {1, 0},{0,-1},{-1,0} }; // Only move in row or column direction

    while (pq.count > 0) {
        Node1* u = pqPop(&pq);
        int ux = u->x, uy = u->y;
        sptSet[ux][uy] = true;

        for (int i = 0; i < 4; i++) {
            int vx = ux + directions[i][0];
            int vy = uy + directions[i][1];

            if (isValid(vx, vy) && !sptSet[vx][vy]) {
                int newDist = dist[ux][uy] + nodes[vx][vy].score;
                if (newDist > dist[vx][vy]) {
                    dist[vx][vy] = newDist;
                    parent[vx][vy].x = ux;
                    parent[vx][vy].y = uy;
                    pqPush(&pq, &nodes[vx][vy]);
                    //printf("(%d,%d) Moved to (%d, %d) with accumulated score %d\n", ux, uy, vx, vy, newDist);
                }
            }
        }
    }

    // Print the path
    printPath(parent, startX, startY, endX, endY);

    // 결과 출력
    printf("Max Score Path from (%d, %d) to (%d, %d): %d\n", startX, startY, endX, endY, dist[endX][endY]);
}

void write_i2c_block_data(int file, u_int8_t device_addr, u_int8_t reg, u_int8_t* data, size_t length) {
    u_int8_t buffer[length + 1];
    buffer[0] = reg;

    for (size_t i = 1; i <= length; i++) {
        buffer[i] = data[i - 1];
    }

    if (write(file, buffer, length + 1) != length + 1) {
        perror("Failed to write to the i2c bus");
    }
}


int motor_run(int state1, int speed1, int state2, int speed2) {
   u_int8_t reg = 0x1;
   u_int8_t data[4] = {state1, speed1, state2, speed2};

   write_i2c_block_data(file, device_addr, reg, data, sizeof(data));
   return 0;
}


int motor_stop() {
   u_int8_t reg = 0x2;
   u_int8_t data[1] = {0};

   write_i2c_block_data(file, device_addr, reg, data, sizeof(data));
   return 0;
}

void turn_left_90() {
        motor_run(0,0,1,160);
        delay(700);
        motor_stop();
}
void turn_right_90() {
        motor_run(1,160,0,0);
        delay(700);
        motor_stop();
}

void setup() {
    // Initialize wiringPi
    wiringPiSetupGpio();

    // Set sensor pins as input
    pinMode(Left1, INPUT);
    pinMode(Left2, INPUT);
    pinMode(Right1, INPUT);
    pinMode(Right2, INPUT);


    // Open I2C bus
    if ((file = open(filename, O_RDWR)) < 0) {
        perror("Failed to open the i2c bus");
        exit(1);
    }
    if (ioctl(file, I2C_SLAVE, device_addr) < 0) {
        perror("Failed to acquire bus access and/or talk to slave");
        exit(1);
    }
}

int result = 0;

int startX = 0;
int startY = 0;
int ppX = 0;
int ppY = -1;

int val2XY() {
    ppX = startX;
    ppY = startY;
    startX = result/10;
    startY = result%10;
    return 0;
}
DGIST dgist;
int updateMap(int row, int col) {
    if ((dgist.map[row][col]).item.status == nothing){
        return 0;
    }
    else if ((dgist.map[row][col]).item.status == item) {
        return dgist.map[row][col].item.score;
    }
    else if ((dgist.map[row][col]).item.status == trap) {
        return -1;
    }
}
int coor4[2] = {3,3};
int bombing = 0;
void* calc(void* unused) {
    while(result != 0) {
    }
    while(1) {
        
        srand(time(NULL));
        Node1 nodes[SIZE][SIZE];
        initializeNodes(nodes);

        int seconds = 0;

        for (int row = 0; row < 5; row++) {
            for (int col = 0; col < 5; col++) {
                int h = updateMap(row, col);
                if (h == 0) {
                }
                else if (h == -1) {
                    updateBombs(nodes, row, col);
                }
                else {
                    updateItems(nodes, row, col, h);
                    if (h == 4) {
                        coor4[0] = row;
                        coor4[1] = col;
                    }
                }
            }
        }

        // 초기 아이템 및 폭탄 배치 후 그리드 출력
        printGrid(nodes);
        val2XY();

        // 사용자로부터 출발지와 목적지 입력받기




        dijkstra(nodes, startX, startY, coor4[0], coor4[1]);

        printf("Current Position: (%d, %d)\n", startX, startY);
        printf("Next Position: (%d, %d)\n", prevX, prevY);
        if ((prevX == coor4[0]) && (prevY == coor4[1])){
            bombing = 1;
        }


        // 최적 경로 찾기
        delay(100);
    }
}


int algorithm() {
	int prev10 = ppX;

	int next10 = prevX;
	int present10 = startX;

	int prev1 = ppY;

	int next1 = prevY;
	int present1 = startY;
	printf("prev10: %d, prev1: %d, next10: %d, next1: %d, present10: %d, present1: %d\n", prev10, prev1, next10, next1, present10, present1);


	int towardUD, towardRL;

	if ((prev10 - present10) >= 1) { towardUD = 1; }
	else if ((prev10 - present10) <= -1) { towardUD = -1; }
	else towardUD = 0;
	if ((prev1 - present1) >= 1) { towardRL = 1; }
	else if ((prev1 - present1) <= -1) { towardRL = -1; }
	else towardRL = 0;

	if ((next10 - present10) >= 1) { // go up
		printf("go up\n");
		if (towardRL > 0) {turn_left_90();} // turn left
		else if (towardRL < 0) {turn_right_90();} // turn right
		else if (towardUD > 0) {
        	motor_run(1,50, 1, 50);
        	delay(500);// go straight
    	}
	}

	else if ((next10 - present10) <= -1) { // go downa
		printf("go down\n");
		if (towardRL > 0) {turn_right_90();} // turn right
		else if (towardRL < 0){turn_left_90();} // turn left
		else if (towardUD < 0){
        	motor_run(1,50, 1, 50);
        	delay(500);// go straight
    	} // go straight
	}
	else if ((next1 - present1) >= 1) { // go right
		printf("go right\n");
		if (towardRL > 0) {
           	motor_run(1,50, 1, 50);
           	delay(500);// go straight// go straight
       	}
		else if (towardUD > 0){turn_right_90();} // turn right
		else if (towardUD < 0) {turn_left_90();}// turn left
    }
	else if ((next1 - present1) <= -1) { // go left
		printf("");
		if (towardUD < 0){turn_right_90();} // turn right
		else if (towardRL < 0){
           	motor_run(1,50, 1, 50);
       		delay(500);// go straight
       	} // go straight
		else if (towardUD > 0){turn_left_90();} // turn left
        }
	else {
	}
    return 0;
}





void *loop(void* unused) {
    while (1) {
        //int QR_return = read_QRCode(NULL);
        //printf("QR return : %d \n", QR_return);
        // Read sensor values
        int left1 = digitalRead(Left1);
        int left2 = digitalRead(Left2);
        int right1 = digitalRead(Right1);
        int right2 = digitalRead(Right2);

        //printf("left0 : %d | left2 : %d | right1 : %d | right2 : %d \n", left1, left2, right1, right2);

        if (left1 == HIGH && left2 == LOW && right1 == LOW && right2 == HIGH ) {motor_run(1,60, 1, 60);}
        else if ( left1 == HIGH && left2 == HIGH && right1 == HIGH && right2 == HIGH) {motor_run(1, 40, 1, 40);}
        //else if ( left1 == LOW && left2 == LOW && right1 == LOW && right2 == HIGH) {algorithm();printf("left\n");}
        else if ( left1 == LOW && left2 == LOW && right1 == LOW && right2 == HIGH) {turn_left_90();}
        //else if ( left1 == HIGH && left2 == LOW && right1 == LOW && right2 == LOW) {algorithm();printf("right\n");}
        else if ( left1 == HIGH && left2 == LOW && right1 == LOW && right2 == LOW) {turn_right_90();}
        else if ( left1 == LOW && left2 == LOW && right1 == LOW && right2 == LOW) {
            int randi = rand() %2;
            printf("rand: %d", randi);
            if ( randi == 0) {
                turn_right_90();
            }
            else {
                turn_left_90();
            }
        }

        else if (left1 == LOW && left2 == LOW && right1 == HIGH && right2 == HIGH ) {motor_run(0, 0, 1, 70);}
        else if (left1 == LOW  && left2 == HIGH && right1 == HIGH && right2 == HIGH ) {motor_run(0, 0, 1, 80);}
        else if (left1 == HIGH && left2 == LOW && right1 == HIGH && right2 ==HIGH ) {motor_run(1, 30, 1, 50);}

        else if (left1 == HIGH  && left2 == HIGH && right1 == LOW && right2 == LOW ) {motor_run(1, 70, 0, 0);}
        else if (left1 == HIGH  && left2 == HIGH && right1 == HIGH && right2 == LOW ) {motor_run(1, 80, 0, 0);}
        else if (left1 == HIGH && left2 == HIGH && right1 == LOW  && right2 == HIGH ) {motor_run(1, 50, 1, 30);}
        else {motor_stop();}
        delay(20);
    }
}





int sock = 0, valread;
struct sockaddr_in serv_addr;
ClientAction cAction;

void* client_send(void* arg) {
    while(1) {
        cAction.row = result/10;
        cAction.col = result%10;
        printf("Enter action (0 for move, 1 for set bomb): ");
        if ((bombing == 1) && (bombcnt > 0)) {
            cAction.action = setBomb;    
            bombcnt -= 1;
            bombing = 0;       
        }
        else {
            cAction.action = move;
        }
        send(sock, &cAction, sizeof(ClientAction), 0);
        printf("Client action sent\n");
        delay(1000);
    }
}
void* client_recieve(void* arg){
    while(1) {    
        valread = recv(sock, &dgist, sizeof(DGIST), 0);
        if (valread == 0) {
            printf("Server disconnected\n");
            break;
        } else if (valread < 0) {
            printf("Error in receiving data from server\n");
            break;
        } 

        else {
            // Successfully received DGIST structure
            // Process the DGIST structure (Example: Print the map and player info)
            printf("Received DGIST structure from server\n");

            // Print players' info
            for (int i = 0; i < MAX_CLIENTS; i++) {
                printf("Player %d - Row: %d, Col: %d, Score: %d, Bombs: %d\n", 
                        i, dgist.players[i].row, dgist.players[i].col, dgist.players[i].score, dgist.players[i].bomb);
            }

        }
        
    }
}

void* client_main() {
    pthread_t thread11, thread22;

    if ((sock = socket(AF_INET, SOCK_STREAM, 0)) < 0) {
        printf("\n Socket creation error \n");
        return NULL;
    }

    serv_addr.sin_family = AF_INET;
    serv_addr.sin_port = htons(PORT);

    if (inet_pton(AF_INET, IP, &serv_addr.sin_addr) <= 0) {
        printf("\nInvalid address/ Address not supported \n");
        return NULL;
    }

    if (connect(sock, (struct sockaddr *)&serv_addr, sizeof(serv_addr)) < 0) {
        printf("\nConnection Failed \n");
        return NULL;
    }

    if (pthread_create(&thread11, NULL, client_send, NULL)) {
        fprintf(stderr, "Error creating thread 11\n");
        return NULL;
    }
    if (pthread_create(&thread22, NULL, client_recieve, NULL)) {
        fprintf(stderr, "Error creating thread 22\n");
        return NULL;
    }
    pthread_join(thread11, NULL);
    pthread_join(thread22, NULL);
        
    close(sock);
    return NULL;
}


int main() {
    pthread_t thread1, thread2, thread4;
    setup();
    if (pthread_create(&thread1, NULL, read_QRCode, NULL)) {
        fprintf(stderr, "Error creating thread 1\n");
        return 1;
    }
    if (pthread_create(&thread2, NULL, loop, NULL)) {
        fprintf(stderr, "Error creating thread 2\n");
        return 1;
    }
    if (pthread_create(&thread4, NULL, calc, NULL)) {
        fprintf(stderr, "Error creating thread 4\n");
        return 1;
    }
    client_main();
    pthread_join(thread1, NULL);
    pthread_join(thread2, NULL);
    pthread_join(thread4, NULL);

    close(file);
    return 0;
}

