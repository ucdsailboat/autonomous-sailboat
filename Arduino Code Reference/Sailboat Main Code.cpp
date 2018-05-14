// The code for the main sail boat, GPS, compass, wind vane, and sd card (shield ethernet)
// corresponding to the angle of the rudder sent to the driver of the rudder 
// this code is for Arduino Mega

#include <StandardCplusplus.h>
#include <serstream>
#include <string>
#include <vector>
#include <iterator>
#include <inttypes.h>
#include <Servo.h>
#include <math.h>
#include <SoftwareSerial.h>
#include <TinyGPS.h>
#include <Math.h>
#include <Wire.h>
#include <stdio.h>
#include <stdlib.h>
#include <SPI.h>
#include <SD.h>

File dataFile;

#define PI 3.14159265

using namespace std;

//Compass: VCC -> 3,3; SDA -> A4; SCL -> A5;;
//GPS: YELLOW -> GND; ORANGE -> 5V; WITH -> PORTA 2;
//NORTH OF WIND INDICATOR = FRONT OF BOAT

struct location {
	float latitude;
	float longitude;
};

TinyGPS gps;
int cont_debug = 0;
float start, tempo;

const int buttonPin = 2;
float starttime, endtime;
int buttonState = 0;         // variable for reading the pushbutton status
int buttonState_ant = 0;         // variable for reading the pushbutton status
int cont = 0;
float velocidade_vento;

int recByte;

//variables of the compass
int HMC6352Address = 0x42;
int slaveAddress;
byte headingData[2];
int i, headingValue;

float xCom = 0;
char inData[11]; // Allocate some space for the string
char inChar = -1;
byte index = 0;
int contadorCom = 1; // counter
int iCom = 0;
int dado; // data

//variables of the GPS

int contador_pontos = 0; // counter_points

float velocidade; // velocity
float direcao; // direction
float distanciaInicial, distanciaPercorrida, lastDistanciaDestino = 0; 
// initial distance, distance travelled, last distance of destination
float distanciaAoDestino = 99999999; // distance to destination

//variables of the PID
float erroAtual = 0;  
//difference btwn setpoint and the actual direction of boat. actualError
float erroAtualalt = 0;
float erroAnterior = 0; // previous error
float Ianterior = 0;    // acummulator of integrative error. PreviousI
float Kp = 2;           // proportional gain of PID
float Ki = 0.005;       // integrative gain of PID
float valor_PID;
float angulo_rudder, angulo_rudder_ant, angulo_rudder_sig;

//approximate time of code execution
float tempo_do_ciclo = 0.2; // cycle time

//Initial configurations 
float incremento_sail = 3;// sail increment 
float pos_sail = 60;
boolean aproximando = false; // approximate boolean
float teste;

//Variables at the start of the direction control of the boat
float heading = 0;  // goin thru the compass
//insert it at the time; align boat with desired direction.
float sp;       // going throuhg einstein's algorithm (PID rudder)
float biruta;      // going through wind vane
float norte_quadr_buss = 104.00; // you have to calibrate it
int birutaPin = A0; // wind vane in
boolean alcancouAlvo = false; // boolean hit the target = false

boolean isBordejando = false; // boolean  is following wind direction
int waypointId = 0, waypointBId = 0, controle = 0;
float angulo_bordejo = 60; // angle_boat following wind direction.
float taxa_dist = 0.2; //rate_distance

location startLocation, currentLocation, lastLocation, nextLocation, targetLocation;

vector<location> waypoints;
vector<location> pontos_bordejar; //points that the sailboat follows the wind direction

vector<float> dados; // data 
String content = "";
char character;

const int chipSelect = 4;

int contador_bug = 0; // counter 

// actual                                  destination
// atual is a pointer to location object 
// borderjar - following wind direction
vector<location> bordejar(struct location *atual, struct location *destino)
{

	// bordejo - noun version 
	// holds the points 
	vector<location> pontos_bordejo;

// biruta - wind direction from the wind vane (in dgs)
	float heeling = biruta + heading;
	heeling = saturador(heeling);
	
	// latitude of actual location
	float x0 = atual->latitude; //accessing member
	//longitude of actual location
	float y0 = atual->longitude;
	float x = destino->latitude;
	float y = destino->longitude;

//  distance_following wind direction = rate_dist (value = 0.2) * distanceInitial
	float distancia_bordejar = taxa_dist * distanciaInicial;

// calculates the path between the start and the destination point
	// a_A - slope 
	float a_A = (y - y0) / (x - x0); //destination - actual 
	// point-slope form & b_A = intersect
	float b_A = y0 - (a_A * x0);
	//float x_temp = x0;
	//float y_temp = a_A * x_temp + b_A;

	//calculation for a specific case 
	// heeling (heading and biruta) must be greater than heading
	if (heeling - heading < 0){
		// 60 = -60
		 angulo_bordejo = -angulo_bordejo;
	}

	//float thetaAB = abs(saturador(biruta)) - angulo_bordejo;
	// slope of the line is set to 60
	float thetaAB = angulo_bordejo;
	thetaAB = saturador(thetaAB);
	
	//encontra a reta B tal que thetaAB = 45º - biruta e a interseção seja o ponto inicial
	//find the line B such that thetaAB = 45 degress - wind vane and the intersection are at the initial point 
	// find the path that you are going 45 degrees (ideal case?)
	// changes degrees to radians 
	float tan_thetaAB = (float)tan((thetaAB) * (PI / 180));
	// A - first point
	// B - second point 
	// calculates the angle between the two points (A & B)
	float a_B = (a_A - tan_thetaAB) / (tan_thetaAB * a_A + 1);
    // 
	float b_B = y0 - (a_B * x0);

	//initialization of auxiliary variables 
	float d = distanciaInicial;
	float d_p0 = 9999999;

// x - destination latitude
// y - destination longitude 
	float latBord = x;
	float lonBord = y;

	float latProj = x;
	float lonProj = y;
	// ponto - score 
	location pontoProj;

	float latIni = x0;
	float lonIni = y0;
	float latFim = x;
	float lonFim = y;

	while (abs(d - distancia_bordejar) > 1) {
		//latAverage	 latEnd    latStart
		float latMedia = (latFim + latIni) / 2;
		//longAverage     longEnd    longStart
		float lonMedia = (lonFim + lonIni) / 2;
		
		// projection2d_mod
		pontoProj = projecao2d_mod(latMedia, lonMedia, a_A, b_A, a_B, b_B);
		
		d = (float)TinyGPS::distance_between(latMedia, lonMedia, pontoProj.latitude, pontoProj.longitude);

		if (d > distancia_bordejar){
			latFim = latMedia;
			lonFim = lonMedia;
		} else if (d < distancia_bordejar){
			latIni = latMedia;
			lonIni = lonMedia;
		}

		latBord = latMedia;
		lonBord = lonMedia;
	}

	location ponto_projetado = angleToLocation(latBord, lonBord);

	latProj = pontoProj.latitude;
	lonProj = pontoProj.longitude;

	location p0m = projecao2d(latProj, lonProj, a_A, b_A);
	d_p0 = (float)TinyGPS::distance_between(p0m.latitude, p0m.longitude, x0, y0);

	//insere o ponto encontrado no vetor com os pontos do bordejo
	// inserts the point found in the vector with the points of the edge 
	pontos_bordejo.push_back(angleToLocation(latProj, lonProj));

	//encontra as retas paralelas a reta A, onde estão localizados os pontos de bordejo
	// find the lines parallel to the line A, where the points of edge are located 
	float b_linha1 = (-a_A * pontos_bordejo.at(0).latitude + pontos_bordejo.at(0).longitude);
	float y_tira_teima1 = (a_A * pontos_bordejo.at(0).latitude + b_linha1);
	float x_tira_teima1 = (pontos_bordejo.at(0).longitude - b_linha1) / a_A;
	float b_linha2 = 0;

	if (b_linha1 > b_A)
	{
		b_linha2 = b_A - (b_linha1 - b_A);
	}
	else
	{
		b_linha2 = b_A + (b_A - b_linha1);
	}

	//encontrando o número total de pontos de bordejo
	// finding the total number of border points 
	int num_pontos = (int)floor((float)TinyGPS::distance_between(x0, y0, x, y) / d_p0);
	float teste = (float)TinyGPS::distance_between(x0, y0, x, y);

	float delta_x = (p0m.latitude - x0);
	float delta_y = (p0m.longitude - y0);

	float tt_x = x0 + delta_x;
	float tt_y = y0 + delta_y;

	//encontra a projeção dos pontos iniciais nas duas retas paralelas
	// finds the projection of the initial points in the two parallel lines 
	location p0l1 = projecao2d(p0m.latitude, p0m.longitude, a_A, b_linha1);
	location p0l2 = projecao2d(p0m.latitude, p0m.longitude, a_A, b_linha2);
	int bom = 1;
	int ruim = 1;

	vector<float> dados;
	String content = "";
	char character;

	for (int z = 1; z < num_pontos; z++)
	{

		float delta_x_temp = z * delta_x;
		float delta_y_temp = z * delta_y;

		if (z % 2 == 0)
		{
			location controle1 = pontos_bordejo.at(z - 1);
			float lat_temp = (float)p0l1.latitude + delta_x_temp;
			float lon_temp = (float)p0l1.longitude + delta_y_temp;
			location controle_loc = angleToLocation(lat_temp, lon_temp);
			pontos_bordejo.push_back(controle_loc);
			bom = z;
		}
		else {
							// vector.at - access an element 
			location controle1 = pontos_bordejo.at(z - 1);
			float aux = abs((heeling - abs(saturador(sp))));
			float delta_xaux = aux * delta_x / 31;
			float delta_yaux = aux * delta_y / 31;
			float lat_temp = p0l2.latitude + delta_x_temp - delta_xaux;
			float lon_temp = p0l2.longitude + delta_y_temp - delta_yaux;
			location controle_loc = angleToLocation(lat_temp, lon_temp);
			// push_back - add element at the end 
			pontos_bordejo.push_back(controle_loc);
			ruim = z;
		}
	}
	// size_points_following wind direction
	int tamanho_pontos_bordejo = pontos_bordejo.size();
									    //size_points_following the wind direction
	float d_p0_pu = (float)TinyGPS::distance_between(pontos_bordejo.at(tamanho_pontos_bordejo-1).latitude, pontos_bordejo.at(tamanho_pontos_bordejo-1).longitude, atual->latitude, atual->longitude);
	float d_p0_pd = (float)TinyGPS::distance_between(x0, y0, x, y);

	location aux;
	aux.latitude = destino->latitude;
	aux.longitude = destino->longitude;

	if(d_p0_pu > d_p0_pd){
		pontos_bordejo.at(tamanho_pontos_bordejo-1) = aux;
	} else {
		pontos_bordejo.push_back(aux);
	} 
	
	return pontos_bordejo;
}

void setup()
{
	// escolhendo waypoins
	// choosing waypoints 
	targetLocation.latitude = 0.0013888888;
	targetLocation.longitude = 0;
	waypoints.push_back(targetLocation);

	targetLocation.latitude = 0.0006944444;
	targetLocation.longitude = 0.0006944444;
	waypoints.push_back(targetLocation);

	targetLocation.latitude = 0;
	targetLocation.longitude = 0;
	waypoints.push_back(targetLocation);

	slaveAddress = HMC6352Address >> 1;
	Serial.begin(9600);  // Comunicação exterior xbee
	Serial1.begin(9600); // Communication b/w arduinos (recieve weather station and send setpoint to the rudder)
	//Serial2.begin(9600); 
	Serial3.begin(4800); // GPS
	Wire.begin();       // Compass 
	starttime = millis();
}

void loop() {
	
	for(int c = 0; c < 10; c++){
		// updates position 10 times before starting 
		atualizar_posicao(startLocation);
		delay(500);
	}

	// saves the currentLocation at lastLocation variable
	lastLocation = currentLocation;
	// updates the latitude and longitude of currentLocation 
	atualizar_posicao(currentLocation);
	// updates heading from the compass
	heading = dir_bussola();

	while (1) {
		// if more than 1 waypoint is stored in waypoints (type vector)
		if (waypoints.size() != 0) {
			if (distanciaAoDestino < 10) {
				if (isBordejando) {
					if (waypointBId < pontos_bordejar.size() - 1) {
						waypointBId += 1;
						nextLocation = pontos_bordejar.at(waypointBId);
					} else {
						isBordejando = false;
						waypointBId = 0;
					}
				} else {
					waypointId += 1;
					waypointId = waypointId % waypoints.size();
					nextLocation = waypoints.at(waypointId);          
				}
			} else {
				if (isBordejando) {
					nextLocation = pontos_bordejar.at(waypointBId);
				} else {
					nextLocation = waypoints.at(waypointId);
				}
			}
			// if no desired course is set (this defined by using controle)
			if (controle == 0) {
				lastLocation = currentLocation;
				distanciaInicial = calcular_distancia(startLocation, nextLocation);
				controle = 1;
			}
			//calculates the course and finds the direction(angle) that you must travel 
			sp = calcular_orientacao(currentLocation, nextLocation);

			lastDistanciaDestino = distanciaAoDestino;
			// distance to destination
			distanciaAoDestino = calcular_distancia(currentLocation, nextLocation);
			//total travelled distance
			distanciaPercorrida += calcular_distancia(lastLocation, currentLocation);

			// refer to control_leme 
			// this function controls the rudder 
			controle_rudder();
			
			// caso o veleiro não esteja avançando ao destino. solução: selecionar outro waypoint
			// if the sailboat is not advancing to the destination. Solution: select another waypoint
			if(distanciaAoDestino >= lastDistanciaDestino){
				if(contador_bug == 0){
					start = millis();
					contador_bug = 1;
				}
			} else {
				contador_bug = 0;
			}

			tempo = millis();
		// after 20000 milliseconds (20 seconds), set the distance to 0 
		if((tempo - start) > 20000){
			distanciaAoDestino = 0;
		}
			// calculates wind direction from the wind vane 
			biruta = wind_dir();
			// wind drirection is read, waypoint not reached, and error is within 5 
			if (abs(biruta) < 30 && !isBordejando && abs(erroAtual) < 5) {
				isBordejando = true;
				// stores bordejar ( which does __) into points_bordejar
				pontos_bordejar = bordejar(&lastLocation, &nextLocation);
			}
		}
		lastLocation = currentLocation;
		//update_ position
		atualizar_posicao(currentLocation);
		// update heading from the compass 
		heading = dir_bussola();
		// save_data()
		salvar_dados();
	}
}
    // send_check_database
void enviar_dados_estacaobase(){
	Serial.print('i');
	Serial.print(currentLocation.latitude, 6);
	Serial.print(currentLocation.longitude, 6);
	biruta = 100;
	heading = 101;
	Serial.print(biruta+400, 2);
	Serial.print(heading+400, 2);
	Serial.print(startLocation.latitude, 6);
	Serial.print(startLocation.longitude, 6);
	targetLocation = startLocation;
	Serial.print(targetLocation.latitude, 6);
	Serial.print(targetLocation.longitude, 6);
	Serial.print((int)sp+400);
	Serial.print((int)distanciaAoDestino+1000);
	Serial.print((int)distanciaInicial+1000);
	Serial.print('f');
}
 // save_data 
void salvar_dados(){
	dataFile = SD.open("testecom.txt", FILE_WRITE);
	if (dataFile) {
		dataFile.print(currentLocation.latitude, 6);
		dataFile.print(" ");
		dataFile.print(currentLocation.longitude, 6);
		dataFile.print(" ");
		dataFile.print(startLocation.latitude, 6);
		dataFile.print(" ");
		dataFile.print(startLocation.longitude, 6);
		dataFile.print(" ");
		dataFile.print(nextLocation.latitude, 6);
		dataFile.print(" ");
		dataFile.print(nextLocation.longitude, 6);
		dataFile.print(" ");
		dataFile.print(heading, 2);
		dataFile.print(" ");
		dataFile.print(sp);
		dataFile.print(" ");
		dataFile.print(biruta, 2);
		dataFile.print(" ");
		dataFile.println(distanciaInicial);
		dataFile.print(" ");
		dataFile.print(distanciaAoDestino);
		dataFile.close();
	}
}