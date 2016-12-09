/**
 * @version 2013
 * Código utilizado para o Campeonato Brasileiro de Robótica de 2013 na categoria IEEE Very Small Size
 * Posição: 2º lugar
 * @version 2014
 * Código reutilizado para o Campeonato Brasileiro de Robótica de 2014 na categoria IEEE Very Small Size
 * Posição: Desclassificado fase de grupos [Problema de Hardware no transmissor de rádio]
 * @version 2015
 * Código a ser utilizado para o Campeonato Brasileiro de Robótica de 2015 na categoria IEEE Very Small Size
 * Start at commit: ???
 * Posição: ???
 * Mudanças: ???
 */

#include "TiposClasses.h"
#include "Auxiliares.h"
#include "Controle.h"

#define CteEstCmd 0.5
#define CteEstVel 0.5
#define CTE_PARADA 4

#define CPH_ou_CPO_ou_CPLO // SE FOR DEFINIDO UM DESSES CAMPOS POTENCIAIS (CPH, CPO E CPLO) DEFINIR ESSA CONSTANTE, SENÁO NÁO DEFINIR
#define CPH
//#define CPO
//#define CPLO
//#define CPK
//#define SCP

#define VELOCIDADE_ANGULAR_MAXIMA 17.8	//(vMaxRd-vMaxRe)*RAIO_RODA/DIST_RODAS//Vel1 = 8.9 //Vel2 = 17.85 //Vel3 = 26.8 //Vel4 = 35.7
#define VELOCIDADE_MAX 107.1			//(vMaxRe+vMaxRd)*RAIO_RODA/2 //Vel3 = 107.1 //Vel4 = 142.8 //Vel5 = 178.5
#define RAIO_DA_RODA 1.6
#define RAIO_DISTANCIA 5
#define VEL_MAX 2
#define MASSA_ROBO 0.89

extern FutebolCamera *futCam[NUM_CAMERAS];

extern Estado estado[NUM_ROBOS_TIME * 2 + 1], estadoAnt[NUM_ROBOS_TIME * 2 + 1],
		estadoPrev[NUM_ROBOS_TIME * 2 + 1];

extern CmdEnviado cmdEnviado[10][NUM_ROBOS_TIME]; //comando enviado aos robos

float xObjAnt[3] = { 0, 0, 0 }, yObjAnt[3] = { 0, 0, 0 };

extern int indAtacante;
extern int indVolante;
extern int indGoleiro;

/**
 * CAMPOS POTENCIAIS
 * 1 - Bibliotecas
 * 2 - Constantes
 * 3 - Estrutura de Dados
 * 4 - Funções
 */

/**
 * 1 - Bibliotecas
 */
#include <stdlib.h>
#include <stdio.h>
#include <stdbool.h>
#include <math.h>

/**
 * 2 - Constantes
 */
#define MAX_X 43
#define MAX_Y 33			// Tem que ser 35 e deslocar 1 em todas as coordenadas em Y
#define DIV_CAMPO 4
#define W_SOR 1.8
#define	WND_X 1350			// TAMANHO DA JANELA GRÁFICA EM X ----- VOLTAR PARA 1350
#define	WND_Y 350			// TAMANHO DA JANELA GRÁFICA EM Y -----	VOLTAR PARA 350
#define TAM_RET 10 			// SEMPRE QUE ALTERAR TAM_RET DEVE-SE ALTERAR LINE_LENGTH ------ VOLTAR PARA 10
#define LINE_LENGTH 8.0		// TAM_RET - 0.2*TAM_RET -> ESSA CONSTATE SEMPRE SERÁ DOUBLE/FLOAT
#define E 0.00001			// PRECISÃO DE CONVERGÊNCIA
#define X 0
#define Y 1
#define E_CPO 1

/**
 * 3 - Estrutura de Dados
 */
typedef struct position {
	int posX;
	int posY;
	position *nextPosition;
} position_list;

position_list *caminhoRobo[3] = { NULL, NULL, NULL };

typedef struct campo {
	float matPot[MAX_X][MAX_Y]; // Valores pertencentes ao intervalo [0,1]
	bool matBoolPot[MAX_X][MAX_Y]; // true = obstáculo ou meta, false = espaço livre
} campoPot;

typedef struct campoK {
	double matAng[MAX_X][MAX_Y]; // Ângulo do vetor de força daquela célula
	float matPot[MAX_X][MAX_Y]; // Valores pertencentes ao intervalo [0,1]
	bool matBoolPot[MAX_X][MAX_Y]; // (Meta e Obstáculo) = TRUE (Célula livre) = FALSE
} campoPotKhatib;

/**
 * 4 - Funções
 */

/**
 * Função que inicializa a matriz de campo potenciais com os valores homogêneos
 * @param campoPotencial Matriz do Campo Potencial
 * @param xObjetivo      Coordenada X do objetivo vinda da estratégia
 * @param yObjetivo      Coordenada Y do objetivo vinda da estratégia
 * @param indJogador     Índice do Jogador ao qual o Campo Potencial se refere
 */
void inicializa_obst_meta(campoPot *campoPotencial, int xObjetivo,
		int yObjetivo, int indJogador) {
	int i, j, k;

	/**
	 * Inicializa paredes do campo de futebol
	 */
	for (i = 0; i < MAX_Y; i++) { // Parede da esquerda
		campoPotencial->matPot[0][i] = 1;
		campoPotencial->matBoolPot[0][i] = true;
	}
	for (i = 0; i < MAX_Y; i++) { // Parede da direita
		campoPotencial->matPot[MAX_X - 1][i] = 1;
		campoPotencial->matBoolPot[MAX_X - 1][i] = true;
	}
	for (i = 0; i < MAX_X; i++) { // Parede de baixo
		campoPotencial->matPot[i][0] = 1;
		campoPotencial->matBoolPot[i][0] = true;
	}
	for (i = 0; i < MAX_X; i++) { // Parede de cima
		campoPotencial->matPot[i][MAX_Y - 1] = 1;
		campoPotencial->matBoolPot[i][MAX_Y - 1] = true;
	}
	/**
	 * Inicializa células livres
	 */
	for (i = 1; i < MAX_X - 1; i++) {
		for (j = 1; j < MAX_Y - 1; j++) {
			campoPotencial->matBoolPot[i][j] = false;
			campoPotencial->matPot[i][j] = 0;
		}
	}
	/**
	 * Define Células que contém o objetivo do robô
	 */
	campoPotencial->matBoolPot[xObjetivo][yObjetivo] = true;

	/**
	 * Parede virtual atrás da bola para evitar que o robô conduza a bola contra o próprio gol
	 */
	if (indJogador == indAtacante) {
		for (j = yObjetivo - 1; j <= yObjetivo + 2; j++) {
			for (i = xObjetivo + 1; i < xObjetivo + 4; i++) {
				campoPotencial->matBoolPot[i][j] = true; // parede virtual da meta
				campoPotencial->matPot[i][j] = 1;
			}
		}
	}
	for (i = 0; i <= 2; i++) {
		if (i != indJogador) {
			if ((xObjetivo < estadoPrev[i].x / DIV_CAMPO - 1
					|| xObjetivo > estadoPrev[i].x / DIV_CAMPO + 1)
					&& (yObjetivo < estadoPrev[i].y / DIV_CAMPO - 1
							|| yObjetivo > estadoPrev[i].y / DIV_CAMPO + 1))
				for (j = (int) estadoPrev[i].x / DIV_CAMPO - 1;
						j <= (int) estadoPrev[i].x / DIV_CAMPO + 1; j++) {
					for (k = (int) estadoPrev[i].y / DIV_CAMPO - 1;
							k <= (int) estadoPrev[i].y / DIV_CAMPO + 1; k++) {
						campoPotencial->matPot[j][k] = 1;
						campoPotencial->matBoolPot[j][k] = true;
					}
				}
		}
	}
	/**
	 * Insere obstáculos no campo potencial referente aos robôs adversários
	 */
	for (i = 4; i <= 6; i++) {
		campoPotencial->matPot[(int) estadoPrev[i].x / DIV_CAMPO][(int) estadoPrev[i].y
				/ DIV_CAMPO] = 1;
		campoPotencial->matBoolPot[(int) estadoPrev[i].x / DIV_CAMPO][(int) estadoPrev[i].y
				/ DIV_CAMPO] = true;

	}
}

/**
 * Função que inicializa a matriz de campo potenciais de Khatib com os valores homogêneos
 * @param campoPotencial Matriz do Campo Potencial
 * @param xObjetivo      Coordenada X do objetivo vinda da estratégia
 * @param yObjetivo      Coordenada Y do objetivo vinda da estratégia
 * @param indJogador     Índice do Jogador ao qual o Campo Potencial se refere
 * @param angObjetivo    Ângulo objetivo vindo da estratégia
 */
void inicializa_obst_meta_khatib(campoPotKhatib *campoPotencial, int xObjetivo,
		int yObjetivo, int indJogador, double angObjetivo) {
	int i, j, k;
	float K = 0.5;
	double auxAng;

	for (i = 0; i < MAX_Y; i++) {
		campoPotencial->matPot[0][i] = 1;
		campoPotencial->matBoolPot[0][i] = true;
	}
	for (i = 0; i < MAX_Y; i++) {
		campoPotencial->matPot[MAX_X - 1][i] = 1;
		campoPotencial->matBoolPot[MAX_X - 1][i] = true;
	}
	for (i = 0; i < MAX_X; i++) {
		campoPotencial->matPot[i][0] = 1;
		campoPotencial->matBoolPot[i][0] = true;
	}
	for (i = 0; i < MAX_X; i++) {
		campoPotencial->matPot[i][MAX_Y - 1] = 1;	// inicializam as paredes
		campoPotencial->matBoolPot[i][MAX_Y - 1] = true;
	}

	for (i = 1; i < MAX_X - 1; i++) {
		for (j = 1; j < MAX_Y - 1; j++) {
			campoPotencial->matBoolPot[i][j] = false; // inicializa células livres
			campoPotencial->matPot[i][j] = 0;
		}
	}

	campoPotencial->matBoolPot[xObjetivo][yObjetivo] = true;

	for (i = 0; i <= 6; i++) {
		if (i != indJogador && i != 3) {
			if ((xObjetivo < estadoPrev[i].x / DIV_CAMPO - 1
					|| xObjetivo > estadoPrev[i].x / DIV_CAMPO + 1)
					&& (yObjetivo < estadoPrev[i].y / DIV_CAMPO - 1
							|| yObjetivo > estadoPrev[i].y / DIV_CAMPO + 1))
				for (j = (int) estadoPrev[i].x / DIV_CAMPO;
						j <= (int) estadoPrev[i].x / DIV_CAMPO + 1; j++) {
					for (k = (int) estadoPrev[i].y / DIV_CAMPO;
							k <= (int) estadoPrev[i].y / DIV_CAMPO + 1; k++) {
						campoPotencial->matPot[j][k] = 1;
						campoPotencial->matBoolPot[j][k] = true;
					}
				}
		}
	}

	for (i = 1; i < MAX_X - 1; i++) {
		for (j = 1; j < MAX_Y - 1; j++) {
			if (!campoPotencial->matBoolPot[i][j]) { // inicializa células livres
				campoPotencial->matPot[i][j] = K;
				auxAng = atan2((double) (yObjetivo - j),
						(double) (xObjetivo - i));
				if (auxAng < 0)

					auxAng += 2 * M_PI;
				campoPotencial->matAng[i][j] = auxAng;
				printf("%f\n", campoPotencial->matAng[i][j]);

			}
		}
	}
}

/**
 * Calcula Campo Potencial Harmônico utilizando método de relaxação/relaxamento SOR
 * @param  campoPotencial Matriz do Campo Potencial
 * @return boolean		Boolean indicando convergência do cálculo do campo potencial
 */
bool calcula_campo_SOR(campoPot *campoPotencial) {
	int i, j;
	float resultTemp;
	bool convergiu = true;
	do {
		convergiu = true;
		for (i = 1; i < MAX_X - 1; i++) {
			for (j = 1; j < MAX_Y - 1; j++) {
				if (campoPotencial->matBoolPot[i][j] == false) {
					resultTemp = W_SOR
							* (campoPotencial->matPot[i + 1][j]
									+ campoPotencial->matPot[i - 1][j]
									+ campoPotencial->matPot[i][j + 1]
									+ campoPotencial->matPot[i][j - 1]
									- 4 * campoPotencial->matPot[i][j]) / 4
							+ campoPotencial->matPot[i][j];
					if ((campoPotencial->matPot[i][j] - resultTemp > E)
							|| (resultTemp - campoPotencial->matPot[i][j] > E))
						convergiu = false;
					campoPotencial->matPot[i][j] = resultTemp;
				}

			}
		}
	} while (!convergiu);
	return convergiu;
}

/**
 * Calcula Campo Potencial Localmente Orientado utilizando método de relaxação/relaxamento SOR
 * @param  campoPotencial Matriz do Campo Potencial
 * @param  v_CPO          Vetor com a orientação do campo potencial
 * @param  xObjetivo      Coordenada X do objetivo vinda da estratégia
 * @param  yObjetivo      Coordenada Y do objetivo vinda da estratégia
 * @return boolean        Boolean indicando convergência do cálculo do campo potencial
 */
bool calcula_campo_SOR_CPLO(campoPot *campoPotencial, float v_CPO[2],
		int xObjetivo, int yObjetivo) {
	int i, j, cont = 0;
	float resultTemp;
	bool convergiu = true;
	do {
		for (i = 1; i < MAX_X - 1; i++) {
			for (j = 1; j < MAX_Y - 1; j++) {
				if (campoPotencial->matBoolPot[i][j] == false) {
					if ((i < xObjetivo + 5 && j < yObjetivo + 5)
							&& (i > xObjetivo - 5 && j > yObjetivo - 5)) {
						resultTemp = (campoPotencial->matPot[i + 1][j]
								+ campoPotencial->matPot[i - 1][j]
								+ campoPotencial->matPot[i][j + 1]
								+ campoPotencial->matPot[i][j - 1]) / 4
								+ ((campoPotencial->matPot[i + 1][j]
										- campoPotencial->matPot[i - 1][j])
										* v_CPO[X]
										+ (campoPotencial->matPot[i][j + 1]
												- campoPotencial->matPot[i][j
														- 1]) * v_CPO[Y])
										* E_CPO / 8;
					} else {
						resultTemp = W_SOR
								* (campoPotencial->matPot[i + 1][j]
										+ campoPotencial->matPot[i - 1][j]
										+ campoPotencial->matPot[i][j + 1]
										+ campoPotencial->matPot[i][j - 1]
										- 4 * campoPotencial->matPot[i][j]) / 4
								+ campoPotencial->matPot[i][j];

					}
					if ((campoPotencial->matPot[i][j] - resultTemp > E)
							|| (resultTemp - campoPotencial->matPot[i][j] > E))
						convergiu = false;
					campoPotencial->matPot[i][j] = resultTemp;
				}

			}
		}
		cont++;
	} while (!convergiu && cont < 500);
	return convergiu;
}

/**
 * Calcula Campo Potencial Orientado utilizando método de relaxação/relaxamento SOR
 * @param  campoPotencial Matriz do Campo Potencial
 * @param  v_CPO          Vetor com a orientação do campo potencial
 * @return boolean  	  Boolean indicando convergência do cálculo do campo potencial
 */
bool calcula_campo_SOR_CPO(campoPot *campoPotencial, float v_CPO[2]) {
	int i, j, cont = 0;
	float resultTemp;

	bool convergiu = true;
	do {
		for (i = 1; i < MAX_X - 1; i++) {
			for (j = 1; j < MAX_Y - 1; j++) {
				if (campoPotencial->matBoolPot[i][j] == false) {
					resultTemp = (campoPotencial->matPot[i + 1][j]
							+ campoPotencial->matPot[i - 1][j]
							+ campoPotencial->matPot[i][j + 1]
							+ campoPotencial->matPot[i][j - 1]) / 4
							+ ((campoPotencial->matPot[i + 1][j]
									- campoPotencial->matPot[i - 1][j])
									* v_CPO[X]
									+ (campoPotencial->matPot[i][j + 1]
											- campoPotencial->matPot[i][j - 1])
											* v_CPO[Y]) * E_CPO / 8;
					if ((campoPotencial->matPot[i][j] - resultTemp > E)
							|| (resultTemp - campoPotencial->matPot[i][j] > E))
						convergiu = false;
					campoPotencial->matPot[i][j] = resultTemp;
				}

			}
		}
		cont++;
	} while (!convergiu && cont < 500);
	return convergiu;
}

/**
 * Calcula Campo Potencial de Khatib
 * @param  campoPotencial Matriz do Campo Potencial
 * @return boolean  	  Boolean indicando convergência do cálculo do campo potencial
 */
bool calcula_campo_CP(campoPotKhatib *campoPotencial) {
	int i, j, k, l, sum, sum2;
	float F1, F2, FR;
	double auxAng, difAng;
#define K  0.5
	int d = 4;
	bool convergiu = true;
	do {
		convergiu = true;
		for (i = 1; i < MAX_X - 1; i++) {
			for (j = 1; j < MAX_Y - 1; j++) {
				if (campoPotencial->matBoolPot[i][j]
						&& campoPotencial->matPot[i][j] == 1) {
					sum = i - d;
					if (sum < 1)
						sum = 1;
					sum2 = j - d;
					if (sum2 < 1)
						sum2 = 1;
					for (k = sum; k < i + d && k <= MAX_X - 1; k++) {
						for (l = sum2; l < j + d && k <= MAX_Y - 1; l++) {
							if (!campoPotencial->matBoolPot[k][l]) {
								auxAng = atan2((double) (j - l) * (-1),
										(double) (i - k) * (-1));
								if (auxAng < 0)
									auxAng += 2 * M_PI;
								if (campoPotencial->matAng[k][l] < 0)
									campoPotencial->matAng[k][l] += 2 * M_PI;
								if (auxAng
										>= campoPotencial->matBoolPot[k][l]) {
									difAng = auxAng
											- campoPotencial->matAng[k][l];
									F2 = campoPotencial->matPot[k][l];
									F1 = K / (pow(k - i, 2) + pow(l - j, 2));
									auxAng = campoPotencial->matAng[k][l];
								} else {
									difAng = campoPotencial->matAng[k][l]
											- auxAng;
									F1 = campoPotencial->matPot[k][l];
									F2 = K / (pow(k - i, 2) + pow(l - j, 2));
								}
								FR = sqrt(
										pow(F1, 2) + pow(F2, 2)
												+ 2 * F1 * F2 * cos(difAng));
								campoPotencial->matAng[k][l] = asin(
										(F2 * sin(M_PI - difAng)) / FR)
										+ auxAng;
								if (FR > 1)
									FR = 1;
								campoPotencial->matPot[k][l] = FR;
							}
						}
					}
				}
				while (campoPotencial->matAng[i][j] < -M_PI)
					campoPotencial->matAng[i][j] += 2 * M_PI;
				while (campoPotencial->matAng[i][j] > M_PI)
					campoPotencial->matAng[i][j] -= 2 * M_PI;
			}
		}
	} while (!convergiu);
	return convergiu;
}
/**
 * Variáveis e Constantes Controle PID
 */
#define TEMPO_EXECUCAO 1/30000 		// em milisegundos
#define KP 4						// Constante Proporcional de ganho do PID
#define KI 2 * TEMPO_EXECUCAO 	// Constante Integrativa de ganho do PID
#define KD 2 / TEMPO_EXECUCAO 	// Constante Derivativa de ganho do PID
static double ITerm;
//Constante da força aplicada pelo campo potencial
#define KF 0.5
#define K_ALFA 0.07

/**
 * PID baseado na biblioteca do PIDArduino
 * Utlizado para calcular a velocidade angular considerando a diferença entre ângulos
 * @param  error     diferença entre ângulos
 * @param  lastInput Último ângulo calculado do Robô
 * @param  input     Atual ângulo calculado do Robô
 * @return           Velocidade Agular
 */
double PID(double error, double lastInput, double input) {
	ITerm += (KI * error);
	double dInput = (input - lastInput);
	/*Compute PID Output*/
	// double output = KP * error + ITerm - KD * dInput;
	double output = KP * error - KD * dInput;

	return output;
}

campoPot campoPotencial[3];
campoPotKhatib campoPotencialKhatib[3];
/**
 * Calcula comando a ser enviado pelo rádio para as rodas esquerda e direita do robô
 * @param indJogador  Índice do Jogador ao qual o Campo Potencial se refere
 * @param angObjetivo Ângulo do objetivo vinda da estratégia
 * @param xObjetivo   Coordenada X do objetivo vinda da estratégia
 * @param yObjetivo   Coordenada Y do objetivo vinda da estratégia
 * @param velObjetivo Velocidade desejada vinda da estratégia
 * @return cmdEnviado[0][indJogador].esq = pe; Altera Variável Global de Comando que vai ser enviado para o rádio - Roda Esquerda
 * @return cmdEnviado[0][indJogador].dir = pd; Altera Variável Global de Comando que vai ser enviado para o rádio - Roda Direita
 */
void calculaCmd(int indJogador, int angObjetivo, int xObjetivo, int yObjetivo,
		int velObjetivo, bool isKick) {

	float wObj, vObj, vAnt = 0, F, dx, dy, v_CPO[2], d, angRobo, angRoboAnt,
			xRobo, yRobo, ang, auxAng;
	int ve, vd, pe, pd, i, j, wSignal, vSignal, flagDirection;
	double directionAngle, K_ro, K_alfa, lim = 180 / 8;
	static int count = 0;

	/**
	 * Controle dos Robôs Atacante e Defensor
	 */
	if (indJogador == indAtacante || indJogador == indVolante) {
		if (!isKick) {
#ifndef SCP
			if (xObjetivo > 0 && yObjetivo > 0) {
#ifdef CPH_ou_CPO_ou_CPLO
				v_CPO[0] = cos((double) (angObjetivo) / 180 * M_PI);
				v_CPO[1] = sin((double) (angObjetivo) / 180 * M_PI);

				// v_CPO[0] = 1;
				// v_CPO[1] = 1;
				inicializa_obst_meta(&campoPotencial[indJogador],
						xObjetivo / DIV_CAMPO, yObjetivo / DIV_CAMPO,
						indJogador);
#ifdef CPH
				calcula_campo_SOR(&campoPotencial[indJogador]);
#endif
#ifdef CPO
				calcula_campo_SOR_CPO(&campoPotencial[indJogador], v_CPO);
#endif
#ifdef CPLO
				calcula_campo_SOR_CPLO(&campoPotencial[indJogador], v_CPO, xObjetivo, yObjetivo);
#endif
#endif
#ifdef CPK
				inicializa_obst_meta_khatib(&campoPotencialKhatib[indJogador],
						xObjetivo / DIV_CAMPO, yObjetivo / DIV_CAMPO, indJogador,
						angObjetivo);
				calcula_campo_CP(&campoPotencialKhatib[indJogador]);
#endif
			}
#endif

			xObjAnt[indJogador] = xObjetivo;
			yObjAnt[indJogador] = yObjetivo;
			angRobo = estadoPrev[indJogador].angulo;
			angRoboAnt = angRobo;
			xRobo = estadoPrev[indJogador].x;
			yRobo = estadoPrev[indJogador].y;
			dx = xObjetivo - xRobo;
			dy = yObjetivo - yRobo;
			d = sqrt(dx * dx + dy * dy);
			i = xRobo / DIV_CAMPO;
			j = yRobo / DIV_CAMPO;

#ifdef CPH_ou_CPO_ou_CPLO
			directionAngle = atan2(
					(double) (campoPotencial[indJogador].matPot[i][j - 1]
							- campoPotencial[indJogador].matPot[i][j + 1]),
					(double) (campoPotencial[indJogador].matPot[i - 1][j]
							- campoPotencial[indJogador].matPot[i + 1][j]))
					* 180 / M_PI;
			ang = (float) directionAngle;

			if (ang < 0) {
				ang += 360;
			}
#endif

#ifdef CPK
			directionAngle = campoPotencialKhatib[indJogador].matAng[i][j] * 180/M_PI;
			ang = (float) directionAngle;
			if (ang < 0) {
				ang += 360;
			}
#endif

#ifdef DEBUG
			printf("[%d](%f)\n", indJogador, (float)directionAngle);
			printf("[%d](%f)\n[        ][%f][        ]\n[%f][        ][%f]\n[        ][%f][        ]\n",
					indJogador, (float) directionAngle,
					campoPotencial[indJogador].matPot[i][j + 1],
					campoPotencial[indJogador].matPot[i - 1][j],
					campoPotencial[indJogador].matPot[i + 1][j],
					campoPotencial[indJogador].matPot[i][j - 1]);
#endif
#ifdef SCP
			ang = atan2(dy, dx) * 180 / M_PI;
#endif
			printf("i=%d | j=%d \n", i, j);

			/**
			 * ver cálculo dos ângulos para que o robô não precise dar uma volta inteira.
			 * Utilizando flag de direção, primeiro e quarto quadrante flag = 1, e segundo e terceiro quadrante flag = -1.
			 */

			/**
			 * Início do Novo Controle de Velocidade
			 */
			if (angRobo > 180) {
				angRobo -= 360;
			}
			double dAng = ang - angRobo;
			if (dAng > 180) {
				dAng -= 360;
			}
			auxAng = dAng;
			if ((dAng <= 90 && dAng > 0) || (dAng <= 0 && dAng >= -90)) {
				flagDirection = 1;
			} else if ((dAng <= 180 && dAng > 90)
					|| (dAng < -90 && dAng >= -180)) {
				flagDirection = -1;
				if (dAng <= 180 && dAng > 90) {
					dAng = 180 - dAng;
				} else {
					dAng = -180 - dAng;
				}
			}
			/**
			 * Início do trecho do código que deve ser alterado
			 * Alterar cálculo das velocidades angular e linear a seguir
			 */

			//Força aplicada na célula  do campo potencial
			F = KF * campoPotencial[indJogador].matPot[i][j];
			printf("dAng = %f - %f = %f  \n", ang, angRobo, dAng);

			//Velocidade do robô calculada em função da força vinda do campo potencial
			vObj = (2 / MASSA_ROBO)
					* (F * sin(directionAngle * (M_PI / 180)) * (dy)
							+ F * cos(directionAngle * (M_PI / 180)) * (dx))
					+ pow(vAnt, 2);
			K_ro = vObj / (d * cos(dAng * (M_PI / 180)));
			if (vObj < 127 && vObj >= 0) {
				vObj = 127 - vObj;
			} else {
				if (vObj > -127 && vObj < 0)
					vObj = 127 + vObj;
			}
			vObj = vObj * cos(dAng * (M_PI / 180));

			vAnt = vObj;
			// double errorAng = atan2(sin(dAng * (M_PI/180)), cos(dAng * (M_PI/180)));
			// wObj = K_ro*sin(dAng)*cos(dAng) + PID(errorAng, angRoboAnt, angRobo);
			wObj = K_ro * sin(dAng * (M_PI / 180)) * cos(dAng * (M_PI / 180))
					+ K_ALFA * dAng;
			if (wObj < 0) {
				wSignal = -1;
			} else {
				wSignal = 1;
			}
			if (vObj < 0) {
				vSignal = -1;
			} else {
				vSignal = 1;
			}

			if (abs(vObj) > 127)
				vObj = 127 * vSignal;
			if (indJogador != indAtacante) { // Robo defensor
				if (d < RAIO_DISTANCIA) { // Se a distancia for menor que o raio de distancia aceitavel chegou no objetivo
					if (abs(vObj) > 0)
						vObj = 0;
				}
			}
			if (d > RAIO_DISTANCIA) { // Se a distancia for maior que o raio de distancia aceitavel esta errado
				if (abs(vObj) < 42)
					vObj = 42 * vSignal;
			} else {
				if (abs(dAng) >= 80 || velObjetivo == 0)
					if (vObj > 0)
						vObj = 0;
			}
			switch ((int) abs(vObj / 21)) {
			case 1:
				if (abs(wObj) > 6)
					wObj = 6 * wSignal;
				break;
			case 2:
				if (abs(wObj) > 11)
					wObj = 11 * wSignal;

				break;
			case 3:
				if (abs(wObj) > 17)
					wObj = 17 * wSignal;
				break;
			case 4:
				if (abs(wObj) > 22)
					wObj = 22 * wSignal;
				break;
			case 5:
				if (abs(wObj) > 30)
					wObj = 30 * wSignal;
				break;
			case 6:
				if (abs(wObj) > 36)
					wObj = 36 * wSignal;
				break;
				// case 7:
				// if (abs(wObj) > 42)
				// 	wObj = 42 * wSignal;
				// break;
			}

			/**
			 * Fim do trecho do código que deve ser alterado para o cálculo de
			 * velocidade considerando a função candidata de Lyapunov
			 */
			printf("vObj = %f| wObj = %f |", vObj, wObj);
			pe = (int) ((2 * vObj - wObj * DIST_ENTRE_RODAS)
					/ (2 * RAIO_DA_RODA)) / 21;
			printf("pe = %d |", pe);
			pe *= flagDirection;
			if (pe > 7) {
				pe = 7;
			} else if (pe < -7)
				pe = -7;
			if (pe < 0)
				pe = -pe + 8;
			pd = (int) ((2 * vObj + wObj * DIST_ENTRE_RODAS)
					/ (2 * RAIO_DA_RODA)) / 21;
			printf("pd = %d |\n", pd);
			pd *= flagDirection;
			if (pd > 7) {
				pd = 7;
			} else if (pd < -7)
				pd = -7;
			if (pd < 0)
				pd = -pd + 8;
			/**
			 * Fim do novo controle de velocidade
			 */

			/**
			 * MB Após escolher a velocidade chamar a função calcula velocidade dos motores
			 * presente no controle de 2012, para evitar possíveis erros de
			 * velocidade 
			 */

			cmdEnviado[0][indJogador].esq = pe * 1;
			cmdEnviado[0][indJogador].dir = pd * 1;
			/**
			 * Chute rodando no pr´oprio eixo
			 */
		} else {
			xRobo = estadoPrev[indJogador].x;
			yRobo = estadoPrev[indJogador].y;
			if (yRobo <= 65) { //Canto inferior
				cmdEnviado[0][indJogador].esq = -5 * 1;
				cmdEnviado[0][indJogador].dir = 5 * 1;
			} else { // Canto superior
				cmdEnviado[0][indJogador].esq = 5 * 1;
				cmdEnviado[0][indJogador].dir = -5 * 1;
				printf("chute esquerda\n");
			}
		}

		/**
		 * Controle do Robô Goleiro
		 */
	} else {
		double lim = 180 / 8;

		angRobo = estadoPrev[indJogador].angulo;
		xRobo = estadoPrev[indJogador].x;
		yRobo = estadoPrev[indJogador].y;
		dx = xObjetivo - xRobo;
		dy = yObjetivo - yRobo;
		d = sqrt(dx * dx + dy * dy);

		ang = atan2(dy, dx) * 180 / M_PI;
		if (ang < 0) {
			ang += 2 * 180;
		}
		double dAng = ang - angRobo;
		if (dAng < 0) {
			dAng += 2 * 180;
		}

		if (d < 10) {
			vObj = 2;
			wObj = 1;
		} else if (d < 6) {
			vObj = 0;
			wObj = 0;
		} else {
			vObj = 3;
			wObj = 1;
		}

		if (dAng < lim) {
			ve = vObj;
			vd = vObj;
		} else if (dAng < 180 / 2 - lim) {
			ve = 0;
			vd = wObj;
		} else if (dAng < 180 / 2) {
			ve = -wObj;
			vd = wObj;
		} else if (dAng < 180 / 2 + lim) {
			ve = wObj;
			vd = -wObj;
		} else if (dAng < 180 - lim) {
			ve = 0;
			vd = -wObj;
		} else if (dAng < 180 + lim) {
			ve = -vObj;
			vd = -vObj;
		} else if (dAng < 3 * 180 / 2 - lim) {
			ve = -wObj;
			vd = 0;
		} else if (dAng < 3 * 180 / 2) {
			ve = -wObj;
			vd = wObj;
		} else if (dAng < 3 * 180 / 2 + lim) {
			ve = wObj;
			vd = -wObj;
		} else if (dAng < 2 * 180 - lim) {
			ve = wObj;
			vd = 0;
		} else {
			ve = vObj;
			vd = vObj;
		}

		cmdEnviado[0][indJogador].esq = ve * 1;
		cmdEnviado[0][indJogador].dir = vd * 1;
	}
}

