/*
 * ESTE software foi fornecido como exemplo de controlador de futebol de robôs na Segunda Oficina Brasileira de Futebol de Robôs realizada junto ao 5o Workshop em Automação e Robótica Aplicada (Robocontrol) 2010.

 * Você que está de posse dESTE software, está livre para utilizá-lo, alterá-lo, copiá-lo e incluí-lo parcial ou integralmente em outros software desde que acompanhado da seguinte indicação:
 * "Este software tem seções de código desenvolvidas por Rene Pegoraro no Laboratório de Integração de Sistemas e Dispositivos Inteligentes (LISDI) do Departamento de Computação da Faculdade de Ciências da Universidade Estadual Paulista (UNESP) - Campos de Bauru - SP - Brasil"

 * Se qualquer publicação for gerada pela utilização de software utilizando parcial ou integralmente ESTE software, esta publicação deve conter os devidos créditos para o "Grupo de Integração de Sistemas e Dispositivos Inteligentes (GISDI) do Departamento de Computação da Faculdade de Ciências da Universidade Estadual Paulista (UNESP) - Campos de Bauru - SP - Brasil"
 */

#ifndef TIPOSCLASSES_H_
#define TIPOSCLASSES_H_

#include <gtkmm.h>
#include <glib.h>
#include <iostream>
#include <fstream>
#include <map>
#include <string>
#include <cstring>
#include <cmath>
#include <algorithm>
#include <limits.h>
//#include <gdk/gdkkeysyms.h>
using namespace std;

//#include <iostream>
//#include <fstream>
//#include <map>
//#include <string>
//#include <cstring>
//#include <cmath>
//#include <algorithm>
//using namespace std;

typedef unsigned char guint8;

//#define SEM_PREVISAO

//#define SERIAL	//usado para retirar as rotina da serial- SUBSTITUIDO por um dentro da serial.h

const int TAM_IMAGEM_640x480x3 = 640 * 480 * 3;
const int TAM_IMAGEM_640x480x1 = 640 * 480;

#define NUM_CAMERAS 1
/** Caracter de preenchimento dos limites para simplificar o tratamento das imagens */
#define SENTINELA 255
/** GAP espaço que nao precisa ser da cor atual, mas que será considerado como sendo (da cor atual)
 * se seguido por outro pixel da cor atual
 */
#define GAP 4
#define TAM_X_CAMPO_SEM_GOL 150	// em cemtimetros
#define TAM_Y_CAMPO 130	// em cemtimetros
#define TAM_X_DO_GOL 10	// em cemtimetros
#define TAM_X_CAMPO (TAM_X_CAMPO_SEM_GOL + TAM_X_DO_GOL+TAM_X_DO_GOL)	// em cemtimetros
#define TAM_Y_DO_GOL 40
#define CENTRO_X_GOL (TAM_X_CAMPO-TAM_X_DO_GOL)
#define CENTRO_Y_GOL TAM_Y_CAMPO/2
#define POS_X_GOLEIRO (TAM_X_DO_GOL + TAM_ROBO/2 + 2)	// 2 cm a mais para o robo ter espaco para andar
#define TAM_X_AREA 15
#define TAM_Y_AREA 70
#define TAM_ROBO 8 //tamanho do lado do robo em cm
#define DIST_GIRO TAM_ROBO*2
#define TAM_BOLA 4 //tamanho do raio da bola
//numero de robos em cada time
#define NUM_ROBOS_TIME	3
// numero de cores de etiquetas auxiliares
#define NUM_CORES_AUX NUM_ROBOS_TIME

#define NUM_IMGS_INTERVALO_CAMPO_VAZIO 13 // a cada 13 imagens, uma sera considerada no campo vazio
#define NUM_IMGS_CAMPO_VAZIO 5

#define IND_BOLA NUM_ROBOS_TIME

#define LINHA_DEFESA_VOLANTE ???
#define RAIO_ATACA 20 + TAM_ROBO / 2

//numero de quadros de atraso entre a visao e o comando ser recebido pelos robos
#define N_IMAGENS_ATRASO 5
#define N_IMAGENS_INERCIA 3

/** custo com alta penalidade indicando que nao existe
 * usado também -NAO_EXISTE, para agilizar comparacao <0, pode ser mais rapido que !=NAO_EXISTE (especialmente em reais (float)
 */
#define NAO_EXISTE INT_MAX/100

#define VELOCIDADE_MAXIMA 200	// maxima velocidade que se considera que um robo pode se deslocar
#define VEL_MAXIMA_ROBO_CM 150.0	// velocidade maxima de robo em cm
#define VEL_MAXIMA_ROBO_UNID 7		// velocidade que se pode atribuir ao robo
#define FATOR_VEL_ROBO_UNID_POR_CM (VEL_MAXIMA_ROBO_UNID/VEL_MAXIMA_ROBO_CM) // fator de conversao entre a velocidade em cm para unidades
#define DIST_ENTRE_RODAS 7
#define FATOR_ANGULO_DIF_RODAS_UNID_VEL 5.83 // atang2(VEL_MAXIMA_ROBO_CM/7/30, DIST_ENTRE_RODAS) a cada 5.83 graus significa uma diferenca entre rodas de uma unidade de velocidade em 1/30s
#if NUM_ROBOS_TIME==3
#define MAX_CONT_COMB 6
#elif NUM_ROBOS_TIME==4
#define MAX_CONT_COMB 24
#elif NUM_ROBOS_TIME==5
#define MAX_CONT_COMB 120
#else
Erro, precisa rever a criacao das combinacoes
#endif

enum IndCores {
	COR_BOLA, COR_TIME, COR_CONTRA, COR_AUX1, COR_AUX2, COR_AUX3, MAX_IND_CORES
};
#define SEM_COR MAX_IND_CORES

struct Cor {
	unsigned char b, g, r;
};

class Props: public map<string, string> {
	string nomeArq;

public:
	Props(string nomeArq);
	void carrega();
	string get(string chave, string padrao = "");
	int getInt(string chave, int padrao = 0);
	int getFloat(string chave, float padrao = 0.0);
	void put(string chave, string valor);
	void put(string chave, int valor);
	void put(string chave, float valor);
	void salva(string header = "");
	~Props();
};

enum Lado {
	ESQUERDA = 1, DIREITA
};

struct Estado {
	float angulo;
	float dAngulo;
	float x, y;
	float dx, dy;
};

struct PosXYAux {
	float x, y, xAux, yAux;
};

struct PosXY {
	float x, y;
};

struct Objetivo {
	float angulo;
	float x, y;
	float vel;
};

struct LimitesCampo {
	int minXSemGol, maxXSemGol;
	int minX, maxX, minY, maxY;
};

struct CmdEnviado {
	signed char esq, dir, vel; //v e' a velocidade esperada
};

void erro(string msg);

#endif /* TIPOSCLASSES_H_ */
