/*
 * ESTE software foi fornecido como exemplo de controlador de futebol de robôs na Segunda Oficina Brasileira de Futebol de Robôs realizada junto ao 5o Workshop em Automação e Robótica Aplicada (Robocontrol) 2010.

 * Você que está de posse dESTE software, está livre para utilizá-lo, alterá-lo, copiá-lo e incluí-lo parcial ou integralmente em outros software desde que acompanhado da seguinte indicação:
 * "Este software tem seções de código desenvolvidas por Rene Pegoraro no Laboratório de Integração de Sistemas e Dispositivos Inteligentes (LISDI) do Departamento de Computação da Faculdade de Ciências da Universidade Estadual Paulista (UNESP) - Campos de Bauru - SP - Brasil"

 * Se qualquer publicação for gerada pela utilização de software utilizando parcial ou integralmente ESTE software, esta publicação deve conter os devidos créditos para o "Grupo de Integração de Sistemas e Dispositivos Inteligentes (GISDI) do Departamento de Computação da Faculdade de Ciências da Universidade Estadual Paulista (UNESP) - Campos de Bauru - SP - Brasil"
 */

#include "Simulador.h"

extern bool emJogo;

extern int indGoleiro;
extern int indVolante;
extern int indAtacante;
int Goleiro1 = 0;
int Volante1 = 1;
int Atacante1 = 2;
extern bool tiroMeta;
extern bool emPenalidade;
extern bool emPosiciona;
extern bool emInicio;
extern bool emSimulacao;

int quadro;

extern Estado estado[NUM_ROBOS_TIME * 2 + 1], estadoAnt[NUM_ROBOS_TIME * 2 + 1], estadoPrev[NUM_ROBOS_TIME * 2 + 1];
Estado estado1[NUM_ROBOS_TIME * 2 + 1], estado1Ant[NUM_ROBOS_TIME * 2 + 1], estado1Prev[NUM_ROBOS_TIME * 2 + 1];
Estado estadoReal[NUM_ROBOS_TIME * 2 + 1], estadoRealAnt[NUM_ROBOS_TIME * 2 + 1];

extern Objetivo objetivoRobo[NUM_ROBOS_TIME];

#define ESCALA 4


void PosicionaRobos(void) {

}
float xVisao[7][3], yVisao[7][3], angVisao[7][3];

    void Visao(void) {
        int x, y, ang;
        int i;
        // ------------- nosso time
        for (i = 0; i < 7; i++) {
            estadoAnt[i] = estado[i];
            if (emJogo) {
                x = estado[i].x; //coordenada obtida da camera
                y = estado[i].y;
                ang = estado[i].angulo;

                xVisao[i][2] = xVisao[i][1];
                xVisao[i][1] = xVisao[i][0];
                xVisao[i][0] = x;

                yVisao[i][2] = yVisao[i][1];
                yVisao[i][1] = yVisao[i][0];
                yVisao[i][0] = y;

                angVisao[i][2] = angVisao[i][1];
                angVisao[i][1] = angVisao[i][0];
                angVisao[i][0] = ang;

                estado[i].angulo = angVisao[i][2];

                estado[i].dx = xVisao[i][2] - estado[i].x;
                estado[i].dy = yVisao[i][2] - estado[i].y;

                estado[i].x = xVisao[i][2];
                estado[i].y = yVisao[i][2];
            }
        }

        for (i = 0; i < 7; i++) {
            estado1[i].x = 170 - (estadoReal[6 - i].x); //+random(3)-1);
            estado1[i].y = 130 - (estadoReal[6 - i].y); //+random(3)-1);
            estado1[i].dx = estado1[i].x - estado1Ant[i].x;
            estado1[i].dy = estado1[i].y - estado1Ant[i].y;
            estado1[i].angulo = atang2(estado1[i].dy, estado1[i].dx);
        }
        printf("%5d\n", quadro++);
    }

    void PosicionaRobos1(void) {
        int i, j;
        for (i = 0; i < 3; i++) {
            for (j = i + 1; j < 3; j++) { //repete para os tres primeiros
                if (estado1[j].x < estado1[i].x) {
                    Estado Tmp = estado1[j];
                    estado1[j] = estado1[i];
                    estado1[i] = Tmp;
                }
            }
        }
        for (i = 0; i < 3; i++) {
            estado1Ant[i] = estado1Prev[i] = estado1[i];
        }
        Goleiro1 = 0;
        Volante1 = 1;
        Atacante1 = 2;
    }

static float vdAnt[3][TAM_HISTORIA], veAnt[3][TAM_HISTORIA];
static float fdang[3] = { 0, 0, 0 };
static float fdx[3] = { 0, 0, 0 }, fdy[3] = { 0, 0, 0 };


void CorrigeLimites(int i) {
	if (i != 3) {
		if (41 < estadoReal[i].y && estadoReal[i].y < 89) {
			if (estadoReal[i].x < 4) {
				estadoReal[i].x = 4;
			} else if (estadoReal[i].x > 166) {
				estadoReal[i].x = 166;
			}
		} else {
			if (estadoReal[i].x < 14) {
				estadoReal[i].x = 14;
			} else if (estadoReal[i].x > 156) {
				estadoReal[i].x = 156;
			}
			if (estadoReal[i].y < 4) {
				estadoReal[i].y = 4;
			} else if (estadoReal[i].y > 126) {
				estadoReal[i].y = 126;
			}
		}
	} else { // é a bola
		int dx = estadoReal[i].dx;
		int dy = estadoReal[i].dy;
		if (42 < estadoReal[i].y && estadoReal[i].y < 87) { // bola na direcao (y) dos limites do gol?
			if (estadoReal[i].x < 3) { //bola no fundo do gol?
				estadoReal[i].x = 3;
				estadoReal[i].dx = -dx - 1;
			} else if (estadoReal[i].x > 167) { //bola no fundo do gol?
				estadoReal[i].x = 167;
				estadoReal[i].dx = -dx + 1;
			}
		} else { //bola fora da direcao do gol
			if (estadoReal[i].x - estadoReal[i].y > 151 || // canto inferior direito
					estadoReal[i].x - estadoReal[i].y < -110 || // canto superio esquerdo
					estadoReal[i].x + estadoReal[i].y > 280 || // canto superio direito
					estadoReal[i].x + estadoReal[i].y < 20) { // canto inferior esquerdo
				while (estadoReal[i].x - estadoReal[i].y > 151) {
					estadoReal[i].x--;
					estadoReal[i].y++;
				}
				while (estadoReal[i].x - estadoReal[i].y < -110) {
					estadoReal[i].x++;
					estadoReal[i].y--;
				}
				while (estadoReal[i].x + estadoReal[i].y > 280) {
					estadoReal[i].x--;
					estadoReal[i].y--;
				}
				while (estadoReal[i].x + estadoReal[i].y < 20) {
					estadoReal[i].x++;
					estadoReal[i].y++;
				}
				if (abs(dx) > abs(dy)) {
					dx > 0 ? dx-- : dx++;
				} else if (abs(dx) < abs(dy)) {
					dy > 0 ? dy-- : dy++;
				} else if (abs(dx) > 0) {
					dx > 0 ? dx-- : dx++;
					dy > 0 ? dy-- : dy++;
				}
				if (estadoReal[i].x + estadoReal[i].y > 280 || // canto superio direito
						estadoReal[i].x + estadoReal[i].y < 20) {
					estadoReal[i].dx = -dy;
					estadoReal[i].dy = -dx;
				} else {
					estadoReal[i].dx = dy;
					estadoReal[i].dy = dx;
				}
				return;
			}
			if (estadoReal[i].x < 13) { //linha de fundo?
				estadoReal[i].x = 13;
				estadoReal[i].dx = -dx - 1;
			} else if (estadoReal[i].x > 157) { //linha de fundo?
				estadoReal[i].x = 157;
				estadoReal[i].dx = -dx + 1;
			}
			if (estadoReal[i].y < 3) { //na lateral?
				estadoReal[i].y = 3;
				estadoReal[i].dy = -dy - 1;
			} else if (estadoReal[i].y > 127) { //na lateral?
				estadoReal[i].y = 127;
				estadoReal[i].dy = -dy + 1;
			}
		}
	}
}

void AtualizaJogador(int i) {
	estadoRealAnt[i] = estadoReal[i];
	//  estadoReal[i].Dx+=random(3)-1;
	//  estadoReal[i].Dy+=random(3)-1;
	if (estadoReal[i].dx > 7)
		estadoReal[i].dx = 7;
	if (estadoReal[i].dy > 7)
		estadoReal[i].dy = 7;
	estadoReal[i].x += estadoReal[i].dx;
	estadoReal[i].y += estadoReal[i].dy;

	CorrigeLimites(i);

	estadoReal[i].dx = estadoReal[i].x - estadoRealAnt[i].x;
	estadoReal[i].dy = estadoReal[i].y - estadoRealAnt[i].y;
	//  estadoReal[i].Angulo=iatan2(estadoReal[i].Dy, estadoReal[i].Dx);
}

void AtualizaBola(void) {
	int i = 3;
	estadoRealAnt[i] = estadoReal[i];
	if (estadoReal[i].dx > 7)
		estadoReal[i].dx = 7;
	if (estadoReal[i].dy > 7)
		estadoReal[i].dy = 7;
	estadoReal[i].x += estadoReal[i].dx;
	estadoReal[i].y += estadoReal[i].dy;

	CorrigeLimites(i);

	estadoReal[i].angulo = atang2(estadoReal[i].dy, estadoReal[i].dx);
}

int VerificaColisao(int r1, int r2) {
	int x1r1, x2r1, x1r2, x2r2;
	int y1r1, y2r1, y1r2, y2r2;
	int Colisaox = 0, Colisaoy = 0;
	int d;

	if (r1 == r2)
		return 0;

	if (r1 == 3 || r2 == 3)
		d = 6;
	else
		d = 8;

	if (estadoReal[r1].x > estadoRealAnt[r1].x) { //teste pelo cruzamento
		x1r1 = estadoRealAnt[r1].x;
		x2r1 = estadoReal[r1].x;
	} else {
		x1r1 = estadoReal[r1].x;
		x2r1 = estadoRealAnt[r1].x;
	}
	if (estadoReal[r2].x > estadoRealAnt[r2].x) {
		x1r2 = estadoRealAnt[r2].x;
		x2r2 = estadoReal[r2].x;
	} else {
		x1r2 = estadoReal[r2].x;
		x2r2 = estadoRealAnt[r2].x;
	}
	if (x1r1 < x2r2) {
		if (x2r1 > x2r2 || x2r1 > x1r2) {
			Colisaox = 1;
		}
	}
	if (estadoReal[r1].y > estadoRealAnt[r1].y) { //teste pelo cruzamento
		y1r1 = estadoRealAnt[r1].y;
		y2r1 = estadoReal[r1].y;
	} else {
		y1r1 = estadoReal[r1].y;
		y2r1 = estadoRealAnt[r1].y;
	}
	if (estadoReal[r2].y > estadoRealAnt[r2].y) {
		y1r2 = estadoRealAnt[r2].y;
		y2r2 = estadoReal[r2].y;
	} else {
		y1r2 = estadoReal[r2].y;
		y2r2 = estadoRealAnt[r2].y;
	}
	if (y1r1 < y2r2) {
		if (y2r1 > y2r2 || y2r1 > y1r2) {
			Colisaoy = 1;
		}
	}

	if (abs(x2r1 - x2r2) < d && abs(y2r1 - y2r2) < d) { //teste pela distancia
		Colisaox = 1;
		Colisaoy = 1;
	}
	return Colisaox && Colisaoy;
}

void CorrigePosicao(int r1, int r2) { //corrige colisao
	int xc, yc;
	int d1, d2, dx, dy, h;

	if (r1 != 3 && r2 != 3) {
		xc = (estadoRealAnt[r1].x + estadoRealAnt[r2].x) / 2;
		yc = (estadoRealAnt[r1].y + estadoRealAnt[r2].y) / 2;
		d1 = (abs(estadoRealAnt[r1].dx) + abs(estadoRealAnt[r1].dy)) / 2;
		if (d1 != 0) {
			estadoReal[r1].x = xc - estadoRealAnt[r1].dx * 2 / d1;
			estadoReal[r1].y = yc - estadoRealAnt[r1].dy * 2 / d1;
		}
		d2 = (abs(estadoRealAnt[r2].dx) + abs(estadoRealAnt[r2].dy)) / 2;
		if (d2 != 0) {
			estadoReal[r2].x = xc - estadoRealAnt[r2].dx * 2 / d2;
			estadoReal[r2].y = yc - estadoRealAnt[r2].dy * 2 / d2;
		}
		estadoReal[r1].dx = 0;
		estadoReal[r1].dy = 0;
		estadoReal[r2].dx = 0;
		estadoReal[r2].dy = 0;

		dx = estadoReal[r1].x - estadoReal[r2].x;
		dy = estadoReal[r1].y - estadoReal[r2].y;
		h = sqrt(dx * dx + dy * dy);
		if (h <= 10) {
			if (h == 0)
				h = 1;
			dx = 11 * dx / h;
			dy = 11 * dy / h;
			estadoReal[r1].x = xc + dx / 2;
			estadoReal[r1].y = yc + dy / 2;
			estadoReal[r2].x = xc - dx / 2;
			estadoReal[r2].y = yc - dy / 2;
		}
	} else {
		if (r2 == 3) {
			int tmp = r1;
			r1 = r2;
			r2 = tmp;
		}
		xc = (estadoRealAnt[r1].x + estadoRealAnt[r2].x) / 2;
		yc = (estadoRealAnt[r1].y + estadoRealAnt[r2].y) / 2;
		d1 = (abs(estadoRealAnt[r1].dx) + abs(estadoRealAnt[r1].dy)) / 2;
		if (d1 != 0) {
			estadoReal[r1].x = xc - estadoRealAnt[r1].dx / d1;
			estadoReal[r1].y = yc - estadoRealAnt[r1].dy / d1;
		}
		d2 = (abs(estadoRealAnt[r2].dx) + abs(estadoRealAnt[r2].dy)) / 2;
		if (d2 != 0) {
			estadoReal[r2].x = xc - estadoRealAnt[r2].dx * 2 / d2;
			estadoReal[r2].y = yc - estadoRealAnt[r2].dy * 2 / d2;
		}
		estadoReal[r1].dx = estadoRealAnt[r2].dx - estadoRealAnt[r1].dx / 2;
		estadoReal[r1].dy = estadoRealAnt[r2].dy - estadoRealAnt[r1].dy / 2;
		estadoReal[r2].dx /= 2;
		estadoReal[r2].dy /= 2;

		dx = estadoReal[r1].x - estadoReal[r2].x;
		dy = estadoReal[r1].y - estadoReal[r2].y;
		h = sqrt(dx * dx + dy * dy);
		if (h <= 8) {
			if (h == 0)
				h = 1;
			dx = 9 * dx / h;
			dy = 9 * dy / h;
			estadoReal[r1].x = xc + dx / 2;
			estadoReal[r1].y = yc + dy / 2;
			estadoReal[r2].x = xc - dx / 2;
			estadoReal[r2].y = yc - dy / 2;
		}
	}

	CorrigeLimites(r1);
	CorrigeLimites(r2);
}


void Simulador(void) {
	int i, j;

	int indGoleiroTemp = indGoleiro;
	int indVolanteTemp = indVolante;
	int indAtacanteTemp = indAtacante;
	bool tiroMetaTemp = tiroMeta;
	bool emPenalidadeTemp = emPenalidade;
	bool emPosicionaTemp = emPosiciona;
	bool emInicioTemp = emInicio;
	
	Estado estadoTemp[NUM_ROBOS_TIME * 2 + 1], estadoAntTemp[NUM_ROBOS_TIME * 2 + 1], estadoPrevTemp[NUM_ROBOS_TIME * 2 + 1];
	Objetivo objetivoRoboTemp[NUM_ROBOS_TIME];

	for (i = 0; i < 7; i++)
	{
		estadoTemp[i] = estado[i];
		estadoAntTemp[i] = estadoAnt[i];
		estadoPrevTemp[i] = estadoPrev[i];

		if (i < 3)
		{
			objetivoRoboTemp[i] = objetivoRobo[i];		
		}
		
	}


	emSimulacao = true;

	// Inicia Posicao jogadores
		Visao();

		for (i = 0; i < 7; i++) {
			if (i != 3) {
				AtualizaJogador(i);
			} else {
				AtualizaBola();
			}
		}

		estrategia();

		for (j = 0; j < 30; j++) {
			for (i = 0; i < 7; i++) {
				estado[i].x = estadoReal[i].x;
				estado[i].y = estadoReal[i].y;
				estado[i].dx = estadoReal[i].dx;
				estado[i].dy = estadoReal[i].dy;
				estado[i].angulo = estadoReal[i].angulo;
			}
			memcpy(estadoAnt, estado, sizeof(Estado[7]));
			memcpy(estadoPrev, estado, sizeof(Estado[7]));
			memcpy(estadoRealAnt, estadoReal, sizeof(estadoReal[7]));

			memset(vdAnt, 0, sizeof(float[3][3]));
			memset(veAnt, 0, sizeof(float[3][3]));
			memset(fdang, 0, sizeof(float[3]));
			memset(fdx, 0, sizeof(float[3]));
			memset(fdy, 0, sizeof(float[3]));

			PosicionaRobos();
			PosicionaRobos1();

			for (i = 0; i < 7; i++) {
				int j;
				for (j = 0; j < 3; j++) {
					xVisao[i][j] = estado[i].x;
					yVisao[i][j] = estado[i].y;
					angVisao[i][j] = estado[i].angulo;
				}
			}
			//         srand(1);
			quadro = 0;
	}

	for (i = 0; i < 7; i++)
	{
		if (i < 3)
		{
			if (i == indAtacante && (abs(estado[indAtacante].x - estado[3].y) < 4) && (abs(estado[indAtacante].y - estado[3].y) < 4)) {
				objetivoRobo[i].x = estado[i].x;
				objetivoRobo[i].y = estado[i].y;
				objetivoRobo[i].angulo = estado[i].angulo;
			}
			else 
				objetivoRobo[i] = objetivoRoboTemp[i];
		}

		estado[i] = estadoTemp[i];
		estadoAnt[i] = estadoAntTemp[i];
		estadoPrev[i] = estadoPrevTemp[i];
	}

	indGoleiro = indGoleiroTemp;
	indVolante = indVolanteTemp;
	indAtacante = indAtacanteTemp;
	tiroMeta = tiroMetaTemp;
	emPenalidade = emPenalidadeTemp;
	emPosiciona = emPosicionaTemp;
	emInicio = emInicioTemp;

	emSimulacao = false;
}