/*
 * ESTE software foi fornecido como exemplo de controlador de futebol de robôs na Segunda Oficina Brasileira de Futebol de Robôs realizada junto ao 5o Workshop em Automação e Robótica Aplicada (Robocontrol) 2010.

 * Você que está de posse dESTE software, está livre para utilizá-lo, alterá-lo, copiá-lo e incluí-lo parcial ou integralmente em outros software desde que acompanhado da seguinte indicação:
 * "Este software tem seções de código desenvolvidas por Rene Pegoraro no Laboratório de Integração de Sistemas e Dispositivos Inteligentes (LISDI) do Departamento de Computação da Faculdade de Ciências da Universidade Estadual Paulista (UNESP) - Campos de Bauru - SP - Brasil"

 * Se qualquer publicação for gerada pela utilização de software utilizando parcial ou integralmente ESTE software, esta publicação deve conter os devidos créditos para o "Grupo de Integração de Sistemas e Dispositivos Inteligentes (GISDI) do Departamento de Computação da Faculdade de Ciências da Universidade Estadual Paulista (UNESP) - Campos de Bauru - SP - Brasil"
 */

#include "Serial.h"

//const char *device = "/dev/ttyS0";
const char *device = "/dev/ttyUSB0";

struct termios cfg;

int fd = 0;

void erro(char *s) {
	perror(s);
	exit(0);
}

void iniciaComunicacao(void) {
#ifdef SERIAL
	fd = open(device, O_RDWR | O_NOCTTY | O_NDELAY);
	if (fd == -1) {
		erro("erro na abertura da interface serial\n");
	}

	if (!isatty(fd)) {
		erro(
				"A interface serial aberta nao é realmente uma interface serial!\n");
	}

	if (tcgetattr(fd, &cfg) < 0) {
		erro("A configuracao da interface serial nao pode ser lida!\n");
	}

	cfg.c_iflag = IGNBRK | IGNPAR;
	cfg.c_oflag = 0;
	cfg.c_lflag = 0;
	cfg.c_cflag = B115200 | CS8; // | CRTSCTS;// | CSTOPB;
	cfg.c_ispeed = 115200;
	cfg.c_ospeed = 115200;

	cfg.c_cc[VMIN] = 1;
	cfg.c_cc[VTIME] = 0;

//	int sercmd = 0;// desliga os bits TIOCM_RTS | TIOCM_DTR para colocar 12V na saida
//	ioctl(fd, TIOCMBIC, &sercmd); // Set the RTS pin.

	if (tcsetattr(fd, TCSAFLUSH, &cfg) < 0) {
		erro("A configuracao da interface serial nao pode ser alterada!\n");
	}

	if (cfsetispeed(&cfg, B115200) < 0 || cfsetospeed(&cfg, B115200) < 0) {
		erro("A interface serial nao pode ser configurada!\n");
	}
#endif
}

int recebeByte() {
	unsigned char c;
	if (read(fd, &c, 1) >= 0) {
		//printf("(%02x) ", c);
		return c;
	} else {
		return -1;
	}
}

void enviaByteEspera(int n) {
// não necesário nesta vesão de rádio
}

void enviaDados(unsigned char b1, unsigned char b2, unsigned char b3,
		unsigned char b4, unsigned char b5, unsigned char b6) {
#ifdef SERIAL
	tcflush(fd, TCIOFLUSH);
	//unsigned char b[NUM_ROBOS_TIME*2 + 1];
	unsigned char b[8];

	b[0] = 0x80;
	write(fd, b, 1);

	// CODIGO PARA 3 ROBÔS
	b[0] = b1;
	b[1] = b2;
	b[2] = b3;
	b[3] = b4;
	b[4] = b5;
	b[5] = b6;
	b[6] = '0';
	b[7] = '0';
	int resp = write(fd, b, 8);

//	b[0] = 0x88;	// ocupa o radio para nao morrer
//	b[1] = 0x88;	// ocupa o radio para nao morrer
//	b[2] = 0x88;	// ocupa o radio para nao morrer
//	write(fd, b, 1);

	if (resp < 0) {
		erro("Interface serial - nao pode enviar comandos aos robos.\n");
	}

#endif
}

void terminaComunicacao(void) {
#ifdef SERIAL
	int sercmd = TIOCM_RTS | TIOCM_DTR;
	ioctl(fd, TIOCMBIC, &sercmd);
	close(fd);
#endif
}
