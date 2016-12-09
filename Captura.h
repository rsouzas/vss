/*
 * ESTE software foi fornecido como exemplo de controlador de futebol de robôs na Segunda Oficina Brasileira de Futebol de Robôs realizada junto ao 5o Workshop em Automação e Robótica Aplicada (Robocontrol) 2010.

 * Você que está de posse dESTE software, está livre para utilizá-lo, alterá-lo, copiá-lo e incluí-lo parcial ou integralmente em outros software desde que acompanhado da seguinte indicação:
 * "Este software tem seções de código desenvolvidas por Rene Pegoraro no Laboratório de Integração de Sistemas e Dispositivos Inteligentes (LISDI) do Departamento de Computação da Faculdade de Ciências da Universidade Estadual Paulista (UNESP) - Campos de Bauru - SP - Brasil"

 * Se qualquer publicação for gerada pela utilização de software utilizando parcial ou integralmente ESTE software, esta publicação deve conter os devidos créditos para o "Grupo de Integração de Sistemas e Dispositivos Inteligentes (GISDI) do Departamento de Computação da Faculdade de Ciências da Universidade Estadual Paulista (UNESP) - Campos de Bauru - SP - Brasil"
 */

#ifndef CAPTURA_H_
#define CAPTURA_H_

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <assert.h>
#include <getopt.h>             /* getopt_long() */
#include <fcntl.h>              /* low-level i/o */
#include <unistd.h>
#include <errno.h>
#include <malloc.h>
#include <sys/stat.h>
#include <sys/types.h>
#include <sys/time.h>
#include <sys/mman.h>
#include <sys/ioctl.h>
#include <sys/time.h>
#include <asm/types.h>          /* for videodev2.h */
#include <linux/videodev2.h>
#include <string.h>
//#include "cxtypes.h"

#define CLEAR(x) memset (&(x), 0, sizeof (x))

class Captura {
	struct buffer {
		void * start;
		size_t length;
	};

	char devName[20];
	int fd;
	struct buffer * buffers;
	unsigned int n_buffers;
	int videoInput;

	void errnoExit(const char * s);
	int xioctl(int fd, int request, void * arg);
	void stopCapturing(void);
	void startCapturing(void);
	void uninitDevice(void);
	void initMmap(void);
	void initDevice(void);
	void closeDevice(void);
	void openDevice(void);
	bool queryCurrentFrame();

public:
	int numErro; // se um erro ocorrer o numero sera atribuido aqui; necessario para recuperar do erro "VIDIOC_DQBUF error 5, Input/output error"

	Captura(char * devName = "/dev/video0", int videoInput = 0);
	void queryFrames(unsigned int countFrames = 1);
	int getFD();
	virtual void video(const void * p) {
		printf(".");
//		errnoExit("Esta funcao deve ser substituida na classe descendente!\n");
	}

	~Captura();

	void enumCaptureFormats();
};

void yuyv_to_rgb24(int width, int height, unsigned char *src,
		unsigned char *dst);

#endif /* CAPTURA_H_ */

