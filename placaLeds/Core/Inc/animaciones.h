#ifndef INC_ANIMACIONES_H_
#define INC_ANIMACIONES_H_

#include "main.h"

extern volatile uint8_t etapa;
extern volatile uint8_t efectoActual;

void animaciones();
void animacion0();
void animacion1();
void animacion2();
void animacion3();
void avanzar_etapa();

#endif /* INC_ANIMACIONES_H_ */
