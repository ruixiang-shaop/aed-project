#include "animaciones.h"

void avanzar_etapa()
{
  switch (efectoActual) {
    case 0:
      etapa = (etapa + 1) % 5;
      break;
    case 1:
      etapa = (etapa + 1) % 5;
      break;
    case 2:
      etapa = (etapa + 1) % 3;
      break;
    case 3:
      etapa = (etapa + 1) % 3;
      break;

    default:
      break;
  }
}

void animaciones()
{
  switch (efectoActual) {
    case 0:
      animacion0();
      break;
    case 1:
      animacion1();
      break;
    case 2:
      animacion2();
      break;
    case 3:
      animacion3();
      break;

    default:
      break;
  }
}

void animacion0()
{
  switch (etapa) {
      case 0:
	WRITE_EXTERNAL_LEDS(1, 0, 0, 0, 0);
        break;
      case 1:
	WRITE_EXTERNAL_LEDS(0, 1, 0, 0, 0);
	break;
      case 2:
	WRITE_EXTERNAL_LEDS(0, 0, 1, 0, 0);
	break;
      case 3:
	WRITE_EXTERNAL_LEDS(0, 0, 0, 1, 0);
	break;
      case 4:
	WRITE_EXTERNAL_LEDS(0, 0, 0, 0, 1);
	break;
      default:
        break;
  }
}

void animacion1()
{
  switch (etapa) {
      case 0:
	WRITE_EXTERNAL_LEDS(0, 0, 0, 0, 1);
        break;
      case 1:
	WRITE_EXTERNAL_LEDS(0, 0, 0, 1, 0);
	break;
      case 2:
	WRITE_EXTERNAL_LEDS(0, 0, 1, 0, 0);
	break;
      case 3:
	WRITE_EXTERNAL_LEDS(0, 1, 0, 0, 0);
	break;
      case 4:
	WRITE_EXTERNAL_LEDS(1, 0, 0, 0, 0);
	break;
      default:
        break;
  }
}

void animacion2()
{
  switch (etapa) {
      case 0:
	WRITE_EXTERNAL_LEDS(0, 0, 1, 0, 0);
        break;
      case 1:
	WRITE_EXTERNAL_LEDS(0, 1, 0, 1, 0);
	break;
      case 2:
	WRITE_EXTERNAL_LEDS(1, 0, 0, 0, 1);
	break;
      default:
        break;
  }
}

void animacion3()
{
  switch (etapa) {
      case 0:
	WRITE_EXTERNAL_LEDS(1, 0, 0, 0, 1);
	break;
      case 1:
	WRITE_EXTERNAL_LEDS(0, 1, 0, 1, 0);
	break;
      case 2:
	WRITE_EXTERNAL_LEDS(0, 0, 1, 0, 0);
        break;
      default:
        break;
  }
}
