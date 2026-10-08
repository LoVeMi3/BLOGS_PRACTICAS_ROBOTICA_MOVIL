#include "HAL.hpp"
#include "WebGUI.hpp"
#include "Frequency.hpp"

const int ideal_rate = 50;

/*
Si está en curva que la veloidad disminuya
Si está en recta que aumente la velocidad
Tener unos parámetros de velocidad para curva y otros para recta
*/
const int V_BASE = 7;
const int V_TURN = 5;
const int V_SEARCH = 6;
const int V_BACK = 5;

//estado de carrera
volatile bool lapStarted = false;

//estado de línea
bool lineaPerdida = bool;
bool buscando = false;
int ultimaDireccion = 1; //1 = derecha, -1 = izquierda

//esatdística línea visible (ticks con línea / ticks totales)
unsigned long ticksTotal = 0;
unsigned long ticksConLinea = 0;

enum State {
    STRAIGHT,
    TURN,
    LOST,
    FOUND
};

void exercise() {
    Frequency freq = Frequency();

    State STATE = STRAIGHT;

    while (true)
    {
        freq.tick(ideal_rate);
        
        //HAY QUE UTILIZAR HSV, NO RGB
        /*HAL::get_image(); - to get the image (cv::Mat).
        HAL::set_v(velocity); - to set the linear speed.
        HAL::set_w(velocity); - to set the angular velocity*/

        const Image* imageRGB = HAL::get_image();
        if (imageRGB == nullptr) {
            HAL::set_v(0.0f);
            HAL::set_w(0.0f);
            continue;
        }

    }
}
    
