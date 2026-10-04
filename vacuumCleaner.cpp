#include <algorithm>
#include <cmath>
#include <cstdlib>
#include "HAL.hpp"
#include "WebGUI.hpp"
#include "Frequency.hpp"

enum State { 
  SPIRAL, 
  FORWARD, 
  BACKUP, 
  TURN 
};

void exercise() {
  Frequency freq = Frequency();
  const int ideal_rate = 50;
  // Enter sequential code!

  State state = SPIRAL;
  int ticks = 0;
  int turn_ticks = 0;
  float turn_dir = 1.0f;  // +1 izquierda, -1 derecha

  while (true) {
    freq.tick(ideal_rate);
    // Enter iterative code!
    const LaserData* laser = HAL::get_laser_data();
    if (laser == nullptr || laser->values.size() < 180) {
      //comprobando si los datos son válidos, si no, al siguiente ciclo
      HAL::set_v(0.0f);
      HAL::set_w(0.0f);
      continue;
    }
    auto rd = [&](int i) {
      float r = laser->values[i];
      return std::isfinite(r) ? std::min(r, 5.0f) : 5.0f;
    };
    /*
    //distancia del centro del laser es 90, cono de visión entre 75 y 105
    float center = 90.0f, left = 0.0f, right = 0.0f;
    for (int i = 75; i <= 105; i++) center = std::min(center, rd(i));
    for (int i = 135; i < 180; i++) left  += rd(i);
    for (int i = 0;   i < 45;  i++) right += rd(i);

    bool obst = center < 0.35f; //distancia obstáculo 0.35
    */
    float center = 5.0f, sideL = 5.0f, sideR = 5.0f;
    float left = 0.0f, right = 0.0f;
    for (int i = 60; i <= 120; i++) center = std::min(center, rd(i));   // ±30º
    for (int i = 140; i < 180; i++) sideL = std::min(sideL, rd(i));     // lado izq
    for (int i = 0;   i < 40;  i++) sideR = std::min(sideR, rd(i));     // lado dcho
    for (int i = 135; i < 180; i++) left  += rd(i);
    for (int i = 0;   i < 45;  i++) right += rd(i);

    bool obst = center < 0.40f || sideL < 0.22f || sideR < 0.22f;

    switch (state) {
      case SPIRAL: {
        float v = 0.3f; //velocidad lineal
        float r = 0.4f + 0.004f * ticks; // radio creciente, con r aumentando con el tiempo
        HAL::set_v(v);
        HAL::set_w(v / r); //velocidad angular
        ticks++;
        if (obst) {
          ticks = 0;
          state = BACKUP;
        } else if (ticks > 400) { 
          ticks = 0; 
          state = FORWARD; 
        } //si el radio de la espiral es muy grande, va recto
        break;
      }
      case FORWARD: {
        HAL::set_v(0.4f);
        HAL::set_w(0.0f);
        if (obst) {
          ticks = 0;
          state = BACKUP;
        }
        break;
      }
      case BACKUP: {
        HAL::set_v(-0.15f);
        HAL::set_w(0.0f);
        if (++ticks > 10) {
          turn_dir = (left > right) ? 1.0f : -1.0f;
          // giro aleatorio entre ~90º y ~180º con w = 1 rad/s
          float angle = 1.0f + (rand() / (float)RAND_MAX) * 1.5f;
          turn_ticks = (int)(angle * ideal_rate);
          ticks = 0;
          state = TURN;
        }
        break;
      }
      case TURN: {
        HAL::set_v(0.0f);
        HAL::set_w(turn_dir * 1.0f);
        if (++ticks > turn_ticks) {
          ticks = 0;
          state = (center > 1.5f && sideL > 1.0f && sideR > 1.0f) ? SPIRAL : FORWARD; //spiral sólo si hay mucho espacio, si no, recto
        }
        break;
      }
    }
  }
}
